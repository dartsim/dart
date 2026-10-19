// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "detail/dart_numpy_capi.h"
#include "detail/eigen.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <functional>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <cmath>
#include <cstring>

namespace dart {
namespace python {
namespace {

// Default validation tolerance for |R^T R - I|_F, |det R - 1|, the
// homogeneous row and |norm(q) - 1|. Calibrated in the DART 7 transform PR:
// composition of 1e4 rotations drifts below 1e-13, quaternion round trips
// stay below 1e-14, text with six significant digits reaches 2.3e-6, and
// per-entry noise of 1e-6 already produces 1e-5. One order of magnitude above
// the worst legitimate source and one below the smallest input that tests
// must reject (1e-4 noise, 1e-3 error).
constexpr double kTolerance = 1e-5;
// normalize=True projects a matrix only when its singular values all lie
// within kNormalizeStretch of 1 (operator-norm distance to the nearest
// rotation) and their relative spread (max - min) / max stays within
// kNormalizeSpread. Stretch alone admits a shear of 0.005, whose singular
// values are 1 +- 0.0025; spread catches shear and non-uniform scale while a
// uniform 1.001 scale or float32 rounding (spread ~1e-7) still passes.
constexpr double kNormalizeStretch = 1e-2;
constexpr double kNormalizeSpread = 1e-3;

using Isometry = Eigen::Isometry3d;
using Storage = std::vector<Isometry, Eigen::aligned_allocator<Isometry>>;
// Hashing, equality and pickling treat an item as 16 contiguous doubles.
static_assert(
    sizeof(Isometry) == 16 * sizeof(double),
    "Isometry3d must be a padding-free 4x4 double matrix");

struct Poses
{
  Storage items;
  bool batched = false;
  std::size_t count() const
  {
    return items.size();
  }
};
struct TransformValue : Poses
{
};
struct RotationValue : Poses
{
};

// Read-only float64 view of an array-like input. NumPy arrays of the right
// dtype and layout are borrowed through the buffer protocol; flat tuples and
// lists of numbers are copied directly; everything else goes through
// numpy.ascontiguousarray.
class InArray
{
public:
  InArray() = default;
  InArray(InArray&& other) noexcept
    : view_(other.view_),
      owned_(std::move(other.owned_)),
      shape_(std::move(other.shape_)),
      data_(other.data_)
  {
    other.view_.obj = nullptr;
  }
  InArray(const InArray&) = delete;
  InArray& operator=(const InArray&) = delete;
  InArray& operator=(InArray&&) = delete;
  ~InArray()
  {
    if (view_.obj)
      PyBuffer_Release(&view_);
  }

  bool acquire(nb::handle obj)
  {
    if (PyObject_GetBuffer(
            obj.ptr(), &view_, PyBUF_ND | PyBUF_FORMAT | PyBUF_C_CONTIGUOUS)
        != 0) {
      PyErr_Clear();
      view_.obj = nullptr;
      return false;
    }
    const char* format = view_.format ? view_.format : "B";
    if (format[0] == '@' || format[0] == '=' || format[0] == '<')
      ++format;
    if (view_.itemsize != sizeof(double) || std::strcmp(format, "d") != 0) {
      PyBuffer_Release(&view_);
      view_.obj = nullptr;
      return false;
    }
    shape_.assign(view_.shape, view_.shape + view_.ndim);
    data_ = static_cast<const double*>(view_.buf);
    return true;
  }
  void adopt(std::vector<double> values)
  {
    owned_ = std::move(values);
    shape_ = {owned_.size()};
    data_ = owned_.data();
  }

  const double* data() const
  {
    return data_;
  }
  std::size_t ndim() const
  {
    return shape_.size();
  }
  std::size_t shape(std::size_t i) const
  {
    return shape_[i];
  }
  const std::vector<std::size_t>& shapeVector() const
  {
    return shape_;
  }
  std::size_t size() const
  {
    std::size_t n = 1;
    for (std::size_t s : shape_)
      n *= s;
    return n;
  }

private:
  Py_buffer view_{};
  std::vector<double> owned_;
  std::vector<std::size_t> shape_;
  const double* data_ = nullptr;
};

InArray asArray(nb::handle obj)
{
  InArray out;
  if (out.acquire(obj))
    return out;
  if (nb::isinstance<nb::tuple>(obj) || nb::isinstance<nb::list>(obj)) {
    const Py_ssize_t n = PySequence_Size(obj.ptr());
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(std::max<Py_ssize_t>(n, 0)));
    bool flat = n >= 0;
    for (Py_ssize_t i = 0; flat && i < n; ++i) {
      nb::object item = nb::steal(PySequence_GetItem(obj.ptr(), i));
      const double value = item.is_valid() ? PyFloat_AsDouble(item.ptr()) : -1;
      if (value == -1.0 && PyErr_Occurred()) {
        PyErr_Clear();
        flat = false;
      } else {
        values.push_back(value);
      }
    }
    if (flat) {
      out.adopt(std::move(values));
      return out;
    }
  }
  nb::object np = nb::module_::import_("numpy");
  nb::object array = np.attr("ascontiguousarray")(
      obj, nb::arg("dtype") = np.attr("float64"));
  if (!out.acquire(array))
    throw nb::type_error("expected an array-like of numbers");
  return out;
}

void requireFinite(const InArray& array, const char* name)
{
  const double* data = array.data();
  for (std::size_t i = 0; i < array.size(); ++i) {
    if (!std::isfinite(data[i]))
      throw nb::value_error((std::string(name) + " must be finite").c_str());
  }
}

// Fresh read-only float64 array (writeable=False) filled by fn.
template <class F>
nb::object makeArray(const std::size_t* shape, std::size_t ndim, F&& fill)
{
  std::size_t n = 1;
  for (std::size_t d = 0; d < ndim; ++d)
    n *= shape[d];
  double* buffer = new double[n];
  fill(buffer);
  nb::capsule owner(
      buffer, [](void* p) noexcept { delete[] static_cast<double*>(p); });
  const int fastExport = dart_numpy_capi::numpy2_export();
  if (fastExport < 0)
    throw nb::python_error();
  // The C-API export handles up to 3-D; higher ranks take nanobind's path.
  if (fastExport && ndim <= 3) {
    int64_t strides[3] = {1, 1, 1};
    for (std::size_t d = ndim; d-- > 1;)
      strides[d - 1] = strides[d] * static_cast<int64_t>(shape[d]);
    PyObject* result = dart_numpy_capi::from_data(
        static_cast<int>(ndim), shape, strides, buffer, owner.ptr(), false);
    if (!result)
      throw nb::python_error();
    return nb::steal(result);
  }
  return nb::cast(
      nb::ndarray<nb::numpy, const double>(buffer, ndim, shape, owner));
}

template <class F>
nb::object makeArray(std::initializer_list<std::size_t> shape, F&& fill)
{
  return makeArray(shape.begin(), shape.size(), std::forward<F>(fill));
}

std::string number(double value)
{
  std::ostringstream out;
  out.precision(2);
  out << std::scientific << value;
  return out.str();
}

// Singular values sigma (descending) of a candidate rotation.
struct Deviation
{
  double stretch;
  double spread;
  explicit Deviation(const Eigen::Vector3d& sigma)
    : stretch((sigma.array() - 1).abs().maxCoeff()),
      spread((sigma[0] - sigma[2]) / sigma[0])
  {
  }
  bool nearRotation() const
  {
    return stretch <= kNormalizeStretch && spread <= kNormalizeSpread;
  }
};

Eigen::Matrix3d validateRotation(
    const Eigen::Matrix3d& R, bool normalize, const char* name)
{
  const double orthogonality
      = (R.transpose() * R - Eigen::Matrix3d::Identity()).norm();
  const double det = R.determinant();
  if (!normalize) {
    if (orthogonality <= kTolerance && std::abs(det - 1) <= kTolerance)
      return R;
    std::string message = std::string(name) + " is not a rotation matrix: "
                          "|R^T R - I|_F = " + number(orthogonality)
                          + ", |det R - 1| = " + number(std::abs(det - 1))
                          + " (tolerance " + number(kTolerance) + ")";
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R);
    if (det > 0 && Deviation(svd.singularValues()).nearRotation())
      message += "; pass normalize=True to project it onto the nearest "
                 "rotation";
    throw nb::value_error(message.c_str());
  }
  if (det <= 0)
    throw nb::value_error((std::string(name)
                           + " is a reflection or singular (det = "
                           + number(det) + "); normalize=True cannot repair it")
                              .c_str());
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Deviation deviation(svd.singularValues());
  if (!deviation.nearRotation())
    throw nb::value_error((std::string(name)
                           + " has shear or non-uniform scale (singular "
                             "values deviate from 1 by "
                           + number(deviation.stretch) + ", limit "
                           + number(kNormalizeStretch) + "; relative spread "
                           + number(deviation.spread) + ", limit "
                           + number(kNormalizeSpread)
                           + "); normalize=True only projects "
                             "near-rotations")
                              .c_str());
  Eigen::Matrix3d projected = svd.matrixU() * svd.matrixV().transpose();
  if (projected.determinant() < 0)
    projected = svd.matrixU() * Eigen::Vector3d(1, 1, -1).asDiagonal()
                * svd.matrixV().transpose();
  return projected;
}

// q is wxyz. Each coefficient is first divided by the largest magnitude and
// then by the norm of that scaled vector, so huge or subnormal coefficients
// neither overflow nor underflow (the full norm itself may be inf or 0).
Eigen::Matrix3d fromQuaternion(
    const double* q, bool normalize, const char* name)
{
  double scale = 0;
  for (int i = 0; i < 4; ++i)
    scale = std::max(scale, std::abs(q[i]));
  if (scale == 0)
    throw nb::value_error(
        (std::string(name) + " must not be the zero quaternion").c_str());
  double scaled[4];
  double sum = 0;
  for (int i = 0; i < 4; ++i) {
    scaled[i] = q[i] / scale;
    sum += scaled[i] * scaled[i];
  }
  const double scaledNorm = std::sqrt(sum);
  const double norm = scale * scaledNorm;
  if (!normalize && !(std::abs(norm - 1) <= kTolerance))
    throw nb::value_error((std::string(name) + " has norm " + number(norm)
                           + ", not 1 within " + number(kTolerance)
                           + "; pass normalize=True to rescale it")
                              .c_str());
  for (double& value : scaled)
    value /= scaledNorm;
  return Eigen::Quaterniond(scaled[0], scaled[1], scaled[2], scaled[3])
      .toRotationMatrix();
}

// Extrinsic X-Y-Z: Rz(yaw) Ry(pitch) Rx(roll), as URDF and SDF define it.
Eigen::Matrix3d fromRpy(const double* rpy)
{
  return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

Eigen::Vector3d toRpy(const Eigen::Matrix3d& R)
{
  // pitch from the robust two-argument form; at gimbal lock (|pitch| = pi/2)
  // roll is set to 0 and yaw absorbs the remaining rotation.
  const double pitch
      = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
  if (std::abs(std::cos(pitch)) < 1e-12)
    return {0.0, pitch, std::atan2(-R(0, 1), R(1, 1))};
  return {std::atan2(R(2, 1), R(2, 2)), pitch, std::atan2(R(1, 0), R(0, 0))};
}

// Eigen converts through the largest-diagonal branch; the sign policy makes
// w >= 0 and, for w == 0, the first nonzero of (x, y, z) positive. The test
// is on the exact bits: a rounded w of -1e-17 negates the quaternion, and
// only an exact 0 (or -0) falls through to the next coefficient.
Eigen::Vector4d toWxyz(const Eigen::Matrix3d& R)
{
  const Eigen::Quaterniond q(R);
  Eigen::Vector4d out(q.w(), q.x(), q.y(), q.z());
  out /= out.cwiseAbs().maxCoeff();
  out /= out.norm();
  for (int i = 0; i < 4; ++i) {
    if (out[i] != 0) {
      if (out[i] < 0)
        out = -out;
      break;
    }
  }
  return out;
}

struct Parsed
{
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>
      rotations;
  std::vector<Eigen::Vector3d> translations;
  bool batched = false;
};

void requireShape(
    const InArray& array,
    std::size_t inner,
    std::size_t innerCols,
    const char* name,
    const char* expected)
{
  const std::size_t base = innerCols ? 2 : 1;
  const bool ok = (array.ndim() == base || array.ndim() == base + 1)
                  && array.shape(array.ndim() - base) == inner
                  && (!innerCols || array.shape(array.ndim() - 1) == innerCols);
  if (!ok)
    throw nb::value_error(
        (std::string(name) + " must have shape " + expected).c_str());
  if (array.size() == 0)
    throw nb::value_error(
        (std::string(name) + " must not be empty; a batch needs N >= 1")
            .c_str());
}

// itemRank is 1 for vectors and 2 for matrices; one more dimension is a batch.
template <class F>
void forEachItem(
    const InArray& array,
    std::size_t itemSize,
    std::size_t itemRank,
    bool& batched,
    F&& fn)
{
  batched = array.ndim() > itemRank;
  for (std::size_t i = 0; i < array.size() / itemSize; ++i)
    fn(array.data() + i * itemSize);
}

Parsed parse(
    nb::handle rotation,
    nb::handle wxyz,
    nb::handle xyzw,
    nb::handle rpy,
    nb::handle matrix,
    nb::handle translation,
    bool normalize,
    const char* what)
{
  Parsed parsed;
  int forms = 0;
  for (nb::handle form : {rotation, wxyz, xyzw, rpy, matrix})
    forms += !form.is_none();
  if (forms > 1)
    throw nb::value_error((std::string(what)
                           + " takes at most one of rotation=, wxyz=, xyzw=, "
                             "rpy= and matrix=")
                              .c_str());
  if (!matrix.is_none() && !translation.is_none())
    throw nb::value_error(
        (std::string(what) + " takes either matrix= or translation=").c_str());

  bool rotationBatched = false;
  if (!rotation.is_none() && nb::isinstance<RotationValue>(rotation)) {
    const auto& value = nb::cast<const RotationValue&>(rotation);
    rotationBatched = value.batched;
    // Without normalize the stored matrix is copied as is, so composition
    // results round-trip bit-exactly; normalize=True projects every rotation
    // input, whatever its type.
    for (const Isometry& item : value.items)
      parsed.rotations.push_back(
          normalize ? validateRotation(item.linear(), true, "rotation")
                    : item.linear());
  } else if (!rotation.is_none()) {
    InArray array = asArray(rotation);
    requireShape(array, 3, 3, "rotation", "(3, 3) or (N, 3, 3)");
    requireFinite(array, "rotation");
    forEachItem(array, 9, 2, rotationBatched, [&](const double* data) {
      parsed.rotations.push_back(validateRotation(
          Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(data),
          normalize,
          "rotation"));
    });
  } else if (!wxyz.is_none() || !xyzw.is_none()) {
    const bool scalarFirst = !wxyz.is_none();
    const char* name = scalarFirst ? "wxyz" : "xyzw";
    InArray array = asArray(scalarFirst ? wxyz : xyzw);
    requireShape(array, 4, 0, name, "(4,) or (N, 4)");
    requireFinite(array, name);
    forEachItem(array, 4, 1, rotationBatched, [&](const double* data) {
      const double ordered[4]
          = {scalarFirst ? data[0] : data[3],
             scalarFirst ? data[1] : data[0],
             scalarFirst ? data[2] : data[1],
             scalarFirst ? data[3] : data[2]};
      parsed.rotations.push_back(fromQuaternion(ordered, normalize, name));
    });
  } else if (!rpy.is_none()) {
    InArray array = asArray(rpy);
    requireShape(array, 3, 0, "rpy", "(3,) or (N, 3)");
    requireFinite(array, "rpy");
    forEachItem(array, 3, 1, rotationBatched, [&](const double* data) {
      parsed.rotations.push_back(fromRpy(data));
    });
  } else if (!matrix.is_none()) {
    InArray array = asArray(matrix);
    requireShape(array, 4, 4, "matrix", "(4, 4) or (N, 4, 4)");
    requireFinite(array, "matrix");
    forEachItem(array, 16, 2, rotationBatched, [&](const double* data) {
      Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> M(data);
      const double rowError
          = (M.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).cwiseAbs().maxCoeff();
      if (rowError > kTolerance)
        throw nb::value_error(
            ("matrix must have homogeneous row [0, 0, 0, 1] (error "
             + number(rowError) + ")")
                .c_str());
      parsed.rotations.push_back(
          validateRotation(M.topLeftCorner<3, 3>(), normalize, "matrix"));
      parsed.translations.push_back(M.topRightCorner<3, 1>());
    });
  }

  bool translationBatched = false;
  if (!translation.is_none()) {
    InArray array = asArray(translation);
    requireShape(array, 3, 0, "translation", "(3,) or (N, 3)");
    requireFinite(array, "translation");
    forEachItem(array, 3, 1, translationBatched, [&](const double* data) {
      parsed.translations.push_back(Eigen::Map<const Eigen::Vector3d>(data));
    });
  }

  if (parsed.rotations.empty())
    parsed.rotations.push_back(Eigen::Matrix3d::Identity());
  if (parsed.translations.empty())
    parsed.translations.push_back(Eigen::Vector3d::Zero());
  if (matrix.is_none() && rotationBatched && translationBatched
      && parsed.rotations.size() != parsed.translations.size())
    throw nb::value_error(("rotation batch of "
                           + std::to_string(parsed.rotations.size())
                           + " does not match translation batch of "
                           + std::to_string(parsed.translations.size()))
                              .c_str());
  parsed.batched = rotationBatched || translationBatched;
  return parsed;
}

template <class T>
T build(const Parsed& parsed)
{
  T out;
  out.batched = parsed.batched;
  const std::size_t n
      = std::max(parsed.rotations.size(), parsed.translations.size());
  out.items.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    Isometry item = Isometry::Identity();
    item.linear() = parsed.rotations[parsed.rotations.size() == 1 ? 0 : i];
    item.translation()
        = parsed.translations[parsed.translations.size() == 1 ? 0 : i];
    out.items.push_back(item);
  }
  return out;
}

template <class T>
T fromStorage(Storage items, bool batched)
{
  T out;
  out.items = std::move(items);
  out.batched = batched;
  return out;
}

// The checks construction applies to a complete 4x4 item: finite entries, the
// homogeneous row and a rotation within kTolerance. Rotations also need a
// zero translation. Shared by from_isometry and unpickling.
template <class T>
// Pickle state restores exact bits without projecting, so it checks the
// rotation against the same envelope that normalize=True accepts (positive
// determinant, singular values near 1): composition may drift past the
// construction tolerance, but scale and shear in crafted bytes are rejected.
Isometry validateItem(
    const Eigen::Matrix4d& M, const char* name, bool checkRotation = true)
{
  if (!M.allFinite())
    throw nb::value_error((std::string(name) + " must be finite").c_str());
  const double rowError
      = (M.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).cwiseAbs().maxCoeff();
  if (rowError > kTolerance)
    throw nb::value_error(
        (std::string(name) + " must have homogeneous row [0, 0, 0, 1]")
            .c_str());
  Isometry item = Isometry::Identity();
  if (checkRotation) {
    item.linear() = validateRotation(M.topLeftCorner<3, 3>(), false, name);
  } else {
    const Eigen::Matrix3d R = M.topLeftCorner<3, 3>();
    if (!(R.determinant() > 0.0))
      throw nb::value_error(
          (std::string(name) + " must have a positive determinant").c_str());
    validateRotation(R, true, name); // envelope check; projection discarded
    item.linear() = R;
  }
  item.translation() = M.topRightCorner<3, 1>();
  if constexpr (std::is_same_v<T, RotationValue>) {
    if (!item.translation().isZero(0))
      throw nb::value_error(
          (std::string(name) + " for a Rotation must have zero translation")
              .c_str());
  }
  return item;
}

// Keep SVD and error formatting out of the composition fast path.
EIGEN_DONT_INLINE void requireResultEnvelopeFull(
    const Eigen::Matrix3d& R, const char* op, bool batched, std::size_t index)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R);
  if (R.determinant() > 0 && Deviation(svd.singularValues()).nearRotation())
    return;
  std::string name = std::string(op) + " result";
  if (batched)
    name += " at batch index " + std::to_string(index);
  throw nb::value_error(
      (name
       + " rotation is outside the near-rotation envelope; rebuild the "
         "drifted operand from its matrix with normalize=True before "
         "composing or inverting it")
          .c_str());
}

void requireResultEnvelope(
    const Isometry& item, const char* op, bool batched, std::size_t index)
{
  const auto& R = item.matrix();
  // R^T R - I is symmetric; compute its six distinct entries directly.
  const double g00
      = R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0) + R(2, 0) * R(2, 0) - 1;
  const double g11
      = R(0, 1) * R(0, 1) + R(1, 1) * R(1, 1) + R(2, 1) * R(2, 1) - 1;
  const double g22
      = R(0, 2) * R(0, 2) + R(1, 2) * R(1, 2) + R(2, 2) * R(2, 2) - 1;
  const double g01 = R(0, 0) * R(0, 1) + R(1, 0) * R(1, 1) + R(2, 0) * R(2, 1);
  const double g02 = R(0, 0) * R(0, 2) + R(1, 0) * R(1, 2) + R(2, 0) * R(2, 2);
  const double g12 = R(0, 1) * R(0, 2) + R(1, 1) * R(1, 2) + R(2, 1) * R(2, 2);
  // Products and inverses retain positive determinants; this bound also
  // implies the singular-value stretch and spread limits.
  if (g00 * g00 + g11 * g11 + g22 * g22
          + 2 * (g01 * g01 + g02 * g02 + g12 * g12)
      <= 1e-6)
    return;
  requireResultEnvelopeFull(item.linear(), op, batched, index);
}

// scalar (x) batch broadcasts; batches must have equal length.
template <class F>
Storage zip(const Poses& a, const Poses& b, const char* op, F&& fn)
{
  if (a.batched && b.batched && a.count() != b.count())
    throw nb::value_error((std::string(op) + ": batch sizes "
                           + std::to_string(a.count()) + " and "
                           + std::to_string(b.count()) + " do not match")
                              .c_str());
  Storage out;
  if (!a.batched && !b.batched) {
    out.push_back(fn(a.items[0], b.items[0], 0));
    return out;
  }
  const std::size_t n = std::max(a.count(), b.count());
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    out.push_back(
        fn(a.items[a.batched ? i : 0], b.items[b.batched ? i : 0], i));
  return out;
}

nb::object compose(const Poses& a, const Poses& b, bool rotationResult)
{
  const bool batched = a.batched || b.batched;
  Storage out = zip(
      a,
      b,
      "@",
      [batched](const Isometry& x, const Isometry& y, std::size_t i) {
        Isometry result(x * y);
        requireResultEnvelope(result, "@", batched, i);
        return result;
      });
  if (rotationResult)
    return nb::cast(fromStorage<RotationValue>(std::move(out), batched));
  return nb::cast(fromStorage<TransformValue>(std::move(out), batched));
}

nb::object applyPoints(const Poses& self, nb::handle obj)
{
  InArray points = asArray(obj);
  if (points.ndim() == 0 || points.shape(points.ndim() - 1) != 3)
    throw nb::value_error("points must have shape (..., 3)");
  const double* data = points.data();
  if (!self.batched) {
    const std::size_t n = points.size() / 3;
    const Isometry& X = self.items[0];
    return makeArray(
        points.shapeVector().data(), points.ndim(), [&](double* buffer) {
          for (std::size_t i = 0; i < n; ++i)
            Eigen::Map<Eigen::Vector3d>(buffer + 3 * i)
                = X * Eigen::Map<const Eigen::Vector3d>(data + 3 * i);
        });
  }
  const std::size_t n = self.count();
  const bool pairwise = points.ndim() == 2 && points.shape(0) == n;
  if (!pairwise && points.ndim() != 1)
    throw nb::value_error(("a batch of " + std::to_string(n)
                           + " accepts points of shape (3,) or ("
                           + std::to_string(n) + ", 3)")
                              .c_str());
  return makeArray({n, 3}, [&](double* buffer) {
    for (std::size_t i = 0; i < n; ++i)
      Eigen::Map<Eigen::Vector3d>(buffer + 3 * i)
          = self.items[i]
            * Eigen::Map<const Eigen::Vector3d>(data + (pairwise ? 3 * i : 0));
  });
}

nb::object matmul(nb::handle self, nb::handle other)
{
  const bool selfRotation = nb::isinstance<RotationValue>(self);
  const Poses& a
      = selfRotation
            ? static_cast<const Poses&>(nb::cast<const RotationValue&>(self))
            : nb::cast<const TransformValue&>(self);
  if (nb::isinstance<TransformValue>(other))
    return compose(a, nb::cast<const TransformValue&>(other), false);
  if (nb::isinstance<RotationValue>(other))
    return compose(a, nb::cast<const RotationValue&>(other), selfRotation);
  return applyPoints(a, other);
}

template <class T>
nb::object quaternion(const T& self, bool scalarFirst)
{
  const std::size_t n = self.count();
  auto fill = [&](double* buffer) {
    for (std::size_t i = 0; i < n; ++i) {
      const Eigen::Vector4d q = toWxyz(self.items[i].linear());
      Eigen::Map<Eigen::Vector4d> out(buffer + 4 * i);
      if (scalarFirst)
        out = q;
      else
        out << q[1], q[2], q[3], q[0];
    }
  };
  return self.batched ? makeArray({n, 4}, fill) : makeArray({4}, fill);
}

template <class T>
Py_hash_t hashValue(const T& self)
{
  if (self.batched)
    throw nb::type_error("batched values are unhashable; index an item");
  // Bit-exact, like ==: -0.0 and 0.0 hash differently. The static_assert on
  // sizeof(Isometry) guarantees the 16 doubles are contiguous and unpadded.
  const auto h
      = static_cast<Py_hash_t>(std::hash<std::string_view>{}(std::string_view(
          reinterpret_cast<const char*>(self.items[0].data()),
          sizeof(Isometry))));
  return h == -1 ? -2 : h; // -1 is CPython's error sentinel
}

template <class T>
nb::object equal(const T& self, nb::handle otherArg)
{
  if (!nb::isinstance<T>(otherArg))
    return nb::not_implemented();
  const T& other = nb::cast<const T&>(otherArg);
  if (self.batched != other.batched || self.count() != other.count())
    return nb::bool_(false);
  return nb::bool_(
      std::memcmp(
          self.items.data(),
          other.items.data(),
          sizeof(double) * 16 * self.count())
      == 0);
}

template <class T>
T getItem(const T& self, Py_ssize_t index)
{
  if (!self.batched)
    throw nb::type_error("a scalar value is not indexable");
  const Py_ssize_t n = static_cast<Py_ssize_t>(self.count());
  if (index < 0)
    index += n;
  if (index < 0 || index >= n)
    throw nb::index_error("index out of range");
  return fromStorage<T>({self.items[static_cast<std::size_t>(index)]}, false);
}

template <class T>
std::size_t length(const T& self)
{
  if (!self.batched)
    throw nb::type_error("a scalar value has no len(); use a batch");
  return self.count();
}

template <class T>
nb::tuple getState(const T& self)
{
  return nb::make_tuple(
      self.batched,
      nb::bytes(
          reinterpret_cast<const char*>(self.items.data()),
          sizeof(double) * 16 * self.count()));
}

template <class T>
void setState(T& self, nb::tuple state)
{
  if (state.size() != 2)
    throw nb::value_error("invalid pickle state");
  if (!nb::isinstance<nb::bool_>(state[0])
      || !nb::isinstance<nb::bytes>(state[1]))
    throw nb::value_error("invalid pickle state");
  const bool batched = nb::cast<bool>(state[0]);
  nb::bytes raw = nb::cast<nb::bytes>(state[1]);
  const std::size_t itemBytes = sizeof(Isometry);
  if (raw.size() % itemBytes != 0 || (raw.size() == 0)
      || (!batched && raw.size() != itemBytes))
    throw nb::value_error("invalid pickle state: wrong byte length");
  // Crafted bytes get the normalize=True envelope check (see validateItem).
  Storage items;
  items.reserve(raw.size() / itemBytes);
  for (std::size_t i = 0; i < raw.size() / itemBytes; ++i) {
    Eigen::Matrix4d M;
    std::memcpy(M.data(), raw.c_str() + itemBytes * i, itemBytes);
    items.push_back(validateItem<T>(M, "pickle state", false));
  }
  new (&self) T(fromStorage<T>(std::move(items), batched));
}

nb::str formatVector(const double* data, std::size_t n)
{
  nb::list values;
  for (std::size_t i = 0; i < n; ++i)
    values.append(nb::float_(data[i]));
  return nb::str(nb::repr(values));
}

template <class T>
nb::str repr(const T& self, const char* name)
{
  if (self.batched)
    return nb::str("{}(<batch of {}>)").format(name, self.count());
  const Eigen::Vector4d q = toWxyz(self.items[0].linear());
  nb::str out = nb::str("{}(wxyz={}").format(name, formatVector(q.data(), 4));
  if constexpr (std::is_same_v<T, TransformValue>) {
    const Eigen::Vector3d t = self.items[0].translation();
    out = nb::str("{}, translation={}").format(out, formatVector(t.data(), 3));
  }
  return nb::str("{})").format(out);
}

template <class T>
T inverse(const T& self)
{
  Storage out;
  out.reserve(self.count());
  for (const Isometry& item : self.items) {
    // A general inverse of the stored rotation block: values may carry drift
    // within the tolerance, which the transpose would double.
    out.push_back(item.inverse(Eigen::Affine));
    requireResultEnvelope(
        out.back(), "inverse()", self.batched, out.size() - 1);
  }
  return fromStorage<T>(std::move(out), self.batched);
}

template <class T, class Class>
void bindCommon(Class& cls, const char* name)
{
  cls.def_prop_ro(
         "wxyz",
         [](const T& self) { return quaternion(self, true); },
         nb::rv_policy::automatic,
         "Unit quaternion as (w, x, y, z), w >= 0; (N, 4) for a batch.")
      .def_prop_ro(
          "xyzw",
          [](const T& self) { return quaternion(self, false); },
          nb::rv_policy::automatic,
          "Unit quaternion as (x, y, z, w), w >= 0; (N, 4) for a batch.")
      .def(
          "inverse",
          &inverse<T>,
          "Inverse of the stored value. A rotation accepted within the "
          "tolerance or drifted by composition is inverted as stored, not "
          "transposed, so value @ value.inverse() is the identity to "
          "floating-point precision. Raises ValueError if the result leaves "
          "the normalize=True near-rotation envelope.")
      .def(
          "__matmul__",
          [](nb::handle self, nb::handle other) { return matmul(self, other); },
          nb::arg("other"))
      .def(
          "__rmatmul__",
          [](nb::handle, nb::handle) -> nb::object {
            throw nb::type_error(
                "points @ value is not defined; use value @ points");
          },
          nb::arg("other"))
      .def("__len__", &length<T>)
      .def("__getitem__", &getItem<T>, nb::arg("index"))
      .def("__eq__", &equal<T>, nb::arg("other"))
      .def("__hash__", &hashValue<T>)
      .def("__repr__", [name](const T& self) { return repr(self, name); })
      .def("__getstate__", &getState<T>)
      .def("__setstate__", &setState<T>)
      .def_static("identity", [] {
        return fromStorage<T>({Isometry::Identity()}, false);
      });
  cls.attr("__array_ufunc__") = nb::none();
}

} // namespace

void Transform(nb::module_& sm)
{
  nb::class_<RotationValue> rotation(
      sm,
      "Rotation",
      nb::is_final(),
      "Immutable 3D rotation value, scalar or batched (N).");
  rotation
      .def(
          "__init__",
          [](RotationValue* self,
             nb::handle rotationArg,
             nb::handle wxyz,
             nb::handle xyzw,
             nb::handle rpy,
             bool normalize) {
            new (self) RotationValue(build<RotationValue>(parse(
                rotationArg,
                wxyz,
                xyzw,
                rpy,
                nb::none(),
                nb::none(),
                normalize,
                "Rotation")));
          },
          nb::kw_only(),
          nb::arg("rotation") = nb::none(),
          nb::arg("wxyz") = nb::none(),
          nb::arg("xyzw") = nb::none(),
          nb::arg("rpy") = nb::none(),
          nb::arg("normalize") = false,
          "Build from at most one of rotation= (3x3 or Rotation), wxyz=, "
          "xyzw= or rpy= (extrinsic X-Y-Z: Rz(yaw) Ry(pitch) Rx(roll)); no "
          "form gives the identity. Arrays with a leading N >= 1 build a "
          "batch.")
      .def_static(
          "from_axis_angle",
          [](nb::handle axisArg, double angle) {
            InArray axis = asArray(axisArg);
            if (axis.ndim() != 1 || axis.shape(0) != 3)
              throw nb::value_error("axis must have shape (3,)");
            requireFinite(axis, "axis");
            if (!std::isfinite(angle))
              throw nb::value_error("angle must be finite");
            Eigen::Vector3d direction
                = Eigen::Map<const Eigen::Vector3d>(axis.data());
            // Divide by the largest component first so huge and subnormal
            // axes normalize without overflow or underflow.
            const double scale = direction.cwiseAbs().maxCoeff();
            if (scale == 0)
              throw nb::value_error("axis must be nonzero");
            direction /= scale;
            direction /= direction.norm();
            Isometry item = Isometry::Identity();
            item.linear()
                = Eigen::AngleAxisd(angle, direction).toRotationMatrix();
            return fromStorage<RotationValue>({item}, false);
          },
          nb::arg("axis"),
          nb::arg("angle"),
          "Rotation by angle (radians) about axis, which is scaled to unit "
          "length.")
      .def_prop_ro(
          "matrix",
          [](const RotationValue& self) {
            const std::size_t n = self.count();
            auto fill = [&](double* buffer) {
              for (std::size_t i = 0; i < n; ++i)
                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                    buffer + 9 * i)
                    = self.items[i].linear();
            };
            return self.batched ? makeArray({n, 3, 3}, fill)
                                : makeArray({3, 3}, fill);
          },
          nb::rv_policy::automatic,
          "Read-only (3, 3) or (N, 3, 3) rotation matrix copy.")
      .def(
          "as_rpy",
          [](const RotationValue& self) {
            const std::size_t n = self.count();
            auto fill = [&](double* buffer) {
              for (std::size_t i = 0; i < n; ++i)
                Eigen::Map<Eigen::Vector3d>(buffer + 3 * i)
                    = toRpy(self.items[i].linear());
            };
            return self.batched ? makeArray({n, 3}, fill)
                                : makeArray({3}, fill);
          },
          "(roll, pitch, yaw) with R = Rz(yaw) Ry(pitch) Rx(roll); roll is 0 "
          "at "
          "gimbal lock.");
  bindCommon<RotationValue>(rotation, "Rotation");

  nb::class_<TransformValue> transform(
      sm,
      "Transform",
      nb::is_final(),
      "Immutable rigid transform value, scalar or batched (N).");
  transform
      .def(
          "__init__",
          [](TransformValue* self,
             nb::handle rotationArg,
             nb::handle wxyz,
             nb::handle xyzw,
             nb::handle rpy,
             nb::handle matrix,
             nb::handle translation,
             bool normalize) {
            new (self) TransformValue(build<TransformValue>(parse(
                rotationArg,
                wxyz,
                xyzw,
                rpy,
                matrix,
                translation,
                normalize,
                "Transform")));
          },
          nb::kw_only(),
          nb::arg("rotation") = nb::none(),
          nb::arg("wxyz") = nb::none(),
          nb::arg("xyzw") = nb::none(),
          nb::arg("rpy") = nb::none(),
          nb::arg("matrix") = nb::none(),
          nb::arg("translation") = nb::none(),
          nb::arg("normalize") = false,
          "Build from at most one rotation form (rotation=, wxyz=, xyzw=, rpy= "
          "or a homogeneous matrix=) plus translation=; omitted parts are the "
          "identity. Arrays with a leading N >= 1 build a batch.")
      .def_static(
          "from_isometry",
          [](nb::handle iso) {
            if (!nb::isinstance<Isometry>(iso))
              throw nb::type_error(
                  "from_isometry expects dartpy.math.Isometry3");
            const Isometry& value = nb::cast<const Isometry&>(iso);
            return fromStorage<TransformValue>(
                {validateItem<TransformValue>(value.matrix(), "isometry")},
                false);
          },
          nb::arg("isometry"),
          "Validated copy of a dartpy.math.Isometry3.")
      .def(
          "to_isometry",
          [](const TransformValue& self) {
            if (self.batched)
              throw nb::type_error(
                  "to_isometry needs a scalar Transform; index the batch");
            return self.items[0];
          },
          "Mutable dartpy.math.Isometry3 copy for the camelCase API.")
      .def_prop_ro(
          "translation",
          [](const TransformValue& self) {
            const std::size_t n = self.count();
            auto fill = [&](double* buffer) {
              for (std::size_t i = 0; i < n; ++i)
                Eigen::Map<Eigen::Vector3d>(buffer + 3 * i)
                    = self.items[i].translation();
            };
            return self.batched ? makeArray({n, 3}, fill)
                                : makeArray({3}, fill);
          },
          nb::rv_policy::automatic,
          "Read-only (3,) or (N, 3) translation copy.")
      .def_prop_ro(
          "rotation",
          [](const TransformValue& self) {
            Storage items;
            items.reserve(self.count());
            for (const Isometry& item : self.items) {
              Isometry rotationOnly = Isometry::Identity();
              rotationOnly.linear() = item.linear();
              items.push_back(rotationOnly);
            }
            return fromStorage<RotationValue>(std::move(items), self.batched);
          },
          "The rotation part as a Rotation.")
      .def_prop_ro(
          "matrix",
          [](const TransformValue& self) {
            const std::size_t n = self.count();
            auto fill = [&](double* buffer) {
              for (std::size_t i = 0; i < n; ++i)
                Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
                    buffer + 16 * i)
                    = self.items[i].matrix();
            };
            return self.batched ? makeArray({n, 4, 4}, fill)
                                : makeArray({4, 4}, fill);
          },
          nb::rv_policy::automatic,
          "Read-only (4, 4) or (N, 4, 4) homogeneous matrix copy.");
  bindCommon<TransformValue>(transform, "Transform");
}

} // namespace python
} // namespace dart
