//
// All components of Drake are licensed under the BSD 3-Clause License
// shown below. Where noted in the source code, some portions may
// be subject to other permissive, non-viral licenses.
//
// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "eigen_geometry_pybind.h"

#include <cassert>
#include <cmath>

#include "dart/common/all.hpp"

#include "pybind11/pybind11.h"

using std::fabs;

namespace dart {
namespace python {
namespace {

// TODO(eric.cousineau): There is validation from Python to C++, but no
// validation in the other direction. Consider intercepting this.

// TODO(eric.cousineau): Add operator overloads.

// TODO(eric.cousineau): Disable tolerance checks for the symbolic case.

// N.B. This could potentially interfere with another library's bindings of
// Eigen types. If/when this happens, this should be addressed for both these
// and AutoDiff types.

// N.B. Use a loose tolerance, so that we don't have to be super strict with
// C++.
[[maybe_unused]] const double kCheckTolerance = 1e-5;

template <typename T>
void CheckRotMat(const Eigen::Matrix<T, 3, 3>& R)
{
  // See `ExpectRotMat`.
  const T identity_error
      = (R * R.transpose() - Eigen::Matrix<T, 3, 3>::Identity())
            .array()
            .abs()
            .maxCoeff();
  DART_UNUSED(identity_error);
  assert(
      identity_error < kCheckTolerance && "Rotation matrix is not orthonormal");
  const T det_error = fabs(R.determinant() - 1);
  DART_UNUSED(det_error);
  assert(
      det_error < kCheckTolerance
      && "Rotation matrix violates right-hand rule");
}

template <typename T>
void CheckIsometry(const Eigen::Transform<T, 3, Eigen::Isometry>& X)
{
  CheckRotMat<T>(X.linear());
  Eigen::Matrix<T, 1, 4> bottom_expected;
  bottom_expected << 0, 0, 0, 1;
  const T bottom_error
      = (X.matrix().bottomRows(1) - bottom_expected).array().abs().maxCoeff();

  DART_UNUSED(bottom_error);
  assert(
      bottom_error < kCheckTolerance
      && "Homogeneous matrix is improperly scaled.");
}

template <typename T>
void CheckQuaternion(const Eigen::Quaternion<T>& q)
{
  const T norm_error = fabs(q.coeffs().norm() - 1);
  DART_UNUSED(norm_error);
  assert(norm_error < kCheckTolerance && "Quaternion is not normalized");
}

template <typename T>
void CheckAngleAxis(const Eigen::AngleAxis<T>& value)
{
  const T norm_error = fabs(value.axis().norm() - 1);
  DART_UNUSED(norm_error);
  assert(norm_error < kCheckTolerance && "Axis is not normalized");
}

} // namespace

// PYBIND11_MODULE(eigen_geometry, m) {
void eigen_geometry(pybind11::module& parent_m)
{
  auto m = parent_m.def_submodule("math");

  m.doc() = "Bindings for Eigen geometric types.";

  using T = double;

  // Do not return references to matrices (e.g. `Eigen::Ref<>`) so that we have
  // tighter control over validation.

  // Isometry3d.
  // @note `linear` implies rotation, and `affine` implies translation.
  {
    using Class = Eigen::Transform<T, 3, Eigen::Isometry>;
    ::pybind11::class_<Class> py_class(m, "Isometry3");
    py_class
        .def(::pybind11::init([]() {
          return Class::Identity();
        }))
        .def_static(
            "Identity",
            []() {
              return Class::Identity();
            })
        .def(
            ::pybind11::init([](const Eigen::Matrix<T, 4, 4>& matrix) {
              Class out(matrix);
              CheckIsometry(out);
              return out;
            }),
            ::pybind11::arg("matrix"))
        .def(
            ::pybind11::init([](const Eigen::Matrix<T, 3, 3>& rotation,
                                const Eigen::Matrix<T, 3, 1>& translation) {
              CheckRotMat(rotation);
              Class out = Class::Identity();
              out.linear() = rotation;
              out.translation() = translation;
              return out;
            }),
            ::pybind11::arg("rotation"),
            ::pybind11::arg("translation"))
        .def(
            ::pybind11::init([](const Eigen::Quaternion<T>& q,
                                const Eigen::Matrix<T, 3, 1>& translation) {
              CheckQuaternion(q);
              Class out = Class::Identity();
              out.linear() = q.toRotationMatrix();
              out.translation() = translation;
              return out;
            }),
            ::pybind11::arg("quaternion"),
            ::pybind11::arg("translation"))
        .def(
            ::pybind11::init([](const Class& other) {
              CheckIsometry(other);
              return other;
            }),
            ::pybind11::arg("other"))
        .def(
            "matrix",
            [](const Class* self) -> Eigen::Matrix<T, 4, 4> {
              return self->matrix();
            })
        .def(
            "set_matrix",
            [](Class* self, const Eigen::Matrix<T, 4, 4>& matrix) {
              Class update(matrix);
              CheckIsometry(update);
              *self = update;
            })
        .def(
            "translation",
            [](const Class* self) -> Eigen::Matrix<T, 3, 1> {
              return self->translation();
            })
        .def(
            "set_translation",
            [](Class* self, const Eigen::Matrix<T, 3, 1>& translation) {
              self->translation() = translation;
            })
        .def(
            "rotation",
            [](const Class* self) -> Eigen::Matrix<T, 3, 3> {
              return self->linear();
            })
        .def(
            "set_rotation",
            [](Class* self, const Eigen::Matrix<T, 3, 3>& rotation) {
              CheckRotMat(rotation);
              self->linear() = rotation;
            })
        .def(
            "quaternion",
            [](const Class* self) {
              return Eigen::Quaternion<T>(self->linear());
            })
        .def(
            "set_quaternion",
            [](Class* self, const Eigen::Quaternion<T>& q) {
              CheckQuaternion(q);
              self->linear() = q.toRotationMatrix();
            })
        .def(
            "__str__",
            [](::pybind11::object self) {
              return ::pybind11::str(self.attr("matrix")());
            })
        // Do not define operator `__mul__` until we have the Python3 `@`
        // operator so that operations are similar to those of arrays.
        .def(
            "multiply",
            [](const Class& self, const Class& other) {
              return self * other;
            },
            ::pybind11::arg("other"))
        .def(
            "multiply",
            [](const Class& self, const Eigen::Matrix<T, 3, 1>& position) {
              return self * position;
            },
            ::pybind11::arg("position"))
        .def(
            "inverse",
            [](const Class* self) {
              return self->inverse();
            })
        //========================
        // Begin: added by dartpy
        //========================
        .def(
            "translate",
            [](Class* self, const Eigen::Matrix<T, 3, 1>& other) {
              self->translate(other);
            },
            ::pybind11::arg("other"))
        .def(
            "pretranslate",
            [](Class* self, const Eigen::Matrix<T, 3, 1>& other) {
              self->pretranslate(other);
            },
            ::pybind11::arg("other"))
        //========================
        // End: added by dartpy
        //========================
        ;
    ::pybind11::implicitly_convertible<Eigen::Matrix<T, 4, 4>, Class>();
  }

  // Quaternion.
  // Since the Eigen API for Quaternion is insufficiently explicit, we will
  // deviate some from the API to maintain clarity.
  // TODO(eric.cousineau): Should this not be restricted to a unit quaternion?
  {
    using Class = Eigen::Quaternion<T>;
    ::pybind11::class_<Class> py_class(m, "Quaternion");
    py_class.attr("__doc__")
        = "Provides a unit quaternion binding of Eigen::Quaternion<>.";
    ::pybind11::object py_class_obj = py_class;
    py_class
        .def(::pybind11::init([]() {
          return Class::Identity();
        }))
        .def_static(
            "Identity",
            []() {
              return Class::Identity();
            })
        .def(
            ::pybind11::init([](const Eigen::Matrix<T, 4, 1>& wxyz) {
              Class out(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
              CheckQuaternion(out);
              return out;
            }),
            ::pybind11::arg("wxyz"))
        .def(
            ::pybind11::init([](T w, T x, T y, T z) {
              Class out(w, x, y, z);
              CheckQuaternion(out);
              return out;
            }),
            ::pybind11::arg("w"),
            ::pybind11::arg("x"),
            ::pybind11::arg("y"),
            ::pybind11::arg("z"))
        .def(
            ::pybind11::init([](const Eigen::Matrix<T, 3, 3>& rotation) {
              Class out(rotation);
              CheckQuaternion(out);
              return out;
            }),
            ::pybind11::arg("rotation"))
        .def(
            ::pybind11::init([](const Class& other) {
              CheckQuaternion(other);
              return other;
            }),
            ::pybind11::arg("other"))
        .def(
            "w",
            [](const Class* self) {
              return self->w();
            })
        .def(
            "x",
            [](const Class* self) {
              return self->x();
            })
        .def(
            "y",
            [](const Class* self) {
              return self->y();
            })
        .def(
            "z",
            [](const Class* self) {
              return self->z();
            })
        .def(
            "xyz",
            [](const Class* self) {
              return self->vec();
            })
        .def(
            "wxyz",
            [](Class* self) {
              Eigen::Matrix<T, 4, 1> wxyz;
              wxyz << self->w(), self->vec();
              return wxyz;
            })
        .def(
            "set_wxyz",
            [](Class* self, const Eigen::Matrix<T, 4, 1>& wxyz) {
              Class update;
              update.w() = wxyz(0);
              update.vec() = wxyz.tail(3);
              CheckQuaternion(update);
              *self = update;
            },
            ::pybind11::arg("wxyz"))
        .def(
            "set_wxyz",
            [](Class* self, T w, T x, T y, T z) {
              Class update(w, x, y, z);
              CheckQuaternion(update);
              *self = update;
            },
            ::pybind11::arg("w"),
            ::pybind11::arg("x"),
            ::pybind11::arg("y"),
            ::pybind11::arg("z"))
        .def(
            "rotation",
            [](const Class* self) {
              return self->toRotationMatrix();
            })
        .def(
            "set_rotation",
            [](Class* self, const Eigen::Matrix<T, 3, 3>& rotation) {
              Class update(rotation);
              CheckQuaternion(update);
              *self = update;
            })
        .def(
            "__str__",
            [py_class_obj](const Class* self) {
              return ::pybind11::str("{}(w={}, x={}, y={}, z={})")
                  .format(
                      py_class_obj.attr("__name__"),
                      self->w(),
                      self->x(),
                      self->y(),
                      self->z());
            })
        // Do not define operator `__mul__` until we have the Python3 `@`
        // operator so that operations are similar to those of arrays.
        .def(
            "multiply",
            [](const Class& self, const Class& other) {
              return self * other;
            })
        .def(
            "multiply",
            [](const Class& self, const Eigen::Matrix<T, 3, 1>& position) {
              return self * position;
            },
            ::pybind11::arg("position"))
        .def(
            "inverse",
            [](const Class* self) {
              return self->inverse();
            })
        .def(
            "conjugate",
            [](const Class* self) {
              return self->conjugate();
            })
        //========================
        // Begin: added by dartpy
        //========================
        .def(
            "to_rotation_matrix",
            [](Class* self) -> Eigen::Matrix<T, 3, 3> {
              return self->toRotationMatrix();
            })
        //========================
        // End: added by dartpy
        //========================
        ;
  }

  // Angle-axis.
  {
    using Class = Eigen::AngleAxis<T>;
    ::pybind11::class_<Class> py_class(m, "AngleAxis");
    py_class.attr("__doc__") = "Bindings for Eigen::AngleAxis<>.";
    ::pybind11::object py_class_obj = py_class;
    py_class
        .def(::pybind11::init([]() {
          return Class::Identity();
        }))
        .def_static(
            "Identity",
            []() {
              return Class::Identity();
            })
        .def(
            ::pybind11::init(
                [](const T& angle, const Eigen::Matrix<T, 3, 1>& axis) {
                  Class out(angle, axis);
                  CheckAngleAxis(out);
                  return out;
                }),
            ::pybind11::arg("angle"),
            ::pybind11::arg("axis"))
        .def(
            ::pybind11::init([](const Eigen::Quaternion<T>& q) {
              Class out(q);
              CheckAngleAxis(out);
              return out;
            }),
            ::pybind11::arg("quaternion"))
        .def(
            ::pybind11::init([](const Eigen::Matrix<T, 3, 3>& rotation) {
              Class out(rotation);
              CheckAngleAxis(out);
              return out;
            }),
            ::pybind11::arg("rotation"))
        .def(
            ::pybind11::init([](const Class& other) {
              CheckAngleAxis(other);
              return other;
            }),
            ::pybind11::arg("other"))
        .def(
            "angle",
            [](const Class* self) {
              return self->angle();
            })
        .def(
            "axis",
            [](const Class* self) {
              return self->axis();
            })
        .def(
            "set_angle",
            [](Class* self, const T& angle) {
              // N.B. Since `axis` should already be valid, do not need to
              // check.
              self->angle() = angle;
            },
            ::pybind11::arg("angle"))
        .def(
            "set_axis",
            [](Class* self, const Eigen::Matrix<T, 3, 1>& axis) {
              Class update(self->angle(), axis);
              CheckAngleAxis(update);
              *self = update;
            },
            ::pybind11::arg("axis"))
        .def(
            "rotation",
            [](const Class* self) {
              return self->toRotationMatrix();
            })
        .def(
            "set_rotation",
            [](Class* self, const Eigen::Matrix<T, 3, 3>& rotation) {
              Class update(rotation);
              CheckAngleAxis(update);
              *self = update;
            })
        .def(
            "quaternion",
            [](const Class* self) {
              return Eigen::Quaternion<T>(*self);
            })
        .def(
            "set_quaternion",
            [](Class* self, const Eigen::Quaternion<T>& q) {
              CheckQuaternion(q);
              Class update(q);
              CheckAngleAxis(update);
              *self = update;
            })
        .def(
            "__str__",
            [py_class_obj](const Class* self) {
              return ::pybind11::str("{}(angle={}, axis={})")
                  .format(
                      py_class_obj.attr("__name__"),
                      self->angle(),
                      self->axis());
            })
        // Do not define operator `__mul__` until we have the Python3 `@`
        // operator so that operations are similar to those of arrays.
        .def(
            "multiply",
            [](const Class& self, const Class& other) {
              return self * other;
            })
        .def(
            "inverse",
            [](const Class* self) {
              return self->inverse();
            })
        //========================
        // Begin: added by dartpy
        //========================
        .def(
            "to_rotation_matrix",
            [](Class* self) -> Eigen::Matrix<T, 3, 3> {
              return self->toRotationMatrix();
            })
        //========================
        // End: added by dartpy
        //========================
        ;
  }
}

} // namespace python
} // namespace dart
