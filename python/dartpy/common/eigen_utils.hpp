#pragma once

#include <Eigen/Dense>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/vector.h>

#include <array>
#include <ranges>
#include <vector>

namespace dart::python_nb {

// Returns the element stride (in elements) for the given axis of a numpy
// ndarray, falling back to a C-contiguous layout when strides are unavailable.
// Kept at binding-namespace scope (not detail) so binding sources may share it
// without crossing the public/internal API boundary.
inline int64_t numpyStride(const nanobind::ndarray<double>& array, size_t axis)
{
  if (array.stride_ptr()) {
    return array.stride(axis);
  }
  int64_t s = 1;
  for (size_t i = array.ndim(); i-- > axis + 1;) {
    s *= array.shape(i);
  }
  return s;
}

namespace detail {

template <typename Array>
inline Eigen::Vector3d vector3FromArray(const Array& values)
{
  Eigen::Vector3d out;
  out << values[0], values[1], values[2];
  return out;
}

inline Eigen::Vector3d vector3FromNumpy(const nanobind::ndarray<double>& array)
{
  if (array.ndim() == 1 && array.shape(0) == 3) {
    return Eigen::Vector3d(
        array.data()[0],
        array.data()[numpyStride(array, 0)],
        array.data()[2 * numpyStride(array, 0)]);
  }

  if (array.ndim() == 2) {
    const double* base = array.data();
    const int64_t s0 = numpyStride(array, 0);
    const int64_t s1 = numpyStride(array, 1);
    if (array.shape(0) == 3 && array.shape(1) == 1) {
      Eigen::Vector3d out;
      for (const auto i : std::views::iota(Eigen::Index{0}, Eigen::Index{3})) {
        out[i] = base[i * s0 + 0 * s1];
      }
      return out;
    }
    if (array.shape(0) == 1 && array.shape(1) == 3) {
      Eigen::Vector3d out;
      for (const auto i : std::views::iota(Eigen::Index{0}, Eigen::Index{3})) {
        out[i] = base[0 * s0 + i * s1];
      }
      return out;
    }
  }

  throw nanobind::type_error("Expected a 3D vector");
}

inline Eigen::Matrix3d matrix3FromNumpy(const nanobind::ndarray<double>& array)
{
  if (array.ndim() != 2 || array.shape(0) != 3 || array.shape(1) != 3) {
    throw nanobind::type_error("Expected a 3x3 matrix");
  }

  Eigen::Matrix3d out;
  const double* base = array.data();
  const int64_t s0 = numpyStride(array, 0);
  const int64_t s1 = numpyStride(array, 1);
  for (const auto r : std::views::iota(Eigen::Index{0}, Eigen::Index{3})) {
    for (const auto c : std::views::iota(Eigen::Index{0}, Eigen::Index{3})) {
      out(r, c) = base[r * s0 + c * s1];
    }
  }
  return out;
}

inline Eigen::MatrixXd matrixFromArray(
    const std::vector<std::vector<double>>& rows)
{
  if (rows.empty()) {
    return Eigen::MatrixXd();
  }
  const std::size_t cols = rows.front().size();
  Eigen::MatrixXd out(rows.size(), cols);
  for (const auto r : std::views::iota(std::size_t{0}, rows.size())) {
    if (rows[r].size() != cols) {
      throw nanobind::type_error("Matrix rows must have consistent length");
    }
    for (const auto c : std::views::iota(std::size_t{0}, cols)) {
      out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c))
          = rows[r][c];
    }
  }
  return out;
}

} // namespace detail

inline Eigen::Vector3d toVector3(const nanobind::handle& value)
{
  if (nanobind::isinstance<nanobind::ndarray<double>>(value)) {
    return detail::vector3FromNumpy(
        nanobind::cast<nanobind::ndarray<double>>(value));
  }
  std::array<double, 3> data = nanobind::cast<std::array<double, 3>>(value);
  return detail::vector3FromArray(data);
}

inline Eigen::VectorXd toVector(const nanobind::handle& value)
{
  if (nanobind::isinstance<nanobind::ndarray<double>>(value)) {
    nanobind::ndarray<double> array
        = nanobind::cast<nanobind::ndarray<double>>(value);
    if (array.ndim() == 1) {
      Eigen::VectorXd out(array.shape(0));
      const double* base = array.data();
      const int64_t s0 = numpyStride(array, 0);
      for (const auto i : std::views::iota(Eigen::Index{0}, out.size())) {
        out[i] = base[i * s0];
      }
      return out;
    }
    if (array.ndim() == 2 && array.shape(1) == 1) {
      Eigen::VectorXd out(array.shape(0));
      const double* base = array.data();
      const int64_t s0 = numpyStride(array, 0);
      const int64_t s1 = numpyStride(array, 1);
      for (const auto i : std::views::iota(Eigen::Index{0}, out.size())) {
        out[i] = base[i * s0 + 0 * s1];
      }
      return out;
    }
    if (array.ndim() == 2 && array.shape(0) == 1) {
      Eigen::VectorXd out(array.shape(1));
      const double* base = array.data();
      const int64_t s0 = numpyStride(array, 0);
      const int64_t s1 = numpyStride(array, 1);
      for (const auto i : std::views::iota(Eigen::Index{0}, out.size())) {
        out[i] = base[0 * s0 + i * s1];
      }
      return out;
    }
  }

  std::vector<double> data = nanobind::cast<std::vector<double>>(value);
  Eigen::VectorXd out(data.size());
  for (const auto i : std::views::iota(Eigen::Index{0}, out.size())) {
    out[i] = data[static_cast<std::size_t>(i)];
  }
  return out;
}

inline Eigen::Matrix3d toMatrix3(const nanobind::handle& value)
{
  if (nanobind::isinstance<nanobind::ndarray<double>>(value)) {
    return detail::matrix3FromNumpy(
        nanobind::cast<nanobind::ndarray<double>>(value));
  }

  const auto nested
      = nanobind::cast<std::array<std::array<double, 3>, 3>>(value);
  Eigen::Matrix3d out;
  for (const auto r : std::views::iota(std::size_t{0}, nested.size())) {
    for (const auto c : std::views::iota(std::size_t{0}, nested[r].size())) {
      out(r, c) = nested[r][c];
    }
  }
  return out;
}

} // namespace dart::python_nb
