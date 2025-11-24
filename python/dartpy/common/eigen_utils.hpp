#pragma once

#include <Eigen/Dense>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/vector.h>

#include <array>
#include <vector>

namespace dart::python_nb {

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
  auto stride = [&](size_t axis) -> int64_t {
    if (array.stride_ptr())
      return array.stride(axis);
    int64_t s = 1;
    for (size_t i = array.ndim(); i-- > axis + 1;)
      s *= array.shape(i);
    return s;
  };

  if (array.ndim() == 1 && array.shape(0) == 3)
    return Eigen::Vector3d(
        array.data()[0],
        array.data()[stride(0)],
        array.data()[2 * stride(0)]);

  if (array.ndim() == 2) {
    const double* base = array.data();
    const int64_t s0 = stride(0);
    const int64_t s1 = stride(1);
    if (array.shape(0) == 3 && array.shape(1) == 1) {
      Eigen::Vector3d out;
      for (Eigen::Index i = 0; i < 3; ++i)
        out[i] = base[i * s0 + 0 * s1];
      return out;
    }
    if (array.shape(0) == 1 && array.shape(1) == 3) {
      Eigen::Vector3d out;
      for (Eigen::Index i = 0; i < 3; ++i)
        out[i] = base[0 * s0 + i * s1];
      return out;
    }
  }

  throw nanobind::type_error("Expected a 3D vector");
}

inline Eigen::Matrix3d matrix3FromNumpy(const nanobind::ndarray<double>& array)
{
  if (array.ndim() != 2 || array.shape(0) != 3 || array.shape(1) != 3)
    throw nanobind::type_error("Expected a 3x3 matrix");

  Eigen::Matrix3d out;
  const double* base = array.data();
  const auto stride = [&](size_t axis) -> int64_t {
    if (array.stride_ptr())
      return array.stride(axis);
    int64_t s = 1;
    for (size_t i = array.ndim(); i-- > axis + 1;)
      s *= array.shape(i);
    return s;
  };
  const int64_t s0 = stride(0);
  const int64_t s1 = stride(1);
  for (Eigen::Index r = 0; r < 3; ++r)
    for (Eigen::Index c = 0; c < 3; ++c)
      out(r, c) = base[r * s0 + c * s1];
  return out;
}

inline Eigen::MatrixXd matrixFromArray(
    const std::vector<std::vector<double>>& rows)
{
  if (rows.empty())
    return Eigen::MatrixXd();
  const std::size_t cols = rows.front().size();
  Eigen::MatrixXd out(rows.size(), cols);
  for (std::size_t r = 0; r < rows.size(); ++r) {
    if (rows[r].size() != cols)
      throw nanobind::type_error("Matrix rows must have consistent length");
    for (std::size_t c = 0; c < cols; ++c)
      out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c))
          = rows[r][c];
  }
  return out;
}

} // namespace detail

inline Eigen::Vector3d toVector3(const nanobind::handle& value)
{
  if (nanobind::isinstance<nanobind::ndarray<double>>(value))
    return detail::vector3FromNumpy(
        nanobind::cast<nanobind::ndarray<double>>(value));
  std::array<double, 3> data = nanobind::cast<std::array<double, 3>>(value);
  return detail::vector3FromArray(data);
}

inline Eigen::VectorXd toVector(const nanobind::handle& value)
{
  if (nanobind::isinstance<nanobind::ndarray<double>>(value)) {
    nanobind::ndarray<double> array
        = nanobind::cast<nanobind::ndarray<double>>(value);
    auto stride = [&](size_t axis) -> int64_t {
      if (array.stride_ptr())
        return array.stride(axis);
      int64_t s = 1;
      for (size_t i = array.ndim(); i-- > axis + 1;)
        s *= array.shape(i);
      return s;
    };
    if (array.ndim() == 1) {
      Eigen::VectorXd out(array.shape(0));
      const double* base = array.data();
      const int64_t s0 = stride(0);
      for (Eigen::Index i = 0; i < out.size(); ++i)
        out[i] = base[i * s0];
      return out;
    }
    if (array.ndim() == 2 && array.shape(1) == 1) {
      Eigen::VectorXd out(array.shape(0));
      const double* base = array.data();
      const int64_t s0 = stride(0);
      const int64_t s1 = stride(1);
      for (Eigen::Index i = 0; i < out.size(); ++i)
        out[i] = base[i * s0 + 0 * s1];
      return out;
    }
  }

  std::vector<double> data = nanobind::cast<std::vector<double>>(value);
  Eigen::VectorXd out(data.size());
  for (Eigen::Index i = 0; i < out.size(); ++i)
    out[i] = data[static_cast<std::size_t>(i)];
  return out;
}

inline Eigen::Matrix3d toMatrix3(const nanobind::handle& value)
{
  if (nanobind::isinstance<nanobind::ndarray<double>>(value))
    return detail::matrix3FromNumpy(
        nanobind::cast<nanobind::ndarray<double>>(value));

  const auto nested
      = nanobind::cast<std::array<std::array<double, 3>, 3>>(value);
  Eigen::Matrix3d out;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      out(r, c) = nested[r][c];
  return out;
}

} // namespace dart::python_nb
