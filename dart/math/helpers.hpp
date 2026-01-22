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

#ifndef DART_MATH_HELPERS_HPP_
#define DART_MATH_HELPERS_HPP_

// Standard Libraries
#include <array>
#include <bit>
#include <concepts>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <type_traits>

#include <cfloat>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <ctime>

// External Libraries
#include <Eigen/Dense>
// Local Headers
#include <dart/math/constants.hpp>
#include <dart/math/math_types.hpp>
#include <dart/math/random.hpp>

namespace dart {
namespace math {

//==============================================================================
template <typename T>
constexpr T toRadian(const T& degree)
{
  return degree * pi_v<T> / 180.0;
}

//==============================================================================
template <typename T>
constexpr T toDegree(const T& radian)
{
  return radian * 180.0 / pi_v<T>;
}

/// @brief a cross b = (CR*a) dot b
/// const Matd CR(2,2,0.0,-1.0,1.0,0.0);
const Eigen::Matrix2d CR((Eigen::Matrix2d() << 0.0, -1.0, 1.0, 0.0).finished());

inline int delta(int _i, int _j)
{
  if (_i == _j) {
    return 1;
  }
  return 0;
}

template <typename T>
inline constexpr int sign(T x, std::false_type)
{
  return static_cast<T>(0) < x;
}

template <typename T>
inline constexpr int sign(T x, std::true_type)
{
  return (static_cast<T>(0) < x) - (x < static_cast<T>(0));
}

template <typename T>
inline constexpr int sign(T x)
{
  return sign(x, std::is_signed<T>());
}

inline double sqr(double _x)
{
  return _x * _x;
}

inline double Tsinc(double _theta)
{
  return 0.5 - sqrt(_theta) / 48;
}

inline double asinh(double _X)
{
  return log(_X + sqrt(_X * _X + 1));
}

inline double acosh(double _X)
{
  return log(_X + sqrt(_X * _X - 1));
}

inline double atanh(double _X)
{
  return log((1 + _X) / (1 - _X)) / 2;
}

inline double asech(double _X)
{
  return log((sqrt(-_X * _X + 1) + 1) / _X);
}

inline double acosech(double _X)
{
  return log((sign(_X) * sqrt(_X * _X + 1) + 1) / _X);
}

inline double acotanh(double _X)
{
  return log((_X + 1) / (_X - 1)) / 2;
}

inline double round2(double _x)
{
  int gintx = static_cast<int>(std::floor(_x));
  if (_x - gintx < 0.5) {
    return static_cast<double>(gintx);
  } else {
    return static_cast<double>(gintx + 1.0);
  }
}

template <typename T>
inline T clip(const T& val, const T& lower, const T& upper)
{
  return std::max(lower, std::min(val, upper));
}

template <typename DerivedA, typename DerivedB>
inline typename DerivedA::PlainObject clip(
    const Eigen::MatrixBase<DerivedA>& val,
    const Eigen::MatrixBase<DerivedB>& lower,
    const Eigen::MatrixBase<DerivedB>& upper)
{
  return lower.cwiseMax(val.cwiseMin(upper));
}

template <typename T>
constexpr T defaultAbsTolerance()
{
  return static_cast<T>(1e-12);
}

template <typename T>
constexpr T defaultRelTolerance()
{
  return static_cast<T>(1e-12);
}

namespace detail {

template <typename Float>
using FloatByteArray = std::array<std::byte, sizeof(Float)>;

template <typename Float>
inline bool valueEqualFloating(Float lhs, Float rhs)
{
  static_assert(
      std::is_floating_point_v<Float>,
      "valueEqualFloating expects floating point");

  if (std::isnan(lhs) || std::isnan(rhs)) {
    return false;
  }

  if (std::fpclassify(lhs) == FP_ZERO && std::fpclassify(rhs) == FP_ZERO) {
    return true;
  }

  return std::bit_cast<FloatByteArray<Float>>(lhs)
         == std::bit_cast<FloatByteArray<Float>>(rhs);
}

} // namespace detail

template <typename T>
  requires std::floating_point<T>
inline bool valueEqual(const T& lhs, const T& rhs)
{
  return detail::valueEqualFloating(lhs, rhs);
}

template <typename T>
  requires std::integral<T>
inline bool valueEqual(const T& lhs, const T& rhs)
{
  return lhs == rhs;
}

template <typename DerivedA, typename DerivedB>
inline bool valueEqual(
    const Eigen::MatrixBase<DerivedA>& lhs,
    const Eigen::MatrixBase<DerivedB>& rhs)
{
  if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) {
    return false;
  }

  for (Eigen::Index r = 0; r < lhs.rows(); ++r) {
    for (Eigen::Index c = 0; c < lhs.cols(); ++c) {
      if (!valueEqual(lhs(r, c), rhs(r, c))) {
        return false;
      }
    }
  }

  return true;
}

template <typename T>
inline bool isEqual(const T& lhs, const T& rhs)
{
  return valueEqual(lhs, rhs);
}

template <typename T>
  requires std::floating_point<T>
inline bool isApprox(
    const T& lhs,
    const T& rhs,
    const T& abs_tol = defaultAbsTolerance<T>(),
    const T& rel_tol = defaultRelTolerance<T>())
{
  if (std::isnan(lhs) || std::isnan(rhs)) {
    return false;
  }

  if (std::isinf(lhs) || std::isinf(rhs)) {
    return valueEqual(lhs, rhs);
  }

  const T diff = std::abs(lhs - rhs);
  const T scale = std::max(std::abs(lhs), std::abs(rhs));
  return diff <= std::max(abs_tol, rel_tol * scale);
}

template <typename T>
  requires std::integral<T>
inline bool isApprox(
    const T& lhs, const T& rhs, double abs_tol = 0.0, double rel_tol = 0.0)
{
  return isApprox(
      static_cast<double>(lhs), static_cast<double>(rhs), abs_tol, rel_tol);
}

template <typename DerivedA, typename DerivedB>
  requires std::floating_point<typename DerivedA::Scalar>
           && std::floating_point<typename DerivedB::Scalar>
inline bool isApprox(
    const Eigen::MatrixBase<DerivedA>& lhs,
    const Eigen::MatrixBase<DerivedB>& rhs,
    const typename DerivedA::Scalar abs_tol
    = defaultAbsTolerance<typename DerivedA::Scalar>(),
    const typename DerivedA::Scalar rel_tol
    = defaultRelTolerance<typename DerivedA::Scalar>())
{
  if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) {
    return false;
  }

  for (Eigen::Index r = 0; r < lhs.rows(); ++r) {
    for (Eigen::Index c = 0; c < lhs.cols(); ++c) {
      if (!isApprox(lhs(r, c), rhs(r, c), abs_tol, rel_tol)) {
        return false;
      }
    }
  }

  return true;
}

template <typename DerivedA, typename DerivedB>
  requires(
      std::is_arithmetic_v<typename DerivedA::Scalar>
      && std::is_arithmetic_v<typename DerivedB::Scalar>
      && !(
          std::floating_point<typename DerivedA::Scalar>
          && std::floating_point<typename DerivedB::Scalar>))
inline bool isApprox(
    const Eigen::MatrixBase<DerivedA>& lhs,
    const Eigen::MatrixBase<DerivedB>& rhs,
    const std::common_type_t<
        typename DerivedA::Scalar,
        typename DerivedB::Scalar,
        double> abs_tol
    = std::common_type_t<
        typename DerivedA::Scalar,
        typename DerivedB::Scalar,
        double>{0},
    const std::common_type_t<
        typename DerivedA::Scalar,
        typename DerivedB::Scalar,
        double> rel_tol
    = std::common_type_t<
        typename DerivedA::Scalar,
        typename DerivedB::Scalar,
        double>{0})
{
  using Common = std::common_type_t<
      typename DerivedA::Scalar,
      typename DerivedB::Scalar,
      double>;
  return isApprox(
      lhs.template cast<Common>(),
      rhs.template cast<Common>(),
      static_cast<Common>(abs_tol),
      static_cast<Common>(rel_tol));
}

template <typename T>
  requires std::floating_point<T>
inline bool isZero(const T& value, const T& abs_tol = defaultAbsTolerance<T>())
{
  if (std::isnan(value)) {
    return false;
  }

  return std::abs(value) <= abs_tol;
}

template <typename T>
  requires std::integral<T>
inline bool isZero(const T& value)
{
  return value == T{0};
}

template <typename Derived>
  requires std::floating_point<typename Derived::Scalar>
inline bool isZero(
    const Eigen::MatrixBase<Derived>& values,
    const typename Derived::Scalar abs_tol
    = defaultAbsTolerance<typename Derived::Scalar>())
{
  if (values.size() == 0) {
    return true;
  }

  return values.cwiseAbs().maxCoeff() <= abs_tol;
}

// check if it is an integer
inline bool isInt(double _x)
{
  return isApprox(round(_x), _x, static_cast<double>(1e-6), 0.0);
}

/// @brief Returns whether _v is a NaN (Not-A-Number) value
inline bool isNan(double _v)
{
#ifdef _WIN32
  return _isnan(_v) != 0;
#else
  return std::isnan(_v);
#endif
}

/// @brief Returns whether _m is a NaN (Not-A-Number) matrix
inline bool isNan(const Eigen::MatrixXd& _m)
{
  for (int i = 0; i < _m.rows(); ++i) {
    for (int j = 0; j < _m.cols(); ++j) {
      if (isNan(_m(i, j))) {
        return true;
      }
    }
  }

  return false;
}

/// @brief Returns whether _v is an infinity value (either positive infinity or
/// negative infinity).
inline bool isInf(double _v)
{
#ifdef _WIN32
  return !_finite(_v);
#else
  return std::isinf(_v);
#endif
}

/// @brief Returns whether _m is an infinity matrix (either positive infinity or
/// negative infinity).
inline bool isInf(const Eigen::MatrixXd& _m)
{
  for (int i = 0; i < _m.rows(); ++i) {
    for (int j = 0; j < _m.cols(); ++j) {
      if (isInf(_m(i, j))) {
        return true;
      }
    }
  }

  return false;
}

/// @brief Returns whether _m is symmetric or not
inline bool isSymmetric(const Eigen::MatrixXd& _m, double _tol = 1e-6)
{
  std::size_t rows = _m.rows();
  std::size_t cols = _m.cols();

  if (rows != cols) {
    return false;
  }

  for (std::size_t i = 0; i < rows; ++i) {
    for (std::size_t j = i + 1; j < cols; ++j) {
      if (std::abs(_m(i, j) - _m(j, i)) > _tol) {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < rows; ++k) {
          for (std::size_t l = 0; l < cols; ++l) {
            std::cout << std::setprecision(4) << _m(k, l) << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << _m(i, j) << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << _m(i, j) << std::endl;
        return false;
      }
    }
  }

  return true;
}

namespace suffixes {

//==============================================================================
constexpr double operator"" _pi(long double x)
{
  return x * pi_v<double>;
}

//==============================================================================
constexpr double operator"" _pi(unsigned long long int x)
{
  return operator"" _pi(static_cast<long double>(x));
}

//==============================================================================
constexpr double operator"" _rad(long double angle)
{
  return angle;
}

//==============================================================================
constexpr double operator"" _rad(unsigned long long int angle)
{
  return operator"" _rad(static_cast<long double>(angle));
}

//==============================================================================
constexpr double operator"" _deg(long double angle)
{
  return toRadian(angle);
}

//==============================================================================
constexpr double operator"" _deg(unsigned long long int angle)
{
  return operator"" _deg(static_cast<long double>(angle));
}

} // namespace suffixes

} // namespace math

namespace Color {

inline Eigen::Vector4d Red(double alpha)
{
  return Eigen::Vector4d(0.9, 0.1, 0.1, alpha);
}

inline Eigen::Vector3d Red()
{
  return Eigen::Vector3d(0.9, 0.1, 0.1);
}

inline Eigen::Vector3d Fuchsia()
{
  return Eigen::Vector3d(1.0, 0.0, 0.5);
}

inline Eigen::Vector4d Fuchsia(double alpha)
{
  return Eigen::Vector4d(1.0, 0.0, 0.5, alpha);
}

inline Eigen::Vector4d Orange(double alpha)
{
  return Eigen::Vector4d(1.0, 0.63, 0.0, alpha);
}

inline Eigen::Vector3d Orange()
{
  return Eigen::Vector3d(1.0, 0.63, 0.0);
}

inline Eigen::Vector4d Green(double alpha)
{
  return Eigen::Vector4d(0.1, 0.9, 0.1, alpha);
}

inline Eigen::Vector3d Green()
{
  return Eigen::Vector3d(0.1, 0.9, 0.1);
}

inline Eigen::Vector4d Blue(double alpha)
{
  return Eigen::Vector4d(0.1, 0.1, 0.9, alpha);
}

inline Eigen::Vector3d Blue()
{
  return Eigen::Vector3d(0.1, 0.1, 0.9);
}

inline Eigen::Vector4d White(double alpha)
{
  return Eigen::Vector4d(1.0, 1.0, 1.0, alpha);
}

inline Eigen::Vector3d White()
{
  return Eigen::Vector3d(1.0, 1.0, 1.0);
}

inline Eigen::Vector4d Black(double alpha)
{
  return Eigen::Vector4d(0.05, 0.05, 0.05, alpha);
}

inline Eigen::Vector3d Black()
{
  return Eigen::Vector3d(0.05, 0.05, 0.05);
}

inline Eigen::Vector4d LightGray(double alpha)
{
  return Eigen::Vector4d(0.9, 0.9, 0.9, alpha);
}

inline Eigen::Vector3d LightGray()
{
  return Eigen::Vector3d(0.9, 0.9, 0.9);
}

inline Eigen::Vector4d Gray(double alpha)
{
  return Eigen::Vector4d(0.6, 0.6, 0.6, alpha);
}

inline Eigen::Vector3d Gray()
{
  return Eigen::Vector3d(0.6, 0.6, 0.6);
}

inline Eigen::Vector4d Random(double alpha)
{
  return Eigen::Vector4d(
      math::Random::uniform(0.0, 1.0),
      math::Random::uniform(0.0, 1.0),
      math::Random::uniform(0.0, 1.0),
      alpha);
}

inline Eigen::Vector3d Random()
{
  return Eigen::Vector3d(
      math::Random::uniform(0.0, 1.0),
      math::Random::uniform(0.0, 1.0),
      math::Random::uniform(0.0, 1.0));
}

} // namespace Color

} // namespace dart

#endif // DART_MATH_HELPERS_HPP_
