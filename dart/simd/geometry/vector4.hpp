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

#pragma once

#include <dart/simd/geometry/vector3.hpp>

#include <Eigen/Core>

#include <cmath>

namespace dart::simd {

/// @brief SIMD-backed 4D vector using native Vec<T, 4> storage
/// @tparam T Scalar type (float or double)
template <typename T>
struct Vector4
{
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Vector4 only supports float or double");

  Vec<T, 4> data;

  Vector4() = default;

  DART_SIMD_INLINE Vector4(T x, T y, T z, T w)
    : data(Vec<T, 4>::set(x, y, z, w))
  {
  }

  DART_SIMD_INLINE explicit Vector4(const Vec<T, 4>& v) : data(v) {}

  DART_SIMD_INLINE explicit Vector4(const Eigen::Matrix<T, 4, 1>& v)
    : data(Vec<T, 4>::set(v.x(), v.y(), v.z(), v.w()))
  {
  }

  DART_SIMD_INLINE static Vector4 broadcast(T s)
  {
    return Vector4(Vec<T, 4>::broadcast(s));
  }

  DART_SIMD_INLINE static Vector4 zero()
  {
    return Vector4(Vec<T, 4>::zero());
  }

  [[nodiscard]] DART_SIMD_INLINE T x() const
  {
    return data[0];
  }
  [[nodiscard]] DART_SIMD_INLINE T y() const
  {
    return data[1];
  }
  [[nodiscard]] DART_SIMD_INLINE T z() const
  {
    return data[2];
  }
  [[nodiscard]] DART_SIMD_INLINE T w() const
  {
    return data[3];
  }

  [[nodiscard]] DART_SIMD_INLINE T operator[](std::size_t i) const
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4 operator+(const Vector4& rhs) const
  {
    return Vector4(data + rhs.data);
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4 operator-(const Vector4& rhs) const
  {
    return Vector4(data - rhs.data);
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4 operator-() const
  {
    return Vector4(-data);
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4 operator*(const Vector4& rhs) const
  {
    return Vector4(data * rhs.data);
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4 operator*(T s) const
  {
    return Vector4(data * Vec<T, 4>::broadcast(s));
  }

  [[nodiscard]] DART_SIMD_INLINE friend Vector4 operator*(T s, const Vector4& v)
  {
    return v * s;
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4 operator/(T s) const
  {
    return Vector4(data / Vec<T, 4>::broadcast(s));
  }

  DART_SIMD_INLINE Vector4& operator+=(const Vector4& rhs)
  {
    data = data + rhs.data;
    return *this;
  }

  DART_SIMD_INLINE Vector4& operator-=(const Vector4& rhs)
  {
    data = data - rhs.data;
    return *this;
  }

  DART_SIMD_INLINE Vector4& operator*=(T s)
  {
    data = data * Vec<T, 4>::broadcast(s);
    return *this;
  }

  DART_SIMD_INLINE Vector4& operator/=(T s)
  {
    data = data / Vec<T, 4>::broadcast(s);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const Vector4& rhs) const
  {
    return x() == rhs.x() && y() == rhs.y() && z() == rhs.z() && w() == rhs.w();
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const Vector4& rhs) const
  {
    return !(*this == rhs);
  }

  [[nodiscard]] DART_SIMD_INLINE T squaredNorm() const
  {
    return hsum(data * data);
  }

  [[nodiscard]] DART_SIMD_INLINE T norm() const
  {
    return std::sqrt(squaredNorm());
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4 normalized() const
  {
    T len = norm();
    return (len > T(0)) ? (*this / len) : zero();
  }

  DART_SIMD_INLINE void normalize()
  {
    *this = normalized();
  }

  [[nodiscard]] DART_SIMD_INLINE Eigen::Matrix<T, 4, 1> toEigen() const
  {
    return Eigen::Matrix<T, 4, 1>(x(), y(), z(), w());
  }

  [[nodiscard]] DART_SIMD_INLINE static Vector4 fromEigen(
      const Eigen::Matrix<T, 4, 1>& v)
  {
    return Vector4(v);
  }
};

template <typename T>
[[nodiscard]] DART_SIMD_INLINE T dot(const Vector4<T>& a, const Vector4<T>& b)
{
  return hsum(a.data * b.data);
}

template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector4<T> min(
    const Vector4<T>& a, const Vector4<T>& b)
{
  return Vector4<T>(min(a.data, b.data));
}

template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector4<T> max(
    const Vector4<T>& a, const Vector4<T>& b)
{
  return Vector4<T>(max(a.data, b.data));
}

template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector4<T> abs(const Vector4<T>& v)
{
  return Vector4<T>(abs(v.data));
}

template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector4<T> lerp(
    const Vector4<T>& a, const Vector4<T>& b, T t)
{
  return a + (b - a) * t;
}

using Vector4f = Vector4<float>;
using Vector4d = Vector4<double>;

} // namespace dart::simd
