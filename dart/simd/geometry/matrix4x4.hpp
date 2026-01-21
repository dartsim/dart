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
#include <dart/simd/geometry/vector4.hpp>

#include <Eigen/Core>

namespace dart::simd {

/// @brief SIMD-backed 4x4 matrix (column-major) for homogeneous transforms
///
/// Four columns of Vec<T, 4> provide perfect SIMD utilization.
/// Column-major storage matches Eigen's default.
///
/// @tparam T Scalar type (float or double)
template <typename T>
struct Matrix4x4
{
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Matrix4x4 only supports float or double");

  Vec<T, 4> col0, col1, col2, col3;

  Matrix4x4() = default;

  DART_SIMD_INLINE Matrix4x4(
      const Vec<T, 4>& c0,
      const Vec<T, 4>& c1,
      const Vec<T, 4>& c2,
      const Vec<T, 4>& c3)
    : col0(c0), col1(c1), col2(c2), col3(c3)
  {
  }

  DART_SIMD_INLINE explicit Matrix4x4(const Eigen::Matrix<T, 4, 4>& m)
    : col0(Vec<T, 4>::set(m(0, 0), m(1, 0), m(2, 0), m(3, 0))),
      col1(Vec<T, 4>::set(m(0, 1), m(1, 1), m(2, 1), m(3, 1))),
      col2(Vec<T, 4>::set(m(0, 2), m(1, 2), m(2, 2), m(3, 2))),
      col3(Vec<T, 4>::set(m(0, 3), m(1, 3), m(2, 3), m(3, 3)))
  {
  }

  DART_SIMD_INLINE static Matrix4x4 identity()
  {
    return Matrix4x4(
        Vec<T, 4>::set(T(1), T(0), T(0), T(0)),
        Vec<T, 4>::set(T(0), T(1), T(0), T(0)),
        Vec<T, 4>::set(T(0), T(0), T(1), T(0)),
        Vec<T, 4>::set(T(0), T(0), T(0), T(1)));
  }

  DART_SIMD_INLINE static Matrix4x4 zero()
  {
    return Matrix4x4(
        Vec<T, 4>::zero(),
        Vec<T, 4>::zero(),
        Vec<T, 4>::zero(),
        Vec<T, 4>::zero());
  }

  [[nodiscard]] DART_SIMD_INLINE T
  operator()(std::size_t row, std::size_t col) const
  {
    switch (col) {
      case 0:
        return col0[row];
      case 1:
        return col1[row];
      case 2:
        return col2[row];
      case 3:
        return col3[row];
      default:
        return T(0);
    }
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix4x4 operator+(const Matrix4x4& rhs) const
  {
    return Matrix4x4(
        col0 + rhs.col0, col1 + rhs.col1, col2 + rhs.col2, col3 + rhs.col3);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix4x4 operator-(const Matrix4x4& rhs) const
  {
    return Matrix4x4(
        col0 - rhs.col0, col1 - rhs.col1, col2 - rhs.col2, col3 - rhs.col3);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix4x4 operator*(T s) const
  {
    Vec<T, 4> sv = Vec<T, 4>::broadcast(s);
    return Matrix4x4(col0 * sv, col1 * sv, col2 * sv, col3 * sv);
  }

  [[nodiscard]] DART_SIMD_INLINE friend Matrix4x4 operator*(
      T s, const Matrix4x4& m)
  {
    return m * s;
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4<T> operator*(const Vector4<T>& v) const
  {
    Vec<T, 4> result = col0 * Vec<T, 4>::broadcast(v.x())
                       + col1 * Vec<T, 4>::broadcast(v.y())
                       + col2 * Vec<T, 4>::broadcast(v.z())
                       + col3 * Vec<T, 4>::broadcast(v.w());
    return Vector4<T>(result);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix4x4 operator*(const Matrix4x4& rhs) const
  {
    return Matrix4x4(
        ((*this) * Vector4<T>(rhs.col0)).data,
        ((*this) * Vector4<T>(rhs.col1)).data,
        ((*this) * Vector4<T>(rhs.col2)).data,
        ((*this) * Vector4<T>(rhs.col3)).data);
  }

  [[nodiscard]] DART_SIMD_INLINE Vector3<T> transformPoint(
      const Vector3<T>& p) const
  {
    Vec<T, 4> result = col0 * Vec<T, 4>::broadcast(p.x())
                       + col1 * Vec<T, 4>::broadcast(p.y())
                       + col2 * Vec<T, 4>::broadcast(p.z()) + col3;
    T w = result[3];
    if (w != T(0) && w != T(1)) {
      result = result / Vec<T, 4>::broadcast(w);
    }
    return Vector3<T>(result);
  }

  [[nodiscard]] DART_SIMD_INLINE Vector3<T> transformVector(
      const Vector3<T>& v) const
  {
    Vec<T, 4> result = col0 * Vec<T, 4>::broadcast(v.x())
                       + col1 * Vec<T, 4>::broadcast(v.y())
                       + col2 * Vec<T, 4>::broadcast(v.z());
    return Vector3<T>(result);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix4x4 transposed() const
  {
    return Matrix4x4(
        Vec<T, 4>::set(col0[0], col1[0], col2[0], col3[0]),
        Vec<T, 4>::set(col0[1], col1[1], col2[1], col3[1]),
        Vec<T, 4>::set(col0[2], col1[2], col2[2], col3[2]),
        Vec<T, 4>::set(col0[3], col1[3], col2[3], col3[3]));
  }

  [[nodiscard]] DART_SIMD_INLINE Eigen::Matrix<T, 4, 4> toEigen() const
  {
    Eigen::Matrix<T, 4, 4> m;
    m << col0[0], col1[0], col2[0], col3[0], col0[1], col1[1], col2[1], col3[1],
        col0[2], col1[2], col2[2], col3[2], col0[3], col1[3], col2[3], col3[3];
    return m;
  }

  [[nodiscard]] DART_SIMD_INLINE static Matrix4x4 fromEigen(
      const Eigen::Matrix<T, 4, 4>& m)
  {
    return Matrix4x4(m);
  }
};

using Matrix4x4f = Matrix4x4<float>;
using Matrix4x4d = Matrix4x4<double>;

} // namespace dart::simd
