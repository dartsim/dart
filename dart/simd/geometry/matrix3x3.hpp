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

/// @brief SIMD-backed 3x3 matrix (column-major) for rotations and transforms
///
/// Each column is stored as Vec<T, 4> with padding in the 4th element.
/// Column-major storage matches Eigen's default and is optimal for mat-vec.
///
/// @tparam T Scalar type (float or double)
template <typename T>
struct Matrix3x3
{
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Matrix3x3 only supports float or double");

  Vec<T, 4> col0, col1, col2;

  Matrix3x3() = default;

  DART_SIMD_INLINE Matrix3x3(
      const Vec<T, 4>& c0, const Vec<T, 4>& c1, const Vec<T, 4>& c2)
    : col0(c0), col1(c1), col2(c2)
  {
  }

  DART_SIMD_INLINE Matrix3x3(
      T m00, T m01, T m02, T m10, T m11, T m12, T m20, T m21, T m22)
    : col0(Vec<T, 4>::set(m00, m10, m20, T(0))),
      col1(Vec<T, 4>::set(m01, m11, m21, T(0))),
      col2(Vec<T, 4>::set(m02, m12, m22, T(0)))
  {
  }

  DART_SIMD_INLINE explicit Matrix3x3(const Eigen::Matrix<T, 3, 3>& m)
    : col0(Vec<T, 4>::set(m(0, 0), m(1, 0), m(2, 0), T(0))),
      col1(Vec<T, 4>::set(m(0, 1), m(1, 1), m(2, 1), T(0))),
      col2(Vec<T, 4>::set(m(0, 2), m(1, 2), m(2, 2), T(0)))
  {
  }

  DART_SIMD_INLINE static Matrix3x3 identity()
  {
    return Matrix3x3(
        Vec<T, 4>::set(T(1), T(0), T(0), T(0)),
        Vec<T, 4>::set(T(0), T(1), T(0), T(0)),
        Vec<T, 4>::set(T(0), T(0), T(1), T(0)));
  }

  DART_SIMD_INLINE static Matrix3x3 zero()
  {
    return Matrix3x3(Vec<T, 4>::zero(), Vec<T, 4>::zero(), Vec<T, 4>::zero());
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
      default:
        return T(0);
    }
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix3x3 operator+(const Matrix3x3& rhs) const
  {
    return Matrix3x3(col0 + rhs.col0, col1 + rhs.col1, col2 + rhs.col2);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix3x3 operator-(const Matrix3x3& rhs) const
  {
    return Matrix3x3(col0 - rhs.col0, col1 - rhs.col1, col2 - rhs.col2);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix3x3 operator*(T s) const
  {
    Vec<T, 4> sv = Vec<T, 4>::broadcast(s);
    return Matrix3x3(col0 * sv, col1 * sv, col2 * sv);
  }

  [[nodiscard]] DART_SIMD_INLINE friend Matrix3x3 operator*(
      T s, const Matrix3x3& m)
  {
    return m * s;
  }

  [[nodiscard]] DART_SIMD_INLINE Vector3<T> operator*(const Vector3<T>& v) const
  {
    Vec<T, 4> result = col0 * Vec<T, 4>::broadcast(v.x())
                       + col1 * Vec<T, 4>::broadcast(v.y())
                       + col2 * Vec<T, 4>::broadcast(v.z());
    return Vector3<T>(result);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix3x3 operator*(const Matrix3x3& rhs) const
  {
    return Matrix3x3(
        ((*this) * Vector3<T>(rhs.col0)).data,
        ((*this) * Vector3<T>(rhs.col1)).data,
        ((*this) * Vector3<T>(rhs.col2)).data);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix3x3 transposed() const
  {
    return Matrix3x3(
        col0[0],
        col0[1],
        col0[2],
        col1[0],
        col1[1],
        col1[2],
        col2[0],
        col2[1],
        col2[2]);
  }

  [[nodiscard]] DART_SIMD_INLINE T determinant() const
  {
    T a = col0[0], b = col1[0], c = col2[0];
    T d = col0[1], e = col1[1], f = col2[1];
    T g = col0[2], h = col1[2], i = col2[2];
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  }

  [[nodiscard]] DART_SIMD_INLINE T trace() const
  {
    return col0[0] + col1[1] + col2[2];
  }

  [[nodiscard]] DART_SIMD_INLINE T frobeniusNorm() const
  {
    T sum = T(0);
    for (int i = 0; i < 3; ++i) {
      sum += col0[i] * col0[i] + col1[i] * col1[i] + col2[i] * col2[i];
    }
    return std::sqrt(sum);
  }

  [[nodiscard]] DART_SIMD_INLINE T squaredFrobeniusNorm() const
  {
    T sum = T(0);
    for (int i = 0; i < 3; ++i) {
      sum += col0[i] * col0[i] + col1[i] * col1[i] + col2[i] * col2[i];
    }
    return sum;
  }

  [[nodiscard]] DART_SIMD_INLINE Vector3<T> diagonal() const
  {
    return Vector3<T>(col0[0], col1[1], col2[2]);
  }

  [[nodiscard]] DART_SIMD_INLINE bool isSymmetric(T tol = T(1e-6)) const
  {
    return std::abs(col1[0] - col0[1]) < tol
           && std::abs(col2[0] - col0[2]) < tol
           && std::abs(col2[1] - col1[2]) < tol;
  }

  [[nodiscard]] DART_SIMD_INLINE bool isOrthogonal(T tol = T(1e-6)) const
  {
    Matrix3x3 product = (*this) * transposed();
    Matrix3x3 I = Matrix3x3::identity();
    return std::abs(product(0, 0) - I(0, 0)) < tol
           && std::abs(product(0, 1) - I(0, 1)) < tol
           && std::abs(product(0, 2) - I(0, 2)) < tol
           && std::abs(product(1, 0) - I(1, 0)) < tol
           && std::abs(product(1, 1) - I(1, 1)) < tol
           && std::abs(product(1, 2) - I(1, 2)) < tol
           && std::abs(product(2, 0) - I(2, 0)) < tol
           && std::abs(product(2, 1) - I(2, 1)) < tol
           && std::abs(product(2, 2) - I(2, 2)) < tol;
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix3x3 inverse() const
  {
    T a = col0[0], b = col1[0], c = col2[0];
    T d = col0[1], e = col1[1], f = col2[1];
    T g = col0[2], h = col1[2], i = col2[2];

    T det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    T invDet = T(1) / det;

    return Matrix3x3(
        (e * i - f * h) * invDet,
        (c * h - b * i) * invDet,
        (b * f - c * e) * invDet,
        (f * g - d * i) * invDet,
        (a * i - c * g) * invDet,
        (c * d - a * f) * invDet,
        (d * h - e * g) * invDet,
        (b * g - a * h) * invDet,
        (a * e - b * d) * invDet);
  }

  [[nodiscard]] DART_SIMD_INLINE bool tryInverse(Matrix3x3& out) const
  {
    T a = col0[0], b = col1[0], c = col2[0];
    T d = col0[1], e = col1[1], f = col2[1];
    T g = col0[2], h = col1[2], i = col2[2];

    T det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

    constexpr T eps = std::is_same_v<T, float> ? T(1e-6) : T(1e-12);
    if (std::abs(det) < eps) {
      return false;
    }

    T invDet = T(1) / det;
    out = Matrix3x3(
        (e * i - f * h) * invDet,
        (c * h - b * i) * invDet,
        (b * f - c * e) * invDet,
        (f * g - d * i) * invDet,
        (a * i - c * g) * invDet,
        (c * d - a * f) * invDet,
        (d * h - e * g) * invDet,
        (b * g - a * h) * invDet,
        (a * e - b * d) * invDet);
    return true;
  }

  [[nodiscard]] DART_SIMD_INLINE Eigen::Matrix<T, 3, 3> toEigen() const
  {
    Eigen::Matrix<T, 3, 3> m;
    m << col0[0], col1[0], col2[0], col0[1], col1[1], col2[1], col0[2], col1[2],
        col2[2];
    return m;
  }

  [[nodiscard]] DART_SIMD_INLINE static Matrix3x3 fromEigen(
      const Eigen::Matrix<T, 3, 3>& m)
  {
    return Matrix3x3(m);
  }
};

/// Outer product: produces a 3x3 matrix from two vectors
/// result[i][j] = a[i] * b[j]
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Matrix3x3<T> outer(
    const Vector3<T>& a, const Vector3<T>& b)
{
  return Matrix3x3<T>(
      Vec<T, 4>::set(a.x() * b.x(), a.y() * b.x(), a.z() * b.x(), T(0)),
      Vec<T, 4>::set(a.x() * b.y(), a.y() * b.y(), a.z() * b.y(), T(0)),
      Vec<T, 4>::set(a.x() * b.z(), a.y() * b.z(), a.z() * b.z(), T(0)));
}

using Matrix3x3f = Matrix3x3<float>;
using Matrix3x3d = Matrix3x3<double>;

} // namespace dart::simd
