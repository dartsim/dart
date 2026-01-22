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

#include <cmath>

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

  [[nodiscard]] DART_SIMD_INLINE T determinant() const
  {
    T m00 = col0[0], m10 = col0[1], m20 = col0[2], m30 = col0[3];
    T m01 = col1[0], m11 = col1[1], m21 = col1[2], m31 = col1[3];
    T m02 = col2[0], m12 = col2[1], m22 = col2[2], m32 = col2[3];
    T m03 = col3[0], m13 = col3[1], m23 = col3[2], m33 = col3[3];

    T s0 = m00 * m11 - m10 * m01;
    T s1 = m00 * m12 - m10 * m02;
    T s2 = m00 * m13 - m10 * m03;
    T s3 = m01 * m12 - m11 * m02;
    T s4 = m01 * m13 - m11 * m03;
    T s5 = m02 * m13 - m12 * m03;

    T c5 = m22 * m33 - m32 * m23;
    T c4 = m21 * m33 - m31 * m23;
    T c3 = m21 * m32 - m31 * m22;
    T c2 = m20 * m33 - m30 * m23;
    T c1 = m20 * m32 - m30 * m22;
    T c0 = m20 * m31 - m30 * m21;

    return s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
  }

  [[nodiscard]] DART_SIMD_INLINE T trace() const
  {
    return col0[0] + col1[1] + col2[2] + col3[3];
  }

  [[nodiscard]] DART_SIMD_INLINE T frobeniusNorm() const
  {
    T sum = T(0);
    for (int i = 0; i < 4; ++i) {
      sum += col0[i] * col0[i] + col1[i] * col1[i] + col2[i] * col2[i]
             + col3[i] * col3[i];
    }
    return std::sqrt(sum);
  }

  [[nodiscard]] DART_SIMD_INLINE T squaredFrobeniusNorm() const
  {
    T sum = T(0);
    for (int i = 0; i < 4; ++i) {
      sum += col0[i] * col0[i] + col1[i] * col1[i] + col2[i] * col2[i]
             + col3[i] * col3[i];
    }
    return sum;
  }

  [[nodiscard]] DART_SIMD_INLINE Vector4<T> diagonal() const
  {
    return Vector4<T>(col0[0], col1[1], col2[2], col3[3]);
  }

  [[nodiscard]] DART_SIMD_INLINE bool isSymmetric(T tol = T(1e-6)) const
  {
    return std::abs(col1[0] - col0[1]) < tol
           && std::abs(col2[0] - col0[2]) < tol
           && std::abs(col3[0] - col0[3]) < tol
           && std::abs(col2[1] - col1[2]) < tol
           && std::abs(col3[1] - col1[3]) < tol
           && std::abs(col3[2] - col2[3]) < tol;
  }

  [[nodiscard]] DART_SIMD_INLINE bool isOrthogonal(T tol = T(1e-6)) const
  {
    Matrix4x4 product = (*this) * transposed();
    Matrix4x4 I = Matrix4x4::identity();
    return std::abs(product(0, 0) - I(0, 0)) < tol
           && std::abs(product(0, 1) - I(0, 1)) < tol
           && std::abs(product(0, 2) - I(0, 2)) < tol
           && std::abs(product(0, 3) - I(0, 3)) < tol
           && std::abs(product(1, 0) - I(1, 0)) < tol
           && std::abs(product(1, 1) - I(1, 1)) < tol
           && std::abs(product(1, 2) - I(1, 2)) < tol
           && std::abs(product(1, 3) - I(1, 3)) < tol
           && std::abs(product(2, 0) - I(2, 0)) < tol
           && std::abs(product(2, 1) - I(2, 1)) < tol
           && std::abs(product(2, 2) - I(2, 2)) < tol
           && std::abs(product(2, 3) - I(2, 3)) < tol
           && std::abs(product(3, 0) - I(3, 0)) < tol
           && std::abs(product(3, 1) - I(3, 1)) < tol
           && std::abs(product(3, 2) - I(3, 2)) < tol
           && std::abs(product(3, 3) - I(3, 3)) < tol;
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix4x4 inverse() const
  {
    T m00 = col0[0], m10 = col0[1], m20 = col0[2], m30 = col0[3];
    T m01 = col1[0], m11 = col1[1], m21 = col1[2], m31 = col1[3];
    T m02 = col2[0], m12 = col2[1], m22 = col2[2], m32 = col2[3];
    T m03 = col3[0], m13 = col3[1], m23 = col3[2], m33 = col3[3];

    T s0 = m00 * m11 - m10 * m01;
    T s1 = m00 * m12 - m10 * m02;
    T s2 = m00 * m13 - m10 * m03;
    T s3 = m01 * m12 - m11 * m02;
    T s4 = m01 * m13 - m11 * m03;
    T s5 = m02 * m13 - m12 * m03;

    T c5 = m22 * m33 - m32 * m23;
    T c4 = m21 * m33 - m31 * m23;
    T c3 = m21 * m32 - m31 * m22;
    T c2 = m20 * m33 - m30 * m23;
    T c1 = m20 * m32 - m30 * m22;
    T c0 = m20 * m31 - m30 * m21;

    T det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    T invDet = T(1) / det;

    return Matrix4x4(
        Vec<T, 4>::set(
            (m11 * c5 - m12 * c4 + m13 * c3) * invDet,
            (-m10 * c5 + m12 * c2 - m13 * c1) * invDet,
            (m10 * c4 - m11 * c2 + m13 * c0) * invDet,
            (-m10 * c3 + m11 * c1 - m12 * c0) * invDet),
        Vec<T, 4>::set(
            (-m01 * c5 + m02 * c4 - m03 * c3) * invDet,
            (m00 * c5 - m02 * c2 + m03 * c1) * invDet,
            (-m00 * c4 + m01 * c2 - m03 * c0) * invDet,
            (m00 * c3 - m01 * c1 + m02 * c0) * invDet),
        Vec<T, 4>::set(
            (m31 * s5 - m32 * s4 + m33 * s3) * invDet,
            (-m30 * s5 + m32 * s2 - m33 * s1) * invDet,
            (m30 * s4 - m31 * s2 + m33 * s0) * invDet,
            (-m30 * s3 + m31 * s1 - m32 * s0) * invDet),
        Vec<T, 4>::set(
            (-m21 * s5 + m22 * s4 - m23 * s3) * invDet,
            (m20 * s5 - m22 * s2 + m23 * s1) * invDet,
            (-m20 * s4 + m21 * s2 - m23 * s0) * invDet,
            (m20 * s3 - m21 * s1 + m22 * s0) * invDet));
  }

  [[nodiscard]] DART_SIMD_INLINE bool tryInverse(Matrix4x4& out) const
  {
    T m00 = col0[0], m10 = col0[1], m20 = col0[2], m30 = col0[3];
    T m01 = col1[0], m11 = col1[1], m21 = col1[2], m31 = col1[3];
    T m02 = col2[0], m12 = col2[1], m22 = col2[2], m32 = col2[3];
    T m03 = col3[0], m13 = col3[1], m23 = col3[2], m33 = col3[3];

    T s0 = m00 * m11 - m10 * m01;
    T s1 = m00 * m12 - m10 * m02;
    T s2 = m00 * m13 - m10 * m03;
    T s3 = m01 * m12 - m11 * m02;
    T s4 = m01 * m13 - m11 * m03;
    T s5 = m02 * m13 - m12 * m03;

    T c5 = m22 * m33 - m32 * m23;
    T c4 = m21 * m33 - m31 * m23;
    T c3 = m21 * m32 - m31 * m22;
    T c2 = m20 * m33 - m30 * m23;
    T c1 = m20 * m32 - m30 * m22;
    T c0 = m20 * m31 - m30 * m21;

    T det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;

    constexpr T eps = std::is_same_v<T, float> ? T(1e-6) : T(1e-12);
    if (std::abs(det) < eps) {
      return false;
    }

    T invDet = T(1) / det;
    out = Matrix4x4(
        Vec<T, 4>::set(
            (m11 * c5 - m12 * c4 + m13 * c3) * invDet,
            (-m10 * c5 + m12 * c2 - m13 * c1) * invDet,
            (m10 * c4 - m11 * c2 + m13 * c0) * invDet,
            (-m10 * c3 + m11 * c1 - m12 * c0) * invDet),
        Vec<T, 4>::set(
            (-m01 * c5 + m02 * c4 - m03 * c3) * invDet,
            (m00 * c5 - m02 * c2 + m03 * c1) * invDet,
            (-m00 * c4 + m01 * c2 - m03 * c0) * invDet,
            (m00 * c3 - m01 * c1 + m02 * c0) * invDet),
        Vec<T, 4>::set(
            (m31 * s5 - m32 * s4 + m33 * s3) * invDet,
            (-m30 * s5 + m32 * s2 - m33 * s1) * invDet,
            (m30 * s4 - m31 * s2 + m33 * s0) * invDet,
            (-m30 * s3 + m31 * s1 - m32 * s0) * invDet),
        Vec<T, 4>::set(
            (-m21 * s5 + m22 * s4 - m23 * s3) * invDet,
            (m20 * s5 - m22 * s2 + m23 * s1) * invDet,
            (-m20 * s4 + m21 * s2 - m23 * s0) * invDet,
            (m20 * s3 - m21 * s1 + m22 * s0) * invDet));
    return true;
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

/// Outer product: produces a 4x4 matrix from two vectors
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Matrix4x4<T> outer(
    const Vector4<T>& a, const Vector4<T>& b)
{
  return Matrix4x4<T>(
      a.data * Vec<T, 4>::broadcast(b.x()),
      a.data * Vec<T, 4>::broadcast(b.y()),
      a.data * Vec<T, 4>::broadcast(b.z()),
      a.data * Vec<T, 4>::broadcast(b.w()));
}

using Matrix4x4f = Matrix4x4<float>;
using Matrix4x4d = Matrix4x4<double>;

} // namespace dart::simd
