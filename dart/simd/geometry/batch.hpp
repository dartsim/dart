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

#include <dart/simd/fwd.hpp>
#include <dart/simd/geometry/matrix3x3.hpp>
#include <dart/simd/geometry/matrix4x4.hpp>
#include <dart/simd/geometry/vector3.hpp>
#include <dart/simd/memory.hpp>

#include <array>

namespace dart::simd {

// =============================================================================
// Matrix3x3SoA: N matrices in Structure-of-Arrays layout for SIMD processing
// =============================================================================

/// @brief SoA layout for N 3x3 matrices
/// Stores elements as 9 vectors, each containing N values from N matrices.
/// This layout enables SIMD-parallel computation of determinant, inverse, etc.
///
/// Layout: m[row][col] stored as m_rc where r=row, c=col (column-major)
/// - m00, m10, m20 = column 0
/// - m01, m11, m21 = column 1
/// - m02, m12, m22 = column 2
template <typename T, std::size_t N>
struct Matrix3x3SoA
{
  static_assert(N >= 1 && N <= 16, "N must be between 1 and 16");
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Matrix3x3SoA only supports float or double");

  // Column 0
  Vec<T, N> m00, m10, m20;
  // Column 1
  Vec<T, N> m01, m11, m21;
  // Column 2
  Vec<T, N> m02, m12, m22;

  Matrix3x3SoA() = default;

  /// Load N matrices from an array of Matrix3x3
  explicit Matrix3x3SoA(const std::array<Matrix3x3<T>, N>& aos) noexcept
  {
    alignas(64) T arr00[N], arr10[N], arr20[N];
    alignas(64) T arr01[N], arr11[N], arr21[N];
    alignas(64) T arr02[N], arr12[N], arr22[N];

    for (std::size_t i = 0; i < N; ++i) {
      arr00[i] = aos[i].col0[0];
      arr10[i] = aos[i].col0[1];
      arr20[i] = aos[i].col0[2];
      arr01[i] = aos[i].col1[0];
      arr11[i] = aos[i].col1[1];
      arr21[i] = aos[i].col1[2];
      arr02[i] = aos[i].col2[0];
      arr12[i] = aos[i].col2[1];
      arr22[i] = aos[i].col2[2];
    }

    m00 = Vec<T, N>::load(arr00);
    m10 = Vec<T, N>::load(arr10);
    m20 = Vec<T, N>::load(arr20);
    m01 = Vec<T, N>::load(arr01);
    m11 = Vec<T, N>::load(arr11);
    m21 = Vec<T, N>::load(arr21);
    m02 = Vec<T, N>::load(arr02);
    m12 = Vec<T, N>::load(arr12);
    m22 = Vec<T, N>::load(arr22);
  }

  /// Load from 9 contiguous arrays (one per matrix element), each of length N
  static Matrix3x3SoA loadFromArrays(
      const T* arr00,
      const T* arr10,
      const T* arr20,
      const T* arr01,
      const T* arr11,
      const T* arr21,
      const T* arr02,
      const T* arr12,
      const T* arr22) noexcept
  {
    Matrix3x3SoA result;
    result.m00 = Vec<T, N>::loadu(arr00);
    result.m10 = Vec<T, N>::loadu(arr10);
    result.m20 = Vec<T, N>::loadu(arr20);
    result.m01 = Vec<T, N>::loadu(arr01);
    result.m11 = Vec<T, N>::loadu(arr11);
    result.m21 = Vec<T, N>::loadu(arr21);
    result.m02 = Vec<T, N>::loadu(arr02);
    result.m12 = Vec<T, N>::loadu(arr12);
    result.m22 = Vec<T, N>::loadu(arr22);
    return result;
  }

  /// Store to 9 contiguous arrays
  void storeToArrays(
      T* arr00,
      T* arr10,
      T* arr20,
      T* arr01,
      T* arr11,
      T* arr21,
      T* arr02,
      T* arr12,
      T* arr22) const noexcept
  {
    m00.storeu(arr00);
    m10.storeu(arr10);
    m20.storeu(arr20);
    m01.storeu(arr01);
    m11.storeu(arr11);
    m21.storeu(arr21);
    m02.storeu(arr02);
    m12.storeu(arr12);
    m22.storeu(arr22);
  }

  /// Convert back to array of Matrix3x3
  [[nodiscard]] std::array<Matrix3x3<T>, N> toAos() const noexcept
  {
    alignas(64) T arr00[N], arr10[N], arr20[N];
    alignas(64) T arr01[N], arr11[N], arr21[N];
    alignas(64) T arr02[N], arr12[N], arr22[N];

    m00.store(arr00);
    m10.store(arr10);
    m20.store(arr20);
    m01.store(arr01);
    m11.store(arr11);
    m21.store(arr21);
    m02.store(arr02);
    m12.store(arr12);
    m22.store(arr22);

    std::array<Matrix3x3<T>, N> result;
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = Matrix3x3<T>(
          arr00[i],
          arr01[i],
          arr02[i],
          arr10[i],
          arr11[i],
          arr12[i],
          arr20[i],
          arr21[i],
          arr22[i]);
    }
    return result;
  }

  /// Get a single matrix at index i
  [[nodiscard]] Matrix3x3<T> get(std::size_t i) const noexcept
  {
    return Matrix3x3<T>(
        m00[i],
        m01[i],
        m02[i],
        m10[i],
        m11[i],
        m12[i],
        m20[i],
        m21[i],
        m22[i]);
  }

  /// Compute N determinants in parallel - SIMD optimized, no scalar extraction
  [[nodiscard]] Vec<T, N> determinant() const noexcept
  {
    // det = m00*(m11*m22 - m12*m21) - m01*(m10*m22 - m12*m20) +
    // m02*(m10*m21 - m11*m20)
    Vec<T, N> cofactor0 = fmsub(m11, m22, m12 * m21);
    Vec<T, N> cofactor1 = fmsub(m10, m22, m12 * m20);
    Vec<T, N> cofactor2 = fmsub(m10, m21, m11 * m20);
    return fmadd(m00, cofactor0, fmsub(m02, cofactor2, m01 * cofactor1));
  }

  /// Compute N matrix inverses in parallel - SIMD optimized
  [[nodiscard]] Matrix3x3SoA inverse() const noexcept
  {
    // Compute determinant
    Vec<T, N> cofactor00 = fmsub(m11, m22, m12 * m21);
    Vec<T, N> cofactor10 = fmsub(m12, m20, m10 * m22);
    Vec<T, N> cofactor20 = fmsub(m10, m21, m11 * m20);

    Vec<T, N> det = fmadd(m00, cofactor00, fmadd(m01, cofactor10, m02 * cofactor20));
    Vec<T, N> invDet = Vec<T, N>::broadcast(T(1)) / det;

    // Compute adjugate (transposed cofactor matrix)
    Matrix3x3SoA result;
    result.m00 = cofactor00 * invDet;
    result.m10 = cofactor10 * invDet;
    result.m20 = cofactor20 * invDet;

    result.m01 = fmsub(m02, m21, m01 * m22) * invDet;
    result.m11 = fmsub(m00, m22, m02 * m20) * invDet;
    result.m21 = fmsub(m01, m20, m00 * m21) * invDet;

    result.m02 = fmsub(m01, m12, m02 * m11) * invDet;
    result.m12 = fmsub(m02, m10, m00 * m12) * invDet;
    result.m22 = fmsub(m00, m11, m01 * m10) * invDet;

    return result;
  }
};

// Type aliases for common widths
using Matrix3x3SoAf4 = Matrix3x3SoA<float, 4>;
using Matrix3x3SoAf8 = Matrix3x3SoA<float, 8>;
using Matrix3x3SoAd2 = Matrix3x3SoA<double, 2>;
using Matrix3x3SoAd4 = Matrix3x3SoA<double, 4>;

// =============================================================================
// Matrix4x4SoA: N matrices in Structure-of-Arrays layout
// =============================================================================

/// @brief SoA layout for N 4x4 matrices
template <typename T, std::size_t N>
struct Matrix4x4SoA
{
  static_assert(N >= 1 && N <= 16, "N must be between 1 and 16");
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Matrix4x4SoA only supports float or double");

  // Column 0
  Vec<T, N> m00, m10, m20, m30;
  // Column 1
  Vec<T, N> m01, m11, m21, m31;
  // Column 2
  Vec<T, N> m02, m12, m22, m32;
  // Column 3
  Vec<T, N> m03, m13, m23, m33;

  Matrix4x4SoA() = default;

  /// Load N matrices from an array of Matrix4x4
  explicit Matrix4x4SoA(const std::array<Matrix4x4<T>, N>& aos) noexcept
  {
    alignas(64) T arr[16][N];

    for (std::size_t i = 0; i < N; ++i) {
      arr[0][i] = aos[i].col0[0];
      arr[1][i] = aos[i].col0[1];
      arr[2][i] = aos[i].col0[2];
      arr[3][i] = aos[i].col0[3];
      arr[4][i] = aos[i].col1[0];
      arr[5][i] = aos[i].col1[1];
      arr[6][i] = aos[i].col1[2];
      arr[7][i] = aos[i].col1[3];
      arr[8][i] = aos[i].col2[0];
      arr[9][i] = aos[i].col2[1];
      arr[10][i] = aos[i].col2[2];
      arr[11][i] = aos[i].col2[3];
      arr[12][i] = aos[i].col3[0];
      arr[13][i] = aos[i].col3[1];
      arr[14][i] = aos[i].col3[2];
      arr[15][i] = aos[i].col3[3];
    }

    m00 = Vec<T, N>::load(arr[0]);
    m10 = Vec<T, N>::load(arr[1]);
    m20 = Vec<T, N>::load(arr[2]);
    m30 = Vec<T, N>::load(arr[3]);
    m01 = Vec<T, N>::load(arr[4]);
    m11 = Vec<T, N>::load(arr[5]);
    m21 = Vec<T, N>::load(arr[6]);
    m31 = Vec<T, N>::load(arr[7]);
    m02 = Vec<T, N>::load(arr[8]);
    m12 = Vec<T, N>::load(arr[9]);
    m22 = Vec<T, N>::load(arr[10]);
    m32 = Vec<T, N>::load(arr[11]);
    m03 = Vec<T, N>::load(arr[12]);
    m13 = Vec<T, N>::load(arr[13]);
    m23 = Vec<T, N>::load(arr[14]);
    m33 = Vec<T, N>::load(arr[15]);
  }

  /// Load from 16 contiguous arrays (one per matrix element), each of length N
  static Matrix4x4SoA loadFromArrays(
      const T* arr00,
      const T* arr10,
      const T* arr20,
      const T* arr30,
      const T* arr01,
      const T* arr11,
      const T* arr21,
      const T* arr31,
      const T* arr02,
      const T* arr12,
      const T* arr22,
      const T* arr32,
      const T* arr03,
      const T* arr13,
      const T* arr23,
      const T* arr33) noexcept
  {
    Matrix4x4SoA result;
    result.m00 = Vec<T, N>::loadu(arr00);
    result.m10 = Vec<T, N>::loadu(arr10);
    result.m20 = Vec<T, N>::loadu(arr20);
    result.m30 = Vec<T, N>::loadu(arr30);
    result.m01 = Vec<T, N>::loadu(arr01);
    result.m11 = Vec<T, N>::loadu(arr11);
    result.m21 = Vec<T, N>::loadu(arr21);
    result.m31 = Vec<T, N>::loadu(arr31);
    result.m02 = Vec<T, N>::loadu(arr02);
    result.m12 = Vec<T, N>::loadu(arr12);
    result.m22 = Vec<T, N>::loadu(arr22);
    result.m32 = Vec<T, N>::loadu(arr32);
    result.m03 = Vec<T, N>::loadu(arr03);
    result.m13 = Vec<T, N>::loadu(arr13);
    result.m23 = Vec<T, N>::loadu(arr23);
    result.m33 = Vec<T, N>::loadu(arr33);
    return result;
  }

  /// Get a single matrix at index i
  [[nodiscard]] Matrix4x4<T> get(std::size_t i) const noexcept
  {
    return Matrix4x4<T>(
        Vec<T, 4>::set(m00[i], m10[i], m20[i], m30[i]),
        Vec<T, 4>::set(m01[i], m11[i], m21[i], m31[i]),
        Vec<T, 4>::set(m02[i], m12[i], m22[i], m32[i]),
        Vec<T, 4>::set(m03[i], m13[i], m23[i], m33[i]));
  }

  /// Compute N determinants in parallel - SIMD optimized
  /// Uses 2x2 minor expansion for efficiency
  /// Formula: det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0
  [[nodiscard]] Vec<T, N> determinant() const noexcept
  {
    // Compute 2x2 minors for rows 0,1
    Vec<T, N> s0 = fmsub(m00, m11, m10 * m01);
    Vec<T, N> s1 = fmsub(m00, m12, m10 * m02);
    Vec<T, N> s2 = fmsub(m00, m13, m10 * m03);
    Vec<T, N> s3 = fmsub(m01, m12, m11 * m02);
    Vec<T, N> s4 = fmsub(m01, m13, m11 * m03);
    Vec<T, N> s5 = fmsub(m02, m13, m12 * m03);

    // Compute 2x2 minors for rows 2,3
    Vec<T, N> c5 = fmsub(m22, m33, m32 * m23);
    Vec<T, N> c4 = fmsub(m21, m33, m31 * m23);
    Vec<T, N> c3 = fmsub(m21, m32, m31 * m22);
    Vec<T, N> c2 = fmsub(m20, m33, m30 * m23);
    Vec<T, N> c1 = fmsub(m20, m32, m30 * m22);
    Vec<T, N> c0 = fmsub(m20, m31, m30 * m21);

    // det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0
    return s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
  }

  /// Compute N matrix inverses in parallel - SIMD optimized
  [[nodiscard]] Matrix4x4SoA inverse() const noexcept
  {
    Vec<T, N> s0 = fmsub(m00, m11, m10 * m01);
    Vec<T, N> s1 = fmsub(m00, m12, m10 * m02);
    Vec<T, N> s2 = fmsub(m00, m13, m10 * m03);
    Vec<T, N> s3 = fmsub(m01, m12, m11 * m02);
    Vec<T, N> s4 = fmsub(m01, m13, m11 * m03);
    Vec<T, N> s5 = fmsub(m02, m13, m12 * m03);

    Vec<T, N> c5 = fmsub(m22, m33, m32 * m23);
    Vec<T, N> c4 = fmsub(m21, m33, m31 * m23);
    Vec<T, N> c3 = fmsub(m21, m32, m31 * m22);
    Vec<T, N> c2 = fmsub(m20, m33, m30 * m23);
    Vec<T, N> c1 = fmsub(m20, m32, m30 * m22);
    Vec<T, N> c0 = fmsub(m20, m31, m30 * m21);

    Vec<T, N> det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    Vec<T, N> invDet = Vec<T, N>::broadcast(T(1)) / det;

    Matrix4x4SoA result;

    // Column 0 of inverse (transposed from row-major adjugate formula)
    result.m00 = (m11 * c5 - m12 * c4 + m13 * c3) * invDet;
    result.m10 = (-m10 * c5 + m12 * c2 - m13 * c1) * invDet;
    result.m20 = (m10 * c4 - m11 * c2 + m13 * c0) * invDet;
    result.m30 = (-m10 * c3 + m11 * c1 - m12 * c0) * invDet;

    // Column 1 of inverse
    result.m01 = (-m01 * c5 + m02 * c4 - m03 * c3) * invDet;
    result.m11 = (m00 * c5 - m02 * c2 + m03 * c1) * invDet;
    result.m21 = (-m00 * c4 + m01 * c2 - m03 * c0) * invDet;
    result.m31 = (m00 * c3 - m01 * c1 + m02 * c0) * invDet;

    // Column 2 of inverse
    result.m02 = (m31 * s5 - m32 * s4 + m33 * s3) * invDet;
    result.m12 = (-m30 * s5 + m32 * s2 - m33 * s1) * invDet;
    result.m22 = (m30 * s4 - m31 * s2 + m33 * s0) * invDet;
    result.m32 = (-m30 * s3 + m31 * s1 - m32 * s0) * invDet;

    // Column 3 of inverse
    result.m03 = (-m21 * s5 + m22 * s4 - m23 * s3) * invDet;
    result.m13 = (m20 * s5 - m22 * s2 + m23 * s1) * invDet;
    result.m23 = (-m20 * s4 + m21 * s2 - m23 * s0) * invDet;
    result.m33 = (m20 * s3 - m21 * s1 + m22 * s0) * invDet;

    return result;
  }
};

// Type aliases for common widths
using Matrix4x4SoAf4 = Matrix4x4SoA<float, 4>;
using Matrix4x4SoAf8 = Matrix4x4SoA<float, 8>;
using Matrix4x4SoAd2 = Matrix4x4SoA<double, 2>;
using Matrix4x4SoAd4 = Matrix4x4SoA<double, 4>;

// =============================================================================
// Vector3SoA: N vectors in Structure-of-Arrays layout
// =============================================================================

/// @brief SoA layout for N 3D vectors - extends EigenSoA3 with more operations
template <typename T, std::size_t N>
struct Vector3SoA
{
  static_assert(N >= 1 && N <= 16, "N must be between 1 and 16");
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Vector3SoA only supports float or double");

  Vec<T, N> x, y, z;

  Vector3SoA() = default;

  Vector3SoA(const Vec<T, N>& x_, const Vec<T, N>& y_, const Vec<T, N>& z_)
    : x(x_), y(y_), z(z_)
  {
  }

  explicit Vector3SoA(const std::array<Vector3<T>, N>& aos) noexcept
  {
    alignas(64) T xs[N], ys[N], zs[N];
    for (std::size_t i = 0; i < N; ++i) {
      xs[i] = aos[i].x();
      ys[i] = aos[i].y();
      zs[i] = aos[i].z();
    }
    x = Vec<T, N>::load(xs);
    y = Vec<T, N>::load(ys);
    z = Vec<T, N>::load(zs);
  }

  static Vector3SoA loadFromArrays(
      const T* xs, const T* ys, const T* zs) noexcept
  {
    Vector3SoA result;
    result.x = Vec<T, N>::loadu(xs);
    result.y = Vec<T, N>::loadu(ys);
    result.z = Vec<T, N>::loadu(zs);
    return result;
  }

  void storeToArrays(T* xs, T* ys, T* zs) const noexcept
  {
    x.storeu(xs);
    y.storeu(ys);
    z.storeu(zs);
  }

  [[nodiscard]] std::array<Vector3<T>, N> toAos() const noexcept
  {
    alignas(64) T xs[N], ys[N], zs[N];
    x.store(xs);
    y.store(ys);
    z.store(zs);

    std::array<Vector3<T>, N> result;
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = Vector3<T>(xs[i], ys[i], zs[i]);
    }
    return result;
  }

  [[nodiscard]] Vector3<T> get(std::size_t i) const noexcept
  {
    return Vector3<T>(x[i], y[i], z[i]);
  }

  /// Element-wise addition
  [[nodiscard]] Vector3SoA operator+(const Vector3SoA& rhs) const noexcept
  {
    return Vector3SoA(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  /// Element-wise subtraction
  [[nodiscard]] Vector3SoA operator-(const Vector3SoA& rhs) const noexcept
  {
    return Vector3SoA(x - rhs.x, y - rhs.y, z - rhs.z);
  }

  /// Scalar multiplication
  [[nodiscard]] Vector3SoA operator*(const Vec<T, N>& s) const noexcept
  {
    return Vector3SoA(x * s, y * s, z * s);
  }

  /// Negation
  [[nodiscard]] Vector3SoA operator-() const noexcept
  {
    return Vector3SoA(-x, -y, -z);
  }
};

/// Batch dot product: returns N dot products
template <typename T, std::size_t N>
[[nodiscard]] inline Vec<T, N> dot(
    const Vector3SoA<T, N>& a, const Vector3SoA<T, N>& b) noexcept
{
  return fmadd(a.x, b.x, fmadd(a.y, b.y, a.z * b.z));
}

/// Batch cross product: returns N cross products
template <typename T, std::size_t N>
[[nodiscard]] inline Vector3SoA<T, N> cross(
    const Vector3SoA<T, N>& a, const Vector3SoA<T, N>& b) noexcept
{
  return Vector3SoA<T, N>(
      fmsub(a.y, b.z, a.z * b.y),
      fmsub(a.z, b.x, a.x * b.z),
      fmsub(a.x, b.y, a.y * b.x));
}

/// Batch squared norm: returns N squared norms
template <typename T, std::size_t N>
[[nodiscard]] inline Vec<T, N> squaredNorm(const Vector3SoA<T, N>& v) noexcept
{
  return fmadd(v.x, v.x, fmadd(v.y, v.y, v.z * v.z));
}

/// Batch normalize: returns N normalized vectors
template <typename T, std::size_t N>
[[nodiscard]] inline Vector3SoA<T, N> normalize(
    const Vector3SoA<T, N>& v) noexcept
{
  Vec<T, N> invLen = rsqrt(squaredNorm(v));
  return Vector3SoA<T, N>(v.x * invLen, v.y * invLen, v.z * invLen);
}

/// Batch outer product: returns N outer products (3x3 matrices)
template <typename T, std::size_t N>
[[nodiscard]] inline Matrix3x3SoA<T, N> outer(
    const Vector3SoA<T, N>& a, const Vector3SoA<T, N>& b) noexcept
{
  Matrix3x3SoA<T, N> result;
  // Column 0: a * b.x
  result.m00 = a.x * b.x;
  result.m10 = a.y * b.x;
  result.m20 = a.z * b.x;
  // Column 1: a * b.y
  result.m01 = a.x * b.y;
  result.m11 = a.y * b.y;
  result.m21 = a.z * b.y;
  // Column 2: a * b.z
  result.m02 = a.x * b.z;
  result.m12 = a.y * b.z;
  result.m22 = a.z * b.z;
  return result;
}

/// Batch reflect: v - 2*(v·n)*n for each of N vectors
/// Assumes n is normalized
template <typename T, std::size_t N>
[[nodiscard]] inline Vector3SoA<T, N> reflect(
    const Vector3SoA<T, N>& v, const Vector3SoA<T, N>& n) noexcept
{
  Vec<T, N> d = dot(v, n);
  Vec<T, N> two_d = d + d;
  return Vector3SoA<T, N>(
      v.x - two_d * n.x,
      v.y - two_d * n.y,
      v.z - two_d * n.z);
}

/// Batch project: (a·b / b·b) * b for each of N vectors
template <typename T, std::size_t N>
[[nodiscard]] inline Vector3SoA<T, N> project(
    const Vector3SoA<T, N>& a, const Vector3SoA<T, N>& b) noexcept
{
  Vec<T, N> ab = dot(a, b);
  Vec<T, N> bb = dot(b, b);
  Vec<T, N> scale = ab / bb;
  return Vector3SoA<T, N>(b.x * scale, b.y * scale, b.z * scale);
}

// Type aliases for common widths
using Vector3SoAf4 = Vector3SoA<float, 4>;
using Vector3SoAf8 = Vector3SoA<float, 8>;
using Vector3SoAd2 = Vector3SoA<double, 2>;
using Vector3SoAd4 = Vector3SoA<double, 4>;

} // namespace dart::simd
