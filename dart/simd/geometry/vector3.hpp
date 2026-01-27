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

#include <dart/simd/config.hpp>
#include <dart/simd/detail/scalar/operations.hpp>
#include <dart/simd/detail/scalar/vec.hpp>
#include <dart/simd/fwd.hpp>

#if defined(DART_SIMD_SSE42)
  #include <dart/simd/detail/sse42/operations.hpp>
  #include <dart/simd/detail/sse42/vec.hpp>
#endif

#if defined(DART_SIMD_AVX) && !defined(DART_SIMD_AVX2)
  #include <dart/simd/detail/avx/operations.hpp>
  #include <dart/simd/detail/avx/vec.hpp>
#endif

#if defined(DART_SIMD_AVX2)
  #include <dart/simd/detail/avx2/operations.hpp>
  #include <dart/simd/detail/avx2/vec.hpp>
#endif

#if defined(DART_SIMD_SVE)
  #include <dart/simd/detail/sve/operations.hpp>
  #include <dart/simd/detail/sve/vec.hpp>
#elif defined(DART_SIMD_NEON)
  #include <dart/simd/detail/neon/operations.hpp>
  #include <dart/simd/detail/neon/vec.hpp>
#endif

#if defined(DART_SIMD_AVX512)
  #include <dart/simd/detail/avx512/operations.hpp>
  #include <dart/simd/detail/avx512/vec.hpp>
#endif

#include <Eigen/Core>

#include <cmath>

namespace dart::simd {

/// @brief SIMD-backed 3D vector with Vec<T, 4> storage (padded for efficiency)
///
/// Vector3 stores [x, y, z, 0] in a 4-wide SIMD register. The padding lane
/// is always zero and ignored in computations. This allows efficient SIMD
/// operations while maintaining a 3D vector interface.
///
/// @tparam T Scalar type (float or double)
template <typename T>
struct Vector3
{
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Vector3 only supports float or double");

  /// Internal storage: [x, y, z, 0]
  Vec<T, 4> data;

  // ===========================================================================
  // Constructors
  // ===========================================================================

  /// Default constructor (uninitialized)
  Vector3() = default;

  /// Construct from components
  DART_SIMD_INLINE Vector3(T x, T y, T z) : data(Vec<T, 4>::set(x, y, z, T(0)))
  {
  }

  /// Construct from Vec<T, 4> (assumes w=0)
  DART_SIMD_INLINE explicit Vector3(const Vec<T, 4>& v) : data(v) {}

  /// Construct from Eigen vector
  DART_SIMD_INLINE explicit Vector3(const Eigen::Matrix<T, 3, 1>& v)
    : data(Vec<T, 4>::set(v.x(), v.y(), v.z(), T(0)))
  {
  }

  /// Broadcast scalar to all components
  DART_SIMD_INLINE static Vector3 broadcast(T s)
  {
    return Vector3(Vec<T, 4>::set(s, s, s, T(0)));
  }

  /// Zero vector
  DART_SIMD_INLINE static Vector3 zero()
  {
    return Vector3(Vec<T, 4>::zero());
  }

  /// Unit X vector (1, 0, 0)
  DART_SIMD_INLINE static Vector3 unit_x()
  {
    return Vector3(T(1), T(0), T(0));
  }

  /// Unit Y vector (0, 1, 0)
  DART_SIMD_INLINE static Vector3 unit_y()
  {
    return Vector3(T(0), T(1), T(0));
  }

  /// Unit Z vector (0, 0, 1)
  DART_SIMD_INLINE static Vector3 unit_z()
  {
    return Vector3(T(0), T(0), T(1));
  }

  // ===========================================================================
  // Element Access
  // ===========================================================================

  /// Access x component
  [[nodiscard]] DART_SIMD_INLINE T x() const
  {
    return data[0];
  }

  /// Access y component
  [[nodiscard]] DART_SIMD_INLINE T y() const
  {
    return data[1];
  }

  /// Access z component
  [[nodiscard]] DART_SIMD_INLINE T z() const
  {
    return data[2];
  }

  /// Access component by index (0=x, 1=y, 2=z)
  [[nodiscard]] DART_SIMD_INLINE T operator[](std::size_t i) const
  {
    return data[i];
  }

  // ===========================================================================
  // Arithmetic Operators
  // ===========================================================================

  /// Vector addition
  [[nodiscard]] DART_SIMD_INLINE Vector3 operator+(const Vector3& rhs) const
  {
    return Vector3(data + rhs.data);
  }

  /// Vector subtraction
  [[nodiscard]] DART_SIMD_INLINE Vector3 operator-(const Vector3& rhs) const
  {
    return Vector3(data - rhs.data);
  }

  /// Negation
  [[nodiscard]] DART_SIMD_INLINE Vector3 operator-() const
  {
    return Vector3(-data);
  }

  /// Component-wise multiplication
  [[nodiscard]] DART_SIMD_INLINE Vector3 operator*(const Vector3& rhs) const
  {
    return Vector3(data * rhs.data);
  }

  /// Scalar multiplication
  [[nodiscard]] DART_SIMD_INLINE Vector3 operator*(T s) const
  {
    return Vector3(data * Vec<T, 4>::broadcast(s));
  }

  /// Scalar multiplication (scalar on left)
  [[nodiscard]] DART_SIMD_INLINE friend Vector3 operator*(T s, const Vector3& v)
  {
    return v * s;
  }

  /// Scalar division
  [[nodiscard]] DART_SIMD_INLINE Vector3 operator/(T s) const
  {
    return Vector3(data / Vec<T, 4>::broadcast(s));
  }

  /// Compound addition
  DART_SIMD_INLINE Vector3& operator+=(const Vector3& rhs)
  {
    data = data + rhs.data;
    return *this;
  }

  /// Compound subtraction
  DART_SIMD_INLINE Vector3& operator-=(const Vector3& rhs)
  {
    data = data - rhs.data;
    return *this;
  }

  /// Compound scalar multiplication
  DART_SIMD_INLINE Vector3& operator*=(T s)
  {
    data = data * Vec<T, 4>::broadcast(s);
    return *this;
  }

  /// Compound scalar division
  DART_SIMD_INLINE Vector3& operator/=(T s)
  {
    data = data / Vec<T, 4>::broadcast(s);
    return *this;
  }

  // ===========================================================================
  // Comparison Operators
  // ===========================================================================

  /// Equality comparison
  [[nodiscard]] DART_SIMD_INLINE bool operator==(const Vector3& rhs) const
  {
    // Compare x, y, z only (ignore w padding)
    return x() == rhs.x() && y() == rhs.y() && z() == rhs.z();
  }

  /// Inequality comparison
  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const Vector3& rhs) const
  {
    return !(*this == rhs);
  }

  // ===========================================================================
  // Vector Operations
  // ===========================================================================

  /// Squared length (faster than length())
  [[nodiscard]] DART_SIMD_INLINE T squaredNorm() const
  {
    Vec<T, 4> sq = data * data;
    return sq[0] + sq[1] + sq[2];
  }

  /// Length (Euclidean norm)
  [[nodiscard]] DART_SIMD_INLINE T norm() const
  {
    return std::sqrt(squaredNorm());
  }

  /// Normalize to unit length
  [[nodiscard]] DART_SIMD_INLINE Vector3 normalized() const
  {
    T len = norm();
    return (len > T(0)) ? (*this / len) : zero();
  }

  /// Normalize in place
  DART_SIMD_INLINE void normalize()
  {
    *this = normalized();
  }

  // ===========================================================================
  // Eigen Interop
  // ===========================================================================

  /// Convert to Eigen vector (copy)
  [[nodiscard]] DART_SIMD_INLINE Eigen::Matrix<T, 3, 1> toEigen() const
  {
    return Eigen::Matrix<T, 3, 1>(x(), y(), z());
  }

  /// Create from Eigen vector
  [[nodiscard]] DART_SIMD_INLINE static Vector3 fromEigen(
      const Eigen::Matrix<T, 3, 1>& v)
  {
    return Vector3(v);
  }
};

// =============================================================================
// Free Functions
// =============================================================================

/// Dot product
template <typename T>
[[nodiscard]] DART_SIMD_INLINE T dot(const Vector3<T>& a, const Vector3<T>& b)
{
  Vec<T, 4> prod = a.data * b.data;
  return prod[0] + prod[1] + prod[2];
}

/// Cross product
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector3<T> cross(
    const Vector3<T>& a, const Vector3<T>& b)
{
  // cross = (a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)
  T cx = a.y() * b.z() - a.z() * b.y();
  T cy = a.z() * b.x() - a.x() * b.z();
  T cz = a.x() * b.y() - a.y() * b.x();
  return Vector3<T>(cx, cy, cz);
}

/// Component-wise minimum
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector3<T> min(
    const Vector3<T>& a, const Vector3<T>& b)
{
  return Vector3<T>(min(a.data, b.data));
}

/// Component-wise maximum
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector3<T> max(
    const Vector3<T>& a, const Vector3<T>& b)
{
  return Vector3<T>(max(a.data, b.data));
}

/// Component-wise absolute value
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector3<T> abs(const Vector3<T>& v)
{
  return Vector3<T>(abs(v.data));
}

/// Linear interpolation: a + t * (b - a)
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector3<T> lerp(
    const Vector3<T>& a, const Vector3<T>& b, T t)
{
  return a + (b - a) * t;
}

/// Distance between two points
template <typename T>
[[nodiscard]] DART_SIMD_INLINE T
distance(const Vector3<T>& a, const Vector3<T>& b)
{
  return (b - a).norm();
}

/// Squared distance between two points (faster)
template <typename T>
[[nodiscard]] DART_SIMD_INLINE T
squaredDistance(const Vector3<T>& a, const Vector3<T>& b)
{
  return (b - a).squaredNorm();
}

/// Reflect vector v around normal n (n must be normalized)
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector3<T> reflect(
    const Vector3<T>& v, const Vector3<T>& n)
{
  return v - T(2) * dot(v, n) * n;
}

/// Project vector a onto vector b
template <typename T>
[[nodiscard]] DART_SIMD_INLINE Vector3<T> project(
    const Vector3<T>& a, const Vector3<T>& b)
{
  T bLenSq = b.squaredNorm();
  if (bLenSq < T(1e-12)) {
    return Vector3<T>::zero();
  }
  return (dot(a, b) / bLenSq) * b;
}

// =============================================================================
// Type Aliases
// =============================================================================

using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;

} // namespace dart::simd
