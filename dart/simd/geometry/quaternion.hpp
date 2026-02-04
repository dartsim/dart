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

#include <dart/simd/geometry/matrix3x3.hpp>
#include <dart/simd/geometry/vector3.hpp>

#include <Eigen/Geometry>

#include <cmath>

namespace dart::simd {

// Storage: [x, y, z, w] matching Eigen::Quaternion internal order
template <typename T>
struct Quaternion
{
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Quaternion only supports float or double");

  Vec<T, 4> data;

  Quaternion() = default;

  DART_SIMD_INLINE Quaternion(T w, T x, T y, T z)
    : data(Vec<T, 4>::set(x, y, z, w))
  {
  }

  DART_SIMD_INLINE explicit Quaternion(const Vec<T, 4>& v) : data(v) {}

  DART_SIMD_INLINE explicit Quaternion(const Eigen::Quaternion<T>& q)
    : data(Vec<T, 4>::set(q.x(), q.y(), q.z(), q.w()))
  {
  }

  [[nodiscard]] DART_SIMD_INLINE static Quaternion identity()
  {
    return Quaternion(T(1), T(0), T(0), T(0));
  }

  [[nodiscard]] DART_SIMD_INLINE static Quaternion fromAxisAngle(
      const Vector3<T>& axis, T angle)
  {
    T half_angle = angle * T(0.5);
    T s = std::sin(half_angle);
    T c = std::cos(half_angle);
    Vector3<T> n = axis.normalized();
    return Quaternion(c, n.x() * s, n.y() * s, n.z() * s);
  }

  [[nodiscard]] DART_SIMD_INLINE static Quaternion fromRotationMatrix(
      const Matrix3x3<T>& m)
  {
    T trace = m(0, 0) + m(1, 1) + m(2, 2);
    T w, x, y, z;

    if (trace > T(0)) {
      T s = std::sqrt(trace + T(1)) * T(2);
      w = T(0.25) * s;
      x = (m(2, 1) - m(1, 2)) / s;
      y = (m(0, 2) - m(2, 0)) / s;
      z = (m(1, 0) - m(0, 1)) / s;
    } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
      T s = std::sqrt(T(1) + m(0, 0) - m(1, 1) - m(2, 2)) * T(2);
      w = (m(2, 1) - m(1, 2)) / s;
      x = T(0.25) * s;
      y = (m(0, 1) + m(1, 0)) / s;
      z = (m(0, 2) + m(2, 0)) / s;
    } else if (m(1, 1) > m(2, 2)) {
      T s = std::sqrt(T(1) + m(1, 1) - m(0, 0) - m(2, 2)) * T(2);
      w = (m(0, 2) - m(2, 0)) / s;
      x = (m(0, 1) + m(1, 0)) / s;
      y = T(0.25) * s;
      z = (m(1, 2) + m(2, 1)) / s;
    } else {
      T s = std::sqrt(T(1) + m(2, 2) - m(0, 0) - m(1, 1)) * T(2);
      w = (m(1, 0) - m(0, 1)) / s;
      x = (m(0, 2) + m(2, 0)) / s;
      y = (m(1, 2) + m(2, 1)) / s;
      z = T(0.25) * s;
    }

    return Quaternion(w, x, y, z);
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

  [[nodiscard]] DART_SIMD_INLINE Vec<T, 4> vec() const
  {
    return data;
  }

  [[nodiscard]] DART_SIMD_INLINE Quaternion conjugate() const
  {
    return Quaternion(Vec<T, 4>::set(-x(), -y(), -z(), w()));
  }

  [[nodiscard]] DART_SIMD_INLINE T squaredNorm() const
  {
    return hsum(data * data);
  }

  [[nodiscard]] DART_SIMD_INLINE T norm() const
  {
    return std::sqrt(squaredNorm());
  }

  [[nodiscard]] DART_SIMD_INLINE Quaternion normalized() const
  {
    T len = norm();
    if (len > T(0)) {
      return Quaternion(data / Vec<T, 4>::broadcast(len));
    }
    return identity();
  }

  DART_SIMD_INLINE void normalize()
  {
    *this = normalized();
  }

  [[nodiscard]] DART_SIMD_INLINE Quaternion inverse() const
  {
    T sq_norm = squaredNorm();
    if (sq_norm > T(0)) {
      Quaternion conj = conjugate();
      return Quaternion(conj.data / Vec<T, 4>::broadcast(sq_norm));
    }
    return identity();
  }

  // Hamilton product: q1 * q2
  [[nodiscard]] DART_SIMD_INLINE Quaternion
  operator*(const Quaternion& rhs) const
  {
    T w1 = w(), x1 = x(), y1 = y(), z1 = z();
    T w2 = rhs.w(), x2 = rhs.x(), y2 = rhs.y(), z2 = rhs.z();

    return Quaternion(
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2);
  }

  DART_SIMD_INLINE Quaternion& operator*=(const Quaternion& rhs)
  {
    *this = *this * rhs;
    return *this;
  }

  // Rodrigues rotation: v' = v + 2w(q_xyz × v) + 2(q_xyz × (q_xyz × v))
  [[nodiscard]] DART_SIMD_INLINE Vector3<T> rotate(const Vector3<T>& v) const
  {
    Vector3<T> qvec(x(), y(), z());
    Vector3<T> t = T(2) * cross(qvec, v);
    return v + w() * t + cross(qvec, t);
  }

  [[nodiscard]] DART_SIMD_INLINE Matrix3x3<T> toRotationMatrix() const
  {
    T xx = x() * x(), yy = y() * y(), zz = z() * z();
    T xy = x() * y(), xz = x() * z(), yz = y() * z();
    T wx = w() * x(), wy = w() * y(), wz = w() * z();

    return Matrix3x3<T>(
        Vec<T, 4>::set(
            T(1) - T(2) * (yy + zz), T(2) * (xy + wz), T(2) * (xz - wy), T(0)),
        Vec<T, 4>::set(
            T(2) * (xy - wz), T(1) - T(2) * (xx + zz), T(2) * (yz + wx), T(0)),
        Vec<T, 4>::set(
            T(2) * (xz + wy), T(2) * (yz - wx), T(1) - T(2) * (xx + yy), T(0)));
  }

  [[nodiscard]] DART_SIMD_INLINE Eigen::Quaternion<T> toEigen() const
  {
    return Eigen::Quaternion<T>(w(), x(), y(), z());
  }

  [[nodiscard]] DART_SIMD_INLINE static Quaternion fromEigen(
      const Eigen::Quaternion<T>& q)
  {
    return Quaternion(q);
  }
};

template <typename T>
[[nodiscard]] DART_SIMD_INLINE T
dot(const Quaternion<T>& a, const Quaternion<T>& b)
{
  return hsum(a.data * b.data);
}

template <typename T>
[[nodiscard]] DART_SIMD_INLINE Quaternion<T> slerp(
    const Quaternion<T>& a, const Quaternion<T>& b, T t)
{
  T d = dot(a, b);

  Quaternion<T> b_adj = b;
  if (d < T(0)) {
    b_adj = Quaternion<T>(-b.data);
    d = -d;
  }

  constexpr T threshold = T(0.9995);
  if (d > threshold) {
    Quaternion<T> result(
        a.data + Vec<T, 4>::broadcast(t) * (b_adj.data - a.data));
    return result.normalized();
  }

  T theta_0 = std::acos(d);
  T theta = theta_0 * t;
  T sin_theta = std::sin(theta);
  T sin_theta_0 = std::sin(theta_0);

  T s0 = std::cos(theta) - d * sin_theta / sin_theta_0;
  T s1 = sin_theta / sin_theta_0;

  return Quaternion<T>(
      Vec<T, 4>::broadcast(s0) * a.data
      + Vec<T, 4>::broadcast(s1) * b_adj.data);
}

using Quaternionf = Quaternion<float>;
using Quaterniond = Quaternion<double>;

} // namespace dart::simd
