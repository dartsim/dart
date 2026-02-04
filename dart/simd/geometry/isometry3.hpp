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

#include <dart/simd/geometry/quaternion.hpp>

#include <Eigen/Geometry>

namespace dart::simd {

template <typename T>
struct Isometry3
{
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "Isometry3 only supports float or double");

  Quaternion<T> rotation;
  Vector3<T> translation;

  Isometry3() = default;

  DART_SIMD_INLINE Isometry3(const Quaternion<T>& r, const Vector3<T>& t)
    : rotation(r), translation(t)
  {
  }

  DART_SIMD_INLINE explicit Isometry3(
      const Eigen::Transform<T, 3, Eigen::Isometry>& iso)
    : rotation(Eigen::Quaternion<T>(iso.rotation())),
      translation(Vector3<T>::fromEigen(iso.translation()))
  {
  }

  [[nodiscard]] DART_SIMD_INLINE static Isometry3 identity()
  {
    return Isometry3(Quaternion<T>::identity(), Vector3<T>::zero());
  }

  [[nodiscard]] DART_SIMD_INLINE static Isometry3 fromTranslation(
      const Vector3<T>& t)
  {
    return Isometry3(Quaternion<T>::identity(), t);
  }

  [[nodiscard]] DART_SIMD_INLINE static Isometry3 fromRotation(
      const Quaternion<T>& r)
  {
    return Isometry3(r, Vector3<T>::zero());
  }

  [[nodiscard]] DART_SIMD_INLINE Vector3<T> transformPoint(
      const Vector3<T>& p) const
  {
    return rotation.rotate(p) + translation;
  }

  [[nodiscard]] DART_SIMD_INLINE Vector3<T> transformVector(
      const Vector3<T>& v) const
  {
    return rotation.rotate(v);
  }

  [[nodiscard]] DART_SIMD_INLINE Isometry3 operator*(const Isometry3& rhs) const
  {
    return Isometry3(
        rotation * rhs.rotation,
        rotation.rotate(rhs.translation) + translation);
  }

  DART_SIMD_INLINE Isometry3& operator*=(const Isometry3& rhs)
  {
    *this = *this * rhs;
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE Isometry3 inverse() const
  {
    Quaternion<T> inv_rot = rotation.inverse();
    return Isometry3(inv_rot, inv_rot.rotate(-translation));
  }

  [[nodiscard]] DART_SIMD_INLINE Eigen::Transform<T, 3, Eigen::Isometry>
  toEigen() const
  {
    Eigen::Transform<T, 3, Eigen::Isometry> result
        = Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    result.linear() = rotation.toEigen().toRotationMatrix();
    result.translation() = translation.toEigen();
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE static Isometry3 fromEigen(
      const Eigen::Transform<T, 3, Eigen::Isometry>& iso)
  {
    return Isometry3(iso);
  }
};

template <typename T>
[[nodiscard]] DART_SIMD_INLINE Isometry3<T> lerp(
    const Isometry3<T>& a, const Isometry3<T>& b, T t)
{
  return Isometry3<T>(
      slerp(a.rotation, b.rotation, t),
      simd::lerp(a.translation, b.translation, t));
}

using Isometry3f = Isometry3<float>;
using Isometry3d = Isometry3<double>;

} // namespace dart::simd
