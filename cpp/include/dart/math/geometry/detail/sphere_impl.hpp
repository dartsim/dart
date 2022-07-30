/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <cmath>

#include "dart/math/constant.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace math {

//==============================================================================
template <typename S>
const std::string& Sphere<S>::GetType()
{
  static const std::string type("Sphere");
  return type;
}

//==============================================================================
template <typename S>
S Sphere<S>::ComputeVolume(S radius)
{
  return pi<S>() * S(4) / S(3) * radius * radius * radius;
}

//==============================================================================
template <typename S>
Eigen::Matrix<S, 3, 3> Sphere<S>::ComputeInertiaFromMass(S radius, S mass)
{
  Eigen::Matrix<S, 3, 3> inertia = Eigen::Matrix<S, 3, 3>::Zero();

  inertia(0, 0) = S(2) / S(5) * mass * radius * radius;
  inertia(1, 1) = inertia(0, 0);
  inertia(2, 2) = inertia(0, 0);

  return inertia;
}

//==============================================================================
template <typename S>
Eigen::Matrix<S, 3, 3> Sphere<S>::ComputeInertiaFromDensity(S radius, S density)
{
  const S mass = ComputeVolume(radius) * density;
  return ComputeInertiaFromMass(radius, mass);
}

//==============================================================================
template <typename S>
Sphere<S>::Sphere(S radius) : m_radius(radius)
{
  // Do nothing
}

//==============================================================================
template <typename S>
const std::string& Sphere<S>::get_type() const
{
  return GetType();
}

//==============================================================================
template <typename S>
S Sphere<S>::get_radius() const
{
  return m_radius;
}

//==============================================================================
template <typename S>
void Sphere<S>::set_radius(S radius)
{
  m_radius = radius;
}

//==============================================================================
template <typename S>
typename Sphere<S>::Vector3 Sphere<S>::get_local_support_point(
    const typename Sphere<S>::Vector3& direction) const
{
  return m_radius * direction.normalized();
}

//==============================================================================
template <typename Scalar>
bool Sphere<Scalar>::is_local_aabb_rotation_invariant() const
{
  return true;
}

//==============================================================================
template <typename Scalar>
void Sphere<Scalar>::update_local_aabb_impl() const
{
  this->m_local_aabb.set_from_sphere(math::Vector3<Scalar>::Zero(), m_radius);
}

} // namespace math
} // namespace dart
