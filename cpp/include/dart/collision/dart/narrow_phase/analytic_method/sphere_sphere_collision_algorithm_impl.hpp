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

#include "dart/collision/dart/narrow_phase/analytic_method/sphere_sphere_collision_algorithm.hpp"
#include "dart/math/geometry/sphere.hpp"
#include "dart/math/lie_group/r.hpp"
#include "dart/math/type.hpp"

namespace dart::collision::detail {

//==============================================================================
template <typename Scalar>
SphereSphereCollisionAlgorithm<Scalar>::SphereSphereCollisionAlgorithm(
    common::MemoryAllocator& allocator)
  : CollisionAlgorithm<Scalar>(allocator)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
SphereSphereCollisionAlgorithm<Scalar>::~SphereSphereCollisionAlgorithm()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool SphereSphereCollisionAlgorithm<Scalar>::run(
    const Object<Scalar>& object_a,
    const Object<Scalar>& object_b,
    NarrowPhaseCallback<Scalar>* callback)
{
  DART_NOT_IMPLEMENTED;

  math::R3<Scalar> diff = object_b.get_position() - object_a.get_position();
  const Scalar len = diff.norm();

  auto sphere_a = object_a.get_geometry()->template as<math::Sphere<Scalar>>();
  auto sphere_b = object_b.get_geometry()->template as<math::Sphere<Scalar>>();

  assert(sphere_a);
  assert(sphere_b);

  if (len > sphere_a->get_radius() + sphere_b->get_radius()) {
    return false;
  }

  if (!callback) {
    return true;
  }

  const math::Vector3<Scalar> normal = len > 0 ? (diff / len) : diff;
  const math::Vector3<Scalar> point
      = object_a.get_position() + diff * sphere_a->get_radius() / 1;
  const Scalar depth = sphere_a->get_radius() + sphere_b->get_radius() - len;

  (*callback)(point, normal, depth);

  return true;
}

} // namespace dart::collision::detail
