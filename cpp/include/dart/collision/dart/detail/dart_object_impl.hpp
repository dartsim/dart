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

#include "dart/collision/dart/dart_object.hpp"
#include "dart/collision/dart/dart_scene.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
math::SE3<Scalar> DartObject<Scalar>::get_pose() const
{
  return m_pose.to_transformation();
}

//==============================================================================
template <typename Scalar>
void DartObject<Scalar>::set_pose(const math::SE3<Scalar>& tf)
{
  m_pose = tf;
  m_aabb_dirty = true;
}

//==============================================================================
template <typename Scalar>
math::R3<Scalar> DartObject<Scalar>::get_position() const
{
  return m_pose.position();
}

//==============================================================================
template <typename Scalar>
void DartObject<Scalar>::set_position(const math::R3<Scalar>& pos)
{
  m_pose.position() = pos;
  m_aabb_dirty = true;
}

//==============================================================================
template <typename Scalar>
const math::Aabb3<Scalar>& DartObject<Scalar>::get_aabb() const
{
  if (m_aabb_dirty) {
    update_aabb();
    m_aabb_dirty = false;
  }

  return m_aabb;
}

//==============================================================================
template <typename Scalar>
void DartObject<Scalar>::update_aabb() const
{
  if (!this->m_geometry) {
    m_aabb.reset();
    m_aabb_dirty = false;
    return;
  }

  const math::Aabb3<Scalar>& local_aabb = this->m_geometry->get_local_aabb();
  if (this->m_geometry->is_local_aabb_rotation_invariant()) {
    m_aabb.setTransformed(local_aabb, m_pose.to_transformation());
  } else {
    const math::Vector3<Scalar> center
        = local_aabb.get_center() + m_pose.to_translation();
    const Scalar radius = local_aabb.radius();
    m_aabb.set_from_sphere(center, radius);
  }
}

//==============================================================================
template <typename Scalar>
DartObject<Scalar>::DartObject(
    DartScene<Scalar>* group, ObjectId id, math::Geometry3Ptr<Scalar> geometry)
  : Object<Scalar>(group, id, std::move(geometry))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void DartObject<Scalar>::update_engine_data()
{
  // Do nothing
}

} // namespace collision
} // namespace dart
