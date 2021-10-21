/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/collision/fcl/fcl_conversion.hpp"
#include "dart/collision/fcl/fcl_object.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
math::Isometry3<Scalar> FclObject<Scalar>::get_pose() const
{
  return to_pose3<Scalar>(m_fcl_collision_object->getTransform());
}

//==============================================================================
template <typename Scalar>
void FclObject<Scalar>::set_pose(const math::Isometry3<Scalar>& tf)
{
  m_fcl_collision_object->setTransform(to_fcl_pose3<Scalar>(tf));
}

//==============================================================================
template <typename Scalar>
math::Vector3<Scalar> FclObject<Scalar>::get_position() const
{
  return to_vector3<Scalar>(m_fcl_collision_object->getTranslation());
}

//==============================================================================
template <typename Scalar>
void FclObject<Scalar>::set_position(const math::Vector3<Scalar>& pos)
{
  m_fcl_collision_object->setTranslation(to_fcl_vector3<Scalar>(pos));
}

//==============================================================================
template <typename Scalar>
FclCollisionObject<Scalar>* FclObject<Scalar>::get_fcl_collision_object()
{
  return m_fcl_collision_object.get();
}

//==============================================================================
template <typename Scalar>
const FclCollisionObject<Scalar>* FclObject<Scalar>::get_fcl_collision_object()
    const
{
  return m_fcl_collision_object.get();
}

//==============================================================================
template <typename Scalar>
FclObject<Scalar>::FclObject(
    Scene<Scalar>* collision_scene,
    math::GeometryPtr shape,
    const std::shared_ptr<FclCollisionGeometry<Scalar>>& fcl_coll_geom)
  : Object<Scalar>(collision_scene, shape),
    m_fcl_collision_object(new FclCollisionObject<Scalar>(fcl_coll_geom))
{
  assert(fcl_coll_geom);
  m_fcl_collision_object->setUserData(this);
}

//==============================================================================
template <typename Scalar>
void FclObject<Scalar>::update_engine_data()
{
  m_fcl_collision_object->setTransform(to_fcl_pose3(get_pose()));
  m_fcl_collision_object->computeAABB();
}

} // namespace collision
} // namespace dart
