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

#include "dart/collision/fcl/fcl_engine.hpp"
#include "dart/collision/fcl/fcl_object.hpp"
#include "dart/collision/fcl/fcl_scene.hpp"
#include "dart/collision/object.hpp"
#include "dart/common/logging.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
FclScene<Scalar>::FclScene(Engine<Scalar>* engine)
  : Scene<Scalar>(engine),
    m_broad_phase_alg(new FclDynamicAABBTreeCollisionManager<Scalar>())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
ObjectPtr<Scalar> FclScene<Scalar>::create_object(math::GeometryPtr shape)
{
  if (!shape) {
    DART_WARN("Not allowed to create a collision object for a null shape");
    return nullptr;
  }

  auto fcl_collision_geometry
      = get_mutable_fcl_engine()->create_fcl_collision_geometry(shape);
  if (!fcl_collision_geometry) {
    DART_WARN("Failed to create FCL collision geometry.");
    return nullptr;
  }

  return std::shared_ptr<FclObject<Scalar>>(
      new FclObject<Scalar>(this, std::move(shape), fcl_collision_geometry));
}

//==============================================================================
template <typename Scalar>
FclEngine<Scalar>* FclScene<Scalar>::get_mutable_fcl_engine()
{
  return static_cast<FclEngine<Scalar>*>(this->m_engine);
}

//==============================================================================
template <typename Scalar>
const FclEngine<Scalar>* FclScene<Scalar>::get_fcl_engine() const
{
  return static_cast<const FclEngine<Scalar>*>(this->m_engine);
}

//==============================================================================
template <typename Scalar>
typename FclScene<Scalar>::FCLCollisionManager*
FclScene<Scalar>::get_fcl_collision_manager()
{
  return m_broad_phase_alg.get();
}

//==============================================================================
template <typename Scalar>
const typename FclScene<Scalar>::FCLCollisionManager*
FclScene<Scalar>::get_fcl_collision_manager() const
{
  return m_broad_phase_alg.get();
}

} // namespace collision
} // namespace dart
