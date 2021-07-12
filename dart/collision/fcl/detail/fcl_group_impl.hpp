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
#include "dart/collision/fcl/fcl_group.hpp"
#include "dart/collision/fcl/fcl_object.hpp"
#include "dart/collision/object.hpp"
#include "dart/common/Console.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
std::shared_ptr<FclCollisionGeometry<S>> create_fcl_collision_geometry(
    const math::ConstGeometryPtr& shape,
    typename FclEngine<S>::PrimitiveShape /*type*/) {
  FclCollisionGeometry<S>* geom = nullptr;

  if (auto sphere = shape->as<math::Sphered>()) {
    geom = new FclSphere<S>(sphere->get_radius());
  } else {
    dterr << "Unsupported geometry type: " << shape->get_type() << "\n";
  }

  return std::shared_ptr<FclCollisionGeometry<S>>(geom);
}

//==============================================================================
template <typename S>
FclGroup<S>::FclGroup(Engine<S>* engine)
  : Group<S>(engine),
    m_broad_phase_alg(new FclDynamicAABBTreeCollisionManager<S>()) {
  // Do nothing
}

//==============================================================================
template <typename S>
ObjectPtr<S> FclGroup<S>::create_object(math::GeometryPtr shape) {
  if (!shape) {
    DART_WARN("Not allowed to create a collision object for a null shape");
    return nullptr;
  }

  auto fcl_collision_geometry
      = create_fcl_collision_geometry<S>(shape, FclEngine<S>::MESH);
  if (!fcl_collision_geometry) {
    DART_WARN("Failed to create FCL collision geometry.");
    return nullptr;
  }

  return std::shared_ptr<FclObject<S>>(
      new FclObject<S>(this, std::move(shape), fcl_collision_geometry));
}

//==============================================================================
template <typename S>
FclEngine<S>* FclGroup<S>::get_mutable_fcl_engine() {
  return static_cast<FclEngine<S>*>(this->m_engine);
}

//==============================================================================
template <typename S>
const FclEngine<S>* FclGroup<S>::get_fcl_engine() const {
  return static_cast<const FclEngine<S>*>(this->m_engine);
}

//==============================================================================
template <typename S>
typename FclGroup<S>::FCLCollisionManager*
FclGroup<S>::get_fcl_collision_manager() {
  return m_broad_phase_alg.get();
}

//==============================================================================
template <typename S>
const typename FclGroup<S>::FCLCollisionManager*
FclGroup<S>::get_fcl_collision_manager() const {
  return m_broad_phase_alg.get();
}

} // namespace collision
} // namespace dart
