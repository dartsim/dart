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

#include "dart/collision/object.hpp"
#include "dart/collision/ode/ode_engine.hpp"
#include "dart/collision/ode/ode_group.hpp"
#include "dart/collision/ode/ode_object.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
OdeGroup<S>::OdeGroup(Engine<S>* engine) : Group<S>(engine) {
  // This uses an internal data structure that records how each geom overlaps
  // cells in one of several three dimensional grids. Each grid has cubical
  // cells of side lengths 2i, where i is an integer that ranges from a minimum
  // to a maximum value. The time required to do intersection testing for n
  // objects is O(n) (as long as those objects are not clustered together too
  // closely), as each object can be quickly paired with the objects around it.
  //
  // Source:
  // https://www.ode-wiki.org/wiki/index.php?title=Manual:_Collision_Detection#Space_functions
  m_ode_space_id = dHashSpaceCreate(nullptr);
  DART_ASSERT(m_ode_space_id);
  dHashSpaceSetLevels(m_ode_space_id, -2, 8);
}

//==============================================================================
template <typename S>
OdeGroup<S>::~OdeGroup() {
  dSpaceDestroy(m_ode_space_id);
}

//==============================================================================
template <typename S>
ObjectPtr<S> OdeGroup<S>::create_object(math::GeometryPtr shape) {
  if (!shape) {
    DART_WARN("Not allowed to create a collision object for a null shape");
    return nullptr;
  }

  return std::shared_ptr<OdeObject<S>>(
      new OdeObject<S>(this, std::move(shape)));
}

//==============================================================================
template <typename S>
OdeEngine<S>* OdeGroup<S>::get_mutable_ode_engine() {
  return static_cast<OdeEngine<S>*>(this->m_engine);
}

//==============================================================================
template <typename S>
const OdeEngine<S>* OdeGroup<S>::get_ode_engine() const {
  return static_cast<const OdeEngine<S>*>(this->m_engine);
}

//==============================================================================
template <typename S_>
dSpaceID OdeGroup<S_>::get_ode_space_id() const {
  return m_ode_space_id;
}

} // namespace collision
} // namespace dart
