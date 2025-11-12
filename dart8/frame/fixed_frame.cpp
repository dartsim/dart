/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart8/frame/fixed_frame.hpp"

#include "dart8/common/exceptions.hpp"
#include "dart8/comps/all.hpp"
#include "dart8/world.hpp"

namespace dart8 {

//==============================================================================
FixedFrame::FixedFrame(entt::entity entity, World* world)
  : Frame(entity, world),
    EntityObjectWith<
        TagComps<comps::FixedFrameTag>,
        ReadOnlyComps<>,
        WriteOnlyComps<>,
        ReadWriteComps<comps::FixedFrameProperties>>()
{
  // Validate entity has FixedFrameProperties
#ifndef NDEBUG
  auto& registry = world->getRegistry();
  DART8_THROW_T_IF(
      !registry.all_of<comps::FixedFrameProperties>(entity),
      InvalidArgumentException,
      "Entity does not have FixedFrameProperties component");
#endif
}

//==============================================================================
void FixedFrame::setLocalTransform(const Eigen::Isometry3d& transform)
{
  auto& registry = m_world->getRegistry();
  auto& props = registry.get<comps::FixedFrameProperties>(m_entity);
  props.localTransform = transform;

  // Invalidate cache
  auto& cache = registry.get<comps::FrameCache>(m_entity);
  cache.needTransformUpdate = true;
}

//==============================================================================
const Eigen::Isometry3d& FixedFrame::getLocalTransform() const
{
  const auto& properties
      = m_world->getRegistry().get<comps::FixedFrameProperties>(m_entity);
  return properties.localTransform;
}

} // namespace dart8
