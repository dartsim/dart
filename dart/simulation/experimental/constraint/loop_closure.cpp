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

#include "dart/simulation/experimental/constraint/loop_closure.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/loop_closure.hpp"
#include "dart/simulation/experimental/comps/name.hpp"
#include "dart/simulation/experimental/frame/frame.hpp"
#include "dart/simulation/experimental/world.hpp"

namespace dart::simulation::experimental {

namespace {

const comps::LoopClosure& getLoopClosureComponent(const LoopClosure& closure)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !closure.isValid(),
      InvalidArgumentException,
      "Invalid loop closure handle");

  return closure.getWorld()->getRegistry().get<comps::LoopClosure>(
      closure.getEntity());
}

} // namespace

//==============================================================================
LoopClosure::LoopClosure(entt::entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

//==============================================================================
std::string_view LoopClosure::getName() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid loop closure handle");

  const auto& name = m_world->getRegistry().get<comps::Name>(m_entity);
  return name.name;
}

//==============================================================================
LoopClosureFamily LoopClosure::getFamily() const
{
  return getLoopClosureComponent(*this).family;
}

//==============================================================================
Frame LoopClosure::getFrameA() const
{
  return Frame(getLoopClosureComponent(*this).frameA, m_world);
}

//==============================================================================
Frame LoopClosure::getFrameB() const
{
  return Frame(getLoopClosureComponent(*this).frameB, m_world);
}

//==============================================================================
const Eigen::Isometry3d& LoopClosure::getOffsetA() const
{
  return getLoopClosureComponent(*this).offsetA;
}

//==============================================================================
const Eigen::Isometry3d& LoopClosure::getOffsetB() const
{
  return getLoopClosureComponent(*this).offsetB;
}

//==============================================================================
entt::entity LoopClosure::getEntity() const
{
  return m_entity;
}

//==============================================================================
World* LoopClosure::getWorld() const
{
  return m_world;
}

//==============================================================================
bool LoopClosure::isValid() const
{
  return m_world != nullptr && m_world->getRegistry().valid(m_entity)
         && m_world->getRegistry().all_of<comps::LoopClosure>(m_entity);
}

} // namespace dart::simulation::experimental
