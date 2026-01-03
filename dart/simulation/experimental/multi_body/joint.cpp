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

#include "dart/simulation/experimental/multi_body/joint.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/multi_body/link.hpp"
#include "dart/simulation/experimental/world.hpp"

namespace dart::simulation::experimental {

//==============================================================================
Joint::Joint(entt::entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

//==============================================================================
std::string_view Joint::getName() const
{
  const auto& jointComp = m_world->getRegistry().get<comps::Joint>(m_entity);
  return jointComp.name;
}

//==============================================================================
comps::JointType Joint::getType() const
{
  const auto& jointComp = m_world->getRegistry().get<comps::Joint>(m_entity);
  return jointComp.type;
}

//==============================================================================
Eigen::Vector3d Joint::getAxis() const
{
  const auto& jointComp = m_world->getRegistry().get<comps::Joint>(m_entity);

  // Validate joint type - Ball and Free joints don't have axes
  DART_EXPERIMENTAL_THROW_T_IF(
      jointComp.type == comps::JointType::Ball
          || jointComp.type == comps::JointType::Free,
      InvalidArgumentException,
      "getAxis() is not valid for Ball and Free joints (no axis concept)");

  return jointComp.axis;
}

//==============================================================================
Eigen::Vector3d Joint::getAxis2() const
{
  const auto& jointComp = m_world->getRegistry().get<comps::Joint>(m_entity);

  // Validate joint type
  DART_EXPERIMENTAL_THROW_T_IF(
      jointComp.type != comps::JointType::Universal
          && jointComp.type != comps::JointType::Planar,
      InvalidArgumentException,
      "getAxis2() is only valid for Universal and Planar joints");

  return jointComp.axis2;
}

//==============================================================================
double Joint::getPitch() const
{
  const auto& jointComp = m_world->getRegistry().get<comps::Joint>(m_entity);

  // Validate joint type
  DART_EXPERIMENTAL_THROW_T_IF(
      jointComp.type != comps::JointType::Screw,
      InvalidArgumentException,
      "getPitch() is only valid for Screw joints");

  return jointComp.pitch;
}

//==============================================================================
Link Joint::getParentLink() const
{
  const auto& jointComp = m_world->getRegistry().get<comps::Joint>(m_entity);
  return Link(jointComp.parentLink, m_world);
}

//==============================================================================
Link Joint::getChildLink() const
{
  const auto& jointComp = m_world->getRegistry().get<comps::Joint>(m_entity);
  return Link(jointComp.childLink, m_world);
}

//==============================================================================
entt::entity Joint::getEntity() const
{
  return m_entity;
}

//==============================================================================
bool Joint::isValid() const
{
  return m_world->getRegistry().valid(m_entity);
}

} // namespace dart::simulation::experimental
