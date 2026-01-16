/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include "dart/simulation/experimental/multi_body/multi_body.hpp"

#include "dart/simulation/experimental/common/ecs_utils.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/multi_body/joint.hpp"
#include "dart/simulation/experimental/multi_body/link.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Geometry>

#include <format>

namespace dart::simulation::experimental {

//==============================================================================
MultiBody::MultiBody(entt::entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

//==============================================================================
std::string_view MultiBody::getName() const
{
  const auto& nameComp = safeGet<comps::Name>(m_world->getRegistry(), m_entity);
  return nameComp.name;
}

//==============================================================================
void MultiBody::setName(std::string_view name)
{
  auto& nameComp = safeGet<comps::Name>(m_world->getRegistry(), m_entity);
  nameComp.name = name;
}

//==============================================================================
std::size_t MultiBody::getLinkCount() const
{
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);
  return structure.links.size();
}

//==============================================================================
std::size_t MultiBody::getJointCount() const
{
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);
  return structure.joints.size();
}

//==============================================================================
std::size_t MultiBody::getDOFCount() const
{
  std::size_t dof = 0;
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);
  const auto& registry = m_world->getRegistry();

  for (const auto& jointEntity : structure.joints) {
    const auto& joint = safeGet<comps::Joint>(registry, jointEntity);
    dof += joint.getDOF();
  }

  return dof;
}

//==============================================================================
std::optional<Link> MultiBody::getLink(std::string_view name) const
{
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);
  const auto& registry = m_world->getRegistry();

  for (const auto& linkEntity : structure.links) {
    const auto& nameComp = safeGet<comps::Name>(registry, linkEntity);
    if (nameComp.name == name) {
      return Link(linkEntity, m_world);
    }
  }

  return std::nullopt;
}

//==============================================================================
std::optional<Joint> MultiBody::getJoint(std::string_view name) const
{
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);
  const auto& registry = m_world->getRegistry();

  for (const auto& jointEntity : structure.joints) {
    const auto& nameComp = safeGet<comps::Name>(registry, jointEntity);
    if (nameComp.name == name) {
      return Joint(jointEntity, m_world);
    }
  }

  return std::nullopt;
}

//==============================================================================
World* MultiBody::getWorld() const
{
  return m_world;
}

Link MultiBody::addLink(std::string_view name)
{
  // Check design mode
  DART_EXPERIMENTAL_THROW_T_IF(
      m_world->isSimulationMode(),
      InvalidArgumentException,
      "Cannot create Link in simulation mode");

  // Auto-generate name if not provided
  std::string actualName;
  if (name.empty()) {
    actualName = std::format("link_{:03d}", ++m_world->m_linkCounter);
  } else {
    actualName = std::string(name);
  }

  // Create link entity
  auto linkEntity = m_world->getRegistry().create();

  // Add Name for consistency (all named entities should have this)
  m_world->getRegistry().emplace<comps::Name>(linkEntity, actualName);

  // Add frame and link components
  auto& registry = m_world->getRegistry();
  registry.emplace<comps::FrameTag>(linkEntity);
  auto& frameState = registry.emplace<comps::FrameState>(linkEntity);
  frameState.parentFrame = entt::null;
  auto& frameCache = registry.emplace<comps::FrameCache>(linkEntity);
  frameCache.worldTransform = Eigen::Isometry3d::Identity();
  frameCache.needTransformUpdate = true;
  registry.emplace<comps::FreeFrameProperties>(linkEntity).localTransform
      = Eigen::Isometry3d::Identity();

  auto& linkComp = m_world->getRegistry().emplace<comps::Link>(linkEntity);
  linkComp.name = std::move(actualName);
  linkComp.parentJoint = entt::null;

  // Add to MultiBody structure
  auto& structure
      = m_world->getRegistry().get<comps::MultiBodyStructure>(m_entity);
  structure.links.push_back(linkEntity);

  return Link(linkEntity, m_world);
}

//==============================================================================
Link MultiBody::addLink(std::string_view name, const LinkOptions& options)
{
  // Check design mode
  DART_EXPERIMENTAL_THROW_T_IF(
      m_world->isSimulationMode(),
      InvalidArgumentException,
      "Cannot create link in simulation mode. Links must be created in "
      "design mode.");

  // Validate parent link exists
  auto& registry = m_world->getRegistry();
  auto parentEntity = options.parentLink.getEntity();

  DART_EXPERIMENTAL_THROW_T_IF(
      !registry.valid(parentEntity),
      InvalidArgumentException,
      "Parent link is invalid");

  // Auto-generate link name if not provided
  std::string actualLinkName;
  if (name.empty()) {
    actualLinkName = std::format("link_{:03d}", ++m_world->m_linkCounter);
  } else {
    actualLinkName = std::string(name);
  }

  // Auto-generate joint name if not provided
  std::string actualJointName;
  if (options.jointName.empty()) {
    actualJointName = std::format("joint_{:03d}", ++m_world->m_jointCounter);
  } else {
    actualJointName = options.jointName;
  }

  // Create link entity
  auto linkEntity = registry.create();

  // Add Name for consistency (all named entities should have this)
  registry.emplace<comps::Name>(linkEntity, actualLinkName);

  // Add frame components required by Frame interface
  registry.emplace<comps::FrameTag>(linkEntity);
  auto& frameState = registry.emplace<comps::FrameState>(linkEntity);
  frameState.parentFrame = parentEntity;
  auto& frameCache = registry.emplace<comps::FrameCache>(linkEntity);
  frameCache.worldTransform = Eigen::Isometry3d::Identity();
  frameCache.needTransformUpdate = true;
  registry.emplace<comps::FreeFrameProperties>(linkEntity).localTransform
      = Eigen::Isometry3d::Identity();

  // Add link component
  auto& linkComp = registry.emplace<comps::Link>(linkEntity);
  linkComp.name = std::move(actualLinkName);

  // Create joint entity
  auto jointEntity = registry.create();

  // Add Name (for consistency with other named entities)
  registry.emplace<comps::Name>(jointEntity, actualJointName);

  // Add Joint with type-specific configuration
  auto& jointComp = registry.emplace<comps::Joint>(jointEntity);
  jointComp.type = options.jointType;
  jointComp.name = std::move(actualJointName);

  const double axisNorm = options.axis.norm();
  DART_EXPERIMENTAL_THROW_T_IF(
      axisNorm <= 1e-9,
      InvalidArgumentException,
      "Joint axis must be non-zero");
  jointComp.axis = options.axis / axisNorm;
  jointComp.parentLink = parentEntity;
  jointComp.childLink = linkEntity;

  // Initialize state based on joint type
  auto dof = jointComp.getDOF();
  jointComp.position = Eigen::VectorXd::Zero(dof);
  jointComp.velocity = Eigen::VectorXd::Zero(dof);
  jointComp.acceleration = Eigen::VectorXd::Zero(dof);
  jointComp.torque = Eigen::VectorXd::Zero(dof);

  // Link the parent link to this joint
  auto& parentLinkComp = safeGet<comps::Link>(registry, parentEntity);
  parentLinkComp.childJoints.push_back(jointEntity);

  // Link this link to the parent joint
  linkComp.parentJoint = jointEntity;

  // Add to MultiBody structure
  auto& structure = safeGet<comps::MultiBodyStructure>(registry, m_entity);
  structure.links.push_back(linkEntity);
  structure.joints.push_back(jointEntity);

  return Link(linkEntity, m_world);
}

} // namespace dart::simulation::experimental
