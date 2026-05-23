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

#include "dart/simulation/experimental/multi_body/multi_body.hpp"

#include "dart/simulation/experimental/common/ecs_utils.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/multi_body/joint.hpp"
#include "dart/simulation/experimental/multi_body/link.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <format>

namespace dart::simulation::experimental {

namespace {

template <typename Component>
bool hasOtherEntityWithName(
    const entt::registry& registry,
    entt::entity excludedEntity,
    std::string_view name)
{
  auto view = registry.view<Component, comps::Name>();
  for (auto entity : view) {
    if (entity == excludedEntity) {
      continue;
    }

    const auto& nameComp = view.template get<comps::Name>(entity);
    if (nameComp.name == name) {
      return true;
    }
  }

  return false;
}

bool containsEntity(
    const std::vector<entt::entity>& entities, entt::entity candidate)
{
  return std::find(entities.begin(), entities.end(), candidate)
         != entities.end();
}

bool containsName(
    const entt::registry& registry,
    const std::vector<entt::entity>& entities,
    std::string_view name)
{
  for (const auto& entity : entities) {
    if (safeGet<comps::Name>(registry, entity).name == name) {
      return true;
    }
  }

  return false;
}

comps::JointType toComponentJointType(JointType type)
{
  switch (type) {
    case JointType::Fixed:
      return comps::JointType::Fixed;
    case JointType::Revolute:
      return comps::JointType::Revolute;
    case JointType::Prismatic:
      return comps::JointType::Prismatic;
    case JointType::Screw:
      return comps::JointType::Screw;
    case JointType::Universal:
      return comps::JointType::Universal;
    case JointType::Ball:
      return comps::JointType::Ball;
    case JointType::Planar:
      return comps::JointType::Planar;
    case JointType::Free:
      return comps::JointType::Free;
    case JointType::Custom:
      return comps::JointType::Custom;
  }

  return comps::JointType::Custom;
}

} // namespace

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
  DART_EXPERIMENTAL_THROW_T_IF(
      name.empty(), InvalidArgumentException, "MultiBody name cannot be empty");

  DART_EXPERIMENTAL_THROW_T_IF(
      hasOtherEntityWithName<comps::MultiBodyTag>(
          m_world->getRegistry(), m_entity, name),
      InvalidArgumentException,
      "MultiBody '{}' already exists",
      name);

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
std::vector<Link> MultiBody::getLinks() const
{
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);

  std::vector<Link> links;
  links.reserve(structure.links.size());
  for (const auto& linkEntity : structure.links) {
    links.emplace_back(linkEntity, m_world);
  }

  return links;
}

//==============================================================================
std::vector<Joint> MultiBody::getJoints() const
{
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);

  std::vector<Joint> joints;
  joints.reserve(structure.joints.size());
  for (const auto& jointEntity : structure.joints) {
    joints.emplace_back(jointEntity, m_world);
  }

  return joints;
}

//==============================================================================
std::vector<std::string> MultiBody::getLinkNames() const
{
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);
  const auto& registry = m_world->getRegistry();

  std::vector<std::string> names;
  names.reserve(structure.links.size());
  for (const auto& linkEntity : structure.links) {
    names.push_back(safeGet<comps::Name>(registry, linkEntity).name);
  }

  return names;
}

//==============================================================================
std::vector<std::string> MultiBody::getJointNames() const
{
  const auto& structure
      = safeGet<comps::MultiBodyStructure>(m_world->getRegistry(), m_entity);
  const auto& registry = m_world->getRegistry();

  std::vector<std::string> names;
  names.reserve(structure.joints.size());
  for (const auto& jointEntity : structure.joints) {
    names.push_back(safeGet<comps::Name>(registry, jointEntity).name);
  }

  return names;
}

//==============================================================================
entt::entity MultiBody::getEntity() const
{
  return m_entity;
}

//==============================================================================
World* MultiBody::getWorld() const
{
  return m_world;
}

//==============================================================================
bool MultiBody::isValid() const
{
  return m_world != nullptr && m_world->getRegistry().valid(m_entity)
         && m_world->getRegistry().all_of<comps::MultiBodyTag>(m_entity);
}

Link MultiBody::addLink(std::string_view name)
{
  // Check design mode
  DART_EXPERIMENTAL_THROW_T_IF(
      m_world->isSimulationMode(),
      InvalidArgumentException,
      "Cannot create Link in simulation mode");

  auto& registry = m_world->getRegistry();
  auto& structure = safeGet<comps::MultiBodyStructure>(registry, m_entity);

  // Auto-generate name if not provided
  std::string actualName;
  if (name.empty()) {
    do {
      actualName = std::format("link_{:03d}", ++m_world->m_linkCounter);
    } while (containsName(registry, structure.links, actualName));
  } else {
    actualName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        containsName(registry, structure.links, actualName),
        InvalidArgumentException,
        "Link '{}' already exists in MultiBody '{}'",
        actualName,
        getName());
  }

  // Create link entity
  auto linkEntity = registry.create();

  // Add Name for consistency (all named entities should have this)
  registry.emplace<comps::Name>(linkEntity, actualName);

  // Add frame and link components
  registry.emplace<comps::FrameTag>(linkEntity);
  auto& frameState = registry.emplace<comps::FrameState>(linkEntity);
  frameState.parentFrame = entt::null;
  auto& frameCache = registry.emplace<comps::FrameCache>(linkEntity);
  frameCache.worldTransform = Eigen::Isometry3d::Identity();
  frameCache.needTransformUpdate = true;
  registry.emplace<comps::FreeFrameProperties>(linkEntity).localTransform
      = Eigen::Isometry3d::Identity();

  auto& linkComp = registry.emplace<comps::Link>(linkEntity);
  linkComp.name = std::move(actualName);
  linkComp.parentJoint = entt::null;

  // Add to MultiBody structure
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
      options.parentLink.getWorld() != m_world,
      InvalidArgumentException,
      "Parent link belongs to a different world");

  DART_EXPERIMENTAL_THROW_T_IF(
      !registry.valid(parentEntity),
      InvalidArgumentException,
      "Parent link is invalid");

  auto& structure = safeGet<comps::MultiBodyStructure>(registry, m_entity);
  DART_EXPERIMENTAL_THROW_T_IF(
      !containsEntity(structure.links, parentEntity),
      InvalidArgumentException,
      "Parent link does not belong to MultiBody '{}'",
      getName());

  // Auto-generate link name if not provided
  std::string actualLinkName;
  if (name.empty()) {
    do {
      actualLinkName = std::format("link_{:03d}", ++m_world->m_linkCounter);
    } while (containsName(registry, structure.links, actualLinkName));
  } else {
    actualLinkName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        containsName(registry, structure.links, actualLinkName),
        InvalidArgumentException,
        "Link '{}' already exists in MultiBody '{}'",
        actualLinkName,
        getName());
  }

  // Auto-generate joint name if not provided
  std::string actualJointName;
  if (options.jointName.empty()) {
    do {
      actualJointName = std::format("joint_{:03d}", ++m_world->m_jointCounter);
    } while (containsName(registry, structure.joints, actualJointName));
  } else {
    actualJointName = options.jointName;
    DART_EXPERIMENTAL_THROW_T_IF(
        containsName(registry, structure.joints, actualJointName),
        InvalidArgumentException,
        "Joint '{}' already exists in MultiBody '{}'",
        actualJointName,
        getName());
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
  jointComp.type = toComponentJointType(options.jointType);
  jointComp.name = std::move(actualJointName);

  DART_EXPERIMENTAL_THROW_T_IF(
      !options.axis.allFinite(),
      InvalidArgumentException,
      "Joint axis must contain only finite values");
  const double axisNorm = options.axis.norm();
  DART_EXPERIMENTAL_THROW_T_IF(
      axisNorm <= 1e-9,
      InvalidArgumentException,
      "Joint axis must be non-zero");
  jointComp.axis = options.axis / axisNorm;

  // Planar and universal joints build a second in-plane/rotation direction from
  // axis2. If axis is parallel to axis2 the derived basis collapses (its cross
  // product is zero), producing degenerate transforms during sync/step, so
  // reject it at construction time.
  if (jointComp.type == comps::JointType::Planar
      || jointComp.type == comps::JointType::Universal) {
    DART_EXPERIMENTAL_THROW_T_IF(
        jointComp.axis.cross(jointComp.axis2).norm() <= 1e-6,
        InvalidArgumentException,
        "Joint axis must not be parallel to axis2 for planar and universal "
        "joints");
  }

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
  structure.links.push_back(linkEntity);
  structure.joints.push_back(jointEntity);

  return Link(linkEntity, m_world);
}

//==============================================================================
Link MultiBody::addLink(
    std::string_view name, const Link& parentLink, const JointSpec& joint)
{
  return addLink(
      name,
      LinkOptions{
          .parentLink = parentLink,
          .jointName = joint.name,
          .jointType = joint.type,
          .axis = joint.axis,
      });
}

} // namespace dart::simulation::experimental
