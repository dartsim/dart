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

#include "dart/simulation/experimental/multibody/multibody.hpp"

#include "dart/simulation/experimental/common/ecs_utils.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/compute/multibody_dynamics.hpp"
#include "dart/simulation/experimental/multibody/joint.hpp"
#include "dart/simulation/experimental/multibody/link.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <format>
#include <limits>

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
    case JointType::Spherical:
      return comps::JointType::Spherical;
    case JointType::Planar:
      return comps::JointType::Planar;
    case JointType::Floating:
      return comps::JointType::Floating;
    case JointType::Custom:
      return comps::JointType::Custom;
  }

  return comps::JointType::Custom;
}

} // namespace

//==============================================================================
Multibody::Multibody(entt::entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

//==============================================================================
std::string_view Multibody::getName() const
{
  const auto& nameComp = safeGet<comps::Name>(m_world->getRegistry(), m_entity);
  return nameComp.name;
}

//==============================================================================
void Multibody::setName(std::string_view name)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      name.empty(), InvalidArgumentException, "Multibody name cannot be empty");

  DART_EXPERIMENTAL_THROW_T_IF(
      hasOtherEntityWithName<comps::MultibodyTag>(
          m_world->getRegistry(), m_entity, name),
      InvalidArgumentException,
      "Multibody '{}' already exists",
      name);

  auto& nameComp = safeGet<comps::Name>(m_world->getRegistry(), m_entity);
  nameComp.name = name;
}

//==============================================================================
std::size_t Multibody::getLinkCount() const
{
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);
  return structure.links.size();
}

//==============================================================================
std::size_t Multibody::getJointCount() const
{
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);
  return structure.joints.size();
}

//==============================================================================
std::size_t Multibody::getDOFCount() const
{
  std::size_t dof = 0;
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);
  const auto& registry = m_world->getRegistry();

  for (const auto& jointEntity : structure.joints) {
    const auto& joint = safeGet<comps::Joint>(registry, jointEntity);
    dof += joint.getDOF();
  }

  return dof;
}

//==============================================================================
std::optional<Link> Multibody::getLink(std::string_view name) const
{
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);
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
std::optional<Joint> Multibody::getJoint(std::string_view name) const
{
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);
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
std::vector<Link> Multibody::getLinks() const
{
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);

  std::vector<Link> links;
  links.reserve(structure.links.size());
  for (const auto& linkEntity : structure.links) {
    links.emplace_back(linkEntity, m_world);
  }

  return links;
}

//==============================================================================
std::vector<Joint> Multibody::getJoints() const
{
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);

  std::vector<Joint> joints;
  joints.reserve(structure.joints.size());
  for (const auto& jointEntity : structure.joints) {
    joints.emplace_back(jointEntity, m_world);
  }

  return joints;
}

//==============================================================================
std::vector<std::string> Multibody::getLinkNames() const
{
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);
  const auto& registry = m_world->getRegistry();

  std::vector<std::string> names;
  names.reserve(structure.links.size());
  for (const auto& linkEntity : structure.links) {
    names.push_back(safeGet<comps::Name>(registry, linkEntity).name);
  }

  return names;
}

//==============================================================================
std::vector<std::string> Multibody::getJointNames() const
{
  const auto& structure
      = safeGet<comps::MultibodyStructure>(m_world->getRegistry(), m_entity);
  const auto& registry = m_world->getRegistry();

  std::vector<std::string> names;
  names.reserve(structure.joints.size());
  for (const auto& jointEntity : structure.joints) {
    names.push_back(safeGet<comps::Name>(registry, jointEntity).name);
  }

  return names;
}

//==============================================================================
entt::entity Multibody::getEntity() const
{
  return m_entity;
}

//==============================================================================
World* Multibody::getWorld() const
{
  return m_world;
}

//==============================================================================
bool Multibody::isValid() const
{
  return m_world != nullptr && m_world->getRegistry().valid(m_entity)
         && m_world->getRegistry().all_of<comps::MultibodyTag>(m_entity);
}

Link Multibody::addLink(std::string_view name)
{
  // Check design mode
  DART_EXPERIMENTAL_THROW_T_IF(
      m_world->isSimulationMode(),
      InvalidArgumentException,
      "Cannot create Link in simulation mode");

  auto& registry = m_world->getRegistry();
  auto& structure = safeGet<comps::MultibodyStructure>(registry, m_entity);

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
        "Link '{}' already exists in Multibody '{}'",
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

  // Add to Multibody structure
  structure.links.push_back(linkEntity);

  return Link(linkEntity, m_world);
}

//==============================================================================
Link Multibody::addLink(std::string_view name, const LinkOptions& options)
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

  auto& structure = safeGet<comps::MultibodyStructure>(registry, m_entity);
  DART_EXPERIMENTAL_THROW_T_IF(
      !containsEntity(structure.links, parentEntity),
      InvalidArgumentException,
      "Parent link does not belong to Multibody '{}'",
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
        "Link '{}' already exists in Multibody '{}'",
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
        "Joint '{}' already exists in Multibody '{}'",
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

  DART_EXPERIMENTAL_THROW_T_IF(
      !options.transformFromParent.matrix().allFinite(),
      InvalidArgumentException,
      "Link transform from parent must contain only finite values");
  linkComp.transformFromParentJoint = options.transformFromParent;

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

  DART_EXPERIMENTAL_THROW_T_IF(
      !options.axis2.allFinite(),
      InvalidArgumentException,
      "Joint axis2 must contain only finite values");
  const double axis2Norm = options.axis2.norm();
  DART_EXPERIMENTAL_THROW_T_IF(
      axis2Norm <= 1e-9,
      InvalidArgumentException,
      "Joint axis2 must be non-zero");
  jointComp.axis2 = options.axis2 / axis2Norm;

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
  jointComp.springStiffness = Eigen::VectorXd::Zero(dof);
  jointComp.dampingCoefficient = Eigen::VectorXd::Zero(dof);
  jointComp.restPosition = Eigen::VectorXd::Zero(dof);
  jointComp.armature = Eigen::VectorXd::Zero(dof);
  jointComp.coulombFriction = Eigen::VectorXd::Zero(dof);
  jointComp.commandVelocity = Eigen::VectorXd::Zero(dof);

  // Position, velocity, and effort limits default to unbounded.
  const double infinity = std::numeric_limits<double>::infinity();
  jointComp.limits.lower = Eigen::VectorXd::Constant(dof, -infinity);
  jointComp.limits.upper = Eigen::VectorXd::Constant(dof, infinity);
  jointComp.limits.velocityLower = Eigen::VectorXd::Constant(dof, -infinity);
  jointComp.limits.velocityUpper = Eigen::VectorXd::Constant(dof, infinity);
  jointComp.limits.effortLower = Eigen::VectorXd::Constant(dof, -infinity);
  jointComp.limits.effortUpper = Eigen::VectorXd::Constant(dof, infinity);

  // Link the parent link to this joint
  auto& parentLinkComp = safeGet<comps::Link>(registry, parentEntity);
  parentLinkComp.childJoints.push_back(jointEntity);

  // Link this link to the parent joint
  linkComp.parentJoint = jointEntity;

  // Add to Multibody structure
  structure.links.push_back(linkEntity);
  structure.joints.push_back(jointEntity);

  return Link(linkEntity, m_world);
}

//==============================================================================
Link Multibody::addLink(
    std::string_view name, const Link& parentLink, const JointSpec& joint)
{
  return addLink(
      name,
      LinkOptions{
          .parentLink = parentLink,
          .jointName = joint.name,
          .jointType = joint.type,
          .axis = joint.axis,
          .axis2 = joint.axis2,
          .transformFromParent = joint.transformFromParent,
      });
}

//==============================================================================
Eigen::MatrixXd Multibody::getMassMatrix() const
{
  auto& registry = m_world->getRegistry();
  const auto& structure
      = safeGet<comps::MultibodyStructure>(registry, m_entity);
  return compute::computeMultibodyDynamicsTerms(
             registry, structure, m_world->getGravity())
      .massMatrix;
}

//==============================================================================
Eigen::MatrixXd Multibody::getInverseMassMatrix() const
{
  const Eigen::MatrixXd mass = getMassMatrix();
  if (mass.size() == 0) {
    return mass;
  }
  return mass.ldlt().solve(Eigen::MatrixXd::Identity(mass.rows(), mass.cols()));
}

//==============================================================================
Eigen::VectorXd Multibody::getCoriolisForces() const
{
  auto& registry = m_world->getRegistry();
  const auto& structure
      = safeGet<comps::MultibodyStructure>(registry, m_entity);
  return compute::computeMultibodyDynamicsTerms(
             registry, structure, m_world->getGravity())
      .coriolisForces;
}

//==============================================================================
Eigen::VectorXd Multibody::getGravityForces() const
{
  auto& registry = m_world->getRegistry();
  const auto& structure
      = safeGet<comps::MultibodyStructure>(registry, m_entity);
  return compute::computeMultibodyDynamicsTerms(
             registry, structure, m_world->getGravity())
      .gravityForces;
}

//==============================================================================
Eigen::VectorXd Multibody::getCoriolisAndGravityForces() const
{
  auto& registry = m_world->getRegistry();
  const auto& structure
      = safeGet<comps::MultibodyStructure>(registry, m_entity);
  const compute::MultibodyDynamicsTerms terms
      = compute::computeMultibodyDynamicsTerms(
          registry, structure, m_world->getGravity());
  return terms.coriolisForces + terms.gravityForces;
}

//==============================================================================
Eigen::VectorXd Multibody::computeInverseDynamics(
    const Eigen::VectorXd& desiredAcceleration) const
{
  auto& registry = m_world->getRegistry();
  const auto& structure
      = safeGet<comps::MultibodyStructure>(registry, m_entity);
  return compute::computeMultibodyInverseDynamics(
      registry, structure, m_world->getGravity(), desiredAcceleration);
}

//==============================================================================
Eigen::VectorXd Multibody::computeImpulseResponse(
    const Eigen::VectorXd& jointImpulse) const
{
  const Eigen::MatrixXd massMatrix = getMassMatrix();
  if (massMatrix.size() == 0) {
    return {};
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      jointImpulse.size() != massMatrix.rows(),
      InvalidArgumentException,
      "Joint impulse dimension ({}) must match the multibody DOF count ({})",
      jointImpulse.size(),
      massMatrix.rows());

  return massMatrix.ldlt().solve(jointImpulse);
}

//==============================================================================
Eigen::MatrixXd Multibody::getJacobian(const Link& link) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      link.getWorld() != m_world,
      InvalidArgumentException,
      "Link belongs to a different world");

  auto& registry = m_world->getRegistry();
  const auto& structure
      = safeGet<comps::MultibodyStructure>(registry, m_entity);
  return compute::computeMultibodyLinkJacobian(
      registry, structure, link.getEntity());
}

//==============================================================================
Eigen::MatrixXd Multibody::getWorldJacobian(const Link& link) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      link.getWorld() != m_world,
      InvalidArgumentException,
      "Link belongs to a different world");

  auto& registry = m_world->getRegistry();
  const auto& structure
      = safeGet<comps::MultibodyStructure>(registry, m_entity);
  return compute::computeMultibodyLinkWorldJacobian(
      registry, structure, link.getEntity());
}

//==============================================================================
void Multibody::setGroundContact(
    const Eigen::Vector3d& planeNormal,
    const Eigen::Vector3d& planePoint,
    double stiffness,
    double frictionCoefficient,
    double frictionRegularization,
    double dampingCoefficient)
{
  auto& registry = m_world->getRegistry();
  auto& contact
      = registry.emplace_or_replace<comps::VariationalContact>(m_entity);
  contact.planeNormal = planeNormal;
  contact.planePoint = planePoint;
  contact.stiffness = stiffness;
  contact.frictionCoefficient = frictionCoefficient;
  contact.frictionRegularization = frictionRegularization;
  contact.dampingCoefficient = dampingCoefficient;
}

//==============================================================================
void Multibody::addGroundContactPoint(
    const Link& link, const Eigen::Vector3d& localPoint)
{
  auto& registry = m_world->getRegistry();
  auto* contact = registry.try_get<comps::VariationalContact>(m_entity);
  DART_EXPERIMENTAL_THROW_T_IF(
      contact == nullptr,
      InvalidArgumentException,
      "addGroundContactPoint requires setGroundContact() to be called first");
  const auto& structure
      = safeGet<comps::MultibodyStructure>(registry, m_entity);
  const auto it = std::find(
      structure.links.begin(), structure.links.end(), link.getEntity());
  DART_EXPERIMENTAL_THROW_T_IF(
      it == structure.links.end(),
      InvalidArgumentException,
      "addGroundContactPoint: link is not part of this multibody");
  contact->pointLinkIndices.push_back(
      static_cast<std::size_t>(it - structure.links.begin()));
  contact->pointLocalPositions.push_back(localPoint);
}

} // namespace dart::simulation::experimental
