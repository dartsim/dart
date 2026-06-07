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

#include "dart/simulation/multibody/multibody.hpp"

#include "dart/simulation/common/ecs_utils.hpp"
#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/all.hpp"
#include "dart/simulation/compute/multibody_dynamics.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/multibody/joint.hpp"
#include "dart/simulation/multibody/link.hpp"
#include "dart/simulation/world.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <format>
#include <limits>

namespace dart::simulation {

namespace {

comps::VariationalContact makeVariationalContact(
    common::MemoryAllocator& allocator)
{
  comps::VariationalContact contact;
  contact.pointLinkIndices = comps::VariationalContact::LinkIndexVector{
      common::StlAllocator<std::size_t>{allocator}};
  contact.pointLocalPositions = comps::VariationalContact::PointVector{
      common::StlAllocator<Eigen::Vector3d>{allocator}};
  return contact;
}

void ensureVariationalContactAllocator(
    comps::VariationalContact& contact, common::MemoryAllocator& allocator)
{
  const common::StlAllocator<std::size_t> targetLinkAllocator{allocator};
  if (contact.pointLinkIndices.get_allocator() != targetLinkAllocator) {
    comps::VariationalContact::LinkIndexVector linkIndices{targetLinkAllocator};
    linkIndices.assign(
        contact.pointLinkIndices.begin(), contact.pointLinkIndices.end());
    contact.pointLinkIndices = std::move(linkIndices);
  }

  const common::StlAllocator<Eigen::Vector3d> targetPointAllocator{allocator};
  if (contact.pointLocalPositions.get_allocator() != targetPointAllocator) {
    comps::VariationalContact::PointVector localPositions{targetPointAllocator};
    localPositions.assign(
        contact.pointLocalPositions.begin(), contact.pointLocalPositions.end());
    contact.pointLocalPositions = std::move(localPositions);
  }
}

template <typename Component, typename Registry>
bool hasOtherEntityWithName(
    const Registry& registry,
    entt::entity excludedEntity,
    std::string_view name)
{
  auto view = registry.template view<Component, comps::Name>();
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

template <typename EntityList>
bool containsEntity(const EntityList& entities, entt::entity candidate)
{
  return std::find(entities.begin(), entities.end(), candidate)
         != entities.end();
}

template <typename Registry, typename EntityList>
bool containsName(
    const Registry& registry, const EntityList& entities, std::string_view name)
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
Multibody::Multibody(Entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

//==============================================================================
std::string_view Multibody::getName() const
{
  const auto& nameComp = safeGet<comps::Name>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  return nameComp.name;
}

//==============================================================================
void Multibody::setName(std::string_view name)
{
  DART_SIMULATION_THROW_T_IF(
      name.empty(), InvalidArgumentException, "Multibody name cannot be empty");

  DART_SIMULATION_THROW_T_IF(
      hasOtherEntityWithName<comps::MultibodyTag>(
          dart::simulation::detail::registryOf(*m_world),
          detail::toRegistryEntity(m_entity),
          name),
      InvalidArgumentException,
      "Multibody '{}' already exists",
      name);

  auto& nameComp = safeGet<comps::Name>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  nameComp.name = name;
}

//==============================================================================
std::size_t Multibody::getLinkCount() const
{
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  return structure.links.size();
}

//==============================================================================
std::size_t Multibody::getJointCount() const
{
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  return structure.joints.size();
}

//==============================================================================
std::size_t Multibody::getDOFCount() const
{
  std::size_t dof = 0;
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  const auto& registry = dart::simulation::detail::registryOf(*m_world);

  for (const auto& jointEntity : structure.joints) {
    const auto& jointModel = safeGet<comps::JointModel>(registry, jointEntity);
    dof += jointModel.getDOF();
  }

  return dof;
}

//==============================================================================
std::optional<Link> Multibody::getLink(std::string_view name) const
{
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  const auto& registry = dart::simulation::detail::registryOf(*m_world);

  for (const auto& linkEntity : structure.links) {
    const auto& nameComp = safeGet<comps::Name>(registry, linkEntity);
    if (nameComp.name == name) {
      return Link(detail::fromRegistryEntity(linkEntity), m_world);
    }
  }

  return std::nullopt;
}

//==============================================================================
std::optional<Joint> Multibody::getJoint(std::string_view name) const
{
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  const auto& registry = dart::simulation::detail::registryOf(*m_world);

  for (const auto& jointEntity : structure.joints) {
    const auto& nameComp = safeGet<comps::Name>(registry, jointEntity);
    if (nameComp.name == name) {
      return Joint(detail::fromRegistryEntity(jointEntity), m_world);
    }
  }

  return std::nullopt;
}

//==============================================================================
std::vector<Link> Multibody::getLinks() const
{
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));

  std::vector<Link> links;
  links.reserve(structure.links.size());
  for (const auto& linkEntity : structure.links) {
    links.emplace_back(detail::fromRegistryEntity(linkEntity), m_world);
  }

  return links;
}

//==============================================================================
std::vector<Joint> Multibody::getJoints() const
{
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));

  std::vector<Joint> joints;
  joints.reserve(structure.joints.size());
  for (const auto& jointEntity : structure.joints) {
    joints.emplace_back(detail::fromRegistryEntity(jointEntity), m_world);
  }

  return joints;
}

//==============================================================================
std::vector<std::string> Multibody::getLinkNames() const
{
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  const auto& registry = dart::simulation::detail::registryOf(*m_world);

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
  const auto& structure = safeGet<comps::MultibodyStructure>(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
  const auto& registry = dart::simulation::detail::registryOf(*m_world);

  std::vector<std::string> names;
  names.reserve(structure.joints.size());
  for (const auto& jointEntity : structure.joints) {
    names.push_back(safeGet<comps::Name>(registry, jointEntity).name);
  }

  return names;
}

//==============================================================================
Entity Multibody::getEntity() const
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
  const auto entity = detail::toRegistryEntity(m_entity);
  return m_world != nullptr
         && dart::simulation::detail::registryOf(*m_world).valid(entity)
         && dart::simulation::detail::registryOf(*m_world)
                .all_of<comps::MultibodyTag>(entity);
}

//==============================================================================
bool Multibody::isSleeping() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid multibody handle");

  return m_world->isDeactivationEntitySleeping(m_entity);
}

//==============================================================================
int Multibody::getDeactivationGroupIndex() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid multibody handle");

  return m_world->getDeactivationGroupIndex(m_entity);
}

Link Multibody::addLink(std::string_view name)
{
  // Check design mode
  DART_SIMULATION_THROW_T_IF(
      m_world->isSimulationMode(),
      InvalidArgumentException,
      "Cannot create Link in simulation mode");

  auto& registry = dart::simulation::detail::registryOf(*m_world);
  auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));

  // Auto-generate name if not provided
  std::string actualName;
  if (name.empty()) {
    do {
      actualName = std::format("link_{:03d}", ++m_world->m_linkCounter);
    } while (containsName(registry, structure.links, actualName));
  } else {
    actualName = std::string(name);
    DART_SIMULATION_THROW_T_IF(
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
  linkComp.childJoints
      = comps::Link::EntityVector{dart::common::StlAllocator<entt::entity>{
          m_world->getMemoryManager().getFreeAllocator()}};
  linkComp.name = std::move(actualName);
  linkComp.parentJoint = entt::null;

  // Add to Multibody structure
  structure.links.push_back(linkEntity);
  m_world->markFrameTopologyChanged();

  return Link(detail::fromRegistryEntity(linkEntity), m_world);
}

//==============================================================================
Link Multibody::addLink(std::string_view name, const LinkOptions& options)
{
  // Check design mode
  DART_SIMULATION_THROW_T_IF(
      m_world->isSimulationMode(),
      InvalidArgumentException,
      "Cannot create link in simulation mode. Links must be created in "
      "design mode.");

  // Validate parent link exists
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  auto parentEntity = detail::toRegistryEntity(options.parentLink.getEntity());

  DART_SIMULATION_THROW_T_IF(
      options.parentLink.getWorld() != m_world,
      InvalidArgumentException,
      "Parent link belongs to a different world");

  DART_SIMULATION_THROW_T_IF(
      !registry.valid(parentEntity),
      InvalidArgumentException,
      "Parent link is invalid");

  auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
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
    DART_SIMULATION_THROW_T_IF(
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
    DART_SIMULATION_THROW_T_IF(
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
  linkComp.childJoints
      = comps::Link::EntityVector{dart::common::StlAllocator<entt::entity>{
          m_world->getMemoryManager().getFreeAllocator()}};
  linkComp.name = std::move(actualLinkName);

  DART_SIMULATION_THROW_T_IF(
      !options.transformToParent.matrix().allFinite(),
      InvalidArgumentException,
      "Link transform to parent must contain only finite values");
  DART_SIMULATION_THROW_T_IF(
      !options.transformFromParent.matrix().allFinite(),
      InvalidArgumentException,
      "Link transform from parent must contain only finite values");
  linkComp.transformFromParentToJoint = options.transformToParent;
  linkComp.transformFromParentJoint = options.transformFromParent;

  // Create joint entity
  auto jointEntity = registry.create();

  // Add Name (for consistency with other named entities)
  registry.emplace<comps::Name>(jointEntity, actualJointName);

  // Add Joint with type-specific configuration
  auto& jointModel = registry.emplace<comps::JointModel>(jointEntity);
  auto& jointState = registry.emplace<comps::JointState>(jointEntity);
  auto& jointActuation = registry.emplace<comps::JointActuation>(jointEntity);
  jointModel.type = toComponentJointType(options.jointType);
  jointModel.name = std::move(actualJointName);

  DART_SIMULATION_THROW_T_IF(
      !options.axis.allFinite(),
      InvalidArgumentException,
      "Joint axis must contain only finite values");
  const double axisNorm = options.axis.norm();
  DART_SIMULATION_THROW_T_IF(
      axisNorm <= 1e-9,
      InvalidArgumentException,
      "Joint axis must be non-zero");
  jointModel.axis = options.axis / axisNorm;

  DART_SIMULATION_THROW_T_IF(
      !options.axis2.allFinite(),
      InvalidArgumentException,
      "Joint axis2 must contain only finite values");
  const double axis2Norm = options.axis2.norm();
  DART_SIMULATION_THROW_T_IF(
      axis2Norm <= 1e-9,
      InvalidArgumentException,
      "Joint axis2 must be non-zero");
  jointModel.axis2 = options.axis2 / axis2Norm;

  // Planar and universal joints build a second in-plane/rotation direction from
  // axis2. If axis is parallel to axis2 the derived basis collapses (its cross
  // product is zero), producing degenerate transforms during sync/step, so
  // reject it at construction time.
  if (jointModel.type == comps::JointType::Planar
      || jointModel.type == comps::JointType::Universal) {
    DART_SIMULATION_THROW_T_IF(
        jointModel.axis.cross(jointModel.axis2).norm() <= 1e-6,
        InvalidArgumentException,
        "Joint axis must not be parallel to axis2 for planar and universal "
        "joints");
  }

  jointModel.parentLink = parentEntity;
  jointModel.childLink = linkEntity;

  // Initialize state based on joint type
  const auto dof = static_cast<Eigen::Index>(jointModel.getDOF());
  jointState.position = comps::makeJointVector(dof, 0.0);
  jointState.velocity = comps::makeJointVector(dof, 0.0);
  jointState.acceleration = comps::makeJointVector(dof, 0.0);
  jointActuation.torque = comps::makeJointVector(dof, 0.0);
  jointModel.springStiffness = comps::makeJointVector(dof, 0.0);
  jointModel.dampingCoefficient = comps::makeJointVector(dof, 0.0);
  jointModel.restPosition = comps::makeJointVector(dof, 0.0);
  jointModel.armature = comps::makeJointVector(dof, 0.0);
  jointModel.coulombFriction = comps::makeJointVector(dof, 0.0);
  jointActuation.commandVelocity = comps::makeJointVector(dof, 0.0);

  // Position, velocity, and effort limits default to unbounded.
  const double infinity = std::numeric_limits<double>::infinity();
  jointModel.limits.lower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.upper = comps::makeJointVector(dof, infinity);
  jointModel.limits.velocityLower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.velocityUpper = comps::makeJointVector(dof, infinity);
  jointModel.limits.effortLower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.effortUpper = comps::makeJointVector(dof, infinity);

  // Link the parent link to this joint
  auto& parentLinkComp = safeGet<comps::Link>(registry, parentEntity);
  parentLinkComp.childJoints.push_back(jointEntity);

  // Link this link to the parent joint
  linkComp.parentJoint = jointEntity;

  // Add to Multibody structure
  structure.links.push_back(linkEntity);
  structure.joints.push_back(jointEntity);
  m_world->markFrameTopologyChanged();

  return Link(detail::fromRegistryEntity(linkEntity), m_world);
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
          .transformToParent = joint.transformToParent,
          .transformFromParent = joint.transformFromParent,
      });
}

//==============================================================================
Eigen::MatrixXd Multibody::getMassMatrix() const
{
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
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
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
  return compute::computeMultibodyDynamicsTerms(
             registry, structure, m_world->getGravity())
      .coriolisForces;
}

//==============================================================================
Eigen::VectorXd Multibody::getGravityForces() const
{
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
  return compute::computeMultibodyDynamicsTerms(
             registry, structure, m_world->getGravity())
      .gravityForces;
}

//==============================================================================
Eigen::VectorXd Multibody::getCoriolisAndGravityForces() const
{
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
  const compute::MultibodyDynamicsTerms terms
      = compute::computeMultibodyDynamicsTerms(
          registry, structure, m_world->getGravity());
  return terms.coriolisForces + terms.gravityForces;
}

//==============================================================================
Eigen::VectorXd Multibody::computeInverseDynamics(
    const Eigen::VectorXd& desiredAcceleration) const
{
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
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

  DART_SIMULATION_THROW_T_IF(
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
  DART_SIMULATION_THROW_T_IF(
      link.getWorld() != m_world,
      InvalidArgumentException,
      "Link belongs to a different world");

  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
  return compute::computeMultibodyLinkJacobian(
      registry, structure, detail::toRegistryEntity(link.getEntity()));
}

//==============================================================================
Eigen::MatrixXd Multibody::getWorldJacobian(const Link& link) const
{
  DART_SIMULATION_THROW_T_IF(
      link.getWorld() != m_world,
      InvalidArgumentException,
      "Link belongs to a different world");

  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
  return compute::computeMultibodyLinkWorldJacobian(
      registry, structure, detail::toRegistryEntity(link.getEntity()));
}

//==============================================================================
void Multibody::setGroundContact(
    const Eigen::Vector3d& planeNormal,
    const Eigen::Vector3d& planePoint,
    double stiffness,
    double frictionCoefficient,
    double frictionRegularization,
    double dampingCoefficient,
    std::size_t dualUpdateCadence)
{
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  auto& contact = registry.emplace_or_replace<comps::VariationalContact>(
      detail::toRegistryEntity(m_entity),
      makeVariationalContact(m_world->getMemoryManager().getFreeAllocator()));
  contact.planeNormal = planeNormal;
  contact.planePoint = planePoint;
  contact.stiffness = stiffness;
  contact.frictionCoefficient = frictionCoefficient;
  contact.frictionRegularization = frictionRegularization;
  contact.dampingCoefficient = dampingCoefficient;
  contact.dualUpdateCadence = dualUpdateCadence;
  // Reconfiguring contact resets the augmented-Lagrangian dual accumulator so a
  // new (or re-pointed) contact set starts cold rather than with stale duals.
  registry.remove<comps::VariationalContactDualState>(
      detail::toRegistryEntity(m_entity));
}

//==============================================================================
void Multibody::addGroundContactPoint(
    const Link& link, const Eigen::Vector3d& localPoint)
{
  // Reject links from a different World up front: separate registries can reuse
  // the same raw entity id, so a foreign link could otherwise alias a
  // numerically-equal link here and register contact on the wrong body (mirrors
  // the getWorldJacobian guard).
  DART_SIMULATION_THROW_T_IF(
      link.getWorld() != m_world,
      InvalidArgumentException,
      "addGroundContactPoint: link belongs to a different world");
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  auto* contact = registry.try_get<comps::VariationalContact>(
      detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
      contact == nullptr,
      InvalidArgumentException,
      "addGroundContactPoint requires setGroundContact() to be called first");
  const auto& structure = safeGet<comps::MultibodyStructure>(
      registry, detail::toRegistryEntity(m_entity));
  const auto it = std::find(
      structure.links.begin(),
      structure.links.end(),
      detail::toRegistryEntity(link.getEntity()));
  DART_SIMULATION_THROW_T_IF(
      it == structure.links.end(),
      InvalidArgumentException,
      "addGroundContactPoint: link is not part of this multibody");
  ensureVariationalContactAllocator(
      *contact, m_world->getMemoryManager().getFreeAllocator());
  contact->pointLinkIndices.push_back(
      static_cast<std::size_t>(it - structure.links.begin()));
  contact->pointLocalPositions.push_back(localPoint);
}

} // namespace dart::simulation
