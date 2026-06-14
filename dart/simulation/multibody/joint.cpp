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

#include "dart/simulation/multibody/joint.hpp"

#include "dart/simulation/body/rigid_body.hpp"
#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/all.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/multibody/link.hpp"
#include "dart/simulation/world.hpp"

#include <dart/common/stl_allocator.hpp>

#include <vector>

#include <cmath>

namespace dart::simulation {

namespace {

JointType toPublicJointType(comps::JointType type)
{
  switch (type) {
    case comps::JointType::Fixed:
      return JointType::Fixed;
    case comps::JointType::Revolute:
      return JointType::Revolute;
    case comps::JointType::Prismatic:
      return JointType::Prismatic;
    case comps::JointType::Screw:
      return JointType::Screw;
    case comps::JointType::Universal:
      return JointType::Universal;
    case comps::JointType::Spherical:
      return JointType::Spherical;
    case comps::JointType::Planar:
      return JointType::Planar;
    case comps::JointType::Floating:
      return JointType::Floating;
    case comps::JointType::Custom:
      return JointType::Custom;
  }

  return JointType::Custom;
}

comps::ActuatorType toComponentActuatorType(ActuatorType type)
{
  switch (type) {
    case ActuatorType::Force:
      return comps::ActuatorType::Force;
    case ActuatorType::Passive:
      return comps::ActuatorType::Passive;
    case ActuatorType::Servo:
      return comps::ActuatorType::Servo;
    case ActuatorType::Velocity:
      return comps::ActuatorType::Velocity;
    case ActuatorType::Acceleration:
      return comps::ActuatorType::Acceleration;
    case ActuatorType::Locked:
      return comps::ActuatorType::Locked;
    case ActuatorType::Mimic:
      return comps::ActuatorType::Mimic;
  }

  return comps::ActuatorType::Force;
}

ActuatorType toPublicActuatorType(comps::ActuatorType type)
{
  switch (type) {
    case comps::ActuatorType::Force:
      return ActuatorType::Force;
    case comps::ActuatorType::Passive:
      return ActuatorType::Passive;
    case comps::ActuatorType::Servo:
      return ActuatorType::Servo;
    case comps::ActuatorType::Velocity:
      return ActuatorType::Velocity;
    case comps::ActuatorType::Acceleration:
      return ActuatorType::Acceleration;
    case comps::ActuatorType::Locked:
      return ActuatorType::Locked;
    case comps::ActuatorType::Mimic:
      return ActuatorType::Mimic;
  }

  return ActuatorType::Force;
}

comps::Joint& getJointComponent(World* world, Entity entityToken)
{
  DART_SIMULATION_THROW_T_IF(
      world == nullptr, InvalidArgumentException, "Invalid joint handle");

  const auto entity = detail::toRegistryEntity(entityToken);
  auto& registry = dart::simulation::detail::registryOf(*world);
  DART_SIMULATION_THROW_T_IF(
      !registry.valid(entity) || !registry.all_of<comps::Joint>(entity),
      InvalidArgumentException,
      "Invalid joint handle");

  return registry.get<comps::Joint>(entity);
}

detail::deformable_vbd::AvbdRigidWorldPointJointConfig*
tryGetAvbdRigidWorldPointJointConfig(World* world, Entity entityToken)
{
  (void)getJointComponent(world, entityToken);
  auto& registry = dart::simulation::detail::registryOf(*world);
  const auto entity = detail::toRegistryEntity(entityToken);
  return registry
      .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(entity);
}

void validateAvbdRigidWorldPointJointFacade(
    const dart::simulation::detail::WorldRegistry& registry,
    entt::entity jointEntity,
    const comps::Joint& joint)
{
  DART_SIMULATION_THROW_T_IF(
      !detail::deformable_vbd::isAvbdRigidWorldPointJointFacade(
          registry, jointEntity, joint),
      InvalidArgumentException,
      "Joint is not an AVBD point-joint facade");
}

// Materialize and return the AVBD point-joint stiffness sidecar for the joint,
// emplacing it if absent. Returns a reference into the registry storage.
comps::AvbdJointStiffness& materializeAvbdStiffnessState(
    dart::simulation::detail::WorldRegistry& registry,
    entt::entity jointEntity,
    comps::Joint& joint)
{
  if (auto* stiffness
      = registry.try_get<comps::AvbdJointStiffness>(jointEntity)) {
    return *stiffness;
  }

  comps::AvbdJointStiffness stiffness;

  if (const auto* config
      = registry
            .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                jointEntity)) {
    stiffness.startStiffness = config->startStiffness;
    stiffness.linearStiffness = config->linearMaterialStiffness;
    stiffness.angularStiffness = config->angularMaterialStiffness;
    stiffness.maxStiffness = config->maxStiffness;
    return registry.emplace<comps::AvbdJointStiffness>(jointEntity, stiffness);
  }

  double startStiffness = 0.0;
  double maxStiffness = 0.0;
  DART_SIMULATION_THROW_T_IF(
      !detail::deformable_vbd::computeAvbdRigidWorldPointJointDefaultStiffness(
          registry, joint, startStiffness, maxStiffness),
      InvalidArgumentException,
      "Joint is not an AVBD point-joint facade");

  stiffness.startStiffness = startStiffness;
  stiffness.maxStiffness = maxStiffness;
  return registry.emplace<comps::AvbdJointStiffness>(jointEntity, stiffness);
}

void validateFiniteNonnegative(
    double value, std::string_view fieldName, std::string_view jointName)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(value) || value < 0.0,
      InvalidArgumentException,
      "Joint '{}' {} must be finite and non-negative",
      jointName,
      fieldName);
}

void validateMaterialStiffness(
    double value, std::string_view fieldName, std::string_view jointName)
{
  DART_SIMULATION_THROW_T_IF(
      std::isnan(value) || value < 0.0,
      InvalidArgumentException,
      "Joint '{}' {} must be non-negative or infinity",
      jointName,
      fieldName);
}

void validateJointStateVector(
    const Eigen::VectorXd& value, std::size_t dof, const char* fieldName)
{
  DART_SIMULATION_THROW_T_IF(
      value.size() != static_cast<Eigen::Index>(dof),
      InvalidArgumentException,
      "Joint {} dimension ({}) must match DOF count ({})",
      fieldName,
      value.size(),
      dof);

  DART_SIMULATION_THROW_T_IF(
      !value.allFinite(),
      InvalidArgumentException,
      "Joint {} must contain only finite values",
      fieldName);
}

// Validate a lower/upper limit pair. Infinite bounds are allowed (they mark an
// unbounded coordinate); only NaN and lower > upper are rejected.
void validateJointLimitPair(
    const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper,
    std::size_t dof,
    const char* fieldName)
{
  const auto dofIndex = static_cast<Eigen::Index>(dof);
  DART_SIMULATION_THROW_T_IF(
      lower.size() != dofIndex || upper.size() != dofIndex,
      InvalidArgumentException,
      "Joint {} limit dimensions must match DOF count ({})",
      fieldName,
      dof);
  DART_SIMULATION_THROW_T_IF(
      lower.array().isNaN().any() || upper.array().isNaN().any(),
      InvalidArgumentException,
      "Joint {} limits must not be NaN",
      fieldName);
  DART_SIMULATION_THROW_T_IF(
      (lower.array() > upper.array()).any(),
      InvalidArgumentException,
      "Joint lower {} limits must not exceed upper limits",
      fieldName);
}

void markSubtreeTransformCacheDirty(
    auto& registry, entt::entity root, dart::common::MemoryAllocator& allocator)
{
  if (root == entt::null || !registry.valid(root)) {
    return;
  }

  std::vector<entt::entity, dart::common::StlAllocator<entt::entity>> stack(
      dart::common::StlAllocator<entt::entity>{allocator});
  stack.push_back(root);

  auto frameStateView = registry.template view<comps::FrameState>();

  while (!stack.empty()) {
    const auto entity = stack.back();
    stack.pop_back();

    if (!registry.valid(entity)) {
      continue;
    }

    if (auto* cache = registry.template try_get<comps::FrameCache>(entity)) {
      cache->needTransformUpdate = true;
    }

    for (const auto child : frameStateView) {
      const auto& state = frameStateView.template get<comps::FrameState>(child);
      if (state.parentFrame == entity) {
        stack.push_back(child);
      }
    }
  }
}

} // namespace

//==============================================================================
Joint::Joint(Entity entity, World* world) : m_entity(entity), m_world(world) {}

//==============================================================================
std::string_view Joint::getName() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.name;
}

//==============================================================================
JointType Joint::getType() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return toPublicJointType(jointComp.type);
}

//==============================================================================
ActuatorType Joint::getActuatorType() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return toPublicActuatorType(jointComp.actuatorType);
}

//==============================================================================
void Joint::setActuatorType(ActuatorType actuatorType)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  jointComp.actuatorType = toComponentActuatorType(actuatorType);
}

//==============================================================================
Eigen::VectorXd Joint::getCommandVelocity() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.commandVelocity;
}

//==============================================================================
void Joint::setCommandVelocity(const Eigen::VectorXd& velocity)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(velocity, jointComp.getDOF(), "command velocity");
  jointComp.commandVelocity = velocity;
}

//==============================================================================
Eigen::Vector3d Joint::getAxis() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);

  // Validate joint type - Spherical and Floating joints don't have axes
  DART_SIMULATION_THROW_T_IF(
      jointComp.type == comps::JointType::Spherical
          || jointComp.type == comps::JointType::Floating,
      InvalidArgumentException,
      "getAxis() is not valid for Spherical and Floating joints (no axis "
      "concept)");

  return jointComp.axis;
}

//==============================================================================
Eigen::Vector3d Joint::getAxis2() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);

  // Validate joint type
  DART_SIMULATION_THROW_T_IF(
      jointComp.type != comps::JointType::Universal
          && jointComp.type != comps::JointType::Planar,
      InvalidArgumentException,
      "getAxis2() is only valid for Universal and Planar joints");

  return jointComp.axis2;
}

//==============================================================================
double Joint::getPitch() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);

  // Validate joint type
  DART_SIMULATION_THROW_T_IF(
      jointComp.type != comps::JointType::Screw,
      InvalidArgumentException,
      "getPitch() is only valid for Screw joints");

  return jointComp.pitch;
}

//==============================================================================
void Joint::setPitch(double pitch)
{
  auto& jointComp = getJointComponent(m_world, m_entity);

  DART_SIMULATION_THROW_T_IF(
      jointComp.type != comps::JointType::Screw,
      InvalidArgumentException,
      "setPitch() is only valid for Screw joints");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(pitch),
      InvalidArgumentException,
      "Joint pitch must be finite");

  jointComp.pitch = pitch;
}

//==============================================================================
std::size_t Joint::getDOFCount() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.getDOF();
}

//==============================================================================
Eigen::VectorXd Joint::getPosition() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.position;
}

//==============================================================================
void Joint::setPosition(const Eigen::VectorXd& position)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(position, jointComp.getDOF(), "position");

  jointComp.position = position;
  markSubtreeTransformCacheDirty(
      dart::simulation::detail::registryOf(*m_world),
      jointComp.childLink,
      m_world->getMemoryManager().getFreeAllocator());
}

//==============================================================================
Eigen::VectorXd Joint::getVelocity() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.velocity;
}

//==============================================================================
void Joint::setVelocity(const Eigen::VectorXd& velocity)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(velocity, jointComp.getDOF(), "velocity");

  jointComp.velocity = velocity;
}

//==============================================================================
Eigen::VectorXd Joint::getForce() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.torque;
}

//==============================================================================
void Joint::setForce(const Eigen::VectorXd& force)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(force, jointComp.getDOF(), "force");

  jointComp.torque = force;
}

//==============================================================================
Eigen::VectorXd Joint::getAcceleration() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.acceleration;
}

//==============================================================================
Eigen::VectorXd Joint::getSpringStiffness() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.springStiffness;
}

//==============================================================================
void Joint::setSpringStiffness(const Eigen::VectorXd& stiffness)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(stiffness, jointComp.getDOF(), "spring stiffness");
  DART_SIMULATION_THROW_T_IF(
      (stiffness.array() < 0.0).any(),
      InvalidArgumentException,
      "Joint spring stiffness must be non-negative");

  jointComp.springStiffness = stiffness;
}

//==============================================================================
Eigen::VectorXd Joint::getRestPosition() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.restPosition;
}

//==============================================================================
void Joint::setRestPosition(const Eigen::VectorXd& restPosition)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(restPosition, jointComp.getDOF(), "rest position");

  jointComp.restPosition = restPosition;
}

//==============================================================================
Eigen::VectorXd Joint::getDampingCoefficient() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.dampingCoefficient;
}

//==============================================================================
void Joint::setDampingCoefficient(const Eigen::VectorXd& damping)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(damping, jointComp.getDOF(), "damping coefficient");
  DART_SIMULATION_THROW_T_IF(
      (damping.array() < 0.0).any(),
      InvalidArgumentException,
      "Joint damping coefficient must be non-negative");

  jointComp.dampingCoefficient = damping;
}

//==============================================================================
Eigen::VectorXd Joint::getArmature() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.armature;
}

//==============================================================================
void Joint::setArmature(const Eigen::VectorXd& armature)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(armature, jointComp.getDOF(), "armature");
  DART_SIMULATION_THROW_T_IF(
      (armature.array() < 0.0).any(),
      InvalidArgumentException,
      "Joint armature must be non-negative");

  jointComp.armature = armature;
}

//==============================================================================
Eigen::VectorXd Joint::getCoulombFriction() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return jointComp.coulombFriction;
}

//==============================================================================
void Joint::setCoulombFriction(const Eigen::VectorXd& friction)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointStateVector(friction, jointComp.getDOF(), "Coulomb friction");
  DART_SIMULATION_THROW_T_IF(
      (friction.array() < 0.0).any(),
      InvalidArgumentException,
      "Joint Coulomb friction must be non-negative");

  jointComp.coulombFriction = friction;
}

//==============================================================================
void Joint::setPositionLimits(
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointLimitPair(lower, upper, jointComp.getDOF(), "position");
  jointComp.limits.lower = lower;
  jointComp.limits.upper = upper;
}

//==============================================================================
Eigen::VectorXd Joint::getPositionLowerLimits() const
{
  return getJointComponent(m_world, m_entity).limits.lower;
}

//==============================================================================
Eigen::VectorXd Joint::getPositionUpperLimits() const
{
  return getJointComponent(m_world, m_entity).limits.upper;
}

//==============================================================================
void Joint::setVelocityLimits(
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointLimitPair(lower, upper, jointComp.getDOF(), "velocity");
  jointComp.limits.velocityLower = lower;
  jointComp.limits.velocityUpper = upper;
}

//==============================================================================
Eigen::VectorXd Joint::getVelocityLowerLimits() const
{
  return getJointComponent(m_world, m_entity).limits.velocityLower;
}

//==============================================================================
Eigen::VectorXd Joint::getVelocityUpperLimits() const
{
  return getJointComponent(m_world, m_entity).limits.velocityUpper;
}

//==============================================================================
void Joint::setEffortLimits(
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  validateJointLimitPair(lower, upper, jointComp.getDOF(), "effort");
  jointComp.limits.effortLower = lower;
  jointComp.limits.effortUpper = upper;
}

//==============================================================================
Eigen::VectorXd Joint::getEffortLowerLimits() const
{
  return getJointComponent(m_world, m_entity).limits.effortLower;
}

//==============================================================================
Eigen::VectorXd Joint::getEffortUpperLimits() const
{
  return getJointComponent(m_world, m_entity).limits.effortUpper;
}

//==============================================================================
void Joint::setBreakForce(double breakForce)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(breakForce) || breakForce < 0.0,
      InvalidArgumentException,
      "Joint '{}' break force must be finite and non-negative",
      getName());

  getJointComponent(m_world, m_entity).breakForce = breakForce;
}

//==============================================================================
double Joint::getBreakForce() const
{
  return getJointComponent(m_world, m_entity).breakForce;
}

//==============================================================================
bool Joint::isBroken() const
{
  return getJointComponent(m_world, m_entity).broken;
}

//==============================================================================
void Joint::resetBreakage()
{
  getJointComponent(m_world, m_entity).broken = false;
}

//==============================================================================
void Joint::setAvbdStartStiffness(double stiffness)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto entity = detail::toRegistryEntity(m_entity);
  validateAvbdRigidWorldPointJointFacade(registry, entity, jointComp);
  validateFiniteNonnegative(stiffness, "AVBD start stiffness", jointComp.name);
  auto& jointStiffness
      = materializeAvbdStiffnessState(registry, entity, jointComp);

  DART_SIMULATION_THROW_T_IF(
      stiffness > jointStiffness.maxStiffness,
      InvalidArgumentException,
      "Joint '{}' AVBD start stiffness must not exceed max stiffness",
      jointComp.name);
  jointStiffness.startStiffness = stiffness;
  if (auto* config = tryGetAvbdRigidWorldPointJointConfig(m_world, m_entity)) {
    config->startStiffness = stiffness;
  }
}

//==============================================================================
double Joint::getAvbdStartStiffness() const
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto entity = detail::toRegistryEntity(m_entity);
  validateAvbdRigidWorldPointJointFacade(registry, entity, jointComp);
  if (const auto* stiffness
      = registry.try_get<comps::AvbdJointStiffness>(entity)) {
    return stiffness->startStiffness;
  }

  if (const auto* config
      = registry
            .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                entity)) {
    return config->startStiffness;
  }

  double startStiffness = 0.0;
  double maxStiffness = 0.0;
  DART_SIMULATION_THROW_T_IF(
      !detail::deformable_vbd::computeAvbdRigidWorldPointJointDefaultStiffness(
          registry, jointComp, startStiffness, maxStiffness),
      InvalidArgumentException,
      "Joint is not an AVBD point-joint facade");
  return startStiffness;
}

//==============================================================================
void Joint::setAvbdLinearStiffness(double stiffness)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto entity = detail::toRegistryEntity(m_entity);
  validateAvbdRigidWorldPointJointFacade(registry, entity, jointComp);
  validateMaterialStiffness(stiffness, "AVBD linear stiffness", jointComp.name);
  auto& jointStiffness
      = materializeAvbdStiffnessState(registry, entity, jointComp);

  jointStiffness.linearStiffness = stiffness;
  if (auto* config = tryGetAvbdRigidWorldPointJointConfig(m_world, m_entity)) {
    config->linearMaterialStiffness = stiffness;
  }
}

//==============================================================================
double Joint::getAvbdLinearStiffness() const
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto entity = detail::toRegistryEntity(m_entity);
  validateAvbdRigidWorldPointJointFacade(registry, entity, jointComp);
  if (const auto* stiffness
      = registry.try_get<comps::AvbdJointStiffness>(entity)) {
    return stiffness->linearStiffness;
  }

  if (const auto* config
      = registry
            .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                entity)) {
    return config->linearMaterialStiffness;
  }
  return comps::AvbdJointStiffness{}.linearStiffness;
}

//==============================================================================
void Joint::setAvbdAngularStiffness(double stiffness)
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto entity = detail::toRegistryEntity(m_entity);
  validateAvbdRigidWorldPointJointFacade(registry, entity, jointComp);
  validateMaterialStiffness(
      stiffness, "AVBD angular stiffness", jointComp.name);
  auto& jointStiffness
      = materializeAvbdStiffnessState(registry, entity, jointComp);

  jointStiffness.angularStiffness = stiffness;
  if (auto* config = tryGetAvbdRigidWorldPointJointConfig(m_world, m_entity)) {
    config->angularMaterialStiffness = stiffness;
  }
}

//==============================================================================
double Joint::getAvbdAngularStiffness() const
{
  auto& jointComp = getJointComponent(m_world, m_entity);
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  const auto entity = detail::toRegistryEntity(m_entity);
  validateAvbdRigidWorldPointJointFacade(registry, entity, jointComp);
  if (const auto* stiffness
      = registry.try_get<comps::AvbdJointStiffness>(entity)) {
    return stiffness->angularStiffness;
  }

  if (const auto* config
      = registry
            .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                entity)) {
    return config->angularMaterialStiffness;
  }
  return comps::AvbdJointStiffness{}.angularStiffness;
}

//==============================================================================
Link Joint::getParentLink() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  DART_SIMULATION_THROW_T_IF(
      jointComp.parentLink == entt::null
          || !dart::simulation::detail::registryOf(*m_world)
                  .all_of<comps::Link>(jointComp.parentLink),
      InvalidArgumentException,
      "Joint '{}' parent endpoint is not a multibody Link",
      jointComp.name);
  return Link(detail::fromRegistryEntity(jointComp.parentLink), m_world);
}

//==============================================================================
Link Joint::getChildLink() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  DART_SIMULATION_THROW_T_IF(
      jointComp.childLink == entt::null
          || !dart::simulation::detail::registryOf(*m_world)
                  .all_of<comps::Link>(jointComp.childLink),
      InvalidArgumentException,
      "Joint '{}' child endpoint is not a multibody Link",
      jointComp.name);
  return Link(detail::fromRegistryEntity(jointComp.childLink), m_world);
}

//==============================================================================
RigidBody Joint::getParentRigidBody() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  DART_SIMULATION_THROW_T_IF(
      jointComp.parentLink == entt::null
          || !dart::simulation::detail::registryOf(*m_world)
                  .all_of<comps::RigidBodyTag>(jointComp.parentLink),
      InvalidArgumentException,
      "Joint '{}' parent endpoint is not a rigid body",
      jointComp.name);
  return RigidBody(detail::fromRegistryEntity(jointComp.parentLink), m_world);
}

//==============================================================================
RigidBody Joint::getChildRigidBody() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  DART_SIMULATION_THROW_T_IF(
      jointComp.childLink == entt::null
          || !dart::simulation::detail::registryOf(*m_world)
                  .all_of<comps::RigidBodyTag>(jointComp.childLink),
      InvalidArgumentException,
      "Joint '{}' child endpoint is not a rigid body",
      jointComp.name);
  return RigidBody(detail::fromRegistryEntity(jointComp.childLink), m_world);
}

//==============================================================================
Entity Joint::getEntity() const
{
  return m_entity;
}

//==============================================================================
bool Joint::isValid() const
{
  const auto entity = detail::toRegistryEntity(m_entity);
  return m_world != nullptr
         && dart::simulation::detail::registryOf(*m_world).valid(entity)
         && dart::simulation::detail::registryOf(*m_world).all_of<comps::Joint>(
             entity);
}

} // namespace dart::simulation
