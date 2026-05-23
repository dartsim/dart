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

#include "dart/simulation/experimental/multi_body/joint.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/multi_body/link.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <vector>

namespace dart::simulation::experimental {

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
    case comps::JointType::Ball:
      return JointType::Ball;
    case comps::JointType::Planar:
      return JointType::Planar;
    case comps::JointType::Free:
      return JointType::Free;
    case comps::JointType::Custom:
      return JointType::Custom;
  }

  return JointType::Custom;
}

comps::Joint& getJointComponent(World* world, entt::entity entity)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      world == nullptr, InvalidArgumentException, "Invalid joint handle");

  auto& registry = world->getRegistry();
  DART_EXPERIMENTAL_THROW_T_IF(
      !registry.valid(entity) || !registry.all_of<comps::Joint>(entity),
      InvalidArgumentException,
      "Invalid joint handle");

  return registry.get<comps::Joint>(entity);
}

void validateJointStateVector(
    const Eigen::VectorXd& value, std::size_t dof, const char* fieldName)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      value.size() != static_cast<Eigen::Index>(dof),
      InvalidArgumentException,
      "Joint {} dimension ({}) must match DOF count ({})",
      fieldName,
      value.size(),
      dof);

  DART_EXPERIMENTAL_THROW_T_IF(
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
  DART_EXPERIMENTAL_THROW_T_IF(
      lower.size() != dofIndex || upper.size() != dofIndex,
      InvalidArgumentException,
      "Joint {} limit dimensions must match DOF count ({})",
      fieldName,
      dof);
  DART_EXPERIMENTAL_THROW_T_IF(
      lower.array().isNaN().any() || upper.array().isNaN().any(),
      InvalidArgumentException,
      "Joint {} limits must not be NaN",
      fieldName);
  DART_EXPERIMENTAL_THROW_T_IF(
      (lower.array() > upper.array()).any(),
      InvalidArgumentException,
      "Joint lower {} limits must not exceed upper limits",
      fieldName);
}

void markSubtreeTransformCacheDirty(entt::registry& registry, entt::entity root)
{
  if (root == entt::null || !registry.valid(root)) {
    return;
  }

  std::vector<entt::entity> stack;
  stack.push_back(root);

  auto frameStateView = registry.view<comps::FrameState>();

  while (!stack.empty()) {
    const auto entity = stack.back();
    stack.pop_back();

    if (!registry.valid(entity)) {
      continue;
    }

    if (auto* cache = registry.try_get<comps::FrameCache>(entity)) {
      cache->needTransformUpdate = true;
    }

    for (const auto child : frameStateView) {
      const auto& state = frameStateView.get<comps::FrameState>(child);
      if (state.parentFrame == entity) {
        stack.push_back(child);
      }
    }
  }
}

} // namespace

//==============================================================================
Joint::Joint(entt::entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

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
Eigen::Vector3d Joint::getAxis() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);

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
  const auto& jointComp = getJointComponent(m_world, m_entity);

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
  const auto& jointComp = getJointComponent(m_world, m_entity);

  // Validate joint type
  DART_EXPERIMENTAL_THROW_T_IF(
      jointComp.type != comps::JointType::Screw,
      InvalidArgumentException,
      "getPitch() is only valid for Screw joints");

  return jointComp.pitch;
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
  markSubtreeTransformCacheDirty(m_world->getRegistry(), jointComp.childLink);
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
  DART_EXPERIMENTAL_THROW_T_IF(
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
  DART_EXPERIMENTAL_THROW_T_IF(
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
  DART_EXPERIMENTAL_THROW_T_IF(
      (armature.array() < 0.0).any(),
      InvalidArgumentException,
      "Joint armature must be non-negative");

  jointComp.armature = armature;
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
Link Joint::getParentLink() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
  return Link(jointComp.parentLink, m_world);
}

//==============================================================================
Link Joint::getChildLink() const
{
  const auto& jointComp = getJointComponent(m_world, m_entity);
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
  return m_world != nullptr && m_world->getRegistry().valid(m_entity)
         && m_world->getRegistry().all_of<comps::Joint>(m_entity);
}

} // namespace dart::simulation::experimental
