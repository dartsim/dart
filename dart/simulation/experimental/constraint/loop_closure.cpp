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

comps::LoopClosure& getMutableLoopClosureComponent(const LoopClosure& closure)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !closure.isValid(),
      InvalidArgumentException,
      "Invalid loop closure handle");

  return closure.getWorld()->getRegistry().get<comps::LoopClosure>(
      closure.getEntity());
}

bool isValidClosureKinematicsPolicy(ClosureKinematicsPolicy policy)
{
  switch (policy) {
    case ClosureKinematicsPolicy::ResidualOnly:
    case ClosureKinematicsPolicy::Project:
      return true;
  }

  return false;
}

bool isValidClosureDynamicsPolicy(ClosureDynamicsPolicy policy)
{
  switch (policy) {
    case ClosureDynamicsPolicy::ResidualOnly:
    case ClosureDynamicsPolicy::Solve:
      return true;
  }

  return false;
}

void validateLoopClosureRuntimePolicy(const LoopClosureRuntimePolicy& policy)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValidClosureKinematicsPolicy(policy.kinematics),
      InvalidArgumentException,
      "LoopClosureRuntimePolicy.kinematics is invalid");

  DART_EXPERIMENTAL_THROW_T_IF(
      !isValidClosureDynamicsPolicy(policy.dynamics),
      InvalidArgumentException,
      "LoopClosureRuntimePolicy.dynamics is invalid");
}

Eigen::Vector3d computeWorldRotationResidual(
    const Eigen::Isometry3d& endpointA, const Eigen::Isometry3d& endpointB)
{
  const Eigen::Matrix3d relativeRotation
      = endpointB.linear().transpose() * endpointA.linear();
  const Eigen::AngleAxisd angleAxis(relativeRotation);
  return endpointB.linear() * (angleAxis.axis() * angleAxis.angle());
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
LoopClosureRuntimePolicy LoopClosure::getRuntimePolicy() const
{
  return getLoopClosureComponent(*this).runtimePolicy;
}

//==============================================================================
void LoopClosure::setRuntimePolicy(const LoopClosureRuntimePolicy& policy)
{
  validateLoopClosureRuntimePolicy(policy);
  getMutableLoopClosureComponent(*this).runtimePolicy = policy;
}

//==============================================================================
LoopClosureResidual LoopClosure::computeResidual() const
{
  const auto& closure = getLoopClosureComponent(*this);
  const auto endpointA = getFrameA().getTransform() * closure.offsetA;
  const auto endpointB = getFrameB().getTransform() * closure.offsetB;
  const Eigen::Vector3d linearResidual
      = endpointA.translation() - endpointB.translation();

  LoopClosureResidual residual;
  residual.enabled = closure.runtimePolicy.enabled;
  residual.active = closure.runtimePolicy.enabled;

  switch (closure.family) {
    case LoopClosureFamily::Rigid:
      residual.value.resize(6);
      residual.value.head<3>() = linearResidual;
      residual.value.tail<3>()
          = computeWorldRotationResidual(endpointA, endpointB);
      break;
    case LoopClosureFamily::Point:
      residual.value = linearResidual;
      break;
    case LoopClosureFamily::Distance:
      residual.value.resize(1);
      residual.value[0] = linearResidual.norm();
      break;
  }

  residual.norm = residual.value.norm();
  return residual;
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
