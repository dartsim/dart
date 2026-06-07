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

#include "dart/simulation/compute/multibody_constraint.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/joint.hpp"
#include "dart/simulation/comps/link.hpp"
#include "dart/simulation/comps/multibody.hpp"

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>

namespace dart::simulation::compute {

namespace {

//==============================================================================
Eigen::Matrix3d rotationExp(const Eigen::Vector3d& rotationVector)
{
  const double angle = rotationVector.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotationVector / angle).toRotationMatrix();
}

//==============================================================================
Eigen::Vector3d rotationLog(const Eigen::Matrix3d& rotation)
{
  const Eigen::AngleAxisd angleAxis(rotation);
  return angleAxis.angle() * angleAxis.axis();
}

//==============================================================================
void applyPositionLimits(
    comps::Joint& joint, Eigen::Index firstDof, Eigen::Index dofCount)
{
  if (joint.limits.lower.size() != joint.position.size()
      || joint.limits.upper.size() != joint.position.size()) {
    return;
  }

  const Eigen::Index endDof = firstDof + dofCount;
  for (Eigen::Index d = firstDof; d < endDof; ++d) {
    if (joint.position[d] < joint.limits.lower[d]) {
      joint.position[d] = joint.limits.lower[d];
      joint.velocity[d] = std::max(joint.velocity[d], 0.0);
    } else if (joint.position[d] > joint.limits.upper[d]) {
      joint.position[d] = joint.limits.upper[d];
      joint.velocity[d] = std::min(joint.velocity[d], 0.0);
    }
  }
}

} // namespace

//==============================================================================
void integrateMultibodyPositions(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& nextVelocity,
    double timeStep)
{
  Eigen::Index expectedDof = 0;
  for (const auto linkEntity : structure.links) {
    const auto& link = registry.get<comps::Link>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }

    const auto& joint = registry.get<comps::Joint>(link.parentJoint);
    expectedDof += static_cast<Eigen::Index>(joint.getDOF());
  }

  DART_SIMULATION_THROW_T_IF(
      nextVelocity.size() != expectedDof,
      InvalidArgumentException,
      "Next multibody velocity dimension ({}) does not match the expected DOF "
      "count ({})",
      nextVelocity.size(),
      expectedDof);

  Eigen::Index velocityOffset = 0;
  for (const auto linkEntity : structure.links) {
    const auto& link = registry.get<comps::Link>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }

    auto& joint = registry.get<comps::Joint>(link.parentJoint);
    const auto dof = static_cast<Eigen::Index>(joint.getDOF());
    if (dof == 0) {
      continue;
    }

    joint.acceleration = joint.velocity;
    joint.velocity = nextVelocity.segment(velocityOffset, dof);
    velocityOffset += dof;

    const auto updateAccelerationFromVelocityDelta = [&]() {
      joint.acceleration = (joint.velocity - joint.acceleration) / timeStep;
    };

    if (joint.type == comps::JointType::Spherical) {
      // Body angular velocity integrated by right multiplication on SO(3).
      const Eigen::Matrix3d rotation = rotationExp(joint.position.head<3>());
      joint.position.head<3>() = rotationLog(
          rotation * rotationExp(joint.velocity.head<3>() * timeStep));
      updateAccelerationFromVelocityDelta();
      continue;
    }

    if (joint.type == comps::JointType::Floating) {
      // Velocity is [linear; angular] body twist. Translation advances in the
      // parent frame (R * v) and orientation integrates on SO(3).
      const Eigen::Matrix3d rotation = rotationExp(joint.position.tail<3>());
      joint.position.head<3>()
          += rotation * joint.velocity.head<3>() * timeStep;
      joint.position.tail<3>() = rotationLog(
          rotation * rotationExp(joint.velocity.tail<3>() * timeStep));
      applyPositionLimits(joint, 0, 3);
      updateAccelerationFromVelocityDelta();
      continue;
    }

    joint.position += joint.velocity * timeStep;
    applyPositionLimits(joint, 0, joint.position.size());
    updateAccelerationFromVelocityDelta();
  }
}

} // namespace dart::simulation::compute
