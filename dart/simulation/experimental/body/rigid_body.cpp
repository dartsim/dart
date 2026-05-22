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

#include "dart/simulation/experimental/body/rigid_body.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>

#include <cmath>

namespace {

//==============================================================================
void validateFiniteVector(const Eigen::Vector3d& value, const char* fieldName)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !value.allFinite(),
      dart::simulation::experimental::InvalidArgumentException,
      "RigidBody {} must contain only finite values",
      fieldName);
}

//==============================================================================
bool isSymmetricPositiveDefinite(const Eigen::Matrix3d& matrix)
{
  if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-12)) {
    return false;
  }

  Eigen::LLT<Eigen::Matrix3d> factorization(matrix);
  return factorization.info() == Eigen::Success;
}

//==============================================================================
void validateMass(double mass)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(mass) || mass <= 0.0,
      dart::simulation::experimental::InvalidArgumentException,
      "RigidBody mass must be positive and finite");
}

//==============================================================================
void validateInertia(const Eigen::Matrix3d& inertia)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isSymmetricPositiveDefinite(inertia),
      dart::simulation::experimental::InvalidArgumentException,
      "RigidBody inertia must be symmetric positive definite");
}

} // namespace

namespace dart::simulation::experimental {

//==============================================================================
RigidBody::RigidBody(entt::entity entity, World* world) : Frame(entity, world)
{
  // Frame base constructor handles entity and world
}

//==============================================================================
std::string RigidBody::getName() const
{
  const auto& registry = getWorld()->getRegistry();
  if (const auto* name = registry.try_get<comps::Name>(getEntity())) {
    return name->name;
  }
  return "";
}

//==============================================================================
const Eigen::Isometry3d& RigidBody::getLocalTransform() const
{
  const auto& props
      = m_world->getRegistry().get<comps::FreeFrameProperties>(m_entity);
  return props.localTransform;
}

//==============================================================================
void RigidBody::setTransform(const Eigen::Isometry3d& transform)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  DART_EXPERIMENTAL_THROW_T_IF(
      !transform.matrix().allFinite(),
      InvalidArgumentException,
      "RigidBody transform must contain only finite values");

  constexpr double tolerance = 1e-9;
  const auto rotation = transform.rotation();
  const double orthonormalError
      = (rotation * rotation.transpose() - Eigen::Matrix3d::Identity())
            .cwiseAbs()
            .maxCoeff();
  DART_EXPERIMENTAL_THROW_T_IF(
      orthonormalError > tolerance
          || std::abs(rotation.determinant() - 1.0) > tolerance,
      InvalidArgumentException,
      "RigidBody transform rotation must be orthonormal");

  auto& registry = getWorld()->getRegistry();
  auto& freeFrame = registry.get<comps::FreeFrameProperties>(getEntity());
  auto& rigidTransform = registry.get<comps::Transform>(getEntity());

  const Eigen::Isometry3d parentTransform = getParentFrame().getTransform();
  freeFrame.localTransform = parentTransform.inverse() * transform;

  rigidTransform.position = transform.translation();
  rigidTransform.orientation = Eigen::Quaterniond(transform.rotation());
  rigidTransform.orientation.normalize();

  markSubtreeTransformCacheDirty();
}

//==============================================================================
Eigen::Vector3d RigidBody::getLinearVelocity() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().get<comps::Velocity>(getEntity()).linear;
}

//==============================================================================
void RigidBody::setLinearVelocity(const Eigen::Vector3d& velocity)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(velocity, "linear velocity");

  getWorld()->getRegistry().get<comps::Velocity>(getEntity()).linear = velocity;
}

//==============================================================================
Eigen::Vector3d RigidBody::getAngularVelocity() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().get<comps::Velocity>(getEntity()).angular;
}

//==============================================================================
void RigidBody::setAngularVelocity(const Eigen::Vector3d& velocity)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(velocity, "angular velocity");

  getWorld()->getRegistry().get<comps::Velocity>(getEntity()).angular
      = velocity;
}

//==============================================================================
double RigidBody::getMass() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().get<comps::MassProperties>(getEntity()).mass;
}

//==============================================================================
void RigidBody::setMass(double mass)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateMass(mass);

  getWorld()->getRegistry().get<comps::MassProperties>(getEntity()).mass = mass;
}

//==============================================================================
Eigen::Matrix3d RigidBody::getInertia() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()
      ->getRegistry()
      .get<comps::MassProperties>(getEntity())
      .inertia;
}

//==============================================================================
void RigidBody::setInertia(const Eigen::Matrix3d& inertia)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateInertia(inertia);

  getWorld()->getRegistry().get<comps::MassProperties>(getEntity()).inertia
      = inertia;
}

//==============================================================================
Eigen::Vector3d RigidBody::getForce() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().get<comps::Force>(getEntity()).force;
}

//==============================================================================
void RigidBody::setForce(const Eigen::Vector3d& force)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(force, "force");

  getWorld()->getRegistry().get<comps::Force>(getEntity()).force = force;
}

//==============================================================================
void RigidBody::applyForce(const Eigen::Vector3d& force)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(force, "force");

  getWorld()->getRegistry().get<comps::Force>(getEntity()).force += force;
}

//==============================================================================
void RigidBody::clearForce()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  getWorld()->getRegistry().get<comps::Force>(getEntity()).force.setZero();
}

//==============================================================================
Eigen::Vector3d RigidBody::getTorque() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().get<comps::Force>(getEntity()).torque;
}

//==============================================================================
void RigidBody::setTorque(const Eigen::Vector3d& torque)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(torque, "torque");

  getWorld()->getRegistry().get<comps::Force>(getEntity()).torque = torque;
}

//==============================================================================
void RigidBody::applyTorque(const Eigen::Vector3d& torque)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(torque, "torque");

  getWorld()->getRegistry().get<comps::Force>(getEntity()).torque += torque;
}

//==============================================================================
void RigidBody::clearTorque()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  getWorld()->getRegistry().get<comps::Force>(getEntity()).torque.setZero();
}

// getEntity() and isValid() inherited from Frame

} // namespace dart::simulation::experimental
