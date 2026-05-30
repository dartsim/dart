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

//==============================================================================
void validateCollisionShape(
    const dart::simulation::experimental::CollisionShape& shape,
    const char* ownerName)
{
  using dart::simulation::experimental::CollisionShapeType;
  using dart::simulation::experimental::InvalidArgumentException;

  switch (shape.type) {
    case CollisionShapeType::Sphere:
      DART_EXPERIMENTAL_THROW_T_IF(
          !std::isfinite(shape.radius) || shape.radius <= 0.0,
          InvalidArgumentException,
          "{} sphere collision shape radius must be positive and finite",
          ownerName);
      break;
    case CollisionShapeType::Box:
      DART_EXPERIMENTAL_THROW_T_IF(
          !shape.halfExtents.allFinite()
              || (shape.halfExtents.array() <= 0.0).any(),
          InvalidArgumentException,
          "{} box collision shape half extents must be positive and finite",
          ownerName);
      break;
    case CollisionShapeType::Mesh: {
      DART_EXPERIMENTAL_THROW_T_IF(
          shape.vertices.empty(),
          InvalidArgumentException,
          "{} mesh collision shape must contain at least one vertex",
          ownerName);
      DART_EXPERIMENTAL_THROW_T_IF(
          shape.triangles.empty(),
          InvalidArgumentException,
          "{} mesh collision shape must contain at least one triangle",
          ownerName);

      for (const Eigen::Vector3d& vertex : shape.vertices) {
        DART_EXPERIMENTAL_THROW_T_IF(
            !vertex.allFinite(),
            InvalidArgumentException,
            "{} mesh collision shape vertices must be finite",
            ownerName);
      }

      const int vertexCount = static_cast<int>(shape.vertices.size());
      for (const Eigen::Vector3i& triangle : shape.triangles) {
        DART_EXPERIMENTAL_THROW_T_IF(
            triangle.minCoeff() < 0 || triangle.maxCoeff() >= vertexCount,
            InvalidArgumentException,
            "{} mesh collision shape triangle indices are out of range",
            ownerName);
        DART_EXPERIMENTAL_THROW_T_IF(
            triangle.x() == triangle.y() || triangle.x() == triangle.z()
                || triangle.y() == triangle.z(),
            InvalidArgumentException,
            "{} mesh collision shape triangles must use three distinct "
            "vertices",
            ownerName);
      }
      break;
    }
  }
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

//==============================================================================
Eigen::Vector3d RigidBody::getLinearMomentum() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  const auto& mass = registry.get<comps::MassProperties>(getEntity());
  const auto& velocity = registry.get<comps::Velocity>(getEntity());
  return mass.mass * velocity.linear;
}

//==============================================================================
Eigen::Vector3d RigidBody::getAngularMomentum() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  const auto& mass = registry.get<comps::MassProperties>(getEntity());
  const auto& velocity = registry.get<comps::Velocity>(getEntity());
  const auto& transform = registry.get<comps::Transform>(getEntity());

  const Eigen::Matrix3d rotation
      = transform.orientation.normalized().toRotationMatrix();
  const Eigen::Matrix3d worldInertia
      = rotation * mass.inertia * rotation.transpose();
  return worldInertia * velocity.angular;
}

//==============================================================================
double RigidBody::getKineticEnergy() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  const auto& mass = registry.get<comps::MassProperties>(getEntity());
  const auto& velocity = registry.get<comps::Velocity>(getEntity());
  const auto& transform = registry.get<comps::Transform>(getEntity());

  const Eigen::Matrix3d rotation
      = transform.orientation.normalized().toRotationMatrix();
  const Eigen::Matrix3d worldInertia
      = rotation * mass.inertia * rotation.transpose();

  const double translational = 0.5 * mass.mass * velocity.linear.squaredNorm();
  const double rotational
      = 0.5 * velocity.angular.dot(worldInertia * velocity.angular);
  return translational + rotational;
}

//==============================================================================
double RigidBody::getPotentialEnergy() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  const auto& mass = registry.get<comps::MassProperties>(getEntity());
  const auto& transform = registry.get<comps::Transform>(getEntity());
  return -mass.mass * getWorld()->getGravity().dot(transform.position);
}

//==============================================================================
void RigidBody::setStatic(bool isStatic)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  auto& registry = getWorld()->getRegistry();
  if (isStatic) {
    registry.emplace_or_replace<comps::StaticBodyTag>(getEntity());
  } else {
    registry.remove<comps::StaticBodyTag>(getEntity());
  }
}

//==============================================================================
bool RigidBody::isStatic() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().all_of<comps::StaticBodyTag>(getEntity());
}

//==============================================================================
void RigidBody::setKinematic(bool isKinematic)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  auto& registry = getWorld()->getRegistry();
  if (isKinematic) {
    // Kinematic and static are mutually exclusive: a kinematic body is advanced
    // by its prescribed velocity, not held immovable.
    registry.remove<comps::StaticBodyTag>(getEntity());
    registry.emplace_or_replace<comps::KinematicBodyTag>(getEntity());
  } else {
    registry.remove<comps::KinematicBodyTag>(getEntity());
  }
}

//==============================================================================
bool RigidBody::isKinematic() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().all_of<comps::KinematicBodyTag>(getEntity());
}

//==============================================================================
void RigidBody::setRestitution(double restitution)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(restitution) || restitution < 0.0 || restitution > 1.0,
      InvalidArgumentException,
      "RigidBody restitution must be in [0, 1]");

  auto& registry = getWorld()->getRegistry();
  auto& material = registry.get_or_emplace<comps::ContactMaterial>(getEntity());
  material.restitution = restitution;
}

//==============================================================================
double RigidBody::getRestitution() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  if (const auto* material
      = registry.try_get<comps::ContactMaterial>(getEntity())) {
    return material->restitution;
  }
  return 0.0;
}

//==============================================================================
void RigidBody::setFriction(double friction)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(friction) || friction < 0.0,
      InvalidArgumentException,
      "RigidBody friction must be non-negative and finite");

  auto& registry = getWorld()->getRegistry();
  auto& material = registry.get_or_emplace<comps::ContactMaterial>(getEntity());
  material.friction = friction;
}

//==============================================================================
double RigidBody::getFriction() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  if (const auto* material
      = registry.try_get<comps::ContactMaterial>(getEntity())) {
    return material->friction;
  }
  return 1.0;
}

//==============================================================================
void RigidBody::setCollisionShape(const CollisionShape& shape)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateCollisionShape(shape, "RigidBody");

  auto& registry = getWorld()->getRegistry();
  comps::CollisionGeometry geometry{shape};
  registry.emplace_or_replace<comps::CollisionGeometry>(getEntity(), geometry);
}

//==============================================================================
void RigidBody::setDeformableGroundBarrier(bool enabled)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  auto& registry = getWorld()->getRegistry();
  if (enabled) {
    registry.get_or_emplace<comps::DeformableGroundBarrierTag>(getEntity());
  } else {
    registry.remove<comps::DeformableGroundBarrierTag>(getEntity());
  }
}

//==============================================================================
bool RigidBody::isDeformableGroundBarrier() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().all_of<comps::DeformableGroundBarrierTag>(
      getEntity());
}

//==============================================================================
void RigidBody::setDeformableSurfaceCcdObstacle(bool enabled)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  auto& registry = getWorld()->getRegistry();
  if (enabled) {
    registry.get_or_emplace<comps::DeformableSurfaceCcdObstacleTag>(
        getEntity());
  } else {
    registry.remove<comps::DeformableSurfaceCcdObstacleTag>(getEntity());
  }
}

//==============================================================================
bool RigidBody::isDeformableSurfaceCcdObstacle() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()
      ->getRegistry()
      .all_of<comps::DeformableSurfaceCcdObstacleTag>(getEntity());
}

//==============================================================================
std::optional<CollisionShape> RigidBody::getCollisionShape() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  if (const auto* geometry
      = registry.try_get<comps::CollisionGeometry>(getEntity())) {
    return geometry->shape;
  }
  return std::nullopt;
}

//==============================================================================
bool RigidBody::hasCollisionShape() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().all_of<comps::CollisionGeometry>(
      getEntity());
}

// getEntity() and isValid() inherited from Frame

} // namespace dart::simulation::experimental
