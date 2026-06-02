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
#include "dart/simulation/experimental/detail/entity_conversion.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <cstddef>

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
void validateCollisionShapeTransform(
    const dart::simulation::experimental::CollisionShape& shape)
{
  const Eigen::Matrix3d rotation = shape.localTransform.linear();
  DART_EXPERIMENTAL_THROW_T_IF(
      !shape.localTransform.matrix().allFinite()
          || !(rotation.transpose() * rotation)
                  .isApprox(Eigen::Matrix3d::Identity(), 1e-9)
          || std::abs(rotation.determinant() - 1.0) > 1e-9,
      dart::simulation::experimental::InvalidArgumentException,
      "Collision shape local transform must be a finite rigid transform");
}

//==============================================================================
void validateMeshCollisionShape(
    const dart::simulation::experimental::CollisionShape& shape)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      shape.vertices.empty() || shape.triangles.empty(),
      dart::simulation::experimental::InvalidArgumentException,
      "Mesh collision shape must have at least one vertex and one triangle");

  for (const auto& vertex : shape.vertices) {
    DART_EXPERIMENTAL_THROW_T_IF(
        !vertex.allFinite(),
        dart::simulation::experimental::InvalidArgumentException,
        "Mesh collision shape vertices must be finite");
  }

  const auto vertexCount = static_cast<int>(shape.vertices.size());
  for (const auto& triangle : shape.triangles) {
    DART_EXPERIMENTAL_THROW_T_IF(
        triangle.minCoeff() < 0 || triangle.maxCoeff() >= vertexCount,
        dart::simulation::experimental::InvalidArgumentException,
        "Mesh collision shape triangle indices must reference vertices");
    DART_EXPERIMENTAL_THROW_T_IF(
        triangle.x() == triangle.y() || triangle.x() == triangle.z()
            || triangle.y() == triangle.z(),
        dart::simulation::experimental::InvalidArgumentException,
        "Mesh collision shape triangles must use three distinct vertices");
  }
}

//==============================================================================
void validateCollisionShape(
    const dart::simulation::experimental::CollisionShape& shape,
    const char* ownerName)
{
  switch (shape.type) {
    case dart::simulation::experimental::CollisionShapeType::Sphere:
      DART_EXPERIMENTAL_THROW_T_IF(
          !std::isfinite(shape.radius) || shape.radius <= 0.0,
          dart::simulation::experimental::InvalidArgumentException,
          "{} sphere collision shape radius must be positive and finite",
          ownerName);
      break;
    case dart::simulation::experimental::CollisionShapeType::Box:
      DART_EXPERIMENTAL_THROW_T_IF(
          !shape.halfExtents.allFinite()
              || (shape.halfExtents.array() <= 0.0).any(),
          dart::simulation::experimental::InvalidArgumentException,
          "{} box collision shape half extents must be positive and finite",
          ownerName);
      break;
    case dart::simulation::experimental::CollisionShapeType::Capsule:
      DART_EXPERIMENTAL_THROW_T_IF(
          !std::isfinite(shape.radius) || shape.radius <= 0.0
              || !std::isfinite(shape.halfExtents.z())
              || shape.halfExtents.z() <= 0.0,
          dart::simulation::experimental::InvalidArgumentException,
          "{} capsule collision shape radius and half-height must be positive "
          "and finite",
          ownerName);
      break;
    case dart::simulation::experimental::CollisionShapeType::Cylinder:
      DART_EXPERIMENTAL_THROW_T_IF(
          !std::isfinite(shape.radius) || shape.radius <= 0.0
              || !std::isfinite(shape.halfExtents.z())
              || shape.halfExtents.z() <= 0.0,
          dart::simulation::experimental::InvalidArgumentException,
          "{} cylinder collision shape radius and half-height must be positive "
          "and finite",
          ownerName);
      break;
    case dart::simulation::experimental::CollisionShapeType::Plane:
      DART_EXPERIMENTAL_THROW_T_IF(
          !shape.normal.allFinite() || shape.normal.squaredNorm() <= 0.0
              || !std::isfinite(shape.offset),
          dart::simulation::experimental::InvalidArgumentException,
          "{} plane collision shape normal must be finite and nonzero, and "
          "offset must be finite",
          ownerName);
      break;
    case dart::simulation::experimental::CollisionShapeType::Mesh:
      validateMeshCollisionShape(shape);
      break;
  }
  validateCollisionShapeTransform(shape);
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
RigidBody::RigidBody(Entity entity, World* world) : Frame(entity, world)
{
  // Frame base constructor handles entity and world
}

//==============================================================================
std::string RigidBody::getName() const
{
  const auto& registry = getWorld()->getRegistry();
  if (const auto* name
      = registry.try_get<comps::Name>(detail::toRegistryEntity(getEntity()))) {
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
  auto& freeFrame = registry.get<comps::FreeFrameProperties>(
      detail::toRegistryEntity(getEntity()));
  auto& rigidTransform
      = registry.get<comps::Transform>(detail::toRegistryEntity(getEntity()));

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

  return getWorld()
      ->getRegistry()
      .get<comps::Velocity>(detail::toRegistryEntity(getEntity()))
      .linear;
}

//==============================================================================
void RigidBody::setLinearVelocity(const Eigen::Vector3d& velocity)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(velocity, "linear velocity");

  getWorld()
      ->getRegistry()
      .get<comps::Velocity>(detail::toRegistryEntity(getEntity()))
      .linear = velocity;
}

//==============================================================================
Eigen::Vector3d RigidBody::getAngularVelocity() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()
      ->getRegistry()
      .get<comps::Velocity>(detail::toRegistryEntity(getEntity()))
      .angular;
}

//==============================================================================
void RigidBody::setAngularVelocity(const Eigen::Vector3d& velocity)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(velocity, "angular velocity");

  getWorld()
      ->getRegistry()
      .get<comps::Velocity>(detail::toRegistryEntity(getEntity()))
      .angular = velocity;
}

//==============================================================================
double RigidBody::getMass() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()
      ->getRegistry()
      .get<comps::MassProperties>(detail::toRegistryEntity(getEntity()))
      .mass;
}

//==============================================================================
void RigidBody::setMass(double mass)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateMass(mass);

  getWorld()
      ->getRegistry()
      .get<comps::MassProperties>(detail::toRegistryEntity(getEntity()))
      .mass = mass;
}

//==============================================================================
Eigen::Matrix3d RigidBody::getInertia() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()
      ->getRegistry()
      .get<comps::MassProperties>(detail::toRegistryEntity(getEntity()))
      .inertia;
}

//==============================================================================
void RigidBody::setInertia(const Eigen::Matrix3d& inertia)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateInertia(inertia);

  getWorld()
      ->getRegistry()
      .get<comps::MassProperties>(detail::toRegistryEntity(getEntity()))
      .inertia = inertia;
}

//==============================================================================
Eigen::Vector3d RigidBody::getForce() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()
      ->getRegistry()
      .get<comps::Force>(detail::toRegistryEntity(getEntity()))
      .force;
}

//==============================================================================
void RigidBody::setForce(const Eigen::Vector3d& force)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(force, "force");

  getWorld()
      ->getRegistry()
      .get<comps::Force>(detail::toRegistryEntity(getEntity()))
      .force = force;
}

//==============================================================================
void RigidBody::applyForce(const Eigen::Vector3d& force)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(force, "force");

  getWorld()
      ->getRegistry()
      .get<comps::Force>(detail::toRegistryEntity(getEntity()))
      .force += force;
}

//==============================================================================
void RigidBody::clearForce()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  getWorld()
      ->getRegistry()
      .get<comps::Force>(detail::toRegistryEntity(getEntity()))
      .force.setZero();
}

//==============================================================================
Eigen::Vector3d RigidBody::getTorque() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()
      ->getRegistry()
      .get<comps::Force>(detail::toRegistryEntity(getEntity()))
      .torque;
}

//==============================================================================
void RigidBody::setTorque(const Eigen::Vector3d& torque)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(torque, "torque");

  getWorld()
      ->getRegistry()
      .get<comps::Force>(detail::toRegistryEntity(getEntity()))
      .torque = torque;
}

//==============================================================================
void RigidBody::applyTorque(const Eigen::Vector3d& torque)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateFiniteVector(torque, "torque");

  getWorld()
      ->getRegistry()
      .get<comps::Force>(detail::toRegistryEntity(getEntity()))
      .torque += torque;
}

//==============================================================================
void RigidBody::clearTorque()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  getWorld()
      ->getRegistry()
      .get<comps::Force>(detail::toRegistryEntity(getEntity()))
      .torque.setZero();
}

//==============================================================================
Eigen::Vector3d RigidBody::getLinearMomentum() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  const auto& mass = registry.get<comps::MassProperties>(
      detail::toRegistryEntity(getEntity()));
  const auto& velocity
      = registry.get<comps::Velocity>(detail::toRegistryEntity(getEntity()));
  return mass.mass * velocity.linear;
}

//==============================================================================
Eigen::Vector3d RigidBody::getAngularMomentum() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  const auto& mass = registry.get<comps::MassProperties>(
      detail::toRegistryEntity(getEntity()));
  const auto& velocity
      = registry.get<comps::Velocity>(detail::toRegistryEntity(getEntity()));
  const auto& transform
      = registry.get<comps::Transform>(detail::toRegistryEntity(getEntity()));

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
  const auto& mass = registry.get<comps::MassProperties>(
      detail::toRegistryEntity(getEntity()));
  const auto& velocity
      = registry.get<comps::Velocity>(detail::toRegistryEntity(getEntity()));
  const auto& transform
      = registry.get<comps::Transform>(detail::toRegistryEntity(getEntity()));

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
  const auto& mass = registry.get<comps::MassProperties>(
      detail::toRegistryEntity(getEntity()));
  const auto& transform
      = registry.get<comps::Transform>(detail::toRegistryEntity(getEntity()));
  return -mass.mass * getWorld()->getGravity().dot(transform.position);
}

//==============================================================================
void RigidBody::setStatic(bool isStatic)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  auto& registry = getWorld()->getRegistry();
  if (isStatic) {
    registry.emplace_or_replace<comps::StaticBodyTag>(
        detail::toRegistryEntity(getEntity()));
  } else {
    registry.remove<comps::StaticBodyTag>(
        detail::toRegistryEntity(getEntity()));
  }
}

//==============================================================================
bool RigidBody::isStatic() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().all_of<comps::StaticBodyTag>(
      detail::toRegistryEntity(getEntity()));
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
  auto& material = registry.get_or_emplace<comps::ContactMaterial>(
      detail::toRegistryEntity(getEntity()));
  material.restitution = restitution;
}

//==============================================================================
double RigidBody::getRestitution() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  if (const auto* material = registry.try_get<comps::ContactMaterial>(
          detail::toRegistryEntity(getEntity()))) {
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
  auto& material = registry.get_or_emplace<comps::ContactMaterial>(
      detail::toRegistryEntity(getEntity()));
  material.friction = friction;
}

//==============================================================================
double RigidBody::getFriction() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  if (const auto* material = registry.try_get<comps::ContactMaterial>(
          detail::toRegistryEntity(getEntity()))) {
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
  const auto* existing = registry.try_get<comps::CollisionGeometry>(
      detail::toRegistryEntity(getEntity()));
  comps::CollisionGeometry geometry{{shape}};
  geometry.revision = existing ? existing->revision + 1 : 1;
  registry.emplace_or_replace<comps::CollisionGeometry>(
      detail::toRegistryEntity(getEntity()), geometry);
}

//==============================================================================
void RigidBody::addCollisionShape(const CollisionShape& shape)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  validateCollisionShape(shape, "RigidBody");

  auto& registry = getWorld()->getRegistry();
  auto& geometry = registry.get_or_emplace<comps::CollisionGeometry>(
      detail::toRegistryEntity(getEntity()));
  geometry.shapes.push_back(shape);
  ++geometry.revision;
}

//==============================================================================
void RigidBody::setDeformableGroundBarrier(bool enabled)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  auto& registry = getWorld()->getRegistry();
  if (enabled) {
    registry.get_or_emplace<comps::DeformableGroundBarrierTag>(
        detail::toRegistryEntity(getEntity()));
  } else {
    registry.remove<comps::DeformableGroundBarrierTag>(
        detail::toRegistryEntity(getEntity()));
  }
}

//==============================================================================
bool RigidBody::isDeformableGroundBarrier() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().all_of<comps::DeformableGroundBarrierTag>(
      detail::toRegistryEntity(getEntity()));
}

//==============================================================================
void RigidBody::setDeformableSurfaceCcdObstacle(bool enabled)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  auto& registry = getWorld()->getRegistry();
  if (enabled) {
    registry.get_or_emplace<comps::DeformableSurfaceCcdObstacleTag>(
        detail::toRegistryEntity(getEntity()));
  } else {
    registry.remove<comps::DeformableSurfaceCcdObstacleTag>(
        detail::toRegistryEntity(getEntity()));
  }
}

//==============================================================================
bool RigidBody::isDeformableSurfaceCcdObstacle() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()
      ->getRegistry()
      .all_of<comps::DeformableSurfaceCcdObstacleTag>(
          detail::toRegistryEntity(getEntity()));
}

//==============================================================================
void RigidBody::setDeformableObstacleBarrierOnly(bool enabled)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  auto& registry = getWorld()->getRegistry();
  if (enabled) {
    registry.get_or_emplace<comps::DeformableObstacleNoCcdTag>(
        detail::toRegistryEntity(getEntity()));
  } else {
    registry.remove<comps::DeformableObstacleNoCcdTag>(
        detail::toRegistryEntity(getEntity()));
  }
}

//==============================================================================
bool RigidBody::isDeformableObstacleBarrierOnly() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  return getWorld()->getRegistry().all_of<comps::DeformableObstacleNoCcdTag>(
      detail::toRegistryEntity(getEntity()));
}

//==============================================================================
std::optional<CollisionShape> RigidBody::getCollisionShape() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  if (const auto* geometry = registry.try_get<comps::CollisionGeometry>(
          detail::toRegistryEntity(getEntity()))) {
    if (const auto* shape = geometry->getPrimaryShape()) {
      return *shape;
    }
  }
  return std::nullopt;
}

//==============================================================================
std::vector<CollisionShape> RigidBody::getCollisionShapes() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  if (const auto* geometry = registry.try_get<comps::CollisionGeometry>(
          detail::toRegistryEntity(getEntity()))) {
    return geometry->shapes;
  }
  return {};
}

//==============================================================================
bool RigidBody::hasCollisionShape() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid rigid body handle");

  const auto& registry = getWorld()->getRegistry();
  const auto* geometry = registry.try_get<comps::CollisionGeometry>(
      detail::toRegistryEntity(getEntity()));
  return geometry != nullptr && geometry->hasShapes();
}

// detail::toRegistryEntity(getEntity()) and isValid() inherited from Frame

} // namespace dart::simulation::experimental
