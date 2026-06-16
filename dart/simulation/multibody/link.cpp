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

#include "dart/simulation/multibody/link.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/all.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/variational/discrete_mechanics_math.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/multibody/joint.hpp"
#include "dart/simulation/world.hpp"

#include <Eigen/Cholesky>

#include <optional>

#include <cmath>
#include <cstddef>

namespace dart::simulation {

namespace {

//==============================================================================
bool isSymmetricPositiveDefinite(const Eigen::Matrix3d& matrix)
{
  if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-12)) {
    return false;
  }

  Eigen::LLT<Eigen::Matrix3d> factorization(matrix);
  return factorization.info() == Eigen::Success;
}

void validateCollisionShape(const CollisionShape& shape, const char* ownerName)
{
  switch (shape.type) {
    case CollisionShapeType::Sphere:
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(shape.radius) || shape.radius <= 0.0,
          InvalidArgumentException,
          "{} sphere collision shape radius must be positive and finite",
          ownerName);
      break;
    case CollisionShapeType::Box:
      DART_SIMULATION_THROW_T_IF(
          !shape.halfExtents.allFinite()
              || (shape.halfExtents.array() <= 0.0).any(),
          InvalidArgumentException,
          "{} box collision shape half extents must be positive and finite",
          ownerName);
      break;
    case CollisionShapeType::Capsule:
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(shape.radius) || shape.radius <= 0.0
              || !std::isfinite(shape.halfExtents.z())
              || shape.halfExtents.z() <= 0.0,
          InvalidArgumentException,
          "{} capsule collision shape radius and half-height must be positive "
          "and finite",
          ownerName);
      break;
    case CollisionShapeType::Cylinder:
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(shape.radius) || shape.radius <= 0.0
              || !std::isfinite(shape.halfExtents.z())
              || shape.halfExtents.z() <= 0.0,
          InvalidArgumentException,
          "{} cylinder collision shape radius and half-height must be "
          "positive and finite",
          ownerName);
      break;
    case CollisionShapeType::Plane:
      DART_SIMULATION_THROW_T_IF(
          !shape.normal.allFinite() || shape.normal.squaredNorm() <= 0.0
              || !std::isfinite(shape.offset),
          InvalidArgumentException,
          "{} plane collision shape normal must be finite and nonzero, and "
          "offset must be finite",
          ownerName);
      break;
    case CollisionShapeType::Mesh: {
      DART_SIMULATION_THROW_T_IF(
          shape.vertices.empty(),
          InvalidArgumentException,
          "{} mesh collision shape must contain at least one vertex",
          ownerName);
      DART_SIMULATION_THROW_T_IF(
          shape.triangles.empty(),
          InvalidArgumentException,
          "{} mesh collision shape must contain at least one triangle",
          ownerName);

      for (const Eigen::Vector3d& vertex : shape.vertices) {
        DART_SIMULATION_THROW_T_IF(
            !vertex.allFinite(),
            InvalidArgumentException,
            "{} mesh collision shape vertices must be finite",
            ownerName);
      }

      const int vertexCount = static_cast<int>(shape.vertices.size());
      for (const Eigen::Vector3i& triangle : shape.triangles) {
        DART_SIMULATION_THROW_T_IF(
            triangle.minCoeff() < 0 || triangle.maxCoeff() >= vertexCount,
            InvalidArgumentException,
            "{} mesh collision shape triangle indices are out of range",
            ownerName);
        DART_SIMULATION_THROW_T_IF(
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

  const Eigen::Matrix3d rotation = shape.localTransform.linear();
  DART_SIMULATION_THROW_T_IF(
      !shape.localTransform.matrix().allFinite()
          || !(rotation.transpose() * rotation)
                  .isApprox(Eigen::Matrix3d::Identity(), 1e-9)
          || std::abs(rotation.determinant() - 1.0) > 1e-9,
      InvalidArgumentException,
      "{} collision shape local transform must be a finite rigid transform",
      ownerName);
}

} // namespace

//==============================================================================
Link::Link(Entity entity, World* world) : Frame(entity, world) {}

//==============================================================================
std::string_view Link::getName() const
{
  const auto& linkModel
      = dart::simulation::detail::registryOf(*getWorld())
            .get<comps::LinkModel>(detail::toRegistryEntity(getEntity()));
  return linkModel.name;
}

//==============================================================================
Joint Link::getParentJoint() const
{
  const auto& linkModel
      = dart::simulation::detail::registryOf(*getWorld())
            .get<comps::LinkModel>(detail::toRegistryEntity(getEntity()));
  return Joint(detail::fromRegistryEntity(linkModel.parentJoint), getWorld());
}

//==============================================================================
double Link::getMass() const
{
  return dart::simulation::detail::registryOf(*getWorld())
      .get<comps::LinkModel>(detail::toRegistryEntity(getEntity()))
      .mass.mass;
}

//==============================================================================
void Link::setMass(double mass)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(mass) || mass <= 0.0,
      InvalidArgumentException,
      "Link mass must be positive and finite");

  dart::simulation::detail::registryOf(*getWorld())
      .get<comps::LinkModel>(detail::toRegistryEntity(getEntity()))
      .mass.mass = mass;
}

//==============================================================================
Eigen::Matrix3d Link::getInertia() const
{
  return dart::simulation::detail::registryOf(*getWorld())
      .get<comps::LinkModel>(detail::toRegistryEntity(getEntity()))
      .mass.inertia;
}

//==============================================================================
void Link::setInertia(const Eigen::Matrix3d& inertia)
{
  DART_SIMULATION_THROW_T_IF(
      !isSymmetricPositiveDefinite(inertia),
      InvalidArgumentException,
      "Link inertia must be symmetric positive definite");

  dart::simulation::detail::registryOf(*getWorld())
      .get<comps::LinkModel>(detail::toRegistryEntity(getEntity()))
      .mass.inertia = inertia;
}

//==============================================================================
Eigen::Vector3d Link::getCenterOfMass() const
{
  return dart::simulation::detail::registryOf(*getWorld())
      .get<comps::LinkModel>(detail::toRegistryEntity(getEntity()))
      .mass.localCenterOfMass;
}

//==============================================================================
void Link::setCenterOfMass(const Eigen::Vector3d& centerOfMass)
{
  DART_SIMULATION_THROW_T_IF(
      !centerOfMass.allFinite(),
      InvalidArgumentException,
      "Link center of mass must contain only finite values");

  dart::simulation::detail::registryOf(*getWorld())
      .get<comps::LinkModel>(detail::toRegistryEntity(getEntity()))
      .mass.localCenterOfMass = centerOfMass;
}

//==============================================================================
void Link::applyForce(
    const Eigen::Vector3d& force,
    const Eigen::Vector3d& point,
    bool forceInWorldFrame,
    bool pointInWorldFrame)
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid link handle");
  DART_SIMULATION_THROW_T_IF(
      !force.allFinite() || !point.allFinite(),
      InvalidArgumentException,
      "Link external force and point must contain only finite values");

  namespace dm = detail::variational;

  // Mirror legacy BodyNode::addExtForce: resolve the force and application
  // point into the link frame, build the wrench [torque; force] at the point,
  // then transport it to the link origin and accumulate in the link frame.
  const Eigen::Isometry3d& worldTransform = getWorldTransform();
  auto& linkControl
      = dart::simulation::detail::registryOf(*getWorld())
            .get<comps::LinkControl>(detail::toRegistryEntity(getEntity()));

  Eigen::Isometry3d pointTransform = Eigen::Isometry3d::Identity();
  pointTransform.translation()
      = pointInWorldFrame ? worldTransform.inverse() * point : point;

  Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
  wrench.tail<3>()
      = forceInWorldFrame ? worldTransform.linear().transpose() * force : force;

  linkControl.externalForce += dm::dAdInvT(pointTransform, wrench);
}

//==============================================================================
std::optional<CollisionShape> Link::getCollisionShape() const
{
  const auto& registry = dart::simulation::detail::registryOf(*getWorld());
  if (const auto* geometry = registry.try_get<comps::CollisionGeometry>(
          detail::toRegistryEntity(getEntity()))) {
    if (const auto* shape = geometry->getPrimaryShape()) {
      return *shape;
    }
  }
  return std::nullopt;
}

//==============================================================================
std::vector<CollisionShape> Link::getCollisionShapes() const
{
  const auto& registry = dart::simulation::detail::registryOf(*getWorld());
  if (const auto* geometry = registry.try_get<comps::CollisionGeometry>(
          detail::toRegistryEntity(getEntity()))) {
    return geometry->shapes;
  }
  return {};
}

//==============================================================================
void Link::setCollisionShape(const CollisionShape& shape)
{
  validateCollisionShape(shape, "Link");

  auto& registry = dart::simulation::detail::registryOf(*getWorld());
  const auto* existing = registry.try_get<comps::CollisionGeometry>(
      detail::toRegistryEntity(getEntity()));
  comps::CollisionGeometry geometry{{shape}};
  geometry.revision = existing ? existing->revision + 1 : 1;
  registry.emplace_or_replace<comps::CollisionGeometry>(
      detail::toRegistryEntity(getEntity()), geometry);
}

//==============================================================================
void Link::addCollisionShape(const CollisionShape& shape)
{
  validateCollisionShape(shape, "Link");

  auto& registry = dart::simulation::detail::registryOf(*getWorld());
  auto& geometry = registry.get_or_emplace<comps::CollisionGeometry>(
      detail::toRegistryEntity(getEntity()));
  geometry.shapes.push_back(shape);
  ++geometry.revision;
}

//==============================================================================
bool Link::hasCollisionShape() const
{
  const auto& registry = dart::simulation::detail::registryOf(*getWorld());
  const auto* geometry = registry.try_get<comps::CollisionGeometry>(
      detail::toRegistryEntity(getEntity()));
  return geometry != nullptr && geometry->hasShapes();
}

//==============================================================================
const Eigen::Isometry3d& Link::getWorldTransform() const
{
  return Frame::getTransform();
}

} // namespace dart::simulation
