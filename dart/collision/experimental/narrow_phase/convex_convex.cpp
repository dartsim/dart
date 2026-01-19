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

#include <dart/collision/experimental/narrow_phase/convex_convex.hpp>

namespace dart::collision::experimental {

SupportFunction makeConvexSupportFunction(
    const ConvexShape& shape, const Eigen::Isometry3d& transform)
{
  return [&shape, &transform](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport = shape.support(localDir);
    return transform * localSupport;
  };
}

SupportFunction makeMeshSupportFunction(
    const MeshShape& shape, const Eigen::Isometry3d& transform)
{
  return [&shape, &transform](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport = shape.support(localDir);
    return transform * localSupport;
  };
}

SupportFunction makeSphereSupportFunction(
    const SphereShape& shape, const Eigen::Isometry3d& transform)
{
  double radius = shape.getRadius();
  Eigen::Vector3d center = transform.translation();
  return [radius, center](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    double len = dir.norm();
    if (len < 1e-10) {
      return center + Eigen::Vector3d(radius, 0, 0);
    }
    return Eigen::Vector3d(center + radius * dir / len);
  };
}

SupportFunction makeBoxSupportFunction(
    const BoxShape& shape, const Eigen::Isometry3d& transform)
{
  Eigen::Vector3d halfExtents = shape.getHalfExtents();
  return [halfExtents, &transform](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport;
    localSupport.x() = (localDir.x() >= 0) ? halfExtents.x() : -halfExtents.x();
    localSupport.y() = (localDir.y() >= 0) ? halfExtents.y() : -halfExtents.y();
    localSupport.z() = (localDir.z() >= 0) ? halfExtents.z() : -halfExtents.z();
    return transform * localSupport;
  };
}

SupportFunction makeCapsuleSupportFunction(
    const CapsuleShape& shape, const Eigen::Isometry3d& transform)
{
  double radius = shape.getRadius();
  double halfHeight = shape.getHeight() / 2.0;
  return [radius, halfHeight, &transform](
             const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    double len = localDir.norm();
    if (len < 1e-10) {
      return transform * Eigen::Vector3d(radius, 0, halfHeight);
    }
    Eigen::Vector3d dirNorm = localDir / len;
    Eigen::Vector3d axisPoint =
        (dirNorm.z() >= 0) ? Eigen::Vector3d(0, 0, halfHeight)
                           : Eigen::Vector3d(0, 0, -halfHeight);
    Eigen::Vector3d localSupport = axisPoint + radius * dirNorm;
    return transform * localSupport;
  };
}

SupportFunction makeCylinderSupportFunction(
    const CylinderShape& shape, const Eigen::Isometry3d& transform)
{
  double radius = shape.getRadius();
  double halfHeight = shape.getHeight() / 2.0;
  return [radius, halfHeight, &transform](
             const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport;
    double xyLen = std::sqrt(localDir.x() * localDir.x() + localDir.y() * localDir.y());
    if (xyLen < 1e-10) {
      localSupport.x() = radius;
      localSupport.y() = 0;
    } else {
      localSupport.x() = radius * localDir.x() / xyLen;
      localSupport.y() = radius * localDir.y() / xyLen;
    }
    localSupport.z() = (localDir.z() >= 0) ? halfHeight : -halfHeight;
    return transform * localSupport;
  };
}

namespace {

SupportFunction makeSupportFunction(
    const Shape& shape, const Eigen::Isometry3d& transform)
{
  switch (shape.getType()) {
    case ShapeType::Sphere:
      return makeSphereSupportFunction(
          static_cast<const SphereShape&>(shape), transform);
    case ShapeType::Box:
      return makeBoxSupportFunction(
          static_cast<const BoxShape&>(shape), transform);
    case ShapeType::Capsule:
      return makeCapsuleSupportFunction(
          static_cast<const CapsuleShape&>(shape), transform);
    case ShapeType::Cylinder:
      return makeCylinderSupportFunction(
          static_cast<const CylinderShape&>(shape), transform);
    case ShapeType::Convex:
      return makeConvexSupportFunction(
          static_cast<const ConvexShape&>(shape), transform);
    case ShapeType::Mesh:
      return makeMeshSupportFunction(
          static_cast<const MeshShape&>(shape), transform);
    default:
      return [](const Eigen::Vector3d&) { return Eigen::Vector3d::Zero(); };
  }
}

}  // namespace

bool collideConvexConvex(
    const Shape& shape1,
    const Eigen::Isometry3d& tf1,
    const Shape& shape2,
    const Eigen::Isometry3d& tf2,
    CollisionResult& result,
    const CollisionOption& option)
{
  auto supportA = makeSupportFunction(shape1, tf1);
  auto supportB = makeSupportFunction(shape2, tf2);

  Eigen::Vector3d initialDir = tf2.translation() - tf1.translation();
  if (initialDir.squaredNorm() < 1e-10) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  GjkResult gjkResult = Gjk::query(supportA, supportB, initialDir);

  if (!gjkResult.intersecting) {
    return false;
  }

  if (option.maxNumContacts == 0) {
    return true;
  }

  ContactPoint contact;
  contact.depth = 0.01;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.position = (tf1.translation() + tf2.translation()) / 2.0;

  result.addContact(contact);
  return true;
}

}  // namespace dart::collision::experimental
