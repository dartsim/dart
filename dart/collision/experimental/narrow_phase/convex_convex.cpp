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
#include <dart/collision/experimental/narrow_phase/mpr.hpp>

#include <limits>
#include <vector>

#include <cmath>

namespace dart::collision::experimental {

SupportFunction makeConvexSupportFunction(
    const ConvexShape& shape, const Eigen::Isometry3d& transform)
{
  return [&shape, transform](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport = shape.support(localDir);
    return transform * localSupport;
  };
}

SupportFunction makeMeshSupportFunction(
    const MeshShape& shape, const Eigen::Isometry3d& transform)
{
  return [&shape, transform](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
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
  return [halfExtents,
          transform](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
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
  return [radius, halfHeight, transform](
             const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    double len = localDir.norm();
    if (len < 1e-10) {
      return transform * Eigen::Vector3d(radius, 0, halfHeight);
    }
    Eigen::Vector3d dirNorm = localDir / len;
    Eigen::Vector3d axisPoint = (dirNorm.z() >= 0)
                                    ? Eigen::Vector3d(0, 0, halfHeight)
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
  return [radius, halfHeight, transform](
             const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport;
    double xyLen
        = std::sqrt(localDir.x() * localDir.x() + localDir.y() * localDir.y());
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
      return [](const Eigen::Vector3d&) {
        return Eigen::Vector3d::Zero();
      };
  }
}

Eigen::Vector3d averageVertexPosition(
    const std::vector<Eigen::Vector3d>& vertices)
{
  if (vertices.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& v : vertices) {
    sum += v;
  }
  return sum / static_cast<double>(vertices.size());
}

Eigen::Vector3d computeShapeCenter(
    const Shape& shape, const Eigen::Isometry3d& transform)
{
  switch (shape.getType()) {
    case ShapeType::Sphere:
    case ShapeType::Box:
    case ShapeType::Capsule:
    case ShapeType::Cylinder:
      return transform.translation();
    case ShapeType::Convex: {
      const auto& convex = static_cast<const ConvexShape&>(shape);
      return transform * averageVertexPosition(convex.getVertices());
    }
    case ShapeType::Mesh: {
      const auto& mesh = static_cast<const MeshShape&>(shape);
      return transform * averageVertexPosition(mesh.getVertices());
    }
    default:
      return transform.translation();
  }
}

} // namespace

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

  EpaResult epaResult = Epa::penetration(supportA, supportB, gjkResult.simplex);
  Eigen::Vector3d contactNormal = Eigen::Vector3d::Zero();
  double penetrationDepth = 0.0;
  Eigen::Vector3d pointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointB = Eigen::Vector3d::Zero();

  if (epaResult.success) {
    penetrationDepth = epaResult.depth;
    contactNormal = -epaResult.normal;
    pointA = epaResult.pointOnA;
    pointB = epaResult.pointOnB;
  } else {
    const Eigen::Vector3d centerA = computeShapeCenter(shape1, tf1);
    const Eigen::Vector3d centerB = computeShapeCenter(shape2, tf2);
    MprResult mprResult
        = Mpr::penetration(supportA, supportB, centerA, centerB);
    if (mprResult.success) {
      penetrationDepth = mprResult.depth;
      contactNormal = -mprResult.normal;
      pointA = mprResult.pointOnA;
      pointB = mprResult.pointOnB;
    }
  }

  if (contactNormal.squaredNorm() < 1e-12) {
    contactNormal = Eigen::Vector3d::UnitZ();
  } else {
    contactNormal.normalize();
  }

  if (penetrationDepth < 0.0) {
    penetrationDepth = -penetrationDepth;
  }

  ContactPoint contact;
  contact.depth = penetrationDepth;
  contact.normal = contactNormal;
  contact.position = (pointA + pointB) * 0.5;

  result.addContact(contact);
  return true;
}

double distanceConvexConvex(
    const Shape& shape1,
    const Eigen::Isometry3d& tf1,
    const Shape& shape2,
    const Eigen::Isometry3d& tf2,
    DistanceResult& result,
    const DistanceOption& option)
{
  auto supportA = makeSupportFunction(shape1, tf1);
  auto supportB = makeSupportFunction(shape2, tf2);

  Eigen::Vector3d initialDir = tf2.translation() - tf1.translation();
  if (initialDir.squaredNorm() < 1e-10) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  GjkResult gjkResult = Gjk::query(supportA, supportB, initialDir);

  if (!gjkResult.intersecting) {
    if (option.upperBound < gjkResult.distance) {
      result.distance = std::numeric_limits<double>::max();
      return std::numeric_limits<double>::max();
    }

    result.distance = gjkResult.distance;
    if (option.enableNearestPoints) {
      result.pointOnObject1 = gjkResult.closestPointA;
      result.pointOnObject2 = gjkResult.closestPointB;
      if (gjkResult.distance > 1e-12) {
        result.normal
            = (gjkResult.closestPointB - gjkResult.closestPointA).normalized();
      } else {
        result.normal = Eigen::Vector3d::UnitX();
      }
    }
    return gjkResult.distance;
  }

  EpaResult epaResult = Epa::penetration(supportA, supportB, gjkResult.simplex);
  double depth = 0.0;
  Eigen::Vector3d pointA = tf1.translation();
  Eigen::Vector3d pointB = tf2.translation();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitX();

  if (epaResult.success) {
    depth = epaResult.depth;
    pointA = epaResult.pointOnA;
    pointB = epaResult.pointOnB;
    if ((pointB - pointA).squaredNorm() > 1e-12) {
      normal = (pointB - pointA).normalized();
    }
  } else {
    const Eigen::Vector3d centerA = computeShapeCenter(shape1, tf1);
    const Eigen::Vector3d centerB = computeShapeCenter(shape2, tf2);
    MprResult mprResult
        = Mpr::penetration(supportA, supportB, centerA, centerB);
    if (mprResult.success) {
      depth = mprResult.depth;
      pointA = mprResult.pointOnA;
      pointB = mprResult.pointOnB;
      if ((pointB - pointA).squaredNorm() > 1e-12) {
        normal = (pointB - pointA).normalized();
      } else if (mprResult.normal.squaredNorm() > 1e-12) {
        normal = mprResult.normal.normalized();
      }
    }
  }

  result.distance = -depth;
  if (option.enableNearestPoints) {
    result.pointOnObject1 = pointA;
    result.pointOnObject2 = pointB;
    result.normal = normal;
  }

  return -depth;
}

} // namespace dart::collision::experimental
