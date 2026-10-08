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

#pragma once

#include <dart/collision/dart/narrow_phase/ConvexConvex.hpp>
#include <dart/collision/dart/narrow_phase/Epa-impl.hpp>
#include <dart/collision/dart/narrow_phase/Mpr-impl.hpp>

namespace dart::collision::native::detail {

inline auto makeConvexSupportFunctionT(
    const ConvexShape& shape, const Eigen::Isometry3d& transform)
{
  return [&shape, transform](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport = shape.support(localDir);
    return transform * localSupport;
  };
}

inline auto makeMeshSupportFunctionT(
    const MeshShape& shape, const Eigen::Isometry3d& transform)
{
  return [&shape, transform](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport = shape.support(localDir);
    return transform * localSupport;
  };
}

inline auto makeSphereSupportFunctionT(
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

inline auto makeBoxSupportFunctionT(
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

inline auto makeCapsuleSupportFunctionT(
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

inline auto makeCylinderSupportFunctionT(
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

// Concrete support callable for the runtime shape dispatch.
struct ShapeSupport
{
  const Shape& shape;
  Eigen::Isometry3d transform;

  Eigen::Vector3d operator()(const Eigen::Vector3d& dir) const
  {
    switch (shape.getType()) {
      case ShapeType::Sphere:
        return makeSphereSupportFunctionT(
            static_cast<const SphereShape&>(shape), transform)(dir);
      case ShapeType::Box:
        return makeBoxSupportFunctionT(
            static_cast<const BoxShape&>(shape), transform)(dir);
      case ShapeType::Capsule:
        return makeCapsuleSupportFunctionT(
            static_cast<const CapsuleShape&>(shape), transform)(dir);
      case ShapeType::Cylinder:
        return makeCylinderSupportFunctionT(
            static_cast<const CylinderShape&>(shape), transform)(dir);
      case ShapeType::Convex:
        return makeConvexSupportFunctionT(
            static_cast<const ConvexShape&>(shape), transform)(dir);
      case ShapeType::Mesh:
        return makeMeshSupportFunctionT(
            static_cast<const MeshShape&>(shape), transform)(dir);
      default:
        return Eigen::Vector3d::Zero();
    }
  }
};

template <typename SupportA, typename SupportB>
bool collideSupportFunctionsT(
    const SupportA& supportA,
    const Eigen::Vector3d& centerA,
    const SupportB& supportB,
    const Eigen::Vector3d& centerB,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (option.maxNumContacts == 0) {
    return false;
  }

  if (option.enableContact && result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  Eigen::Vector3d initialDir = centerB - centerA;
  if (initialDir.squaredNorm() < 1e-10) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  GjkResult gjkResult = queryT(supportA, supportB, initialDir);

  if (!gjkResult.intersecting) {
    return false;
  }

  if (!option.enableContact) {
    return true;
  }

  EpaResult epaResult = penetrationT(supportA, supportB, gjkResult.simplex);
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
    MprResult mprResult
        = mpr::penetrationT(supportA, supportB, centerA, centerB);
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

} // namespace dart::collision::native::detail
