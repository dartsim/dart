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

#include <dart/collision/native/narrow_phase/convex_convex.hpp>
#include <dart/collision/native/narrow_phase/cylinder_collision.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include <cmath>

namespace dart::collision::native {

namespace {

bool contactBudgetExhausted(
    std::size_t numContacts, const CollisionOption& option)
{
  if (option.maxNumContacts == 0u) {
    return true;
  }

  return option.enableContact && numContacts >= option.maxNumContacts;
}

Eigen::Vector3d chooseRadialDirection(const Eigen::Vector3d& axis)
{
  const Eigen::Vector3d reference = (std::abs(axis.x()) < 0.9)
                                        ? Eigen::Vector3d::UnitX()
                                        : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d radial = reference - axis * reference.dot(axis);
  const double radialNorm = radial.norm();
  if (radialNorm < 1e-10) {
    return Eigen::Vector3d::UnitY();
  }

  return radial / radialNorm;
}

bool addAxialCylinderBoxPatchContacts(
    double cylinderRadius,
    const Eigen::Isometry3d& cylinderTransform,
    double minX,
    double maxX,
    double minY,
    double maxY,
    double contactPlaneZ,
    const Eigen::Vector3d& normalLocal,
    double penetration,
    CollisionResult& result,
    const CollisionOption& option)
{
  const auto contactsBefore = result.numContacts();
  if (contactBudgetExhausted(contactsBefore, option)) {
    return false;
  }

  const double patchMinX = std::max(minX, -cylinderRadius);
  const double patchMaxX = std::min(maxX, cylinderRadius);
  const double patchMinY = std::max(minY, -cylinderRadius);
  const double patchMaxY = std::min(maxY, cylinderRadius);
  if (patchMaxX < patchMinX || patchMaxY < patchMinY) {
    return false;
  }
  if (!option.enableContact) {
    return true;
  }

  ContactManifold manifold;
  manifold.setType(ContactType::Patch);

  const auto normalWorld = cylinderTransform.rotation() * normalLocal;
  const double contactZ = contactPlaneZ + normalLocal.z() * penetration * 0.5;
  const double radiusSq = cylinderRadius * cylinderRadius;
  const double duplicateThresholdSq = 1e-18;

  const auto addCandidate = [&](double x, double y) {
    if (x < patchMinX || x > patchMaxX || y < patchMinY || y > patchMaxY) {
      return;
    }
    if (x * x + y * y > radiusSq + 1e-12) {
      return;
    }

    const Eigen::Vector3d localPosition(x, y, contactZ);
    const Eigen::Vector3d worldPosition = cylinderTransform * localPosition;
    for (const auto& contact : manifold.getContacts()) {
      if ((contact.position - worldPosition).squaredNorm()
          <= duplicateThresholdSq) {
        return;
      }
    }

    ContactPoint contact;
    contact.position = worldPosition;
    contact.normal = normalWorld;
    contact.depth = penetration;
    manifold.addContact(contact);
  };

  addCandidate(patchMinX, patchMinY);
  addCandidate(patchMinX, patchMaxY);
  addCandidate(patchMaxX, patchMinY);
  addCandidate(patchMaxX, patchMaxY);

  const double inscribed = cylinderRadius / std::sqrt(2.0);
  addCandidate(-inscribed, -inscribed);
  addCandidate(-inscribed, inscribed);
  addCandidate(inscribed, -inscribed);
  addCandidate(inscribed, inscribed);

  addCandidate(-cylinderRadius, 0.0);
  addCandidate(cylinderRadius, 0.0);
  addCandidate(0.0, -cylinderRadius);
  addCandidate(0.0, cylinderRadius);
  addCandidate(
      std::clamp(0.0, patchMinX, patchMaxX),
      std::clamp(0.0, patchMinY, patchMaxY));

  if (!manifold.hasContacts()) {
    return false;
  }

  const auto remaining = option.maxNumContacts - contactsBefore;
  if (manifold.numContacts() > remaining) {
    ContactManifold limited;
    limited.setType(remaining == 1 ? ContactType::Point : ContactType::Patch);
    for (const auto& contact : manifold.getContacts()) {
      if (limited.numContacts() >= remaining) {
        break;
      }
      limited.addContact(contact);
    }
    result.addManifold(std::move(limited));
    return true;
  }

  if (manifold.numContacts() == 1) {
    manifold.setType(ContactType::Point);
  }
  result.addManifold(std::move(manifold));
  return true;
}

bool collideCylinderPlaneLikeBox(
    double cylinderRadius,
    double cylinderHalfHeight,
    const Eigen::Isometry3d& cylinderTransform,
    const Eigen::Vector3d& boxHalfExtents,
    const Eigen::Isometry3d& boxTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  constexpr double planeLikeBoxExtentScale = 100.0;
  constexpr double planeLikeBoxPatchDepth = 1e-5;

  if (contactBudgetExhausted(result.numContacts(), option)) {
    return false;
  }

  const double cylinderExtent = std::max(cylinderRadius, cylinderHalfHeight);
  if (boxHalfExtents.minCoeff() <= cylinderExtent * planeLikeBoxExtentScale) {
    return false;
  }

  const Eigen::Vector3d cylinderCenter = cylinderTransform.translation();
  const Eigen::Matrix3d boxRotation = boxTransform.rotation();

  double closestFaceDistance = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d faceNormal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d facePoint = Eigen::Vector3d::Zero();
  int faceAxis = 2;

  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      const Eigen::Vector3d normal = boxRotation.col(axis) * sign;
      const Eigen::Vector3d point
          = boxTransform.translation() + normal * boxHalfExtents[axis];
      const double distance = normal.dot(cylinderCenter - point);
      if (distance > closestFaceDistance) {
        closestFaceDistance = distance;
        faceNormal = normal;
        facePoint = point;
        faceAxis = axis;
      }
    }
  }

  const Eigen::Vector3d cylinderAxis = cylinderTransform.rotation().col(2);
  const double axisDot = cylinderAxis.dot(faceNormal);

  Eigen::Vector3d supportOffset
      = cylinderAxis
        * (axisDot >= 0.0 ? -cylinderHalfHeight : cylinderHalfHeight);
  const Eigen::Vector3d radial = faceNormal - cylinderAxis * axisDot;
  if (radial.squaredNorm() > 1e-20) {
    supportOffset -= radial.normalized() * cylinderRadius;
  }

  const Eigen::Vector3d deepestPoint = cylinderCenter + supportOffset;
  const double signedDistance = faceNormal.dot(deepestPoint - facePoint);
  if (signedDistance > 0.0) {
    return false;
  }

  const Eigen::Vector3d deepestPointInBox
      = boxTransform.inverse() * deepestPoint;
  const double boundsTolerance = cylinderExtent + 1e-9;
  for (int axis = 0; axis < 3; ++axis) {
    if (axis == faceAxis) {
      continue;
    }
    if (std::abs(deepestPointInBox[axis])
        > boxHalfExtents[axis] + boundsTolerance) {
      return false;
    }
  }
  if (!option.enableContact) {
    return true;
  }

  // Keep a tiny active patch when a large box is standing in for a plane.
  const double penetration = std::max(-signedDistance, planeLikeBoxPatchDepth);
  ContactPoint contact;
  contact.position = deepestPoint + faceNormal * (penetration * 0.5);
  contact.normal = faceNormal;
  contact.depth = penetration;

  ContactManifold manifold;
  manifold.setType(ContactType::Patch);
  manifold.addContact(contact);

  const Eigen::Vector3d radialOffset
      = supportOffset - cylinderAxis * supportOffset.dot(cylinderAxis);
  const Eigen::Vector3d axialOffset = supportOffset - radialOffset;
  if (radialOffset.squaredNorm() > 1e-20
      && result.numContacts() + manifold.numContacts()
             < option.maxNumContacts) {
    contact.position = cylinderCenter + axialOffset - radialOffset
                       + faceNormal * (penetration * 0.5);
    manifold.addContact(contact);
  }

  if (manifold.numContacts() == 1) {
    manifold.setType(ContactType::Point);
  }
  result.addManifold(std::move(manifold));
  return true;
}

bool collideParallelCylinders(
    double r1,
    double h1,
    const Eigen::Vector3d& axis1,
    const Eigen::Vector3d& center1,
    double r2,
    double h2,
    const Eigen::Vector3d& center2,
    CollisionResult& result,
    const CollisionOption& option)
{
  constexpr double eps = 1e-10;

  const Eigen::Vector3d centerDelta = center2 - center1;
  const double axialOffset = centerDelta.dot(axis1);
  const Eigen::Vector3d lateralOffset = centerDelta - axis1 * axialOffset;
  const double lateralDist = lateralOffset.norm();

  const double axialOverlap = h1 + h2 - std::abs(axialOffset);
  const double lateralOverlap = r1 + r2 - lateralDist;
  if (axialOverlap < -eps || lateralOverlap < -eps) {
    return false;
  }
  if (!option.enableContact) {
    return true;
  }

  const bool useAxialContact = axialOverlap <= lateralOverlap;

  ContactPoint contact;
  if (useAxialContact) {
    const double direction = axialOffset >= 0.0 ? 1.0 : -1.0;
    const Eigen::Vector3d normal = -direction * axis1;

    const Eigen::Vector3d cap1 = center1 + axis1 * (direction * h1);
    const Eigen::Vector3d cap2 = center2 - axis1 * (direction * h2);

    contact.position = (cap1 + cap2) * 0.5;
    contact.normal = normal;
    contact.depth = std::max(0.0, axialOverlap);
  } else {
    const Eigen::Vector3d lateralDir = lateralDist < eps
                                           ? chooseRadialDirection(axis1)
                                           : lateralOffset / lateralDist;
    const Eigen::Vector3d normal = -lateralDir;

    const double min1 = -h1;
    const double max1 = h1;
    const double min2 = axialOffset - h2;
    const double max2 = axialOffset + h2;
    const double contactAxial
        = 0.5 * (std::max(min1, min2) + std::min(max1, max2));

    const Eigen::Vector3d point1
        = center1 + axis1 * contactAxial + lateralDir * r1;
    const Eigen::Vector3d point2
        = center1 + axis1 * contactAxial + lateralOffset - lateralDir * r2;

    contact.position = (point1 + point2) * 0.5;
    contact.normal = normal;
    contact.depth = std::max(0.0, lateralOverlap);
  }

  result.addContact(contact);
  return true;
}

} // namespace

bool collideCylinders(
    const CylinderShape& cyl1,
    const Eigen::Isometry3d& transform1,
    const CylinderShape& cyl2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (contactBudgetExhausted(result.numContacts(), option)) {
    return false;
  }

  const double r1 = cyl1.getRadius();
  const double h1 = cyl1.getHeight() * 0.5;
  const double r2 = cyl2.getRadius();
  const double h2 = cyl2.getHeight() * 0.5;

  const Eigen::Vector3d axis1 = transform1.rotation().col(2);
  const Eigen::Vector3d axis2 = transform2.rotation().col(2);
  const Eigen::Vector3d center1 = transform1.translation();
  const Eigen::Vector3d center2 = transform2.translation();

  if (std::abs(axis1.dot(axis2)) > 1.0 - 1e-6) {
    return collideParallelCylinders(
        r1, h1, axis1, center1, r2, h2, center2, result, option);
  }

  return collideConvexConvex(
      cyl1, transform1, cyl2, transform2, result, option);
}

void collideCylindersBatch(
    span<const CylinderPair> pairs,
    span<CollisionResult> results,
    const CollisionOption& option)
{
  if (results.size() < pairs.size()) {
    throw std::invalid_argument(
        "collideCylindersBatch requires one result for each pair");
  }

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    if (pair.shapeA == nullptr || pair.shapeB == nullptr) {
      throw std::invalid_argument(
          "collideCylindersBatch received a null cylinder");
    }

    [[maybe_unused]] const bool collided = collideCylinders(
        *pair.shapeA, pair.tfA, *pair.shapeB, pair.tfB, results[i], option);
  }
}

bool collideCylinderSphere(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (contactBudgetExhausted(result.numContacts(), option)) {
    return false;
  }

  const double cylRadius = cylinder.getRadius();
  const double cylHalfHeight = cylinder.getHeight() * 0.5;
  const double sphereRadius = sphere.getRadius();

  const Eigen::Isometry3d cylInv = cylinderTransform.inverse();
  const Eigen::Vector3d sphereCenterLocal
      = cylInv * sphereTransform.translation();
  const double lateralDistSq = sphereCenterLocal.x() * sphereCenterLocal.x()
                               + sphereCenterLocal.y() * sphereCenterLocal.y();
  const double lateralDist = std::sqrt(lateralDistSq);
  const bool insideCylinder = lateralDist <= cylRadius
                              && sphereCenterLocal.z() >= -cylHalfHeight
                              && sphereCenterLocal.z() <= cylHalfHeight;

  if (insideCylinder) {
    const double distanceToTop = cylHalfHeight - sphereCenterLocal.z();
    const double distanceToBottom = cylHalfHeight + sphereCenterLocal.z();
    const double distanceToBarrel = cylRadius - lateralDist;

    Eigen::Vector3d closestLocal;
    Eigen::Vector3d normalLocal;
    double distanceToSurface = distanceToTop;

    if (distanceToTop <= distanceToBottom
        && distanceToTop <= distanceToBarrel) {
      closestLocal = Eigen::Vector3d(
          sphereCenterLocal.x(), sphereCenterLocal.y(), cylHalfHeight);
      normalLocal = -Eigen::Vector3d::UnitZ();
    } else if (distanceToBottom <= distanceToBarrel) {
      closestLocal = Eigen::Vector3d(
          sphereCenterLocal.x(), sphereCenterLocal.y(), -cylHalfHeight);
      normalLocal = Eigen::Vector3d::UnitZ();
      distanceToSurface = distanceToBottom;
    } else {
      const Eigen::Vector3d radialDirection
          = lateralDist > 1e-10 ? Eigen::Vector3d(
                sphereCenterLocal.x() / lateralDist,
                sphereCenterLocal.y() / lateralDist,
                0.0)
                                : Eigen::Vector3d::UnitX();
      closestLocal = Eigen::Vector3d(
          radialDirection.x() * cylRadius,
          radialDirection.y() * cylRadius,
          sphereCenterLocal.z());
      normalLocal = -radialDirection;
      distanceToSurface = distanceToBarrel;
    }

    const double penetration = sphereRadius + distanceToSurface;
    if (!option.enableContact) {
      return true;
    }

    const Eigen::Vector3d normalWorld
        = cylinderTransform.rotation() * normalLocal;
    const Eigen::Vector3d closestWorld = cylinderTransform * closestLocal;
    const Eigen::Vector3d contactPoint
        = closestWorld + normalWorld * (penetration * 0.5);

    ContactPoint contact;
    contact.position = contactPoint;
    contact.normal = normalWorld;
    contact.depth = penetration;

    result.addContact(contact);
    return true;
  }

  const double clampedZ
      = std::clamp(sphereCenterLocal.z(), -cylHalfHeight, cylHalfHeight);

  Eigen::Vector3d closestLocal;

  if (lateralDistSq <= cylRadius * cylRadius) {
    closestLocal = Eigen::Vector3d(
        sphereCenterLocal.x(), sphereCenterLocal.y(), clampedZ);
  } else {
    const double scale = cylRadius / lateralDist;
    closestLocal = Eigen::Vector3d(
        sphereCenterLocal.x() * scale, sphereCenterLocal.y() * scale, clampedZ);
  }

  const Eigen::Vector3d diff = sphereCenterLocal - closestLocal;
  const double distSq = diff.squaredNorm();

  if (distSq > sphereRadius * sphereRadius) {
    return false;
  }

  const double dist = std::sqrt(distSq);
  const double penetration = sphereRadius - dist;
  if (!option.enableContact) {
    return true;
  }

  Eigen::Vector3d normalLocal;
  if (dist < 1e-10) {
    if (std::abs(sphereCenterLocal.z()) > cylHalfHeight - 1e-6) {
      normalLocal
          = Eigen::Vector3d(0, 0, sphereCenterLocal.z() > 0 ? 1.0 : -1.0);
    } else {
      normalLocal = Eigen::Vector3d(1, 0, 0);
    }
  } else {
    normalLocal = diff / dist;
  }

  const Eigen::Vector3d normalWorld
      = cylinderTransform.rotation() * normalLocal;
  const Eigen::Vector3d closestWorld = cylinderTransform * closestLocal;
  const Eigen::Vector3d contactPoint
      = closestWorld + normalWorld * (penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = -normalWorld;
  contact.depth = penetration;

  result.addContact(contact);
  return true;
}

bool collideCylinderBox(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (contactBudgetExhausted(result.numContacts(), option)) {
    return false;
  }

  const double cylRadius = cylinder.getRadius();
  const double cylHalfHeight = cylinder.getHeight() * 0.5;
  const Eigen::Vector3d& boxHalf = box.getHalfExtents();

  const Eigen::Isometry3d cylInv = cylinderTransform.inverse();
  const Eigen::Isometry3d boxInCyl = cylInv * boxTransform;

  if (boxInCyl.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12)) {
    const Eigen::Vector3d center = boxInCyl.translation();
    const double minX = center.x() - boxHalf.x();
    const double maxX = center.x() + boxHalf.x();
    const double minY = center.y() - boxHalf.y();
    const double maxY = center.y() + boxHalf.y();
    const double minZ = center.z() - boxHalf.z();
    const double maxZ = center.z() + boxHalf.z();

    const double zOverlap
        = std::min(cylHalfHeight, maxZ) - std::max(-cylHalfHeight, minZ);
    if (zOverlap < 0.0) {
      return false;
    }
    const double topAxialPen = cylHalfHeight - minZ;
    const double bottomAxialPen = maxZ + cylHalfHeight;
    const bool topAxialContact = topAxialPen <= bottomAxialPen;
    const double axialPen = topAxialContact ? topAxialPen : bottomAxialPen;

    const double closestX = std::clamp(0.0, minX, maxX);
    const double closestY = std::clamp(0.0, minY, maxY);
    const double lateralDistSq = closestX * closestX + closestY * closestY;
    if (lateralDistSq > cylRadius * cylRadius) {
      return false;
    }

    const double lateralDist = std::sqrt(lateralDistSq);
    const bool containsCylinderAxis
        = minX <= 0.0 && maxX >= 0.0 && minY <= 0.0 && maxY >= 0.0;
    const double distToMinX = std::abs(minX);
    const double distToMaxX = std::abs(maxX);
    const double distToMinY = std::abs(minY);
    const double distToMaxY = std::abs(maxY);
    const double minSideDist = std::min(
        std::min(distToMinX, distToMaxX), std::min(distToMinY, distToMaxY));
    const double lateralPen = containsCylinderAxis ? cylRadius + minSideDist
                                                   : cylRadius - lateralDist;

    double penetration = lateralPen;
    Eigen::Vector3d normalLocal;
    Eigen::Vector3d contactLocal(
        closestX, closestY, std::clamp(0.0, minZ, maxZ));

    if (axialPen < lateralPen) {
      penetration = axialPen;
      normalLocal = Eigen::Vector3d(0, 0, topAxialContact ? -1.0 : 1.0);
      contactLocal.z() = topAxialContact ? minZ : maxZ;
      return addAxialCylinderBoxPatchContacts(
          cylRadius,
          cylinderTransform,
          minX,
          maxX,
          minY,
          maxY,
          contactLocal.z(),
          normalLocal,
          penetration,
          result,
          option);
    } else if (lateralDist > 1e-10) {
      normalLocal = Eigen::Vector3d(
          -closestX / lateralDist, -closestY / lateralDist, 0.0);
    } else {
      if (minSideDist == distToMinX) {
        normalLocal = Eigen::Vector3d(1, 0, 0);
        contactLocal.x() = minX;
      } else if (minSideDist == distToMaxX) {
        normalLocal = Eigen::Vector3d(-1, 0, 0);
        contactLocal.x() = maxX;
      } else if (minSideDist == distToMinY) {
        normalLocal = Eigen::Vector3d(0, 1, 0);
        contactLocal.y() = minY;
      } else {
        normalLocal = Eigen::Vector3d(0, -1, 0);
        contactLocal.y() = maxY;
      }
    }

    if (!option.enableContact) {
      return true;
    }

    const Eigen::Vector3d normalWorld
        = cylinderTransform.rotation() * normalLocal;
    const Eigen::Vector3d contactWorld
        = cylinderTransform * contactLocal + normalWorld * (penetration * 0.5);

    ContactPoint contact;
    contact.position = contactWorld;
    contact.normal = normalWorld;
    contact.depth = penetration;

    result.addContact(contact);
    return true;
  }

  double maxPenetration = -std::numeric_limits<double>::max();
  Eigen::Vector3d bestContactPoint;
  Eigen::Vector3d bestNormal;
  bool foundCollision = false;

  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d localCorner(
        (i & 1) ? boxHalf.x() : -boxHalf.x(),
        (i & 2) ? boxHalf.y() : -boxHalf.y(),
        (i & 4) ? boxHalf.z() : -boxHalf.z());

    const Eigen::Vector3d cornerInCyl = boxInCyl * localCorner;

    const double lateralDistSq
        = cornerInCyl.x() * cornerInCyl.x() + cornerInCyl.y() * cornerInCyl.y();
    const double lateralDist = std::sqrt(lateralDistSq);

    const double lateralPen = cylRadius - lateralDist;
    const double axialPen = cylHalfHeight - std::abs(cornerInCyl.z());

    if (lateralPen > 0.0 && axialPen > 0.0) {
      foundCollision = true;

      double penetration;
      Eigen::Vector3d normalLocal;

      if (lateralPen < axialPen) {
        penetration = lateralPen;
        if (lateralDist > 1e-10) {
          normalLocal = Eigen::Vector3d(
              -cornerInCyl.x() / lateralDist,
              -cornerInCyl.y() / lateralDist,
              0);
        } else {
          normalLocal = Eigen::Vector3d(-1, 0, 0);
        }
      } else {
        penetration = axialPen;
        normalLocal = Eigen::Vector3d(0, 0, cornerInCyl.z() > 0 ? -1.0 : 1.0);
      }

      if (penetration > maxPenetration) {
        maxPenetration = penetration;
        bestContactPoint = cornerInCyl;
        bestNormal = normalLocal;
      }
    }
  }

  if (!foundCollision) {
    if (collideCylinderPlaneLikeBox(
            cylRadius,
            cylHalfHeight,
            cylinderTransform,
            boxHalf,
            boxTransform,
            result,
            option)) {
      return true;
    }

    return collideConvexConvex(
        cylinder, cylinderTransform, box, boxTransform, result, option);
  }

  if (!option.enableContact) {
    return true;
  }

  const Eigen::Vector3d normalWorld = cylinderTransform.rotation() * bestNormal;
  const Eigen::Vector3d contactWorld = cylinderTransform * bestContactPoint
                                       + normalWorld * (maxPenetration * 0.5);

  ContactPoint contact;
  contact.position = contactWorld;
  contact.normal = normalWorld;
  contact.depth = maxPenetration;

  result.addContact(contact);
  return true;
}

bool collideCylinderCapsule(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (contactBudgetExhausted(result.numContacts(), option)) {
    return false;
  }

  const double cylRadius = cylinder.getRadius();
  const double cylHalfHeight = cylinder.getHeight() * 0.5;
  const double capRadius = capsule.getRadius();
  const double capHalfHeight = capsule.getHeight() * 0.5;

  const Eigen::Vector3d capAxis = capsuleTransform.rotation().col(2);
  const Eigen::Vector3d capCenter = capsuleTransform.translation();
  const Eigen::Vector3d capTop = capCenter + capAxis * capHalfHeight;
  const Eigen::Vector3d capBot = capCenter - capAxis * capHalfHeight;

  const Eigen::Isometry3d cylInv = cylinderTransform.inverse();
  const Eigen::Vector3d capTopLocal = cylInv * capTop;
  const Eigen::Vector3d capBotLocal = cylInv * capBot;

  double bestPenetration = -std::numeric_limits<double>::max();
  Eigen::Vector3d bestCylPoint;
  Eigen::Vector3d bestCapPoint;
  bool foundCollision = false;
  bool bestEndpointInsideCylinder = false;

  auto checkCapsuleEndpoint = [&](const Eigen::Vector3d& capEndLocal) {
    const double lateralDistSq
        = capEndLocal.x() * capEndLocal.x() + capEndLocal.y() * capEndLocal.y();
    const double lateralDist = std::sqrt(lateralDistSq);

    Eigen::Vector3d closestOnCyl;
    double dist;
    const bool endpointInsideCylinder
        = std::abs(capEndLocal.z()) <= cylHalfHeight
          && lateralDist <= cylRadius;

    if (std::abs(capEndLocal.z()) <= cylHalfHeight) {
      if (lateralDist <= cylRadius) {
        double lateralPen = cylRadius - lateralDist;
        double axialPen = cylHalfHeight - std::abs(capEndLocal.z());
        if (lateralPen < axialPen && lateralDist > 1e-10) {
          closestOnCyl = Eigen::Vector3d(
              capEndLocal.x() * cylRadius / lateralDist,
              capEndLocal.y() * cylRadius / lateralDist,
              capEndLocal.z());
        } else {
          double cappedZ = capEndLocal.z() > 0 ? cylHalfHeight : -cylHalfHeight;
          closestOnCyl
              = Eigen::Vector3d(capEndLocal.x(), capEndLocal.y(), cappedZ);
        }
      } else {
        closestOnCyl = Eigen::Vector3d(
            capEndLocal.x() * cylRadius / lateralDist,
            capEndLocal.y() * cylRadius / lateralDist,
            capEndLocal.z());
      }
    } else {
      double cappedZ
          = std::clamp(capEndLocal.z(), -cylHalfHeight, cylHalfHeight);
      if (lateralDist <= cylRadius) {
        closestOnCyl
            = Eigen::Vector3d(capEndLocal.x(), capEndLocal.y(), cappedZ);
      } else {
        closestOnCyl = Eigen::Vector3d(
            capEndLocal.x() * cylRadius / lateralDist,
            capEndLocal.y() * cylRadius / lateralDist,
            cappedZ);
      }
    }

    dist = (capEndLocal - closestOnCyl).norm();
    double penetration
        = endpointInsideCylinder ? capRadius + dist : capRadius - dist;

    if (penetration > 0 && penetration > bestPenetration) {
      bestPenetration = penetration;
      bestCylPoint = closestOnCyl;
      bestCapPoint = capEndLocal;
      foundCollision = true;
      bestEndpointInsideCylinder = endpointInsideCylinder;
    }
  };

  checkCapsuleEndpoint(capTopLocal);
  checkCapsuleEndpoint(capBotLocal);

  if (!foundCollision) {
    return collideConvexConvex(
        cylinder, cylinderTransform, capsule, capsuleTransform, result, option);
  }

  if (!option.enableContact) {
    return true;
  }

  Eigen::Vector3d normalLocal = bestCapPoint - bestCylPoint;
  if (normalLocal.squaredNorm() < 1e-10) {
    normalLocal = Eigen::Vector3d::UnitZ();
  } else {
    normalLocal.normalize();
  }

  const Eigen::Vector3d normalWorld
      = cylinderTransform.rotation() * normalLocal;
  const Eigen::Vector3d contactWorld = cylinderTransform * bestCylPoint
                                       + normalWorld * (bestPenetration * 0.5);

  ContactPoint contact;
  contact.position = contactWorld;
  contact.normal = bestEndpointInsideCylinder ? normalWorld : -normalWorld;
  contact.depth = bestPenetration;

  result.addContact(contact);
  return true;
}

bool collideCylinderPlane(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const PlaneShape& plane,
    const Eigen::Isometry3d& planeTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (contactBudgetExhausted(result.numContacts(), option)) {
    return false;
  }

  const double cylRadius = cylinder.getRadius();
  const double cylHalfHeight = cylinder.getHeight() * 0.5;

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane.getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane.getOffset();

  const Eigen::Vector3d cylAxis = cylinderTransform.rotation().col(2);
  const Eigen::Vector3d cylCenter = cylinderTransform.translation();

  const Eigen::Vector3d cylTop = cylCenter + cylAxis * cylHalfHeight;
  const Eigen::Vector3d cylBot = cylCenter - cylAxis * cylHalfHeight;

  const double dotAxis = cylAxis.dot(worldNormal);
  const Eigen::Vector3d radialRaw = worldNormal - cylAxis * dotAxis;

  Eigen::Vector3d deepestPoint;
  if (radialRaw.squaredNorm() < 1e-10) {
    const double distTop = worldNormal.dot(cylTop - planePoint);
    const double distBot = worldNormal.dot(cylBot - planePoint);
    deepestPoint = (distTop < distBot) ? cylTop : cylBot;
  } else {
    const Eigen::Vector3d radialDir = radialRaw.normalized();
    const Eigen::Vector3d rimOffset = -radialDir * cylRadius;
    const Eigen::Vector3d topRim = cylTop + rimOffset;
    const Eigen::Vector3d botRim = cylBot + rimOffset;

    const double distTopRim = worldNormal.dot(topRim - planePoint);
    const double distBotRim = worldNormal.dot(botRim - planePoint);

    deepestPoint = (distTopRim < distBotRim) ? topRim : botRim;
  }

  const double signedDist = worldNormal.dot(deepestPoint - planePoint);

  if (signedDist > 0.0) {
    return false;
  }

  const double penetration = -signedDist;
  if (!option.enableContact) {
    return true;
  }

  const Eigen::Vector3d contactPoint
      = deepestPoint + worldNormal * (penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = worldNormal;
  contact.depth = penetration;

  result.addContact(contact);
  return true;
}

} // namespace dart::collision::native
