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

#include <dart/collision/native/narrow_phase/CapsuleSphere.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::native {

namespace {

[[nodiscard]] bool hasIdentityRotation(const Eigen::Isometry3d& transform)
{
  const auto& rotation = transform.linear();
  return rotation(0, 0) == 1.0 && rotation(0, 1) == 0.0 && rotation(0, 2) == 0.0
         && rotation(1, 0) == 0.0 && rotation(1, 1) == 1.0
         && rotation(1, 2) == 0.0 && rotation(2, 0) == 0.0
         && rotation(2, 1) == 0.0 && rotation(2, 2) == 1.0;
}

Eigen::Vector3d closestPointOnSegment(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& segmentStart,
    const Eigen::Vector3d& segmentEnd)
{
  const Eigen::Vector3d segment = segmentEnd - segmentStart;
  const double segmentLengthSq = segment.squaredNorm();

  if (segmentLengthSq < 1e-10) {
    return segmentStart;
  }

  const double t = std::clamp(
      (point - segmentStart).dot(segment) / segmentLengthSq, 0.0, 1.0);
  return segmentStart + segment * t;
}

Eigen::Vector3d chooseRadialNormal(const Eigen::Vector3d& capsuleAxis)
{
  const double axisNorm = capsuleAxis.norm();
  if (axisNorm < 1e-10) {
    return Eigen::Vector3d::UnitX();
  }

  const Eigen::Vector3d axis = capsuleAxis / axisNorm;
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

bool collideVerticalCapsuleSphere(
    double capsuleRadius,
    double sphereRadius,
    double halfHeight,
    const Eigen::Vector3d& capsuleTranslation,
    const Eigen::Vector3d& sphereCenter,
    CollisionResult& result)
{
  const Eigen::Vector3d localSphereCenter = sphereCenter - capsuleTranslation;
  const double closestZ
      = std::clamp(localSphereCenter.z(), -halfHeight, halfHeight);
  const double dx = localSphereCenter.x();
  const double dy = localSphereCenter.y();
  const double dz = localSphereCenter.z() - closestZ;
  const double sumRadii = capsuleRadius + sphereRadius;

  if (dy == 0.0 && dz == 0.0) {
    const double dist = std::abs(dx);
    if (dist > sumRadii) {
      return false;
    }

    const double penetration = sumRadii - dist;
    if (dist < 1e-10) {
      const Eigen::Vector3d normal
          = chooseRadialNormal(Eigen::Vector3d::UnitZ());
      const Eigen::Vector3d closestOnCapsule
          = capsuleTranslation + Eigen::Vector3d(0.0, 0.0, closestZ);
      const Eigen::Vector3d contactPoint
          = closestOnCapsule + (-normal) * (capsuleRadius - penetration * 0.5);

      result.addContact(contactPoint, normal, penetration);
    } else {
      const double normalX = (dx > 0.0) ? -1.0 : 1.0;
      result.addContact(
          capsuleTranslation.x()
              - normalX * (capsuleRadius - penetration * 0.5),
          capsuleTranslation.y(),
          capsuleTranslation.z() + closestZ,
          normalX,
          0.0,
          0.0,
          penetration);
    }
    return true;
  }

  if (dx == 0.0 && dz == 0.0) {
    const double dist = std::abs(dy);
    if (dist > sumRadii) {
      return false;
    }

    const double penetration = sumRadii - dist;
    const double normalY = (dy > 0.0) ? -1.0 : 1.0;
    result.addContact(
        capsuleTranslation.x(),
        capsuleTranslation.y() - normalY * (capsuleRadius - penetration * 0.5),
        capsuleTranslation.z() + closestZ,
        0.0,
        normalY,
        0.0,
        penetration);
    return true;
  }

  const double distSquared = dx * dx + dy * dy + dz * dz;

  if (distSquared > sumRadii * sumRadii) {
    return false;
  }

  const double dist = std::sqrt(distSquared);
  const double penetration = sumRadii - dist;

  Eigen::Vector3d normal;
  if (dist < 1e-10) {
    normal = Eigen::Vector3d::UnitZ();
  } else {
    normal = Eigen::Vector3d(-dx, -dy, -dz) / dist;
  }

  const Eigen::Vector3d closestOnCapsule
      = capsuleTranslation + Eigen::Vector3d(0.0, 0.0, closestZ);
  const Eigen::Vector3d contactPoint
      = closestOnCapsule + (-normal) * (capsuleRadius - penetration * 0.5);

  result.addContact(contactPoint, normal, penetration);
  return true;
}

bool capsuleOverlapsSphere(
    double capsuleRadius,
    double sphereRadius,
    double halfHeight,
    const Eigen::Isometry3d& capsuleTransform,
    const Eigen::Isometry3d& sphereTransform)
{
  const double sumRadii = capsuleRadius + sphereRadius;
  const Eigen::Vector3d sphereCenter = sphereTransform.translation();

  if (hasIdentityRotation(capsuleTransform)) {
    const Eigen::Vector3d localSphereCenter
        = sphereCenter - capsuleTransform.translation();
    const double closestZ
        = std::clamp(localSphereCenter.z(), -halfHeight, halfHeight);
    const Eigen::Vector3d diff(
        localSphereCenter.x(),
        localSphereCenter.y(),
        localSphereCenter.z() - closestZ);
    return diff.squaredNorm() <= sumRadii * sumRadii;
  }

  const Eigen::Vector3d localTop(0, 0, halfHeight);
  const Eigen::Vector3d localBottom(0, 0, -halfHeight);

  const Eigen::Vector3d top = capsuleTransform * localTop;
  const Eigen::Vector3d bottom = capsuleTransform * localBottom;
  const Eigen::Vector3d closestOnCapsule
      = closestPointOnSegment(sphereCenter, bottom, top);

  return (sphereCenter - closestOnCapsule).squaredNorm() <= sumRadii * sumRadii;
}

} // namespace

bool collideCapsuleSphere(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (option.maxNumContacts == 0) {
    return false;
  }

  const double capsuleRadius = detail::getRadius(capsule);
  const double sphereRadius = detail::getRadius(sphere);
  const double halfHeight = detail::getHeight(capsule) * 0.5;

  if (!option.enableContact) {
    return capsuleOverlapsSphere(
        capsuleRadius,
        sphereRadius,
        halfHeight,
        capsuleTransform,
        sphereTransform);
  }

  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  if (hasIdentityRotation(capsuleTransform)) {
    return collideVerticalCapsuleSphere(
        capsuleRadius,
        sphereRadius,
        halfHeight,
        capsuleTransform.translation(),
        sphereTransform.translation(),
        result);
  }

  const Eigen::Vector3d localTop(0, 0, halfHeight);
  const Eigen::Vector3d localBottom(0, 0, -halfHeight);

  const Eigen::Vector3d top = capsuleTransform * localTop;
  const Eigen::Vector3d bottom = capsuleTransform * localBottom;
  const Eigen::Vector3d sphereCenter = sphereTransform.translation();

  const Eigen::Vector3d closestOnCapsule
      = closestPointOnSegment(sphereCenter, bottom, top);

  const Eigen::Vector3d diff = sphereCenter - closestOnCapsule;
  const double distSquared = diff.squaredNorm();
  const double sumRadii = capsuleRadius + sphereRadius;

  if (distSquared > sumRadii * sumRadii) {
    return false;
  }

  const double dist = std::sqrt(distSquared);
  const double penetration = sumRadii - dist;

  Eigen::Vector3d normal;
  if (dist < 1e-10) {
    normal = chooseRadialNormal(top - bottom);
  } else {
    normal = -diff / dist;
  }

  const Eigen::Vector3d contactPoint
      = closestOnCapsule + (-normal) * (capsuleRadius - penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = normal;
  contact.depth = penetration;

  result.addContact(contact);

  return true;
}

} // namespace dart::collision::native
