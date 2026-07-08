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

#include <dart/collision/native/narrow_phase/CapsuleCapsule.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <algorithm>
#include <stdexcept>

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

struct SegmentClosestPointResult
{
  Eigen::Vector3d point1;
  Eigen::Vector3d point2;
  double distSquared;
};

SegmentClosestPointResult closestPointsBetweenSegments(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2)
{
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;

  const double a = d1.squaredNorm();
  const double e = d2.squaredNorm();
  const double f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;

  constexpr double epsilon = 1e-10;

  if (a <= epsilon && e <= epsilon) {
    s = t = 0.0;
  } else if (a <= epsilon) {
    s = 0.0;
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= epsilon) {
      t = 0.0;
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;

      if (denom != 0.0) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      } else {
        s = 0.0;
      }

      t = (b * s + f) / e;

      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }

  SegmentClosestPointResult result;
  result.point1 = p1 + d1 * s;
  result.point2 = p2 + d2 * t;
  result.distSquared = (result.point1 - result.point2).squaredNorm();
  return result;
}

bool collideVerticalCapsules(
    double radius1,
    double radius2,
    double halfHeight1,
    double halfHeight2,
    const Eigen::Vector3d& translation1,
    const Eigen::Vector3d& translation2,
    CollisionResult& result)
{
  const double minZ1 = translation1.z() - halfHeight1;
  const double maxZ1 = translation1.z() + halfHeight1;
  const double minZ2 = translation2.z() - halfHeight2;
  const double maxZ2 = translation2.z() + halfHeight2;

  double z1 = minZ1;
  double z2 = minZ2;
  if (maxZ1 < minZ2) {
    z1 = maxZ1;
    z2 = minZ2;
  } else if (maxZ2 < minZ1) {
    z1 = minZ1;
    z2 = maxZ2;
  } else {
    z1 = z2 = std::max(minZ1, minZ2);
  }

  const double dx = translation1.x() - translation2.x();
  const double dy = translation1.y() - translation2.y();
  const double dz = z1 - z2;
  const double sumRadii = radius1 + radius2;

  if (dy == 0.0 && dz == 0.0) {
    const double dist = std::abs(dx);
    if (dist > sumRadii) {
      return false;
    }

    const double penetration = sumRadii - dist;
    if (dist < 1e-10) {
      result.addContact(
          translation2.x(),
          translation2.y(),
          z2 + radius2 - penetration * 0.5,
          0.0,
          0.0,
          1.0,
          penetration);
    } else {
      const double normalX = dx / dist;
      result.addContact(
          translation2.x() + normalX * (radius2 - penetration * 0.5),
          translation2.y(),
          z2,
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
    const double normalY = dy / dist;
    result.addContact(
        translation2.x(),
        translation2.y() + normalY * (radius2 - penetration * 0.5),
        z2,
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
    normal = Eigen::Vector3d(dx, dy, dz) / dist;
  }

  const Eigen::Vector3d contactPoint
      = Eigen::Vector3d(translation2.x(), translation2.y(), z2)
        + normal * (radius2 - penetration * 0.5);

  result.addContact(contactPoint, normal, penetration);
  return true;
}

bool capsulesOverlap(
    double radius1,
    double radius2,
    double halfHeight1,
    double halfHeight2,
    const Eigen::Isometry3d& transform1,
    const Eigen::Isometry3d& transform2)
{
  const Eigen::Vector3d localTop1(0, 0, halfHeight1);
  const Eigen::Vector3d localBottom1(0, 0, -halfHeight1);
  const Eigen::Vector3d localTop2(0, 0, halfHeight2);
  const Eigen::Vector3d localBottom2(0, 0, -halfHeight2);

  const Eigen::Vector3d top1 = transform1 * localTop1;
  const Eigen::Vector3d bottom1 = transform1 * localBottom1;
  const Eigen::Vector3d top2 = transform2 * localTop2;
  const Eigen::Vector3d bottom2 = transform2 * localBottom2;

  const auto closest
      = closestPointsBetweenSegments(bottom1, top1, bottom2, top2);
  const double sumRadii = radius1 + radius2;
  return closest.distSquared <= sumRadii * sumRadii;
}

} // namespace

bool collideCapsules(
    const CapsuleShape& capsule1,
    const Eigen::Isometry3d& transform1,
    const CapsuleShape& capsule2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (option.maxNumContacts == 0) {
    return false;
  }

  const double radius1 = capsule1.getRadius();
  const double radius2 = capsule2.getRadius();
  const double halfHeight1 = capsule1.getHeight() * 0.5;
  const double halfHeight2 = capsule2.getHeight() * 0.5;

  if (!option.enableContact) {
    return capsulesOverlap(
        radius1, radius2, halfHeight1, halfHeight2, transform1, transform2);
  }

  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  if (hasIdentityRotation(transform1) && hasIdentityRotation(transform2)) {
    return collideVerticalCapsules(
        radius1,
        radius2,
        halfHeight1,
        halfHeight2,
        transform1.translation(),
        transform2.translation(),
        result);
  }

  const Eigen::Vector3d localTop1(0, 0, halfHeight1);
  const Eigen::Vector3d localBottom1(0, 0, -halfHeight1);
  const Eigen::Vector3d localTop2(0, 0, halfHeight2);
  const Eigen::Vector3d localBottom2(0, 0, -halfHeight2);

  const Eigen::Vector3d top1 = transform1 * localTop1;
  const Eigen::Vector3d bottom1 = transform1 * localBottom1;
  const Eigen::Vector3d top2 = transform2 * localTop2;
  const Eigen::Vector3d bottom2 = transform2 * localBottom2;

  auto closest = closestPointsBetweenSegments(bottom1, top1, bottom2, top2);

  const double sumRadii = radius1 + radius2;
  if (closest.distSquared > sumRadii * sumRadii) {
    return false;
  }

  const double dist = std::sqrt(closest.distSquared);
  const double penetration = sumRadii - dist;

  Eigen::Vector3d normal;
  if (dist < 1e-10) {
    normal = Eigen::Vector3d::UnitZ();
  } else {
    normal = (closest.point1 - closest.point2) / dist;
  }

  const Eigen::Vector3d contactPoint
      = closest.point2 + normal * (radius2 - penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = normal;
  contact.depth = penetration;

  result.addContact(contact);

  return true;
}

void collideCapsulesBatch(
    span<const CapsulePair> pairs,
    span<CollisionResult> results,
    const CollisionOption& option)
{
  if (results.size() < pairs.size()) {
    throw std::invalid_argument(
        "collideCapsulesBatch requires one result for each pair");
  }

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    if (pair.shapeA == nullptr || pair.shapeB == nullptr) {
      throw std::invalid_argument(
          "collideCapsulesBatch received a null capsule");
    }

    [[maybe_unused]] const bool collided = collideCapsules(
        *pair.shapeA, pair.tfA, *pair.shapeB, pair.tfB, results[i], option);
  }
}

} // namespace dart::collision::native
