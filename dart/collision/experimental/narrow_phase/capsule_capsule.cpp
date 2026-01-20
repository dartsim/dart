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

#include <dart/collision/experimental/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::experimental {

namespace {

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

} // namespace

bool collideCapsules(
    const CapsuleShape& capsule1,
    const Eigen::Isometry3d& transform1,
    const CapsuleShape& capsule2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double radius1 = capsule1.getRadius();
  const double radius2 = capsule2.getRadius();
  const double halfHeight1 = capsule1.getHeight() * 0.5;
  const double halfHeight2 = capsule2.getHeight() * 0.5;

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

} // namespace dart::collision::experimental
