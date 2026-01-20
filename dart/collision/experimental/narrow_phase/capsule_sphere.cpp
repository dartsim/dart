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

#include <dart/collision/experimental/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::experimental {

namespace {

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

} // namespace

bool collideCapsuleSphere(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double capsuleRadius = capsule.getRadius();
  const double sphereRadius = sphere.getRadius();
  const double halfHeight = capsule.getHeight() * 0.5;

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
    normal = Eigen::Vector3d::UnitZ();
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

} // namespace dart::collision::experimental
