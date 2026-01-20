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

#include <dart/collision/experimental/narrow_phase/capsule_box.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::experimental {

namespace {

Eigen::Vector3d closestPointOnBox(
    const Eigen::Vector3d& point, const Eigen::Vector3d& halfExtents)
{
  return Eigen::Vector3d(
      std::clamp(point.x(), -halfExtents.x(), halfExtents.x()),
      std::clamp(point.y(), -halfExtents.y(), halfExtents.y()),
      std::clamp(point.z(), -halfExtents.z(), halfExtents.z()));
}

Eigen::Vector3d closestPointOnSegmentInBoxSpace(
    const Eigen::Vector3d& segmentStart,
    const Eigen::Vector3d& segmentEnd,
    const Eigen::Vector3d& halfExtents,
    Eigen::Vector3d& closestOnSegment)
{
  const Eigen::Vector3d segment = segmentEnd - segmentStart;
  const double segmentLengthSq = segment.squaredNorm();

  if (segmentLengthSq < 1e-10) {
    closestOnSegment = segmentStart;
    return closestPointOnBox(segmentStart, halfExtents);
  }

  double bestDistSq = std::numeric_limits<double>::max();
  Eigen::Vector3d bestOnBox;
  Eigen::Vector3d bestOnSegment;

  constexpr int numSamples = 8;
  for (int i = 0; i <= numSamples; ++i) {
    const double t = static_cast<double>(i) / numSamples;
    const Eigen::Vector3d pointOnSegment = segmentStart + segment * t;
    const Eigen::Vector3d pointOnBox
        = closestPointOnBox(pointOnSegment, halfExtents);
    const double distSq = (pointOnSegment - pointOnBox).squaredNorm();

    if (distSq < bestDistSq) {
      bestDistSq = distSq;
      bestOnBox = pointOnBox;
      bestOnSegment = pointOnSegment;
    }
  }

  closestOnSegment = bestOnSegment;
  return bestOnBox;
}

} // namespace

bool collideCapsuleBox(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double capsuleRadius = capsule.getRadius();
  const double halfHeight = capsule.getHeight() * 0.5;
  const Eigen::Vector3d& halfExtents = box.getHalfExtents();

  const Eigen::Vector3d localTop(0, 0, halfHeight);
  const Eigen::Vector3d localBottom(0, 0, -halfHeight);

  const Eigen::Vector3d worldTop = capsuleTransform * localTop;
  const Eigen::Vector3d worldBottom = capsuleTransform * localBottom;

  const Eigen::Isometry3d boxInverse = boxTransform.inverse();
  const Eigen::Vector3d boxTop = boxInverse * worldTop;
  const Eigen::Vector3d boxBottom = boxInverse * worldBottom;

  Eigen::Vector3d closestOnSegment;
  const Eigen::Vector3d closestOnBox = closestPointOnSegmentInBoxSpace(
      boxBottom, boxTop, halfExtents, closestOnSegment);

  const Eigen::Vector3d diff = closestOnSegment - closestOnBox;
  const double distSquared = diff.squaredNorm();

  if (distSquared > capsuleRadius * capsuleRadius) {
    return false;
  }

  const double dist = std::sqrt(distSquared);
  const double penetration = capsuleRadius - dist;

  Eigen::Vector3d normalLocal;
  if (dist < 1e-10) {
    const Eigen::Vector3d absClosest = closestOnBox.cwiseAbs();
    const Eigen::Vector3d distToFace = halfExtents - absClosest;
    int minAxis = 0;
    if (distToFace.y() < distToFace.x())
      minAxis = 1;
    if (distToFace.z() < distToFace[minAxis])
      minAxis = 2;
    normalLocal = Eigen::Vector3d::Zero();
    normalLocal[minAxis] = (closestOnSegment[minAxis] >= 0) ? 1.0 : -1.0;
  } else {
    normalLocal = diff / dist;
  }

  const Eigen::Vector3d normalWorld = boxTransform.rotation() * normalLocal;

  const Eigen::Vector3d contactWorld
      = boxTransform * closestOnBox + normalWorld * (penetration * 0.5);

  ContactPoint contact;
  contact.position = contactWorld;
  contact.normal = -normalWorld;
  contact.depth = penetration;

  result.addContact(contact);

  return true;
}

} // namespace dart::collision::experimental
