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

#include <dart/collision/native/narrow_phase/capsule_box.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <algorithm>
#include <array>
#include <limits>

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
  double bestInteriorMargin = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d bestOnBox = Eigen::Vector3d::Zero();
  Eigen::Vector3d bestOnSegment = segmentStart;

  auto computeInteriorMargin = [&](const Eigen::Vector3d& point) {
    double margin = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis) {
      const double axisMargin = halfExtents[axis] - std::abs(point[axis]);
      if (axisMargin < 0.0) {
        return -std::numeric_limits<double>::infinity();
      }
      margin = std::min(margin, axisMargin);
    }
    return margin;
  };

  auto testCandidate = [&](double t) {
    const Eigen::Vector3d pointOnSegment = segmentStart + segment * t;
    const Eigen::Vector3d pointOnBox
        = closestPointOnBox(pointOnSegment, halfExtents);
    const double distSq = (pointOnSegment - pointOnBox).squaredNorm();
    const double interiorMargin = computeInteriorMargin(pointOnSegment);

    if (distSq < bestDistSq - 1e-18
        || (std::abs(distSq - bestDistSq) <= 1e-18
            && interiorMargin > bestInteriorMargin)) {
      bestDistSq = distSq;
      bestInteriorMargin = interiorMargin;
      bestOnBox = pointOnBox;
      bestOnSegment = pointOnSegment;
    }
  };

  std::array<double, 8> breakpoints{};
  std::size_t numBreakpoints = 0;
  breakpoints[numBreakpoints++] = 0.0;
  breakpoints[numBreakpoints++] = 1.0;

  constexpr double kAxisEps = 1e-12;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(segment[axis]) <= kAxisEps) {
      continue;
    }

    for (const double face : {-halfExtents[axis], halfExtents[axis]}) {
      const double t = (face - segmentStart[axis]) / segment[axis];
      if (t > 0.0 && t < 1.0) {
        breakpoints[numBreakpoints++] = t;
      }
    }
  }

  std::sort(breakpoints.begin(), breakpoints.begin() + numBreakpoints);

  std::array<double, 8> uniqueBreakpoints{};
  std::size_t numUniqueBreakpoints = 0;
  for (std::size_t i = 0; i < numBreakpoints; ++i) {
    if (numUniqueBreakpoints == 0
        || std::abs(
               breakpoints[i] - uniqueBreakpoints[numUniqueBreakpoints - 1])
               > kAxisEps) {
      uniqueBreakpoints[numUniqueBreakpoints++] = breakpoints[i];
      testCandidate(breakpoints[i]);
    }
  }

  for (std::size_t i = 0; i + 1 < numUniqueBreakpoints; ++i) {
    const double lo = uniqueBreakpoints[i];
    const double hi = uniqueBreakpoints[i + 1];
    if (hi - lo <= kAxisEps) {
      continue;
    }

    const double mid = 0.5 * (lo + hi);
    double numerator = 0.0;
    double denominator = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double coord = segmentStart[axis] + segment[axis] * mid;
      double face = 0.0;
      if (coord < -halfExtents[axis]) {
        face = -halfExtents[axis];
      } else if (coord > halfExtents[axis]) {
        face = halfExtents[axis];
      } else {
        continue;
      }

      numerator += segment[axis] * (face - segmentStart[axis]);
      denominator += segment[axis] * segment[axis];
    }

    if (denominator > kAxisEps) {
      testCandidate(std::clamp(numerator / denominator, lo, hi));
    } else {
      testCandidate(mid);
    }
  }

  closestOnSegment = bestOnSegment;
  return bestOnBox;
}

bool collideTranslatedVerticalCapsuleBox(
    double capsuleRadius,
    double halfHeight,
    const Eigen::Vector3d& capsuleTranslation,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Vector3d& boxTranslation,
    CollisionResult& result,
    const CollisionOption& option)
{
  const Eigen::Vector3d capsuleCenterLocal
      = capsuleTranslation - boxTranslation;

  const double segmentMinZ = capsuleCenterLocal.z() - halfHeight;
  const double segmentMaxZ = capsuleCenterLocal.z() + halfHeight;

  double axisZ = 0.0;
  if (segmentMinZ > halfExtents.z()) {
    axisZ = segmentMinZ;
  } else if (segmentMaxZ < -halfExtents.z()) {
    axisZ = segmentMaxZ;
  } else {
    const double overlapMin = std::max(segmentMinZ, -halfExtents.z());
    const double overlapMax = std::min(segmentMaxZ, halfExtents.z());
    axisZ = std::clamp(0.0, overlapMin, overlapMax);
  }

  std::array<ContactPoint, 3> pairContacts;
  std::size_t numPairContacts = 0;

  auto addContactForAxisPoint = [&](const Eigen::Vector3d& axisPointLocal) {
    if (result.numContacts() >= option.maxNumContacts) {
      return false;
    }

    Eigen::Vector3d contactOnBoxLocal(
        std::clamp(axisPointLocal.x(), -halfExtents.x(), halfExtents.x()),
        std::clamp(axisPointLocal.y(), -halfExtents.y(), halfExtents.y()),
        std::clamp(axisPointLocal.z(), -halfExtents.z(), halfExtents.z()));

    const Eigen::Vector3d diff = axisPointLocal - contactOnBoxLocal;
    const double distSquared = diff.squaredNorm();
    if (distSquared > capsuleRadius * capsuleRadius) {
      return false;
    }

    Eigen::Vector3d normalLocal;
    Eigen::Vector3d contactPositionLocal;
    double penetration = 0.0;
    const double dist = std::sqrt(distSquared);
    if (dist < 1e-10) {
      const Eigen::Vector3d absAxisPoint = axisPointLocal.cwiseAbs();
      const Eigen::Vector3d distToFace = halfExtents - absAxisPoint;
      int minAxis = 0;
      if (distToFace.y() < distToFace.x()) {
        minAxis = 1;
      }
      if (distToFace.z() < distToFace[minAxis]) {
        minAxis = 2;
      }

      normalLocal = Eigen::Vector3d::Zero();
      normalLocal[minAxis] = (axisPointLocal[minAxis] >= 0.0) ? 1.0 : -1.0;
      contactOnBoxLocal[minAxis] = (normalLocal[minAxis] > 0.0)
                                       ? halfExtents[minAxis]
                                       : -halfExtents[minAxis];
      penetration = capsuleRadius + distToFace[minAxis];
      contactPositionLocal = 0.5
                             * (contactOnBoxLocal + axisPointLocal
                                + normalLocal * capsuleRadius);
    } else {
      normalLocal = diff / dist;
      penetration = capsuleRadius - dist;
      contactPositionLocal
          = contactOnBoxLocal - normalLocal * (penetration * 0.5);
    }

    ContactPoint contact;
    contact.position = boxTranslation + contactPositionLocal;
    contact.normal = normalLocal;
    contact.depth = penetration;

    for (std::size_t i = 0; i < numPairContacts; ++i) {
      const auto& existing = pairContacts[i];
      if ((existing.position - contact.position).squaredNorm() < 1e-12
          && existing.normal.dot(contact.normal) > 0.999) {
        return false;
      }
    }

    result.addContact(contact);
    pairContacts[numPairContacts++] = contact;
    return true;
  };

  bool hit = addContactForAxisPoint(
      Eigen::Vector3d(capsuleCenterLocal.x(), capsuleCenterLocal.y(), axisZ));
  hit = addContactForAxisPoint(Eigen::Vector3d(
            capsuleCenterLocal.x(), capsuleCenterLocal.y(), segmentMinZ))
        || hit;
  hit = addContactForAxisPoint(Eigen::Vector3d(
            capsuleCenterLocal.x(), capsuleCenterLocal.y(), segmentMaxZ))
        || hit;
  return hit;
}

bool capsuleOverlapsBox(
    double capsuleRadius,
    double halfHeight,
    const Eigen::Isometry3d& capsuleTransform,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Isometry3d& boxTransform)
{
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

  return (closestOnSegment - closestOnBox).squaredNorm()
         <= capsuleRadius * capsuleRadius;
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
  if (option.maxNumContacts == 0) {
    return false;
  }

  const double capsuleRadius = detail::getRadius(capsule);
  const double halfHeight = detail::getHeight(capsule) * 0.5;
  const Eigen::Vector3d& halfExtents = detail::getHalfExtents(box);

  if (!option.enableContact) {
    return capsuleOverlapsBox(
        capsuleRadius, halfHeight, capsuleTransform, halfExtents, boxTransform);
  }

  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  if (hasIdentityRotation(capsuleTransform)
      && hasIdentityRotation(boxTransform)) {
    return collideTranslatedVerticalCapsuleBox(
        capsuleRadius,
        halfHeight,
        capsuleTransform.translation(),
        halfExtents,
        boxTransform.translation(),
        result,
        option);
  }

  const Eigen::Vector3d localTop(0, 0, halfHeight);
  const Eigen::Vector3d localBottom(0, 0, -halfHeight);

  const Eigen::Vector3d worldTop = capsuleTransform * localTop;
  const Eigen::Vector3d worldBottom = capsuleTransform * localBottom;

  const Eigen::Isometry3d boxInverse = boxTransform.inverse();
  const Eigen::Vector3d boxTop = boxInverse * worldTop;
  const Eigen::Vector3d boxBottom = boxInverse * worldBottom;

  Eigen::Vector3d closestOnSegment;
  closestPointOnSegmentInBoxSpace(
      boxBottom, boxTop, halfExtents, closestOnSegment);

  std::array<ContactPoint, 3> pairContacts;
  std::size_t numPairContacts = 0;

  auto addContactForAxisPoint = [&](const Eigen::Vector3d& axisPointLocal) {
    if (result.numContacts() >= option.maxNumContacts) {
      return false;
    }

    const Eigen::Vector3d pointOnBoxLocal
        = closestPointOnBox(axisPointLocal, halfExtents);
    const Eigen::Vector3d diff = axisPointLocal - pointOnBoxLocal;
    const double distSquared = diff.squaredNorm();

    if (distSquared > capsuleRadius * capsuleRadius) {
      return false;
    }

    const double dist = std::sqrt(distSquared);

    Eigen::Vector3d normalLocal;
    Eigen::Vector3d contactOnBoxLocal = pointOnBoxLocal;
    Eigen::Vector3d contactPositionLocal;
    double penetration = capsuleRadius - dist;
    if (dist < 1e-10) {
      const Eigen::Vector3d absAxisPoint = axisPointLocal.cwiseAbs();
      const Eigen::Vector3d distToFace = halfExtents - absAxisPoint;
      int minAxis = 0;
      if (distToFace.y() < distToFace.x()) {
        minAxis = 1;
      }
      if (distToFace.z() < distToFace[minAxis]) {
        minAxis = 2;
      }
      normalLocal = Eigen::Vector3d::Zero();
      normalLocal[minAxis] = (axisPointLocal[minAxis] >= 0) ? 1.0 : -1.0;
      contactOnBoxLocal[minAxis] = normalLocal[minAxis] > 0.0
                                       ? halfExtents[minAxis]
                                       : -halfExtents[minAxis];
      penetration = capsuleRadius + distToFace[minAxis];
      contactPositionLocal = 0.5
                             * (contactOnBoxLocal + axisPointLocal
                                + normalLocal * capsuleRadius);
    } else {
      normalLocal = diff / dist;
      contactPositionLocal
          = contactOnBoxLocal - normalLocal * (penetration * 0.5);
    }

    const Eigen::Vector3d normalWorld = boxTransform.rotation() * normalLocal;

    ContactPoint contact;
    contact.position = boxTransform * contactPositionLocal;
    contact.normal = normalWorld;
    contact.depth = penetration;

    for (std::size_t i = 0; i < numPairContacts; ++i) {
      const auto& existing = pairContacts[i];
      if ((existing.position - contact.position).squaredNorm() < 1e-12
          && existing.normal.dot(contact.normal) > 0.999) {
        return false;
      }
    }

    result.addContact(contact);
    pairContacts[numPairContacts++] = contact;
    return true;
  };

  bool hit = addContactForAxisPoint(closestOnSegment);
  hit = addContactForAxisPoint(boxBottom) || hit;
  hit = addContactForAxisPoint(boxTop) || hit;

  return hit;
}

} // namespace dart::collision::native
