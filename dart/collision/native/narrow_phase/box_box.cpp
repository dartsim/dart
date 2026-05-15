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

#include <dart/collision/native/narrow_phase/box_box.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <algorithm>
#include <array>
#include <utility>

#include <cmath>

namespace dart::collision::native {

namespace {

constexpr double kEpsilon = 1e-10;
constexpr double kSurfaceAxisEpsilon = 1e-8;

struct SatResult
{
  double penetration = std::numeric_limits<double>::max();
  Eigen::Vector3d axis = Eigen::Vector3d::Zero();
  int axisIndex = -1;
};

double projectBox(
    const Eigen::Vector3d& halfExtents,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& axis)
{
  return halfExtents.x() * std::abs(axis.dot(rotation.col(0)))
         + halfExtents.y() * std::abs(axis.dot(rotation.col(1)))
         + halfExtents.z() * std::abs(axis.dot(rotation.col(2)));
}

bool testAxis(
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& centerDiff,
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Matrix3d& rotation1,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Matrix3d& rotation2,
    int axisIndex,
    SatResult& best)
{
  double axisLengthSq = axis.squaredNorm();
  if (axisLengthSq < kEpsilon * kEpsilon) {
    return true;
  }

  Eigen::Vector3d normalizedAxis = axis / std::sqrt(axisLengthSq);

  double proj1 = projectBox(halfExtents1, rotation1, normalizedAxis);
  double proj2 = projectBox(halfExtents2, rotation2, normalizedAxis);
  double distance = std::abs(centerDiff.dot(normalizedAxis));

  double overlap = proj1 + proj2 - distance;

  if (overlap < 0) {
    return false;
  }

  if (overlap < best.penetration) {
    best.penetration = overlap;
    best.axis = normalizedAxis;
    best.axisIndex = axisIndex;
  }

  return true;
}

Eigen::Vector3d computeContactPoint(
    const Eigen::Vector3d& center1,
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Matrix3d& rotation1,
    const Eigen::Vector3d& center2,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Matrix3d& rotation2,
    const Eigen::Vector3d& normal)
{
  auto surfacePointNear = [](const Eigen::Vector3d& center,
                             const Eigen::Vector3d& halfExtents,
                             const Eigen::Matrix3d& rotation,
                             const Eigen::Vector3d& direction,
                             const Eigen::Vector3d& nearPoint) {
    Eigen::Vector3d local = rotation.transpose() * (nearPoint - center);
    const Eigen::Vector3d localDirection = rotation.transpose() * direction;

    // Snap axes facing the contact normal to the surface; clamp tangential axes
    // near the opposing point so large boxes do not report far-corner contacts.
    for (int i = 0; i < 3; ++i) {
      if (std::abs(localDirection[i]) > kSurfaceAxisEpsilon) {
        local[i] = (localDirection[i] > 0.0) ? halfExtents[i] : -halfExtents[i];
      } else {
        local[i] = std::clamp(local[i], -halfExtents[i], halfExtents[i]);
      }
    }

    return center + rotation * local;
  };

  Eigen::Vector3d forwardPoint1
      = surfacePointNear(center1, halfExtents1, rotation1, -normal, center2);
  Eigen::Vector3d forwardPoint2 = surfacePointNear(
      center2, halfExtents2, rotation2, normal, forwardPoint1);
  forwardPoint1 = surfacePointNear(
      center1, halfExtents1, rotation1, -normal, forwardPoint2);

  Eigen::Vector3d reversePoint2
      = surfacePointNear(center2, halfExtents2, rotation2, normal, center1);
  Eigen::Vector3d reversePoint1 = surfacePointNear(
      center1, halfExtents1, rotation1, -normal, reversePoint2);
  reversePoint2 = surfacePointNear(
      center2, halfExtents2, rotation2, normal, reversePoint1);

  return (forwardPoint1 + forwardPoint2 + reversePoint1 + reversePoint2) * 0.25;
}

std::array<double, 2> projectInterval(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& axis)
{
  const auto centerProjection = center.dot(axis);
  const auto radius = projectBox(halfExtents, rotation, axis);
  return {centerProjection - radius, centerProjection + radius};
}

std::size_t addCoordinatePair(
    double minValue, double maxValue, std::array<double, 2>& coords)
{
  coords[0] = minValue;
  coords[1] = maxValue;
  return (maxValue - minValue > kEpsilon) ? 2u : 1u;
}

bool addFacePatchContacts(
    const Eigen::Vector3d& center1,
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Matrix3d& rotation1,
    const Eigen::Vector3d& center2,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Matrix3d& rotation2,
    const Eigen::Vector3d& normal,
    const SatResult& best,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (best.axisIndex < 0 || best.axisIndex >= 6) {
    return false;
  }

  const auto contactsBefore = result.numContacts();
  if (contactsBefore >= option.maxNumContacts) {
    return false;
  }

  const auto referenceAxisIndex
      = (best.axisIndex < 3) ? best.axisIndex : best.axisIndex - 3;
  const auto& referenceRotation = (best.axisIndex < 3) ? rotation1 : rotation2;
  const auto& incidentRotation = (best.axisIndex < 3) ? rotation2 : rotation1;
  double incidentFaceAlignment = 0.0;
  for (int i = 0; i < 3; ++i) {
    incidentFaceAlignment = std::max(
        incidentFaceAlignment, std::abs(normal.dot(incidentRotation.col(i))));
  }
  if (incidentFaceAlignment < 1.0 - 1e-6) {
    return false;
  }

  const auto tangent1 = referenceRotation.col((referenceAxisIndex + 1) % 3);
  const auto tangent2 = referenceRotation.col((referenceAxisIndex + 2) % 3);

  const auto interval1T1
      = projectInterval(center1, halfExtents1, rotation1, tangent1);
  const auto interval2T1
      = projectInterval(center2, halfExtents2, rotation2, tangent1);
  const auto interval1T2
      = projectInterval(center1, halfExtents1, rotation1, tangent2);
  const auto interval2T2
      = projectInterval(center2, halfExtents2, rotation2, tangent2);

  const auto minT1 = std::max(interval1T1[0], interval2T1[0]);
  const auto maxT1 = std::min(interval1T1[1], interval2T1[1]);
  const auto minT2 = std::max(interval1T2[0], interval2T2[0]);
  const auto maxT2 = std::min(interval1T2[1], interval2T2[1]);
  if (maxT1 < minT1 || maxT2 < minT2) {
    return false;
  }

  const auto interval1Normal
      = projectInterval(center1, halfExtents1, rotation1, normal);
  const auto interval2Normal
      = projectInterval(center2, halfExtents2, rotation2, normal);
  const auto contactNormalCoord
      = 0.5 * (interval1Normal[0] + interval2Normal[1]);

  std::array<double, 2> coordsT1{};
  std::array<double, 2> coordsT2{};
  const auto numT1 = addCoordinatePair(minT1, maxT1, coordsT1);
  const auto numT2 = addCoordinatePair(minT2, maxT2, coordsT2);

  ContactManifold manifold;
  manifold.setType(ContactType::Face);

  const auto remaining = option.maxNumContacts - contactsBefore;
  for (std::size_t i = 0; i < numT1; ++i) {
    for (std::size_t j = 0; j < numT2; ++j) {
      if (manifold.numContacts() >= remaining) {
        break;
      }

      ContactPoint contact;
      contact.position = contactNormalCoord * normal + coordsT1[i] * tangent1
                         + coordsT2[j] * tangent2;
      contact.normal = normal;
      contact.depth = best.penetration;
      manifold.addContact(contact);
    }
  }

  if (!manifold.hasContacts()) {
    return false;
  }

  result.addManifold(std::move(manifold));
  return true;
}

} // namespace

bool collideBoxes(
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const Eigen::Vector3d center1 = transform1.translation();
  const Eigen::Vector3d center2 = transform2.translation();
  const Eigen::Matrix3d rotation1 = transform1.rotation();
  const Eigen::Matrix3d rotation2 = transform2.rotation();
  const Eigen::Vector3d centerDiff = center2 - center1;

  SatResult best;
  int axisIndex = 0;

  std::array<Eigen::Vector3d, 3> axes1
      = {rotation1.col(0), rotation1.col(1), rotation1.col(2)};
  std::array<Eigen::Vector3d, 3> axes2
      = {rotation2.col(0), rotation2.col(1), rotation2.col(2)};

  for (int i = 0; i < 3; ++i) {
    if (!testAxis(
            axes1[i],
            centerDiff,
            halfExtents1,
            rotation1,
            halfExtents2,
            rotation2,
            axisIndex++,
            best)) {
      return false;
    }
  }

  for (int i = 0; i < 3; ++i) {
    if (!testAxis(
            axes2[i],
            centerDiff,
            halfExtents1,
            rotation1,
            halfExtents2,
            rotation2,
            axisIndex++,
            best)) {
      return false;
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d crossAxis = axes1[i].cross(axes2[j]);
      if (!testAxis(
              crossAxis,
              centerDiff,
              halfExtents1,
              rotation1,
              halfExtents2,
              rotation2,
              axisIndex++,
              best)) {
        return false;
      }
    }
  }

  Eigen::Vector3d normal = best.axis;
  if (centerDiff.dot(normal) > 0) {
    normal = -normal;
  }

  Eigen::Vector3d contactPoint = computeContactPoint(
      center1,
      halfExtents1,
      rotation1,
      center2,
      halfExtents2,
      rotation2,
      normal);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = normal;
  contact.depth = best.penetration;

  if (addFacePatchContacts(
          center1,
          halfExtents1,
          rotation1,
          center2,
          halfExtents2,
          rotation2,
          normal,
          best,
          result,
          option)) {
    return true;
  }

  result.addContact(contact);

  return true;
}

bool collideBoxes(
    const BoxShape& box1,
    const Eigen::Isometry3d& transform1,
    const BoxShape& box2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  return collideBoxes(
      box1.getHalfExtents(),
      transform1,
      box2.getHalfExtents(),
      transform2,
      result,
      option);
}

} // namespace dart::collision::native
