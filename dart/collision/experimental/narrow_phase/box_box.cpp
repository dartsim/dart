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

#include <dart/collision/experimental/narrow_phase/box_box.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <array>

#include <cmath>

namespace dart::collision::experimental {

namespace {

constexpr double kEpsilon = 1e-10;

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
  Eigen::Vector3d point1 = center1;
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d axis = rotation1.col(i);
    double sign = (normal.dot(axis) > 0) ? 1.0 : -1.0;
    point1 -= sign * halfExtents1[i] * axis;
  }

  Eigen::Vector3d point2 = center2;
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d axis = rotation2.col(i);
    double sign = (normal.dot(axis) > 0) ? -1.0 : 1.0;
    point2 -= sign * halfExtents2[i] * axis;
  }

  return (point1 + point2) * 0.5;
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

} // namespace dart::collision::experimental
