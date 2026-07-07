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

#include <dart/collision/native/narrow_phase/BoxBox.hpp>
#include <dart/collision/native/narrow_phase/box_box/ContactReduction.hpp>
#include <dart/collision/native/narrow_phase/box_box/FaceClip.hpp>
#include <dart/collision/native/narrow_phase/box_box/Sat.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include <utility>

#include <cmath>

namespace dart::collision::native {

namespace {

[[nodiscard]] box_box::BoxData makeBoxData(
    const Eigen::Vector3d& halfExtents, const Eigen::Isometry3d& transform)
{
  return {transform.translation(), halfExtents, transform.rotation()};
}

[[nodiscard]] ContactType contactTypeFor(
    const box_box::SatResult& sat, std::size_t numContacts)
{
  if (sat.axisType == box_box::SatAxisType::Edge) {
    return ContactType::Edge;
  }

  return numContacts > 1 ? ContactType::Face : ContactType::Point;
}

[[nodiscard]] bool rotationsMatch(
    const Eigen::Matrix3d& rotation1, const Eigen::Matrix3d& rotation2)
{
  return rotation1.isApprox(rotation2, 1e-12);
}

[[nodiscard]] bool addUniquePoint(
    std::array<Eigen::Vector3d, ContactManifold::kMaxContacts>& points,
    std::size_t& numPoints,
    const Eigen::Vector3d& point)
{
  constexpr double duplicateDistanceSq = 1e-14;
  for (std::size_t i = 0; i < numPoints; ++i) {
    if ((point - points[i]).squaredNorm() <= duplicateDistanceSq) {
      return false;
    }
  }

  points[numPoints++] = point;
  return true;
}

[[nodiscard]] bool tryCollideAlignedBoxes(
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option,
    bool& collided)
{
  collided = false;
  const Eigen::Matrix3d& rotation = transform1.linear();
  if (!rotationsMatch(rotation, transform2.linear())) {
    return false;
  }

  const Eigen::Vector3d centerDiff
      = rotation.transpose()
        * (transform2.translation() - transform1.translation());
  Eigen::Vector3d overlap;
  int contactAxis = 0;
  double penetration = std::numeric_limits<double>::max();
  for (int i = 0; i < 3; ++i) {
    overlap[i] = halfExtents1[i] + halfExtents2[i] - std::abs(centerDiff[i]);
    if (overlap[i] < -1e-12) {
      return true;
    }
    overlap[i] = std::max(0.0, overlap[i]);
    if (overlap[i] < penetration) {
      penetration = overlap[i];
      contactAxis = i;
    }
  }

  collided = true;
  if (!option.enableContact) {
    return true;
  }

  const auto remainingContacts = option.maxNumContacts - result.numContacts();
  if (remainingContacts == 0) {
    collided = false;
    return true;
  }

  const int tangentAxis1 = (contactAxis + 1) % 3;
  const int tangentAxis2 = (contactAxis + 2) % 3;
  const double sign = centerDiff[contactAxis] > 0.0 ? 1.0 : -1.0;
  const double face1 = sign * halfExtents1[contactAxis];
  const double face2
      = centerDiff[contactAxis] - sign * halfExtents2[contactAxis];
  const double contactPlane = 0.5 * (face1 + face2);

  const double min1 = std::max(
      -halfExtents1[tangentAxis1],
      centerDiff[tangentAxis1] - halfExtents2[tangentAxis1]);
  const double max1 = std::min(
      halfExtents1[tangentAxis1],
      centerDiff[tangentAxis1] + halfExtents2[tangentAxis1]);
  const double min2 = std::max(
      -halfExtents1[tangentAxis2],
      centerDiff[tangentAxis2] - halfExtents2[tangentAxis2]);
  const double max2 = std::min(
      halfExtents1[tangentAxis2],
      centerDiff[tangentAxis2] + halfExtents2[tangentAxis2]);

  std::array<Eigen::Vector3d, ContactManifold::kMaxContacts> points;
  std::size_t numPoints = 0;
  for (const double u : {min1, max1}) {
    for (const double v : {min2, max2}) {
      Eigen::Vector3d local = Eigen::Vector3d::Zero();
      local[contactAxis] = contactPlane;
      local[tangentAxis1] = u;
      local[tangentAxis2] = v;
      (void)addUniquePoint(
          points, numPoints, transform1.translation() + rotation * local);
    }
  }

  const std::size_t numContacts
      = std::min({numPoints, remainingContacts, ContactManifold::kMaxContacts});
  if (numContacts == 0) {
    collided = false;
    return true;
  }

  ContactManifold manifold;
  manifold.setType(numContacts > 1 ? ContactType::Face : ContactType::Point);
  const Eigen::Vector3d normal = (centerDiff[contactAxis] > 0.0 ? -1.0 : 1.0)
                                 * rotation.col(contactAxis);

  for (std::size_t i = 0; i < numContacts; ++i) {
    ContactPoint contact;
    contact.position = points[i];
    contact.normal = normal;
    contact.depth = penetration;
    manifold.addContact(contact);
  }

  result.addManifold(std::move(manifold));
  return true;
}

[[nodiscard]] bool collideBoxesImpl(
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (option.enableContact && result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  bool alignedCollision = false;
  if (tryCollideAlignedBoxes(
          halfExtents1,
          transform1,
          halfExtents2,
          transform2,
          result,
          option,
          alignedCollision)) {
    return alignedCollision;
  }

  const auto box1 = makeBoxData(halfExtents1, transform1);
  const auto box2 = makeBoxData(halfExtents2, transform2);

  box_box::SatResult sat;
  if (!box_box::computeBoxBoxSat(box1, box2, sat)) {
    return false;
  }

  if (!option.enableContact) {
    return true;
  }

  const auto remainingContacts = option.maxNumContacts - result.numContacts();
  if (remainingContacts == 0) {
    return false;
  }

  const std::size_t manifoldContactLimit
      = std::min(remainingContacts, ContactManifold::kMaxContacts);
  const auto candidates
      = box_box::computeBoxBoxContactCandidates(box1, box2, sat);
  const auto contacts
      = box_box::reduceContactCandidates(candidates, manifoldContactLimit);
  if (contacts.empty()) {
    return false;
  }

  ContactManifold manifold;
  manifold.setType(contactTypeFor(sat, contacts.size()));

  for (const auto& candidate : contacts) {
    ContactPoint contact;
    contact.position = candidate.position;
    contact.normal = sat.normal;
    contact.depth = std::max(0.0, candidate.depth);
    manifold.addContact(contact);
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
  return collideBoxesImpl(
      halfExtents1, transform1, halfExtents2, transform2, result, option);
}

bool collideBoxes(
    const BoxShape& box1,
    const Eigen::Isometry3d& transform1,
    const BoxShape& box2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (!option.enableContact) {
    return collideBoxesImpl(
        box1.getHalfExtents(),
        transform1,
        box2.getHalfExtents(),
        transform2,
        result,
        option);
  }

  const auto manifoldsBefore = result.numManifolds();
  const BoxPair pair{&box1, &box2, transform1, transform2};
  collideBoxesBatch(
      span<const BoxPair>(&pair, 1), span<CollisionResult>(&result, 1), option);
  return result.numManifolds() > manifoldsBefore;
}

void collideBoxesBatch(
    span<const BoxPair> pairs,
    span<CollisionResult> results,
    const CollisionOption& option)
{
  if (results.size() < pairs.size()) {
    throw std::invalid_argument(
        "collideBoxesBatch requires one result for each pair");
  }

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    if (pair.shapeA == nullptr || pair.shapeB == nullptr) {
      throw std::invalid_argument("collideBoxesBatch received a null box");
    }

    [[maybe_unused]] const bool collided = collideBoxesImpl(
        pair.shapeA->getHalfExtents(),
        pair.tfA,
        pair.shapeB->getHalfExtents(),
        pair.tfB,
        results[i],
        option);
  }
}

} // namespace dart::collision::native
