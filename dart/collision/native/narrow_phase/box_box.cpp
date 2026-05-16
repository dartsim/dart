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
#include <dart/collision/native/narrow_phase/box_box/contact_reduction.hpp>
#include <dart/collision/native/narrow_phase/box_box/face_clip.hpp>
#include <dart/collision/native/narrow_phase/box_box/sat.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

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
      std::span<const BoxPair>(&pair, 1),
      std::span<CollisionResult>(&result, 1),
      option);
  return result.numManifolds() > manifoldsBefore;
}

void collideBoxesBatch(
    std::span<const BoxPair> pairs,
    std::span<CollisionResult> results,
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
