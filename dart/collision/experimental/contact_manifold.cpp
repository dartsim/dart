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

#include <dart/collision/experimental/contact_manifold.hpp>

#include <limits>
#include <stdexcept>

namespace dart::collision::experimental {

namespace {

Eigen::Vector3d normalizedOrDefault(const Eigen::Vector3d& normal)
{
  if (ContactPoint::isZeroNormal(normal)) {
    return Eigen::Vector3d::UnitZ();
  }
  return normal.normalized();
}

Eigen::Vector3d selectReductionNormal(
    const ContactPoint& candidate,
    const std::array<ContactPoint, ContactManifold::kMaxContacts>& contacts,
    std::size_t count)
{
  if (ContactPoint::isNonZeroNormal(candidate.normal)) {
    return normalizedOrDefault(candidate.normal);
  }

  for (std::size_t i = 0; i < count; ++i) {
    if (ContactPoint::isNonZeroNormal(contacts[i].normal)) {
      return normalizedOrDefault(contacts[i].normal);
    }
  }

  return Eigen::Vector3d::UnitZ();
}

double spreadScore(
    const std::array<ContactPoint, ContactManifold::kMaxContacts>& points,
    const Eigen::Vector3d& normal)
{
  const Eigen::Vector3d n = normalizedOrDefault(normal);
  std::array<Eigen::Vector3d, ContactManifold::kMaxContacts> projected;
  for (std::size_t i = 0; i < ContactManifold::kMaxContacts; ++i) {
    projected[i] = points[i].position - n * n.dot(points[i].position);
  }

  double score = 0.0;
  for (std::size_t i = 0; i < ContactManifold::kMaxContacts; ++i) {
    for (std::size_t j = i + 1; j < ContactManifold::kMaxContacts; ++j) {
      score += (projected[i] - projected[j]).squaredNorm();
    }
  }
  return score;
}

} // namespace

void ContactManifold::addContact(const ContactPoint& contact)
{
  if (numContacts_ < kMaxContacts) {
    contacts_[numContacts_++] = contact;
  } else {
    std::array<ContactPoint, kMaxContacts + 1> candidates;
    for (std::size_t i = 0; i < kMaxContacts; ++i) {
      candidates[i] = contacts_[i];
    }
    candidates[kMaxContacts] = contact;

    std::size_t deepestIndex = 0;
    double maxDepth = candidates[0].depth;
    for (std::size_t i = 1; i < candidates.size(); ++i) {
      if (candidates[i].depth > maxDepth) {
        maxDepth = candidates[i].depth;
        deepestIndex = i;
      }
    }

    const Eigen::Vector3d normal = selectReductionNormal(
        contact, contacts_, numContacts_);

    double bestScore = std::numeric_limits<double>::lowest();
    double bestDepthSum = std::numeric_limits<double>::lowest();
    std::size_t bestDrop = deepestIndex;

    // Keep the deepest point; choose the remaining set with widest spread.
    for (std::size_t drop = 0; drop < candidates.size(); ++drop) {
      if (drop == deepestIndex) {
        continue;
      }
      std::array<ContactPoint, kMaxContacts> subset;
      std::size_t subsetIndex = 0;
      double depthSum = 0.0;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (i == drop) {
          continue;
        }
        subset[subsetIndex++] = candidates[i];
        depthSum += candidates[i].depth;
      }
      const double score = spreadScore(subset, normal);
      if (score > bestScore || (score == bestScore && depthSum > bestDepthSum)) {
        bestScore = score;
        bestDepthSum = depthSum;
        bestDrop = drop;
      }
    }

    std::size_t writeIndex = 0;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      if (i == bestDrop) {
        continue;
      }
      contacts_[writeIndex++] = candidates[i];
    }
    numContacts_ = kMaxContacts;
  }

  if (object1_ == nullptr && object2_ == nullptr
      && (contact.object1 != nullptr || contact.object2 != nullptr)) {
    object1_ = contact.object1;
    object2_ = contact.object2;
  }
}

void ContactManifold::clear()
{
  numContacts_ = 0;
  type_ = ContactType::Unknown;
  object1_ = nullptr;
  object2_ = nullptr;
}

std::size_t ContactManifold::numContacts() const
{
  return numContacts_;
}

bool ContactManifold::hasContacts() const
{
  return numContacts_ > 0;
}

const ContactPoint& ContactManifold::getContact(std::size_t i) const
{
  if (i >= numContacts_) {
    throw std::out_of_range("ContactManifold::getContact");
  }
  return contacts_[i];
}

std::span<const ContactPoint> ContactManifold::getContacts() const
{
  return {contacts_.data(), numContacts_};
}

ContactType ContactManifold::getType() const
{
  return type_;
}

void ContactManifold::setType(ContactType type)
{
  type_ = type;
}

bool ContactManifold::isTypeCompatible() const
{
  switch (type_) {
    case ContactType::Unknown:
      return true;
    case ContactType::Point:
      return numContacts_ == 1;
    case ContactType::Edge:
      return numContacts_ == 2;
    case ContactType::Face: {
      const auto shared = getSharedNormal();
      return numContacts_ >= 3 && ContactPoint::isNonZeroNormal(shared);
    }
    case ContactType::Patch: {
      const auto shared = getSharedNormal();
      return numContacts_ >= 2 && ContactPoint::isNonZeroNormal(shared);
    }
  }
  return false;
}

Eigen::Vector3d ContactManifold::getSharedNormal() const
{
  if (numContacts_ == 0) {
    return Eigen::Vector3d::Zero();
  }

  const auto& firstNormal = contacts_[0].normal;
  for (std::size_t i = 1; i < numContacts_; ++i) {
    if ((contacts_[i].normal - firstNormal).squaredNorm()
        > ContactPoint::kNormalEpsilon * ContactPoint::kNormalEpsilon) {
      return Eigen::Vector3d::Zero();
    }
  }
  return firstNormal;
}

const CollisionObject* ContactManifold::getObject1() const
{
  return object1_;
}

const CollisionObject* ContactManifold::getObject2() const
{
  return object2_;
}

void ContactManifold::setObjects(const CollisionObject* o1, const CollisionObject* o2)
{
  object1_ = o1;
  object2_ = o2;
}

} // namespace dart::collision::experimental
