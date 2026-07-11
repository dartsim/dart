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

#include "dart/collision/dart/PersistentManifoldCache.hpp"

#include "dart/collision/dart/detail/InlineVector.hpp"
#include "dart/common/Macros.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include <cmath>

namespace dart {
namespace collision {
namespace native {

namespace {

constexpr double kNormalEpsilon = 1e-9;
constexpr double kNormalEpsilonSquared = kNormalEpsilon * kNormalEpsilon;
constexpr double kFrictionBasisNormalDotTolerance = 1e-6;
constexpr double kFrictionBasisNormalDeltaSquared
    = kFrictionBasisNormalDotTolerance * kFrictionBasisNormalDotTolerance;
constexpr double kFrictionBasisMinNormalDot
    = 1.0 - kFrictionBasisNormalDotTolerance;
constexpr double kFrictionBasisMinNormalDotSquared
    = kFrictionBasisMinNormalDot * kFrictionBasisMinNormalDot;

//==============================================================================
PairKey canonicalPair(std::size_t idA, std::size_t idB)
{
  if (idA <= idB)
    return {idA, idB};

  return {idB, idA};
}

//==============================================================================
Eigen::Vector3d normalizedOrDefault(const Eigen::Vector3d& normal)
{
  const double norm = normal.norm();
  if (norm < kNormalEpsilon)
    return Eigen::Vector3d::UnitZ();

  return normal / norm;
}

//==============================================================================
bool hasMatchingFrictionBasis(
    const CachedContact& existing, const CachedContact& contact)
{
  const double normalDot = existing.normal.dot(contact.normal);
  if (!std::isfinite(normalDot))
    return false;

  if (normalDot <= 0.0)
    return false;

  if ((existing.normal - contact.normal).squaredNorm()
      <= kFrictionBasisNormalDeltaSquared) {
    return true;
  }

  const double existingNormalNormSquared = existing.normal.squaredNorm();
  const double contactNormalNormSquared = contact.normal.squaredNorm();
  if (existingNormalNormSquared < kNormalEpsilonSquared
      || contactNormalNormSquared < kNormalEpsilonSquared) {
    return false;
  }

  return normalDot * normalDot >= kFrictionBasisMinNormalDotSquared
                                      * existingNormalNormSquared
                                      * contactNormalNormSquared;
}

//==============================================================================
// Fixed-capacity candidate buffer: the reduce/overflow paths handle at most
// kMaxContacts + 1 entries, so inline storage keeps them allocation-free.
using FixedCandidates
    = InlineVector<CachedContact, PersistentManifold::kMaxContacts + 4>;

std::array<int, PersistentManifold::kMaxContacts> selectContactIndices(
    const FixedCandidates& candidates)
{
  std::array<int, PersistentManifold::kMaxContacts> selected{-1, -1, -1, -1};
  if (candidates.empty())
    return selected;

  // Bounded: callers pass at most kMaxContacts + 1 candidates; keep the
  // used-flags on the stack so the steady-state cache path never allocates.
  DART_ASSERT(candidates.size() <= 8u);
  std::array<bool, 8> used{};

  int deepest = 0;
  double maxDepth = candidates[0].penetrationDepth;
  for (std::size_t i = 1; i < candidates.size(); ++i) {
    if (candidates[i].penetrationDepth > maxDepth) {
      maxDepth = candidates[i].penetrationDepth;
      deepest = static_cast<int>(i);
    }
  }

  selected[0] = deepest;
  used[static_cast<std::size_t>(deepest)] = true;

  if (candidates.size() == 1)
    return selected;

  const Eigen::Vector3d& p0
      = candidates[static_cast<std::size_t>(selected[0])].localPointA;

  double farthestDist = -1.0;
  int farthest = -1;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (used[i])
      continue;

    const double d2 = (candidates[i].localPointA - p0).squaredNorm();
    if (d2 > farthestDist) {
      farthestDist = d2;
      farthest = static_cast<int>(i);
    }
  }

  if (farthest >= 0) {
    selected[1] = farthest;
    used[static_cast<std::size_t>(farthest)] = true;
  }

  if (candidates.size() <= 2 || selected[1] < 0)
    return selected;

  const Eigen::Vector3d& p1
      = candidates[static_cast<std::size_t>(selected[1])].localPointA;
  const Eigen::Vector3d e01 = p1 - p0;

  double maxArea2 = -1.0;
  int bestThird = -1;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (used[i])
      continue;

    const Eigen::Vector3d e02 = candidates[i].localPointA - p0;
    const double area2 = e01.cross(e02).squaredNorm();
    if (area2 > maxArea2) {
      maxArea2 = area2;
      bestThird = static_cast<int>(i);
    }
  }

  if (bestThird >= 0) {
    selected[2] = bestThird;
    used[static_cast<std::size_t>(bestThird)] = true;
  }

  if (candidates.size() <= 3 || selected[2] < 0)
    return selected;

  const Eigen::Vector3d& p2
      = candidates[static_cast<std::size_t>(selected[2])].localPointA;
  const Eigen::Vector3d e02 = p2 - p0;

  double maxVolume = -1.0;
  int bestFourth = -1;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (used[i])
      continue;

    const Eigen::Vector3d e03 = candidates[i].localPointA - p0;
    const double volume6 = std::abs(e01.dot(e02.cross(e03)));
    if (volume6 > maxVolume) {
      maxVolume = volume6;
      bestFourth = static_cast<int>(i);
    }
  }

  if (bestFourth >= 0)
    selected[3] = bestFourth;

  return selected;
}

//==============================================================================
void writeReducedContacts(
    const FixedCandidates& candidates, PersistentManifold& manifold)
{
  const auto selected = selectContactIndices(candidates);
  int count = 0;
  for (int idx : selected) {
    if (idx < 0 || count >= PersistentManifold::kMaxContacts)
      continue;

    manifold.contacts[static_cast<std::size_t>(count)]
        = candidates[static_cast<std::size_t>(idx)];
    ++count;
  }

  manifold.numContacts = count;
}

} // namespace

//==============================================================================
int PersistentManifold::findMatch(
    const Eigen::Vector3d& localPointA, double threshold) const
{
  if (numContacts <= 0)
    return -1;

  const double threshold2 = threshold * threshold;
  int bestMatch = -1;
  double bestDistance = std::numeric_limits<double>::max();

  for (int i = 0; i < numContacts; ++i) {
    const double d2
        = (contacts[static_cast<std::size_t>(i)].localPointA - localPointA)
              .squaredNorm();
    if (d2 < bestDistance) {
      bestDistance = d2;
      bestMatch = i;
    }
  }

  if (bestDistance <= threshold2)
    return bestMatch;

  return -1;
}

//==============================================================================
void PersistentManifold::addOrReplace(const CachedContact& contact)
{
  const int match = findMatch(contact.localPointA);
  if (match >= 0) {
    CachedContact updated = contact;
    const CachedContact& existing = contacts[static_cast<std::size_t>(match)];
    updated.cachedNormalImpulse = existing.cachedNormalImpulse;
    if (hasMatchingFrictionBasis(existing, contact)) {
      updated.cachedFrictionImpulse1 = existing.cachedFrictionImpulse1;
      updated.cachedFrictionImpulse2 = existing.cachedFrictionImpulse2;
      updated.cachedFrictionBasis1 = existing.cachedFrictionBasis1;
      updated.cachedFrictionBasis2 = existing.cachedFrictionBasis2;
      updated.hasCachedFrictionBasis = existing.hasCachedFrictionBasis;
    } else {
      updated.cachedFrictionImpulse1 = 0.0;
      updated.cachedFrictionImpulse2 = 0.0;
      updated.cachedFrictionBasis1.setZero();
      updated.cachedFrictionBasis2.setZero();
      updated.hasCachedFrictionBasis = false;
    }
    updated.lifetime = 0;
    contacts[static_cast<std::size_t>(match)] = updated;
    return;
  }

  if (numContacts < kMaxContacts) {
    CachedContact toInsert = contact;
    toInsert.lifetime = 0;
    contacts[static_cast<std::size_t>(numContacts)] = toInsert;
    ++numContacts;
    return;
  }

  reduce();

  FixedCandidates candidates;
  for (int i = 0; i < numContacts; ++i)
    candidates.push_back(contacts[static_cast<std::size_t>(i)]);

  CachedContact toInsert = contact;
  toInsert.lifetime = 0;
  candidates.push_back(toInsert);

  writeReducedContacts(candidates, *this);
}

//==============================================================================
void PersistentManifold::reduce()
{
  if (numContacts <= kMaxContacts)
    return;

  FixedCandidates candidates;
  for (int i = 0; i < numContacts; ++i)
    candidates.push_back(contacts[static_cast<std::size_t>(i)]);

  writeReducedContacts(candidates, *this);
}

//==============================================================================
void PersistentManifold::refresh(
    const Eigen::Isometry3d& tfA,
    const Eigen::Isometry3d& tfB,
    double breakingThreshold)
{
  int writeIndex = 0;
  for (int i = 0; i < numContacts; ++i) {
    CachedContact contact = contacts[static_cast<std::size_t>(i)];

    const Eigen::Vector3d worldA = tfA * contact.localPointA;
    const Eigen::Vector3d worldB = tfB * contact.localPointB;
    const Eigen::Vector3d rel = worldB - worldA;

    const Eigen::Vector3d n = normalizedOrDefault(contact.normal);
    const double normalDistance = std::abs(rel.dot(n));
    const Eigen::Vector3d tangential = rel - n * rel.dot(n);
    const double tangentialDrift = tangential.norm();

    if (normalDistance > breakingThreshold
        || tangentialDrift > breakingThreshold) {
      continue;
    }

    ++contact.lifetime;
    contacts[static_cast<std::size_t>(writeIndex)] = contact;
    ++writeIndex;
  }

  numContacts = writeIndex;
}

//==============================================================================
bool PairKey::operator==(const PairKey& other) const
{
  return idA == other.idA && idB == other.idB;
}

//==============================================================================
std::size_t PairKeyHash::operator()(const PairKey& key) const
{
  const std::size_t h1 = std::hash<std::size_t>{}(key.idA);
  const std::size_t h2 = std::hash<std::size_t>{}(key.idB);
  return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6U) + (h1 >> 2U));
}

//==============================================================================
PersistentManifold& PersistentManifoldCache::getOrCreate(
    std::size_t idA, std::size_t idB)
{
  const PairKey key = canonicalPair(idA, idB);
  return mManifolds[key];
}

//==============================================================================
bool PersistentManifoldCache::ownsContact(
    std::size_t idA, std::size_t idB, const void* contact) const
{
  if (contact == nullptr)
    return false;

  const PairKey key = canonicalPair(idA, idB);
  const auto it = mManifolds.find(key);
  if (it == mManifolds.end())
    return false;

  const auto& manifold = it->second;
  for (int i = 0; i < manifold.numContacts; ++i) {
    if (&manifold.contacts[static_cast<std::size_t>(i)] == contact)
      return true;
  }

  return false;
}

//==============================================================================
void PersistentManifoldCache::remove(std::size_t idA, std::size_t idB)
{
  const PairKey key = canonicalPair(idA, idB);
  mManifolds.erase(key);
}

//==============================================================================
void PersistentManifoldCache::removeObject(std::size_t id)
{
  for (auto it = mManifolds.begin(); it != mManifolds.end();) {
    if (it->first.idA == id || it->first.idB == id) {
      it = mManifolds.erase(it);
      continue;
    }

    ++it;
  }
}

//==============================================================================
void PersistentManifoldCache::refreshAll(
    const TransformProvider& transformProvider, double breakingThreshold)
{
  refreshAllWith(transformProvider, breakingThreshold);
}

//==============================================================================
void PersistentManifoldCache::clear()
{
  mManifolds.clear();
}

//==============================================================================
std::size_t PersistentManifoldCache::size() const
{
  return mManifolds.size();
}

} // namespace native
} // namespace collision
} // namespace dart
