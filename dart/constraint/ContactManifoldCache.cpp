/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/constraint/ContactManifoldCache.hpp"

#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/CollisionObject.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <utility>

#include <cmath>

namespace dart {
namespace constraint {
namespace {

struct Candidate
{
  const collision::Contact* contact{nullptr};
  std::uint8_t age{0u};
  bool isFresh{false};
};

struct UpdateScratch
{
  std::vector<Candidate> freshCandidates;
  std::vector<Candidate> staleCandidates;
  std::vector<Candidate> existingCandidates;
  std::vector<unsigned char> matchedExisting;
  std::vector<unsigned char> matchedRaw;
};

UpdateScratch& getUpdateScratch()
{
  static thread_local UpdateScratch scratch;
  return scratch;
}

bool isNormalWithinThreshold(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    double normalThreshold,
    double normalThresholdSquared)
{
  const double normASq = a.squaredNorm();
  const double normBSq = b.squaredNorm();
  if (normASq <= 0.0 || normBSq <= 0.0)
    return false;

  const double dot = a.dot(b);
  if (normalThreshold >= 0.0) {
    if (dot < 0.0)
      return false;
    return dot * dot >= normalThresholdSquared * normASq * normBSq;
  }

  const double denom = std::sqrt(normASq * normBSq);
  return dot / denom >= normalThreshold;
}

bool isContactValid(const collision::Contact& contact)
{
  if (collision::Contact::isZeroNormal(contact.normal))
    return false;

  if (contact.penetrationDepth < 0.0)
    return false;

  return true;
}

bool contactLess(const collision::Contact& a, const collision::Contact& b)
{
  if (a.penetrationDepth != b.penetrationDepth)
    return a.penetrationDepth > b.penetrationDepth;

  if (a.point.x() != b.point.x())
    return a.point.x() < b.point.x();
  if (a.point.y() != b.point.y())
    return a.point.y() < b.point.y();
  if (a.point.z() != b.point.z())
    return a.point.z() < b.point.z();

  if (a.normal.x() != b.normal.x())
    return a.normal.x() < b.normal.x();
  if (a.normal.y() != b.normal.y())
    return a.normal.y() < b.normal.y();
  if (a.normal.z() != b.normal.z())
    return a.normal.z() < b.normal.z();

  return false;
}

bool candidateLess(const Candidate& a, const Candidate& b)
{
  if (a.isFresh != b.isFresh)
    return a.isFresh;

  if (contactLess(*a.contact, *b.contact))
    return true;
  if (contactLess(*b.contact, *a.contact))
    return false;

  return a.age < b.age;
}

bool isNear(
    const collision::Contact& a,
    const collision::Contact& b,
    double positionThresholdSquared,
    double normalThreshold,
    double normalThresholdSquared)
{
  const Eigen::Vector3d delta = a.point - b.point;
  if (delta.squaredNorm() > positionThresholdSquared)
    return false;

  return isNormalWithinThreshold(
      a.normal, b.normal, normalThreshold, normalThresholdSquared);
}

std::vector<Candidate> deduplicateCandidates(
    std::vector<Candidate> candidates,
    double positionThresholdSquared,
    double normalThreshold,
    double normalThresholdSquared)
{
  if (candidates.size() < 2)
    return candidates;

  std::sort(candidates.begin(), candidates.end(), candidateLess);

  std::vector<Candidate> unique;
  unique.reserve(candidates.size());

  for (const auto& candidate : candidates) {
    bool duplicate = false;
    for (const auto& kept : unique) {
      if (isNear(
              *candidate.contact,
              *kept.contact,
              positionThresholdSquared,
              normalThreshold,
              normalThresholdSquared)) {
        duplicate = true;
        break;
      }
    }

    if (!duplicate)
      unique.push_back(candidate);
  }

  return unique;
}

std::vector<Candidate> filterNearExisting(
    std::vector<Candidate> candidates,
    const std::vector<Candidate>& existing,
    double positionThresholdSquared,
    double normalThreshold,
    double normalThresholdSquared)
{
  if (candidates.empty() || existing.empty())
    return candidates;

  std::vector<Candidate> filtered;
  filtered.reserve(candidates.size());

  for (const auto& candidate : candidates) {
    bool duplicate = false;
    for (const auto& kept : existing) {
      if (isNear(
              *candidate.contact,
              *kept.contact,
              positionThresholdSquared,
              normalThreshold,
              normalThresholdSquared)) {
        duplicate = true;
        break;
      }
    }

    if (!duplicate)
      filtered.push_back(candidate);
  }

  return filtered;
}

Candidate selectDeepest(const std::vector<Candidate>& candidates)
{
  auto best = candidates.begin();
  for (auto it = candidates.begin(); it != candidates.end(); ++it) {
    if (contactLess(*it->contact, *best->contact))
      best = it;
  }
  return *best;
}

double minDistanceSquared(
    const collision::Contact& candidate, const std::vector<Candidate>& selected)
{
  double minDist = std::numeric_limits<double>::infinity();
  for (const auto& entry : selected) {
    const double dist = (candidate.point - entry.contact->point).squaredNorm();
    if (dist < minDist)
      minDist = dist;
  }
  return minDist;
}

bool candidateBetter(
    const Candidate& a, double aMinDist, const Candidate& b, double bMinDist)
{
  if (aMinDist != bMinDist)
    return aMinDist > bMinDist;

  if (a.isFresh != b.isFresh)
    return a.isFresh;

  if (contactLess(*a.contact, *b.contact))
    return true;
  if (contactLess(*b.contact, *a.contact))
    return false;

  return a.age < b.age;
}

std::vector<Candidate> selectRepresentativeContacts(
    std::vector<Candidate> freshCandidates,
    std::vector<Candidate> staleCandidates,
    std::size_t maxPoints,
    double positionThresholdSquared,
    double normalThreshold)
{
  if (freshCandidates.empty())
    return {};

  const double normalThresholdSquared = normalThreshold * normalThreshold;
  freshCandidates = deduplicateCandidates(
      freshCandidates,
      positionThresholdSquared,
      normalThreshold,
      normalThresholdSquared);
  staleCandidates = deduplicateCandidates(
      staleCandidates,
      positionThresholdSquared,
      normalThreshold,
      normalThresholdSquared);

  staleCandidates = filterNearExisting(
      std::move(staleCandidates),
      freshCandidates,
      positionThresholdSquared,
      normalThreshold,
      normalThresholdSquared);

  if (freshCandidates.empty())
    return {};

  if (freshCandidates.size() + staleCandidates.size() <= maxPoints) {
    std::vector<Candidate> combined;
    combined.reserve(freshCandidates.size() + staleCandidates.size());
    combined.insert(
        combined.end(),
        std::make_move_iterator(freshCandidates.begin()),
        std::make_move_iterator(freshCandidates.end()));
    combined.insert(
        combined.end(),
        std::make_move_iterator(staleCandidates.begin()),
        std::make_move_iterator(staleCandidates.end()));
    std::sort(
        combined.begin(),
        combined.end(),
        [](const Candidate& a, const Candidate& b) {
          return contactLess(*a.contact, *b.contact);
        });
    return combined;
  }

  std::vector<Candidate> selected;
  selected.reserve(maxPoints);

  Candidate seed = selectDeepest(freshCandidates);
  selected.push_back(seed);

  auto seedMatch = [&](const Candidate& candidate) {
    return !contactLess(*candidate.contact, *seed.contact)
           && !contactLess(*seed.contact, *candidate.contact);
  };

  freshCandidates.erase(
      std::remove_if(freshCandidates.begin(), freshCandidates.end(), seedMatch),
      freshCandidates.end());

  std::vector<Candidate> pool = freshCandidates;
  if (selected.size() + freshCandidates.size() < maxPoints) {
    pool.insert(pool.end(), staleCandidates.begin(), staleCandidates.end());
  }

  while (selected.size() < maxPoints && !pool.empty()) {
    auto bestIt = pool.begin();
    double bestDist = minDistanceSquared(*bestIt->contact, selected);

    for (auto it = pool.begin(); it != pool.end(); ++it) {
      const double dist = minDistanceSquared(*it->contact, selected);
      if (candidateBetter(*it, dist, *bestIt, bestDist)) {
        bestIt = it;
        bestDist = dist;
      }
    }

    selected.push_back(*bestIt);
    pool.erase(bestIt);
  }

  std::sort(
      selected.begin(),
      selected.end(),
      [](const Candidate& a, const Candidate& b) {
        return contactLess(*a.contact, *b.contact);
      });

  if (selected.size() > maxPoints)
    selected.resize(maxPoints);

  return selected;
}

} // namespace

//=============================================================================
void ContactManifoldCache::reset()
{
  mManifolds.clear();
  mRawEntries.clear();
  mOutputRanges.clear();
  mOutputScratch.clear();
  mFrameCounter = 0u;
}

//=============================================================================
void ContactManifoldCache::invalidateCollisionObject(
    const collision::CollisionObject* object)
{
  if (!object)
    return;

  std::erase_if(mManifolds, [object](const auto& entry) {
    return entry.first.first == object || entry.first.second == object;
  });
}

//=============================================================================
void ContactManifoldCache::purgeInvalidManifolds(
    const collision::CollisionGroup* group)
{
  if (!group) {
    mManifolds.clear();
    return;
  }

  std::erase_if(mManifolds, [group](const auto& entry) {
    const auto& key = entry.first;
    const auto* sf1 = key.first ? key.first->getShapeFrame() : nullptr;
    const auto* sf2 = key.second ? key.second->getShapeFrame() : nullptr;
    return !sf1 || !sf2 || !group->hasShapeFrame(sf1)
           || !group->hasShapeFrame(sf2);
  });
}

//=============================================================================
void ContactManifoldCache::update(
    const collision::CollisionResult& rawContacts,
    const ContactManifoldCacheOptions& options,
    std::vector<collision::Contact>& outputContacts)
{
  outputContacts.clear();

  if (!options.enabled) {
    reset();
    return;
  }

  const auto maxPoints = std::min(options.maxPointsPerPair, kMaxManifoldPoints);
  if (maxPoints == 0u) {
    reset();
    return;
  }

  ++mFrameCounter;

  for (auto& [key, patch] : mManifolds) {
    if (patch.framesSinceSeen < std::numeric_limits<std::uint16_t>::max())
      ++patch.framesSinceSeen;
  }

  mRawEntries.clear();
  mRawEntries.reserve(rawContacts.getNumContacts());

  for (const auto& contact : rawContacts.getContacts()) {
    if (!contact.collisionObject1 || !contact.collisionObject2)
      continue;

    if (!isContactValid(contact))
      continue;

    RawEntry entry;
    entry.key = makePairKey(contact.collisionObject1, contact.collisionObject2);
    entry.contact = &contact;
    mRawEntries.push_back(entry);
  }

  const double positionThresholdSquared
      = options.positionThreshold * options.positionThreshold;
  const double normalThreshold = options.normalThreshold;
  const double normalThresholdSquared = normalThreshold * normalThreshold;

  PairKeyEqual pairEqual;
  auto rawEntryLess = [](const RawEntry& a, const RawEntry& b) {
    if (a.key.first != b.key.first)
      return a.key.first < b.key.first;
    if (a.key.second != b.key.second)
      return a.key.second < b.key.second;
    return contactLess(*a.contact, *b.contact);
  };

  std::sort(mRawEntries.begin(), mRawEntries.end(), rawEntryLess);

  mOutputRanges.clear();
  mOutputRanges.reserve(mRawEntries.size());
  mOutputScratch.clear();
  mOutputScratch.reserve(mRawEntries.size());

  std::size_t index = 0u;
  auto& scratch = getUpdateScratch();
  auto& freshCandidates = scratch.freshCandidates;
  auto& staleCandidates = scratch.staleCandidates;
  auto& existingCandidates = scratch.existingCandidates;
  auto& matchedExisting = scratch.matchedExisting;
  auto& matchedRaw = scratch.matchedRaw;
  while (index < mRawEntries.size()) {
    const auto& key = mRawEntries[index].key;
    std::size_t end = index + 1u;
    while (end < mRawEntries.size() && pairEqual(mRawEntries[end].key, key))
      ++end;

    const std::size_t contactCount = end - index;

    Manifold* patch = findOrCreateManifold(key, options);

    if (patch) {
      if (patch->count == 0u && contactCount <= maxPoints) {
        patch->count = 0u;
        patch->framesSinceSeen = 0u;
        patch->lastUpdateFrame = mFrameCounter;

        OutputRange output;
        output.key = key;
        output.start = mOutputScratch.size();

        for (std::size_t i = 0u; i < contactCount; ++i) {
          const auto& contact = *mRawEntries[index + i].contact;
          patch->points[patch->count].contact = contact;
          patch->points[patch->count].age = 0u;
          ++patch->count;
          mOutputScratch.push_back(contact);
          ++output.count;
        }

        mOutputRanges.push_back(std::move(output));
        index = end;
        continue;
      }

      freshCandidates.clear();
      staleCandidates.clear();
      existingCandidates.clear();

      freshCandidates.reserve(contactCount + patch->count);
      staleCandidates.reserve(patch->count);

      existingCandidates.reserve(patch->count);

      for (std::size_t i = 0u; i < patch->count; ++i) {
        Candidate candidate;
        candidate.contact = &patch->points[i].contact;
        candidate.age = patch->points[i].age;
        candidate.isFresh = false;
        existingCandidates.push_back(candidate);
      }

      matchedExisting.assign(existingCandidates.size(), 0u);
      matchedRaw.assign(contactCount, 0u);

      for (std::size_t i = 0u; i < contactCount; ++i) {
        const auto& raw = *mRawEntries[index + i].contact;
        std::size_t bestIndex = existingCandidates.size();
        double bestDistance = positionThresholdSquared;

        for (std::size_t j = 0u; j < existingCandidates.size(); ++j) {
          if (matchedExisting[j])
            continue;

          const auto& candidate = existingCandidates[j].contact;
          if (!isNear(
                  raw,
                  *candidate,
                  positionThresholdSquared,
                  normalThreshold,
                  normalThresholdSquared)) {
            continue;
          }

          const double dist = (raw.point - candidate->point).squaredNorm();
          if (dist <= bestDistance) {
            bestDistance = dist;
            bestIndex = j;
          }
        }

        if (bestIndex < existingCandidates.size()) {
          existingCandidates[bestIndex].contact = &raw;
          existingCandidates[bestIndex].age = 0u;
          existingCandidates[bestIndex].isFresh = true;
          matchedExisting[bestIndex] = 1u;
          matchedRaw[i] = 1u;
        }
      }

      for (std::size_t j = 0u; j < existingCandidates.size(); ++j) {
        if (!existingCandidates[j].isFresh) {
          if (existingCandidates[j].age
              < std::numeric_limits<std::uint8_t>::max())
            ++existingCandidates[j].age;
          staleCandidates.push_back(existingCandidates[j]);
        } else {
          freshCandidates.push_back(existingCandidates[j]);
        }
      }

      for (std::size_t i = 0u; i < contactCount; ++i) {
        if (matchedRaw[i])
          continue;

        Candidate candidate;
        candidate.contact = mRawEntries[index + i].contact;
        candidate.age = 0u;
        candidate.isFresh = true;
        freshCandidates.push_back(candidate);
      }

      const auto selected = selectRepresentativeContacts(
          std::move(freshCandidates),
          std::move(staleCandidates),
          maxPoints,
          positionThresholdSquared,
          options.normalThreshold);

      patch->count = 0u;
      patch->framesSinceSeen = 0u;
      patch->lastUpdateFrame = mFrameCounter;

      OutputRange output;
      output.key = key;
      output.start = mOutputScratch.size();
      const auto selectedCount
          = std::min<std::size_t>(selected.size(), kMaxManifoldPoints);
      std::array<collision::Contact, kMaxManifoldPoints> selectedContacts;
      std::array<std::uint8_t, kMaxManifoldPoints> selectedAges;

      for (std::size_t i = 0u; i < selectedCount; ++i) {
        selectedContacts[i] = *selected[i].contact;
        selectedAges[i] = selected[i].age;
      }

      for (std::size_t i = 0u; i < selectedCount; ++i) {
        patch->points[patch->count].contact = selectedContacts[i];
        patch->points[patch->count].age = selectedAges[i];
        ++patch->count;

        mOutputScratch.push_back(selectedContacts[i]);
        ++output.count;
      }

      mOutputRanges.push_back(std::move(output));
    } else {
      OutputRange output;
      output.key = key;
      output.start = mOutputScratch.size();
      const auto outputCount = std::min<std::size_t>(contactCount, maxPoints);
      for (std::size_t i = 0u; i < outputCount; ++i) {
        mOutputScratch.push_back(*mRawEntries[index + i].contact);
        ++output.count;
      }
      mOutputRanges.push_back(std::move(output));
    }

    index = end;
  }

  pruneStaleManifolds(options);

  std::size_t reserveCount = mOutputScratch.size();
  reserveCount += mManifolds.size() * maxPoints;

  outputContacts.reserve(reserveCount);

  for (const auto& output : mOutputRanges) {
    outputContacts.insert(
        outputContacts.end(),
        mOutputScratch.begin() + output.start,
        mOutputScratch.begin() + output.start + output.count);
  }

  for (const auto& [key, patch] : mManifolds) {
    if (patch.lastUpdateFrame == mFrameCounter)
      continue;
    if (patch.count == 0u)
      continue;
    const auto outputCount = std::min<std::size_t>(patch.count, maxPoints);
    for (std::size_t i = 0u; i < outputCount; ++i)
      outputContacts.push_back(patch.points[i].contact);
  }
}

//=============================================================================
std::size_t ContactManifoldCache::getNumManifolds() const
{
  return mManifolds.size();
}

//=============================================================================
std::uint32_t ContactManifoldCache::getFrameCounter() const
{
  return mFrameCounter;
}

//=============================================================================
ContactManifoldCache::PairKey ContactManifoldCache::makePairKey(
    collision::CollisionObject* first, collision::CollisionObject* second)
{
  if (std::less<collision::CollisionObject*>()(second, first))
    std::swap(first, second);

  return PairKey{first, second};
}

//=============================================================================
ContactManifoldCache::Manifold* ContactManifoldCache::findOrCreateManifold(
    const PairKey& key, const ContactManifoldCacheOptions& options)
{
  auto it = mManifolds.find(key);
  if (it != mManifolds.end()) {
    return &(it->second);
  }

  if (options.maxPairs > 0u && mManifolds.size() >= options.maxPairs)
    return nullptr;

  auto [insertIt, inserted] = mManifolds.emplace(key, Manifold{});
  insertIt->second.pair = key;
  return &(insertIt->second);
}

//=============================================================================
void ContactManifoldCache::pruneStaleManifolds(
    const ContactManifoldCacheOptions& options)
{
  std::erase_if(mManifolds, [&](const auto& entry) {
    return entry.second.framesSinceSeen > options.maxSeparationFrames;
  });
}

} // namespace constraint
} // namespace dart
