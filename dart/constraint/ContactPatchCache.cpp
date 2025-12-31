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

#include "dart/constraint/ContactPatchCache.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <utility>

namespace dart {
namespace constraint {
namespace {

struct Candidate
{
  collision::Contact contact;
  std::uint8_t age{0u};
  bool isFresh{false};
};

double normalizedDot(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const double normA = a.norm();
  const double normB = b.norm();
  const double denom = normA * normB;
  if (denom <= 0.0)
    return -1.0;
  return a.dot(b) / denom;
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

  if (contactLess(a.contact, b.contact))
    return true;
  if (contactLess(b.contact, a.contact))
    return false;

  return a.age < b.age;
}

bool isNear(
    const collision::Contact& a,
    const collision::Contact& b,
    double positionThresholdSquared,
    double normalThreshold)
{
  const Eigen::Vector3d delta = a.point - b.point;
  if (delta.squaredNorm() > positionThresholdSquared)
    return false;

  return normalizedDot(a.normal, b.normal) >= normalThreshold;
}

std::vector<Candidate> deduplicateCandidates(
    std::vector<Candidate> candidates,
    double positionThresholdSquared,
    double normalThreshold)
{
  std::sort(candidates.begin(), candidates.end(), candidateLess);

  std::vector<Candidate> unique;
  unique.reserve(candidates.size());

  for (const auto& candidate : candidates) {
    bool duplicate = false;
    for (const auto& kept : unique) {
      if (isNear(
              candidate.contact,
              kept.contact,
              positionThresholdSquared,
              normalThreshold)) {
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
    const std::vector<Candidate>& candidates,
    const std::vector<Candidate>& existing,
    double positionThresholdSquared,
    double normalThreshold)
{
  std::vector<Candidate> filtered;
  filtered.reserve(candidates.size());

  for (const auto& candidate : candidates) {
    bool duplicate = false;
    for (const auto& kept : existing) {
      if (isNear(
              candidate.contact,
              kept.contact,
              positionThresholdSquared,
              normalThreshold)) {
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
    if (contactLess(it->contact, best->contact))
      best = it;
  }
  return *best;
}

double minDistanceSquared(
    const collision::Contact& candidate,
    const std::vector<Candidate>& selected)
{
  double minDist = std::numeric_limits<double>::infinity();
  for (const auto& entry : selected) {
    const double dist = (candidate.point - entry.contact.point).squaredNorm();
    if (dist < minDist)
      minDist = dist;
  }
  return minDist;
}

bool candidateBetter(
    const Candidate& a,
    double aMinDist,
    const Candidate& b,
    double bMinDist)
{
  if (aMinDist != bMinDist)
    return aMinDist > bMinDist;

  if (a.isFresh != b.isFresh)
    return a.isFresh;

  if (contactLess(a.contact, b.contact))
    return true;
  if (contactLess(b.contact, a.contact))
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

  freshCandidates
      = deduplicateCandidates(freshCandidates, positionThresholdSquared,
          normalThreshold);
  staleCandidates
      = deduplicateCandidates(staleCandidates, positionThresholdSquared,
          normalThreshold);

  staleCandidates = filterNearExisting(
      staleCandidates, freshCandidates, positionThresholdSquared,
      normalThreshold);

  if (freshCandidates.empty())
    return {};

  std::vector<Candidate> selected;
  selected.reserve(maxPoints);

  Candidate seed = selectDeepest(freshCandidates);
  selected.push_back(seed);

  auto seedMatch = [&](const Candidate& candidate) {
    return !contactLess(candidate.contact, seed.contact)
        && !contactLess(seed.contact, candidate.contact);
  };

  freshCandidates.erase(
      std::remove_if(
          freshCandidates.begin(), freshCandidates.end(), seedMatch),
      freshCandidates.end());

  std::vector<Candidate> pool = freshCandidates;
  if (selected.size() + freshCandidates.size() < maxPoints) {
    pool.insert(pool.end(), staleCandidates.begin(), staleCandidates.end());
  }

  while (selected.size() < maxPoints && !pool.empty()) {
    auto bestIt = pool.begin();
    double bestDist = minDistanceSquared(bestIt->contact, selected);

    for (auto it = pool.begin(); it != pool.end(); ++it) {
      const double dist = minDistanceSquared(it->contact, selected);
      if (candidateBetter(*it, dist, *bestIt, bestDist)) {
        bestIt = it;
        bestDist = dist;
      }
    }

    selected.push_back(*bestIt);
    pool.erase(bestIt);
  }

  std::sort(selected.begin(), selected.end(), [](const Candidate& a, const Candidate& b) {
    return contactLess(a.contact, b.contact);
  });

  if (selected.size() > maxPoints)
    selected.resize(maxPoints);

  return selected;
}

} // namespace

//=============================================================================
void ContactPatchCache::reset()
{
  mPatches.clear();
  mFrameCounter = 0u;
}

//=============================================================================
void ContactPatchCache::update(
    const collision::CollisionResult& rawContacts,
    const ContactPatchCacheOptions& options,
    std::vector<collision::Contact>& outputContacts)
{
  outputContacts.clear();

  if (!options.enabled) {
    reset();
    return;
  }

  const auto maxPoints
      = std::min(options.maxPointsPerPair, kMaxPatchPoints);
  if (maxPoints == 0u) {
    reset();
    return;
  }

  ++mFrameCounter;

  for (auto& patch : mPatches) {
    if (patch.framesSinceSeen < std::numeric_limits<std::uint16_t>::max())
      ++patch.framesSinceSeen;
  }

  using PairMap
      = std::map<PairKey, std::vector<const collision::Contact*>, PairKeyLess>;

  PairMap contactsByPair;
  for (const auto& contact : rawContacts.getContacts()) {
    if (!contact.collisionObject1 || !contact.collisionObject2)
      continue;

    if (!isContactValid(contact))
      continue;

    const auto key
        = makePairKey(contact.collisionObject1, contact.collisionObject2);
    contactsByPair[key].push_back(&contact);
  }

  const double positionThresholdSquared
      = options.positionThreshold * options.positionThreshold;

  using OutputMap
      = std::map<PairKey, std::vector<collision::Contact>, PairKeyLess>;
  OutputMap outputByPair;

  for (const auto& entry : contactsByPair) {
    const auto& key = entry.first;
    const auto& contacts = entry.second;

    std::vector<const collision::Contact*> sortedContacts = contacts;
    std::sort(
        sortedContacts.begin(),
        sortedContacts.end(),
        [](const collision::Contact* a, const collision::Contact* b) {
          return contactLess(*a, *b);
        });

    Patch* patch = findOrCreatePatch(key, options);

    std::vector<Candidate> freshCandidates;
    std::vector<Candidate> staleCandidates;

    if (patch) {
      std::vector<Candidate> existingCandidates;
      existingCandidates.reserve(patch->count);

      for (std::size_t i = 0u; i < patch->count; ++i) {
        Candidate candidate;
        candidate.contact = patch->points[i].contact;
        candidate.age = patch->points[i].age;
        candidate.isFresh = false;
        existingCandidates.push_back(candidate);
      }

      std::vector<bool> matchedExisting(existingCandidates.size(), false);
      std::vector<bool> matchedRaw(sortedContacts.size(), false);

      for (std::size_t i = 0u; i < sortedContacts.size(); ++i) {
        const auto& raw = *sortedContacts[i];
        std::size_t bestIndex = existingCandidates.size();
        double bestDistance = positionThresholdSquared;

        for (std::size_t j = 0u; j < existingCandidates.size(); ++j) {
          if (matchedExisting[j])
            continue;

          const auto& candidate = existingCandidates[j].contact;
          if (!isNear(raw, candidate, positionThresholdSquared,
                  options.normalThreshold)) {
            continue;
          }

          const double dist
              = (raw.point - candidate.point).squaredNorm();
          if (dist <= bestDistance) {
            bestDistance = dist;
            bestIndex = j;
          }
        }

        if (bestIndex < existingCandidates.size()) {
          existingCandidates[bestIndex].contact = raw;
          existingCandidates[bestIndex].age = 0u;
          existingCandidates[bestIndex].isFresh = true;
          matchedExisting[bestIndex] = true;
          matchedRaw[i] = true;
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

      for (std::size_t i = 0u; i < sortedContacts.size(); ++i) {
        if (matchedRaw[i])
          continue;

        Candidate candidate;
        candidate.contact = *sortedContacts[i];
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

      auto& outputForPair = outputByPair[key];
      for (const auto& candidate : selected) {
        if (patch->count >= kMaxPatchPoints)
          break;

        patch->points[patch->count].contact = candidate.contact;
        patch->points[patch->count].age = candidate.age;
        ++patch->count;

        outputForPair.push_back(candidate.contact);
      }
    } else {
      for (const auto* contact : sortedContacts) {
        outputByPair[key].push_back(*contact);
        if (outputByPair[key].size() >= maxPoints)
          break;
      }
    }
  }

  pruneStalePatches(options);

  for (const auto& entry : outputByPair) {
    for (const auto& contact : entry.second) {
      outputContacts.push_back(contact);
    }
  }
}

//=============================================================================
std::size_t ContactPatchCache::getNumPatches() const
{
  return mPatches.size();
}

//=============================================================================
std::uint32_t ContactPatchCache::getFrameCounter() const
{
  return mFrameCounter;
}

//=============================================================================
ContactPatchCache::PairKey ContactPatchCache::makePairKey(
    collision::CollisionObject* first, collision::CollisionObject* second)
{
  if (std::less<collision::CollisionObject*>()(second, first))
    std::swap(first, second);

  return PairKey{first, second};
}

//=============================================================================
ContactPatchCache::Patch* ContactPatchCache::findOrCreatePatch(
    const PairKey& key, const ContactPatchCacheOptions& options)
{
  auto it = std::lower_bound(
      mPatches.begin(),
      mPatches.end(),
      key,
      [](const Patch& patch, const PairKey& searchKey) {
        PairKeyLess less;
        return less(patch.pair, searchKey);
      });

  if (it != mPatches.end() && it->pair.first == key.first
      && it->pair.second == key.second) {
    return &(*it);
  }

  if (options.maxPairs > 0u && mPatches.size() >= options.maxPairs)
    return nullptr;

  Patch patch;
  patch.pair = key;

  it = mPatches.insert(it, patch);
  return &(*it);
}

//=============================================================================
void ContactPatchCache::pruneStalePatches(
    const ContactPatchCacheOptions& options)
{
  if (options.maxSeparationFrames == 0u)
    return;

  mPatches.erase(
      std::remove_if(
          mPatches.begin(),
          mPatches.end(),
          [&](const Patch& patch) {
            return patch.framesSinceSeen > options.maxSeparationFrames;
          }),
      mPatches.end());
}

} // namespace constraint
} // namespace dart
