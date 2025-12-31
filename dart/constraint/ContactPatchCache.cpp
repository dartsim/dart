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

namespace dart {
namespace constraint {

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

    const auto key
        = makePairKey(contact.collisionObject1, contact.collisionObject2);
    contactsByPair[key].push_back(&contact);
  }

  for (const auto& entry : contactsByPair) {
    const auto& key = entry.first;
    const auto& contacts = entry.second;

    auto* patch = findOrCreatePatch(key, options);
    if (!patch)
      continue;

    patch->count = 0u;
    patch->framesSinceSeen = 0u;
    patch->lastUpdateFrame = mFrameCounter;

    const auto toCopy = std::min<std::size_t>(maxPoints, contacts.size());
    for (std::size_t i = 0u; i < toCopy; ++i) {
      patch->points[i].contact = *contacts[i];
      patch->points[i].age = 0u;
      ++patch->count;
    }
  }

  pruneStalePatches(options);

  for (const auto& patch : mPatches) {
    if (patch.framesSinceSeen != 0u)
      continue;

    for (std::size_t i = 0u; i < patch.count; ++i) {
      outputContacts.push_back(patch.points[i].contact);
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
