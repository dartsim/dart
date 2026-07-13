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

#include <dart/collision/native/broad_phase/BruteForce.hpp>

#include <algorithm>

namespace dart::collision::native {

namespace {

class Release620BruteForceBroadPhaseLayout : public BroadPhase
{
private:
  std::unordered_map<std::size_t, Aabb> objects_;
  std::vector<std::size_t> orderedIds_;
};

static_assert(
    sizeof(BruteForceBroadPhase)
        == sizeof(Release620BruteForceBroadPhaseLayout),
    "BruteForceBroadPhase must preserve its DART 6.20 binary layout");
static_assert(
    alignof(BruteForceBroadPhase)
        == alignof(Release620BruteForceBroadPhaseLayout),
    "BruteForceBroadPhase must preserve its DART 6.20 alignment");

} // namespace

//==============================================================================
BruteForceBroadPhase::Entry BruteForceBroadPhase::makeEntry(
    std::size_t id, const Aabb& aabb)
{
  return BruteForceBroadPhase::Entry{
      id,
      aabb.min.x(),
      aabb.min.y(),
      aabb.min.z(),
      aabb.max.x(),
      aabb.max.y(),
      aabb.max.z()};
}

//==============================================================================
void BruteForceBroadPhase::storeEntry(std::size_t index, const Entry& entry)
{
  const std::size_t offset = index * kEntryWordCount;
  orderedIds_[offset] = entry.id;

  const auto storeDouble
      = [this, offset](std::size_t coordinate, double value) {
          auto* const begin = orderedIds_.data() + offset + 1u
                              + coordinate * kDoubleWordCount;
          std::fill_n(begin, kDoubleWordCount, 0u);
          std::memcpy(begin, &value, sizeof(value));
        };
  storeDouble(0u, entry.minX);
  storeDouble(1u, entry.minY);
  storeDouble(2u, entry.minZ);
  storeDouble(3u, entry.maxX);
  storeDouble(4u, entry.maxY);
  storeDouble(5u, entry.maxZ);
}

//==============================================================================
std::size_t BruteForceBroadPhase::findEntryIndex(std::size_t id) const
{
  std::size_t first = 0u;
  std::size_t last = entryCount();
  while (first < last) {
    const std::size_t middle = first + (last - first) / 2u;
    if (loadEntry(middle).id < id)
      first = middle + 1u;
    else
      last = middle;
  }

  return first;
}

//==============================================================================
void BruteForceBroadPhase::rebuildEntries()
{
  const std::size_t count = objects_.size();
  orderedIds_.clear();
  orderedIds_.reserve(count * kEntryWordCount);

  for (const auto& object : objects_)
    orderedIds_.push_back(object.first);

  std::sort(orderedIds_.begin(), orderedIds_.end());
  orderedIds_.resize(count * kEntryWordCount);

  // Expand from the back so the packed records do not overwrite IDs that have
  // not been consumed yet.
  for (std::size_t i = count; i > 0u; --i) {
    const std::size_t id = orderedIds_[i - 1u];
    storeEntry(i - 1u, makeEntry(id, objects_.at(id)));
  }
}

//==============================================================================
void BruteForceBroadPhase::rebuildOrderedIds()
{
  rebuildEntries();
}

//==============================================================================
bool BruteForceBroadPhase::overlapsFast(const Entry& a, const Entry& b)
{
  return (a.minX <= b.maxX && a.maxX >= b.minX)
         && (a.minY <= b.maxY && a.maxY >= b.minY)
         && (a.minZ <= b.maxZ && a.maxZ >= b.minZ);
}

//==============================================================================
bool BruteForceBroadPhase::overlapsFast(const Entry& entry, const Aabb& aabb)
{
  return (entry.minX <= aabb.max.x() && entry.maxX >= aabb.min.x())
         && (entry.minY <= aabb.max.y() && entry.maxY >= aabb.min.y())
         && (entry.minZ <= aabb.max.z() && entry.maxZ >= aabb.min.z());
}

//==============================================================================
void BruteForceBroadPhase::clear()
{
  objects_.clear();
  orderedIds_.clear();
}

void BruteForceBroadPhase::add(std::size_t id, const Aabb& aabb)
{
  const auto existing = objects_.find(id);
  if (existing != objects_.end()) {
    existing->second = aabb;
    const std::size_t index = findEntryIndex(id);
    if (index < entryCount() && loadEntry(index).id == id)
      storeEntry(index, makeEntry(id, aabb));
    else
      rebuildEntries();
    return;
  }

  const std::size_t oldCount = objects_.size();
  objects_.emplace(id, aabb);
  if (entryCount() != oldCount) {
    rebuildEntries();
    return;
  }

  const std::size_t index = findEntryIndex(id);
  orderedIds_.insert(
      orderedIds_.begin()
          + static_cast<std::ptrdiff_t>(index * kEntryWordCount),
      kEntryWordCount,
      0u);
  storeEntry(index, makeEntry(id, aabb));
}

void BruteForceBroadPhase::update(std::size_t id, const Aabb& aabb)
{
  const auto object = objects_.find(id);
  if (object == objects_.end())
    return;

  object->second = aabb;
  const std::size_t index = findEntryIndex(id);
  if (index < entryCount() && loadEntry(index).id == id)
    storeEntry(index, makeEntry(id, aabb));
  else
    rebuildEntries();
}

void BruteForceBroadPhase::updateRange(
    span<const std::size_t> ids, span<const Aabb> aabbs)
{
  const std::size_t n = std::min(ids.size(), aabbs.size());

  if (n == entryCount()) {
    bool sameOrder = true;
    for (std::size_t i = 0u; i < n; ++i) {
      if (loadEntry(i).id != ids[i]) {
        sameOrder = false;
        break;
      }
    }

    if (sameOrder) {
      for (std::size_t i = 0u; i < n; ++i) {
        const auto object = objects_.find(ids[i]);
        if (object == objects_.end()) {
          sameOrder = false;
          break;
        }
        object->second = aabbs[i];
        storeEntry(i, makeEntry(ids[i], aabbs[i]));
      }

      if (sameOrder)
        return;

      rebuildEntries();
      return;
    }
  }

  for (std::size_t i = 0u; i < n; ++i)
    update(ids[i], aabbs[i]);
}

void BruteForceBroadPhase::remove(std::size_t id)
{
  const auto object = objects_.find(id);
  if (object == objects_.end())
    return;

  const std::size_t index = findEntryIndex(id);
  const bool hasEntry = index < entryCount() && loadEntry(index).id == id;
  objects_.erase(object);

  if (!hasEntry) {
    rebuildEntries();
    return;
  }

  const auto begin = orderedIds_.begin()
                     + static_cast<std::ptrdiff_t>(index * kEntryWordCount);
  orderedIds_.erase(
      begin, begin + static_cast<std::ptrdiff_t>(kEntryWordCount));
}

std::vector<BroadPhasePair> BruteForceBroadPhase::queryPairs() const
{
  std::vector<BroadPhasePair> pairs;

  const std::size_t count = entryCount();
  for (std::size_t i = 0u; i < count; ++i) {
    const Entry entry1 = loadEntry(i);
    for (std::size_t j = i + 1u; j < count; ++j) {
      const Entry entry2 = loadEntry(j);
      if (overlapsFast(entry1, entry2))
        pairs.emplace_back(entry1.id, entry2.id);
    }
  }

  return pairs;
}

bool BruteForceBroadPhase::visitPairs(
    const BroadPhasePairVisitor& visitor) const
{
  return visitPairsInline(visitor);
}

std::vector<std::size_t> BruteForceBroadPhase::queryOverlapping(
    const Aabb& aabb) const
{
  std::vector<std::size_t> result;

  const std::size_t count = entryCount();
  for (std::size_t i = 0u; i < count; ++i) {
    const Entry entry = loadEntry(i);
    if (overlapsFast(entry, aabb))
      result.push_back(entry.id);
  }

  return result;
}

std::size_t BruteForceBroadPhase::size() const
{
  return objects_.size();
}

} // namespace dart::collision::native
