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

//==============================================================================
BruteForceBroadPhase::Entry BruteForceBroadPhase::makeEntry(
    std::size_t id, const Aabb& aabb)
{
  return BruteForceBroadPhase::Entry{
      id,
      aabb,
      aabb.min.x(),
      aabb.min.y(),
      aabb.min.z(),
      aabb.max.x(),
      aabb.max.y(),
      aabb.max.z()};
}

//==============================================================================
void BruteForceBroadPhase::updateEntryAabb(Entry& entry, const Aabb& aabb)
{
  entry.aabb = aabb;
  entry.minX = aabb.min.x();
  entry.minY = aabb.min.y();
  entry.minZ = aabb.min.z();
  entry.maxX = aabb.max.x();
  entry.maxY = aabb.max.y();
  entry.maxZ = aabb.max.z();
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
  entries_.clear();
  indices_.clear();
}

void BruteForceBroadPhase::add(std::size_t id, const Aabb& aabb)
{
  const auto existing = indices_.find(id);
  if (existing != indices_.end()) {
    updateEntryAabb(entries_[existing->second], aabb);
    return;
  }

  const auto position = std::lower_bound(
      entries_.begin(),
      entries_.end(),
      id,
      [](const Entry& entry, std::size_t value) { return entry.id < value; });
  const auto index
      = static_cast<std::size_t>(std::distance(entries_.begin(), position));
  entries_.insert(position, makeEntry(id, aabb));
  rebuildIndicesFrom(index);
}

void BruteForceBroadPhase::update(std::size_t id, const Aabb& aabb)
{
  const auto it = indices_.find(id);
  if (it != indices_.end())
    updateEntryAabb(entries_[it->second], aabb);
}

void BruteForceBroadPhase::updateRange(
    span<const std::size_t> ids, span<const Aabb> aabbs)
{
  const std::size_t n = std::min(ids.size(), aabbs.size());

  if (n == entries_.size()) {
    bool sameOrder = true;
    for (std::size_t i = 0u; i < n; ++i) {
      if (entries_[i].id != ids[i]) {
        sameOrder = false;
        break;
      }
    }

    if (sameOrder) {
      for (std::size_t i = 0u; i < n; ++i)
        updateEntryAabb(entries_[i], aabbs[i]);
      return;
    }
  }

  for (std::size_t i = 0u; i < n; ++i)
    update(ids[i], aabbs[i]);
}

void BruteForceBroadPhase::remove(std::size_t id)
{
  const auto it = indices_.find(id);
  if (it == indices_.end())
    return;

  const auto index = it->second;
  entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(index));
  indices_.erase(it);
  rebuildIndicesFrom(index);
}

std::vector<BroadPhasePair> BruteForceBroadPhase::queryPairs() const
{
  std::vector<BroadPhasePair> pairs;

  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const auto& entry1 = entries_[i];
    for (std::size_t j = i + 1; j < entries_.size(); ++j) {
      const auto& entry2 = entries_[j];
      if (overlapsFast(entry1, entry2))
        pairs.emplace_back(entry1.id, entry2.id);
    }
  }

  return pairs;
}

bool BruteForceBroadPhase::visitPairs(
    const BroadPhasePairVisitor& visitor) const
{
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const auto& entry1 = entries_[i];
    for (std::size_t j = i + 1; j < entries_.size(); ++j) {
      const auto& entry2 = entries_[j];
      if (overlapsFast(entry1, entry2) && !visitor(entry1.id, entry2.id)) {
        return false;
      }
    }
  }

  return true;
}

std::vector<std::size_t> BruteForceBroadPhase::queryOverlapping(
    const Aabb& aabb) const
{
  std::vector<std::size_t> result;

  for (const auto& entry : entries_) {
    if (overlapsFast(entry, aabb))
      result.push_back(entry.id);
  }

  return result;
}

std::size_t BruteForceBroadPhase::size() const
{
  return entries_.size();
}

void BruteForceBroadPhase::rebuildIndicesFrom(std::size_t begin)
{
  for (std::size_t i = begin; i < entries_.size(); ++i)
    indices_[entries_[i].id] = i;
}

} // namespace dart::collision::native
