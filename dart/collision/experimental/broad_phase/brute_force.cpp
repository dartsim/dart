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

#include <dart/collision/experimental/broad_phase/brute_force.hpp>

#include <algorithm>

namespace dart::collision::experimental {

void BruteForceBroadPhase::clear()
{
  objects_.clear();
  orderedIds_.clear();
}

void BruteForceBroadPhase::add(std::size_t id, const Aabb& aabb)
{
  objects_[id] = aabb;
  rebuildOrderedIds();
}

void BruteForceBroadPhase::update(std::size_t id, const Aabb& aabb)
{
  auto it = objects_.find(id);
  if (it != objects_.end()) {
    it->second = aabb;
  }
}

void BruteForceBroadPhase::remove(std::size_t id)
{
  objects_.erase(id);
  rebuildOrderedIds();
}

std::vector<BroadPhasePair> BruteForceBroadPhase::queryPairs() const
{
  std::vector<BroadPhasePair> pairs;

  for (std::size_t i = 0; i < orderedIds_.size(); ++i) {
    for (std::size_t j = i + 1; j < orderedIds_.size(); ++j) {
      const std::size_t id1 = orderedIds_[i];
      const std::size_t id2 = orderedIds_[j];

      const Aabb& aabb1 = objects_.at(id1);
      const Aabb& aabb2 = objects_.at(id2);

      if (aabb1.overlaps(aabb2)) {
        pairs.emplace_back(id1, id2);
      }
    }
  }

  return pairs;
}

std::vector<std::size_t> BruteForceBroadPhase::queryOverlapping(
    const Aabb& aabb) const
{
  std::vector<std::size_t> result;

  for (const std::size_t id : orderedIds_) {
    if (objects_.at(id).overlaps(aabb)) {
      result.push_back(id);
    }
  }

  return result;
}

std::size_t BruteForceBroadPhase::size() const
{
  return objects_.size();
}

void BruteForceBroadPhase::rebuildOrderedIds()
{
  orderedIds_.clear();
  orderedIds_.reserve(objects_.size());

  for (const auto& [id, aabb] : objects_) {
    orderedIds_.push_back(id);
  }

  std::sort(orderedIds_.begin(), orderedIds_.end());
}

} // namespace dart::collision::experimental
