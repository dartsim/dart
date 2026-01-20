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

#include <dart/collision/experimental/broad_phase/sweep_and_prune.hpp>

#include <algorithm>
#include <set>

namespace dart::collision::experimental {

void SweepAndPruneBroadPhase::clear()
{
  objects_.clear();
  for (int i = 0; i < 3; ++i) {
    endpoints_[i].clear();
  }
  dirty_ = false;
}

void SweepAndPruneBroadPhase::add(std::size_t id, const Aabb& aabb)
{
  objects_[id] = aabb;
  dirty_ = true;
}

void SweepAndPruneBroadPhase::update(std::size_t id, const Aabb& aabb)
{
  auto it = objects_.find(id);
  if (it != objects_.end()) {
    it->second = aabb;
    dirty_ = true;
  }
}

void SweepAndPruneBroadPhase::remove(std::size_t id)
{
  objects_.erase(id);
  dirty_ = true;
}

std::vector<BroadPhasePair> SweepAndPruneBroadPhase::queryPairs() const
{
  if (dirty_) {
    rebuildEndpoints();
  }

  std::vector<BroadPhasePair> pairs;
  if (objects_.size() < 2) {
    return pairs;
  }

  std::set<std::size_t> activeOnXAxis;

  for (const auto& endpoint : endpoints_[0]) {
    if (endpoint.isMin) {
      for (std::size_t activeId : activeOnXAxis) {
        bool overlapsY = overlapsOnAxis(endpoint.objectId, activeId, 1);
        bool overlapsZ = overlapsOnAxis(endpoint.objectId, activeId, 2);
        if (overlapsY && overlapsZ) {
          auto id1 = std::min(endpoint.objectId, activeId);
          auto id2 = std::max(endpoint.objectId, activeId);
          pairs.emplace_back(id1, id2);
        }
      }
      activeOnXAxis.insert(endpoint.objectId);
    } else {
      activeOnXAxis.erase(endpoint.objectId);
    }
  }

  std::sort(pairs.begin(), pairs.end());
  return pairs;
}

std::vector<std::size_t> SweepAndPruneBroadPhase::queryOverlapping(
    const Aabb& aabb) const
{
  std::vector<std::size_t> results;

  for (const auto& [id, objAabb] : objects_) {
    if (aabb.overlaps(objAabb)) {
      results.push_back(id);
    }
  }

  std::sort(results.begin(), results.end());
  return results;
}

std::size_t SweepAndPruneBroadPhase::size() const
{
  return objects_.size();
}

void SweepAndPruneBroadPhase::rebuildEndpoints() const
{
  for (int axis = 0; axis < 3; ++axis) {
    endpoints_[axis].clear();
    endpoints_[axis].reserve(objects_.size() * 2);

    for (const auto& [id, aabb] : objects_) {
      Endpoint minEp;
      minEp.value = aabb.min[axis];
      minEp.objectId = id;
      minEp.isMin = true;

      Endpoint maxEp;
      maxEp.value = aabb.max[axis];
      maxEp.objectId = id;
      maxEp.isMin = false;

      endpoints_[axis].push_back(minEp);
      endpoints_[axis].push_back(maxEp);
    }

    std::sort(endpoints_[axis].begin(), endpoints_[axis].end());
  }

  dirty_ = false;
}

bool SweepAndPruneBroadPhase::overlapsOnAxis(
    std::size_t id1, std::size_t id2, int axis) const
{
  auto it1 = objects_.find(id1);
  auto it2 = objects_.find(id2);
  if (it1 == objects_.end() || it2 == objects_.end()) {
    return false;
  }

  const Aabb& a = it1->second;
  const Aabb& b = it2->second;

  return a.min[axis] <= b.max[axis] && b.min[axis] <= a.max[axis];
}

} // namespace dart::collision::experimental
