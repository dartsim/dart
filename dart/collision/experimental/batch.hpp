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

#pragma once

#include <dart/collision/experimental/broad_phase/broad_phase.hpp>
#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/types.hpp>

#include <Eigen/Geometry>

#include <limits>
#include <span>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::collision::experimental {

struct DART_COLLISION_EXPERIMENTAL_API BatchSettings
{
  int maxThreads = 1;
  bool deterministic = true;
  bool collectStats = false;
  std::size_t grainSize = 256;
};

struct DART_COLLISION_EXPERIMENTAL_API BatchScratch
{
  void reset() {}
};

struct DART_COLLISION_EXPERIMENTAL_API BatchTimings
{
  std::uint64_t aabbUpdateNs = 0;
  std::uint64_t broadPhaseNs = 0;
  std::uint64_t narrowPhaseNs = 0;
  std::uint64_t mergeNs = 0;

  void clear()
  {
    aabbUpdateNs = 0;
    broadPhaseNs = 0;
    narrowPhaseNs = 0;
    mergeNs = 0;
  }
};

struct DART_COLLISION_EXPERIMENTAL_API BatchStats
{
  std::size_t numObjects = 0;
  std::size_t numAabbUpdates = 0;
  std::size_t numPairs = 0;
  std::size_t numPairsTested = 0;
  std::size_t numContacts = 0;
  std::size_t pairBytes = 0;
  std::size_t contactBytes = 0;
  std::size_t tempBytes = 0;
  BatchTimings timings;

  void clear()
  {
    numObjects = 0;
    numAabbUpdates = 0;
    numPairs = 0;
    numPairsTested = 0;
    numContacts = 0;
    pairBytes = 0;
    contactBytes = 0;
    tempBytes = 0;
    timings.clear();
  }
};

struct DART_COLLISION_EXPERIMENTAL_API BatchOutput
{
  CollisionResult result;
  std::vector<BroadPhasePair> pairs;

  void clear()
  {
    result.clear();
    pairs.clear();
  }
};

struct DART_COLLISION_EXPERIMENTAL_API BroadPhaseSnapshot
{
  std::vector<BroadPhasePair> pairs;
  std::size_t numObjects = 0;

  void clear()
  {
    pairs.clear();
    numObjects = 0;
  }
};

constexpr std::size_t kInvalidBatchIndex
    = std::numeric_limits<std::size_t>::max();

constexpr std::uint8_t kBatchDirty = 1u << 0;
constexpr std::uint8_t kBatchEnabled = 1u << 1;
constexpr std::uint8_t kBatchStatic = 1u << 2;

struct BatchView;

struct DART_COLLISION_EXPERIMENTAL_API BatchStorage
{
  std::vector<ObjectId> ids;
  std::vector<const Shape*> shapes;
  std::vector<Eigen::Isometry3d> transforms;
  std::vector<Aabb> aabbs;
  std::vector<std::uint8_t> flags;
  std::vector<std::size_t> idToIndex;

  void clear()
  {
    ids.clear();
    shapes.clear();
    transforms.clear();
    aabbs.clear();
    flags.clear();
    idToIndex.clear();
  }

  void reserve(std::size_t count)
  {
    ids.reserve(count);
    shapes.reserve(count);
    transforms.reserve(count);
    aabbs.reserve(count);
    flags.reserve(count);
  }

  void resetIndex(std::size_t size)
  {
    idToIndex.assign(size, kInvalidBatchIndex);
  }

  [[nodiscard]] std::size_t indexFor(ObjectId id) const
  {
    if (id >= idToIndex.size()) {
      return kInvalidBatchIndex;
    }
    return idToIndex[id];
  }

  [[nodiscard]] bool contains(ObjectId id) const
  {
    return indexFor(id) != kInvalidBatchIndex;
  }

  [[nodiscard]] BatchView view() const;
};

struct DART_COLLISION_EXPERIMENTAL_API BatchView
{
  std::span<const ObjectId> ids;
  std::span<const Shape*> shapes;
  std::span<const Eigen::Isometry3d> transforms;
  std::span<const Aabb> aabbs;
  std::span<const std::uint8_t> flags;
  std::span<const std::size_t> idToIndex;

  [[nodiscard]] std::size_t size() const
  {
    return ids.size();
  }

  [[nodiscard]] std::size_t indexFor(ObjectId id) const
  {
    if (id >= idToIndex.size()) {
      return kInvalidBatchIndex;
    }
    return idToIndex[id];
  }

  [[nodiscard]] bool contains(ObjectId id) const
  {
    return indexFor(id) != kInvalidBatchIndex;
  }

  [[nodiscard]] const Shape* shape(ObjectId id) const
  {
    const auto index = indexFor(id);
    return index == kInvalidBatchIndex ? nullptr : shapes[index];
  }

  [[nodiscard]] const Eigen::Isometry3d* transform(ObjectId id) const
  {
    const auto index = indexFor(id);
    return index == kInvalidBatchIndex ? nullptr : &transforms[index];
  }

  [[nodiscard]] const Aabb* aabb(ObjectId id) const
  {
    const auto index = indexFor(id);
    return index == kInvalidBatchIndex ? nullptr : &aabbs[index];
  }

  [[nodiscard]] std::uint8_t flagsFor(ObjectId id) const
  {
    const auto index = indexFor(id);
    return index == kInvalidBatchIndex ? 0u : flags[index];
  }
};

inline BatchView BatchStorage::view() const
{
  BatchView view;
  view.ids = ids;
  view.shapes = shapes;
  view.transforms = transforms;
  view.aabbs = aabbs;
  view.flags = flags;
  view.idToIndex = idToIndex;
  return view;
}

} // namespace dart::collision::experimental
