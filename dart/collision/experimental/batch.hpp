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
#include <Eigen/StdVector>

#include <limits>
#include <span>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::collision::experimental {

class Shape;

using ObjectId = std::size_t;

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

struct DART_COLLISION_EXPERIMENTAL_API BatchView
{
  std::span<const ObjectId> ids;
  std::span<const Shape* const> shapes;
  std::span<const Eigen::Isometry3d> transforms;
  std::span<const Aabb> aabbs;
  std::span<const std::uint8_t> flags;
  std::span<const std::size_t> idToIndex;
  std::size_t invalidIndex = std::numeric_limits<std::size_t>::max();

  [[nodiscard]] std::size_t size() const
  {
    return ids.size();
  }

  [[nodiscard]] std::size_t indexForId(ObjectId id) const
  {
    if (id >= idToIndex.size()) {
      return invalidIndex;
    }
    return idToIndex[id];
  }

  [[nodiscard]] const Shape* shape(ObjectId id) const
  {
    const std::size_t index = indexForId(id);
    if (index == invalidIndex || index >= shapes.size()) {
      return nullptr;
    }
    return shapes[index];
  }

  [[nodiscard]] const Eigen::Isometry3d* transform(ObjectId id) const
  {
    const std::size_t index = indexForId(id);
    if (index == invalidIndex || index >= transforms.size()) {
      return nullptr;
    }
    return &transforms[index];
  }

  [[nodiscard]] const Aabb* aabb(ObjectId id) const
  {
    const std::size_t index = indexForId(id);
    if (index == invalidIndex || index >= aabbs.size()) {
      return nullptr;
    }
    return &aabbs[index];
  }
};

struct DART_COLLISION_EXPERIMENTAL_API BatchStorage
{
  static constexpr std::size_t kInvalidIndex
      = std::numeric_limits<std::size_t>::max();

  std::vector<ObjectId> ids;
  std::vector<const Shape*> shapes;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      transforms;
  std::vector<Aabb, Eigen::aligned_allocator<Aabb>> aabbs;
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
    if (count > idToIndex.size()) {
      idToIndex.resize(count, kInvalidIndex);
    }
  }

  [[nodiscard]] std::size_t size() const
  {
    return ids.size();
  }

  void add(
      ObjectId id,
      const Shape* shape,
      const Eigen::Isometry3d& transform,
      const Aabb& aabb)
  {
    if (id >= idToIndex.size()) {
      idToIndex.resize(id + 1, kInvalidIndex);
    }
    if (idToIndex[id] != kInvalidIndex) {
      update(id, shape, transform, aabb);
      return;
    }
    idToIndex[id] = ids.size();
    ids.push_back(id);
    shapes.push_back(shape);
    transforms.push_back(transform);
    aabbs.push_back(aabb);
    flags.push_back(0);
  }

  void update(
      ObjectId id,
      const Shape* shape,
      const Eigen::Isometry3d& transform,
      const Aabb& aabb)
  {
    if (id >= idToIndex.size()) {
      return;
    }
    const std::size_t index = idToIndex[id];
    if (index == kInvalidIndex || index >= ids.size()) {
      return;
    }
    shapes[index] = shape;
    transforms[index] = transform;
    aabbs[index] = aabb;
  }

  void remove(ObjectId id)
  {
    if (id >= idToIndex.size()) {
      return;
    }
    const std::size_t index = idToIndex[id];
    if (index == kInvalidIndex || index >= ids.size()) {
      return;
    }
    const std::size_t lastIndex = ids.size() - 1;
    if (index != lastIndex) {
      ids[index] = ids[lastIndex];
      shapes[index] = shapes[lastIndex];
      transforms[index] = transforms[lastIndex];
      aabbs[index] = aabbs[lastIndex];
      flags[index] = flags[lastIndex];
      idToIndex[ids[index]] = index;
    }
    ids.pop_back();
    shapes.pop_back();
    transforms.pop_back();
    aabbs.pop_back();
    flags.pop_back();
    idToIndex[id] = kInvalidIndex;
  }

  [[nodiscard]] BatchView view() const
  {
    BatchView view;
    view.ids = std::span<const ObjectId>(ids.data(), ids.size());
    view.shapes = std::span<const Shape* const>(shapes.data(), shapes.size());
    view.transforms = std::span<const Eigen::Isometry3d>(
        transforms.data(), transforms.size());
    view.aabbs = std::span<const Aabb>(aabbs.data(), aabbs.size());
    view.flags = std::span<const std::uint8_t>(flags.data(), flags.size());
    view.idToIndex
        = std::span<const std::size_t>(idToIndex.data(), idToIndex.size());
    view.invalidIndex = kInvalidIndex;
    return view;
  }
};

} // namespace dart::collision::experimental
