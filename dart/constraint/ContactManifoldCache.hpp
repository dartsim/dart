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

#ifndef DART_CONSTRAINT_CONTACTMANIFOLDCACHE_HPP_
#define DART_CONSTRAINT_CONTACTMANIFOLDCACHE_HPP_

#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/Contact.hpp>
#include <dart/collision/Fwd.hpp>

#include <dart/Export.hpp>

#include <array>
#include <functional>
#include <unordered_map>
#include <vector>

#include <cstdint>

namespace dart {
namespace constraint {

struct DART_API ContactManifoldCacheOptions
{
  /// Whether contact manifold caching is enabled. Default is true.
  /// Disabling is only recommended for debugging/comparison purposes.
  /// Contact manifolds provide stable contacts, solver warm-starting,
  /// and consistent friction - disabling them degrades simulation quality.
  bool enabled{true};

  /// Maximum contact points to retain per collision pair. Default is 4.
  std::size_t maxPointsPerPair{4u};

  /// Maximum number of collision pairs to cache. 0 means unlimited.
  std::size_t maxPairs{0u};

  /// Frames without contact before a manifold is pruned. Default is 4.
  std::size_t maxSeparationFrames{4u};

  /// Distance threshold for matching contacts across frames.
  double positionThreshold{1e-4};

  /// Dot product threshold for matching contact normals (0.99 ≈ 8°).
  double normalThreshold{0.99};

  /// Minimum penetration depth to consider a contact valid.
  double depthEpsilon{1e-6};
};

class DART_API ContactManifoldCache
{
public:
  ContactManifoldCache() = default;

  void reset();

  void update(
      const collision::CollisionResult& rawContacts,
      const ContactManifoldCacheOptions& options,
      std::vector<collision::Contact>& outputContacts);

  /// Remove all manifolds that reference the given collision object.
  /// Call this when a collision object is removed from the world.
  void invalidateCollisionObject(const collision::CollisionObject* object);

  /// Remove manifolds whose collision objects are no longer in the group.
  void purgeInvalidManifolds(const collision::CollisionGroup* group);

  std::size_t getNumManifolds() const;
  std::uint32_t getFrameCounter() const;

private:
  static constexpr std::size_t kMaxManifoldPoints = 4u;

  struct ManifoldPoint
  {
    collision::Contact contact;
    std::uint8_t age{0u};
  };

  struct PairKey
  {
    collision::CollisionObject* first{nullptr};
    collision::CollisionObject* second{nullptr};
  };

  struct PairKeyHash
  {
    std::size_t operator()(const PairKey& key) const noexcept
    {
      auto h1 = std::hash<std::uintptr_t>{}(
          reinterpret_cast<std::uintptr_t>(key.first));
      auto h2 = std::hash<std::uintptr_t>{}(
          reinterpret_cast<std::uintptr_t>(key.second));
      return h1 ^ (h2 << 1);
    }
  };

  struct PairKeyEqual
  {
    bool operator()(const PairKey& a, const PairKey& b) const noexcept
    {
      return a.first == b.first && a.second == b.second;
    }
  };

  struct Manifold
  {
    PairKey pair;
    std::array<ManifoldPoint, kMaxManifoldPoints> points{};
    std::uint8_t count{0u};
    std::uint16_t framesSinceSeen{0u};
    std::uint32_t lastUpdateFrame{0u};
  };

  struct RawEntry
  {
    PairKey key;
    const collision::Contact* contact{nullptr};
  };

  struct OutputRange
  {
    PairKey key;
    std::size_t start{0u};
    std::size_t count{0u};
  };

  static PairKey makePairKey(
      collision::CollisionObject* first, collision::CollisionObject* second);

  Manifold* findOrCreateManifold(
      const PairKey& key, const ContactManifoldCacheOptions& options);

  void pruneStaleManifolds(const ContactManifoldCacheOptions& options);

  std::unordered_map<PairKey, Manifold, PairKeyHash, PairKeyEqual> mManifolds;
  std::vector<RawEntry> mRawEntries;
  std::vector<OutputRange> mOutputRanges;
  std::vector<collision::Contact> mOutputScratch;
  std::uint32_t mFrameCounter{0u};
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_CONTACTMANIFOLDCACHE_HPP_
