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

#include <dart/collision/dart/broad_phase/BroadPhase.hpp>

#include <array>
#include <limits>
#include <vector>

#include <cstdint>

namespace dart::collision::native {

/// Dynamic AABB Tree using Surface Area Heuristic (SAH) for O(n log n)
/// broad-phase. Uses fat AABBs to reduce update frequency when objects move.
/// Object ids index dense storage, so memory grows with the largest id: keep
/// ids compact and reuse freed ones, as DARTCollisionGroup does.
class DART_COLLISION_NATIVE_API AabbTreeBroadPhase : public BroadPhase
{
public:
  static constexpr std::size_t kNullNode
      = std::numeric_limits<std::size_t>::max();
  static constexpr double kDefaultFatAabbMargin = 0.1;

  explicit AabbTreeBroadPhase(double fatAabbMargin = kDefaultFatAabbMargin);

  void clear() override;
  void add(std::size_t id, const Aabb& aabb) override;
  void update(std::size_t id, const Aabb& aabb) override;
  void remove(std::size_t id) override;

  [[nodiscard]] std::vector<BroadPhasePair> queryPairs() const override;
  [[nodiscard]] std::vector<std::size_t> queryOverlapping(
      const Aabb& aabb) const override;
  [[nodiscard]] std::size_t size() const override;

  void queryPairs(std::vector<BroadPhasePair>& out) const override;
  bool visitPairs(const BroadPhasePairVisitor& visitor) const override;
  /// Visits candidates without an ordering or uniqueness guarantee, allowing
  /// order-independent queries to stop without materializing the pair set.
  bool visitPairsAnyOrder(const BroadPhasePairVisitor& visitor) const;
  void buildDebugSnapshot(BroadPhaseDebugSnapshot& out) const override;

  void build(span<const std::size_t> ids, span<const Aabb> aabbs) override;
  void updateRange(
      span<const std::size_t> ids, span<const Aabb> aabbs) override;

  [[nodiscard]] double getFatAabbMargin() const;
  void setFatAabbMargin(double margin);
  [[nodiscard]] std::size_t getHeight() const;
  [[nodiscard]] bool validate() const;

private:
  // The private layout is ABI from DART 6.20.0 on; later changes need a pimpl.
  using NodeIndex = std::int32_t;
  static constexpr NodeIndex kInvalidNode = -1;

  struct alignas(64) Node
  {
    Aabb fatAabb;
    NodeIndex left = kInvalidNode;
    NodeIndex right = kInvalidNode;
    std::int32_t height = 0;
    // A leaf stores its own id; an internal node stores the maximum below it.
    std::uint32_t maxObjectId = 0;

    [[nodiscard]] bool isLeaf() const
    {
      return left == kInvalidNode;
    }
  };
  static_assert(sizeof(Node) == 64, "AABB tree nodes must remain 64 bytes");

  std::vector<Node> nodes_;
  // Queries do not read parents. Free nodes reuse this link for the free list.
  std::vector<NodeIndex> parents_;
  NodeIndex root_ = kInvalidNode;
  std::size_t nodeCount_ = 0;
  NodeIndex freeList_ = kInvalidNode;
  std::vector<NodeIndex> objectToNode_;
  std::array<std::vector<double>, 3> tightMin_;
  std::array<std::vector<double>, 3> tightMax_;
  double fatAabbMargin_;

  // Membership changes invalidate the sorted-id cache; updates leave it intact.
  mutable std::vector<std::size_t> orderedIds_;
  mutable bool idsSorted_ = true;
  // Reused query scratch keeps prepared steady-state stepping allocation-free.
  // The tree is single-query at a time by contract.
  mutable std::vector<std::size_t> mOverlapScratch;
  mutable std::vector<NodeIndex> mQueryStack;

  void setTightAabb(std::size_t id, const Aabb& aabb);
  [[nodiscard]] Aabb tightAabb(std::size_t id) const;
  [[nodiscard]] bool overlapsTight(std::size_t id, const Aabb& aabb) const;
  [[nodiscard]] NodeIndex allocateNode();
  void freeNode(NodeIndex nodeIndex);
  void insertLeaf(NodeIndex leafIndex);
  void removeLeaf(NodeIndex leafIndex);
  [[nodiscard]] NodeIndex findBestSibling(const Aabb& aabb) const;
  void rebalance(NodeIndex nodeIndex);
  [[nodiscard]] NodeIndex balance(NodeIndex nodeIndex);
  [[nodiscard]] static Aabb combine(const Aabb& a, const Aabb& b);
  [[nodiscard]] static double surfaceArea(const Aabb& aabb);

  bool visitPairsRecursive(
      NodeIndex nodeA,
      NodeIndex nodeB,
      const BroadPhasePairVisitor& visitor) const;

  void queryOverlappingImpl(
      const Aabb& aabb,
      std::vector<std::size_t>& results,
      std::size_t minId,
      bool higherIdsOnly) const;

  [[nodiscard]] bool validateStructure(NodeIndex nodeIndex) const;
};

} // namespace dart::collision::native
