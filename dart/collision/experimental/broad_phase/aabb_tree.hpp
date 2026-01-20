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

#include <limits>
#include <unordered_map>
#include <vector>

namespace dart::collision::experimental {

/// Dynamic AABB Tree using Surface Area Heuristic (SAH) for O(n log n)
/// broad-phase. Uses fat AABBs to reduce update frequency when objects move.
class DART_COLLISION_EXPERIMENTAL_API AabbTreeBroadPhase : public BroadPhase
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

  [[nodiscard]] double getFatAabbMargin() const
  {
    return fatAabbMargin_;
  }
  void setFatAabbMargin(double margin)
  {
    fatAabbMargin_ = margin;
  }
  [[nodiscard]] std::size_t getHeight() const;
  [[nodiscard]] bool validate() const;

private:
  struct Node
  {
    Aabb fatAabb;
    Aabb tightAabb;
    std::size_t parent = kNullNode;
    std::size_t left = kNullNode;
    std::size_t right = kNullNode;
    std::size_t objectId = kNullNode;
    std::size_t height = 0;

    [[nodiscard]] bool isLeaf() const
    {
      return left == kNullNode;
    }
  };

  std::vector<Node> nodes_;
  std::size_t root_ = kNullNode;
  std::size_t nodeCount_ = 0;
  std::size_t freeList_ = kNullNode;
  std::unordered_map<std::size_t, std::size_t> objectToNode_;
  double fatAabbMargin_;

  [[nodiscard]] std::size_t allocateNode();
  void freeNode(std::size_t nodeIndex);
  void insertLeaf(std::size_t leafIndex);
  void removeLeaf(std::size_t leafIndex);
  [[nodiscard]] std::size_t findBestSibling(const Aabb& aabb) const;
  void rebalance(std::size_t nodeIndex);
  [[nodiscard]] std::size_t balance(std::size_t nodeIndex);
  [[nodiscard]] static Aabb combine(const Aabb& a, const Aabb& b);
  [[nodiscard]] static double surfaceArea(const Aabb& aabb);

  void queryPairsRecursive(
      std::size_t nodeA,
      std::size_t nodeB,
      std::vector<BroadPhasePair>& pairs) const;

  void queryOverlappingRecursive(
      std::size_t nodeIndex,
      const Aabb& aabb,
      std::vector<std::size_t>& results) const;

  [[nodiscard]] std::size_t computeHeight(std::size_t nodeIndex) const;
  [[nodiscard]] bool validateStructure(std::size_t nodeIndex) const;
};

} // namespace dart::collision::experimental
