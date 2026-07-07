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

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <memory>
#include <vector>

using namespace dart::collision::native;

namespace {

Aabb makeAabb(double min, double max)
{
  return Aabb(Eigen::Vector3d(min, min, min), Eigen::Vector3d(max, max, max));
}

class RecordingBroadPhase : public BroadPhase
{
public:
  void clear() override
  {
    ++clearCount;
    addedIds.clear();
  }

  void add(std::size_t id, const Aabb&) override
  {
    addedIds.push_back(id);
  }

  void update(std::size_t id, const Aabb&) override
  {
    updatedIds.push_back(id);
  }

  void remove(std::size_t id) override
  {
    removedIds.push_back(id);
  }

  [[nodiscard]] std::vector<BroadPhasePair> queryPairs() const override
  {
    return pairs;
  }

  [[nodiscard]] std::vector<std::size_t> queryOverlapping(
      const Aabb&) const override
  {
    return overlappingIds;
  }

  [[nodiscard]] std::size_t size() const override
  {
    return numObjects;
  }

  std::vector<BroadPhasePair> pairs;
  std::vector<std::size_t> overlappingIds;
  std::vector<std::size_t> addedIds;
  std::vector<std::size_t> updatedIds;
  std::vector<std::size_t> removedIds;
  std::size_t numObjects = 0;
  int clearCount = 0;
};

} // namespace

TEST(BruteForceBroadPhase, DefaultConstruction)
{
  BruteForceBroadPhase bp;
  EXPECT_EQ(bp.size(), 0);
  EXPECT_TRUE(bp.queryPairs().empty());
}

TEST(BruteForceBroadPhase, AddSingle)
{
  BruteForceBroadPhase bp;

  Aabb aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
  bp.add(0, aabb);

  EXPECT_EQ(bp.size(), 1);
  EXPECT_TRUE(bp.queryPairs().empty());
}

TEST(BruteForceBroadPhase, AddTwo_NoOverlap)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));
  bp.add(1, Aabb(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(6, 6, 6)));

  EXPECT_EQ(bp.size(), 2);
  EXPECT_TRUE(bp.queryPairs().empty());
}

TEST(BruteForceBroadPhase, AddTwo_Overlap)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  EXPECT_EQ(bp.size(), 2);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_EQ(pairs[0].first, 0);
  EXPECT_EQ(pairs[0].second, 1);
}

TEST(BruteForceBroadPhase, AddThree_TwoOverlapping)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));
  bp.add(2, Aabb(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11)));

  EXPECT_EQ(bp.size(), 3);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_EQ(pairs[0].first, 0);
  EXPECT_EQ(pairs[0].second, 1);
}

TEST(BruteForceBroadPhase, AddThree_AllOverlapping)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(3, 3, 3)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(4, 4, 4)));
  bp.add(2, Aabb(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(5, 5, 5)));

  EXPECT_EQ(bp.size(), 3);

  auto pairs = bp.queryPairs();
  EXPECT_EQ(pairs.size(), 3);
}

TEST(BruteForceBroadPhase, Remove)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  EXPECT_EQ(bp.queryPairs().size(), 1);

  bp.remove(0);

  EXPECT_EQ(bp.size(), 1);
  EXPECT_TRUE(bp.queryPairs().empty());
}

TEST(BruteForceBroadPhase, RemoveMissingIdPreservesObjectsAndPairs)
{
  BruteForceBroadPhase bp;

  bp.add(0, makeAabb(0, 2));
  bp.add(1, makeAabb(1, 3));

  bp.remove(99);

  EXPECT_EQ(bp.size(), 2);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_EQ(pairs[0].first, 0);
  EXPECT_EQ(pairs[0].second, 1);
}

TEST(BruteForceBroadPhase, Update)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11)));

  EXPECT_TRUE(bp.queryPairs().empty());

  bp.update(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
}

TEST(BruteForceBroadPhase, UpdateMissingIdDoesNotInsert)
{
  BruteForceBroadPhase bp;

  bp.add(0, makeAabb(0, 1));

  bp.update(99, makeAabb(0, 1));

  EXPECT_EQ(bp.size(), 1);
  EXPECT_TRUE(bp.queryPairs().empty());
}

TEST(BruteForceBroadPhase, Clear)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  bp.clear();

  EXPECT_EQ(bp.size(), 0);
  EXPECT_TRUE(bp.queryPairs().empty());
}

TEST(BruteForceBroadPhase, QueryOverlapping)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(7, 7, 7)));
  bp.add(2, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  Aabb query(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5));

  auto overlapping = bp.queryOverlapping(query);

  EXPECT_EQ(overlapping.size(), 2);

  std::sort(overlapping.begin(), overlapping.end());
  EXPECT_EQ(overlapping[0], 0);
  EXPECT_EQ(overlapping[1], 2);
}

TEST(BruteForceBroadPhase, QueryOverlapping_Empty)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));

  Aabb query(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11));

  EXPECT_TRUE(bp.queryOverlapping(query).empty());
}

TEST(BruteForceBroadPhase, DeterministicPairOrder)
{
  for (int trial = 0; trial < 10; ++trial) {
    BruteForceBroadPhase bp;

    bp.add(5, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(5, 5, 5)));
    bp.add(3, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(6, 6, 6)));
    bp.add(9, Aabb(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(7, 7, 7)));
    bp.add(1, Aabb(Eigen::Vector3d(3, 3, 3), Eigen::Vector3d(8, 8, 8)));

    auto pairs = bp.queryPairs();
    ASSERT_EQ(pairs.size(), 6);

    EXPECT_EQ(pairs[0].first, 1);
    EXPECT_EQ(pairs[0].second, 3);
    EXPECT_EQ(pairs[1].first, 1);
    EXPECT_EQ(pairs[1].second, 5);
    EXPECT_EQ(pairs[2].first, 1);
    EXPECT_EQ(pairs[2].second, 9);
    EXPECT_EQ(pairs[3].first, 3);
    EXPECT_EQ(pairs[3].second, 5);
    EXPECT_EQ(pairs[4].first, 3);
    EXPECT_EQ(pairs[4].second, 9);
    EXPECT_EQ(pairs[5].first, 5);
    EXPECT_EQ(pairs[5].second, 9);
  }
}

TEST(BruteForceBroadPhase, NonContiguousIds)
{
  BruteForceBroadPhase bp;

  bp.add(100, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(200, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));
  bp.add(50, Aabb(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11)));

  EXPECT_EQ(bp.size(), 3);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_EQ(pairs[0].first, 100);
  EXPECT_EQ(pairs[0].second, 200);
}

TEST(BruteForceBroadPhase, Touching)
{
  BruteForceBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 1, 1)));

  auto pairs = bp.queryPairs();
  EXPECT_EQ(pairs.size(), 1);
}

TEST(BruteForceBroadPhase, VisitPairsStopsWhenVisitorReturnsFalse)
{
  BruteForceBroadPhase bp;

  bp.add(3, makeAabb(0, 3));
  bp.add(1, makeAabb(0, 3));
  bp.add(2, makeAabb(0, 3));

  std::vector<BroadPhasePair> visited;
  const bool completed
      = bp.visitPairs([&](std::size_t first, std::size_t second) {
          visited.emplace_back(first, second);
          return false;
        });

  EXPECT_FALSE(completed);
  ASSERT_EQ(visited.size(), 1);
  EXPECT_EQ(visited[0].first, 1);
  EXPECT_EQ(visited[0].second, 2);
}

TEST(BruteForceBroadPhase, VisitPairsReturnsTrueAfterAllPairs)
{
  BruteForceBroadPhase bp;

  bp.add(3, makeAabb(0, 3));
  bp.add(1, makeAabb(0, 3));
  bp.add(2, makeAabb(0, 3));

  std::vector<BroadPhasePair> visited;
  const bool completed
      = bp.visitPairs([&](std::size_t first, std::size_t second) {
          visited.emplace_back(first, second);
          return true;
        });

  EXPECT_TRUE(completed);
  EXPECT_EQ(visited.size(), 3);
}

TEST(BroadPhaseInterface, Polymorphism)
{
  std::unique_ptr<BroadPhase> bp = std::make_unique<BruteForceBroadPhase>();

  bp->add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp->add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  EXPECT_EQ(bp->size(), 2);
  EXPECT_EQ(bp->queryPairs().size(), 1);
}

TEST(BroadPhaseInterface, DebugNodeReportsLeafOnlyForValidObject)
{
  BroadPhaseDebugNode node;
  EXPECT_FALSE(node.isLeaf());

  node.objectId = 42;
  EXPECT_TRUE(node.isLeaf());
}

TEST(BroadPhaseInterface, DebugSnapshotClearResetsAllFields)
{
  BroadPhaseDebugSnapshot snapshot;
  snapshot.nodes.push_back(BroadPhaseDebugNode());
  snapshot.cells.push_back(BroadPhaseDebugCell());
  snapshot.endpoints.push_back(BroadPhaseDebugEndpoint());
  snapshot.candidatePairs.emplace_back(1, 2);
  snapshot.rootNode = 3;
  snapshot.numObjects = 4;
  snapshot.spatialHashCellSize = 5.0;
  snapshot.hasTreeTopology = true;
  snapshot.hasSpatialHashCells = true;
  snapshot.hasSweepEndpoints = true;

  snapshot.clear();

  EXPECT_TRUE(snapshot.nodes.empty());
  EXPECT_TRUE(snapshot.cells.empty());
  EXPECT_TRUE(snapshot.endpoints.empty());
  EXPECT_TRUE(snapshot.candidatePairs.empty());
  EXPECT_EQ(snapshot.rootNode, BroadPhaseDebugNode::kInvalidIndex);
  EXPECT_EQ(snapshot.numObjects, 0u);
  EXPECT_EQ(snapshot.spatialHashCellSize, 0.0);
  EXPECT_FALSE(snapshot.hasTreeTopology);
  EXPECT_FALSE(snapshot.hasSpatialHashCells);
  EXPECT_FALSE(snapshot.hasSweepEndpoints);
}

TEST(BroadPhaseInterface, DefaultDebugSnapshotUsesPairsAndSize)
{
  RecordingBroadPhase bp;
  bp.pairs = {{7, 9}, {10, 11}};
  bp.numObjects = 5;

  BroadPhaseDebugSnapshot snapshot;
  snapshot.nodes.push_back(BroadPhaseDebugNode());
  bp.buildDebugSnapshot(snapshot);

  EXPECT_TRUE(snapshot.nodes.empty());
  ASSERT_EQ(snapshot.candidatePairs.size(), 2);
  EXPECT_EQ(snapshot.candidatePairs[0].first, 7);
  EXPECT_EQ(snapshot.candidatePairs[0].second, 9);
  EXPECT_EQ(snapshot.candidatePairs[1].first, 10);
  EXPECT_EQ(snapshot.candidatePairs[1].second, 11);
  EXPECT_EQ(snapshot.numObjects, 5u);
}

TEST(BroadPhaseInterface, DefaultQueryPairsOverloadReplacesOutput)
{
  RecordingBroadPhase bp;
  bp.pairs = {{2, 4}, {6, 8}};

  std::vector<BroadPhasePair> pairs{{99, 100}};
  bp.BroadPhase::queryPairs(pairs);

  ASSERT_EQ(pairs.size(), 2);
  EXPECT_EQ(pairs[0].first, 2);
  EXPECT_EQ(pairs[0].second, 4);
  EXPECT_EQ(pairs[1].first, 6);
  EXPECT_EQ(pairs[1].second, 8);
}

TEST(BroadPhaseInterface, DefaultVisitPairsHonorsVisitorResult)
{
  RecordingBroadPhase bp;
  bp.pairs = {{2, 4}, {6, 8}};

  std::vector<BroadPhasePair> visited;
  const bool completed
      = bp.BroadPhase::visitPairs([&](std::size_t first, std::size_t second) {
          visited.emplace_back(first, second);
          return first != 2;
        });

  EXPECT_FALSE(completed);
  ASSERT_EQ(visited.size(), 1);
  EXPECT_EQ(visited[0].first, 2);
  EXPECT_EQ(visited[0].second, 4);

  visited.clear();
  EXPECT_TRUE(
      bp.BroadPhase::visitPairs([&](std::size_t first, std::size_t second) {
        visited.emplace_back(first, second);
        return true;
      }));
  EXPECT_EQ(visited.size(), 2);
}

TEST(BroadPhaseInterface, DefaultQueryPairsFilteredKeepsMatchingPairs)
{
  RecordingBroadPhase bp;
  bp.pairs = {{1, 2}, {3, 4}, {5, 6}};

  std::vector<BroadPhasePair> pairs{{99, 100}};
  bp.queryPairsFiltered(
      pairs, [](std::size_t first, std::size_t) { return first >= 3; });

  ASSERT_EQ(pairs.size(), 2);
  EXPECT_EQ(pairs[0].first, 3);
  EXPECT_EQ(pairs[0].second, 4);
  EXPECT_EQ(pairs[1].first, 5);
  EXPECT_EQ(pairs[1].second, 6);
}

TEST(BroadPhaseInterface, DefaultBuildClearsAndAddsMinInputCount)
{
  RecordingBroadPhase bp;
  bp.addedIds.push_back(99);

  const std::array<std::size_t, 3> ids{{10, 20, 30}};
  const std::array<Aabb, 2> aabbs{{makeAabb(0, 1), makeAabb(1, 2)}};

  bp.build(ids, aabbs);

  EXPECT_EQ(bp.clearCount, 1);
  ASSERT_EQ(bp.addedIds.size(), 2);
  EXPECT_EQ(bp.addedIds[0], 10);
  EXPECT_EQ(bp.addedIds[1], 20);
}

TEST(BroadPhaseInterface, DefaultUpdateRangeUsesMinInputCount)
{
  RecordingBroadPhase bp;

  const std::array<std::size_t, 3> ids{{10, 20, 30}};
  const std::array<Aabb, 2> aabbs{{makeAabb(0, 1), makeAabb(1, 2)}};

  bp.updateRange(ids, aabbs);

  ASSERT_EQ(bp.updatedIds.size(), 2);
  EXPECT_EQ(bp.updatedIds[0], 10);
  EXPECT_EQ(bp.updatedIds[1], 20);
}
