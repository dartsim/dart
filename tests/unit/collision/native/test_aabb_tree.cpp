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

#include <dart/collision/native/broad_phase/AabbTree.hpp>
#include <dart/collision/native/broad_phase/BruteForce.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using namespace dart::collision::native;

namespace {

Aabb makeAabb(double min, double max)
{
  return Aabb(Eigen::Vector3d(min, min, min), Eigen::Vector3d(max, max, max));
}

Aabb makeAabb(const Eigen::Vector3d& center, const Eigen::Vector3d& halfExtents)
{
  return Aabb(center - halfExtents, center + halfExtents);
}

std::vector<BroadPhasePair> collectVisitedPairs(const BroadPhase& broadPhase)
{
  std::vector<BroadPhasePair> visited;
  const bool completed
      = broadPhase.visitPairs([&](std::size_t first, std::size_t second) {
          visited.emplace_back(first, second);
          return true;
        });

  EXPECT_TRUE(completed);
  return visited;
}

void expectExactBruteForceMatch(
    const AabbTreeBroadPhase& tree, const BruteForceBroadPhase& brute)
{
  EXPECT_TRUE(tree.validate());
  EXPECT_EQ(tree.size(), brute.size());

  const auto brutePairs = brute.queryPairs();
  EXPECT_EQ(tree.queryPairs(), brutePairs);
  EXPECT_EQ(collectVisitedPairs(tree), brutePairs);
}

} // namespace

TEST(AabbTreeBroadPhase, DefaultConstruction)
{
  AabbTreeBroadPhase bp;
  EXPECT_EQ(bp.size(), 0u);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddSingle)
{
  AabbTreeBroadPhase bp;

  Aabb aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
  bp.add(0, aabb);

  EXPECT_EQ(bp.size(), 1u);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, EmptyAndMissingOperationsAreNoOps)
{
  AabbTreeBroadPhase bp;

  bp.remove(123);
  bp.update(7, makeAabb(0, 1));
  EXPECT_EQ(bp.size(), 0u);
  EXPECT_TRUE(bp.validate());

  bp.add(7, makeAabb(2, 3));
  EXPECT_EQ(bp.size(), 1u);
  EXPECT_TRUE(bp.queryPairs().empty());

  std::vector<std::size_t> ids;
  std::vector<Aabb> aabbs;
  bp.build(ids, aabbs);
  EXPECT_EQ(bp.size(), 0u);
  EXPECT_TRUE(bp.queryOverlapping(makeAabb(0, 1)).empty());

  bool visited = false;
  EXPECT_TRUE(bp.visitPairs([&](std::size_t, std::size_t) {
    visited = true;
    return true;
  }));
  EXPECT_FALSE(visited);
}

TEST(AabbTreeBroadPhase, AddTwo_NoOverlap)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 1));
  bp.add(1, makeAabb(5, 6));

  EXPECT_EQ(bp.size(), 2u);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddTwo_Overlap)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 2));
  bp.add(1, makeAabb(1, 3));

  EXPECT_EQ(bp.size(), 2u);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].first, 0u);
  EXPECT_EQ(pairs[0].second, 1u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddThree_TwoOverlapping)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 2));
  bp.add(1, makeAabb(1, 3));
  bp.add(2, makeAabb(10, 11));

  EXPECT_EQ(bp.size(), 3u);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].first, 0u);
  EXPECT_EQ(pairs[0].second, 1u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddThree_AllOverlapping)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 3));
  bp.add(1, makeAabb(1, 4));
  bp.add(2, makeAabb(2, 5));

  EXPECT_EQ(bp.size(), 3u);
  EXPECT_EQ(bp.queryPairs().size(), 3u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Remove)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 2));
  bp.add(1, makeAabb(1, 3));

  EXPECT_EQ(bp.queryPairs().size(), 1u);

  bp.remove(0);

  EXPECT_EQ(bp.size(), 1u);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Update)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 2));
  bp.add(1, makeAabb(10, 11));

  EXPECT_TRUE(bp.queryPairs().empty());

  bp.update(1, makeAabb(1, 3));

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Clear)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 2));
  bp.add(1, makeAabb(1, 3));

  bp.clear();

  EXPECT_EQ(bp.size(), 0u);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, QueryOverlapping)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 2));
  bp.add(1, makeAabb(5, 7));
  bp.add(2, makeAabb(1, 3));

  auto overlapping = bp.queryOverlapping(
      Aabb(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5)));

  ASSERT_EQ(overlapping.size(), 2u);
  EXPECT_EQ(overlapping[0], 0u);
  EXPECT_EQ(overlapping[1], 2u);
}

TEST(AabbTreeBroadPhase, QueryOverlapping_Empty)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 1));

  EXPECT_TRUE(bp.queryOverlapping(makeAabb(10, 11)).empty());
}

TEST(AabbTreeBroadPhase, DeterministicPairOrder)
{
  for (int trial = 0; trial < 10; ++trial) {
    AabbTreeBroadPhase bp;

    bp.add(5, makeAabb(0, 5));
    bp.add(3, makeAabb(1, 6));
    bp.add(9, makeAabb(2, 7));
    bp.add(1, makeAabb(3, 8));

    const std::vector<BroadPhasePair> expected{
        {1, 3}, {1, 5}, {1, 9}, {3, 5}, {3, 9}, {5, 9}};

    EXPECT_EQ(bp.queryPairs(), expected);
    EXPECT_EQ(collectVisitedPairs(bp), expected);
    EXPECT_TRUE(bp.validate());
  }
}

TEST(AabbTreeBroadPhase, NonContiguousIds)
{
  AabbTreeBroadPhase bp;

  bp.add(100, makeAabb(0, 2));
  bp.add(200, makeAabb(1, 3));
  bp.add(50, makeAabb(10, 11));

  EXPECT_EQ(bp.size(), 3u);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].first, 100u);
  EXPECT_EQ(pairs[0].second, 200u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Touching)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 1, 1)));

  EXPECT_EQ(bp.queryPairs().size(), 1u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Polymorphism)
{
  std::unique_ptr<BroadPhase> bp = std::make_unique<AabbTreeBroadPhase>();

  bp->add(0, makeAabb(0, 2));
  bp->add(1, makeAabb(1, 3));

  EXPECT_EQ(bp->size(), 2u);
  EXPECT_EQ(bp->queryPairs().size(), 1u);
}

TEST(AabbTreeBroadPhase, DebugSnapshotExposesTreeTopology)
{
  AabbTreeBroadPhase bp;

  const std::unordered_map<std::size_t, Aabb> tightAabbs{
      {10u, makeAabb(0, 2)},
      {20u, makeAabb(1, 3)},
      {30u, makeAabb(8, 9)},
  };

  for (const auto& entry : tightAabbs) {
    bp.add(entry.first, entry.second);
  }

  BroadPhaseDebugSnapshot snapshot;
  bp.buildDebugSnapshot(snapshot);

  EXPECT_TRUE(snapshot.hasTreeTopology);
  EXPECT_EQ(snapshot.numObjects, tightAabbs.size());
  EXPECT_NE(snapshot.rootNode, BroadPhaseDebugNode::kInvalidIndex);
  EXPECT_EQ(snapshot.nodes.size(), 2u * tightAabbs.size() - 1u);
  ASSERT_EQ(snapshot.candidatePairs.size(), 1u);
  EXPECT_EQ(snapshot.candidatePairs[0], BroadPhasePair(10u, 20u));

  std::unordered_map<std::size_t, const BroadPhaseDebugNode*> nodesById;
  std::unordered_map<std::size_t, const BroadPhaseDebugNode*> leavesByObject;
  for (const auto& node : snapshot.nodes) {
    nodesById.emplace(node.nodeId, &node);
    if (node.isLeaf()) {
      leavesByObject.emplace(node.objectId, &node);
      EXPECT_TRUE(node.aabb.contains(node.tightAabb));
    } else {
      EXPECT_NE(node.left, BroadPhaseDebugNode::kInvalidIndex);
      EXPECT_NE(node.right, BroadPhaseDebugNode::kInvalidIndex);
    }
  }

  ASSERT_NE(nodesById.find(snapshot.rootNode), nodesById.end());
  EXPECT_EQ(
      nodesById.at(snapshot.rootNode)->parent,
      BroadPhaseDebugNode::kInvalidIndex);

  ASSERT_EQ(leavesByObject.size(), tightAabbs.size());
  for (const auto& entry : tightAabbs) {
    ASSERT_NE(leavesByObject.find(entry.first), leavesByObject.end());
    const BroadPhaseDebugNode& leaf = *leavesByObject.at(entry.first);
    EXPECT_TRUE(leaf.tightAabb.min.isApprox(entry.second.min));
    EXPECT_TRUE(leaf.tightAabb.max.isApprox(entry.second.max));
  }

  for (const auto& node : snapshot.nodes) {
    if (node.isLeaf()) {
      continue;
    }

    ASSERT_NE(nodesById.find(node.left), nodesById.end());
    ASSERT_NE(nodesById.find(node.right), nodesById.end());
    EXPECT_EQ(nodesById.at(node.left)->parent, node.nodeId);
    EXPECT_EQ(nodesById.at(node.right)->parent, node.nodeId);
  }
}

TEST(AabbTreeBroadPhase, FatAabbOptimization)
{
  AabbTreeBroadPhase bp(0.5);

  bp.add(0, makeAabb(0, 1));

  bp.update(
      0, Aabb(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(1.1, 1.1, 1.1)));
  bp.update(
      0, Aabb(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(1.2, 1.2, 1.2)));
  bp.update(
      0, Aabb(Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(1.3, 1.3, 1.3)));

  EXPECT_EQ(bp.size(), 1u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, RejectsInvalidFatAabbMargins)
{
  EXPECT_THROW(AabbTreeBroadPhase(-0.1), std::invalid_argument);
  // Parenthesized so the temporary is an expression, not a vexing-parse
  // declaration of ::infinity/::quiet_NaN, which AppleClang rejects.
  EXPECT_THROW(
      (AabbTreeBroadPhase(std::numeric_limits<double>::infinity())),
      std::invalid_argument);
  EXPECT_THROW(
      (AabbTreeBroadPhase(std::numeric_limits<double>::quiet_NaN())),
      std::invalid_argument);

  AabbTreeBroadPhase bp(0.25);
  EXPECT_THROW(bp.setFatAabbMargin(-0.1), std::invalid_argument);
  EXPECT_THROW(
      bp.setFatAabbMargin(std::numeric_limits<double>::infinity()),
      std::invalid_argument);
  EXPECT_THROW(
      bp.setFatAabbMargin(std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
  EXPECT_DOUBLE_EQ(bp.getFatAabbMargin(), 0.25);

  EXPECT_NO_THROW(bp.setFatAabbMargin(0.0));
  EXPECT_DOUBLE_EQ(bp.getFatAabbMargin(), 0.0);
}

TEST(AabbTreeBroadPhase, FatAabbDoesNotReportLoosePairs)
{
  AabbTreeBroadPhase bp(2.0);

  bp.add(0, makeAabb(0, 1));
  bp.add(1, makeAabb(2.5, 3.5));

  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(collectVisitedPairs(bp).empty());
  EXPECT_TRUE(bp.queryOverlapping(makeAabb(1.4, 1.6)).empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, TreeHeight)
{
  AabbTreeBroadPhase bp;

  EXPECT_EQ(bp.getHeight(), 0u);

  bp.add(0, makeAabb(0, 1));
  EXPECT_EQ(bp.getHeight(), 0u);

  bp.add(1, makeAabb(2, 3));
  EXPECT_GE(bp.getHeight(), 1u);

  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, ManyObjects)
{
  AabbTreeBroadPhase bp;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-100.0, 100.0);

  constexpr std::size_t numObjects = 100;

  for (std::size_t i = 0; i < numObjects; ++i) {
    const double x = dist(rng);
    const double y = dist(rng);
    const double z = dist(rng);
    bp.add(
        i,
        Aabb(Eigen::Vector3d(x, y, z), Eigen::Vector3d(x + 1, y + 1, z + 1)));
  }

  EXPECT_EQ(bp.size(), numObjects);
  EXPECT_TRUE(bp.validate());
  (void)bp.queryPairs();
}

TEST(AabbTreeBroadPhase, RemoveMultiple)
{
  AabbTreeBroadPhase bp;

  for (std::size_t i = 0; i < 10u; ++i) {
    bp.add(i, makeAabb(static_cast<double>(i) * 2.0, i * 2.0 + 1.0));
  }

  EXPECT_EQ(bp.size(), 10u);

  for (std::size_t i = 0; i < 5u; ++i) {
    bp.remove(i * 2u);
  }

  EXPECT_EQ(bp.size(), 5u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, ConsistentWithBruteForce)
{
  std::mt19937 rng(123);
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  AabbTreeBroadPhase tree;
  BruteForceBroadPhase brute;

  constexpr std::size_t numObjects = 50;

  for (std::size_t i = 0; i < numObjects; ++i) {
    const double x = dist(rng);
    const double y = dist(rng);
    const double z = dist(rng);
    const Aabb aabb(
        Eigen::Vector3d(x, y, z), Eigen::Vector3d(x + 2, y + 2, z + 2));

    tree.add(i, aabb);
    brute.add(i, aabb);
  }

  expectExactBruteForceMatch(tree, brute);
}

TEST(AabbTreeBroadPhase, QueryOverlappingConsistent)
{
  std::mt19937 rng(456);
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  AabbTreeBroadPhase tree;
  BruteForceBroadPhase brute;

  constexpr std::size_t numObjects = 30;

  for (std::size_t i = 0; i < numObjects; ++i) {
    const double x = dist(rng);
    const double y = dist(rng);
    const double z = dist(rng);
    const Aabb aabb(
        Eigen::Vector3d(x, y, z), Eigen::Vector3d(x + 1, y + 1, z + 1));

    tree.add(i, aabb);
    brute.add(i, aabb);
  }

  const Aabb query(Eigen::Vector3d(-5, -5, -5), Eigen::Vector3d(5, 5, 5));

  EXPECT_EQ(tree.queryOverlapping(query), brute.queryOverlapping(query));
}

TEST(AabbTreeBroadPhase, BulkBuild)
{
  const std::vector<std::size_t> ids = {10, 20, 30};
  const std::vector<Aabb> aabbs
      = {makeAabb(0, 2), makeAabb(1, 3), makeAabb(10, 11)};

  AabbTreeBroadPhase bp;
  bp.build(ids, aabbs);

  EXPECT_EQ(bp.size(), 3u);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].first, 10u);
  EXPECT_EQ(pairs[0].second, 20u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, BulkUpdateRange)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 1));
  bp.add(1, makeAabb(10, 11));
  bp.add(2, makeAabb(20, 21));

  EXPECT_TRUE(bp.queryPairs().empty());

  const std::vector<std::size_t> ids = {1, 2};
  const std::vector<Aabb> aabbs = {
      Aabb(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5)),
      Aabb(Eigen::Vector3d(0.8, 0.8, 0.8), Eigen::Vector3d(1.8, 1.8, 1.8)),
  };

  bp.updateRange(ids, aabbs);

  EXPECT_EQ(bp.queryPairs().size(), 3u);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, QueryPairsWithOutput)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 2));
  bp.add(1, makeAabb(1, 3));

  std::vector<BroadPhasePair> out;
  bp.queryPairs(out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].first, 0u);
  EXPECT_EQ(out[0].second, 1u);
}

TEST(AabbTreeBroadPhase, VisitPairsCanStopEarly)
{
  AabbTreeBroadPhase bp;

  bp.add(3, makeAabb(0, 3));
  bp.add(1, makeAabb(0, 3));
  bp.add(2, makeAabb(0, 3));

  std::vector<BroadPhasePair> visited;
  const bool completed = bp.visitPairs([&](std::size_t id1, std::size_t id2) {
    visited.emplace_back(id1, id2);
    return false;
  });

  EXPECT_FALSE(completed);
  ASSERT_EQ(visited.size(), 1u);
  EXPECT_EQ(visited[0], BroadPhasePair(1u, 2u));

  visited.clear();
  EXPECT_TRUE(bp.visitPairs([&](std::size_t id1, std::size_t id2) {
    visited.emplace_back(id1, id2);
    return true;
  }));
  EXPECT_EQ(visited, bp.queryPairs());
}

//==============================================================================
TEST(AabbTreeBroadPhase, VisitPairsStopsEarlyInDenseGroup)
{
  AabbTreeBroadPhase bp;
  constexpr std::size_t numObjects = 2048u;
  for (std::size_t id = 0; id < numObjects; ++id) {
    bp.add(id, makeAabb(0, 1));
  }

  std::size_t visited = 0u;
  const bool completed
      = bp.visitPairs([&](std::size_t first, std::size_t second) {
          ++visited;
          EXPECT_EQ(first, 0u);
          EXPECT_EQ(second, 1u);
          return false;
        });

  EXPECT_FALSE(completed);
  EXPECT_EQ(visited, 1u);
}

TEST(AabbTreeBroadPhase, QueryPairsFiltered)
{
  AabbTreeBroadPhase bp;

  bp.add(0, makeAabb(0, 3));
  bp.add(1, makeAabb(1, 4));
  bp.add(2, makeAabb(2, 5));

  auto allPairs = bp.queryPairs();
  EXPECT_EQ(allPairs.size(), 3u);

  std::vector<BroadPhasePair> filtered;
  bp.queryPairsFiltered(filtered, [](std::size_t id1, std::size_t id2) {
    return id1 == 0u || id2 == 0u;
  });

  EXPECT_EQ(filtered.size(), 2u);
  for (const auto& pair : filtered) {
    EXPECT_TRUE(pair.first == 0u || pair.second == 0u);
  }

  filtered.clear();
  bp.queryPairsFiltered(
      filtered, [](std::size_t, std::size_t) { return false; });
  EXPECT_TRUE(filtered.empty());
}

TEST(AabbTreeBroadPhase, RandomizedExactEquivalenceWithBruteForce)
{
  std::mt19937 rng(20260710);
  std::uniform_real_distribution<double> centerDist(-25.0, 25.0);
  std::uniform_real_distribution<double> extentDist(0.05, 1.75);
  std::uniform_int_distribution<int> objectDist(0, 199);

  constexpr std::size_t numObjects = 200;

  AabbTreeBroadPhase tree;
  BruteForceBroadPhase brute;
  std::vector<Aabb> aabbs(numObjects);
  std::vector<bool> active(numObjects, true);
  std::vector<std::size_t> ids(numObjects);
  std::iota(ids.begin(), ids.end(), 0u);
  std::shuffle(ids.begin(), ids.end(), rng);

  auto randomAabb = [&]() {
    const Eigen::Vector3d center(
        centerDist(rng), centerDist(rng), centerDist(rng));
    const Eigen::Vector3d halfExtents(
        extentDist(rng), extentDist(rng), extentDist(rng));
    return makeAabb(center, halfExtents);
  };

  for (const std::size_t id : ids) {
    aabbs[id] = randomAabb();
    tree.add(id, aabbs[id]);
    brute.add(id, aabbs[id]);
  }

  expectExactBruteForceMatch(tree, brute);

  for (int step = 0; step < 8; ++step) {
    for (int update = 0; update < 80; ++update) {
      const std::size_t id = static_cast<std::size_t>(objectDist(rng));
      aabbs[id] = randomAabb();
      tree.update(id, aabbs[id]);
      brute.update(id, aabbs[id]);
    }

    expectExactBruteForceMatch(tree, brute);

    for (int remove = 0; remove < 12; ++remove) {
      const std::size_t id = static_cast<std::size_t>(objectDist(rng));
      active[id] = false;
      tree.remove(id);
      brute.remove(id);
    }

    expectExactBruteForceMatch(tree, brute);

    std::vector<std::size_t> buildIds;
    std::vector<Aabb> buildAabbs;
    for (std::size_t id = 0; id < numObjects; ++id) {
      if (active[id]) {
        buildIds.push_back(id);
        buildAabbs.push_back(aabbs[id]);
      }
    }

    tree.build(buildIds, buildAabbs);
    brute.build(buildIds, buildAabbs);
    expectExactBruteForceMatch(tree, brute);

    std::vector<std::size_t> updateIds;
    std::vector<Aabb> updateAabbs;
    for (int update = 0; update < 60; ++update) {
      const std::size_t id = static_cast<std::size_t>(objectDist(rng));
      aabbs[id] = randomAabb();
      updateIds.push_back(id);
      updateAabbs.push_back(aabbs[id]);
    }

    tree.updateRange(updateIds, updateAabbs);
    brute.updateRange(updateIds, updateAabbs);
    expectExactBruteForceMatch(tree, brute);
  }
}

//==============================================================================
TEST(AabbTreeBroadPhase, VisitPairsAnyOrderMatchesPairSetAndStopsEarly)
{
  AabbTreeBroadPhase tree;
  std::mt19937 rng(20260710u);
  std::uniform_real_distribution<double> pos(-5.0, 5.0);
  std::uniform_real_distribution<double> extent(0.2, 1.5);
  for (std::size_t id = 0; id < 128; ++id) {
    const Eigen::Vector3d center(pos(rng), pos(rng), pos(rng));
    const Eigen::Vector3d half = Eigen::Vector3d::Constant(extent(rng)) * 0.5;
    tree.add(id, Aabb(center - half, center + half));
  }

  const auto ordered = tree.queryPairs();
  std::set<BroadPhasePair> orderedSet(ordered.begin(), ordered.end());

  // The any-order walk may emit duplicates but must cover exactly the same
  // pair set as the ordered walk.
  std::set<BroadPhasePair> streamedSet;
  const bool completed
      = tree.visitPairsAnyOrder([&](std::size_t first, std::size_t second) {
          streamedSet.emplace(first, second);
          return true;
        });
  EXPECT_TRUE(completed);
  EXPECT_EQ(streamedSet, orderedSet);

  // Early exit: the traversal must stop at the first rejection instead of
  // materializing/visiting the remaining candidates.
  if (!orderedSet.empty()) {
    std::size_t visited = 0;
    const bool aborted = tree.visitPairsAnyOrder([&](std::size_t, std::size_t) {
      ++visited;
      return false;
    });
    EXPECT_FALSE(aborted);
    EXPECT_EQ(visited, 1u);
  }
}
