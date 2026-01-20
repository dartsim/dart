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

#include <dart/collision/experimental/broad_phase/aabb_tree.hpp>
#include <dart/collision/experimental/broad_phase/brute_force.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <random>

using namespace dart::collision::experimental;

TEST(AabbTreeBroadPhase, DefaultConstruction)
{
  AabbTreeBroadPhase bp;
  EXPECT_EQ(bp.size(), 0);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddSingle)
{
  AabbTreeBroadPhase bp;

  Aabb aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
  bp.add(0, aabb);

  EXPECT_EQ(bp.size(), 1);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddTwo_NoOverlap)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));
  bp.add(1, Aabb(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(6, 6, 6)));

  EXPECT_EQ(bp.size(), 2);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddTwo_Overlap)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  EXPECT_EQ(bp.size(), 2);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_EQ(pairs[0].first, 0);
  EXPECT_EQ(pairs[0].second, 1);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddThree_TwoOverlapping)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));
  bp.add(2, Aabb(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11)));

  EXPECT_EQ(bp.size(), 3);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_EQ(pairs[0].first, 0);
  EXPECT_EQ(pairs[0].second, 1);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, AddThree_AllOverlapping)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(3, 3, 3)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(4, 4, 4)));
  bp.add(2, Aabb(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(5, 5, 5)));

  EXPECT_EQ(bp.size(), 3);

  auto pairs = bp.queryPairs();
  EXPECT_EQ(pairs.size(), 3);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Remove)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  EXPECT_EQ(bp.queryPairs().size(), 1);

  bp.remove(0);

  EXPECT_EQ(bp.size(), 1);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Update)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11)));

  EXPECT_TRUE(bp.queryPairs().empty());

  bp.update(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Clear)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  bp.clear();

  EXPECT_EQ(bp.size(), 0);
  EXPECT_TRUE(bp.queryPairs().empty());
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, QueryOverlapping)
{
  AabbTreeBroadPhase bp;

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

TEST(AabbTreeBroadPhase, QueryOverlapping_Empty)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));

  Aabb query(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11));

  EXPECT_TRUE(bp.queryOverlapping(query).empty());
}

TEST(AabbTreeBroadPhase, DeterministicPairOrder)
{
  for (int trial = 0; trial < 10; ++trial) {
    AabbTreeBroadPhase bp;

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

TEST(AabbTreeBroadPhase, NonContiguousIds)
{
  AabbTreeBroadPhase bp;

  bp.add(100, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(200, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));
  bp.add(50, Aabb(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11)));

  EXPECT_EQ(bp.size(), 3);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_EQ(pairs[0].first, 100);
  EXPECT_EQ(pairs[0].second, 200);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Touching)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 1, 1)));

  auto pairs = bp.queryPairs();
  EXPECT_EQ(pairs.size(), 1);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, Polymorphism)
{
  std::unique_ptr<BroadPhase> bp = std::make_unique<AabbTreeBroadPhase>();

  bp->add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp->add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  EXPECT_EQ(bp->size(), 2);
  EXPECT_EQ(bp->queryPairs().size(), 1);
}

TEST(AabbTreeBroadPhase, FatAabbOptimization)
{
  AabbTreeBroadPhase bp(0.5);

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));

  bp.update(
      0, Aabb(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(1.1, 1.1, 1.1)));
  bp.update(
      0, Aabb(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(1.2, 1.2, 1.2)));
  bp.update(
      0, Aabb(Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(1.3, 1.3, 1.3)));

  EXPECT_EQ(bp.size(), 1);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, TreeHeight)
{
  AabbTreeBroadPhase bp;

  EXPECT_EQ(bp.getHeight(), 0);

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));
  EXPECT_EQ(bp.getHeight(), 0);

  bp.add(1, Aabb(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(3, 3, 3)));
  EXPECT_GE(bp.getHeight(), 1);

  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, ManyObjects)
{
  AabbTreeBroadPhase bp;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-100.0, 100.0);

  constexpr int numObjects = 100;

  for (int i = 0; i < numObjects; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double z = dist(rng);
    bp.add(
        i,
        Aabb(Eigen::Vector3d(x, y, z), Eigen::Vector3d(x + 1, y + 1, z + 1)));
  }

  EXPECT_EQ(bp.size(), numObjects);
  EXPECT_TRUE(bp.validate());

  auto pairs = bp.queryPairs();
  (void)pairs;
}

TEST(AabbTreeBroadPhase, RemoveMultiple)
{
  AabbTreeBroadPhase bp;

  for (int i = 0; i < 10; ++i) {
    bp.add(
        i,
        Aabb(
            Eigen::Vector3d(i * 2.0, 0, 0),
            Eigen::Vector3d(i * 2.0 + 1, 1, 1)));
  }

  EXPECT_EQ(bp.size(), 10);

  for (int i = 0; i < 5; ++i) {
    bp.remove(i * 2);
  }

  EXPECT_EQ(bp.size(), 5);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, ConsistentWithBruteForce)
{
  std::mt19937 rng(123);
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  AabbTreeBroadPhase tree;
  BruteForceBroadPhase brute;

  constexpr int numObjects = 50;

  for (int i = 0; i < numObjects; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double z = dist(rng);
    Aabb aabb(Eigen::Vector3d(x, y, z), Eigen::Vector3d(x + 2, y + 2, z + 2));

    tree.add(i, aabb);
    brute.add(i, aabb);
  }

  auto treePairs = tree.queryPairs();
  auto brutePairs = brute.queryPairs();

  std::sort(treePairs.begin(), treePairs.end());
  std::sort(brutePairs.begin(), brutePairs.end());

  EXPECT_EQ(treePairs.size(), brutePairs.size());
  EXPECT_EQ(treePairs, brutePairs);
}

TEST(AabbTreeBroadPhase, QueryOverlappingConsistent)
{
  std::mt19937 rng(456);
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  AabbTreeBroadPhase tree;
  BruteForceBroadPhase brute;

  constexpr int numObjects = 30;

  for (int i = 0; i < numObjects; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double z = dist(rng);
    Aabb aabb(Eigen::Vector3d(x, y, z), Eigen::Vector3d(x + 1, y + 1, z + 1));

    tree.add(i, aabb);
    brute.add(i, aabb);
  }

  Aabb query(Eigen::Vector3d(-5, -5, -5), Eigen::Vector3d(5, 5, 5));

  auto treeResults = tree.queryOverlapping(query);
  auto bruteResults = brute.queryOverlapping(query);

  std::sort(treeResults.begin(), treeResults.end());
  std::sort(bruteResults.begin(), bruteResults.end());

  EXPECT_EQ(treeResults, bruteResults);
}

TEST(AabbTreeBroadPhase, BulkBuild)
{
  std::vector<std::size_t> ids = {10, 20, 30};
  std::vector<Aabb> aabbs = {
      Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)),
      Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)),
      Aabb(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11)),
  };

  AabbTreeBroadPhase bp;
  bp.build(ids, aabbs);

  EXPECT_EQ(bp.size(), 3);

  auto pairs = bp.queryPairs();
  ASSERT_EQ(pairs.size(), 1);
  EXPECT_EQ(pairs[0].first, 10);
  EXPECT_EQ(pairs[0].second, 20);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, BulkUpdateRange)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));
  bp.add(1, Aabb(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11)));
  bp.add(2, Aabb(Eigen::Vector3d(20, 20, 20), Eigen::Vector3d(21, 21, 21)));

  EXPECT_TRUE(bp.queryPairs().empty());

  std::vector<std::size_t> ids = {1, 2};
  std::vector<Aabb> aabbs = {
      Aabb(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5)),
      Aabb(Eigen::Vector3d(0.8, 0.8, 0.8), Eigen::Vector3d(1.8, 1.8, 1.8)),
  };

  bp.updateRange(ids, aabbs);

  auto pairs = bp.queryPairs();
  EXPECT_EQ(pairs.size(), 3);
  EXPECT_TRUE(bp.validate());
}

TEST(AabbTreeBroadPhase, QueryPairsWithOutput)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)));

  std::vector<BroadPhasePair> out;
  bp.queryPairs(out);

  ASSERT_EQ(out.size(), 1);
  EXPECT_EQ(out[0].first, 0);
  EXPECT_EQ(out[0].second, 1);
}

TEST(AabbTreeBroadPhase, QueryPairsFiltered)
{
  AabbTreeBroadPhase bp;

  bp.add(0, Aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(3, 3, 3)));
  bp.add(1, Aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(4, 4, 4)));
  bp.add(2, Aabb(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(5, 5, 5)));

  auto allPairs = bp.queryPairs();
  EXPECT_EQ(allPairs.size(), 3);

  std::vector<BroadPhasePair> filtered;
  bp.queryPairsFiltered(filtered, [](std::size_t id1, std::size_t id2) {
    return id1 == 0 || id2 == 0;
  });

  EXPECT_EQ(filtered.size(), 2);
  for (const auto& pair : filtered) {
    EXPECT_TRUE(pair.first == 0 || pair.second == 0);
  }

  filtered.clear();
  bp.queryPairsFiltered(filtered, [](std::size_t, std::size_t) {
    return false;
  });
  EXPECT_TRUE(filtered.empty());
}
