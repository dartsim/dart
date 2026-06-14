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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/detail/deformable_contact/candidate_set.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/memory_manager.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace common = dart::common;
namespace sx = dart::simulation;
namespace dc = dart::simulation::detail::deformable_contact;

namespace {

class CountingMemoryAllocator final : public common::MemoryAllocator
{
public:
  std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  void* allocate(std::size_t bytes) noexcept override
  {
    ++allocations;
    return common::MemoryAllocator::GetDefault().allocate(bytes);
  }

  void* allocate(std::size_t bytes, std::size_t alignment) noexcept override
  {
    ++allocations;
    return common::MemoryAllocator::GetDefault().allocate(bytes, alignment);
  }

  void deallocate(void* pointer, std::size_t bytes) override
  {
    common::MemoryAllocator::GetDefault().deallocate(pointer, bytes);
  }

  void deallocate(
      void* pointer, std::size_t bytes, std::size_t alignment) override
  {
    common::MemoryAllocator::GetDefault().deallocate(pointer, bytes, alignment);
  }

  std::size_t allocations = 0u;
};

//==============================================================================
dc::SurfaceEdge edge(std::size_t a, std::size_t b)
{
  if (b < a) {
    std::swap(a, b);
  }
  return dc::SurfaceEdge{a, b};
}

//==============================================================================
std::size_t findEdge(
    std::span<const dc::SurfaceEdge> edges, std::size_t a, std::size_t b)
{
  const auto target = edge(a, b);
  const auto it = std::find(edges.begin(), edges.end(), target);
  EXPECT_NE(it, edges.end());
  return static_cast<std::size_t>(std::distance(edges.begin(), it));
}

//==============================================================================
bool containsPointTriangle(
    const dc::ContactCandidateSet& candidates,
    const std::size_t point,
    const std::size_t triangle)
{
  return std::any_of(
      candidates.pointTriangleCandidates.begin(),
      candidates.pointTriangleCandidates.end(),
      [&](const dc::PointTriangleCandidate& candidate) {
        return candidate.point == point && candidate.triangle == triangle;
      });
}

//==============================================================================
bool containsEdgeEdge(
    const dc::ContactCandidateSet& candidates,
    std::size_t edgeA,
    std::size_t edgeB)
{
  if (edgeB < edgeA) {
    std::swap(edgeA, edgeB);
  }

  return std::any_of(
      candidates.edgeEdgeCandidates.begin(),
      candidates.edgeEdgeCandidates.end(),
      [&](const dc::EdgeEdgeCandidate& candidate) {
        return candidate.edgeA == edgeA && candidate.edgeB == edgeB;
      });
}

//==============================================================================
dc::detail::SweepItem sweepItem(
    std::size_t id, const Eigen::Vector3d& min, const Eigen::Vector3d& max)
{
  return dc::detail::SweepItem{id, dc::detail::CandidateAabb{min, max}};
}

//==============================================================================
std::vector<std::pair<std::size_t, std::size_t>> visitSweepPairsOptimized(
    std::vector<dc::detail::SweepItem> lhs,
    std::vector<dc::detail::SweepItem> rhs)
{
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  dc::detail::visitSweepPairs(
      lhs, rhs, [&](std::size_t lhsId, std::size_t rhsId) {
        pairs.emplace_back(lhsId, rhsId);
      });
  return pairs;
}

//==============================================================================
std::vector<std::pair<std::size_t, std::size_t>> visitSweepPairsNaiveReference(
    std::vector<dc::detail::SweepItem> lhs,
    std::vector<dc::detail::SweepItem> rhs)
{
  dc::detail::sortSweepItems(lhs);
  dc::detail::sortSweepItems(rhs);

  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  for (const auto& lhsItem : lhs) {
    for (const auto& rhsItem : rhs) {
      if (rhsItem.aabb.min.x() > lhsItem.aabb.max.x()) {
        break;
      }
      if (rhsItem.aabb.max.x() < lhsItem.aabb.min.x()) {
        continue;
      }
      if (lhsItem.aabb.overlaps(rhsItem.aabb)) {
        pairs.emplace_back(lhsItem.id, rhsItem.id);
      }
    }
  }
  return pairs;
}

//==============================================================================
void expectCandidatesEqual(
    const dc::ContactCandidateSet& actual,
    const dc::ContactCandidateSet& expected)
{
  ASSERT_EQ(actual.surfaceEdges, expected.surfaceEdges);

  ASSERT_EQ(
      actual.pointTriangleCandidates.size(),
      expected.pointTriangleCandidates.size());
  for (std::size_t i = 0; i < actual.pointTriangleCandidates.size(); ++i) {
    EXPECT_EQ(
        actual.pointTriangleCandidates[i].point,
        expected.pointTriangleCandidates[i].point);
    EXPECT_EQ(
        actual.pointTriangleCandidates[i].triangle,
        expected.pointTriangleCandidates[i].triangle);
    EXPECT_NEAR(
        actual.pointTriangleCandidates[i].squaredDistance,
        expected.pointTriangleCandidates[i].squaredDistance,
        1e-12);
  }

  ASSERT_EQ(
      actual.edgeEdgeCandidates.size(), expected.edgeEdgeCandidates.size());
  for (std::size_t i = 0; i < actual.edgeEdgeCandidates.size(); ++i) {
    EXPECT_EQ(
        actual.edgeEdgeCandidates[i].edgeA,
        expected.edgeEdgeCandidates[i].edgeA);
    EXPECT_EQ(
        actual.edgeEdgeCandidates[i].edgeB,
        expected.edgeEdgeCandidates[i].edgeB);
    EXPECT_NEAR(
        actual.edgeEdgeCandidates[i].squaredDistance,
        expected.edgeEdgeCandidates[i].squaredDistance,
        1e-12);
  }
}

//==============================================================================
void expectStatsEqual(
    const dc::ContactCandidateStats& actual,
    const dc::ContactCandidateStats& expected)
{
  EXPECT_EQ(actual.pointCount, expected.pointCount);
  EXPECT_EQ(actual.triangleCount, expected.triangleCount);
  EXPECT_EQ(actual.edgeCount, expected.edgeCount);
  EXPECT_EQ(actual.broadPhaseOverlapCount, expected.broadPhaseOverlapCount);
  EXPECT_EQ(actual.exactDistanceCheckCount, expected.exactDistanceCheckCount);
  EXPECT_EQ(
      actual.incidentPointTriangleRejectCount,
      expected.incidentPointTriangleRejectCount);
  EXPECT_EQ(
      actual.adjacentEdgeEdgeRejectCount, expected.adjacentEdgeEdgeRejectCount);
  EXPECT_EQ(
      actual.pointTriangleCandidateCount, expected.pointTriangleCandidateCount);
  EXPECT_EQ(actual.edgeEdgeCandidateCount, expected.edgeEdgeCandidateCount);
}

//==============================================================================
void expectCandidateSetsEqualIncludingStats(
    const dc::ContactCandidateSet& actual,
    const dc::ContactCandidateSet& expected)
{
  expectCandidatesEqual(actual, expected);
  expectStatsEqual(actual.stats, expected.stats);
}

//==============================================================================
void expectSweepMatchesBruteForce(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<sx::DeformableSurfaceTriangle>& triangles,
    const double activationDistance)
{
  dc::ContactCandidateOptions options;
  options.activationDistance = activationDistance;

  const auto sweep
      = dc::buildContactCandidatesSweep(positions, triangles, options);
  const auto brute
      = dc::buildContactCandidatesBruteForce(positions, triangles, options);

  expectCandidatesEqual(sweep, brute);
  EXPECT_GT(sweep.stats.broadPhaseOverlapCount, 0u);
  EXPECT_TRUE(
      std::is_sorted(
          sweep.pointTriangleCandidates.begin(),
          sweep.pointTriangleCandidates.end()));
  EXPECT_TRUE(
      std::is_sorted(
          sweep.edgeEdgeCandidates.begin(), sweep.edgeEdgeCandidates.end()));
}

//==============================================================================
void expectMotionAwareSweepMatchesBruteForce(
    const std::vector<Eigen::Vector3d>& start,
    const std::vector<Eigen::Vector3d>& end,
    const std::vector<sx::DeformableSurfaceTriangle>& triangles,
    const double activationDistance)
{
  dc::ContactCandidateOptions options;
  options.activationDistance = activationDistance;

  const auto sweep = dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options);
  const auto brute = dc::buildMotionAwareContactCandidatesBruteForce(
      start, end, triangles, options);

  expectCandidatesEqual(sweep, brute);
  expectStatsEqual(sweep.stats, brute.stats);
  EXPECT_TRUE(
      std::is_sorted(
          sweep.pointTriangleCandidates.begin(),
          sweep.pointTriangleCandidates.end()));
  EXPECT_TRUE(
      std::is_sorted(
          sweep.edgeEdgeCandidates.begin(), sweep.edgeEdgeCandidates.end()));
}

} // namespace

//==============================================================================
TEST(IpcContactCandidateSet, VisitSweepPairsMatchesNaiveReference)
{
  std::vector<dc::detail::SweepItem> lhs{
      sweepItem(
          2, Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(3.0, 1.0, 1.0)),
      sweepItem(
          0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0)),
      sweepItem(
          1, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(2.0, 1.0, 1.0)),
  };
  std::vector<dc::detail::SweepItem> rhs{
      sweepItem(
          20,
          Eigen::Vector3d(-5.0, 5.0, 0.0),
          Eigen::Vector3d(100.0, 6.0, 1.0)),
      sweepItem(
          21, Eigen::Vector3d(-4.0, 0.0, 0.0), Eigen::Vector3d(-3.5, 1.0, 1.0)),
      sweepItem(
          10, Eigen::Vector3d(-3.0, 0.0, 0.0), Eigen::Vector3d(-2.0, 1.0, 1.0)),
      sweepItem(
          12, Eigen::Vector3d(2.5, 0.0, 0.0), Eigen::Vector3d(2.75, 1.0, 1.0)),
      sweepItem(
          11, Eigen::Vector3d(0.5, 2.0, 0.0), Eigen::Vector3d(1.5, 3.0, 1.0)),
      sweepItem(
          14, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(1.25, 1.0, 1.0)),
      sweepItem(
          13, Eigen::Vector3d(4.0, 0.0, 0.0), Eigen::Vector3d(5.0, 1.0, 1.0)),
  };

  const auto pairs = visitSweepPairsOptimized(lhs, rhs);
  const auto referencePairs = visitSweepPairsNaiveReference(lhs, rhs);

  EXPECT_EQ(pairs, referencePairs);
  const std::vector<std::pair<std::size_t, std::size_t>> expectedPairs{
      {0, 14},
      {1, 14},
      {2, 12},
  };
  EXPECT_EQ(pairs, expectedPairs);
}

//==============================================================================
// Stresses the active-set ("sweep and prune") live-list maintenance: a
// never-expiring early interval that overlaps nothing must stay in the list,
// while a head item and several mid-list items expire at staggered sweep
// positions across successive lhs. The optimized traversal must still emit
// exactly the naive reference's pairs in the same order.
TEST(IpcContactCandidateSet, VisitSweepPairsActiveSetUnlinksStaggeredExpiries)
{
  std::vector<dc::detail::SweepItem> lhs{
      sweepItem(
          0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.5, 1.0, 1.0)),
      sweepItem(
          1, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(1.5, 1.0, 1.0)),
      sweepItem(
          2, Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(2.5, 1.0, 1.0)),
      sweepItem(
          3, Eigen::Vector3d(3.0, 0.0, 0.0), Eigen::Vector3d(3.5, 1.0, 1.0)),
  };
  std::vector<dc::detail::SweepItem> rhs{
      // Long-lived but y-disjoint: never expires, never overlaps -> must remain
      // in the live list and be x-checked for every lhs without being unlinked.
      sweepItem(
          100,
          Eigen::Vector3d(-1.0, 10.0, 0.0),
          Eigen::Vector3d(100.0, 11.0, 1.0)),
      // Expires before any lhs (head unlink at the first lhs).
      sweepItem(
          1, Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d(-0.2, 1.0, 1.0)),
      sweepItem(
          2, Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d(0.4, 1.0, 1.0)),
      // Falls between lhs 0 and lhs 1: never overlaps, unlinked mid-list at 1.
      sweepItem(
          3, Eigen::Vector3d(0.6, 0.0, 0.0), Eigen::Vector3d(0.9, 1.0, 1.0)),
      sweepItem(
          4, Eigen::Vector3d(1.1, 0.0, 0.0), Eigen::Vector3d(1.4, 1.0, 1.0)),
      sweepItem(
          5, Eigen::Vector3d(2.1, 0.0, 0.0), Eigen::Vector3d(2.4, 1.0, 1.0)),
      sweepItem(
          6, Eigen::Vector3d(3.1, 0.0, 0.0), Eigen::Vector3d(3.4, 1.0, 1.0)),
  };

  const auto pairs = visitSweepPairsOptimized(lhs, rhs);
  const auto referencePairs = visitSweepPairsNaiveReference(lhs, rhs);

  EXPECT_EQ(pairs, referencePairs);
  const std::vector<std::pair<std::size_t, std::size_t>> expectedPairs{
      {0, 2},
      {1, 4},
      {2, 5},
      {3, 6},
  };
  EXPECT_EQ(pairs, expectedPairs);
}

//==============================================================================
TEST(IpcContactCandidateSet, DerivesUniqueSurfaceEdgesDeterministically)
{
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {2, 1, 3},
      {2, 1, 0},
  };

  const auto edges = dc::makeUniqueSurfaceEdges(triangles);
  const std::vector<dc::SurfaceEdge> expected = {
      edge(0, 1),
      edge(0, 2),
      edge(1, 2),
      edge(1, 3),
      edge(2, 3),
  };
  EXPECT_EQ(edges, expected);
}

//==============================================================================
TEST(IpcContactCandidateSet, MatchesBruteForceOnSingleTriangleAndClothPatch)
{
  {
    const std::vector<Eigen::Vector3d> positions = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.25, 0.25, 0.2},
    };
    const std::vector<sx::DeformableSurfaceTriangle> triangles = {{0, 1, 2}};
    expectSweepMatchesBruteForce(positions, triangles, 0.25);
  }

  {
    const std::vector<Eigen::Vector3d> positions = {
        {-1.0, -1.0, 0.0},
        {0.0, -1.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {-0.55, -0.55, 0.05},
    };
    const std::vector<sx::DeformableSurfaceTriangle> triangles = {
        {0, 1, 2},
        {1, 3, 2},
    };
    expectSweepMatchesBruteForce(positions, triangles, 0.2);
  }
}

//==============================================================================
TEST(IpcContactCandidateSet, MatchesBruteForceOnFoldedAndTetraBoundaryMeshes)
{
  {
    const std::vector<Eigen::Vector3d> positions = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.1, 0.1, 0.04},
        {1.1, 0.1, 0.04},
        {0.1, 1.1, 0.04},
    };
    const std::vector<sx::DeformableSurfaceTriangle> triangles = {
        {0, 1, 2},
        {3, 4, 5},
    };
    expectSweepMatchesBruteForce(positions, triangles, 0.06);
  }

  {
    const std::vector<Eigen::Vector3d> positions = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
    };
    const std::vector<sx::DeformableSurfaceTriangle> triangles = {
        {0, 2, 1},
        {0, 1, 3},
        {1, 2, 3},
        {2, 0, 3},
    };
    expectSweepMatchesBruteForce(positions, triangles, 0.6);
  }
}

//==============================================================================
TEST(IpcContactCandidateSet, DefaultFiltersExcludeIncidentAndAdjacentPairs)
{
  const std::vector<Eigen::Vector3d> positions = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.25, 0.25, 0.0},
  };
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {{0, 1, 2}};

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.0;
  auto candidates
      = dc::buildContactCandidatesSweep(positions, triangles, options);
  ASSERT_EQ(candidates.pointTriangleCandidates.size(), 1u);
  EXPECT_TRUE(containsPointTriangle(candidates, 3, 0));
  EXPECT_TRUE(candidates.edgeEdgeCandidates.empty());
  EXPECT_EQ(candidates.stats.incidentPointTriangleRejectCount, 3u);
  EXPECT_EQ(candidates.stats.adjacentEdgeEdgeRejectCount, 3u);

  options.excludeIncidentPointTriangles = false;
  options.excludeAdjacentEdges = false;
  candidates = dc::buildContactCandidatesSweep(positions, triangles, options);
  EXPECT_EQ(candidates.pointTriangleCandidates.size(), 4u);
  EXPECT_EQ(candidates.edgeEdgeCandidates.size(), 3u);
}

//==============================================================================
TEST(IpcContactCandidateSet, IncludesExactActivationDistanceBoundary)
{
  const std::vector<Eigen::Vector3d> positions = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.2, 1.0, 0.0},
      {-2.0, -2.0, 0.0},
      {0.0, 0.25, 0.0},
      {1.0, 0.25, 0.0},
      {0.2, -1.0, 0.0},
      {2.0, 2.0, 0.0},
  };
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {4, 5, 6},
  };

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.25;
  const auto candidates
      = dc::buildContactCandidatesSweep(positions, triangles, options);
  const auto brute
      = dc::buildContactCandidatesBruteForce(positions, triangles, options);
  expectCandidatesEqual(candidates, brute);

  const std::size_t edgeA = findEdge(candidates.surfaceEdges, 0, 1);
  const std::size_t edgeB = findEdge(candidates.surfaceEdges, 4, 5);
  EXPECT_TRUE(containsEdgeEdge(candidates, edgeA, edgeB));
}

//==============================================================================
TEST(IpcContactCandidateSet, HandlesEmptyInputsAndDeterministicOutput)
{
  dc::ContactCandidateOptions options;
  options.activationDistance = 0.1;

  const auto empty = dc::buildContactCandidatesSweep(
      std::vector<Eigen::Vector3d>{},
      std::vector<sx::DeformableSurfaceTriangle>{},
      options);
  EXPECT_TRUE(empty.surfaceEdges.empty());
  EXPECT_TRUE(empty.pointTriangleCandidates.empty());
  EXPECT_TRUE(empty.edgeEdgeCandidates.empty());

  const std::vector<Eigen::Vector3d> positions = {
      {-0.5, 0.0, 0.0},
      {0.5, 0.0, 0.0},
      {0.0, 0.5, 0.0},
      {-0.5, 0.0, 0.03},
      {0.5, 0.0, 0.03},
      {0.0, 0.5, 0.03},
  };
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {3, 5, 4},
  };

  const auto first
      = dc::buildContactCandidatesSweep(positions, triangles, options);
  const auto second
      = dc::buildContactCandidatesSweep(positions, triangles, options);
  expectCandidatesEqual(first, second);
  EXPECT_TRUE(
      std::is_sorted(
          first.pointTriangleCandidates.begin(),
          first.pointTriangleCandidates.end()));
  EXPECT_TRUE(
      std::is_sorted(
          first.edgeEdgeCandidates.begin(), first.edgeEdgeCandidates.end()));
}

//==============================================================================
TEST(IpcContactCandidateSet, MotionAwareStaticPoseMatchesAabbOnlySweep)
{
  const std::vector<Eigen::Vector3d> positions = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.2, 0.2, 0.4},
      {0.8, 0.2, 0.4},
      {0.2, 0.8, 0.4},
  };
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {3, 5, 4},
  };

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.5;
  options.exactDistanceFilter = false;

  const auto staticSweep
      = dc::buildContactCandidatesSweep(positions, triangles, options);
  const auto motionSweep = dc::buildMotionAwareContactCandidatesSweep(
      positions, positions, triangles, options);
  expectCandidatesEqual(motionSweep, staticSweep);
}

//==============================================================================
TEST(IpcContactCandidateSet, MotionAwareSweepMatchesBruteForceOnSweptPairs)
{
  const std::vector<Eigen::Vector3d> start = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
      {-1.0, 0.0, 0.8},
      {1.0, 0.0, 0.8},
      {0.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {-0.5, -0.5, 0.8},
  };
  auto end = start;
  end[3] = Eigen::Vector3d(0.0, 0.0, -1.0);
  end[4].z() = -0.8;
  end[5].z() = -0.8;
  end[8] = Eigen::Vector3d(-0.5, -0.5, -0.8);

  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {4, 5, 8},
      {6, 7, 8},
  };

  expectMotionAwareSweepMatchesBruteForce(start, end, triangles, 0.05);
}

//==============================================================================
TEST(IpcContactCandidateSet, MotionAwareKeepsFastPointTriangleCrossing)
{
  const std::vector<Eigen::Vector3d> start = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  };
  auto end = start;
  end[3] = Eigen::Vector3d(0.0, 0.0, -1.0);

  const std::vector<sx::DeformableSurfaceTriangle> triangles = {{0, 1, 2}};

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.05;
  options.exactDistanceFilter = true;

  const auto staticStart
      = dc::buildContactCandidatesSweep(start, triangles, options);
  const auto staticEnd
      = dc::buildContactCandidatesSweep(end, triangles, options);
  EXPECT_FALSE(containsPointTriangle(staticStart, 3, 0));
  EXPECT_FALSE(containsPointTriangle(staticEnd, 3, 0));

  const auto motion = dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options);
  const auto brute = dc::buildMotionAwareContactCandidatesBruteForce(
      start, end, triangles, options);
  expectCandidatesEqual(motion, brute);

  ASSERT_TRUE(containsPointTriangle(motion, 3, 0));
  ASSERT_EQ(motion.pointTriangleCandidates.size(), 1u);
  EXPECT_NEAR(
      motion.pointTriangleCandidates.front().squaredDistance, 1.0, 1e-12);
}

//==============================================================================
TEST(IpcContactCandidateSet, MotionAwareKeepsFastEdgeEdgeCrossing)
{
  const std::vector<Eigen::Vector3d> start = {
      {-1.0, 0.0, 0.6},
      {1.0, 0.0, 0.6},
      {0.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {-1.0, 0.5, 0.6},
      {0.5, -1.0, 0.0},
  };
  auto end = start;
  end[0].z() = -0.6;
  end[1].z() = -0.6;
  end[4].z() = -0.6;

  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 4},
      {2, 3, 5},
  };

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.05;
  options.exactDistanceFilter = true;

  const auto staticStart
      = dc::buildContactCandidatesSweep(start, triangles, options);
  const std::size_t movingEdge = findEdge(staticStart.surfaceEdges, 0, 1);
  const std::size_t staticEdge = findEdge(staticStart.surfaceEdges, 2, 3);
  EXPECT_FALSE(containsEdgeEdge(staticStart, movingEdge, staticEdge));

  const auto staticEnd
      = dc::buildContactCandidatesSweep(end, triangles, options);
  EXPECT_FALSE(containsEdgeEdge(staticEnd, movingEdge, staticEdge));

  const auto motion = dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options);
  const auto brute = dc::buildMotionAwareContactCandidatesBruteForce(
      start, end, triangles, options);
  expectCandidatesEqual(motion, brute);

  ASSERT_TRUE(containsEdgeEdge(motion, movingEdge, staticEdge));
  const auto it = std::find_if(
      motion.edgeEdgeCandidates.begin(),
      motion.edgeEdgeCandidates.end(),
      [&](const dc::EdgeEdgeCandidate& candidate) {
        return candidate.edgeA == movingEdge && candidate.edgeB == staticEdge;
      });
  ASSERT_NE(it, motion.edgeEdgeCandidates.end());
  EXPECT_NEAR(it->squaredDistance, 0.36, 1e-12);
}

//==============================================================================
TEST(IpcContactCandidateSet, MotionAwarePreservesTopologyFilters)
{
  const std::vector<Eigen::Vector3d> positions = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
  };
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {{0, 1, 2}};

  dc::ContactCandidateOptions options;
  options.activationDistance = 1.0;
  options.exactDistanceFilter = true;

  auto candidates = dc::buildMotionAwareContactCandidatesSweep(
      positions, positions, triangles, options);
  EXPECT_TRUE(candidates.pointTriangleCandidates.empty());
  EXPECT_TRUE(candidates.edgeEdgeCandidates.empty());
  EXPECT_EQ(candidates.stats.incidentPointTriangleRejectCount, 3u);
  EXPECT_EQ(candidates.stats.adjacentEdgeEdgeRejectCount, 3u);

  options.excludeIncidentPointTriangles = false;
  options.excludeAdjacentEdges = false;
  candidates = dc::buildMotionAwareContactCandidatesSweep(
      positions, positions, triangles, options);
  EXPECT_EQ(candidates.pointTriangleCandidates.size(), 3u);
  EXPECT_EQ(candidates.edgeEdgeCandidates.size(), 3u);
}

//==============================================================================
TEST(
    IpcContactCandidateSet, MotionAwareHandlesEmptyInputsAndDeterministicOutput)
{
  dc::ContactCandidateOptions options;
  options.activationDistance = 0.1;

  const auto empty = dc::buildMotionAwareContactCandidatesSweep(
      std::vector<Eigen::Vector3d>{},
      std::vector<Eigen::Vector3d>{},
      std::vector<sx::DeformableSurfaceTriangle>{},
      options);
  EXPECT_TRUE(empty.surfaceEdges.empty());
  EXPECT_TRUE(empty.pointTriangleCandidates.empty());
  EXPECT_TRUE(empty.edgeEdgeCandidates.empty());

  const std::vector<Eigen::Vector3d> start = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  };
  auto end = start;
  end[3] = Eigen::Vector3d(0.0, 0.0, -1.0);
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {{0, 1, 2}};

  const auto first = dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options);
  const auto second = dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options);
  expectCandidatesEqual(first, second);
  EXPECT_TRUE(
      std::is_sorted(
          first.pointTriangleCandidates.begin(),
          first.pointTriangleCandidates.end()));
  EXPECT_TRUE(
      std::is_sorted(
          first.edgeEdgeCandidates.begin(), first.edgeEdgeCandidates.end()));
}

//==============================================================================
TEST(IpcContactCandidateSet, ReusableBuildersMatchReturnWrappers)
{
  const std::vector<Eigen::Vector3d> start = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
      {-1.0, 0.0, 0.7},
      {1.0, 0.0, 0.7},
      {0.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {-0.5, -0.5, 0.7},
  };
  auto end = start;
  end[3] = Eigen::Vector3d(0.0, 0.0, -1.0);
  end[4].z() = -0.7;
  end[5].z() = -0.7;
  end[8] = Eigen::Vector3d(-0.5, -0.5, -0.7);

  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {4, 5, 8},
      {6, 7, 8},
  };

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.1;

  dc::ContactCandidateSet reusable;
  dc::buildContactCandidatesBruteForce(start, triangles, options, reusable);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildContactCandidatesBruteForce(start, triangles, options));

  dc::buildContactCandidatesSweep(start, triangles, options, reusable);
  expectCandidateSetsEqualIncludingStats(
      reusable, dc::buildContactCandidatesSweep(start, triangles, options));

  dc::detail::ContactCandidateSweepScratch scratch;
  dc::buildContactCandidatesSweep(start, triangles, options, reusable, scratch);
  expectCandidateSetsEqualIncludingStats(
      reusable, dc::buildContactCandidatesSweep(start, triangles, options));

  dc::buildMotionAwareContactCandidatesBruteForce(
      start, end, triangles, options, reusable);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildMotionAwareContactCandidatesBruteForce(
          start, end, triangles, options));

  dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options, reusable);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildMotionAwareContactCandidatesSweep(
          start, end, triangles, options));

  dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options, reusable, scratch);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildMotionAwareContactCandidatesSweep(
          start, end, triangles, options));
}

//==============================================================================
TEST(IpcContactCandidateSet, ReusableBuildersCanBorrowMemoryAllocator)
{
  const std::vector<Eigen::Vector3d> start = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
      {-1.0, 0.0, 0.7},
      {1.0, 0.0, 0.7},
      {0.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {-0.5, -0.5, 0.7},
  };
  auto end = start;
  end[3] = Eigen::Vector3d(0.0, 0.0, -1.0);
  end[4].z() = -0.7;
  end[5].z() = -0.7;
  end[8] = Eigen::Vector3d(-0.5, -0.5, -0.7);

  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {4, 5, 8},
      {6, 7, 8},
  };

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.1;

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBefore = freeList.getAllocationCount();

  {
    dc::ContactCandidateSet candidates(memoryManager.getFreeAllocator());
    dc::detail::ContactCandidateSweepScratch scratch(
        memoryManager.getFreeAllocator());

    dc::buildMotionAwareContactCandidatesSweep(
        start, end, triangles, options, candidates, scratch);

    expectCandidateSetsEqualIncludingStats(
        candidates,
        dc::buildMotionAwareContactCandidatesSweep(
            start, end, triangles, options));
    EXPECT_GT(freeList.getAllocationCount(), allocationsBefore)
        << "allocator-aware contact candidates and sweep scratch should "
           "reserve from the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBefore);
}

//==============================================================================
TEST(IpcContactCandidateSet, NoScratchSweepBuildersBorrowCandidateAllocator)
{
  const std::vector<Eigen::Vector3d> start = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.25, 0.25, 0.0},
      {1.25, 0.25, 0.0},
      {0.25, 1.25, 0.0},
      {0.5, 0.5, 0.5},
  };
  auto end = start;
  end[6] = Eigen::Vector3d(0.5, 0.5, -0.5);
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {3, 5, 4},
  };

  dc::ContactCandidateOptions options;
  options.activationDistance = 1.0;
  options.excludeIncidentPointTriangles = false;
  options.excludeAdjacentEdges = false;

  CountingMemoryAllocator allocator;
  dc::ContactCandidateSet candidates(allocator);
  candidates.surfaceEdges.reserve(6u);
  candidates.pointTriangleCandidates.reserve(64u);
  candidates.edgeEdgeCandidates.reserve(64u);

  const std::size_t allocationsBeforeStatic = allocator.allocations;
  dc::buildContactCandidatesSweep(start, triangles, options, candidates);
  EXPECT_GT(allocator.allocations, allocationsBeforeStatic)
      << "static no-scratch sweep builder should borrow candidate allocator "
         "for local sweep scratch";
  expectCandidateSetsEqualIncludingStats(
      candidates, dc::buildContactCandidatesSweep(start, triangles, options));

  const std::size_t allocationsBeforeMotionAware = allocator.allocations;
  dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options, candidates);
  EXPECT_GT(allocator.allocations, allocationsBeforeMotionAware)
      << "motion-aware no-scratch sweep builder should borrow candidate "
         "allocator for local sweep scratch";
  expectCandidateSetsEqualIncludingStats(
      candidates,
      dc::buildMotionAwareContactCandidatesSweep(
          start, end, triangles, options));
}

//==============================================================================
TEST(IpcContactCandidateSet, ReturnBuildersCanUseProvidedAllocator)
{
  const std::vector<Eigen::Vector3d> start = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.25, 0.25, 0.0},
      {1.25, 0.25, 0.0},
      {0.25, 1.25, 0.0},
      {0.5, 0.5, 0.5},
  };
  auto end = start;
  end[6] = Eigen::Vector3d(0.5, 0.5, -0.5);
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {
      {0, 1, 2},
      {3, 5, 4},
  };

  dc::ContactCandidateOptions options;
  options.activationDistance = 1.0;
  options.excludeIncidentPointTriangles = false;
  options.excludeAdjacentEdges = false;

  CountingMemoryAllocator allocator;
  const dc::ContactCandidateSet::SurfaceEdgeAllocator edgeAllocator{allocator};
  const dc::ContactCandidateSet::PointTriangleAllocator pointAllocator{
      allocator};
  const dc::ContactCandidateSet::EdgeEdgeAllocator edgeEdgeAllocator{allocator};

  const auto expectAllocatorBacked =
      [&](const dc::ContactCandidateSet& candidates) {
        EXPECT_EQ(candidates.surfaceEdges.get_allocator(), edgeAllocator);
        EXPECT_EQ(
            candidates.pointTriangleCandidates.get_allocator(), pointAllocator);
        EXPECT_EQ(
            candidates.edgeEdgeCandidates.get_allocator(), edgeEdgeAllocator);
      };

  const std::size_t allocationsBefore = allocator.allocations;

  const auto staticBrute = dc::buildContactCandidatesBruteForce(
      start, triangles, options, allocator);
  expectAllocatorBacked(staticBrute);
  expectCandidateSetsEqualIncludingStats(
      staticBrute,
      dc::buildContactCandidatesBruteForce(start, triangles, options));

  const auto staticSweep
      = dc::buildContactCandidatesSweep(start, triangles, options, allocator);
  expectAllocatorBacked(staticSweep);
  expectCandidateSetsEqualIncludingStats(
      staticSweep, dc::buildContactCandidatesSweep(start, triangles, options));

  const auto motionBrute = dc::buildMotionAwareContactCandidatesBruteForce(
      start, end, triangles, options, allocator);
  expectAllocatorBacked(motionBrute);
  expectCandidateSetsEqualIncludingStats(
      motionBrute,
      dc::buildMotionAwareContactCandidatesBruteForce(
          start, end, triangles, options));

  const auto motionSweep = dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options, allocator);
  expectAllocatorBacked(motionSweep);
  expectCandidateSetsEqualIncludingStats(
      motionSweep,
      dc::buildMotionAwareContactCandidatesSweep(
          start, end, triangles, options));

  EXPECT_GT(allocator.allocations, allocationsBefore)
      << "allocator-aware return builders should reserve candidate and sweep "
         "scratch storage through the provided allocator";
}

//==============================================================================
TEST(IpcContactCandidateSet, ReusableBuildersClearStaleStateAndPreserveCapacity)
{
  const std::vector<Eigen::Vector3d> dense = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.25, 0.25, 0.0},
      {1.25, 0.25, 0.0},
      {0.25, 1.25, 0.0},
      {0.5, 0.5, 0.05},
  };
  const std::vector<sx::DeformableSurfaceTriangle> denseTriangles = {
      {0, 1, 2},
      {3, 5, 4},
  };

  dc::ContactCandidateOptions options;
  options.activationDistance = 1.0;
  options.excludeIncidentPointTriangles = false;
  options.excludeAdjacentEdges = false;

  dc::ContactCandidateSet reusable;
  reusable.surfaceEdges.reserve(64);
  reusable.pointTriangleCandidates.reserve(64);
  reusable.edgeEdgeCandidates.reserve(64);
  const auto surfaceCapacity = reusable.surfaceEdges.capacity();
  const auto pointTriangleCapacity
      = reusable.pointTriangleCandidates.capacity();
  const auto edgeEdgeCapacity = reusable.edgeEdgeCandidates.capacity();

  dc::buildContactCandidatesSweep(dense, denseTriangles, options, reusable);
  ASSERT_FALSE(reusable.surfaceEdges.empty());
  ASSERT_FALSE(reusable.pointTriangleCandidates.empty());
  ASSERT_FALSE(reusable.edgeEdgeCandidates.empty());
  ASSERT_GT(reusable.stats.pointTriangleCandidateCount, 0u);
  ASSERT_GT(reusable.stats.edgeEdgeCandidateCount, 0u);

  const std::vector<Eigen::Vector3d> empty;
  const std::vector<sx::DeformableSurfaceTriangle> noTriangles;
  options.excludeIncidentPointTriangles = true;
  options.excludeAdjacentEdges = true;
  dc::buildContactCandidatesSweep(empty, noTriangles, options, reusable);

  EXPECT_TRUE(reusable.surfaceEdges.empty());
  EXPECT_TRUE(reusable.pointTriangleCandidates.empty());
  EXPECT_TRUE(reusable.edgeEdgeCandidates.empty());
  EXPECT_EQ(reusable.stats.pointCount, 0u);
  EXPECT_EQ(reusable.stats.triangleCount, 0u);
  EXPECT_EQ(reusable.stats.edgeCount, 0u);
  EXPECT_EQ(reusable.stats.broadPhaseOverlapCount, 0u);
  EXPECT_EQ(reusable.stats.exactDistanceCheckCount, 0u);
  EXPECT_EQ(reusable.stats.incidentPointTriangleRejectCount, 0u);
  EXPECT_EQ(reusable.stats.adjacentEdgeEdgeRejectCount, 0u);
  EXPECT_EQ(reusable.stats.pointTriangleCandidateCount, 0u);
  EXPECT_EQ(reusable.stats.edgeEdgeCandidateCount, 0u);
  EXPECT_GE(reusable.surfaceEdges.capacity(), surfaceCapacity);
  EXPECT_GE(reusable.pointTriangleCandidates.capacity(), pointTriangleCapacity);
  EXPECT_GE(reusable.edgeEdgeCandidates.capacity(), edgeEdgeCapacity);
}

//==============================================================================
TEST(IpcContactCandidateSet, StaticSweepScratchClearsStaleState)
{
  const std::vector<Eigen::Vector3d> dense = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.25, 0.25, 0.0},
      {1.25, 0.25, 0.0},
      {0.25, 1.25, 0.0},
      {0.5, 0.5, 0.05},
  };
  const std::vector<sx::DeformableSurfaceTriangle> denseTriangles = {
      {0, 1, 2},
      {3, 5, 4},
  };

  const std::vector<Eigen::Vector3d> smaller = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 0.02},
  };
  const std::vector<sx::DeformableSurfaceTriangle> smallerTriangles
      = {{0, 1, 2}};

  dc::ContactCandidateOptions options;
  options.activationDistance = 1.0;
  options.excludeIncidentPointTriangles = false;
  options.excludeAdjacentEdges = false;

  dc::ContactCandidateSet reusable;
  dc::detail::ContactCandidateSweepScratch scratch;
  scratch.pointItems.resize(128);
  scratch.triangleItems.resize(128);
  scratch.edgeItems.resize(128);

  const auto pointCapacity = scratch.pointItems.capacity();
  const auto triangleCapacity = scratch.triangleItems.capacity();
  const auto edgeCapacity = scratch.edgeItems.capacity();

  dc::buildContactCandidatesSweep(
      dense, denseTriangles, options, reusable, scratch);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildContactCandidatesSweep(dense, denseTriangles, options));
  ASSERT_GT(scratch.pointItems.size(), 0u);
  ASSERT_GT(scratch.triangleItems.size(), 0u);
  ASSERT_GT(scratch.edgeItems.size(), 0u);

  const std::vector<Eigen::Vector3d> empty;
  const std::vector<sx::DeformableSurfaceTriangle> noTriangles;
  dc::buildContactCandidatesSweep(
      empty, noTriangles, options, reusable, scratch);
  expectCandidateSetsEqualIncludingStats(
      reusable, dc::buildContactCandidatesSweep(empty, noTriangles, options));
  EXPECT_TRUE(scratch.pointItems.empty());
  EXPECT_TRUE(scratch.triangleItems.empty());
  EXPECT_TRUE(scratch.edgeItems.empty());

  dc::buildContactCandidatesSweep(
      smaller, smallerTriangles, options, reusable, scratch);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildContactCandidatesSweep(smaller, smallerTriangles, options));
  EXPECT_EQ(scratch.pointItems.size(), smaller.size());
  EXPECT_EQ(scratch.triangleItems.size(), smallerTriangles.size());
  EXPECT_EQ(scratch.edgeItems.size(), reusable.surfaceEdges.size());
  EXPECT_GE(scratch.pointItems.capacity(), pointCapacity);
  EXPECT_GE(scratch.triangleItems.capacity(), triangleCapacity);
  EXPECT_GE(scratch.edgeItems.capacity(), edgeCapacity);
}

//==============================================================================
TEST(IpcContactCandidateSet, ReusableBuildersHandleTopologyChanges)
{
  const std::vector<Eigen::Vector3d> first = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.5, 0.5, 0.1},
      {1.5, 0.5, 0.1},
      {0.5, 1.5, 0.1},
  };
  const std::vector<sx::DeformableSurfaceTriangle> firstTriangles = {
      {0, 1, 2},
      {3, 5, 4},
  };

  const std::vector<Eigen::Vector3d> second = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 0.02},
  };
  const std::vector<sx::DeformableSurfaceTriangle> secondTriangles
      = {{0, 1, 2}};

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.2;

  dc::ContactCandidateSet reusable;
  dc::buildContactCandidatesSweep(first, firstTriangles, options, reusable);
  ASSERT_EQ(reusable.surfaceEdges.size(), 6u);

  dc::buildContactCandidatesSweep(second, secondTriangles, options, reusable);
  const auto fresh
      = dc::buildContactCandidatesSweep(second, secondTriangles, options);

  expectCandidateSetsEqualIncludingStats(reusable, fresh);
  ASSERT_EQ(reusable.surfaceEdges.size(), 3u);
  EXPECT_TRUE(containsPointTriangle(reusable, 3, 0));
}

//==============================================================================
TEST(IpcContactCandidateSet, ReusableMotionAwareBuildersPreserveCapacity)
{
  const std::vector<Eigen::Vector3d> start = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  };
  auto end = start;
  end[3] = Eigen::Vector3d(0.0, 0.0, -1.0);
  const std::vector<sx::DeformableSurfaceTriangle> triangles = {{0, 1, 2}};

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.05;

  dc::ContactCandidateSet reusable;
  reusable.surfaceEdges.reserve(32);
  reusable.pointTriangleCandidates.reserve(32);
  reusable.edgeEdgeCandidates.reserve(32);
  const auto surfaceCapacity = reusable.surfaceEdges.capacity();
  const auto pointTriangleCapacity
      = reusable.pointTriangleCandidates.capacity();
  const auto edgeEdgeCapacity = reusable.edgeEdgeCandidates.capacity();

  dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, options, reusable);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildMotionAwareContactCandidatesSweep(
          start, end, triangles, options));

  dc::buildMotionAwareContactCandidatesSweep(
      std::vector<Eigen::Vector3d>{},
      std::vector<Eigen::Vector3d>{},
      std::vector<sx::DeformableSurfaceTriangle>{},
      options,
      reusable);
  EXPECT_TRUE(reusable.surfaceEdges.empty());
  EXPECT_TRUE(reusable.pointTriangleCandidates.empty());
  EXPECT_TRUE(reusable.edgeEdgeCandidates.empty());
  EXPECT_EQ(reusable.stats.pointTriangleCandidateCount, 0u);
  EXPECT_EQ(reusable.stats.edgeEdgeCandidateCount, 0u);
  EXPECT_GE(reusable.surfaceEdges.capacity(), surfaceCapacity);
  EXPECT_GE(reusable.pointTriangleCandidates.capacity(), pointTriangleCapacity);
  EXPECT_GE(reusable.edgeEdgeCandidates.capacity(), edgeEdgeCapacity);
}

//==============================================================================
TEST(IpcContactCandidateSet, MotionAwareSweepScratchClearsStaleState)
{
  const std::vector<Eigen::Vector3d> denseStart = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.25, 0.25, 0.0},
      {1.25, 0.25, 0.0},
      {0.25, 1.25, 0.0},
      {0.5, 0.5, 1.0},
  };
  auto denseEnd = denseStart;
  denseEnd[6] = Eigen::Vector3d(0.5, 0.5, -1.0);
  const std::vector<sx::DeformableSurfaceTriangle> denseTriangles = {
      {0, 1, 2},
      {3, 5, 4},
  };

  const std::vector<Eigen::Vector3d> smallerStart = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  };
  auto smallerEnd = smallerStart;
  smallerEnd[3] = Eigen::Vector3d(0.0, 0.0, -1.0);
  const std::vector<sx::DeformableSurfaceTriangle> smallerTriangles
      = {{0, 1, 2}};

  dc::ContactCandidateOptions options;
  options.activationDistance = 0.05;

  dc::ContactCandidateSet reusable;
  dc::detail::ContactCandidateSweepScratch scratch;
  scratch.pointItems.resize(128);
  scratch.triangleItems.resize(128);
  scratch.edgeItems.resize(128);

  const auto pointCapacity = scratch.pointItems.capacity();
  const auto triangleCapacity = scratch.triangleItems.capacity();
  const auto edgeCapacity = scratch.edgeItems.capacity();

  dc::buildMotionAwareContactCandidatesSweep(
      denseStart, denseEnd, denseTriangles, options, reusable, scratch);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildMotionAwareContactCandidatesSweep(
          denseStart, denseEnd, denseTriangles, options));
  ASSERT_GT(scratch.pointItems.size(), 0u);
  ASSERT_GT(scratch.triangleItems.size(), 0u);
  ASSERT_GT(scratch.edgeItems.size(), 0u);

  const std::vector<Eigen::Vector3d> empty;
  const std::vector<sx::DeformableSurfaceTriangle> noTriangles;
  dc::buildMotionAwareContactCandidatesSweep(
      empty, empty, noTriangles, options, reusable, scratch);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildMotionAwareContactCandidatesSweep(
          empty, empty, noTriangles, options));
  EXPECT_TRUE(scratch.pointItems.empty());
  EXPECT_TRUE(scratch.triangleItems.empty());
  EXPECT_TRUE(scratch.edgeItems.empty());

  dc::buildMotionAwareContactCandidatesSweep(
      smallerStart, smallerEnd, smallerTriangles, options, reusable, scratch);
  expectCandidateSetsEqualIncludingStats(
      reusable,
      dc::buildMotionAwareContactCandidatesSweep(
          smallerStart, smallerEnd, smallerTriangles, options));
  EXPECT_EQ(scratch.pointItems.size(), smallerStart.size());
  EXPECT_EQ(scratch.triangleItems.size(), smallerTriangles.size());
  EXPECT_EQ(scratch.edgeItems.size(), reusable.surfaceEdges.size());
  EXPECT_GE(scratch.pointItems.capacity(), pointCapacity);
  EXPECT_GE(scratch.triangleItems.capacity(), triangleCapacity);
  EXPECT_GE(scratch.edgeItems.capacity(), edgeCapacity);
}
