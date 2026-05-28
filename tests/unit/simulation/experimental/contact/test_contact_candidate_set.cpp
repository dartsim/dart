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

#include <dart/simulation/experimental/detail/deformable_contact/candidate_set.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace sx = dart::simulation::experimental;
namespace dc = dart::simulation::experimental::detail::deformable_contact;

namespace {

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
    const std::vector<dc::SurfaceEdge>& edges, std::size_t a, std::size_t b)
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
