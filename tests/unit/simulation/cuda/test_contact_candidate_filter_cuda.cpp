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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/detail/deformable_contact/candidate_set.hpp>

#include <Eigen/Core>
#include <dart/simulation/compute/cuda/contact_candidate_filter_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <vector>

#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;
namespace dc = dart::simulation::detail::deformable_contact;

namespace {

struct Fixture
{
  std::vector<double> positions;
  std::vector<std::uint32_t> triangles;
  std::vector<cuda::PointTriangleContactStencil> stencils;
};

struct EdgeEdgeFixture
{
  std::vector<double> positions;
  std::vector<cuda::EdgeEdgeContactStencil> stencils;
};

struct SweptFixture
{
  std::vector<double> startPositions;
  std::vector<double> endPositions;
  std::vector<std::uint32_t> triangles;
  std::vector<std::uint32_t> edges;
};

template <typename FixtureT>
void appendPoint(FixtureT& fixture, const Eigen::Vector3d& point)
{
  fixture.positions.push_back(point.x());
  fixture.positions.push_back(point.y());
  fixture.positions.push_back(point.z());
}

void appendPoint(std::vector<double>& positions, const Eigen::Vector3d& point)
{
  positions.push_back(point.x());
  positions.push_back(point.y());
  positions.push_back(point.z());
}

Fixture makeFixture()
{
  Fixture fixture;
  fixture.positions.reserve(24);
  fixture.triangles.reserve(6);
  fixture.stencils.reserve(2);

  appendPoint(fixture, {0.0, 0.0, 0.0});
  appendPoint(fixture, {1.0, 0.0, 0.0});
  appendPoint(fixture, {0.0, 1.0, 0.0});
  appendPoint(fixture, {0.25, 0.25, 0.02});
  fixture.triangles.insert(fixture.triangles.end(), {0u, 1u, 2u});
  fixture.stencils.push_back({3u, 0u});

  appendPoint(fixture, {2.0, 0.0, 0.0});
  appendPoint(fixture, {3.0, 0.0, 0.0});
  appendPoint(fixture, {2.0, 1.0, 0.0});
  appendPoint(fixture, {2.25, 0.25, 0.08});
  fixture.triangles.insert(fixture.triangles.end(), {4u, 5u, 6u});
  fixture.stencils.push_back({7u, 1u});

  return fixture;
}

EdgeEdgeFixture makeEdgeEdgeFixture()
{
  EdgeEdgeFixture fixture;
  fixture.positions.reserve(24);
  fixture.stencils.reserve(2);

  appendPoint(fixture, {0.0, 0.0, 0.0});
  appendPoint(fixture, {1.0, 0.0, 0.0});
  appendPoint(fixture, {0.25, 0.02, -0.5});
  appendPoint(fixture, {0.25, 0.02, 0.5});
  fixture.stencils.push_back({0u, 1u, 2u, 3u});

  appendPoint(fixture, {2.0, 0.0, 0.0});
  appendPoint(fixture, {3.0, 0.0, 0.0});
  appendPoint(fixture, {2.25, 0.08, -0.5});
  appendPoint(fixture, {2.25, 0.08, 0.5});
  fixture.stencils.push_back({4u, 5u, 6u, 7u});

  return fixture;
}

SweptFixture makeSweptFixture()
{
  SweptFixture fixture;
  fixture.startPositions.reserve(18);
  fixture.endPositions.reserve(18);
  fixture.triangles.reserve(3);
  fixture.edges.reserve(4);

  appendPoint(fixture.startPositions, {0.0, 0.0, 0.0});
  appendPoint(fixture.endPositions, {0.0, 0.0, 0.0});
  appendPoint(fixture.startPositions, {1.0, 0.0, 0.0});
  appendPoint(fixture.endPositions, {1.0, 0.0, 0.0});
  appendPoint(fixture.startPositions, {0.0, 1.0, 0.0});
  appendPoint(fixture.endPositions, {0.0, 1.0, 0.0});
  appendPoint(fixture.startPositions, {0.25, 0.25, 0.20});
  appendPoint(fixture.endPositions, {0.25, 0.25, -0.20});
  fixture.triangles.insert(fixture.triangles.end(), {0u, 1u, 2u});

  appendPoint(fixture.startPositions, {2.0, 0.0, 0.0});
  appendPoint(fixture.endPositions, {2.0, 0.0, 0.0});
  appendPoint(fixture.startPositions, {3.0, 0.0, 0.0});
  appendPoint(fixture.endPositions, {3.0, 0.0, 0.0});
  appendPoint(fixture.startPositions, {2.25, 0.20, -0.5});
  appendPoint(fixture.endPositions, {2.25, -0.20, -0.5});
  appendPoint(fixture.startPositions, {2.25, 0.20, 0.5});
  appendPoint(fixture.endPositions, {2.25, -0.20, 0.5});
  fixture.edges.insert(fixture.edges.end(), {4u, 5u, 6u, 7u});

  return fixture;
}

Eigen::Vector3d pointAt(
    const std::vector<double>& positions, const std::size_t i)
{
  const std::size_t base = 3u * i;
  return {positions[base], positions[base + 1u], positions[base + 2u]};
}

bool isIncidentPointTriangle(
    const std::uint32_t point,
    const std::vector<std::uint32_t>& triangles,
    const std::size_t triangle)
{
  const std::size_t tri = 3u * triangle;
  return point == triangles[tri] || point == triangles[tri + 1u]
         || point == triangles[tri + 2u];
}

std::vector<double> cpuSquaredDistances(const Fixture& fixture)
{
  std::vector<double> distances;
  distances.reserve(fixture.stencils.size());
  for (const auto& stencil : fixture.stencils) {
    const std::size_t tri = 3u * static_cast<std::size_t>(stencil.triangle);
    const auto distance = dc::pointTriangleSquaredDistance(
        pointAt(fixture.positions, stencil.point),
        pointAt(fixture.positions, fixture.triangles[tri]),
        pointAt(fixture.positions, fixture.triangles[tri + 1u]),
        pointAt(fixture.positions, fixture.triangles[tri + 2u]));
    distances.push_back(distance.squaredDistance);
  }
  return distances;
}

std::vector<double> cpuSquaredDistances(const EdgeEdgeFixture& fixture)
{
  std::vector<double> distances;
  distances.reserve(fixture.stencils.size());
  for (const auto& stencil : fixture.stencils) {
    const auto distance = dc::edgeEdgeSquaredDistance(
        pointAt(fixture.positions, stencil.edgeAStart),
        pointAt(fixture.positions, stencil.edgeAEnd),
        pointAt(fixture.positions, stencil.edgeBStart),
        pointAt(fixture.positions, stencil.edgeBEnd));
    distances.push_back(distance.squaredDistance);
  }
  return distances;
}

struct CandidateMaskExpectation
{
  std::vector<double> squaredDistances;
  std::vector<std::uint8_t> accepted;
  std::vector<std::uint32_t> acceptedPointIndices;
  std::vector<std::uint32_t> acceptedTriangleIndices;
  std::size_t acceptedCount = 0;
};

struct EdgeEdgeCandidateMaskExpectation
{
  std::vector<double> squaredDistances;
  std::vector<std::uint8_t> accepted;
  std::vector<std::uint32_t> acceptedEdgeAIndices;
  std::vector<std::uint32_t> acceptedEdgeBIndices;
  std::size_t acceptedCount = 0;
};

struct SweptCandidateMaskExpectation
{
  std::vector<double> endpointSquaredDistances;
  std::vector<std::uint8_t> accepted;
  std::vector<std::uint32_t> acceptedPointIndices;
  std::vector<std::uint32_t> acceptedTriangleIndices;
  std::size_t acceptedCount = 0;
};

struct SweptEdgeEdgeCandidateMaskExpectation
{
  std::vector<double> endpointSquaredDistances;
  std::vector<std::uint8_t> accepted;
  std::vector<std::uint32_t> acceptedEdgeAIndices;
  std::vector<std::uint32_t> acceptedEdgeBIndices;
  std::size_t acceptedCount = 0;
};

CandidateMaskExpectation cpuCandidateMask(
    const Fixture& fixture,
    const std::vector<std::uint32_t>& pointIndices,
    const double activationDistance)
{
  CandidateMaskExpectation expected;
  const std::size_t triangleCount = fixture.triangles.size() / 3u;
  expected.squaredDistances.reserve(pointIndices.size() * triangleCount);
  expected.accepted.reserve(pointIndices.size() * triangleCount);

  for (const std::uint32_t point : pointIndices) {
    for (std::size_t triangle = 0; triangle < triangleCount; ++triangle) {
      const std::size_t tri = 3u * triangle;
      double squaredDistance = 0.0;
      bool accepted = false;
      if (!isIncidentPointTriangle(point, fixture.triangles, triangle)) {
        const auto distance = dc::pointTriangleSquaredDistance(
            pointAt(fixture.positions, point),
            pointAt(fixture.positions, fixture.triangles[tri]),
            pointAt(fixture.positions, fixture.triangles[tri + 1u]),
            pointAt(fixture.positions, fixture.triangles[tri + 2u]));
        squaredDistance = distance.squaredDistance;
        accepted = dc::detail::withinActivationDistance(
            squaredDistance, activationDistance);
      }
      expected.squaredDistances.push_back(squaredDistance);
      expected.accepted.push_back(accepted ? 1u : 0u);
      if (accepted) {
        ++expected.acceptedCount;
        expected.acceptedPointIndices.push_back(point);
        expected.acceptedTriangleIndices.push_back(
            static_cast<std::uint32_t>(triangle));
      }
    }
  }

  return expected;
}

bool edgesShareVertex(
    const std::uint32_t a0,
    const std::uint32_t a1,
    const std::uint32_t b0,
    const std::uint32_t b1)
{
  return a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1;
}

EdgeEdgeCandidateMaskExpectation cpuEdgeEdgeCandidateMask(
    const EdgeEdgeFixture& fixture,
    const std::vector<std::uint32_t>& edgeIndices,
    const double activationDistance)
{
  EdgeEdgeCandidateMaskExpectation expected;
  const std::size_t edgeCount = edgeIndices.size() / 2u;
  expected.squaredDistances.reserve(edgeCount * edgeCount);
  expected.accepted.reserve(edgeCount * edgeCount);

  for (std::size_t edgeA = 0; edgeA < edgeCount; ++edgeA) {
    for (std::size_t edgeB = 0; edgeB < edgeCount; ++edgeB) {
      double squaredDistance = 0.0;
      bool accepted = false;
      const std::uint32_t a0 = edgeIndices[2u * edgeA];
      const std::uint32_t a1 = edgeIndices[2u * edgeA + 1u];
      const std::uint32_t b0 = edgeIndices[2u * edgeB];
      const std::uint32_t b1 = edgeIndices[2u * edgeB + 1u];
      if (edgeA < edgeB && !edgesShareVertex(a0, a1, b0, b1)) {
        const auto distance = dc::edgeEdgeSquaredDistance(
            pointAt(fixture.positions, a0),
            pointAt(fixture.positions, a1),
            pointAt(fixture.positions, b0),
            pointAt(fixture.positions, b1));
        squaredDistance = distance.squaredDistance;
        accepted = dc::detail::withinActivationDistance(
            squaredDistance, activationDistance);
      }
      expected.squaredDistances.push_back(squaredDistance);
      expected.accepted.push_back(accepted ? 1u : 0u);
      if (accepted) {
        ++expected.acceptedCount;
        expected.acceptedEdgeAIndices.push_back(
            static_cast<std::uint32_t>(edgeA));
        expected.acceptedEdgeBIndices.push_back(
            static_cast<std::uint32_t>(edgeB));
      }
    }
  }

  return expected;
}

SweptCandidateMaskExpectation cpuSweptPointTriangleCandidateMask(
    const SweptFixture& fixture,
    const std::vector<std::uint32_t>& pointIndices,
    const double activationDistance)
{
  SweptCandidateMaskExpectation expected;
  const std::size_t triangleCount = fixture.triangles.size() / 3u;
  expected.endpointSquaredDistances.reserve(
      pointIndices.size() * triangleCount);
  expected.accepted.reserve(pointIndices.size() * triangleCount);
  const double margin = 0.5 * std::max(0.0, activationDistance);

  for (const std::uint32_t point : pointIndices) {
    for (std::size_t triangle = 0; triangle < triangleCount; ++triangle) {
      const std::size_t tri = 3u * triangle;
      double endpointSquaredDistance = 0.0;
      bool accepted = false;
      if (!isIncidentPointTriangle(point, fixture.triangles, triangle)) {
        const auto pointAabb = dc::detail::makeSweptPointAabb(
            pointAt(fixture.startPositions, point),
            pointAt(fixture.endPositions, point),
            margin);
        const auto triangleAabb = dc::detail::makeSweptTriangleAabb(
            pointAt(fixture.startPositions, fixture.triangles[tri]),
            pointAt(fixture.endPositions, fixture.triangles[tri]),
            pointAt(fixture.startPositions, fixture.triangles[tri + 1u]),
            pointAt(fixture.endPositions, fixture.triangles[tri + 1u]),
            pointAt(fixture.startPositions, fixture.triangles[tri + 2u]),
            pointAt(fixture.endPositions, fixture.triangles[tri + 2u]),
            margin);
        accepted = pointAabb.overlaps(triangleAabb);
        if (accepted) {
          const auto startDistance = dc::pointTriangleSquaredDistance(
              pointAt(fixture.startPositions, point),
              pointAt(fixture.startPositions, fixture.triangles[tri]),
              pointAt(fixture.startPositions, fixture.triangles[tri + 1u]),
              pointAt(fixture.startPositions, fixture.triangles[tri + 2u]));
          const auto endDistance = dc::pointTriangleSquaredDistance(
              pointAt(fixture.endPositions, point),
              pointAt(fixture.endPositions, fixture.triangles[tri]),
              pointAt(fixture.endPositions, fixture.triangles[tri + 1u]),
              pointAt(fixture.endPositions, fixture.triangles[tri + 2u]));
          endpointSquaredDistance = std::min(
              startDistance.squaredDistance, endDistance.squaredDistance);
        }
      }
      expected.endpointSquaredDistances.push_back(endpointSquaredDistance);
      expected.accepted.push_back(accepted ? 1u : 0u);
      if (accepted) {
        ++expected.acceptedCount;
        expected.acceptedPointIndices.push_back(point);
        expected.acceptedTriangleIndices.push_back(
            static_cast<std::uint32_t>(triangle));
      }
    }
  }

  return expected;
}

SweptEdgeEdgeCandidateMaskExpectation cpuSweptEdgeEdgeCandidateMask(
    const SweptFixture& fixture,
    const std::vector<std::uint32_t>& edgeIndices,
    const double activationDistance)
{
  SweptEdgeEdgeCandidateMaskExpectation expected;
  const std::size_t edgeCount = edgeIndices.size() / 2u;
  expected.endpointSquaredDistances.reserve(edgeCount * edgeCount);
  expected.accepted.reserve(edgeCount * edgeCount);
  const double margin = 0.5 * std::max(0.0, activationDistance);

  for (std::size_t edgeA = 0; edgeA < edgeCount; ++edgeA) {
    for (std::size_t edgeB = 0; edgeB < edgeCount; ++edgeB) {
      double endpointSquaredDistance = 0.0;
      bool accepted = false;
      const std::uint32_t a0 = edgeIndices[2u * edgeA];
      const std::uint32_t a1 = edgeIndices[2u * edgeA + 1u];
      const std::uint32_t b0 = edgeIndices[2u * edgeB];
      const std::uint32_t b1 = edgeIndices[2u * edgeB + 1u];
      if (edgeA < edgeB && !edgesShareVertex(a0, a1, b0, b1)) {
        const auto edgeAAabb = dc::detail::makeSweptSegmentAabb(
            pointAt(fixture.startPositions, a0),
            pointAt(fixture.endPositions, a0),
            pointAt(fixture.startPositions, a1),
            pointAt(fixture.endPositions, a1),
            margin);
        const auto edgeBAabb = dc::detail::makeSweptSegmentAabb(
            pointAt(fixture.startPositions, b0),
            pointAt(fixture.endPositions, b0),
            pointAt(fixture.startPositions, b1),
            pointAt(fixture.endPositions, b1),
            margin);
        accepted = edgeAAabb.overlaps(edgeBAabb);
        if (accepted) {
          const auto startDistance = dc::edgeEdgeSquaredDistance(
              pointAt(fixture.startPositions, a0),
              pointAt(fixture.startPositions, a1),
              pointAt(fixture.startPositions, b0),
              pointAt(fixture.startPositions, b1));
          const auto endDistance = dc::edgeEdgeSquaredDistance(
              pointAt(fixture.endPositions, a0),
              pointAt(fixture.endPositions, a1),
              pointAt(fixture.endPositions, b0),
              pointAt(fixture.endPositions, b1));
          endpointSquaredDistance = std::min(
              startDistance.squaredDistance, endDistance.squaredDistance);
        }
      }
      expected.endpointSquaredDistances.push_back(endpointSquaredDistance);
      expected.accepted.push_back(accepted ? 1u : 0u);
      if (accepted) {
        ++expected.acceptedCount;
        expected.acceptedEdgeAIndices.push_back(
            static_cast<std::uint32_t>(edgeA));
        expected.acceptedEdgeBIndices.push_back(
            static_cast<std::uint32_t>(edgeB));
      }
    }
  }

  return expected;
}

} // namespace

//==============================================================================
TEST(ContactCandidateFilterCuda, MatchesCpuPointTriangleStencilFilter)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const Fixture fixture = makeFixture();
  const std::vector<double> expectedDistances = cpuSquaredDistances(fixture);

  cuda::PointTriangleCandidateFilterResult result;
  cuda::filterPointTriangleContactStencilsCuda(
      fixture.positions, fixture.triangles, fixture.stencils, 0.05, result);

  ASSERT_EQ(result.squaredDistances.size(), expectedDistances.size());
  ASSERT_EQ(result.accepted.size(), expectedDistances.size());
  ASSERT_EQ(result.acceptedCount, 1u);
  EXPECT_NEAR(result.squaredDistances[0], expectedDistances[0], 1e-14);
  EXPECT_NEAR(result.squaredDistances[1], expectedDistances[1], 1e-14);
  EXPECT_EQ(result.accepted[0], 1u);
  EXPECT_EQ(result.accepted[1], 0u);
  EXPECT_GT(result.timing.kernelNs, 0.0);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, MatchesCpuPointTriangleCandidateMask)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const Fixture fixture = makeFixture();
  const std::vector<std::uint32_t> pointIndices = {3u, 7u};
  const CandidateMaskExpectation expected
      = cpuCandidateMask(fixture, pointIndices, 0.05);

  cuda::PointTriangleCandidateBuildResult result;
  cuda::buildPointTriangleContactCandidateMaskCuda(
      fixture.positions, pointIndices, fixture.triangles, 0.05, result);

  ASSERT_EQ(result.squaredDistances.size(), expected.squaredDistances.size());
  ASSERT_EQ(result.accepted.size(), expected.accepted.size());
  EXPECT_EQ(result.pointCount, pointIndices.size());
  EXPECT_EQ(result.triangleCount, fixture.triangles.size() / 3u);
  EXPECT_EQ(result.pairCount, expected.squaredDistances.size());
  EXPECT_EQ(result.acceptedCount, expected.acceptedCount);
  EXPECT_EQ(result.acceptedPointIndices, expected.acceptedPointIndices);
  EXPECT_EQ(result.acceptedTriangleIndices, expected.acceptedTriangleIndices);

  for (std::size_t pair = 0; pair < expected.squaredDistances.size(); ++pair) {
    EXPECT_NEAR(
        result.squaredDistances[pair], expected.squaredDistances[pair], 1e-14)
        << pair;
    EXPECT_EQ(result.accepted[pair], expected.accepted[pair]) << pair;
  }
  EXPECT_GT(result.timing.kernelNs, 0.0);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, MasksIncidentPointTriangleCandidatePairs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const Fixture fixture = makeFixture();
  const std::vector<std::uint32_t> pointIndices = {0u};

  cuda::PointTriangleCandidateBuildResult result;
  cuda::buildPointTriangleContactCandidateMaskCuda(
      fixture.positions, pointIndices, fixture.triangles, 0.05, result);

  ASSERT_EQ(result.pairCount, 2u);
  ASSERT_EQ(result.squaredDistances.size(), 2u);
  ASSERT_EQ(result.accepted.size(), 2u);
  EXPECT_EQ(result.squaredDistances[0], 0.0);
  EXPECT_EQ(result.accepted[0], 0u);
  EXPECT_TRUE(result.acceptedPointIndices.empty());
  EXPECT_TRUE(result.acceptedTriangleIndices.empty());
}

//==============================================================================
TEST(ContactCandidateFilterCuda, MatchesCpuEdgeEdgeCandidateMask)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const EdgeEdgeFixture fixture = makeEdgeEdgeFixture();
  const std::vector<std::uint32_t> edgeIndices
      = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
  const EdgeEdgeCandidateMaskExpectation expected
      = cpuEdgeEdgeCandidateMask(fixture, edgeIndices, 0.05);

  cuda::EdgeEdgeCandidateBuildResult result;
  cuda::buildEdgeEdgeContactCandidateMaskCuda(
      fixture.positions, edgeIndices, 0.05, result);

  ASSERT_EQ(result.squaredDistances.size(), expected.squaredDistances.size());
  ASSERT_EQ(result.accepted.size(), expected.accepted.size());
  EXPECT_EQ(result.edgeCount, edgeIndices.size() / 2u);
  EXPECT_EQ(result.pairCount, expected.squaredDistances.size());
  EXPECT_EQ(result.acceptedCount, expected.acceptedCount);
  EXPECT_EQ(result.acceptedEdgeAIndices, expected.acceptedEdgeAIndices);
  EXPECT_EQ(result.acceptedEdgeBIndices, expected.acceptedEdgeBIndices);

  for (std::size_t pair = 0; pair < expected.squaredDistances.size(); ++pair) {
    EXPECT_NEAR(
        result.squaredDistances[pair], expected.squaredDistances[pair], 1e-14)
        << pair;
    EXPECT_EQ(result.accepted[pair], expected.accepted[pair]) << pair;
  }
  EXPECT_GT(result.timing.kernelNs, 0.0);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, MasksIncidentEdgeEdgeCandidatePairs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  EdgeEdgeFixture fixture;
  appendPoint(fixture, {0.0, 0.0, 0.0});
  appendPoint(fixture, {1.0, 0.0, 0.0});
  appendPoint(fixture, {1.0, 1.0, 0.0});
  const std::vector<std::uint32_t> edgeIndices = {0u, 1u, 1u, 2u};

  cuda::EdgeEdgeCandidateBuildResult result;
  cuda::buildEdgeEdgeContactCandidateMaskCuda(
      fixture.positions, edgeIndices, 0.05, result);

  ASSERT_EQ(result.pairCount, 4u);
  ASSERT_EQ(result.squaredDistances.size(), 4u);
  ASSERT_EQ(result.accepted.size(), 4u);
  EXPECT_EQ(result.squaredDistances[1], 0.0);
  EXPECT_EQ(result.accepted[1], 0u);
  EXPECT_TRUE(result.acceptedEdgeAIndices.empty());
  EXPECT_TRUE(result.acceptedEdgeBIndices.empty());
}

//==============================================================================
TEST(ContactCandidateFilterCuda, MatchesCpuSweptPointTriangleCandidateMask)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const SweptFixture fixture = makeSweptFixture();
  const std::vector<std::uint32_t> pointIndices = {3u};
  const SweptCandidateMaskExpectation expected
      = cpuSweptPointTriangleCandidateMask(fixture, pointIndices, 0.05);

  cuda::SweptPointTriangleCandidateBuildResult result;
  cuda::buildSweptPointTriangleContactCandidateMaskCuda(
      fixture.startPositions,
      fixture.endPositions,
      pointIndices,
      fixture.triangles,
      0.05,
      result);

  ASSERT_EQ(
      result.endpointSquaredDistances.size(),
      expected.endpointSquaredDistances.size());
  ASSERT_EQ(result.accepted.size(), expected.accepted.size());
  EXPECT_EQ(result.pointCount, pointIndices.size());
  EXPECT_EQ(result.triangleCount, fixture.triangles.size() / 3u);
  EXPECT_EQ(result.pairCount, expected.endpointSquaredDistances.size());
  EXPECT_EQ(result.acceptedCount, expected.acceptedCount);
  EXPECT_EQ(result.acceptedPointIndices, expected.acceptedPointIndices);
  EXPECT_EQ(result.acceptedTriangleIndices, expected.acceptedTriangleIndices);

  for (std::size_t pair = 0; pair < expected.endpointSquaredDistances.size();
       ++pair) {
    EXPECT_NEAR(
        result.endpointSquaredDistances[pair],
        expected.endpointSquaredDistances[pair],
        1e-14)
        << pair;
    EXPECT_EQ(result.accepted[pair], expected.accepted[pair]) << pair;
  }
  EXPECT_GT(result.timing.kernelNs, 0.0);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, MasksIncidentSweptPointTriangleCandidatePairs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const SweptFixture fixture = makeSweptFixture();
  const std::vector<std::uint32_t> pointIndices = {0u};

  cuda::SweptPointTriangleCandidateBuildResult result;
  cuda::buildSweptPointTriangleContactCandidateMaskCuda(
      fixture.startPositions,
      fixture.endPositions,
      pointIndices,
      fixture.triangles,
      0.05,
      result);

  ASSERT_EQ(result.pairCount, 1u);
  ASSERT_EQ(result.endpointSquaredDistances.size(), 1u);
  ASSERT_EQ(result.accepted.size(), 1u);
  EXPECT_EQ(result.endpointSquaredDistances[0], 0.0);
  EXPECT_EQ(result.accepted[0], 0u);
  EXPECT_TRUE(result.acceptedPointIndices.empty());
  EXPECT_TRUE(result.acceptedTriangleIndices.empty());
}

//==============================================================================
TEST(ContactCandidateFilterCuda, MatchesCpuSweptEdgeEdgeCandidateMask)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const SweptFixture fixture = makeSweptFixture();
  const SweptEdgeEdgeCandidateMaskExpectation expected
      = cpuSweptEdgeEdgeCandidateMask(fixture, fixture.edges, 0.05);

  cuda::SweptEdgeEdgeCandidateBuildResult result;
  cuda::buildSweptEdgeEdgeContactCandidateMaskCuda(
      fixture.startPositions,
      fixture.endPositions,
      fixture.edges,
      0.05,
      result);

  ASSERT_EQ(
      result.endpointSquaredDistances.size(),
      expected.endpointSquaredDistances.size());
  ASSERT_EQ(result.accepted.size(), expected.accepted.size());
  EXPECT_EQ(result.edgeCount, fixture.edges.size() / 2u);
  EXPECT_EQ(result.pairCount, expected.endpointSquaredDistances.size());
  EXPECT_EQ(result.acceptedCount, expected.acceptedCount);
  EXPECT_EQ(result.acceptedEdgeAIndices, expected.acceptedEdgeAIndices);
  EXPECT_EQ(result.acceptedEdgeBIndices, expected.acceptedEdgeBIndices);

  for (std::size_t pair = 0; pair < expected.endpointSquaredDistances.size();
       ++pair) {
    EXPECT_NEAR(
        result.endpointSquaredDistances[pair],
        expected.endpointSquaredDistances[pair],
        1e-14)
        << pair;
    EXPECT_EQ(result.accepted[pair], expected.accepted[pair]) << pair;
  }
  EXPECT_GT(result.timing.kernelNs, 0.0);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, KeepsSmallValidTriangleNondegenerate)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  Fixture fixture;
  appendPoint(fixture, {0.0, 0.0, 0.0});
  appendPoint(fixture, {1e-8, 0.0, 0.0});
  appendPoint(fixture, {0.0, 1e-8, 0.0});
  appendPoint(fixture, {2.5e-9, 2.5e-9, 0.0});
  fixture.triangles.insert(fixture.triangles.end(), {0u, 1u, 2u});
  fixture.stencils.push_back({3u, 0u});

  const std::vector<double> expectedDistances = cpuSquaredDistances(fixture);

  cuda::PointTriangleCandidateFilterResult result;
  cuda::filterPointTriangleContactStencilsCuda(
      fixture.positions, fixture.triangles, fixture.stencils, 1e-10, result);

  ASSERT_EQ(result.squaredDistances.size(), expectedDistances.size());
  ASSERT_EQ(result.accepted.size(), expectedDistances.size());
  EXPECT_EQ(result.acceptedCount, 1u);
  EXPECT_NEAR(result.squaredDistances[0], expectedDistances[0], 1e-30);
  EXPECT_EQ(result.accepted[0], 1u);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, MatchesCpuEdgeEdgeStencilFilter)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const EdgeEdgeFixture fixture = makeEdgeEdgeFixture();
  const std::vector<double> expectedDistances = cpuSquaredDistances(fixture);

  cuda::EdgeEdgeCandidateFilterResult result;
  cuda::filterEdgeEdgeContactStencilsCuda(
      fixture.positions, fixture.stencils, 0.05, result);

  ASSERT_EQ(result.squaredDistances.size(), expectedDistances.size());
  ASSERT_EQ(result.accepted.size(), expectedDistances.size());
  ASSERT_EQ(result.acceptedCount, 1u);
  EXPECT_NEAR(result.squaredDistances[0], expectedDistances[0], 1e-14);
  EXPECT_NEAR(result.squaredDistances[1], expectedDistances[1], 1e-14);
  EXPECT_EQ(result.accepted[0], 1u);
  EXPECT_EQ(result.accepted[1], 0u);
  EXPECT_GT(result.timing.kernelNs, 0.0);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, RejectsInvalidStencil)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  Fixture fixture = makeFixture();
  fixture.stencils.push_back({99u, 0u});

  cuda::PointTriangleCandidateFilterResult result;
  EXPECT_THROW(
      cuda::filterPointTriangleContactStencilsCuda(
          fixture.positions, fixture.triangles, fixture.stencils, 0.05, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, RejectsInvalidCandidateMaskInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  Fixture fixture = makeFixture();
  const std::vector<std::uint32_t> pointIndices = {99u};

  cuda::PointTriangleCandidateBuildResult result;
  EXPECT_THROW(
      cuda::buildPointTriangleContactCandidateMaskCuda(
          fixture.positions, pointIndices, fixture.triangles, 0.05, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, RejectsInvalidEdgeEdgeStencil)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  EdgeEdgeFixture fixture = makeEdgeEdgeFixture();
  fixture.stencils.push_back({0u, 1u, 99u, 3u});

  cuda::EdgeEdgeCandidateFilterResult result;
  EXPECT_THROW(
      cuda::filterEdgeEdgeContactStencilsCuda(
          fixture.positions, fixture.stencils, 0.05, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, RejectsInvalidEdgeEdgeCandidateMaskInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  EdgeEdgeFixture fixture = makeEdgeEdgeFixture();
  const std::vector<std::uint32_t> edgeIndices = {0u, 1u, 99u, 3u};

  cuda::EdgeEdgeCandidateBuildResult result;
  EXPECT_THROW(
      cuda::buildEdgeEdgeContactCandidateMaskCuda(
          fixture.positions, edgeIndices, 0.05, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, RejectsInvalidSweptCandidateMaskInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  SweptFixture fixture = makeSweptFixture();
  fixture.endPositions.pop_back();

  cuda::SweptPointTriangleCandidateBuildResult pointTriangleResult;
  EXPECT_THROW(
      cuda::buildSweptPointTriangleContactCandidateMaskCuda(
          fixture.startPositions,
          fixture.endPositions,
          std::vector<std::uint32_t>{3u},
          fixture.triangles,
          0.05,
          pointTriangleResult),
      dart::simulation::InvalidArgumentException);

  cuda::SweptEdgeEdgeCandidateBuildResult edgeEdgeResult;
  EXPECT_THROW(
      cuda::buildSweptEdgeEdgeContactCandidateMaskCuda(
          fixture.startPositions,
          fixture.endPositions,
          fixture.edges,
          0.05,
          edgeEdgeResult),
      dart::simulation::InvalidArgumentException);
}
