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

#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/detail/deformable_contact/candidate_set.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/contact_candidate_filter_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>
#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;
namespace dc = dart::simulation::detail::deformable_contact;
namespace sx = dart::simulation;

namespace {

constexpr double kActivationDistance = 0.05;
constexpr double kSceneRuntimeCandidateTimeStep = 1.0;

struct CandidateFixture
{
  std::vector<double> positions;
  std::vector<std::uint32_t> triangles;
  std::vector<cuda::PointTriangleContactStencil> stencils;
  std::vector<double> cpuSquaredDistances;
  std::vector<std::uint8_t> cpuAccepted;
  std::size_t acceptedCount = 0;
};

struct EdgeEdgeCandidateFixture
{
  std::vector<double> positions;
  std::vector<cuda::EdgeEdgeContactStencil> stencils;
  std::vector<double> cpuSquaredDistances;
  std::vector<std::uint8_t> cpuAccepted;
  std::size_t acceptedCount = 0;
};

struct EdgeEdgeCandidateMaskFixture
{
  std::vector<double> positions;
  std::vector<std::uint32_t> edgeIndices;
  std::vector<double> cpuSquaredDistances;
  std::vector<std::uint8_t> cpuAccepted;
  std::size_t edgeCount = 0;
  std::size_t pairCount = 0;
  std::size_t acceptedCount = 0;
};

struct CandidateMaskFixture
{
  std::vector<double> positions;
  std::vector<std::uint32_t> pointIndices;
  std::vector<std::uint32_t> triangles;
  std::vector<double> cpuSquaredDistances;
  std::vector<std::uint8_t> cpuAccepted;
  std::size_t pointCount = 0;
  std::size_t triangleCount = 0;
  std::size_t pairCount = 0;
  std::size_t acceptedCount = 0;
};

struct SweptCandidateMaskFixture
{
  std::vector<double> startPositions;
  std::vector<double> endPositions;
  std::vector<std::uint32_t> pointIndices;
  std::vector<std::uint32_t> triangles;
  std::vector<double> cpuEndpointSquaredDistances;
  std::vector<std::uint8_t> cpuAccepted;
  std::size_t pointCount = 0;
  std::size_t triangleCount = 0;
  std::size_t pairCount = 0;
  std::size_t acceptedCount = 0;
};

struct SweptEdgeEdgeCandidateMaskFixture
{
  std::vector<double> startPositions;
  std::vector<double> endPositions;
  std::vector<std::uint32_t> edgeIndices;
  std::vector<double> cpuEndpointSquaredDistances;
  std::vector<std::uint8_t> cpuAccepted;
  std::size_t edgeCount = 0;
  std::size_t pairCount = 0;
  std::size_t acceptedCount = 0;
};

struct SweptCandidateSweepFixture
{
  std::vector<double> startPositions;
  std::vector<double> endPositions;
  std::vector<std::uint32_t> pointIndices;
  std::vector<std::uint32_t> triangles;
  std::vector<std::uint32_t> cpuAcceptedPointIndices;
  std::vector<std::uint32_t> cpuAcceptedTriangleIndices;
  std::vector<double> cpuAcceptedEndpointSquaredDistances;
  std::size_t pointCount = 0;
  std::size_t triangleCount = 0;
  std::size_t pairCapacity = 0;
  std::size_t acceptedCount = 0;
  std::size_t sceneBodyCount = 0;
};

struct SweptEdgeEdgeSweepFixture
{
  std::vector<double> startPositions;
  std::vector<double> endPositions;
  std::vector<std::uint32_t> edgeIndices;
  std::vector<std::uint32_t> cpuAcceptedEdgeAIndices;
  std::vector<std::uint32_t> cpuAcceptedEdgeBIndices;
  std::vector<double> cpuAcceptedEndpointSquaredDistances;
  std::size_t edgeCount = 0;
  std::size_t pairCapacity = 0;
  std::size_t acceptedCount = 0;
  std::size_t sceneBodyCount = 0;
};

struct CombinedSweptCandidateSweepFixture
{
  SweptCandidateSweepFixture pointTriangle;
  SweptEdgeEdgeSweepFixture edgeEdge;
};

struct WorldStepSurfaceContactCounters
{
  std::size_t lineSearchTrials = 0;
  std::size_t candidateBuilds = 0;
  std::size_t candidatePairCapacity = 0;
  std::size_t rejectedPairs = 0;
  std::size_t pointTriangleCandidates = 0;
  std::size_t edgeEdgeCandidates = 0;
  std::size_t ccdPointTriangleChecks = 0;
  std::size_t ccdEdgeEdgeChecks = 0;
  std::size_t ccdHits = 0;
  std::size_t ccdLimitedSteps = 0;
  std::size_t ccdZeroStepCount = 0;
  std::size_t interBodyCandidateBuilds = 0;
  std::size_t interBodyCandidatePairCapacity = 0;
  std::size_t interBodyCandidateRejectedPairs = 0;
  std::size_t interBodyPointTriangleCandidates = 0;
  std::size_t interBodyEdgeEdgeCandidates = 0;
};

struct RuntimeSweepCandidateBufferFixture
{
  std::vector<Eigen::Vector3d> start;
  std::vector<Eigen::Vector3d> end;
  std::vector<dart::simulation::DeformableSurfaceTriangle> triangles;
  std::vector<double> startPositions;
  std::vector<double> endPositions;
  std::vector<std::uint32_t> triangleIndices;
  std::vector<std::uint32_t> edgeIndices;
  std::vector<std::uint32_t> candidatePointIndices;
  std::vector<std::uint32_t> candidateTriangleIndices;
  std::vector<double> pointTriangleEndpointSquaredDistances;
  std::vector<std::uint32_t> candidateEdgeAIndices;
  std::vector<std::uint32_t> candidateEdgeBIndices;
  std::vector<double> edgeEdgeEndpointSquaredDistances;
  std::size_t sceneBodyCount = 0;
  WorldStepSurfaceContactCounters worldStepSurfaceContact;
};

struct CombinedSceneSweptCandidateSweepFixture
{
  CombinedSweptCandidateSweepFixture sweep;
  WorldStepSurfaceContactCounters worldStepSurfaceContact;
};

void appendPoint(
    CandidateFixture& fixture, const double x, const double y, const double z)
{
  fixture.positions.push_back(x);
  fixture.positions.push_back(y);
  fixture.positions.push_back(z);
}

void appendPoint(
    EdgeEdgeCandidateFixture& fixture,
    const double x,
    const double y,
    const double z)
{
  fixture.positions.push_back(x);
  fixture.positions.push_back(y);
  fixture.positions.push_back(z);
}

void appendPoint(
    EdgeEdgeCandidateMaskFixture& fixture,
    const double x,
    const double y,
    const double z)
{
  fixture.positions.push_back(x);
  fixture.positions.push_back(y);
  fixture.positions.push_back(z);
}

void appendPoint(
    CandidateMaskFixture& fixture,
    const double x,
    const double y,
    const double z)
{
  fixture.positions.push_back(x);
  fixture.positions.push_back(y);
  fixture.positions.push_back(z);
}

void appendPoint(
    SweptCandidateMaskFixture& fixture,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)
{
  fixture.startPositions.push_back(start.x());
  fixture.startPositions.push_back(start.y());
  fixture.startPositions.push_back(start.z());
  fixture.endPositions.push_back(end.x());
  fixture.endPositions.push_back(end.y());
  fixture.endPositions.push_back(end.z());
}

void appendPoint(
    SweptEdgeEdgeCandidateMaskFixture& fixture,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)
{
  fixture.startPositions.push_back(start.x());
  fixture.startPositions.push_back(start.y());
  fixture.startPositions.push_back(start.z());
  fixture.endPositions.push_back(end.x());
  fixture.endPositions.push_back(end.y());
  fixture.endPositions.push_back(end.z());
}

void appendPoint(
    RuntimeSweepCandidateBufferFixture& fixture,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)
{
  fixture.start.push_back(start);
  fixture.end.push_back(end);
  fixture.startPositions.push_back(start.x());
  fixture.startPositions.push_back(start.y());
  fixture.startPositions.push_back(start.z());
  fixture.endPositions.push_back(end.x());
  fixture.endPositions.push_back(end.y());
  fixture.endPositions.push_back(end.z());
}

Eigen::Vector3d pointAt(
    const std::vector<double>& positions, const std::size_t point)
{
  const std::size_t base = 3u * point;
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

bool edgesShareVertex(
    const std::uint32_t a0,
    const std::uint32_t a1,
    const std::uint32_t b0,
    const std::uint32_t b1)
{
  return a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1;
}

CandidateFixture makeCandidateFixture(const int stencilCount)
{
  CandidateFixture fixture;
  fixture.positions.reserve(static_cast<std::size_t>(4 * stencilCount * 3));
  fixture.triangles.reserve(static_cast<std::size_t>(3 * stencilCount));
  fixture.stencils.reserve(static_cast<std::size_t>(stencilCount));
  fixture.cpuSquaredDistances.reserve(static_cast<std::size_t>(stencilCount));
  fixture.cpuAccepted.reserve(static_cast<std::size_t>(stencilCount));

  for (int i = 0; i < stencilCount; ++i) {
    const double x = static_cast<double>(i % 512) * 2.0;
    const double y = static_cast<double>(i / 512) * 2.0;
    const std::uint32_t base
        = static_cast<std::uint32_t>(fixture.positions.size() / 3u);
    appendPoint(fixture, x, y, 0.0);
    appendPoint(fixture, x + 1.0, y, 0.0);
    appendPoint(fixture, x, y + 1.0, 0.0);

    const bool accepted = (i % 4) != 0;
    const double height = accepted ? 0.02 : 0.08;
    appendPoint(fixture, x + 0.25, y + 0.25, height);

    fixture.triangles.insert(
        fixture.triangles.end(), {base, base + 1u, base + 2u});
    fixture.stencils.push_back(
        {base + 3u, static_cast<std::uint32_t>(fixture.stencils.size())});
  }

  for (const auto& stencil : fixture.stencils) {
    const std::size_t tri = 3u * static_cast<std::size_t>(stencil.triangle);
    const auto distance = dc::pointTriangleSquaredDistance(
        pointAt(fixture.positions, stencil.point),
        pointAt(fixture.positions, fixture.triangles[tri]),
        pointAt(fixture.positions, fixture.triangles[tri + 1u]),
        pointAt(fixture.positions, fixture.triangles[tri + 2u]));
    fixture.cpuSquaredDistances.push_back(distance.squaredDistance);
    const bool accepted = dc::detail::withinActivationDistance(
        distance.squaredDistance, kActivationDistance);
    fixture.cpuAccepted.push_back(accepted ? 1u : 0u);
    if (accepted) {
      ++fixture.acceptedCount;
    }
  }

  return fixture;
}

EdgeEdgeCandidateFixture makeEdgeEdgeCandidateFixture(const int stencilCount)
{
  EdgeEdgeCandidateFixture fixture;
  fixture.positions.reserve(static_cast<std::size_t>(4 * stencilCount * 3));
  fixture.stencils.reserve(static_cast<std::size_t>(stencilCount));
  fixture.cpuSquaredDistances.reserve(static_cast<std::size_t>(stencilCount));
  fixture.cpuAccepted.reserve(static_cast<std::size_t>(stencilCount));

  for (int i = 0; i < stencilCount; ++i) {
    const double x = static_cast<double>(i % 512) * 2.0;
    const double y = static_cast<double>(i / 512) * 2.0;
    const std::uint32_t base
        = static_cast<std::uint32_t>(fixture.positions.size() / 3u);
    const bool accepted = (i % 4) != 0;
    const double offset = accepted ? 0.02 : 0.08;

    appendPoint(fixture, x, y, 0.0);
    appendPoint(fixture, x + 1.0, y, 0.0);
    appendPoint(fixture, x + 0.25, y + offset, -0.5);
    appendPoint(fixture, x + 0.25, y + offset, 0.5);
    fixture.stencils.push_back({base, base + 1u, base + 2u, base + 3u});
  }

  for (const auto& stencil : fixture.stencils) {
    const auto distance = dc::edgeEdgeSquaredDistance(
        pointAt(fixture.positions, stencil.edgeAStart),
        pointAt(fixture.positions, stencil.edgeAEnd),
        pointAt(fixture.positions, stencil.edgeBStart),
        pointAt(fixture.positions, stencil.edgeBEnd));
    fixture.cpuSquaredDistances.push_back(distance.squaredDistance);
    const bool accepted = dc::detail::withinActivationDistance(
        distance.squaredDistance, kActivationDistance);
    fixture.cpuAccepted.push_back(accepted ? 1u : 0u);
    if (accepted) {
      ++fixture.acceptedCount;
    }
  }

  return fixture;
}

EdgeEdgeCandidateMaskFixture makeEdgeEdgeCandidateMaskFixture(
    const int pairCount)
{
  const int side = std::max(2, static_cast<int>(std::sqrt(pairCount)));
  EdgeEdgeCandidateMaskFixture fixture;
  fixture.edgeCount = static_cast<std::size_t>(side);
  fixture.pairCount = fixture.edgeCount * fixture.edgeCount;
  fixture.positions.reserve(2u * fixture.edgeCount * 3u);
  fixture.edgeIndices.reserve(2u * fixture.edgeCount);
  fixture.cpuSquaredDistances.reserve(fixture.pairCount);
  fixture.cpuAccepted.reserve(fixture.pairCount);

  for (int i = 0; i < side; ++i) {
    const int group = i / 2;
    const double x = static_cast<double>(group % 64) * 2.0;
    const double y = static_cast<double>(group / 64) * 2.0;
    const std::uint32_t base
        = static_cast<std::uint32_t>(fixture.positions.size() / 3u);
    if ((i % 2) == 0) {
      appendPoint(fixture, x, y, 0.0);
      appendPoint(fixture, x + 1.0, y, 0.0);
    } else {
      const bool accepted = (group % 4) != 0;
      const double offset = accepted ? 0.02 : 0.08;
      appendPoint(fixture, x + 0.25, y + offset, -0.5);
      appendPoint(fixture, x + 0.25, y + offset, 0.5);
    }
    fixture.edgeIndices.insert(fixture.edgeIndices.end(), {base, base + 1u});
  }

  for (std::size_t edgeA = 0; edgeA < fixture.edgeCount; ++edgeA) {
    for (std::size_t edgeB = 0; edgeB < fixture.edgeCount; ++edgeB) {
      double squaredDistance = 0.0;
      bool accepted = false;
      const std::uint32_t a0 = fixture.edgeIndices[2u * edgeA];
      const std::uint32_t a1 = fixture.edgeIndices[2u * edgeA + 1u];
      const std::uint32_t b0 = fixture.edgeIndices[2u * edgeB];
      const std::uint32_t b1 = fixture.edgeIndices[2u * edgeB + 1u];
      if (edgeA < edgeB && !edgesShareVertex(a0, a1, b0, b1)) {
        const auto distance = dc::edgeEdgeSquaredDistance(
            pointAt(fixture.positions, a0),
            pointAt(fixture.positions, a1),
            pointAt(fixture.positions, b0),
            pointAt(fixture.positions, b1));
        squaredDistance = distance.squaredDistance;
        accepted = dc::detail::withinActivationDistance(
            squaredDistance, kActivationDistance);
      }
      fixture.cpuSquaredDistances.push_back(squaredDistance);
      fixture.cpuAccepted.push_back(accepted ? 1u : 0u);
      if (accepted) {
        ++fixture.acceptedCount;
      }
    }
  }

  return fixture;
}

CandidateMaskFixture makeCandidateMaskFixture(const int pairCount)
{
  const int side = std::max(1, static_cast<int>(std::sqrt(pairCount)));
  CandidateMaskFixture fixture;
  fixture.pointCount = static_cast<std::size_t>(side);
  fixture.triangleCount = static_cast<std::size_t>(side);
  fixture.pairCount = fixture.pointCount * fixture.triangleCount;
  fixture.positions.reserve(4u * fixture.triangleCount * 3u);
  fixture.pointIndices.reserve(fixture.pointCount);
  fixture.triangles.reserve(3u * fixture.triangleCount);
  fixture.cpuSquaredDistances.reserve(fixture.pairCount);
  fixture.cpuAccepted.reserve(fixture.pairCount);

  for (int i = 0; i < side; ++i) {
    const double x = static_cast<double>(i % 64) * 2.0;
    const double y = static_cast<double>(i / 64) * 2.0;
    const std::uint32_t base
        = static_cast<std::uint32_t>(fixture.positions.size() / 3u);
    appendPoint(fixture, x, y, 0.0);
    appendPoint(fixture, x + 1.0, y, 0.0);
    appendPoint(fixture, x, y + 1.0, 0.0);
    const bool accepted = (i % 4) != 0;
    appendPoint(fixture, x + 0.25, y + 0.25, accepted ? 0.02 : 0.08);
    fixture.triangles.insert(
        fixture.triangles.end(), {base, base + 1u, base + 2u});
    fixture.pointIndices.push_back(base + 3u);
  }

  for (const std::uint32_t point : fixture.pointIndices) {
    for (std::size_t triangle = 0; triangle < fixture.triangleCount;
         ++triangle) {
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
            squaredDistance, kActivationDistance);
      }
      fixture.cpuSquaredDistances.push_back(squaredDistance);
      fixture.cpuAccepted.push_back(accepted ? 1u : 0u);
      if (accepted) {
        ++fixture.acceptedCount;
      }
    }
  }

  return fixture;
}

SweptCandidateMaskFixture makeSweptCandidateMaskFixture(const int pairCount)
{
  const int side = std::max(1, static_cast<int>(std::sqrt(pairCount)));
  SweptCandidateMaskFixture fixture;
  fixture.pointCount = static_cast<std::size_t>(side);
  fixture.triangleCount = static_cast<std::size_t>(side);
  fixture.pairCount = fixture.pointCount * fixture.triangleCount;
  fixture.startPositions.reserve(4u * fixture.triangleCount * 3u);
  fixture.endPositions.reserve(4u * fixture.triangleCount * 3u);
  fixture.pointIndices.reserve(fixture.pointCount);
  fixture.triangles.reserve(3u * fixture.triangleCount);
  fixture.cpuEndpointSquaredDistances.reserve(fixture.pairCount);
  fixture.cpuAccepted.reserve(fixture.pairCount);

  for (int i = 0; i < side; ++i) {
    const double x = static_cast<double>(i % 64) * 2.0;
    const double y = static_cast<double>(i / 64) * 2.0;
    const std::uint32_t base
        = static_cast<std::uint32_t>(fixture.startPositions.size() / 3u);
    appendPoint(fixture, {x, y, 0.0}, {x, y, 0.0});
    appendPoint(fixture, {x + 1.0, y, 0.0}, {x + 1.0, y, 0.0});
    appendPoint(fixture, {x, y + 1.0, 0.0}, {x, y + 1.0, 0.0});
    appendPoint(
        fixture, {x + 0.25, y + 0.25, 0.20}, {x + 0.25, y + 0.25, -0.20});
    fixture.triangles.insert(
        fixture.triangles.end(), {base, base + 1u, base + 2u});
    fixture.pointIndices.push_back(base + 3u);
  }

  const double margin = 0.5 * kActivationDistance;
  for (const std::uint32_t point : fixture.pointIndices) {
    for (std::size_t triangle = 0; triangle < fixture.triangleCount;
         ++triangle) {
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
      fixture.cpuEndpointSquaredDistances.push_back(endpointSquaredDistance);
      fixture.cpuAccepted.push_back(accepted ? 1u : 0u);
      if (accepted) {
        ++fixture.acceptedCount;
      }
    }
  }

  return fixture;
}

SweptEdgeEdgeCandidateMaskFixture makeSweptEdgeEdgeCandidateMaskFixture(
    const int pairCount)
{
  const int side = std::max(2, static_cast<int>(std::sqrt(pairCount)));
  SweptEdgeEdgeCandidateMaskFixture fixture;
  fixture.edgeCount = static_cast<std::size_t>(side);
  fixture.pairCount = fixture.edgeCount * fixture.edgeCount;
  fixture.startPositions.reserve(2u * fixture.edgeCount * 3u);
  fixture.endPositions.reserve(2u * fixture.edgeCount * 3u);
  fixture.edgeIndices.reserve(2u * fixture.edgeCount);
  fixture.cpuEndpointSquaredDistances.reserve(fixture.pairCount);
  fixture.cpuAccepted.reserve(fixture.pairCount);

  for (int i = 0; i < side; ++i) {
    const int group = i / 2;
    const double x = static_cast<double>(group % 64) * 2.0;
    const double y = static_cast<double>(group / 64) * 2.0;
    const std::uint32_t base
        = static_cast<std::uint32_t>(fixture.startPositions.size() / 3u);
    if ((i % 2) == 0) {
      appendPoint(fixture, {x, y, 0.0}, {x, y, 0.0});
      appendPoint(fixture, {x + 1.0, y, 0.0}, {x + 1.0, y, 0.0});
    } else {
      appendPoint(
          fixture, {x + 0.25, y + 0.20, -0.5}, {x + 0.25, y - 0.20, -0.5});
      appendPoint(
          fixture, {x + 0.25, y + 0.20, 0.5}, {x + 0.25, y - 0.20, 0.5});
    }
    fixture.edgeIndices.insert(fixture.edgeIndices.end(), {base, base + 1u});
  }

  const double margin = 0.5 * kActivationDistance;
  for (std::size_t edgeA = 0; edgeA < fixture.edgeCount; ++edgeA) {
    for (std::size_t edgeB = 0; edgeB < fixture.edgeCount; ++edgeB) {
      double endpointSquaredDistance = 0.0;
      bool accepted = false;
      const std::uint32_t a0 = fixture.edgeIndices[2u * edgeA];
      const std::uint32_t a1 = fixture.edgeIndices[2u * edgeA + 1u];
      const std::uint32_t b0 = fixture.edgeIndices[2u * edgeB];
      const std::uint32_t b1 = fixture.edgeIndices[2u * edgeB + 1u];
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
      fixture.cpuEndpointSquaredDistances.push_back(endpointSquaredDistance);
      fixture.cpuAccepted.push_back(accepted ? 1u : 0u);
      if (accepted) {
        ++fixture.acceptedCount;
      }
    }
  }

  return fixture;
}

void appendSweptPointTriangleSweepCandidates(
    SweptCandidateSweepFixture& fixture)
{
  std::vector<dc::detail::SweepItem> pointItems;
  std::vector<dc::detail::SweepItem> triangleItems;
  pointItems.reserve(fixture.pointIndices.size());
  triangleItems.reserve(fixture.triangleCount);

  const double margin = 0.5 * kActivationDistance;
  for (const std::uint32_t point : fixture.pointIndices) {
    pointItems.push_back(
        dc::detail::SweepItem{
            point,
            dc::detail::makeSweptPointAabb(
                pointAt(fixture.startPositions, point),
                pointAt(fixture.endPositions, point),
                margin)});
  }
  for (std::size_t triangle = 0; triangle < fixture.triangleCount; ++triangle) {
    const std::size_t tri = 3u * triangle;
    triangleItems.push_back(
        dc::detail::SweepItem{
            triangle,
            dc::detail::makeSweptTriangleAabb(
                pointAt(fixture.startPositions, fixture.triangles[tri]),
                pointAt(fixture.endPositions, fixture.triangles[tri]),
                pointAt(fixture.startPositions, fixture.triangles[tri + 1u]),
                pointAt(fixture.endPositions, fixture.triangles[tri + 1u]),
                pointAt(fixture.startPositions, fixture.triangles[tri + 2u]),
                pointAt(fixture.endPositions, fixture.triangles[tri + 2u]),
                margin)});
  }

  dc::detail::visitSweepPairs(
      pointItems,
      triangleItems,
      [&](const std::size_t point, const std::size_t triangle) {
        if (isIncidentPointTriangle(
                static_cast<std::uint32_t>(point),
                fixture.triangles,
                triangle)) {
          return;
        }
        const std::size_t tri = 3u * triangle;
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
        fixture.cpuAcceptedPointIndices.push_back(
            static_cast<std::uint32_t>(point));
        fixture.cpuAcceptedTriangleIndices.push_back(
            static_cast<std::uint32_t>(triangle));
        fixture.cpuAcceptedEndpointSquaredDistances.push_back(
            std::min(
                startDistance.squaredDistance, endDistance.squaredDistance));
      });

  fixture.acceptedCount = fixture.cpuAcceptedPointIndices.size();
}

SweptCandidateSweepFixture makeSweptCandidateSweepFixture(const int pairCount)
{
  const SweptCandidateMaskFixture maskFixture
      = makeSweptCandidateMaskFixture(pairCount);
  SweptCandidateSweepFixture fixture;
  fixture.startPositions = maskFixture.startPositions;
  fixture.endPositions = maskFixture.endPositions;
  fixture.pointIndices = maskFixture.pointIndices;
  fixture.triangles = maskFixture.triangles;
  fixture.pointCount = maskFixture.pointCount;
  fixture.triangleCount = maskFixture.triangleCount;
  fixture.pairCapacity = maskFixture.pairCount;
  appendSweptPointTriangleSweepCandidates(fixture);
  return fixture;
}

void appendSweptEdgeEdgeSweepCandidates(SweptEdgeEdgeSweepFixture& fixture)
{
  std::vector<dc::detail::SweepItem> edgeItems;
  edgeItems.reserve(fixture.edgeCount);

  const double margin = 0.5 * kActivationDistance;
  for (std::size_t edge = 0; edge < fixture.edgeCount; ++edge) {
    const std::uint32_t a0 = fixture.edgeIndices[2u * edge];
    const std::uint32_t a1 = fixture.edgeIndices[2u * edge + 1u];
    edgeItems.push_back(
        dc::detail::SweepItem{
            edge,
            dc::detail::makeSweptSegmentAabb(
                pointAt(fixture.startPositions, a0),
                pointAt(fixture.endPositions, a0),
                pointAt(fixture.startPositions, a1),
                pointAt(fixture.endPositions, a1),
                margin)});
  }

  dc::detail::visitSelfSweepPairs(
      edgeItems, [&](const std::size_t edgeA, const std::size_t edgeB) {
        const std::uint32_t a0 = fixture.edgeIndices[2u * edgeA];
        const std::uint32_t a1 = fixture.edgeIndices[2u * edgeA + 1u];
        const std::uint32_t b0 = fixture.edgeIndices[2u * edgeB];
        const std::uint32_t b1 = fixture.edgeIndices[2u * edgeB + 1u];
        if (edgesShareVertex(a0, a1, b0, b1)) {
          return;
        }
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
        fixture.cpuAcceptedEdgeAIndices.push_back(
            static_cast<std::uint32_t>(edgeA));
        fixture.cpuAcceptedEdgeBIndices.push_back(
            static_cast<std::uint32_t>(edgeB));
        fixture.cpuAcceptedEndpointSquaredDistances.push_back(
            std::min(
                startDistance.squaredDistance, endDistance.squaredDistance));
      });

  fixture.acceptedCount = fixture.cpuAcceptedEdgeAIndices.size();
}

SweptEdgeEdgeSweepFixture makeSweptEdgeEdgeSweepFixture(const int pairCount)
{
  const SweptEdgeEdgeCandidateMaskFixture maskFixture
      = makeSweptEdgeEdgeCandidateMaskFixture(pairCount);
  SweptEdgeEdgeSweepFixture fixture;
  fixture.startPositions = maskFixture.startPositions;
  fixture.endPositions = maskFixture.endPositions;
  fixture.edgeIndices = maskFixture.edgeIndices;
  fixture.edgeCount = maskFixture.edgeCount;
  fixture.pairCapacity = maskFixture.pairCount;
  appendSweptEdgeEdgeSweepCandidates(fixture);
  return fixture;
}

void populateRuntimeSweepCandidateBuffers(
    RuntimeSweepCandidateBufferFixture& fixture)
{
  fixture.triangleIndices.clear();
  fixture.triangleIndices.reserve(3u * fixture.triangles.size());
  for (const auto& triangle : fixture.triangles) {
    fixture.triangleIndices.push_back(
        static_cast<std::uint32_t>(triangle.nodeA));
    fixture.triangleIndices.push_back(
        static_cast<std::uint32_t>(triangle.nodeB));
    fixture.triangleIndices.push_back(
        static_cast<std::uint32_t>(triangle.nodeC));
  }

  dc::ContactCandidateOptions options;
  options.activationDistance = kActivationDistance;
  const dc::ContactCandidateSet candidates
      = dc::buildMotionAwareContactCandidatesSweep(
          fixture.start, fixture.end, fixture.triangles, options);

  fixture.edgeIndices.clear();
  fixture.edgeIndices.reserve(2u * candidates.surfaceEdges.size());
  for (const auto& edge : candidates.surfaceEdges) {
    fixture.edgeIndices.push_back(static_cast<std::uint32_t>(edge.nodeA));
    fixture.edgeIndices.push_back(static_cast<std::uint32_t>(edge.nodeB));
  }
  fixture.candidatePointIndices.clear();
  fixture.candidateTriangleIndices.clear();
  fixture.pointTriangleEndpointSquaredDistances.clear();
  fixture.candidatePointIndices.reserve(
      candidates.pointTriangleCandidates.size());
  fixture.candidateTriangleIndices.reserve(
      candidates.pointTriangleCandidates.size());
  fixture.pointTriangleEndpointSquaredDistances.reserve(
      candidates.pointTriangleCandidates.size());
  for (const auto& candidate : candidates.pointTriangleCandidates) {
    fixture.candidatePointIndices.push_back(
        static_cast<std::uint32_t>(candidate.point));
    fixture.candidateTriangleIndices.push_back(
        static_cast<std::uint32_t>(candidate.triangle));
    fixture.pointTriangleEndpointSquaredDistances.push_back(
        candidate.squaredDistance);
  }
  fixture.candidateEdgeAIndices.clear();
  fixture.candidateEdgeBIndices.clear();
  fixture.edgeEdgeEndpointSquaredDistances.clear();
  fixture.candidateEdgeAIndices.reserve(candidates.edgeEdgeCandidates.size());
  fixture.candidateEdgeBIndices.reserve(candidates.edgeEdgeCandidates.size());
  fixture.edgeEdgeEndpointSquaredDistances.reserve(
      candidates.edgeEdgeCandidates.size());
  for (const auto& candidate : candidates.edgeEdgeCandidates) {
    fixture.candidateEdgeAIndices.push_back(
        static_cast<std::uint32_t>(candidate.edgeA));
    fixture.candidateEdgeBIndices.push_back(
        static_cast<std::uint32_t>(candidate.edgeB));
    fixture.edgeEdgeEndpointSquaredDistances.push_back(
        candidate.squaredDistance);
  }
}

RuntimeSweepCandidateBufferFixture makeRuntimeSweepCandidateBufferFixture(
    const int pairCount)
{
  const int groupCount = std::max(1, static_cast<int>(std::sqrt(pairCount)));
  RuntimeSweepCandidateBufferFixture fixture;
  fixture.start.reserve(static_cast<std::size_t>(10 * groupCount));
  fixture.end.reserve(static_cast<std::size_t>(10 * groupCount));
  fixture.triangles.reserve(static_cast<std::size_t>(3 * groupCount));

  for (int i = 0; i < groupCount; ++i) {
    const double x = static_cast<double>(i % 64) * 4.0;
    const double y = static_cast<double>(i / 64) * 4.0;
    const std::size_t base = fixture.start.size();

    appendPoint(fixture, {x, y, 0.0}, {x, y, 0.0});
    appendPoint(fixture, {x + 1.0, y, 0.0}, {x + 1.0, y, 0.0});
    appendPoint(fixture, {x, y + 1.0, 0.0}, {x, y + 1.0, 0.0});
    appendPoint(
        fixture, {x + 0.25, y + 0.25, 0.20}, {x + 0.25, y + 0.25, -0.20});
    fixture.triangles.push_back({base, base + 1u, base + 2u});

    appendPoint(fixture, {x + 2.0, y, 0.0}, {x + 2.0, y, 0.0});
    appendPoint(fixture, {x + 3.0, y, 0.0}, {x + 3.0, y, 0.0});
    appendPoint(fixture, {x + 2.0, y + 1.0, 0.0}, {x + 2.0, y + 1.0, 0.0});
    fixture.triangles.push_back({base + 4u, base + 5u, base + 6u});

    appendPoint(
        fixture, {x + 2.25, y + 0.20, -0.5}, {x + 2.25, y - 0.20, -0.5});
    appendPoint(fixture, {x + 2.25, y + 0.20, 0.5}, {x + 2.25, y - 0.20, 0.5});
    appendPoint(fixture, {x + 3.25, y + 1.25, 0.0}, {x + 3.25, y + 1.25, 0.0});
    fixture.triangles.push_back({base + 7u, base + 8u, base + 9u});
  }

  populateRuntimeSweepCandidateBuffers(fixture);

  return fixture;
}

sx::DeformableBodyOptions makeSceneRuntimeCandidateBodyOptions(
    const int pairCount)
{
  const int groupCount = std::max(1, static_cast<int>(std::sqrt(pairCount)));
  sx::DeformableBodyOptions options;
  options.positions.reserve(static_cast<std::size_t>(10 * groupCount));
  options.velocities.reserve(static_cast<std::size_t>(10 * groupCount));
  options.masses.reserve(static_cast<std::size_t>(10 * groupCount));
  options.surfaceTriangles.reserve(static_cast<std::size_t>(3 * groupCount));

  const auto appendSceneNode
      = [&options](const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
          options.positions.push_back(start);
          options.velocities.push_back(
              (end - start) / kSceneRuntimeCandidateTimeStep);
          options.masses.push_back(1.0);
        };

  for (int i = 0; i < groupCount; ++i) {
    const double x = static_cast<double>(i % 64) * 4.0;
    const double y = static_cast<double>(i / 64) * 4.0;
    const std::size_t base = options.positions.size();

    appendSceneNode({x, y, 0.0}, {x, y, 0.0});
    appendSceneNode({x + 1.0, y, 0.0}, {x + 1.0, y, 0.0});
    appendSceneNode({x, y + 1.0, 0.0}, {x, y + 1.0, 0.0});
    appendSceneNode({x + 0.25, y + 0.25, 0.20}, {x + 0.25, y + 0.25, -0.20});
    options.surfaceTriangles.push_back({base, base + 1u, base + 2u});

    appendSceneNode({x + 2.0, y, 0.0}, {x + 2.0, y, 0.0});
    appendSceneNode({x + 3.0, y, 0.0}, {x + 3.0, y, 0.0});
    appendSceneNode({x + 2.0, y + 1.0, 0.0}, {x + 2.0, y + 1.0, 0.0});
    options.surfaceTriangles.push_back({base + 4u, base + 5u, base + 6u});

    appendSceneNode({x + 2.25, y + 0.20, -0.5}, {x + 2.25, y - 0.20, -0.5});
    appendSceneNode({x + 2.25, y + 0.20, 0.5}, {x + 2.25, y - 0.20, 0.5});
    appendSceneNode({x + 3.25, y + 1.25, 0.0}, {x + 3.25, y + 1.25, 0.0});
    options.surfaceTriangles.push_back({base + 7u, base + 8u, base + 9u});
  }

  return options;
}

// Minimal two-body inter-body deformable witness. A moving-point body crosses a
// stationary triangle obstacle within a single ``World::step`` so the built-in
// deformable diagnostics report nonzero inter-body surface-contact candidate
// activity. Both bodies share the same offset/scale, so the relative geometry
// (and therefore the inter-body counters) is offset-invariant; this mirrors the
// CPU scene corpus inter-body witness construction.
sx::DeformableBodyOptions makeInterBodyMovingPointWitnessOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 3.0),
         Eigen::Vector3d(1.0, -1.0, 3.0),
         Eigen::Vector3d(0.0, 1.0, 3.0),
         Eigen::Vector3d(0.0, 0.0, 1.0)};
  for (auto& position : options.positions) {
    position *= 0.04;
  }
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  options.velocities[3] = Eigen::Vector3d(0.0, 0.0, -20.0);
  options.masses.assign(options.positions.size(), 0.02);
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

sx::DeformableBodyOptions makeInterBodyObstacleWitnessOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 0.0),
         Eigen::Vector3d(1.0, -1.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  for (auto& position : options.positions) {
    position *= 0.04;
  }
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  options.masses.assign(options.positions.size(), 0.02);
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

WorldStepSurfaceContactCounters captureInterBodyWitnessCounters()
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeCandidateTimeStep);
  world.addDeformableBody(
      "plan083_inter_body_moving_witness",
      makeInterBodyMovingPointWitnessOptions());
  world.addDeformableBody(
      "plan083_inter_body_obstacle_witness",
      makeInterBodyObstacleWitnessOptions());
  world.step();
  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();

  WorldStepSurfaceContactCounters counters;
  counters.interBodyCandidateBuilds
      = diagnostics.interBodySurfaceContactCandidateBuilds;
  counters.interBodyCandidatePairCapacity
      = diagnostics.interBodySurfaceContactCandidatePairCapacity;
  counters.interBodyCandidateRejectedPairs
      = diagnostics.interBodySurfaceContactCandidateRejectedPairs;
  counters.interBodyPointTriangleCandidates
      = diagnostics.interBodySurfaceContactPointTriangleCandidates;
  counters.interBodyEdgeEdgeCandidates
      = diagnostics.interBodySurfaceContactEdgeEdgeCandidates;
  return counters;
}

RuntimeSweepCandidateBufferFixture makeSceneRuntimeCandidateBufferFixture(
    const int pairCount, const bool captureWorldStepSurfaceContact = false)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeCandidateTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_candidate_buffer",
      makeSceneRuntimeCandidateBodyOptions(pairCount));

  RuntimeSweepCandidateBufferFixture fixture;
  fixture.sceneBodyCount = world.getDeformableBodyCount();
  fixture.start.reserve(body.getNodeCount());
  fixture.end.reserve(body.getNodeCount());
  fixture.triangles.reserve(body.getSurfaceTriangleCount());

  for (std::size_t node = 0; node < body.getNodeCount(); ++node) {
    const Eigen::Vector3d start = body.getPosition(node);
    appendPoint(
        fixture, start, start + world.getTimeStep() * body.getVelocity(node));
  }
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    fixture.triangles.push_back(body.getSurfaceTriangle(triangle));
  }

  populateRuntimeSweepCandidateBuffers(fixture);
  if (captureWorldStepSurfaceContact) {
    world.step();
    const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
    fixture.worldStepSurfaceContact.lineSearchTrials
        = diagnostics.lineSearchTrials;
    fixture.worldStepSurfaceContact.candidateBuilds
        = diagnostics.surfaceContactCandidateBuilds;
    fixture.worldStepSurfaceContact.candidatePairCapacity
        = diagnostics.surfaceContactCandidatePairCapacity;
    fixture.worldStepSurfaceContact.rejectedPairs
        = diagnostics.surfaceContactCandidateRejectedPairs;
    fixture.worldStepSurfaceContact.pointTriangleCandidates
        = diagnostics.surfaceContactPointTriangleCandidates;
    fixture.worldStepSurfaceContact.edgeEdgeCandidates
        = diagnostics.surfaceContactEdgeEdgeCandidates;
    fixture.worldStepSurfaceContact.ccdPointTriangleChecks
        = diagnostics.surfaceContactCcdPointTriangleChecks;
    fixture.worldStepSurfaceContact.ccdEdgeEdgeChecks
        = diagnostics.surfaceContactCcdEdgeEdgeChecks;
    fixture.worldStepSurfaceContact.ccdHits = diagnostics.surfaceContactCcdHits;
    fixture.worldStepSurfaceContact.ccdLimitedSteps
        = diagnostics.surfaceContactCcdLimitedSteps;
    fixture.worldStepSurfaceContact.ccdZeroStepCount
        = diagnostics.surfaceContactCcdZeroStepCount;

    const WorldStepSurfaceContactCounters interBody
        = captureInterBodyWitnessCounters();
    fixture.worldStepSurfaceContact.interBodyCandidateBuilds
        = interBody.interBodyCandidateBuilds;
    fixture.worldStepSurfaceContact.interBodyCandidatePairCapacity
        = interBody.interBodyCandidatePairCapacity;
    fixture.worldStepSurfaceContact.interBodyCandidateRejectedPairs
        = interBody.interBodyCandidateRejectedPairs;
    fixture.worldStepSurfaceContact.interBodyPointTriangleCandidates
        = interBody.interBodyPointTriangleCandidates;
    fixture.worldStepSurfaceContact.interBodyEdgeEdgeCandidates
        = interBody.interBodyEdgeEdgeCandidates;
  }
  return fixture;
}

SweptCandidateSweepFixture makeSceneSweptPointTriangleSweepFixture(
    const int pairCount)
{
  const RuntimeSweepCandidateBufferFixture runtimeFixture
      = makeSceneRuntimeCandidateBufferFixture(pairCount);

  SweptCandidateSweepFixture fixture;
  fixture.startPositions = runtimeFixture.startPositions;
  fixture.endPositions = runtimeFixture.endPositions;
  fixture.triangles = runtimeFixture.triangleIndices;
  fixture.pointCount = runtimeFixture.startPositions.size() / 3u;
  fixture.triangleCount = runtimeFixture.triangleIndices.size() / 3u;
  fixture.pairCapacity = fixture.pointCount * fixture.triangleCount;
  fixture.sceneBodyCount = runtimeFixture.sceneBodyCount;
  fixture.pointIndices.reserve(fixture.pointCount);
  for (std::size_t point = 0; point < fixture.pointCount; ++point) {
    fixture.pointIndices.push_back(static_cast<std::uint32_t>(point));
  }
  appendSweptPointTriangleSweepCandidates(fixture);

  return fixture;
}

SweptEdgeEdgeSweepFixture makeSceneSweptEdgeEdgeSweepFixture(
    const int pairCount)
{
  const RuntimeSweepCandidateBufferFixture runtimeFixture
      = makeSceneRuntimeCandidateBufferFixture(pairCount);

  SweptEdgeEdgeSweepFixture fixture;
  fixture.startPositions = runtimeFixture.startPositions;
  fixture.endPositions = runtimeFixture.endPositions;
  fixture.edgeIndices = runtimeFixture.edgeIndices;
  fixture.edgeCount = runtimeFixture.edgeIndices.size() / 2u;
  fixture.pairCapacity = fixture.edgeCount * fixture.edgeCount;
  fixture.sceneBodyCount = runtimeFixture.sceneBodyCount;
  appendSweptEdgeEdgeSweepCandidates(fixture);

  return fixture;
}

CombinedSceneSweptCandidateSweepFixture
makeSceneCombinedSweptCandidateSweepFixture(
    const int pairCount, const bool captureWorldStepSurfaceContact = false)
{
  const RuntimeSweepCandidateBufferFixture runtimeFixture
      = makeSceneRuntimeCandidateBufferFixture(
          pairCount, captureWorldStepSurfaceContact);

  CombinedSceneSweptCandidateSweepFixture fixture;
  fixture.worldStepSurfaceContact = runtimeFixture.worldStepSurfaceContact;

  fixture.sweep.pointTriangle.startPositions = runtimeFixture.startPositions;
  fixture.sweep.pointTriangle.endPositions = runtimeFixture.endPositions;
  fixture.sweep.pointTriangle.triangles = runtimeFixture.triangleIndices;
  fixture.sweep.pointTriangle.pointCount
      = runtimeFixture.startPositions.size() / 3u;
  fixture.sweep.pointTriangle.triangleCount
      = runtimeFixture.triangleIndices.size() / 3u;
  fixture.sweep.pointTriangle.pairCapacity
      = fixture.sweep.pointTriangle.pointCount
        * fixture.sweep.pointTriangle.triangleCount;
  fixture.sweep.pointTriangle.sceneBodyCount = runtimeFixture.sceneBodyCount;
  fixture.sweep.pointTriangle.pointIndices.reserve(
      fixture.sweep.pointTriangle.pointCount);
  for (std::size_t point = 0; point < fixture.sweep.pointTriangle.pointCount;
       ++point) {
    fixture.sweep.pointTriangle.pointIndices.push_back(
        static_cast<std::uint32_t>(point));
  }
  appendSweptPointTriangleSweepCandidates(fixture.sweep.pointTriangle);

  fixture.sweep.edgeEdge.startPositions = runtimeFixture.startPositions;
  fixture.sweep.edgeEdge.endPositions = runtimeFixture.endPositions;
  fixture.sweep.edgeEdge.edgeIndices = runtimeFixture.edgeIndices;
  fixture.sweep.edgeEdge.edgeCount = runtimeFixture.edgeIndices.size() / 2u;
  fixture.sweep.edgeEdge.pairCapacity
      = fixture.sweep.edgeEdge.edgeCount * fixture.sweep.edgeEdge.edgeCount;
  fixture.sweep.edgeEdge.sceneBodyCount = runtimeFixture.sceneBodyCount;
  appendSweptEdgeEdgeSweepCandidates(fixture.sweep.edgeEdge);

  return fixture;
}

void recordSharedCounters(
    benchmark::State& state,
    const CandidateFixture& fixture,
    const double maxError)
{
  state.counters["stencils"] = static_cast<double>(fixture.stencils.size());
  state.counters["accepted_count"] = static_cast<double>(fixture.acceptedCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.stencils.size()));
}

void recordSharedCounters(
    benchmark::State& state,
    const EdgeEdgeCandidateFixture& fixture,
    const double maxError)
{
  state.counters["stencils"] = static_cast<double>(fixture.stencils.size());
  state.counters["accepted_count"] = static_cast<double>(fixture.acceptedCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.stencils.size()));
}

void recordCandidateMaskCounters(
    benchmark::State& state,
    const CandidateMaskFixture& fixture,
    const double maxError)
{
  state.counters["pairs"] = static_cast<double>(fixture.pairCount);
  state.counters["points"] = static_cast<double>(fixture.pointCount);
  state.counters["triangles"] = static_cast<double>(fixture.triangleCount);
  state.counters["accepted_count"] = static_cast<double>(fixture.acceptedCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.pairCount));
}

void recordEdgeEdgeCandidateMaskCounters(
    benchmark::State& state,
    const EdgeEdgeCandidateMaskFixture& fixture,
    const double maxError)
{
  state.counters["pairs"] = static_cast<double>(fixture.pairCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["accepted_count"] = static_cast<double>(fixture.acceptedCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.pairCount));
}

void recordSweptCandidateMaskCounters(
    benchmark::State& state,
    const SweptCandidateMaskFixture& fixture,
    const double maxError)
{
  state.counters["pairs"] = static_cast<double>(fixture.pairCount);
  state.counters["points"] = static_cast<double>(fixture.pointCount);
  state.counters["triangles"] = static_cast<double>(fixture.triangleCount);
  state.counters["accepted_count"] = static_cast<double>(fixture.acceptedCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.pairCount));
}

void recordSweptEdgeEdgeCandidateMaskCounters(
    benchmark::State& state,
    const SweptEdgeEdgeCandidateMaskFixture& fixture,
    const double maxError)
{
  state.counters["pairs"] = static_cast<double>(fixture.pairCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["accepted_count"] = static_cast<double>(fixture.acceptedCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.pairCount));
}

void recordSweptCandidateSweepCounters(
    benchmark::State& state,
    const SweptCandidateSweepFixture& fixture,
    const double maxError)
{
  state.counters["pair_capacity"] = static_cast<double>(fixture.pairCapacity);
  state.counters["points"] = static_cast<double>(fixture.pointCount);
  state.counters["triangles"] = static_cast<double>(fixture.triangleCount);
  state.counters["accepted_count"] = static_cast<double>(fixture.acceptedCount);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.pairCapacity));
}

void recordSweptEdgeEdgeSweepCounters(
    benchmark::State& state,
    const SweptEdgeEdgeSweepFixture& fixture,
    const double maxError)
{
  state.counters["pair_capacity"] = static_cast<double>(fixture.pairCapacity);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["accepted_count"] = static_cast<double>(fixture.acceptedCount);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.pairCapacity));
}

void recordCombinedSweptCandidateSweepCounters(
    benchmark::State& state,
    const CombinedSweptCandidateSweepFixture& fixture,
    const double maxError)
{
  const std::size_t pointTrianglePairCapacity
      = fixture.pointTriangle.pairCapacity;
  const std::size_t edgeEdgePairCapacity = fixture.edgeEdge.pairCapacity;
  const std::size_t pointTriangleAcceptedCount
      = fixture.pointTriangle.acceptedCount;
  const std::size_t edgeEdgeAcceptedCount = fixture.edgeEdge.acceptedCount;

  state.counters["pair_capacity"]
      = static_cast<double>(pointTrianglePairCapacity + edgeEdgePairCapacity);
  state.counters["point_triangle_pair_capacity"]
      = static_cast<double>(pointTrianglePairCapacity);
  state.counters["edge_edge_pair_capacity"]
      = static_cast<double>(edgeEdgePairCapacity);
  state.counters["accepted_count"]
      = static_cast<double>(pointTriangleAcceptedCount + edgeEdgeAcceptedCount);
  state.counters["point_triangle_accepted_count"]
      = static_cast<double>(pointTriangleAcceptedCount);
  state.counters["edge_edge_accepted_count"]
      = static_cast<double>(edgeEdgeAcceptedCount);
  state.counters["points"]
      = static_cast<double>(fixture.pointTriangle.pointCount);
  state.counters["triangles"]
      = static_cast<double>(fixture.pointTriangle.triangleCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeEdge.edgeCount);
  state.counters["scene_bodies"]
      = static_cast<double>(fixture.pointTriangle.sceneBodyCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (pointTrianglePairCapacity + edgeEdgePairCapacity)));
}

void recordRuntimePointTriangleCandidateBufferCounters(
    benchmark::State& state,
    const RuntimeSweepCandidateBufferFixture& fixture,
    const double maxError)
{
  state.counters["candidates"] = static_cast<double>(
      fixture.pointTriangleEndpointSquaredDistances.size());
  state.counters["points"]
      = static_cast<double>(fixture.startPositions.size() / 3u);
  state.counters["triangles"] = static_cast<double>(fixture.triangles.size());
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * fixture.pointTriangleEndpointSquaredDistances.size()));
}

void recordRuntimeEdgeEdgeCandidateBufferCounters(
    benchmark::State& state,
    const RuntimeSweepCandidateBufferFixture& fixture,
    const double maxError)
{
  state.counters["candidates"]
      = static_cast<double>(fixture.edgeEdgeEndpointSquaredDistances.size());
  state.counters["edges"]
      = static_cast<double>(fixture.edgeIndices.size() / 2u);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * fixture.edgeEdgeEndpointSquaredDistances.size()));
}

void recordCombinedRuntimeCandidateBufferCounters(
    benchmark::State& state,
    const RuntimeSweepCandidateBufferFixture& fixture,
    const double maxError)
{
  const std::size_t pointTriangleCandidates
      = fixture.pointTriangleEndpointSquaredDistances.size();
  const std::size_t edgeEdgeCandidates
      = fixture.edgeEdgeEndpointSquaredDistances.size();
  state.counters["candidates"]
      = static_cast<double>(pointTriangleCandidates + edgeEdgeCandidates);
  state.counters["point_triangle_candidates"]
      = static_cast<double>(pointTriangleCandidates);
  state.counters["edge_edge_candidates"]
      = static_cast<double>(edgeEdgeCandidates);
  state.counters["points"]
      = static_cast<double>(fixture.startPositions.size() / 3u);
  state.counters["triangles"] = static_cast<double>(fixture.triangles.size());
  state.counters["edges"]
      = static_cast<double>(fixture.edgeIndices.size() / 2u);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations() * (pointTriangleCandidates + edgeEdgeCandidates)));
}

void recordFilteredCombinedCandidateBufferCounters(
    benchmark::State& state,
    const CombinedSceneSweptCandidateSweepFixture& fixture,
    const double maxError)
{
  const std::size_t pointTrianglePairCapacity
      = fixture.sweep.pointTriangle.pairCapacity;
  const std::size_t edgeEdgePairCapacity = fixture.sweep.edgeEdge.pairCapacity;
  const std::size_t pointTriangleCandidates
      = fixture.sweep.pointTriangle.acceptedCount;
  const std::size_t edgeEdgeCandidates = fixture.sweep.edgeEdge.acceptedCount;
  const std::size_t pairCapacity
      = pointTrianglePairCapacity + edgeEdgePairCapacity;
  const std::size_t candidateCount
      = pointTriangleCandidates + edgeEdgeCandidates;

  state.counters["pair_capacity"] = static_cast<double>(pairCapacity);
  state.counters["point_triangle_pair_capacity"]
      = static_cast<double>(pointTrianglePairCapacity);
  state.counters["edge_edge_pair_capacity"]
      = static_cast<double>(edgeEdgePairCapacity);
  state.counters["candidate_count"] = static_cast<double>(candidateCount);
  state.counters["accepted_count"] = static_cast<double>(candidateCount);
  state.counters["rejected_count"]
      = static_cast<double>(pairCapacity - candidateCount);
  state.counters["point_triangle_candidate_count"]
      = static_cast<double>(pointTriangleCandidates);
  state.counters["edge_edge_candidate_count"]
      = static_cast<double>(edgeEdgeCandidates);
  state.counters["point_triangle_rejected_count"] = static_cast<double>(
      pointTrianglePairCapacity - pointTriangleCandidates);
  state.counters["edge_edge_rejected_count"]
      = static_cast<double>(edgeEdgePairCapacity - edgeEdgeCandidates);
  state.counters["points"]
      = static_cast<double>(fixture.sweep.pointTriangle.pointCount);
  state.counters["triangles"]
      = static_cast<double>(fixture.sweep.pointTriangle.triangleCount);
  state.counters["edges"]
      = static_cast<double>(fixture.sweep.edgeEdge.edgeCount);
  state.counters["scene_bodies"]
      = static_cast<double>(fixture.sweep.pointTriangle.sceneBodyCount);
  state.counters["world_step_line_search_trials"]
      = static_cast<double>(fixture.worldStepSurfaceContact.lineSearchTrials);
  state.counters["world_step_surface_contact_candidate_builds"]
      = static_cast<double>(fixture.worldStepSurfaceContact.candidateBuilds);
  state.counters["world_step_surface_contact_candidate_pair_capacity"]
      = static_cast<double>(
          fixture.worldStepSurfaceContact.candidatePairCapacity);
  state.counters["world_step_surface_contact_candidate_rejected_pairs"]
      = static_cast<double>(fixture.worldStepSurfaceContact.rejectedPairs);
  state.counters["world_step_surface_contact_point_triangle_candidates"]
      = static_cast<double>(
          fixture.worldStepSurfaceContact.pointTriangleCandidates);
  state.counters["world_step_surface_contact_edge_edge_candidates"]
      = static_cast<double>(fixture.worldStepSurfaceContact.edgeEdgeCandidates);
  state.counters["world_step_surface_contact_ccd_point_triangle_checks"]
      = static_cast<double>(
          fixture.worldStepSurfaceContact.ccdPointTriangleChecks);
  state.counters["world_step_surface_contact_ccd_edge_edge_checks"]
      = static_cast<double>(fixture.worldStepSurfaceContact.ccdEdgeEdgeChecks);
  state.counters["world_step_surface_contact_ccd_hits"]
      = static_cast<double>(fixture.worldStepSurfaceContact.ccdHits);
  state.counters["world_step_surface_contact_ccd_limited_steps"]
      = static_cast<double>(fixture.worldStepSurfaceContact.ccdLimitedSteps);
  state.counters["world_step_surface_contact_ccd_zero_step_count"]
      = static_cast<double>(fixture.worldStepSurfaceContact.ccdZeroStepCount);
  state.counters["world_step_inter_body_surface_contact_candidate_builds"]
      = static_cast<double>(
          fixture.worldStepSurfaceContact.interBodyCandidateBuilds);
  state
      .counters["world_step_inter_body_surface_contact_candidate_pair_capacity"]
      = static_cast<double>(
          fixture.worldStepSurfaceContact.interBodyCandidatePairCapacity);
  state.counters
      ["world_step_inter_body_surface_contact_candidate_rejected_pairs"]
      = static_cast<double>(
          fixture.worldStepSurfaceContact.interBodyCandidateRejectedPairs);
  state.counters
      ["world_step_inter_body_surface_contact_point_triangle_candidates"]
      = static_cast<double>(
          fixture.worldStepSurfaceContact.interBodyPointTriangleCandidates);
  state.counters["world_step_inter_body_surface_contact_edge_edge_candidates"]
      = static_cast<double>(
          fixture.worldStepSurfaceContact.interBodyEdgeEdgeCandidates);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * pairCapacity));
}

double updateMaxDistanceError(
    const std::vector<double>& actual,
    const std::vector<double>& expected,
    double maxError)
{
  if (actual.size() != expected.size()) {
    return std::numeric_limits<double>::infinity();
  }

  for (std::size_t i = 0; i < expected.size(); ++i) {
    maxError = std::max(maxError, std::abs(actual[i] - expected[i]));
  }
  return maxError;
}

} // namespace

//==============================================================================
static void BM_Plan083ContactCandidateCpu(benchmark::State& state)
{
  const auto fixture = makeCandidateFixture(static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    acceptedCount = 0;
    for (const auto& stencil : fixture.stencils) {
      const std::size_t tri = 3u * static_cast<std::size_t>(stencil.triangle);
      const auto distance = dc::pointTriangleSquaredDistance(
          pointAt(fixture.positions, stencil.point),
          pointAt(fixture.positions, fixture.triangles[tri]),
          pointAt(fixture.positions, fixture.triangles[tri + 1u]),
          pointAt(fixture.positions, fixture.triangles[tri + 2u]));
      acceptedCount += dc::detail::withinActivationDistance(
                           distance.squaredDistance, kActivationDistance)
                           ? 1u
                           : 0u;
      double squaredDistance = distance.squaredDistance;
      benchmark::DoNotOptimize(squaredDistance);
    }
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordSharedCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083ContactCandidateCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083ContactCandidateCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeCandidateFixture(static_cast<int>(state.range(0)));
  cuda::PointTriangleCandidateFilterResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::filterPointTriangleContactStencilsCuda(
        fixture.positions,
        fixture.triangles,
        fixture.stencils,
        kActivationDistance,
        result);
    benchmark::DoNotOptimize(result.squaredDistances.data());
    benchmark::DoNotOptimize(result.accepted.data());
  }

  for (std::size_t i = 0; i < fixture.cpuSquaredDistances.size(); ++i) {
    maxError = std::max(
        maxError,
        std::abs(result.squaredDistances[i] - fixture.cpuSquaredDistances[i]));
  }

  recordSharedCounters(state, fixture, maxError);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083ContactCandidateCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointTriangleCandidateMaskCpu(benchmark::State& state)
{
  const auto fixture
      = makeCandidateMaskFixture(static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    acceptedCount = 0;
    for (const std::uint32_t point : fixture.pointIndices) {
      for (std::size_t triangle = 0; triangle < fixture.triangleCount;
           ++triangle) {
        const std::size_t tri = 3u * triangle;
        double squaredDistance = 0.0;
        if (!isIncidentPointTriangle(point, fixture.triangles, triangle)) {
          const auto distance = dc::pointTriangleSquaredDistance(
              pointAt(fixture.positions, point),
              pointAt(fixture.positions, fixture.triangles[tri]),
              pointAt(fixture.positions, fixture.triangles[tri + 1u]),
              pointAt(fixture.positions, fixture.triangles[tri + 2u]));
          squaredDistance = distance.squaredDistance;
          acceptedCount += dc::detail::withinActivationDistance(
                               squaredDistance, kActivationDistance)
                               ? 1u
                               : 0u;
        }
        benchmark::DoNotOptimize(squaredDistance);
      }
    }
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordCandidateMaskCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083PointTriangleCandidateMaskCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointTriangleCandidateMaskCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeCandidateMaskFixture(static_cast<int>(state.range(0)));
  cuda::PointTriangleCandidateBuildResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildPointTriangleContactCandidateMaskCuda(
        fixture.positions,
        fixture.pointIndices,
        fixture.triangles,
        kActivationDistance,
        result);
    benchmark::DoNotOptimize(result.squaredDistances.data());
    benchmark::DoNotOptimize(result.accepted.data());
  }

  for (std::size_t i = 0; i < fixture.cpuSquaredDistances.size(); ++i) {
    maxError = std::max(
        maxError,
        std::abs(result.squaredDistances[i] - fixture.cpuSquaredDistances[i]));
  }

  recordCandidateMaskCounters(state, fixture, maxError);
  state.counters["gpu_pairs"] = static_cast<double>(result.pairCount);
  state.counters["gpu_points"] = static_cast<double>(result.pointCount);
  state.counters["gpu_triangles"] = static_cast<double>(result.triangleCount);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["gpu_compacted_count"]
      = static_cast<double>(result.acceptedPointIndices.size());
  state.counters["gpu_compacted_triangle_count"]
      = static_cast<double>(result.acceptedTriangleIndices.size());
  state.counters["gpu_compacted_distance_count"]
      = static_cast<double>(result.acceptedSquaredDistances.size());
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083PointTriangleCandidateMaskCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083EdgeEdgeCandidateMaskCpu(benchmark::State& state)
{
  const auto fixture
      = makeEdgeEdgeCandidateMaskFixture(static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    acceptedCount = 0;
    for (std::size_t edgeA = 0; edgeA < fixture.edgeCount; ++edgeA) {
      for (std::size_t edgeB = 0; edgeB < fixture.edgeCount; ++edgeB) {
        double squaredDistance = 0.0;
        const std::uint32_t a0 = fixture.edgeIndices[2u * edgeA];
        const std::uint32_t a1 = fixture.edgeIndices[2u * edgeA + 1u];
        const std::uint32_t b0 = fixture.edgeIndices[2u * edgeB];
        const std::uint32_t b1 = fixture.edgeIndices[2u * edgeB + 1u];
        if (edgeA < edgeB && !edgesShareVertex(a0, a1, b0, b1)) {
          const auto distance = dc::edgeEdgeSquaredDistance(
              pointAt(fixture.positions, a0),
              pointAt(fixture.positions, a1),
              pointAt(fixture.positions, b0),
              pointAt(fixture.positions, b1));
          squaredDistance = distance.squaredDistance;
          acceptedCount += dc::detail::withinActivationDistance(
                               squaredDistance, kActivationDistance)
                               ? 1u
                               : 0u;
        }
        benchmark::DoNotOptimize(squaredDistance);
      }
    }
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordEdgeEdgeCandidateMaskCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083EdgeEdgeCandidateMaskCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083EdgeEdgeCandidateMaskCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeEdgeEdgeCandidateMaskFixture(static_cast<int>(state.range(0)));
  cuda::EdgeEdgeCandidateBuildResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildEdgeEdgeContactCandidateMaskCuda(
        fixture.positions, fixture.edgeIndices, kActivationDistance, result);
    benchmark::DoNotOptimize(result.squaredDistances.data());
    benchmark::DoNotOptimize(result.accepted.data());
  }

  for (std::size_t i = 0; i < fixture.cpuSquaredDistances.size(); ++i) {
    maxError = std::max(
        maxError,
        std::abs(result.squaredDistances[i] - fixture.cpuSquaredDistances[i]));
  }

  recordEdgeEdgeCandidateMaskCounters(state, fixture, maxError);
  state.counters["gpu_pairs"] = static_cast<double>(result.pairCount);
  state.counters["gpu_edges"] = static_cast<double>(result.edgeCount);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["gpu_compacted_edge_a_count"]
      = static_cast<double>(result.acceptedEdgeAIndices.size());
  state.counters["gpu_compacted_edge_b_count"]
      = static_cast<double>(result.acceptedEdgeBIndices.size());
  state.counters["gpu_compacted_distance_count"]
      = static_cast<double>(result.acceptedSquaredDistances.size());
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083EdgeEdgeCandidateMaskCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SweptPointTriangleCandidateMaskCpu(
    benchmark::State& state)
{
  const auto fixture
      = makeSweptCandidateMaskFixture(static_cast<int>(state.range(0)));
  const double margin = 0.5 * kActivationDistance;

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    acceptedCount = 0;
    for (const std::uint32_t point : fixture.pointIndices) {
      for (std::size_t triangle = 0; triangle < fixture.triangleCount;
           ++triangle) {
        const std::size_t tri = 3u * triangle;
        double endpointSquaredDistance = 0.0;
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
          if (pointAabb.overlaps(triangleAabb)) {
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
            ++acceptedCount;
          }
        }
        benchmark::DoNotOptimize(endpointSquaredDistance);
      }
    }
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordSweptCandidateMaskCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SweptPointTriangleCandidateMaskCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SweptPointTriangleCandidateMaskCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSweptCandidateMaskFixture(static_cast<int>(state.range(0)));
  cuda::SweptPointTriangleCandidateBuildResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildSweptPointTriangleContactCandidateMaskCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.pointIndices,
        fixture.triangles,
        kActivationDistance,
        result);
    benchmark::DoNotOptimize(result.endpointSquaredDistances.data());
    benchmark::DoNotOptimize(result.accepted.data());
  }

  for (std::size_t i = 0; i < fixture.cpuEndpointSquaredDistances.size(); ++i) {
    maxError = std::max(
        maxError,
        std::abs(
            result.endpointSquaredDistances[i]
            - fixture.cpuEndpointSquaredDistances[i]));
  }

  recordSweptCandidateMaskCounters(state, fixture, maxError);
  state.counters["gpu_pairs"] = static_cast<double>(result.pairCount);
  state.counters["gpu_points"] = static_cast<double>(result.pointCount);
  state.counters["gpu_triangles"] = static_cast<double>(result.triangleCount);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["gpu_compacted_count"]
      = static_cast<double>(result.acceptedPointIndices.size());
  state.counters["gpu_compacted_triangle_count"]
      = static_cast<double>(result.acceptedTriangleIndices.size());
  state.counters["gpu_compacted_distance_count"]
      = static_cast<double>(result.acceptedEndpointSquaredDistances.size());
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SweptPointTriangleCandidateMaskCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SweptEdgeEdgeCandidateMaskCpu(benchmark::State& state)
{
  const auto fixture
      = makeSweptEdgeEdgeCandidateMaskFixture(static_cast<int>(state.range(0)));
  const double margin = 0.5 * kActivationDistance;

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    acceptedCount = 0;
    for (std::size_t edgeA = 0; edgeA < fixture.edgeCount; ++edgeA) {
      for (std::size_t edgeB = 0; edgeB < fixture.edgeCount; ++edgeB) {
        double endpointSquaredDistance = 0.0;
        const std::uint32_t a0 = fixture.edgeIndices[2u * edgeA];
        const std::uint32_t a1 = fixture.edgeIndices[2u * edgeA + 1u];
        const std::uint32_t b0 = fixture.edgeIndices[2u * edgeB];
        const std::uint32_t b1 = fixture.edgeIndices[2u * edgeB + 1u];
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
          if (edgeAAabb.overlaps(edgeBAabb)) {
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
            ++acceptedCount;
          }
        }
        benchmark::DoNotOptimize(endpointSquaredDistance);
      }
    }
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordSweptEdgeEdgeCandidateMaskCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SweptEdgeEdgeCandidateMaskCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SweptEdgeEdgeCandidateMaskCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSweptEdgeEdgeCandidateMaskFixture(static_cast<int>(state.range(0)));
  cuda::SweptEdgeEdgeCandidateBuildResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildSweptEdgeEdgeContactCandidateMaskCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.edgeIndices,
        kActivationDistance,
        result);
    benchmark::DoNotOptimize(result.endpointSquaredDistances.data());
    benchmark::DoNotOptimize(result.accepted.data());
  }

  for (std::size_t i = 0; i < fixture.cpuEndpointSquaredDistances.size(); ++i) {
    maxError = std::max(
        maxError,
        std::abs(
            result.endpointSquaredDistances[i]
            - fixture.cpuEndpointSquaredDistances[i]));
  }

  recordSweptEdgeEdgeCandidateMaskCounters(state, fixture, maxError);
  state.counters["gpu_pairs"] = static_cast<double>(result.pairCount);
  state.counters["gpu_edges"] = static_cast<double>(result.edgeCount);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["gpu_compacted_edge_a_count"]
      = static_cast<double>(result.acceptedEdgeAIndices.size());
  state.counters["gpu_compacted_edge_b_count"]
      = static_cast<double>(result.acceptedEdgeBIndices.size());
  state.counters["gpu_compacted_distance_count"]
      = static_cast<double>(result.acceptedEndpointSquaredDistances.size());
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SweptEdgeEdgeCandidateMaskCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SweptPointTriangleSweepCpu(benchmark::State& state)
{
  const auto fixture
      = makeSweptCandidateSweepFixture(static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    SweptCandidateSweepFixture sweepFixture;
    sweepFixture.startPositions = fixture.startPositions;
    sweepFixture.endPositions = fixture.endPositions;
    sweepFixture.pointIndices = fixture.pointIndices;
    sweepFixture.triangles = fixture.triangles;
    sweepFixture.pointCount = fixture.pointCount;
    sweepFixture.triangleCount = fixture.triangleCount;
    sweepFixture.pairCapacity = fixture.pairCapacity;
    appendSweptPointTriangleSweepCandidates(sweepFixture);
    acceptedCount = sweepFixture.acceptedCount;
    benchmark::DoNotOptimize(
        sweepFixture.cpuAcceptedEndpointSquaredDistances.data());
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordSweptCandidateSweepCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SweptPointTriangleSweepCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SweptPointTriangleSweepCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSweptCandidateSweepFixture(static_cast<int>(state.range(0)));
  cuda::SweptPointTriangleSweepResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildSweptPointTriangleSweepCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.pointIndices,
        fixture.triangles,
        kActivationDistance,
        result);
    benchmark::DoNotOptimize(result.acceptedPointIndices.data());
    benchmark::DoNotOptimize(result.acceptedEndpointSquaredDistances.data());
  }

  if (result.acceptedEndpointSquaredDistances.size()
      == fixture.cpuAcceptedEndpointSquaredDistances.size()) {
    for (std::size_t i = 0;
         i < fixture.cpuAcceptedEndpointSquaredDistances.size();
         ++i) {
      maxError = std::max(
          maxError,
          std::abs(
              result.acceptedEndpointSquaredDistances[i]
              - fixture.cpuAcceptedEndpointSquaredDistances[i]));
    }
  } else {
    maxError = std::numeric_limits<double>::infinity();
  }

  recordSweptCandidateSweepCounters(state, fixture, maxError);
  state.counters["gpu_pair_capacity"]
      = static_cast<double>(result.pairCapacity);
  state.counters["gpu_points"] = static_cast<double>(result.pointCount);
  state.counters["gpu_triangles"] = static_cast<double>(result.triangleCount);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["gpu_compacted_count"]
      = static_cast<double>(result.acceptedPointIndices.size());
  state.counters["gpu_compacted_triangle_count"]
      = static_cast<double>(result.acceptedTriangleIndices.size());
  state.counters["gpu_compacted_distance_count"]
      = static_cast<double>(result.acceptedEndpointSquaredDistances.size());
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SweptPointTriangleSweepCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SweptEdgeEdgeSweepCpu(benchmark::State& state)
{
  const auto fixture
      = makeSweptEdgeEdgeSweepFixture(static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    SweptEdgeEdgeSweepFixture sweepFixture;
    sweepFixture.startPositions = fixture.startPositions;
    sweepFixture.endPositions = fixture.endPositions;
    sweepFixture.edgeIndices = fixture.edgeIndices;
    sweepFixture.edgeCount = fixture.edgeCount;
    sweepFixture.pairCapacity = fixture.pairCapacity;
    appendSweptEdgeEdgeSweepCandidates(sweepFixture);
    acceptedCount = sweepFixture.acceptedCount;
    benchmark::DoNotOptimize(
        sweepFixture.cpuAcceptedEndpointSquaredDistances.data());
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordSweptEdgeEdgeSweepCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SweptEdgeEdgeSweepCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SweptEdgeEdgeSweepCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSweptEdgeEdgeSweepFixture(static_cast<int>(state.range(0)));
  cuda::SweptEdgeEdgeSweepResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildSweptEdgeEdgeSweepCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.edgeIndices,
        kActivationDistance,
        result);
    benchmark::DoNotOptimize(result.acceptedEdgeAIndices.data());
    benchmark::DoNotOptimize(result.acceptedEndpointSquaredDistances.data());
  }

  if (result.acceptedEndpointSquaredDistances.size()
      == fixture.cpuAcceptedEndpointSquaredDistances.size()) {
    for (std::size_t i = 0;
         i < fixture.cpuAcceptedEndpointSquaredDistances.size();
         ++i) {
      maxError = std::max(
          maxError,
          std::abs(
              result.acceptedEndpointSquaredDistances[i]
              - fixture.cpuAcceptedEndpointSquaredDistances[i]));
    }
  } else {
    maxError = std::numeric_limits<double>::infinity();
  }

  recordSweptEdgeEdgeSweepCounters(state, fixture, maxError);
  state.counters["gpu_pair_capacity"]
      = static_cast<double>(result.pairCapacity);
  state.counters["gpu_edges"] = static_cast<double>(result.edgeCount);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["gpu_compacted_edge_a_count"]
      = static_cast<double>(result.acceptedEdgeAIndices.size());
  state.counters["gpu_compacted_edge_b_count"]
      = static_cast<double>(result.acceptedEdgeBIndices.size());
  state.counters["gpu_compacted_distance_count"]
      = static_cast<double>(result.acceptedEndpointSquaredDistances.size());
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SweptEdgeEdgeSweepCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimePointTriangleSweepCpu(benchmark::State& state)
{
  const auto fixture = makeSceneSweptPointTriangleSweepFixture(
      static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    SweptCandidateSweepFixture sweepFixture;
    sweepFixture.startPositions = fixture.startPositions;
    sweepFixture.endPositions = fixture.endPositions;
    sweepFixture.pointIndices = fixture.pointIndices;
    sweepFixture.triangles = fixture.triangles;
    sweepFixture.pointCount = fixture.pointCount;
    sweepFixture.triangleCount = fixture.triangleCount;
    sweepFixture.pairCapacity = fixture.pairCapacity;
    sweepFixture.sceneBodyCount = fixture.sceneBodyCount;
    appendSweptPointTriangleSweepCandidates(sweepFixture);
    acceptedCount = sweepFixture.acceptedCount;
    benchmark::DoNotOptimize(
        sweepFixture.cpuAcceptedEndpointSquaredDistances.data());
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordSweptCandidateSweepCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimePointTriangleSweepCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimePointTriangleSweepCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneSweptPointTriangleSweepFixture(
      static_cast<int>(state.range(0)));
  cuda::SweptPointTriangleSweepResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildSweptPointTriangleSweepCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.pointIndices,
        fixture.triangles,
        kActivationDistance,
        result);
    benchmark::DoNotOptimize(result.acceptedPointIndices.data());
    benchmark::DoNotOptimize(result.acceptedEndpointSquaredDistances.data());
  }

  if (result.acceptedEndpointSquaredDistances.size()
      == fixture.cpuAcceptedEndpointSquaredDistances.size()) {
    for (std::size_t i = 0;
         i < fixture.cpuAcceptedEndpointSquaredDistances.size();
         ++i) {
      maxError = std::max(
          maxError,
          std::abs(
              result.acceptedEndpointSquaredDistances[i]
              - fixture.cpuAcceptedEndpointSquaredDistances[i]));
    }
  } else {
    maxError = std::numeric_limits<double>::infinity();
  }

  recordSweptCandidateSweepCounters(state, fixture, maxError);
  state.counters["gpu_pair_capacity"]
      = static_cast<double>(result.pairCapacity);
  state.counters["gpu_points"] = static_cast<double>(result.pointCount);
  state.counters["gpu_triangles"] = static_cast<double>(result.triangleCount);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["gpu_compacted_count"]
      = static_cast<double>(result.acceptedPointIndices.size());
  state.counters["gpu_compacted_triangle_count"]
      = static_cast<double>(result.acceptedTriangleIndices.size());
  state.counters["gpu_compacted_distance_count"]
      = static_cast<double>(result.acceptedEndpointSquaredDistances.size());
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimePointTriangleSweepCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeEdgeEdgeSweepCpu(benchmark::State& state)
{
  const auto fixture
      = makeSceneSweptEdgeEdgeSweepFixture(static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    SweptEdgeEdgeSweepFixture sweepFixture;
    sweepFixture.startPositions = fixture.startPositions;
    sweepFixture.endPositions = fixture.endPositions;
    sweepFixture.edgeIndices = fixture.edgeIndices;
    sweepFixture.edgeCount = fixture.edgeCount;
    sweepFixture.pairCapacity = fixture.pairCapacity;
    sweepFixture.sceneBodyCount = fixture.sceneBodyCount;
    appendSweptEdgeEdgeSweepCandidates(sweepFixture);
    acceptedCount = sweepFixture.acceptedCount;
    benchmark::DoNotOptimize(
        sweepFixture.cpuAcceptedEndpointSquaredDistances.data());
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordSweptEdgeEdgeSweepCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimeEdgeEdgeSweepCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeEdgeEdgeSweepCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSceneSweptEdgeEdgeSweepFixture(static_cast<int>(state.range(0)));
  cuda::SweptEdgeEdgeSweepResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildSweptEdgeEdgeSweepCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.edgeIndices,
        kActivationDistance,
        result);
    benchmark::DoNotOptimize(result.acceptedEdgeAIndices.data());
    benchmark::DoNotOptimize(result.acceptedEndpointSquaredDistances.data());
  }

  if (result.acceptedEndpointSquaredDistances.size()
      == fixture.cpuAcceptedEndpointSquaredDistances.size()) {
    for (std::size_t i = 0;
         i < fixture.cpuAcceptedEndpointSquaredDistances.size();
         ++i) {
      maxError = std::max(
          maxError,
          std::abs(
              result.acceptedEndpointSquaredDistances[i]
              - fixture.cpuAcceptedEndpointSquaredDistances[i]));
    }
  } else {
    maxError = std::numeric_limits<double>::infinity();
  }

  recordSweptEdgeEdgeSweepCounters(state, fixture, maxError);
  state.counters["gpu_pair_capacity"]
      = static_cast<double>(result.pairCapacity);
  state.counters["gpu_edges"] = static_cast<double>(result.edgeCount);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["gpu_compacted_edge_a_count"]
      = static_cast<double>(result.acceptedEdgeAIndices.size());
  state.counters["gpu_compacted_edge_b_count"]
      = static_cast<double>(result.acceptedEdgeBIndices.size());
  state.counters["gpu_compacted_distance_count"]
      = static_cast<double>(result.acceptedEndpointSquaredDistances.size());
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimeEdgeEdgeSweepCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeCombinedSweepFilterCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneCombinedSweptCandidateSweepFixture(
      static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    SweptCandidateSweepFixture pointTriangleFixture;
    pointTriangleFixture.startPositions
        = fixture.sweep.pointTriangle.startPositions;
    pointTriangleFixture.endPositions
        = fixture.sweep.pointTriangle.endPositions;
    pointTriangleFixture.pointIndices
        = fixture.sweep.pointTriangle.pointIndices;
    pointTriangleFixture.triangles = fixture.sweep.pointTriangle.triangles;
    pointTriangleFixture.pointCount = fixture.sweep.pointTriangle.pointCount;
    pointTriangleFixture.triangleCount
        = fixture.sweep.pointTriangle.triangleCount;
    pointTriangleFixture.pairCapacity
        = fixture.sweep.pointTriangle.pairCapacity;
    pointTriangleFixture.sceneBodyCount
        = fixture.sweep.pointTriangle.sceneBodyCount;
    appendSweptPointTriangleSweepCandidates(pointTriangleFixture);

    SweptEdgeEdgeSweepFixture edgeEdgeFixture;
    edgeEdgeFixture.startPositions = fixture.sweep.edgeEdge.startPositions;
    edgeEdgeFixture.endPositions = fixture.sweep.edgeEdge.endPositions;
    edgeEdgeFixture.edgeIndices = fixture.sweep.edgeEdge.edgeIndices;
    edgeEdgeFixture.edgeCount = fixture.sweep.edgeEdge.edgeCount;
    edgeEdgeFixture.pairCapacity = fixture.sweep.edgeEdge.pairCapacity;
    edgeEdgeFixture.sceneBodyCount = fixture.sweep.edgeEdge.sceneBodyCount;
    appendSweptEdgeEdgeSweepCandidates(edgeEdgeFixture);

    acceptedCount
        = pointTriangleFixture.acceptedCount + edgeEdgeFixture.acceptedCount;
    benchmark::DoNotOptimize(
        pointTriangleFixture.cpuAcceptedEndpointSquaredDistances.data());
    benchmark::DoNotOptimize(
        edgeEdgeFixture.cpuAcceptedEndpointSquaredDistances.data());
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordCombinedSweptCandidateSweepCounters(state, fixture.sweep, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimeCombinedSweepFilterCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeCombinedSweepFilterCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneCombinedSweptCandidateSweepFixture(
      static_cast<int>(state.range(0)));
  cuda::SweptPointTriangleSweepResult pointTriangleResult;
  cuda::SweptEdgeEdgeSweepResult edgeEdgeResult;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildSweptPointTriangleSweepCuda(
        fixture.sweep.pointTriangle.startPositions,
        fixture.sweep.pointTriangle.endPositions,
        fixture.sweep.pointTriangle.pointIndices,
        fixture.sweep.pointTriangle.triangles,
        kActivationDistance,
        pointTriangleResult);
    cuda::buildSweptEdgeEdgeSweepCuda(
        fixture.sweep.edgeEdge.startPositions,
        fixture.sweep.edgeEdge.endPositions,
        fixture.sweep.edgeEdge.edgeIndices,
        kActivationDistance,
        edgeEdgeResult);
    benchmark::DoNotOptimize(pointTriangleResult.acceptedPointIndices.data());
    benchmark::DoNotOptimize(
        pointTriangleResult.acceptedEndpointSquaredDistances.data());
    benchmark::DoNotOptimize(edgeEdgeResult.acceptedEdgeAIndices.data());
    benchmark::DoNotOptimize(
        edgeEdgeResult.acceptedEndpointSquaredDistances.data());
  }

  if (pointTriangleResult.acceptedEndpointSquaredDistances.size()
      == fixture.sweep.pointTriangle.cpuAcceptedEndpointSquaredDistances
             .size()) {
    for (std::size_t i = 0; i < fixture.sweep.pointTriangle
                                    .cpuAcceptedEndpointSquaredDistances.size();
         ++i) {
      maxError = std::max(
          maxError,
          std::abs(
              pointTriangleResult.acceptedEndpointSquaredDistances[i]
              - fixture.sweep.pointTriangle
                    .cpuAcceptedEndpointSquaredDistances[i]));
    }
  } else {
    maxError = std::numeric_limits<double>::infinity();
  }
  if (edgeEdgeResult.acceptedEndpointSquaredDistances.size()
      == fixture.sweep.edgeEdge.cpuAcceptedEndpointSquaredDistances.size()) {
    for (std::size_t i = 0;
         i < fixture.sweep.edgeEdge.cpuAcceptedEndpointSquaredDistances.size();
         ++i) {
      maxError = std::max(
          maxError,
          std::abs(
              edgeEdgeResult.acceptedEndpointSquaredDistances[i]
              - fixture.sweep.edgeEdge.cpuAcceptedEndpointSquaredDistances[i]));
    }
  } else {
    maxError = std::numeric_limits<double>::infinity();
  }

  recordCombinedSweptCandidateSweepCounters(state, fixture.sweep, maxError);
  state.counters["gpu_pair_capacity"] = static_cast<double>(
      pointTriangleResult.pairCapacity + edgeEdgeResult.pairCapacity);
  state.counters["gpu_point_triangle_pair_capacity"]
      = static_cast<double>(pointTriangleResult.pairCapacity);
  state.counters["gpu_edge_edge_pair_capacity"]
      = static_cast<double>(edgeEdgeResult.pairCapacity);
  state.counters["gpu_accepted_count"] = static_cast<double>(
      pointTriangleResult.acceptedCount + edgeEdgeResult.acceptedCount);
  state.counters["gpu_point_triangle_accepted_count"]
      = static_cast<double>(pointTriangleResult.acceptedCount);
  state.counters["gpu_edge_edge_accepted_count"]
      = static_cast<double>(edgeEdgeResult.acceptedCount);
  state.counters["gpu_point_triangle_compacted_count"]
      = static_cast<double>(pointTriangleResult.acceptedPointIndices.size());
  state.counters["gpu_point_triangle_compacted_triangle_count"]
      = static_cast<double>(pointTriangleResult.acceptedTriangleIndices.size());
  state.counters["gpu_point_triangle_compacted_distance_count"]
      = static_cast<double>(
          pointTriangleResult.acceptedEndpointSquaredDistances.size());
  state.counters["gpu_edge_edge_compacted_edge_a_count"]
      = static_cast<double>(edgeEdgeResult.acceptedEdgeAIndices.size());
  state.counters["gpu_edge_edge_compacted_edge_b_count"]
      = static_cast<double>(edgeEdgeResult.acceptedEdgeBIndices.size());
  state.counters["gpu_edge_edge_compacted_distance_count"]
      = static_cast<double>(
          edgeEdgeResult.acceptedEndpointSquaredDistances.size());
  state.counters["gpu_points"]
      = static_cast<double>(pointTriangleResult.pointCount);
  state.counters["gpu_triangles"]
      = static_cast<double>(pointTriangleResult.triangleCount);
  state.counters["gpu_edges"] = static_cast<double>(edgeEdgeResult.edgeCount);
  state.counters["host_setup_ns"]
      = pointTriangleResult.timing.setupNs + edgeEdgeResult.timing.setupNs;
  state.counters["host_to_device_ns"]
      = pointTriangleResult.timing.hostToDeviceNs
        + edgeEdgeResult.timing.hostToDeviceNs;
  state.counters["kernel_ns"]
      = pointTriangleResult.timing.kernelNs + edgeEdgeResult.timing.kernelNs;
  state.counters["device_to_host_ns"]
      = pointTriangleResult.timing.deviceToHostNs
        + edgeEdgeResult.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimeCombinedSweepFilterCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083RuntimePointTriangleCandidateBufferCpu(
    benchmark::State& state)
{
  const auto fixture = makeRuntimeSweepCandidateBufferFixture(
      static_cast<int>(state.range(0)));

  for (auto _ : state) {
    for (std::size_t i = 0; i < fixture.candidatePointIndices.size(); ++i) {
      const std::uint32_t point = fixture.candidatePointIndices[i];
      const std::size_t triangle = fixture.candidateTriangleIndices[i];
      const std::size_t tri = 3u * triangle;
      const auto startDistance = dc::pointTriangleSquaredDistance(
          pointAt(fixture.startPositions, point),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri]),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri + 1u]),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri + 2u]));
      const auto endDistance = dc::pointTriangleSquaredDistance(
          pointAt(fixture.endPositions, point),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri]),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri + 1u]),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri + 2u]));
      double endpointSquaredDistance = std::min(
          startDistance.squaredDistance, endDistance.squaredDistance);
      benchmark::DoNotOptimize(endpointSquaredDistance);
    }
  }

  recordRuntimePointTriangleCandidateBufferCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083RuntimePointTriangleCandidateBufferCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083RuntimePointTriangleCandidateBufferCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeRuntimeSweepCandidateBufferFixture(
      static_cast<int>(state.range(0)));
  cuda::SweptPointTriangleCandidateBufferResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluateSweptPointTriangleCandidateBufferCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.triangleIndices,
        fixture.candidatePointIndices,
        fixture.candidateTriangleIndices,
        result);
    benchmark::DoNotOptimize(result.endpointSquaredDistances.data());
  }

  for (std::size_t i = 0;
       i < fixture.pointTriangleEndpointSquaredDistances.size();
       ++i) {
    maxError = std::max(
        maxError,
        std::abs(
            result.endpointSquaredDistances[i]
            - fixture.pointTriangleEndpointSquaredDistances[i]));
  }

  recordRuntimePointTriangleCandidateBufferCounters(state, fixture, maxError);
  state.counters["gpu_candidates"] = static_cast<double>(result.pairCount);
  state.counters["gpu_points"] = static_cast<double>(result.pointCount);
  state.counters["gpu_triangles"] = static_cast<double>(result.triangleCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083RuntimePointTriangleCandidateBufferCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083RuntimeEdgeEdgeCandidateBufferCpu(benchmark::State& state)
{
  const auto fixture = makeRuntimeSweepCandidateBufferFixture(
      static_cast<int>(state.range(0)));

  for (auto _ : state) {
    for (std::size_t i = 0; i < fixture.candidateEdgeAIndices.size(); ++i) {
      const std::size_t edgeA = fixture.candidateEdgeAIndices[i];
      const std::size_t edgeB = fixture.candidateEdgeBIndices[i];
      const std::uint32_t a0 = fixture.edgeIndices[2u * edgeA];
      const std::uint32_t a1 = fixture.edgeIndices[2u * edgeA + 1u];
      const std::uint32_t b0 = fixture.edgeIndices[2u * edgeB];
      const std::uint32_t b1 = fixture.edgeIndices[2u * edgeB + 1u];
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
      double endpointSquaredDistance = std::min(
          startDistance.squaredDistance, endDistance.squaredDistance);
      benchmark::DoNotOptimize(endpointSquaredDistance);
    }
  }

  recordRuntimeEdgeEdgeCandidateBufferCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083RuntimeEdgeEdgeCandidateBufferCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083RuntimeEdgeEdgeCandidateBufferCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeRuntimeSweepCandidateBufferFixture(
      static_cast<int>(state.range(0)));
  cuda::SweptEdgeEdgeCandidateBufferResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluateSweptEdgeEdgeCandidateBufferCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.edgeIndices,
        fixture.candidateEdgeAIndices,
        fixture.candidateEdgeBIndices,
        result);
    benchmark::DoNotOptimize(result.endpointSquaredDistances.data());
  }

  for (std::size_t i = 0; i < fixture.edgeEdgeEndpointSquaredDistances.size();
       ++i) {
    maxError = std::max(
        maxError,
        std::abs(
            result.endpointSquaredDistances[i]
            - fixture.edgeEdgeEndpointSquaredDistances[i]));
  }

  recordRuntimeEdgeEdgeCandidateBufferCounters(state, fixture, maxError);
  state.counters["gpu_candidates"] = static_cast<double>(result.pairCount);
  state.counters["gpu_edges"] = static_cast<double>(result.edgeCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083RuntimeEdgeEdgeCandidateBufferCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimePointTriangleCandidateBufferCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeCandidateBufferFixture(
      static_cast<int>(state.range(0)));

  for (auto _ : state) {
    for (std::size_t i = 0; i < fixture.candidatePointIndices.size(); ++i) {
      const std::uint32_t point = fixture.candidatePointIndices[i];
      const std::size_t triangle = fixture.candidateTriangleIndices[i];
      const std::size_t tri = 3u * triangle;
      const auto startDistance = dc::pointTriangleSquaredDistance(
          pointAt(fixture.startPositions, point),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri]),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri + 1u]),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri + 2u]));
      const auto endDistance = dc::pointTriangleSquaredDistance(
          pointAt(fixture.endPositions, point),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri]),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri + 1u]),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri + 2u]));
      double endpointSquaredDistance = std::min(
          startDistance.squaredDistance, endDistance.squaredDistance);
      benchmark::DoNotOptimize(endpointSquaredDistance);
    }
  }

  recordRuntimePointTriangleCandidateBufferCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimePointTriangleCandidateBufferCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimePointTriangleCandidateBufferCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeCandidateBufferFixture(
      static_cast<int>(state.range(0)));
  cuda::SweptPointTriangleCandidateBufferResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluateSweptPointTriangleCandidateBufferCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.triangleIndices,
        fixture.candidatePointIndices,
        fixture.candidateTriangleIndices,
        result);
    benchmark::DoNotOptimize(result.endpointSquaredDistances.data());
  }

  for (std::size_t i = 0;
       i < fixture.pointTriangleEndpointSquaredDistances.size();
       ++i) {
    maxError = std::max(
        maxError,
        std::abs(
            result.endpointSquaredDistances[i]
            - fixture.pointTriangleEndpointSquaredDistances[i]));
  }

  recordRuntimePointTriangleCandidateBufferCounters(state, fixture, maxError);
  state.counters["gpu_candidates"] = static_cast<double>(result.pairCount);
  state.counters["gpu_points"] = static_cast<double>(result.pointCount);
  state.counters["gpu_triangles"] = static_cast<double>(result.triangleCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimePointTriangleCandidateBufferCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeEdgeEdgeCandidateBufferCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeCandidateBufferFixture(
      static_cast<int>(state.range(0)));

  for (auto _ : state) {
    for (std::size_t i = 0; i < fixture.candidateEdgeAIndices.size(); ++i) {
      const std::size_t edgeA = fixture.candidateEdgeAIndices[i];
      const std::size_t edgeB = fixture.candidateEdgeBIndices[i];
      const std::uint32_t a0 = fixture.edgeIndices[2u * edgeA];
      const std::uint32_t a1 = fixture.edgeIndices[2u * edgeA + 1u];
      const std::uint32_t b0 = fixture.edgeIndices[2u * edgeB];
      const std::uint32_t b1 = fixture.edgeIndices[2u * edgeB + 1u];
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
      double endpointSquaredDistance = std::min(
          startDistance.squaredDistance, endDistance.squaredDistance);
      benchmark::DoNotOptimize(endpointSquaredDistance);
    }
  }

  recordRuntimeEdgeEdgeCandidateBufferCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimeEdgeEdgeCandidateBufferCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeEdgeEdgeCandidateBufferCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeCandidateBufferFixture(
      static_cast<int>(state.range(0)));
  cuda::SweptEdgeEdgeCandidateBufferResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluateSweptEdgeEdgeCandidateBufferCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.edgeIndices,
        fixture.candidateEdgeAIndices,
        fixture.candidateEdgeBIndices,
        result);
    benchmark::DoNotOptimize(result.endpointSquaredDistances.data());
  }

  for (std::size_t i = 0; i < fixture.edgeEdgeEndpointSquaredDistances.size();
       ++i) {
    maxError = std::max(
        maxError,
        std::abs(
            result.endpointSquaredDistances[i]
            - fixture.edgeEdgeEndpointSquaredDistances[i]));
  }

  recordRuntimeEdgeEdgeCandidateBufferCounters(state, fixture, maxError);
  state.counters["gpu_candidates"] = static_cast<double>(result.pairCount);
  state.counters["gpu_edges"] = static_cast<double>(result.edgeCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimeEdgeEdgeCandidateBufferCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeCombinedCandidateBufferCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeCandidateBufferFixture(
      static_cast<int>(state.range(0)));

  std::size_t candidateCount = 0;
  for (auto _ : state) {
    candidateCount = 0;
    for (std::size_t i = 0; i < fixture.candidatePointIndices.size(); ++i) {
      const std::uint32_t point = fixture.candidatePointIndices[i];
      const std::size_t triangle = fixture.candidateTriangleIndices[i];
      const std::size_t tri = 3u * triangle;
      const auto startDistance = dc::pointTriangleSquaredDistance(
          pointAt(fixture.startPositions, point),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri]),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri + 1u]),
          pointAt(fixture.startPositions, fixture.triangleIndices[tri + 2u]));
      const auto endDistance = dc::pointTriangleSquaredDistance(
          pointAt(fixture.endPositions, point),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri]),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri + 1u]),
          pointAt(fixture.endPositions, fixture.triangleIndices[tri + 2u]));
      double endpointSquaredDistance = std::min(
          startDistance.squaredDistance, endDistance.squaredDistance);
      benchmark::DoNotOptimize(endpointSquaredDistance);
      ++candidateCount;
    }
    for (std::size_t i = 0; i < fixture.candidateEdgeAIndices.size(); ++i) {
      const std::size_t edgeA = fixture.candidateEdgeAIndices[i];
      const std::size_t edgeB = fixture.candidateEdgeBIndices[i];
      const std::uint32_t a0 = fixture.edgeIndices[2u * edgeA];
      const std::uint32_t a1 = fixture.edgeIndices[2u * edgeA + 1u];
      const std::uint32_t b0 = fixture.edgeIndices[2u * edgeB];
      const std::uint32_t b1 = fixture.edgeIndices[2u * edgeB + 1u];
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
      double endpointSquaredDistance = std::min(
          startDistance.squaredDistance, endDistance.squaredDistance);
      benchmark::DoNotOptimize(endpointSquaredDistance);
      ++candidateCount;
    }
  }

  benchmark::DoNotOptimize(candidateCount);
  recordCombinedRuntimeCandidateBufferCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimeCombinedCandidateBufferCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeCombinedCandidateBufferCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeCandidateBufferFixture(
      static_cast<int>(state.range(0)));
  cuda::SweptPointTriangleCandidateBufferResult pointTriangleResult;
  cuda::SweptEdgeEdgeCandidateBufferResult edgeEdgeResult;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluateSweptPointTriangleCandidateBufferCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.triangleIndices,
        fixture.candidatePointIndices,
        fixture.candidateTriangleIndices,
        pointTriangleResult);
    cuda::evaluateSweptEdgeEdgeCandidateBufferCuda(
        fixture.startPositions,
        fixture.endPositions,
        fixture.edgeIndices,
        fixture.candidateEdgeAIndices,
        fixture.candidateEdgeBIndices,
        edgeEdgeResult);
    benchmark::DoNotOptimize(
        pointTriangleResult.endpointSquaredDistances.data());
    benchmark::DoNotOptimize(edgeEdgeResult.endpointSquaredDistances.data());
  }

  if (pointTriangleResult.endpointSquaredDistances.size()
      == fixture.pointTriangleEndpointSquaredDistances.size()) {
    for (std::size_t i = 0;
         i < fixture.pointTriangleEndpointSquaredDistances.size();
         ++i) {
      maxError = std::max(
          maxError,
          std::abs(
              pointTriangleResult.endpointSquaredDistances[i]
              - fixture.pointTriangleEndpointSquaredDistances[i]));
    }
  } else {
    maxError = std::numeric_limits<double>::infinity();
  }
  if (edgeEdgeResult.endpointSquaredDistances.size()
      == fixture.edgeEdgeEndpointSquaredDistances.size()) {
    for (std::size_t i = 0; i < fixture.edgeEdgeEndpointSquaredDistances.size();
         ++i) {
      maxError = std::max(
          maxError,
          std::abs(
              edgeEdgeResult.endpointSquaredDistances[i]
              - fixture.edgeEdgeEndpointSquaredDistances[i]));
    }
  } else {
    maxError = std::numeric_limits<double>::infinity();
  }

  recordCombinedRuntimeCandidateBufferCounters(state, fixture, maxError);
  state.counters["gpu_candidates"] = static_cast<double>(
      pointTriangleResult.pairCount + edgeEdgeResult.pairCount);
  state.counters["gpu_point_triangle_candidates"]
      = static_cast<double>(pointTriangleResult.pairCount);
  state.counters["gpu_edge_edge_candidates"]
      = static_cast<double>(edgeEdgeResult.pairCount);
  state.counters["gpu_points"]
      = static_cast<double>(pointTriangleResult.pointCount);
  state.counters["gpu_triangles"]
      = static_cast<double>(pointTriangleResult.triangleCount);
  state.counters["gpu_edges"] = static_cast<double>(edgeEdgeResult.edgeCount);
  state.counters["host_setup_ns"]
      = pointTriangleResult.timing.setupNs + edgeEdgeResult.timing.setupNs;
  state.counters["host_to_device_ns"]
      = pointTriangleResult.timing.hostToDeviceNs
        + edgeEdgeResult.timing.hostToDeviceNs;
  state.counters["kernel_ns"]
      = pointTriangleResult.timing.kernelNs + edgeEdgeResult.timing.kernelNs;
  state.counters["device_to_host_ns"]
      = pointTriangleResult.timing.deviceToHostNs
        + edgeEdgeResult.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimeCombinedCandidateBufferCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeFilteredCandidateBufferCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneCombinedSweptCandidateSweepFixture(
      static_cast<int>(state.range(0)), true);

  std::size_t candidateCount = 0;
  for (auto _ : state) {
    SweptCandidateSweepFixture pointTriangleFixture;
    pointTriangleFixture.startPositions
        = fixture.sweep.pointTriangle.startPositions;
    pointTriangleFixture.endPositions
        = fixture.sweep.pointTriangle.endPositions;
    pointTriangleFixture.pointIndices
        = fixture.sweep.pointTriangle.pointIndices;
    pointTriangleFixture.triangles = fixture.sweep.pointTriangle.triangles;
    pointTriangleFixture.pointCount = fixture.sweep.pointTriangle.pointCount;
    pointTriangleFixture.triangleCount
        = fixture.sweep.pointTriangle.triangleCount;
    pointTriangleFixture.pairCapacity
        = fixture.sweep.pointTriangle.pairCapacity;
    pointTriangleFixture.sceneBodyCount
        = fixture.sweep.pointTriangle.sceneBodyCount;
    appendSweptPointTriangleSweepCandidates(pointTriangleFixture);

    SweptEdgeEdgeSweepFixture edgeEdgeFixture;
    edgeEdgeFixture.startPositions = fixture.sweep.edgeEdge.startPositions;
    edgeEdgeFixture.endPositions = fixture.sweep.edgeEdge.endPositions;
    edgeEdgeFixture.edgeIndices = fixture.sweep.edgeEdge.edgeIndices;
    edgeEdgeFixture.edgeCount = fixture.sweep.edgeEdge.edgeCount;
    edgeEdgeFixture.pairCapacity = fixture.sweep.edgeEdge.pairCapacity;
    edgeEdgeFixture.sceneBodyCount = fixture.sweep.edgeEdge.sceneBodyCount;
    appendSweptEdgeEdgeSweepCandidates(edgeEdgeFixture);

    candidateCount
        = pointTriangleFixture.acceptedCount + edgeEdgeFixture.acceptedCount;
    benchmark::DoNotOptimize(
        pointTriangleFixture.cpuAcceptedEndpointSquaredDistances.data());
    benchmark::DoNotOptimize(
        edgeEdgeFixture.cpuAcceptedEndpointSquaredDistances.data());
  }

  benchmark::DoNotOptimize(candidateCount);
  recordFilteredCombinedCandidateBufferCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimeFilteredCandidateBufferCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeFilteredCandidateBufferCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneCombinedSweptCandidateSweepFixture(
      static_cast<int>(state.range(0)), true);
  cuda::SweptPointTriangleSweepResult pointTriangleSweepResult;
  cuda::SweptEdgeEdgeSweepResult edgeEdgeSweepResult;
  cuda::SweptPointTriangleCandidateBufferResult pointTriangleBufferResult;
  cuda::SweptEdgeEdgeCandidateBufferResult edgeEdgeBufferResult;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::buildSweptPointTriangleSweepCuda(
        fixture.sweep.pointTriangle.startPositions,
        fixture.sweep.pointTriangle.endPositions,
        fixture.sweep.pointTriangle.pointIndices,
        fixture.sweep.pointTriangle.triangles,
        kActivationDistance,
        pointTriangleSweepResult);
    cuda::buildSweptEdgeEdgeSweepCuda(
        fixture.sweep.edgeEdge.startPositions,
        fixture.sweep.edgeEdge.endPositions,
        fixture.sweep.edgeEdge.edgeIndices,
        kActivationDistance,
        edgeEdgeSweepResult);
    cuda::evaluateSweptPointTriangleCandidateBufferCuda(
        fixture.sweep.pointTriangle.startPositions,
        fixture.sweep.pointTriangle.endPositions,
        fixture.sweep.pointTriangle.triangles,
        pointTriangleSweepResult.acceptedPointIndices,
        pointTriangleSweepResult.acceptedTriangleIndices,
        pointTriangleBufferResult);
    cuda::evaluateSweptEdgeEdgeCandidateBufferCuda(
        fixture.sweep.edgeEdge.startPositions,
        fixture.sweep.edgeEdge.endPositions,
        fixture.sweep.edgeEdge.edgeIndices,
        edgeEdgeSweepResult.acceptedEdgeAIndices,
        edgeEdgeSweepResult.acceptedEdgeBIndices,
        edgeEdgeBufferResult);
    benchmark::DoNotOptimize(
        pointTriangleBufferResult.endpointSquaredDistances.data());
    benchmark::DoNotOptimize(
        edgeEdgeBufferResult.endpointSquaredDistances.data());
  }

  maxError = updateMaxDistanceError(
      pointTriangleBufferResult.endpointSquaredDistances,
      fixture.sweep.pointTriangle.cpuAcceptedEndpointSquaredDistances,
      maxError);
  maxError = updateMaxDistanceError(
      edgeEdgeBufferResult.endpointSquaredDistances,
      fixture.sweep.edgeEdge.cpuAcceptedEndpointSquaredDistances,
      maxError);

  recordFilteredCombinedCandidateBufferCounters(state, fixture, maxError);
  state.counters["gpu_pair_capacity"] = static_cast<double>(
      pointTriangleSweepResult.pairCapacity + edgeEdgeSweepResult.pairCapacity);
  state.counters["gpu_point_triangle_pair_capacity"]
      = static_cast<double>(pointTriangleSweepResult.pairCapacity);
  state.counters["gpu_edge_edge_pair_capacity"]
      = static_cast<double>(edgeEdgeSweepResult.pairCapacity);
  state.counters["gpu_candidate_count"] = static_cast<double>(
      pointTriangleBufferResult.pairCount + edgeEdgeBufferResult.pairCount);
  state.counters["gpu_accepted_count"] = static_cast<double>(
      pointTriangleSweepResult.acceptedCount
      + edgeEdgeSweepResult.acceptedCount);
  state.counters["gpu_rejected_count"] = static_cast<double>(
      pointTriangleSweepResult.pairCapacity + edgeEdgeSweepResult.pairCapacity
      - pointTriangleSweepResult.acceptedCount
      - edgeEdgeSweepResult.acceptedCount);
  state.counters["gpu_point_triangle_candidate_count"]
      = static_cast<double>(pointTriangleBufferResult.pairCount);
  state.counters["gpu_edge_edge_candidate_count"]
      = static_cast<double>(edgeEdgeBufferResult.pairCount);
  state.counters["gpu_point_triangle_rejected_count"] = static_cast<double>(
      pointTriangleSweepResult.pairCapacity
      - pointTriangleSweepResult.acceptedCount);
  state.counters["gpu_edge_edge_rejected_count"] = static_cast<double>(
      edgeEdgeSweepResult.pairCapacity - edgeEdgeSweepResult.acceptedCount);
  state.counters["gpu_points"]
      = static_cast<double>(pointTriangleBufferResult.pointCount);
  state.counters["gpu_triangles"]
      = static_cast<double>(pointTriangleBufferResult.triangleCount);
  state.counters["gpu_edges"]
      = static_cast<double>(edgeEdgeBufferResult.edgeCount);
  state.counters["host_setup_ns"] = pointTriangleSweepResult.timing.setupNs
                                    + edgeEdgeSweepResult.timing.setupNs
                                    + pointTriangleBufferResult.timing.setupNs
                                    + edgeEdgeBufferResult.timing.setupNs;
  state.counters["host_to_device_ns"]
      = pointTriangleSweepResult.timing.hostToDeviceNs
        + edgeEdgeSweepResult.timing.hostToDeviceNs
        + pointTriangleBufferResult.timing.hostToDeviceNs
        + edgeEdgeBufferResult.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = pointTriangleSweepResult.timing.kernelNs
                                + edgeEdgeSweepResult.timing.kernelNs
                                + pointTriangleBufferResult.timing.kernelNs
                                + edgeEdgeBufferResult.timing.kernelNs;
  state.counters["device_to_host_ns"]
      = pointTriangleSweepResult.timing.deviceToHostNs
        + edgeEdgeSweepResult.timing.deviceToHostNs
        + pointTriangleBufferResult.timing.deviceToHostNs
        + edgeEdgeBufferResult.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimeFilteredCandidateBufferCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083EdgeEdgeContactCandidateCpu(benchmark::State& state)
{
  const auto fixture
      = makeEdgeEdgeCandidateFixture(static_cast<int>(state.range(0)));

  std::size_t acceptedCount = 0;
  for (auto _ : state) {
    acceptedCount = 0;
    for (const auto& stencil : fixture.stencils) {
      const auto distance = dc::edgeEdgeSquaredDistance(
          pointAt(fixture.positions, stencil.edgeAStart),
          pointAt(fixture.positions, stencil.edgeAEnd),
          pointAt(fixture.positions, stencil.edgeBStart),
          pointAt(fixture.positions, stencil.edgeBEnd));
      acceptedCount += dc::detail::withinActivationDistance(
                           distance.squaredDistance, kActivationDistance)
                           ? 1u
                           : 0u;
      double squaredDistance = distance.squaredDistance;
      benchmark::DoNotOptimize(squaredDistance);
    }
  }

  benchmark::DoNotOptimize(acceptedCount);
  recordSharedCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083EdgeEdgeContactCandidateCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083EdgeEdgeContactCandidateCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeEdgeEdgeCandidateFixture(static_cast<int>(state.range(0)));
  cuda::EdgeEdgeCandidateFilterResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::filterEdgeEdgeContactStencilsCuda(
        fixture.positions, fixture.stencils, kActivationDistance, result);
    benchmark::DoNotOptimize(result.squaredDistances.data());
    benchmark::DoNotOptimize(result.accepted.data());
  }

  for (std::size_t i = 0; i < fixture.cpuSquaredDistances.size(); ++i) {
    maxError = std::max(
        maxError,
        std::abs(result.squaredDistances[i] - fixture.cpuSquaredDistances[i]));
  }

  recordSharedCounters(state, fixture, maxError);
  state.counters["gpu_accepted_count"]
      = static_cast<double>(result.acceptedCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083EdgeEdgeContactCandidateCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(1048576)
    ->UseRealTime();
