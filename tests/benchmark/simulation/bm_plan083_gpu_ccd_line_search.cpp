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
 *     copyright notice, this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/detail/deformable_contact/candidate_set.hpp>
#include <dart/simulation/detail/deformable_contact/continuous_collision_step.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/ccd_line_search_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <algorithm>
#include <vector>

#include <cmath>
#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;
namespace dc = dart::simulation::detail::deformable_contact;
namespace sx = dart::simulation;

namespace {

constexpr std::size_t kRigidCurvedSegmentCount = 8;
constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kSceneRuntimeCandidateActivationDistance = 0.05;
constexpr double kSceneRuntimeCcdTimeStep = 1.0;

struct CcdFixture
{
  std::vector<cuda::PointTriangleCcdLineSearchPair> pairs;
  std::vector<double> cpuStepBounds;
  std::vector<std::uint8_t> cpuHits;
  std::size_t hitCount = 0;
  double minStepBound = 1.0;
};

struct EdgeEdgeCcdFixture
{
  std::vector<cuda::EdgeEdgeCcdLineSearchPair> pairs;
  std::vector<double> cpuStepBounds;
  std::vector<std::uint8_t> cpuHits;
  std::size_t hitCount = 0;
  double minStepBound = 1.0;
};

struct SceneRuntimeCcdSurface
{
  std::vector<Eigen::Vector3d> start;
  std::vector<Eigen::Vector3d> end;
  std::vector<sx::DeformableSurfaceTriangle> triangles;
  dc::ContactCandidateSet candidates;
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
};

struct ScenePointTriangleCcdFixture : CcdFixture
{
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sourcePointTriangleCandidateCount = 0;
};

struct SceneEdgeEdgeCcdFixture : EdgeEdgeCcdFixture
{
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sourceEdgeEdgeCandidateCount = 0;
};

template <typename Pair>
struct RigidCurvedCcdFixture
{
  std::vector<Pair> segmentPairs;
  std::vector<double> cpuStepBounds;
  std::vector<std::uint8_t> cpuHits;
  std::size_t trajectoryCount = 0;
  std::size_t segmentCount = kRigidCurvedSegmentCount;
  std::size_t hitCount = 0;
  double minStepBound = 1.0;
};

void setVec3(double values[3], const Eigen::Vector3d& vector)
{
  for (int i = 0; i < 3; ++i) {
    values[i] = vector[i];
  }
}

Eigen::Vector3d vec3(const double values[3])
{
  return {values[0], values[1], values[2]};
}

cuda::PointTriangleCcdLineSearchPair makePair(
    const Eigen::Vector3d& pointStart,
    const Eigen::Vector3d& pointEnd,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  cuda::PointTriangleCcdLineSearchPair pair;
  setVec3(pair.pointStart, pointStart);
  setVec3(pair.pointEnd, pointEnd);
  setVec3(pair.triangleA, a);
  setVec3(pair.triangleB, b);
  setVec3(pair.triangleC, c);
  return pair;
}

cuda::EdgeEdgeCcdLineSearchPair makeEdgeEdgePair(
    const Eigen::Vector3d& edgeA0Start,
    const Eigen::Vector3d& edgeA0End,
    const Eigen::Vector3d& edgeA1Start,
    const Eigen::Vector3d& edgeA1End,
    const Eigen::Vector3d& edgeB0Start,
    const Eigen::Vector3d& edgeB0End,
    const Eigen::Vector3d& edgeB1Start,
    const Eigen::Vector3d& edgeB1End)
{
  cuda::EdgeEdgeCcdLineSearchPair pair;
  setVec3(pair.edgeA0Start, edgeA0Start);
  setVec3(pair.edgeA0End, edgeA0End);
  setVec3(pair.edgeA1Start, edgeA1Start);
  setVec3(pair.edgeA1End, edgeA1End);
  setVec3(pair.edgeB0Start, edgeB0Start);
  setVec3(pair.edgeB0End, edgeB0End);
  setVec3(pair.edgeB1Start, edgeB1Start);
  setVec3(pair.edgeB1End, edgeB1End);
  return pair;
}

dc::ContinuousCollisionStepResult cpuResult(
    const cuda::PointTriangleCcdLineSearchPair& pair)
{
  return dc::pointTriangleStepBound(
      vec3(pair.pointStart),
      vec3(pair.pointEnd),
      vec3(pair.triangleA),
      vec3(pair.triangleA),
      vec3(pair.triangleB),
      vec3(pair.triangleB),
      vec3(pair.triangleC),
      vec3(pair.triangleC));
}

dc::ContinuousCollisionStepResult cpuResult(
    const cuda::EdgeEdgeCcdLineSearchPair& pair)
{
  return dc::edgeEdgeStepBound(
      vec3(pair.edgeA0Start),
      vec3(pair.edgeA0End),
      vec3(pair.edgeA1Start),
      vec3(pair.edgeA1End),
      vec3(pair.edgeB0Start),
      vec3(pair.edgeB0End),
      vec3(pair.edgeB1Start),
      vec3(pair.edgeB1End));
}

template <typename Fixture>
void populateCpuPairResults(Fixture& fixture)
{
  fixture.cpuStepBounds.clear();
  fixture.cpuHits.clear();
  fixture.cpuStepBounds.reserve(fixture.pairs.size());
  fixture.cpuHits.reserve(fixture.pairs.size());
  fixture.hitCount = 0;
  fixture.minStepBound = 1.0;

  for (const auto& pair : fixture.pairs) {
    const auto result = cpuResult(pair);
    fixture.cpuStepBounds.push_back(result.stepBound);
    fixture.cpuHits.push_back(result.hit ? 1u : 0u);
    if (result.hit) {
      ++fixture.hitCount;
      fixture.minStepBound = std::min(fixture.minStepBound, result.stepBound);
    }
  }
}

double smoothAlpha(const double alpha)
{
  return 0.5 - 0.5 * std::cos(kPi * alpha);
}

Eigen::Vector3d sampledRigidCurvedPoint(
    const double baseX, const double baseY, const double alpha, const bool hit)
{
  const double eased = smoothAlpha(alpha);
  const double lateralPhase = (5.0 * kPi / 1.125) * alpha;
  const double lateral = 0.035 * std::sin(lateralPhase);
  const double z = hit ? 0.32 * (0.5625 - alpha) : 0.18 - 0.08 * eased;
  return {baseX + lateral, baseY + 0.025 * std::sin(lateralPhase), z};
}

Eigen::Vector3d sampledRigidCurvedEdgeEndpoint(
    const double baseY,
    const double alpha,
    const bool hit,
    const double halfLengthSign)
{
  const double eased = smoothAlpha(alpha);
  const double centerX = -0.45 + (hit ? 1.0 : 0.25) * eased;
  const double theta = 0.18 * std::sin(2.0 * kPi * alpha);
  const Eigen::Vector3d direction(std::sin(theta), std::cos(theta), 0.0);
  const Eigen::Vector3d center(centerX, baseY, 0.0);
  return center + (halfLengthSign * 0.21) * direction;
}

template <typename Pair>
void evaluateTrajectorySegments(
    const std::vector<Pair>& segmentPairs,
    const std::size_t trajectoryCount,
    const std::size_t segmentCount,
    std::vector<double>& stepBounds,
    std::vector<std::uint8_t>& hits,
    std::size_t& hitCount,
    double& minStepBound)
{
  stepBounds.assign(trajectoryCount, 1.0);
  hits.assign(trajectoryCount, 0u);
  hitCount = 0;
  minStepBound = 1.0;

  for (std::size_t trajectory = 0; trajectory < trajectoryCount; ++trajectory) {
    for (std::size_t segment = 0; segment < segmentCount; ++segment) {
      const std::size_t pairIndex = trajectory * segmentCount + segment;
      const auto result = cpuResult(segmentPairs[pairIndex]);
      if (!result.hit) {
        continue;
      }

      const double localStep = std::clamp(result.stepBound, 0.0, 1.0);
      const double trajectoryStep = (static_cast<double>(segment) + localStep)
                                    / static_cast<double>(segmentCount);
      stepBounds[trajectory] = trajectoryStep;
      hits[trajectory] = 1u;
      ++hitCount;
      minStepBound = std::min(minStepBound, trajectoryStep);
      break;
    }
  }
}

template <typename Result>
void reduceGpuTrajectorySegments(
    const Result& result,
    const std::size_t trajectoryCount,
    const std::size_t segmentCount,
    std::vector<double>& stepBounds,
    std::vector<std::uint8_t>& hits,
    std::size_t& hitCount,
    double& minStepBound)
{
  stepBounds.assign(trajectoryCount, 1.0);
  hits.assign(trajectoryCount, 0u);
  hitCount = 0;
  minStepBound = 1.0;

  for (std::size_t trajectory = 0; trajectory < trajectoryCount; ++trajectory) {
    for (std::size_t segment = 0; segment < segmentCount; ++segment) {
      const std::size_t pairIndex = trajectory * segmentCount + segment;
      if (result.hits[pairIndex] == 0u) {
        continue;
      }

      const double localStep
          = std::clamp(result.stepBounds[pairIndex], 0.0, 1.0);
      const double trajectoryStep = (static_cast<double>(segment) + localStep)
                                    / static_cast<double>(segmentCount);
      stepBounds[trajectory] = trajectoryStep;
      hits[trajectory] = 1u;
      ++hitCount;
      minStepBound = std::min(minStepBound, trajectoryStep);
      break;
    }
  }
}

double maxAbsDifference(
    const std::vector<double>& lhs, const std::vector<double>& rhs)
{
  double maxError = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    maxError = std::max(maxError, std::abs(lhs[i] - rhs[i]));
  }
  return maxError;
}

CcdFixture makeCcdFixture(const int pairCount)
{
  CcdFixture fixture;
  fixture.pairs.reserve(static_cast<std::size_t>(pairCount));
  fixture.cpuStepBounds.reserve(static_cast<std::size_t>(pairCount));
  fixture.cpuHits.reserve(static_cast<std::size_t>(pairCount));

  const Eigen::Vector3d a(-1.0, -1.0, 0.0);
  const Eigen::Vector3d b(1.0, -1.0, 0.0);
  const Eigen::Vector3d c(0.0, 1.0, 0.0);
  for (int i = 0; i < pairCount; ++i) {
    const double x = static_cast<double>((i % 8) - 4) * 0.05;
    const double y = static_cast<double>(((i / 8) % 8) - 4) * 0.05;
    const bool hit = (i % 4) != 0;
    const double startZ = (i % 8) < 4 ? 0.2 : -0.2;
    const double endZ = hit ? -startZ : 0.5 * startZ;
    if ((i % 2) == 0) {
      fixture.pairs.push_back(makePair({x, y, startZ}, {x, y, endZ}, a, b, c));
    } else {
      fixture.pairs.push_back(makePair({x, y, startZ}, {x, y, endZ}, a, c, b));
    }
  }

  populateCpuPairResults(fixture);
  return fixture;
}

EdgeEdgeCcdFixture makeEdgeEdgeCcdFixture(const int pairCount)
{
  EdgeEdgeCcdFixture fixture;
  fixture.pairs.reserve(static_cast<std::size_t>(pairCount));
  fixture.cpuStepBounds.reserve(static_cast<std::size_t>(pairCount));
  fixture.cpuHits.reserve(static_cast<std::size_t>(pairCount));

  const Eigen::Vector3d b0(0.0, -0.5, 0.0);
  const Eigen::Vector3d b1(0.0, 0.5, 0.0);
  for (int i = 0; i < pairCount; ++i) {
    const double y = static_cast<double>((i % 8) - 4) * 0.02;
    const bool hit = (i % 4) != 0;
    const double endX = hit ? 0.5 : -0.25;
    fixture.pairs.push_back(makeEdgeEdgePair(
        {-0.5, y, 0.0},
        {endX, y, 0.0},
        {-0.5, y + 0.4, 0.0},
        {endX, y + 0.4, 0.0},
        b0,
        b0,
        b1,
        b1));
  }

  populateCpuPairResults(fixture);
  return fixture;
}

sx::DeformableBodyOptions makeSceneRuntimeCcdBodyOptions(const int sampleCount)
{
  const int groupCount = std::max(1, static_cast<int>(std::sqrt(sampleCount)));
  sx::DeformableBodyOptions options;
  options.positions.reserve(static_cast<std::size_t>(10 * groupCount));
  options.velocities.reserve(static_cast<std::size_t>(10 * groupCount));
  options.masses.reserve(static_cast<std::size_t>(10 * groupCount));
  options.surfaceTriangles.reserve(static_cast<std::size_t>(3 * groupCount));

  const auto appendSceneNode =
      [&options](const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
        options.positions.push_back(start);
        options.velocities.push_back((end - start) / kSceneRuntimeCcdTimeStep);
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

SceneRuntimeCcdSurface makeSceneRuntimeCcdSurface(const int sampleCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeCcdTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_ccd", makeSceneRuntimeCcdBodyOptions(sampleCount));

  SceneRuntimeCcdSurface surface;
  surface.sceneBodyCount = world.getDeformableBodyCount();
  surface.sceneNodeCount = body.getNodeCount();
  surface.sceneTriangleCount = body.getSurfaceTriangleCount();
  surface.start.reserve(body.getNodeCount());
  surface.end.reserve(body.getNodeCount());
  surface.triangles.reserve(body.getSurfaceTriangleCount());

  for (std::size_t node = 0; node < body.getNodeCount(); ++node) {
    surface.start.push_back(body.getPosition(node));
    surface.end.push_back(
        surface.start.back() + world.getTimeStep() * body.getVelocity(node));
  }
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    surface.triangles.push_back(body.getSurfaceTriangle(triangle));
  }

  dc::ContactCandidateOptions options;
  options.activationDistance = kSceneRuntimeCandidateActivationDistance;
  surface.candidates = dc::buildMotionAwareContactCandidatesSweep(
      surface.start, surface.end, surface.triangles, options);
  return surface;
}

bool isStaticNode(
    const SceneRuntimeCcdSurface& surface, const std::size_t nodeIndex)
{
  return (surface.end[nodeIndex] - surface.start[nodeIndex]).squaredNorm()
         <= 1e-24;
}

bool isStaticTriangle(
    const SceneRuntimeCcdSurface& surface,
    const sx::DeformableSurfaceTriangle& triangle)
{
  return isStaticNode(surface, triangle.nodeA)
         && isStaticNode(surface, triangle.nodeB)
         && isStaticNode(surface, triangle.nodeC);
}

ScenePointTriangleCcdFixture makeSceneRuntimePointTriangleCcdFixture(
    const int sampleCount)
{
  const SceneRuntimeCcdSurface surface
      = makeSceneRuntimeCcdSurface(sampleCount);

  ScenePointTriangleCcdFixture fixture;
  fixture.sceneBodyCount = surface.sceneBodyCount;
  fixture.sceneNodeCount = surface.sceneNodeCount;
  fixture.sceneTriangleCount = surface.sceneTriangleCount;
  fixture.sourcePointTriangleCandidateCount
      = surface.candidates.pointTriangleCandidates.size();
  fixture.pairs.reserve(surface.candidates.pointTriangleCandidates.size());

  for (const auto& candidate : surface.candidates.pointTriangleCandidates) {
    const auto& triangle = surface.triangles[candidate.triangle];
    if (!isStaticTriangle(surface, triangle)) {
      continue;
    }

    fixture.pairs.push_back(makePair(
        surface.start[candidate.point],
        surface.end[candidate.point],
        surface.start[triangle.nodeA],
        surface.start[triangle.nodeB],
        surface.start[triangle.nodeC]));
  }

  populateCpuPairResults(fixture);
  return fixture;
}

SceneEdgeEdgeCcdFixture makeSceneRuntimeEdgeEdgeCcdFixture(
    const int sampleCount)
{
  const SceneRuntimeCcdSurface surface
      = makeSceneRuntimeCcdSurface(sampleCount);

  SceneEdgeEdgeCcdFixture fixture;
  fixture.sceneBodyCount = surface.sceneBodyCount;
  fixture.sceneNodeCount = surface.sceneNodeCount;
  fixture.sceneTriangleCount = surface.sceneTriangleCount;
  fixture.sourceEdgeEdgeCandidateCount
      = surface.candidates.edgeEdgeCandidates.size();
  fixture.pairs.reserve(surface.candidates.edgeEdgeCandidates.size());

  for (const auto& candidate : surface.candidates.edgeEdgeCandidates) {
    const auto& edgeA = surface.candidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = surface.candidates.surfaceEdges[candidate.edgeB];
    fixture.pairs.push_back(makeEdgeEdgePair(
        surface.start[edgeA.nodeA],
        surface.end[edgeA.nodeA],
        surface.start[edgeA.nodeB],
        surface.end[edgeA.nodeB],
        surface.start[edgeB.nodeA],
        surface.end[edgeB.nodeA],
        surface.start[edgeB.nodeB],
        surface.end[edgeB.nodeB]));
  }

  populateCpuPairResults(fixture);
  return fixture;
}

RigidCurvedCcdFixture<cuda::PointTriangleCcdLineSearchPair>
makeRigidCurvedPointTriangleFixture(const int trajectoryCount)
{
  RigidCurvedCcdFixture<cuda::PointTriangleCcdLineSearchPair> fixture;
  fixture.trajectoryCount = static_cast<std::size_t>(trajectoryCount);
  fixture.segmentPairs.reserve(fixture.trajectoryCount * fixture.segmentCount);

  const Eigen::Vector3d a(-1.0, -1.0, 0.0);
  const Eigen::Vector3d b(1.0, -1.0, 0.0);
  const Eigen::Vector3d c(0.0, 1.0, 0.0);
  for (int i = 0; i < trajectoryCount; ++i) {
    const double baseX = static_cast<double>((i % 16) - 8) * 0.015;
    const double baseY = static_cast<double>(((i / 16) % 16) - 8) * 0.015;
    const bool hit = (i % 4) != 0;
    for (std::size_t segment = 0; segment < fixture.segmentCount; ++segment) {
      const double alpha0 = static_cast<double>(segment)
                            / static_cast<double>(fixture.segmentCount);
      const double alpha1 = static_cast<double>(segment + 1)
                            / static_cast<double>(fixture.segmentCount);
      const Eigen::Vector3d pointStart
          = sampledRigidCurvedPoint(baseX, baseY, alpha0, hit);
      const Eigen::Vector3d pointEnd
          = sampledRigidCurvedPoint(baseX, baseY, alpha1, hit);
      if ((i % 2) == 0) {
        fixture.segmentPairs.push_back(makePair(pointStart, pointEnd, a, b, c));
      } else {
        fixture.segmentPairs.push_back(makePair(pointStart, pointEnd, a, c, b));
      }
    }
  }

  evaluateTrajectorySegments(
      fixture.segmentPairs,
      fixture.trajectoryCount,
      fixture.segmentCount,
      fixture.cpuStepBounds,
      fixture.cpuHits,
      fixture.hitCount,
      fixture.minStepBound);

  return fixture;
}

RigidCurvedCcdFixture<cuda::EdgeEdgeCcdLineSearchPair>
makeRigidCurvedEdgeEdgeFixture(const int trajectoryCount)
{
  RigidCurvedCcdFixture<cuda::EdgeEdgeCcdLineSearchPair> fixture;
  fixture.trajectoryCount = static_cast<std::size_t>(trajectoryCount);
  fixture.segmentPairs.reserve(fixture.trajectoryCount * fixture.segmentCount);

  const Eigen::Vector3d b0(0.0, -0.55, 0.0);
  const Eigen::Vector3d b1(0.0, 0.55, 0.0);
  for (int i = 0; i < trajectoryCount; ++i) {
    const double baseY = static_cast<double>((i % 8) - 4) * 0.015;
    const bool hit = (i % 4) != 0;
    for (std::size_t segment = 0; segment < fixture.segmentCount; ++segment) {
      const double alpha0 = static_cast<double>(segment)
                            / static_cast<double>(fixture.segmentCount);
      const double alpha1 = static_cast<double>(segment + 1)
                            / static_cast<double>(fixture.segmentCount);
      fixture.segmentPairs.push_back(makeEdgeEdgePair(
          sampledRigidCurvedEdgeEndpoint(baseY, alpha0, hit, -1.0),
          sampledRigidCurvedEdgeEndpoint(baseY, alpha1, hit, -1.0),
          sampledRigidCurvedEdgeEndpoint(baseY, alpha0, hit, 1.0),
          sampledRigidCurvedEdgeEndpoint(baseY, alpha1, hit, 1.0),
          b0,
          b0,
          b1,
          b1));
    }
  }

  evaluateTrajectorySegments(
      fixture.segmentPairs,
      fixture.trajectoryCount,
      fixture.segmentCount,
      fixture.cpuStepBounds,
      fixture.cpuHits,
      fixture.hitCount,
      fixture.minStepBound);

  return fixture;
}

void recordSharedCounters(
    benchmark::State& state, const CcdFixture& fixture, const double maxError)
{
  state.counters["pairs"] = static_cast<double>(fixture.pairs.size());
  state.counters["hits"] = static_cast<double>(fixture.hitCount);
  state.counters["min_step_bound"] = fixture.minStepBound;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.pairs.size()));
}

void recordSharedCounters(
    benchmark::State& state,
    const EdgeEdgeCcdFixture& fixture,
    const double maxError)
{
  state.counters["pairs"] = static_cast<double>(fixture.pairs.size());
  state.counters["hits"] = static_cast<double>(fixture.hitCount);
  state.counters["min_step_bound"] = fixture.minStepBound;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.pairs.size()));
}

void recordSceneRuntimeCounters(
    benchmark::State& state,
    const ScenePointTriangleCcdFixture& fixture,
    const double maxError)
{
  recordSharedCounters(state, fixture, maxError);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["runtime_point_triangle_candidates"]
      = static_cast<double>(fixture.sourcePointTriangleCandidateCount);
  state.counters["static_triangle_point_triangle_candidates"]
      = static_cast<double>(fixture.pairs.size());
}

void recordSceneRuntimeCounters(
    benchmark::State& state,
    const SceneEdgeEdgeCcdFixture& fixture,
    const double maxError)
{
  recordSharedCounters(state, fixture, maxError);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["runtime_edge_edge_candidates"]
      = static_cast<double>(fixture.sourceEdgeEdgeCandidateCount);
}

template <typename Pair>
void recordRigidCurvedCounters(
    benchmark::State& state,
    const RigidCurvedCcdFixture<Pair>& fixture,
    const double maxError)
{
  state.counters["pairs"] = static_cast<double>(fixture.trajectoryCount);
  state.counters["segments"] = static_cast<double>(fixture.segmentPairs.size());
  state.counters["samples_per_pair"]
      = static_cast<double>(fixture.segmentCount);
  state.counters["hits"] = static_cast<double>(fixture.hitCount);
  state.counters["min_step_bound"] = fixture.minStepBound;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations() * fixture.segmentPairs.size()));
}

} // namespace

//==============================================================================
static void BM_Plan083CcdLineSearchCpu(benchmark::State& state)
{
  const auto fixture = makeCcdFixture(static_cast<int>(state.range(0)));

  std::size_t hitCount = 0;
  double minStepBound = 1.0;
  for (auto _ : state) {
    hitCount = 0;
    minStepBound = 1.0;
    for (const auto& pair : fixture.pairs) {
      const auto result = cpuResult(pair);
      if (result.hit) {
        ++hitCount;
        minStepBound = std::min(minStepBound, result.stepBound);
      }
      double stepBound = result.stepBound;
      benchmark::DoNotOptimize(stepBound);
    }
  }

  benchmark::DoNotOptimize(hitCount);
  benchmark::DoNotOptimize(minStepBound);
  recordSharedCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083CcdLineSearchCpu)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_Plan083CcdLineSearchCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeCcdFixture(static_cast<int>(state.range(0)));
  cuda::PointTriangleCcdLineSearchResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluatePointTriangleCcdLineSearchCuda(
        fixture.pairs, cuda::CcdLineSearchOptions{}, result);
    benchmark::DoNotOptimize(result.stepBounds.data());
    benchmark::DoNotOptimize(result.hits.data());
  }

  for (std::size_t i = 0; i < fixture.cpuStepBounds.size(); ++i) {
    maxError = std::max(
        maxError, std::abs(result.stepBounds[i] - fixture.cpuStepBounds[i]));
  }

  recordSharedCounters(state, fixture, maxError);
  state.counters["gpu_hits"] = static_cast<double>(result.hitCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083CcdLineSearchCuda)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_Plan083EdgeEdgeCcdLineSearchCpu(benchmark::State& state)
{
  const auto fixture = makeEdgeEdgeCcdFixture(static_cast<int>(state.range(0)));

  std::size_t hitCount = 0;
  double minStepBound = 1.0;
  for (auto _ : state) {
    hitCount = 0;
    minStepBound = 1.0;
    for (const auto& pair : fixture.pairs) {
      const auto result = cpuResult(pair);
      if (result.hit) {
        ++hitCount;
        minStepBound = std::min(minStepBound, result.stepBound);
      }
      double stepBound = result.stepBound;
      benchmark::DoNotOptimize(stepBound);
    }
  }

  benchmark::DoNotOptimize(hitCount);
  benchmark::DoNotOptimize(minStepBound);
  recordSharedCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083EdgeEdgeCcdLineSearchCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083EdgeEdgeCcdLineSearchCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeEdgeEdgeCcdFixture(static_cast<int>(state.range(0)));
  cuda::EdgeEdgeCcdLineSearchResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluateEdgeEdgeCcdLineSearchCuda(
        fixture.pairs, cuda::CcdLineSearchOptions{}, result);
    benchmark::DoNotOptimize(result.stepBounds.data());
    benchmark::DoNotOptimize(result.hits.data());
  }

  for (std::size_t i = 0; i < fixture.cpuStepBounds.size(); ++i) {
    maxError = std::max(
        maxError, std::abs(result.stepBounds[i] - fixture.cpuStepBounds[i]));
  }

  recordSharedCounters(state, fixture, maxError);
  state.counters["gpu_hits"] = static_cast<double>(result.hitCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083EdgeEdgeCcdLineSearchCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimePointTriangleCcdLineSearchCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimePointTriangleCcdFixture(
      static_cast<int>(state.range(0)));

  std::size_t hitCount = 0;
  double minStepBound = 1.0;
  for (auto _ : state) {
    hitCount = 0;
    minStepBound = 1.0;
    for (const auto& pair : fixture.pairs) {
      const auto result = cpuResult(pair);
      if (result.hit) {
        ++hitCount;
        minStepBound = std::min(minStepBound, result.stepBound);
      }
      double stepBound = result.stepBound;
      benchmark::DoNotOptimize(stepBound);
    }
  }

  benchmark::DoNotOptimize(hitCount);
  benchmark::DoNotOptimize(minStepBound);
  recordSceneRuntimeCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimePointTriangleCcdLineSearchCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimePointTriangleCcdLineSearchCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimePointTriangleCcdFixture(
      static_cast<int>(state.range(0)));
  cuda::PointTriangleCcdLineSearchResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluatePointTriangleCcdLineSearchCuda(
        fixture.pairs, cuda::CcdLineSearchOptions{}, result);
    benchmark::DoNotOptimize(result.stepBounds.data());
    benchmark::DoNotOptimize(result.hits.data());
  }

  maxError = maxAbsDifference(result.stepBounds, fixture.cpuStepBounds);
  recordSceneRuntimeCounters(state, fixture, maxError);
  state.counters["gpu_hits"] = static_cast<double>(result.hitCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimePointTriangleCcdLineSearchCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeEdgeEdgeCcdLineSearchCpu(
    benchmark::State& state)
{
  const auto fixture
      = makeSceneRuntimeEdgeEdgeCcdFixture(static_cast<int>(state.range(0)));

  std::size_t hitCount = 0;
  double minStepBound = 1.0;
  for (auto _ : state) {
    hitCount = 0;
    minStepBound = 1.0;
    for (const auto& pair : fixture.pairs) {
      const auto result = cpuResult(pair);
      if (result.hit) {
        ++hitCount;
        minStepBound = std::min(minStepBound, result.stepBound);
      }
      double stepBound = result.stepBound;
      benchmark::DoNotOptimize(stepBound);
    }
  }

  benchmark::DoNotOptimize(hitCount);
  benchmark::DoNotOptimize(minStepBound);
  recordSceneRuntimeCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083SceneRuntimeEdgeEdgeCcdLineSearchCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083SceneRuntimeEdgeEdgeCcdLineSearchCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSceneRuntimeEdgeEdgeCcdFixture(static_cast<int>(state.range(0)));
  cuda::EdgeEdgeCcdLineSearchResult result;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluateEdgeEdgeCcdLineSearchCuda(
        fixture.pairs, cuda::CcdLineSearchOptions{}, result);
    benchmark::DoNotOptimize(result.stepBounds.data());
    benchmark::DoNotOptimize(result.hits.data());
  }

  maxError = maxAbsDifference(result.stepBounds, fixture.cpuStepBounds);
  recordSceneRuntimeCounters(state, fixture, maxError);
  state.counters["gpu_hits"] = static_cast<double>(result.hitCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083SceneRuntimeEdgeEdgeCcdLineSearchCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083RigidCurvedPointTriangleCcdLineSearchCpu(
    benchmark::State& state)
{
  const auto fixture
      = makeRigidCurvedPointTriangleFixture(static_cast<int>(state.range(0)));

  std::vector<double> stepBounds;
  std::vector<std::uint8_t> hits;
  std::size_t hitCount = 0;
  double minStepBound = 1.0;
  for (auto _ : state) {
    evaluateTrajectorySegments(
        fixture.segmentPairs,
        fixture.trajectoryCount,
        fixture.segmentCount,
        stepBounds,
        hits,
        hitCount,
        minStepBound);
    benchmark::DoNotOptimize(stepBounds.data());
    benchmark::DoNotOptimize(hits.data());
  }

  benchmark::DoNotOptimize(hitCount);
  benchmark::DoNotOptimize(minStepBound);
  recordRigidCurvedCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083RigidCurvedPointTriangleCcdLineSearchCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083RigidCurvedPointTriangleCcdLineSearchCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeRigidCurvedPointTriangleFixture(static_cast<int>(state.range(0)));
  cuda::PointTriangleCcdLineSearchResult segmentResult;
  std::vector<double> gpuStepBounds;
  std::vector<std::uint8_t> gpuHits;
  std::size_t gpuHitCount = 0;
  double gpuMinStepBound = 1.0;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluatePointTriangleCcdLineSearchCuda(
        fixture.segmentPairs, cuda::CcdLineSearchOptions{}, segmentResult);
    reduceGpuTrajectorySegments(
        segmentResult,
        fixture.trajectoryCount,
        fixture.segmentCount,
        gpuStepBounds,
        gpuHits,
        gpuHitCount,
        gpuMinStepBound);
    benchmark::DoNotOptimize(gpuStepBounds.data());
    benchmark::DoNotOptimize(gpuHits.data());
  }

  maxError = maxAbsDifference(gpuStepBounds, fixture.cpuStepBounds);
  recordRigidCurvedCounters(state, fixture, maxError);
  state.counters["gpu_hits"] = static_cast<double>(gpuHitCount);
  state.counters["gpu_segments"]
      = static_cast<double>(segmentResult.hits.size());
  state.counters["gpu_min_step_bound"] = gpuMinStepBound;
  state.counters["host_setup_ns"] = segmentResult.timing.setupNs;
  state.counters["host_to_device_ns"] = segmentResult.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = segmentResult.timing.kernelNs;
  state.counters["device_to_host_ns"] = segmentResult.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083RigidCurvedPointTriangleCcdLineSearchCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083RigidCurvedEdgeEdgeCcdLineSearchCpu(
    benchmark::State& state)
{
  const auto fixture
      = makeRigidCurvedEdgeEdgeFixture(static_cast<int>(state.range(0)));

  std::vector<double> stepBounds;
  std::vector<std::uint8_t> hits;
  std::size_t hitCount = 0;
  double minStepBound = 1.0;
  for (auto _ : state) {
    evaluateTrajectorySegments(
        fixture.segmentPairs,
        fixture.trajectoryCount,
        fixture.segmentCount,
        stepBounds,
        hits,
        hitCount,
        minStepBound);
    benchmark::DoNotOptimize(stepBounds.data());
    benchmark::DoNotOptimize(hits.data());
  }

  benchmark::DoNotOptimize(hitCount);
  benchmark::DoNotOptimize(minStepBound);
  recordRigidCurvedCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083RigidCurvedEdgeEdgeCcdLineSearchCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083RigidCurvedEdgeEdgeCcdLineSearchCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeRigidCurvedEdgeEdgeFixture(static_cast<int>(state.range(0)));
  cuda::EdgeEdgeCcdLineSearchResult segmentResult;
  std::vector<double> gpuStepBounds;
  std::vector<std::uint8_t> gpuHits;
  std::size_t gpuHitCount = 0;
  double gpuMinStepBound = 1.0;
  double maxError = 0.0;

  for (auto _ : state) {
    cuda::evaluateEdgeEdgeCcdLineSearchCuda(
        fixture.segmentPairs, cuda::CcdLineSearchOptions{}, segmentResult);
    reduceGpuTrajectorySegments(
        segmentResult,
        fixture.trajectoryCount,
        fixture.segmentCount,
        gpuStepBounds,
        gpuHits,
        gpuHitCount,
        gpuMinStepBound);
    benchmark::DoNotOptimize(gpuStepBounds.data());
    benchmark::DoNotOptimize(gpuHits.data());
  }

  maxError = maxAbsDifference(gpuStepBounds, fixture.cpuStepBounds);
  recordRigidCurvedCounters(state, fixture, maxError);
  state.counters["gpu_hits"] = static_cast<double>(gpuHitCount);
  state.counters["gpu_segments"]
      = static_cast<double>(segmentResult.hits.size());
  state.counters["gpu_min_step_bound"] = gpuMinStepBound;
  state.counters["host_setup_ns"] = segmentResult.timing.setupNs;
  state.counters["host_to_device_ns"] = segmentResult.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = segmentResult.timing.kernelNs;
  state.counters["device_to_host_ns"] = segmentResult.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083RigidCurvedEdgeEdgeCcdLineSearchCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();
