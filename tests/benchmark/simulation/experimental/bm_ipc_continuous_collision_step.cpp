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

#include <dart/simulation/experimental/detail/deformable_contact/continuous_collision_step.hpp>

#include <benchmark/benchmark.h>

#include <vector>

namespace sx = dart::simulation::experimental;
namespace dc = dart::simulation::experimental::detail::deformable_contact;

namespace {

struct MeshFixture
{
  std::vector<Eigen::Vector3d> start;
  std::vector<Eigen::Vector3d> end;
  std::vector<sx::DeformableSurfaceTriangle> triangles;
  dc::ContactCandidateSet candidates;
};

//==============================================================================
MeshFixture makeFallingPointPatch(const int resolution)
{
  MeshFixture fixture;
  fixture.start.reserve(
      static_cast<std::size_t>(resolution + 1) * (resolution + 1) + 1);

  for (int y = 0; y <= resolution; ++y) {
    for (int x = 0; x <= resolution; ++x) {
      fixture.start.emplace_back(
          static_cast<double>(x) / resolution,
          static_cast<double>(y) / resolution,
          0.0);
    }
  }

  const std::size_t fallingPoint = fixture.start.size();
  fixture.start.emplace_back(0.5, 0.5, 0.2);
  fixture.end = fixture.start;
  fixture.end[fallingPoint].z() = -0.2;

  const auto node = [resolution](const int x, const int y) {
    return static_cast<std::size_t>(y * (resolution + 1) + x);
  };

  fixture.triangles.reserve(2 * resolution * resolution);
  for (int y = 0; y < resolution; ++y) {
    for (int x = 0; x < resolution; ++x) {
      fixture.triangles.push_back(
          sx::DeformableSurfaceTriangle{
              node(x, y), node(x + 1, y), node(x, y + 1)});
      fixture.triangles.push_back(
          sx::DeformableSurfaceTriangle{
              node(x + 1, y), node(x + 1, y + 1), node(x, y + 1)});
    }
  }

  dc::ContactCandidateOptions candidateOptions;
  candidateOptions.activationDistance = 0.25;
  candidateOptions.excludeIncidentPointTriangles = true;
  candidateOptions.excludeAdjacentEdges = true;
  fixture.candidates = dc::buildContactCandidatesSweep(
      fixture.start, fixture.triangles, candidateOptions);

  return fixture;
}

//==============================================================================
MeshFixture makeCrossingEdges()
{
  MeshFixture fixture;
  fixture.start = {
      {-1.0, 0.0, 0.5},
      {1.0, 0.0, 0.5},
      {0.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
  };
  fixture.end = fixture.start;
  fixture.end[0].z() = -0.5;
  fixture.end[1].z() = -0.5;
  fixture.candidates.surfaceEdges = {
      dc::SurfaceEdge{0, 1},
      dc::SurfaceEdge{2, 3},
  };
  fixture.candidates.edgeEdgeCandidates.push_back(
      dc::EdgeEdgeCandidate{0, 1, 0.0});
  return fixture;
}

//==============================================================================
void recordCounters(
    benchmark::State& state, const dc::ContinuousCollisionStepResult& result)
{
  state.counters["pt_checks"]
      = static_cast<double>(result.stats.pointTriangleChecks);
  state.counters["ee_checks"]
      = static_cast<double>(result.stats.edgeEdgeChecks);
  state.counters["hits"] = static_cast<double>(result.stats.hits);
  state.counters["misses"] = static_cast<double>(result.stats.misses);
  state.counters["zero_steps"]
      = static_cast<double>(result.stats.zeroStepCount);
  state.counters["step_bound"] = result.stepBound;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (result.stats.pointTriangleChecks + result.stats.edgeEdgeChecks)));
}

//==============================================================================
void consumeResult(const dc::ContinuousCollisionStepResult& result)
{
  bool hit = result.hit;
  double stepBound = result.stepBound;
  std::size_t limitingCandidate = result.limitingCandidate;
  benchmark::DoNotOptimize(hit);
  benchmark::DoNotOptimize(stepBound);
  benchmark::DoNotOptimize(limitingCandidate);
  benchmark::ClobberMemory();
}

} // namespace

//==============================================================================
static void BM_IpcPointTriangleStepBound(benchmark::State& state)
{
  const Eigen::Vector3d a(-1.0, -1.0, 0.0);
  const Eigen::Vector3d b(1.0, -1.0, 0.0);
  const Eigen::Vector3d c(0.0, 1.0, 0.0);
  const Eigen::Vector3d p0(0.0, 0.0, 1.0);
  const Eigen::Vector3d p1(0.0, 0.0, -1.0);

  dc::ContinuousCollisionStepResult result;
  for (auto _ : state) {
    result = dc::pointTriangleStepBound(p0, p1, a, a, b, b, c, c);
    consumeResult(result);
  }

  recordCounters(state, result);
}
BENCHMARK(BM_IpcPointTriangleStepBound);

//==============================================================================
static void BM_IpcEdgeEdgeStepBound(benchmark::State& state)
{
  const auto fixture = makeCrossingEdges();

  dc::ContinuousCollisionStepResult result;
  for (auto _ : state) {
    result = dc::edgeEdgeStepBound(
        fixture.start[0],
        fixture.end[0],
        fixture.start[1],
        fixture.end[1],
        fixture.start[2],
        fixture.end[2],
        fixture.start[3],
        fixture.end[3]);
    consumeResult(result);
  }

  recordCounters(state, result);
}
BENCHMARK(BM_IpcEdgeEdgeStepBound);

//==============================================================================
static void BM_IpcCandidateStepBoundFallingPatch(benchmark::State& state)
{
  const auto fixture = makeFallingPointPatch(static_cast<int>(state.range(0)));

  dc::ContinuousCollisionStepResult result;
  for (auto _ : state) {
    result = dc::contactCandidateStepBound(
        fixture.start, fixture.end, fixture.triangles, fixture.candidates);
    consumeResult(result);
  }

  recordCounters(state, result);
}
BENCHMARK(BM_IpcCandidateStepBoundFallingPatch)->Arg(8)->Arg(16)->Arg(24);
