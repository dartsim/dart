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

#include <dart/simulation/experimental/detail/deformable_contact/candidate_set.hpp>

#include <benchmark/benchmark.h>

#include <vector>

#include <cstdint>

namespace sx = dart::simulation::experimental;
namespace dc = dart::simulation::experimental::detail::deformable_contact;

namespace {

struct MotionFixture
{
  std::vector<Eigen::Vector3d> start;
  std::vector<Eigen::Vector3d> end;
  std::vector<sx::DeformableSurfaceTriangle> triangles;
};

//==============================================================================
MotionFixture makeFallingPointCorpus(const int pairCount)
{
  MotionFixture fixture;
  fixture.start.reserve(static_cast<std::size_t>(4 * pairCount));
  fixture.triangles.reserve(static_cast<std::size_t>(pairCount));

  for (int i = 0; i < pairCount; ++i) {
    const double offset = 3.0 * static_cast<double>(i);
    const std::size_t base = fixture.start.size();
    fixture.start.emplace_back(offset - 0.6, -0.6, 0.0);
    fixture.start.emplace_back(offset + 0.6, -0.6, 0.0);
    fixture.start.emplace_back(offset, 0.6, 0.0);
    fixture.start.emplace_back(offset, 0.0, 0.75);
    fixture.triangles.push_back(
        sx::DeformableSurfaceTriangle{base + 0, base + 1, base + 2});
  }

  fixture.end = fixture.start;
  for (int i = 0; i < pairCount; ++i) {
    fixture.end[4 * static_cast<std::size_t>(i) + 3].z() = -0.75;
  }

  return fixture;
}

//==============================================================================
MotionFixture makeCoherentTranslationCorpus(const int pairCount)
{
  MotionFixture fixture = makeFallingPointCorpus(pairCount);
  fixture.end = fixture.start;

  const Eigen::Vector3d translation(2.25, 0.0, 0.0);
  for (auto& position : fixture.end) {
    position += translation;
  }

  return fixture;
}

//==============================================================================
void recordCounters(
    benchmark::State& state,
    const MotionFixture& fixture,
    const dc::ContactCandidateSet& candidates)
{
  state.counters["vertices"] = static_cast<double>(fixture.start.size());
  state.counters["triangles"] = static_cast<double>(fixture.triangles.size());
  state.counters["edges"] = static_cast<double>(candidates.stats.edgeCount);
  state.counters["swept_aabb_overlaps"]
      = static_cast<double>(candidates.stats.broadPhaseOverlapCount);
  state.counters["endpoint_distance_checks"]
      = static_cast<double>(candidates.stats.exactDistanceCheckCount);
  state.counters["pt_candidates"]
      = static_cast<double>(candidates.stats.pointTriangleCandidateCount);
  state.counters["ee_candidates"]
      = static_cast<double>(candidates.stats.edgeEdgeCandidateCount);
  state.counters["incident_rejects"]
      = static_cast<double>(candidates.stats.incidentPointTriangleRejectCount);
  state.counters["adjacent_rejects"]
      = static_cast<double>(candidates.stats.adjacentEdgeEdgeRejectCount);

  const auto primitiveCount = fixture.start.size() + fixture.triangles.size()
                              + candidates.stats.edgeCount;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * primitiveCount));
}

//==============================================================================
template <typename Builder>
void runMotionAwareCandidateBenchmark(
    benchmark::State& state, const MotionFixture& fixture, Builder&& builder)
{
  dc::ContactCandidateOptions options;
  options.activationDistance = 0.05;
  options.exactDistanceFilter = true;

  dc::ContactCandidateSet candidates;
  for (auto _ : state) {
    candidates
        = builder(fixture.start, fixture.end, fixture.triangles, options);
    benchmark::DoNotOptimize(candidates.pointTriangleCandidates.data());
    benchmark::DoNotOptimize(candidates.edgeEdgeCandidates.data());
  }

  recordCounters(state, fixture, candidates);
}

} // namespace

//==============================================================================
static void BM_IpcMotionAwareCandidateSetSweepFallingPoints(
    benchmark::State& state)
{
  const auto fixture = makeFallingPointCorpus(static_cast<int>(state.range(0)));
  runMotionAwareCandidateBenchmark(
      state, fixture, dc::buildMotionAwareContactCandidatesSweep);
}
BENCHMARK(BM_IpcMotionAwareCandidateSetSweepFallingPoints)
    ->Arg(16)
    ->Arg(64)
    ->Arg(128);

//==============================================================================
static void BM_IpcMotionAwareCandidateSetBruteForceFallingPoints(
    benchmark::State& state)
{
  const auto fixture = makeFallingPointCorpus(static_cast<int>(state.range(0)));
  runMotionAwareCandidateBenchmark(
      state, fixture, dc::buildMotionAwareContactCandidatesBruteForce);
}
BENCHMARK(BM_IpcMotionAwareCandidateSetBruteForceFallingPoints)
    ->Arg(16)
    ->Arg(64);

//==============================================================================
static void BM_IpcMotionAwareCandidateSetSweepCoherentTranslation(
    benchmark::State& state)
{
  const auto fixture
      = makeCoherentTranslationCorpus(static_cast<int>(state.range(0)));
  runMotionAwareCandidateBenchmark(
      state, fixture, dc::buildMotionAwareContactCandidatesSweep);
}
BENCHMARK(BM_IpcMotionAwareCandidateSetSweepCoherentTranslation)
    ->Arg(16)
    ->Arg(64);
