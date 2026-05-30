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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/experimental/detail/deformable_contact/candidate_set.hpp>

#include <benchmark/benchmark.h>

#include <vector>

namespace sx = dart::simulation::experimental;
namespace dc = dart::simulation::experimental::detail::deformable_contact;

namespace {

struct MeshFixture
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<sx::DeformableSurfaceTriangle> triangles;
};

struct SweepPairFixture
{
  std::vector<dc::detail::SweepItem> lhsItems;
  std::vector<dc::detail::SweepItem> rhsItems;
};

//==============================================================================
dc::detail::SweepItem makeSweepItem(
    const std::size_t id, const double minX, const double maxX)
{
  return dc::detail::SweepItem{
      id,
      dc::detail::CandidateAabb{
          Eigen::Vector3d(minX, 0.0, 0.0), Eigen::Vector3d(maxX, 1.0, 1.0)}};
}

//==============================================================================
SweepPairFixture makeExpiredPrefixSweepPairs(const int count)
{
  SweepPairFixture fixture;
  fixture.lhsItems.reserve(static_cast<std::size_t>(count));
  fixture.rhsItems.reserve(static_cast<std::size_t>(count));

  for (int i = 0; i < count; ++i) {
    const double x = static_cast<double>(i);
    fixture.lhsItems.push_back(
        makeSweepItem(static_cast<std::size_t>(i), x, x + 0.25));
    fixture.rhsItems.push_back(
        makeSweepItem(static_cast<std::size_t>(i), x + 0.125, x + 0.375));
  }

  return fixture;
}

//==============================================================================
// The worst case for the old prefix-skip cross-sweep: a single long-lived
// right-hand-side interval that spans the whole x-range (y-disjoint, so it
// never overlaps and never expires) sorts first in min-x order and pins the
// prefix cursor, while every other right-hand-side interval expires as the
// sweep advances. The prefix scan then rescans all already-expired intervals
// for every left-hand-side item (quadratic); the active-set sweep unlinks them
// as they expire and stays near-linear.
SweepPairFixture makeLongLivedIntervalSweepPairs(const int count)
{
  SweepPairFixture fixture;
  fixture.lhsItems.reserve(static_cast<std::size_t>(count));
  fixture.rhsItems.reserve(static_cast<std::size_t>(count) + 1u);

  // The blocker spans the full x extent but lives in y in [10, 11], so it never
  // overlaps a left-hand-side item (all in y in [0, 1]) and never expires.
  fixture.rhsItems.push_back(
      dc::detail::SweepItem{
          static_cast<std::size_t>(count),
          dc::detail::CandidateAabb{
              Eigen::Vector3d(-1.0, 10.0, 0.0),
              Eigen::Vector3d(static_cast<double>(count) + 1.0, 11.0, 1.0)}});

  for (int i = 0; i < count; ++i) {
    const double x = static_cast<double>(i);
    fixture.lhsItems.push_back(
        makeSweepItem(static_cast<std::size_t>(i), x, x + 0.25));
    fixture.rhsItems.push_back(
        makeSweepItem(static_cast<std::size_t>(i), x + 0.125, x + 0.375));
  }

  return fixture;
}

//==============================================================================
MeshFixture makeClothGrid(const int resolution)
{
  MeshFixture fixture;
  fixture.positions.reserve(
      static_cast<std::size_t>(resolution + 1) * (resolution + 1));

  for (int y = 0; y <= resolution; ++y) {
    for (int x = 0; x <= resolution; ++x) {
      const double z = ((x + y) % 3 == 0) ? 0.015 : 0.0;
      fixture.positions.emplace_back(
          static_cast<double>(x) / resolution,
          static_cast<double>(y) / resolution,
          z);
    }
  }

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

  return fixture;
}

//==============================================================================
MeshFixture makeTetraSurfaceGrid(const int resolution)
{
  MeshFixture fixture;
  fixture.positions.reserve(
      static_cast<std::size_t>(4 * resolution * resolution));
  fixture.triangles.reserve(
      static_cast<std::size_t>(4 * resolution * resolution));

  for (int y = 0; y < resolution; ++y) {
    for (int x = 0; x < resolution; ++x) {
      const double ox = static_cast<double>(x);
      const double oy = static_cast<double>(y);
      const std::size_t base = fixture.positions.size();
      fixture.positions.emplace_back(ox, oy, 0.0);
      fixture.positions.emplace_back(ox + 0.35, oy, 0.0);
      fixture.positions.emplace_back(ox, oy + 0.35, 0.0);
      fixture.positions.emplace_back(ox + 0.08, oy + 0.08, 0.35);

      fixture.triangles.push_back(
          sx::DeformableSurfaceTriangle{base + 0, base + 2, base + 1});
      fixture.triangles.push_back(
          sx::DeformableSurfaceTriangle{base + 0, base + 1, base + 3});
      fixture.triangles.push_back(
          sx::DeformableSurfaceTriangle{base + 1, base + 2, base + 3});
      fixture.triangles.push_back(
          sx::DeformableSurfaceTriangle{base + 2, base + 0, base + 3});
    }
  }

  return fixture;
}

//==============================================================================
void recordCounters(
    benchmark::State& state,
    const MeshFixture& fixture,
    const dc::ContactCandidateSet& candidates)
{
  state.counters["vertices"] = static_cast<double>(fixture.positions.size());
  state.counters["triangles"] = static_cast<double>(fixture.triangles.size());
  state.counters["edges"] = static_cast<double>(candidates.stats.edgeCount);
  state.counters["aabb_overlaps"]
      = static_cast<double>(candidates.stats.broadPhaseOverlapCount);
  state.counters["exact_checks"]
      = static_cast<double>(candidates.stats.exactDistanceCheckCount);
  state.counters["pt_candidates"]
      = static_cast<double>(candidates.stats.pointTriangleCandidateCount);
  state.counters["ee_candidates"]
      = static_cast<double>(candidates.stats.edgeEdgeCandidateCount);
  state.counters["incident_rejects"]
      = static_cast<double>(candidates.stats.incidentPointTriangleRejectCount);
  state.counters["adjacent_rejects"]
      = static_cast<double>(candidates.stats.adjacentEdgeEdgeRejectCount);

  const auto primitiveCount = fixture.positions.size()
                              + fixture.triangles.size()
                              + candidates.stats.edgeCount;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * primitiveCount));
}

//==============================================================================
template <typename Builder>
void runCandidateSetBenchmark(
    benchmark::State& state, const MeshFixture& fixture, Builder&& builder)
{
  dc::ContactCandidateOptions options;
  options.activationDistance = 0.05;

  dc::ContactCandidateSet candidates;
  for (auto _ : state) {
    candidates = builder(fixture.positions, fixture.triangles, options);
    benchmark::DoNotOptimize(candidates.pointTriangleCandidates.data());
    benchmark::DoNotOptimize(candidates.edgeEdgeCandidates.data());
  }

  recordCounters(state, fixture, candidates);
}

//==============================================================================
template <typename Builder>
void runReusableCandidateSetBenchmark(
    benchmark::State& state, const MeshFixture& fixture, Builder&& builder)
{
  dc::ContactCandidateOptions options;
  options.activationDistance = 0.05;

  dc::ContactCandidateSet candidates;
  builder(fixture.positions, fixture.triangles, options, candidates);
  for (auto _ : state) {
    builder(fixture.positions, fixture.triangles, options, candidates);
    benchmark::DoNotOptimize(candidates.pointTriangleCandidates.data());
    benchmark::DoNotOptimize(candidates.edgeEdgeCandidates.data());
  }

  recordCounters(state, fixture, candidates);
}

//==============================================================================
template <typename Builder>
void runSweepScratchCandidateSetBenchmark(
    benchmark::State& state, const MeshFixture& fixture, Builder&& builder)
{
  dc::ContactCandidateOptions options;
  options.activationDistance = 0.05;

  dc::ContactCandidateSet candidates;
  dc::detail::ContactCandidateSweepScratch scratch;
  builder(fixture.positions, fixture.triangles, options, candidates, scratch);
  for (auto _ : state) {
    builder(fixture.positions, fixture.triangles, options, candidates, scratch);
    benchmark::DoNotOptimize(candidates.pointTriangleCandidates.data());
    benchmark::DoNotOptimize(candidates.edgeEdgeCandidates.data());
  }

  recordCounters(state, fixture, candidates);
  state.counters["scratch_point_capacity"]
      = static_cast<double>(scratch.pointItems.capacity());
  state.counters["scratch_triangle_capacity"]
      = static_cast<double>(scratch.triangleItems.capacity());
  state.counters["scratch_edge_capacity"]
      = static_cast<double>(scratch.edgeItems.capacity());
}

} // namespace

//==============================================================================
static void BM_IpcCandidateSetCrossSweepExpiredPrefix(benchmark::State& state)
{
  auto fixture = makeExpiredPrefixSweepPairs(static_cast<int>(state.range(0)));

  std::size_t pairCount = 0;
  std::vector<std::size_t> links; // reusable live-list scratch (hot-path form)
  for (auto _ : state) {
    pairCount = 0;
    dc::detail::visitSweepPairs(
        fixture.lhsItems,
        fixture.rhsItems,
        [&](std::size_t lhsId, std::size_t rhsId) {
          ++pairCount;
          benchmark::DoNotOptimize(lhsId);
          benchmark::DoNotOptimize(rhsId);
        },
        links);
    benchmark::DoNotOptimize(pairCount);
  }

  state.counters["lhs_items"] = static_cast<double>(fixture.lhsItems.size());
  state.counters["rhs_items"] = static_cast<double>(fixture.rhsItems.size());
  state.counters["visited_pairs"] = static_cast<double>(pairCount);
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (fixture.lhsItems.size() + fixture.rhsItems.size())));
}
BENCHMARK(BM_IpcCandidateSetCrossSweepExpiredPrefix)
    ->Arg(64)
    ->Arg(256)
    ->Arg(1024);

//==============================================================================
// Long-lived blocker case (see makeLongLivedIntervalSweepPairs): the active-set
// sweep should stay near-linear where the old prefix-skip degraded toward
// quadratic because it could not prune expired intervals behind the blocker.
static void BM_IpcCandidateSetCrossSweepLongLivedInterval(
    benchmark::State& state)
{
  auto fixture
      = makeLongLivedIntervalSweepPairs(static_cast<int>(state.range(0)));

  std::size_t pairCount = 0;
  std::vector<std::size_t> links; // reusable live-list scratch (hot-path form)
  for (auto _ : state) {
    pairCount = 0;
    dc::detail::visitSweepPairs(
        fixture.lhsItems,
        fixture.rhsItems,
        [&](std::size_t lhsId, std::size_t rhsId) {
          ++pairCount;
          benchmark::DoNotOptimize(lhsId);
          benchmark::DoNotOptimize(rhsId);
        },
        links);
    benchmark::DoNotOptimize(pairCount);
  }

  state.counters["lhs_items"] = static_cast<double>(fixture.lhsItems.size());
  state.counters["rhs_items"] = static_cast<double>(fixture.rhsItems.size());
  state.counters["visited_pairs"] = static_cast<double>(pairCount);
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (fixture.lhsItems.size() + fixture.rhsItems.size())));
}
BENCHMARK(BM_IpcCandidateSetCrossSweepLongLivedInterval)
    ->Arg(64)
    ->Arg(256)
    ->Arg(1024);

//==============================================================================
static void BM_IpcCandidateSetSweepCloth(benchmark::State& state)
{
  const auto fixture = makeClothGrid(static_cast<int>(state.range(0)));
  runCandidateSetBenchmark(
      state,
      fixture,
      [](const auto& positions, const auto& triangles, const auto& options) {
        return dc::buildContactCandidatesSweep(positions, triangles, options);
      });
}
BENCHMARK(BM_IpcCandidateSetSweepCloth)->Arg(8)->Arg(16)->Arg(24);

//==============================================================================
static void BM_IpcCandidateSetReusableSweepCloth(benchmark::State& state)
{
  const auto fixture = makeClothGrid(static_cast<int>(state.range(0)));
  runReusableCandidateSetBenchmark(
      state,
      fixture,
      [](const auto& positions,
         const auto& triangles,
         const auto& options,
         auto& candidates) {
        dc::buildContactCandidatesSweep(
            positions, triangles, options, candidates);
      });
}
BENCHMARK(BM_IpcCandidateSetReusableSweepCloth)->Arg(8)->Arg(16)->Arg(24);

//==============================================================================
static void BM_IpcCandidateSetScratchSweepCloth(benchmark::State& state)
{
  const auto fixture = makeClothGrid(static_cast<int>(state.range(0)));
  runSweepScratchCandidateSetBenchmark(
      state,
      fixture,
      [](const auto& positions,
         const auto& triangles,
         const auto& options,
         auto& candidates,
         auto& scratch) {
        dc::buildContactCandidatesSweep(
            positions, triangles, options, candidates, scratch);
      });
}
BENCHMARK(BM_IpcCandidateSetScratchSweepCloth)->Arg(8)->Arg(16)->Arg(24);

//==============================================================================
static void BM_IpcCandidateSetBruteForceCloth(benchmark::State& state)
{
  const auto fixture = makeClothGrid(static_cast<int>(state.range(0)));
  runCandidateSetBenchmark(
      state,
      fixture,
      [](const auto& positions, const auto& triangles, const auto& options) {
        return dc::buildContactCandidatesBruteForce(
            positions, triangles, options);
      });
}
BENCHMARK(BM_IpcCandidateSetBruteForceCloth)->Arg(8)->Arg(16);

//==============================================================================
static void BM_IpcCandidateSetSweepTetraSurface(benchmark::State& state)
{
  const auto fixture = makeTetraSurfaceGrid(static_cast<int>(state.range(0)));
  runCandidateSetBenchmark(
      state,
      fixture,
      [](const auto& positions, const auto& triangles, const auto& options) {
        return dc::buildContactCandidatesSweep(positions, triangles, options);
      });
}
BENCHMARK(BM_IpcCandidateSetSweepTetraSurface)->Arg(4)->Arg(8)->Arg(12);

//==============================================================================
static void BM_IpcCandidateSetReusableSweepTetraSurface(benchmark::State& state)
{
  const auto fixture = makeTetraSurfaceGrid(static_cast<int>(state.range(0)));
  runReusableCandidateSetBenchmark(
      state,
      fixture,
      [](const auto& positions,
         const auto& triangles,
         const auto& options,
         auto& candidates) {
        dc::buildContactCandidatesSweep(
            positions, triangles, options, candidates);
      });
}
BENCHMARK(BM_IpcCandidateSetReusableSweepTetraSurface)->Arg(4)->Arg(8)->Arg(12);

//==============================================================================
static void BM_IpcCandidateSetScratchSweepTetraSurface(benchmark::State& state)
{
  const auto fixture = makeTetraSurfaceGrid(static_cast<int>(state.range(0)));
  runSweepScratchCandidateSetBenchmark(
      state,
      fixture,
      [](const auto& positions,
         const auto& triangles,
         const auto& options,
         auto& candidates,
         auto& scratch) {
        dc::buildContactCandidatesSweep(
            positions, triangles, options, candidates, scratch);
      });
}
BENCHMARK(BM_IpcCandidateSetScratchSweepTetraSurface)->Arg(4)->Arg(8)->Arg(12);

//==============================================================================
static void BM_IpcCandidateSetBruteForceTetraSurface(benchmark::State& state)
{
  const auto fixture = makeTetraSurfaceGrid(static_cast<int>(state.range(0)));
  runCandidateSetBenchmark(
      state,
      fixture,
      [](const auto& positions, const auto& triangles, const auto& options) {
        return dc::buildContactCandidatesBruteForce(
            positions, triangles, options);
      });
}
BENCHMARK(BM_IpcCandidateSetBruteForceTetraSurface)->Arg(4)->Arg(8);
