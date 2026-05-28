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

} // namespace

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
