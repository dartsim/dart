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

#include <dart/simulation/detail/deformable_contact/candidate_set.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/contact_candidate_filter_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <algorithm>
#include <vector>

#include <cmath>
#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;
namespace dc = dart::simulation::detail::deformable_contact;

namespace {

constexpr double kActivationDistance = 0.05;

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
    CandidateMaskFixture& fixture,
    const double x,
    const double y,
    const double z)
{
  fixture.positions.push_back(x);
  fixture.positions.push_back(y);
  fixture.positions.push_back(z);
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
BENCHMARK(BM_Plan083ContactCandidateCpu)->Arg(4096)->Arg(65536)->UseRealTime();

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
BENCHMARK(BM_Plan083ContactCandidateCuda)->Arg(4096)->Arg(65536)->UseRealTime();

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
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083PointTriangleCandidateMaskCuda)
    ->Arg(4096)
    ->Arg(65536)
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
    ->UseRealTime();
