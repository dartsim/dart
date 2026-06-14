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

#include <dart/simulation/detail/deformable_contact/continuous_collision_step.hpp>

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

namespace {

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

  for (const auto& pair : fixture.pairs) {
    const auto result = cpuResult(pair);
    fixture.cpuStepBounds.push_back(result.stepBound);
    fixture.cpuHits.push_back(result.hit ? 1u : 0u);
    if (result.hit) {
      ++fixture.hitCount;
      fixture.minStepBound = std::min(fixture.minStepBound, result.stepBound);
    }
  }

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

  for (const auto& pair : fixture.pairs) {
    const auto result = cpuResult(pair);
    fixture.cpuStepBounds.push_back(result.stepBound);
    fixture.cpuHits.push_back(result.hit ? 1u : 0u);
    if (result.hit) {
      ++fixture.hitCount;
      fixture.minStepBound = std::min(fixture.minStepBound, result.stepBound);
    }
  }

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
