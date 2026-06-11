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

#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>

#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <algorithm>
#include <vector>

#include <cmath>
#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;
namespace nb = dart::simulation::detail::newton_barrier;

namespace {

struct CpuLocalResult
{
  std::vector<double> barrierValues;
  std::vector<double> barrierFirstDerivatives;
  std::vector<double> barrierSecondDerivatives;
  std::vector<double> frictionValues;
  std::vector<double> frictionWorks;
  std::vector<double> frictionFirstDerivatives;
  std::vector<double> frictionSecondDerivatives;
  std::vector<std::uint8_t> activeBarriers;
  std::vector<std::uint8_t> activeFrictions;
  std::vector<std::uint8_t> dynamicFrictions;
  std::size_t activeBarrierCount = 0;
  std::size_t activeFrictionCount = 0;
  std::size_t dynamicFrictionCount = 0;
  double maxBarrierValue = 0.0;
  double maxFrictionWork = 0.0;
};

struct LocalFixture
{
  std::vector<cuda::BarrierFrictionLocalInput> inputs;
  CpuLocalResult cpu;
};

cuda::BarrierFrictionLocalInput makeInput(const int i)
{
  const bool activeBarrier = (i % 5) != 0;
  const bool dynamicFriction = (i % 3) == 0;
  const double normalized = static_cast<double>((i % 17) + 1) / 32.0;
  return {
      .squaredDistance = activeBarrier ? normalized : 1.25 + normalized,
      .squaredActivationDistance = 1.0,
      .stiffness = 1.0 + static_cast<double>(i % 7) * 0.125,
      .tangentialDisplacementNorm
      = dynamicFriction ? 0.3 + 0.01 * static_cast<double>(i % 5)
                        : 0.03 + 0.005 * static_cast<double>(i % 9),
      .frictionWeight = 0.5 + static_cast<double>(i % 11) * 0.25,
      .staticFrictionDisplacement = 0.2,
  };
}

void resizeCpuResult(CpuLocalResult& result, const std::size_t count)
{
  result.barrierValues.assign(count, 0.0);
  result.barrierFirstDerivatives.assign(count, 0.0);
  result.barrierSecondDerivatives.assign(count, 0.0);
  result.frictionValues.assign(count, 0.0);
  result.frictionWorks.assign(count, 0.0);
  result.frictionFirstDerivatives.assign(count, 0.0);
  result.frictionSecondDerivatives.assign(count, 0.0);
  result.activeBarriers.assign(count, 0u);
  result.activeFrictions.assign(count, 0u);
  result.dynamicFrictions.assign(count, 0u);
  result.activeBarrierCount = 0;
  result.activeFrictionCount = 0;
  result.dynamicFrictionCount = 0;
  result.maxBarrierValue = 0.0;
  result.maxFrictionWork = 0.0;
}

void evaluateCpu(
    const std::vector<cuda::BarrierFrictionLocalInput>& inputs,
    CpuLocalResult& result)
{
  resizeCpuResult(result, inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const double stiffness = std::max(0.0, input.stiffness);
    const auto barrier = nb::c2ClampedLogBarrier(
        input.squaredDistance, input.squaredActivationDistance);
    if (barrier.active && stiffness > 0.0) {
      result.activeBarriers[i] = 1u;
      ++result.activeBarrierCount;
      result.barrierValues[i] = stiffness * barrier.value;
      result.barrierFirstDerivatives[i] = stiffness * barrier.firstDerivative;
      result.barrierSecondDerivatives[i] = stiffness * barrier.secondDerivative;
      result.maxBarrierValue
          = std::max(result.maxBarrierValue, result.barrierValues[i]);
    }

    const auto smooth = nb::smoothFrictionNorm(
        input.tangentialDisplacementNorm, input.staticFrictionDisplacement);
    const auto work = nb::frictionWorkContribution(
        input.tangentialDisplacementNorm,
        input.frictionWeight,
        input.staticFrictionDisplacement);
    if (work.active) {
      result.activeFrictions[i] = 1u;
      ++result.activeFrictionCount;
      result.dynamicFrictions[i] = smooth.dynamicBranch ? 1u : 0u;
      result.dynamicFrictionCount += smooth.dynamicBranch ? 1u : 0u;
      result.frictionValues[i] = input.frictionWeight * smooth.value;
      result.frictionWorks[i] = work.work;
      result.frictionFirstDerivatives[i] = smooth.firstDerivative;
      result.frictionSecondDerivatives[i] = smooth.secondDerivative;
      result.maxFrictionWork
          = std::max(result.maxFrictionWork, result.frictionWorks[i]);
    }
  }
}

LocalFixture makeFixture(const int sampleCount)
{
  LocalFixture fixture;
  fixture.inputs.reserve(static_cast<std::size_t>(sampleCount));
  for (int i = 0; i < sampleCount; ++i) {
    fixture.inputs.push_back(makeInput(i));
  }
  evaluateCpu(fixture.inputs, fixture.cpu);
  return fixture;
}

template <typename Lhs, typename Rhs>
double maxAbsDifference(const Lhs& lhs, const Rhs& rhs)
{
  double maxError = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    maxError = std::max(maxError, std::abs(lhs[i] - rhs[i]));
  }
  return maxError;
}

double maxOutputError(
    const CpuLocalResult& expected,
    const cuda::BarrierFrictionLocalResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError, maxAbsDifference(expected.barrierValues, actual.barrierValues));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.barrierFirstDerivatives, actual.barrierFirstDerivatives));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.barrierSecondDerivatives, actual.barrierSecondDerivatives));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.frictionValues, actual.frictionValues));
  maxError = std::max(
      maxError, maxAbsDifference(expected.frictionWorks, actual.frictionWorks));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.frictionFirstDerivatives, actual.frictionFirstDerivatives));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.frictionSecondDerivatives,
          actual.frictionSecondDerivatives));
  return maxError;
}

void recordCounters(
    benchmark::State& state, const LocalFixture& fixture, const double maxError)
{
  state.counters["samples"] = static_cast<double>(fixture.inputs.size());
  state.counters["active_barriers"]
      = static_cast<double>(fixture.cpu.activeBarrierCount);
  state.counters["active_friction"]
      = static_cast<double>(fixture.cpu.activeFrictionCount);
  state.counters["dynamic_friction"]
      = static_cast<double>(fixture.cpu.dynamicFrictionCount);
  state.counters["max_barrier_value"] = fixture.cpu.maxBarrierValue;
  state.counters["max_friction_work"] = fixture.cpu.maxFrictionWork;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.inputs.size()));
}

} // namespace

//==============================================================================
static void BM_Plan083BarrierFrictionLocalCpu(benchmark::State& state)
{
  const auto fixture = makeFixture(static_cast<int>(state.range(0)));
  CpuLocalResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.inputs, result);
    benchmark::DoNotOptimize(result.barrierValues.data());
    benchmark::DoNotOptimize(result.frictionWorks.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083BarrierFrictionLocalCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083BarrierFrictionLocalCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeFixture(static_cast<int>(state.range(0)));
  cuda::BarrierFrictionLocalResult result;

  for (auto _ : state) {
    cuda::evaluateBarrierFrictionLocalKernelsCuda(fixture.inputs, result);
    benchmark::DoNotOptimize(result.barrierValues.data());
    benchmark::DoNotOptimize(result.frictionWorks.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_active_barriers"]
      = static_cast<double>(result.activeBarrierCount);
  state.counters["gpu_active_friction"]
      = static_cast<double>(result.activeFrictionCount);
  state.counters["gpu_dynamic_friction"]
      = static_cast<double>(result.dynamicFrictionCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083BarrierFrictionLocalCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

BENCHMARK_MAIN();
