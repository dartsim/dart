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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; OR
 *   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *   DAMAGE.
 */

// PLAN-083 private GPU packet row for the existing deformable PSD projection
// backend. The benchmark keeps the public API backend-neutral: it measures the
// private CUDA sidecar against the CPU reference on identical 12x12 symmetric
// block batches and exposes enough counters for the packet checker to enforce
// parity, timing-shape, and speedup gates.

#include <benchmark/benchmark.h>
#include <cuda_runtime_api.h>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/deformable_psd_projection_cuda.cuh>

#include <algorithm>
#include <chrono>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;

namespace dart::simulation::compute::cuda::detail {

cudaError_t launchProjectSymmetricBlocksToPsdKernel(
    double* blocks, std::size_t dimension, std::size_t blockCount);

} // namespace dart::simulation::compute::cuda::detail

namespace {

constexpr std::size_t kDimension = 12;
constexpr double kTolerance = 1e-9;

std::vector<double> makeSymmetricBlocks(std::size_t count)
{
  std::vector<double> blocks(kDimension * kDimension * count, 0.0);
  std::uint64_t seed = 0x9e3779b97f4a7c15ULL;
  const auto next = [&seed]() {
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return (static_cast<double>(seed >> 11) / 9007199254740992.0) * 3.0 - 1.5;
  };

  for (std::size_t k = 0; k < count; ++k) {
    double* base = blocks.data() + k * kDimension * kDimension;
    for (std::size_t i = 0; i < kDimension; ++i) {
      for (std::size_t j = i; j < kDimension; ++j) {
        const double value = next();
        base[i * kDimension + j] = value;
        base[j * kDimension + i] = value;
      }
    }
  }
  return blocks;
}

double maxAbsDifference(
    const std::vector<double>& lhs, const std::vector<double>& rhs)
{
  double maxDiff = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    maxDiff = std::max(maxDiff, std::abs(lhs[i] - rhs[i]));
  }
  return maxDiff;
}

double elapsedNs(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& end)
{
  return std::chrono::duration<double, std::nano>(end - start).count();
}

void setCommonCounters(
    benchmark::State& state,
    std::size_t count,
    double maxError,
    double hostSetupNs,
    double hostToDeviceNs,
    double kernelNs,
    double deviceToHostNs)
{
  state.counters["block_count"] = static_cast<double>(count);
  state.counters["dimension"] = static_cast<double>(kDimension);
  state.counters["max_result_abs_error"] = maxError;
  state.counters["result_abs_error_tolerance"] = kTolerance;
  state.counters["host_setup_ns"]
      = benchmark::Counter(hostSetupNs, benchmark::Counter::kAvgIterations);
  state.counters["host_to_device_ns"]
      = benchmark::Counter(hostToDeviceNs, benchmark::Counter::kAvgIterations);
  state.counters["kernel_ns"]
      = benchmark::Counter(kernelNs, benchmark::Counter::kAvgIterations);
  state.counters["device_to_host_ns"]
      = benchmark::Counter(deviceToHostNs, benchmark::Counter::kAvgIterations);
}

void BM_Plan083PsdProjectionCpu(benchmark::State& state)
{
  const auto count = static_cast<std::size_t>(state.range(0));
  const auto fixture = makeSymmetricBlocks(count);

  double hostSetupNs = 0.0;
  std::vector<double> blocks;
  for (auto _ : state) {
    const auto setupStart = std::chrono::steady_clock::now();
    blocks = fixture;
    const auto setupEnd = std::chrono::steady_clock::now();
    cuda::projectSymmetricBlocksToPsdReference(blocks, kDimension, count);
    hostSetupNs += elapsedNs(setupStart, setupEnd);
    benchmark::DoNotOptimize(blocks.data());
    benchmark::ClobberMemory();
  }

  std::vector<double> expected = fixture;
  cuda::projectSymmetricBlocksToPsdReference(expected, kDimension, count);
  setCommonCounters(
      state,
      count,
      maxAbsDifference(blocks, expected),
      hostSetupNs,
      0.0,
      0.0,
      0.0);
}

BENCHMARK(BM_Plan083PsdProjectionCpu)
    ->Arg(1024)
    ->Arg(4096)
    ->Arg(16384)
    ->UseRealTime();

void BM_Plan083PsdProjectionCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto count = static_cast<std::size_t>(state.range(0));
  const auto fixture = makeSymmetricBlocks(count);
  std::vector<double> expected = fixture;
  cuda::projectSymmetricBlocksToPsdReference(expected, kDimension, count);

  const std::size_t valueCount = fixture.size();
  const std::size_t byteSize = valueCount * sizeof(double);
  void* rawDevice = nullptr;
  cuda::throwIfCudaError(
      cudaMalloc(&rawDevice, byteSize), "PSD benchmark allocate");
  double* device = static_cast<double*>(rawDevice);

  std::vector<double> blocks = fixture;
  cuda::throwIfCudaError(
      cudaMemcpy(device, blocks.data(), byteSize, cudaMemcpyHostToDevice),
      "PSD benchmark warmup copy to device");
  cuda::throwIfCudaError(
      cuda::detail::launchProjectSymmetricBlocksToPsdKernel(
          device, kDimension, count),
      "PSD benchmark warmup kernel");
  cuda::throwIfCudaError(
      cudaDeviceSynchronize(), "PSD benchmark warmup synchronize");

  double hostSetupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double kernelNs = 0.0;
  double deviceToHostNs = 0.0;

  for (auto _ : state) {
    const auto setupStart = std::chrono::steady_clock::now();
    blocks = fixture;
    const auto setupEnd = std::chrono::steady_clock::now();

    const auto h2dStart = std::chrono::steady_clock::now();
    cuda::throwIfCudaError(
        cudaMemcpy(device, blocks.data(), byteSize, cudaMemcpyHostToDevice),
        "PSD benchmark copy to device");
    const auto h2dEnd = std::chrono::steady_clock::now();

    const auto kernelStart = std::chrono::steady_clock::now();
    cuda::throwIfCudaError(
        cuda::detail::launchProjectSymmetricBlocksToPsdKernel(
            device, kDimension, count),
        "PSD benchmark kernel");
    cuda::throwIfCudaError(
        cudaDeviceSynchronize(), "PSD benchmark synchronize");
    const auto kernelEnd = std::chrono::steady_clock::now();

    const auto d2hStart = std::chrono::steady_clock::now();
    cuda::throwIfCudaError(
        cudaMemcpy(blocks.data(), device, byteSize, cudaMemcpyDeviceToHost),
        "PSD benchmark copy from device");
    const auto d2hEnd = std::chrono::steady_clock::now();

    hostSetupNs += elapsedNs(setupStart, setupEnd);
    hostToDeviceNs += elapsedNs(h2dStart, h2dEnd);
    kernelNs += elapsedNs(kernelStart, kernelEnd);
    deviceToHostNs += elapsedNs(d2hStart, d2hEnd);
    benchmark::DoNotOptimize(blocks.data());
    benchmark::ClobberMemory();
  }

  cuda::throwIfCudaError(cudaFree(device), "PSD benchmark free");

  setCommonCounters(
      state,
      count,
      maxAbsDifference(blocks, expected),
      hostSetupNs,
      hostToDeviceNs,
      kernelNs,
      deviceToHostNs);
}

BENCHMARK(BM_Plan083PsdProjectionCuda)
    ->Arg(1024)
    ->Arg(4096)
    ->Arg(16384)
    ->UseRealTime();

} // namespace

BENCHMARK_MAIN();
