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

#include <dart/simulation/common/exceptions.hpp>

#include <cuda_runtime_api.h>
#include <dart/simulation/compute/cuda/ccd_line_search_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/device_buffer.cuh>

#include <algorithm>
#include <chrono>

#include <cmath>

namespace sx = dart::simulation;

namespace dart::simulation::compute::cuda {
namespace detail {

cudaError_t launchPointTriangleCcdLineSearchKernel(
    const PointTriangleCcdLineSearchPair* pairs,
    double minSeparation,
    double tolerance,
    double* stepBounds,
    std::uint8_t* hits,
    std::uint8_t* indeterminate,
    std::size_t pairCount);

cudaError_t launchEdgeEdgeCcdLineSearchKernel(
    const EdgeEdgeCcdLineSearchPair* pairs,
    double minSeparation,
    double tolerance,
    int maxIterations,
    double* stepBounds,
    std::uint8_t* hits,
    std::uint8_t* indeterminate,
    std::size_t pairCount);

} // namespace detail
namespace {

using Clock = std::chrono::steady_clock;

double elapsedNs(const Clock::time_point start, const Clock::time_point end)
{
  return static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
          .count());
}

void throwIfCudaRuntimeUnavailable()
{
  DART_SIMULATION_THROW_T_IF(
      !isCudaRuntimeAvailable(),
      sx::InvalidOperationException,
      "CUDA runtime has no available device");
}

void validateOptions(const CcdLineSearchOptions& options)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.minSeparation),
      sx::InvalidArgumentException,
      "CCD line-search CUDA evaluator expects finite minSeparation");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.tolerance),
      sx::InvalidArgumentException,
      "CCD line-search CUDA evaluator expects finite tolerance");
}

} // namespace

//==============================================================================
void evaluatePointTriangleCcdLineSearchCuda(
    const std::vector<PointTriangleCcdLineSearchPair>& pairs,
    const CcdLineSearchOptions& options,
    PointTriangleCcdLineSearchResult& result)
{
  const auto setupStart = Clock::now();
  validateOptions(options);

  result = PointTriangleCcdLineSearchResult{};
  result.stepBounds.resize(pairs.size(), 1.0);
  result.hits.resize(pairs.size(), 0u);
  result.indeterminate.resize(pairs.size(), 0u);

  if (pairs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<PointTriangleCcdLineSearchPair> devicePairs(pairs.size());
  DeviceBuffer<double> deviceStepBounds(pairs.size());
  DeviceBuffer<std::uint8_t> deviceHits(pairs.size());
  DeviceBuffer<std::uint8_t> deviceIndeterminate(pairs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  devicePairs.copyToDevice(pairs, "CCD line-search pairs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointTriangleCcdLineSearchKernel(
          devicePairs.data(),
          std::max(0.0, options.minSeparation),
          std::max(0.0, options.tolerance),
          deviceStepBounds.data(),
          deviceHits.data(),
          deviceIndeterminate.data(),
          pairs.size()),
      "CCD line-search kernel");
  throwIfCudaError(cudaDeviceSynchronize(), "CCD line-search synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceStepBounds.copyFromDevice(
      result.stepBounds, "CCD line-search step bounds copy");
  deviceHits.copyFromDevice(result.hits, "CCD line-search hits copy");
  deviceIndeterminate.copyFromDevice(
      result.indeterminate, "CCD line-search indeterminate copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  result.minStepBound = 1.0;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    if (result.hits[i] != 0u) {
      ++result.hitCount;
      result.minStepBound = std::min(result.minStepBound, result.stepBounds[i]);
    }
  }
}

//==============================================================================
void evaluateEdgeEdgeCcdLineSearchCuda(
    const std::vector<EdgeEdgeCcdLineSearchPair>& pairs,
    const CcdLineSearchOptions& options,
    EdgeEdgeCcdLineSearchResult& result)
{
  const auto setupStart = Clock::now();
  validateOptions(options);

  result = EdgeEdgeCcdLineSearchResult{};
  result.stepBounds.resize(pairs.size(), 1.0);
  result.hits.resize(pairs.size(), 0u);
  result.indeterminate.resize(pairs.size(), 0u);

  if (pairs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<EdgeEdgeCcdLineSearchPair> devicePairs(pairs.size());
  DeviceBuffer<double> deviceStepBounds(pairs.size());
  DeviceBuffer<std::uint8_t> deviceHits(pairs.size());
  DeviceBuffer<std::uint8_t> deviceIndeterminate(pairs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  devicePairs.copyToDevice(pairs, "edge-edge CCD line-search pairs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchEdgeEdgeCcdLineSearchKernel(
          devicePairs.data(),
          std::max(0.0, options.minSeparation),
          std::max(0.0, options.tolerance),
          options.maxIterations,
          deviceStepBounds.data(),
          deviceHits.data(),
          deviceIndeterminate.data(),
          pairs.size()),
      "edge-edge CCD line-search kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "edge-edge CCD line-search synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceStepBounds.copyFromDevice(
      result.stepBounds, "edge-edge CCD line-search step bounds copy");
  deviceHits.copyFromDevice(result.hits, "edge-edge CCD line-search hits copy");
  deviceIndeterminate.copyFromDevice(
      result.indeterminate, "edge-edge CCD line-search indeterminate copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  result.minStepBound = 1.0;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    if (result.hits[i] != 0u) {
      ++result.hitCount;
      result.minStepBound = std::min(result.minStepBound, result.stepBounds[i]);
    }
  }
}

} // namespace dart::simulation::compute::cuda
