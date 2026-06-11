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

#include <dart/simulation/common/exceptions.hpp>

#include <cuda_runtime_api.h>
#include <dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/device_buffer.cuh>

#include <algorithm>
#include <chrono>

#include <cmath>

namespace sx = dart::simulation;

namespace dart::simulation::compute::cuda {
namespace detail {

cudaError_t launchBarrierFrictionLocalKernel(
    const BarrierFrictionLocalInput* inputs,
    double* barrierValues,
    double* barrierFirstDerivatives,
    double* barrierSecondDerivatives,
    double* frictionValues,
    double* frictionWorks,
    double* frictionFirstDerivatives,
    double* frictionSecondDerivatives,
    std::uint8_t* activeBarriers,
    std::uint8_t* activeFrictions,
    std::uint8_t* dynamicFrictions,
    std::size_t inputCount);

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

void validateInputs(const std::vector<BarrierFrictionLocalInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(input.squaredDistance)
            || !std::isfinite(input.squaredActivationDistance)
            || !std::isfinite(input.stiffness)
            || !std::isfinite(input.tangentialDisplacementNorm)
            || !std::isfinite(input.frictionWeight)
            || !std::isfinite(input.staticFrictionDisplacement),
        sx::InvalidArgumentException,
        "evaluateBarrierFrictionLocalKernelsCuda input {} has a non-finite "
        "field",
        i);
  }
}

} // namespace

//==============================================================================
void evaluateBarrierFrictionLocalKernelsCuda(
    const std::vector<BarrierFrictionLocalInput>& inputs,
    BarrierFrictionLocalResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = BarrierFrictionLocalResult{};
  result.barrierValues.resize(inputs.size(), 0.0);
  result.barrierFirstDerivatives.resize(inputs.size(), 0.0);
  result.barrierSecondDerivatives.resize(inputs.size(), 0.0);
  result.frictionValues.resize(inputs.size(), 0.0);
  result.frictionWorks.resize(inputs.size(), 0.0);
  result.frictionFirstDerivatives.resize(inputs.size(), 0.0);
  result.frictionSecondDerivatives.resize(inputs.size(), 0.0);
  result.activeBarriers.resize(inputs.size(), 0u);
  result.activeFrictions.resize(inputs.size(), 0u);
  result.dynamicFrictions.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<BarrierFrictionLocalInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceBarrierValues(inputs.size());
  DeviceBuffer<double> deviceBarrierFirstDerivatives(inputs.size());
  DeviceBuffer<double> deviceBarrierSecondDerivatives(inputs.size());
  DeviceBuffer<double> deviceFrictionValues(inputs.size());
  DeviceBuffer<double> deviceFrictionWorks(inputs.size());
  DeviceBuffer<double> deviceFrictionFirstDerivatives(inputs.size());
  DeviceBuffer<double> deviceFrictionSecondDerivatives(inputs.size());
  DeviceBuffer<std::uint8_t> deviceActiveBarriers(inputs.size());
  DeviceBuffer<std::uint8_t> deviceActiveFrictions(inputs.size());
  DeviceBuffer<std::uint8_t> deviceDynamicFrictions(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "barrier/friction local inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchBarrierFrictionLocalKernel(
          deviceInputs.data(),
          deviceBarrierValues.data(),
          deviceBarrierFirstDerivatives.data(),
          deviceBarrierSecondDerivatives.data(),
          deviceFrictionValues.data(),
          deviceFrictionWorks.data(),
          deviceFrictionFirstDerivatives.data(),
          deviceFrictionSecondDerivatives.data(),
          deviceActiveBarriers.data(),
          deviceActiveFrictions.data(),
          deviceDynamicFrictions.data(),
          inputs.size()),
      "barrier/friction local kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "barrier/friction local synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceBarrierValues.copyFromDevice(
      result.barrierValues, "barrier values copy");
  deviceBarrierFirstDerivatives.copyFromDevice(
      result.barrierFirstDerivatives, "barrier first derivatives copy");
  deviceBarrierSecondDerivatives.copyFromDevice(
      result.barrierSecondDerivatives, "barrier second derivatives copy");
  deviceFrictionValues.copyFromDevice(
      result.frictionValues, "friction values copy");
  deviceFrictionWorks.copyFromDevice(
      result.frictionWorks, "friction works copy");
  deviceFrictionFirstDerivatives.copyFromDevice(
      result.frictionFirstDerivatives, "friction first derivatives copy");
  deviceFrictionSecondDerivatives.copyFromDevice(
      result.frictionSecondDerivatives, "friction second derivatives copy");
  deviceActiveBarriers.copyFromDevice(
      result.activeBarriers, "active barriers copy");
  deviceActiveFrictions.copyFromDevice(
      result.activeFrictions, "active frictions copy");
  deviceDynamicFrictions.copyFromDevice(
      result.dynamicFrictions, "dynamic frictions copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    if (result.activeBarriers[i] != 0u) {
      ++result.activeBarrierCount;
      result.maxBarrierValue
          = std::max(result.maxBarrierValue, result.barrierValues[i]);
    }
    if (result.activeFrictions[i] != 0u) {
      ++result.activeFrictionCount;
      result.maxFrictionWork
          = std::max(result.maxFrictionWork, result.frictionWorks[i]);
    }
    if (result.dynamicFrictions[i] != 0u) {
      ++result.dynamicFrictionCount;
    }
  }
}

} // namespace dart::simulation::compute::cuda
