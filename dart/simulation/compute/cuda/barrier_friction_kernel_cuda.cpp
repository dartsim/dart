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

cudaError_t launchPointTriangleBarrierGradientKernel(
    const PointTriangleBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    std::uint8_t* activeBarriers,
    std::size_t inputCount);

cudaError_t launchPointPointBarrierHessianKernel(
    const PointPointBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
    std::uint8_t* activeBarriers,
    std::size_t inputCount);

cudaError_t launchPointTriangleBarrierHessianKernel(
    const PointTriangleBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
    std::uint8_t* activeBarriers,
    std::size_t inputCount);

cudaError_t launchPointEdgeBarrierHessianKernel(
    const PointEdgeBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
    std::uint8_t* activeBarriers,
    std::size_t inputCount);

cudaError_t launchEdgeEdgeBarrierHessianKernel(
    const EdgeEdgeBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
    std::uint8_t* activeBarriers,
    std::size_t inputCount);

cudaError_t launchPointTriangleTangentStencilKernel(
    const PointTriangleTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    std::size_t inputCount);

cudaError_t launchEdgeEdgeTangentStencilKernel(
    const EdgeEdgeTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    std::size_t inputCount);

cudaError_t launchPointEdgeTangentStencilKernel(
    const PointEdgeTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    std::size_t inputCount);

cudaError_t launchPointPointTangentStencilKernel(
    const PointPointTangentInput* inputs,
    double* basisValues,
    double* projectionValues,
    std::uint8_t* fallbackBases,
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

bool isFiniteVec3(const double values[3])
{
  return std::isfinite(values[0]) && std::isfinite(values[1])
         && std::isfinite(values[2]);
}

void validateInputs(const std::vector<PointTriangleBarrierInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !isFiniteVec3(input.point) || !isFiniteVec3(input.triangleA)
            || !isFiniteVec3(input.triangleB) || !isFiniteVec3(input.triangleC)
            || !std::isfinite(input.squaredActivationDistance)
            || !std::isfinite(input.stiffness),
        sx::InvalidArgumentException,
        "evaluatePointTriangleBarrierGradientsCuda input {} has a non-finite "
        "field",
        i);
  }
}

void validateInputs(const std::vector<PointPointBarrierInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !isFiniteVec3(input.pointA) || !isFiniteVec3(input.pointB)
            || !std::isfinite(input.squaredActivationDistance)
            || !std::isfinite(input.stiffness),
        sx::InvalidArgumentException,
        "evaluatePointPointBarrierHessiansCuda input {} has a non-finite "
        "field",
        i);
  }
}

void validateInputs(const std::vector<PointEdgeBarrierInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !isFiniteVec3(input.point) || !isFiniteVec3(input.edgeA)
            || !isFiniteVec3(input.edgeB)
            || !std::isfinite(input.squaredActivationDistance)
            || !std::isfinite(input.stiffness),
        sx::InvalidArgumentException,
        "evaluatePointEdgeBarrierHessiansCuda input {} has a non-finite "
        "field",
        i);
  }
}

void validateInputs(const std::vector<EdgeEdgeBarrierInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !isFiniteVec3(input.edgeA0) || !isFiniteVec3(input.edgeA1)
            || !isFiniteVec3(input.edgeB0) || !isFiniteVec3(input.edgeB1)
            || !std::isfinite(input.squaredActivationDistance)
            || !std::isfinite(input.stiffness),
        sx::InvalidArgumentException,
        "evaluateEdgeEdgeBarrierHessiansCuda input {} has a non-finite "
        "field",
        i);
  }
}

void validateInputs(const std::vector<PointTriangleTangentInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !isFiniteVec3(input.point) || !isFiniteVec3(input.triangleA)
            || !isFiniteVec3(input.triangleB) || !isFiniteVec3(input.triangleC),
        sx::InvalidArgumentException,
        "evaluatePointTriangleTangentStencilsCuda input {} has a non-finite "
        "field",
        i);
  }
}

void validateInputs(const std::vector<EdgeEdgeTangentInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !isFiniteVec3(input.edgeA0) || !isFiniteVec3(input.edgeA1)
            || !isFiniteVec3(input.edgeB0) || !isFiniteVec3(input.edgeB1),
        sx::InvalidArgumentException,
        "evaluateEdgeEdgeTangentStencilsCuda input {} has a non-finite field",
        i);
  }
}

void validateInputs(const std::vector<PointEdgeTangentInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !isFiniteVec3(input.point) || !isFiniteVec3(input.edgeA)
            || !isFiniteVec3(input.edgeB),
        sx::InvalidArgumentException,
        "evaluatePointEdgeTangentStencilsCuda input {} has a non-finite field",
        i);
  }
}

void validateInputs(const std::vector<PointPointTangentInput>& inputs)
{
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    DART_SIMULATION_THROW_T_IF(
        !isFiniteVec3(input.pointA) || !isFiniteVec3(input.pointB),
        sx::InvalidArgumentException,
        "evaluatePointPointTangentStencilsCuda input {} has a non-finite field",
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

//==============================================================================
void evaluatePointTriangleBarrierGradientsCuda(
    const std::vector<PointTriangleBarrierInput>& inputs,
    PointTriangleBarrierGradientResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = PointTriangleBarrierGradientResult{};
  result.squaredDistances.resize(inputs.size(), 0.0);
  result.barrierValues.resize(inputs.size(), 0.0);
  result.barrierGradients.resize(12 * inputs.size(), 0.0);
  result.activeBarriers.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<PointTriangleBarrierInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceSquaredDistances(inputs.size());
  DeviceBuffer<double> deviceBarrierValues(inputs.size());
  DeviceBuffer<double> deviceBarrierGradients(12 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceActiveBarriers(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "point-triangle barrier inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointTriangleBarrierGradientKernel(
          deviceInputs.data(),
          deviceSquaredDistances.data(),
          deviceBarrierValues.data(),
          deviceBarrierGradients.data(),
          deviceActiveBarriers.data(),
          inputs.size()),
      "point-triangle barrier gradient kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "point-triangle barrier gradient synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances, "point-triangle squared distances copy");
  deviceBarrierValues.copyFromDevice(
      result.barrierValues, "point-triangle barrier values copy");
  deviceBarrierGradients.copyFromDevice(
      result.barrierGradients, "point-triangle barrier gradients copy");
  deviceActiveBarriers.copyFromDevice(
      result.activeBarriers, "point-triangle active barriers copy");
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
  }
}

//==============================================================================
void evaluatePointTriangleBarrierHessiansCuda(
    const std::vector<PointTriangleBarrierInput>& inputs,
    PointTriangleBarrierHessianResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = PointTriangleBarrierHessianResult{};
  result.squaredDistances.resize(inputs.size(), 0.0);
  result.barrierValues.resize(inputs.size(), 0.0);
  result.barrierGradients.resize(12 * inputs.size(), 0.0);
  result.barrierHessians.resize(144 * inputs.size(), 0.0);
  result.activeBarriers.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<PointTriangleBarrierInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceSquaredDistances(inputs.size());
  DeviceBuffer<double> deviceBarrierValues(inputs.size());
  DeviceBuffer<double> deviceBarrierGradients(12 * inputs.size());
  DeviceBuffer<double> deviceBarrierHessians(144 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceActiveBarriers(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "point-triangle barrier inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointTriangleBarrierHessianKernel(
          deviceInputs.data(),
          deviceSquaredDistances.data(),
          deviceBarrierValues.data(),
          deviceBarrierGradients.data(),
          deviceBarrierHessians.data(),
          deviceActiveBarriers.data(),
          inputs.size()),
      "point-triangle barrier Hessian kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "point-triangle barrier Hessian synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances, "point-triangle squared distances copy");
  deviceBarrierValues.copyFromDevice(
      result.barrierValues, "point-triangle barrier values copy");
  deviceBarrierGradients.copyFromDevice(
      result.barrierGradients, "point-triangle barrier gradients copy");
  deviceBarrierHessians.copyFromDevice(
      result.barrierHessians, "point-triangle barrier Hessians copy");
  deviceActiveBarriers.copyFromDevice(
      result.activeBarriers, "point-triangle active barriers copy");
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
  }
}

//==============================================================================
void evaluatePointPointBarrierHessiansCuda(
    const std::vector<PointPointBarrierInput>& inputs,
    PointPointBarrierHessianResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = PointPointBarrierHessianResult{};
  result.squaredDistances.resize(inputs.size(), 0.0);
  result.barrierValues.resize(inputs.size(), 0.0);
  result.barrierGradients.resize(6 * inputs.size(), 0.0);
  result.barrierHessians.resize(36 * inputs.size(), 0.0);
  result.activeBarriers.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<PointPointBarrierInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceSquaredDistances(inputs.size());
  DeviceBuffer<double> deviceBarrierValues(inputs.size());
  DeviceBuffer<double> deviceBarrierGradients(6 * inputs.size());
  DeviceBuffer<double> deviceBarrierHessians(36 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceActiveBarriers(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "point-point barrier inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointPointBarrierHessianKernel(
          deviceInputs.data(),
          deviceSquaredDistances.data(),
          deviceBarrierValues.data(),
          deviceBarrierGradients.data(),
          deviceBarrierHessians.data(),
          deviceActiveBarriers.data(),
          inputs.size()),
      "point-point barrier Hessian kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "point-point barrier Hessian synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances, "point-point squared distances copy");
  deviceBarrierValues.copyFromDevice(
      result.barrierValues, "point-point barrier values copy");
  deviceBarrierGradients.copyFromDevice(
      result.barrierGradients, "point-point barrier gradients copy");
  deviceBarrierHessians.copyFromDevice(
      result.barrierHessians, "point-point barrier Hessians copy");
  deviceActiveBarriers.copyFromDevice(
      result.activeBarriers, "point-point active barriers copy");
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
  }
}

//==============================================================================
void evaluatePointEdgeBarrierHessiansCuda(
    const std::vector<PointEdgeBarrierInput>& inputs,
    PointEdgeBarrierHessianResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = PointEdgeBarrierHessianResult{};
  result.squaredDistances.resize(inputs.size(), 0.0);
  result.barrierValues.resize(inputs.size(), 0.0);
  result.barrierGradients.resize(9 * inputs.size(), 0.0);
  result.barrierHessians.resize(81 * inputs.size(), 0.0);
  result.activeBarriers.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<PointEdgeBarrierInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceSquaredDistances(inputs.size());
  DeviceBuffer<double> deviceBarrierValues(inputs.size());
  DeviceBuffer<double> deviceBarrierGradients(9 * inputs.size());
  DeviceBuffer<double> deviceBarrierHessians(81 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceActiveBarriers(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "point-edge barrier inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointEdgeBarrierHessianKernel(
          deviceInputs.data(),
          deviceSquaredDistances.data(),
          deviceBarrierValues.data(),
          deviceBarrierGradients.data(),
          deviceBarrierHessians.data(),
          deviceActiveBarriers.data(),
          inputs.size()),
      "point-edge barrier Hessian kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "point-edge barrier Hessian synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances, "point-edge squared distances copy");
  deviceBarrierValues.copyFromDevice(
      result.barrierValues, "point-edge barrier values copy");
  deviceBarrierGradients.copyFromDevice(
      result.barrierGradients, "point-edge barrier gradients copy");
  deviceBarrierHessians.copyFromDevice(
      result.barrierHessians, "point-edge barrier Hessians copy");
  deviceActiveBarriers.copyFromDevice(
      result.activeBarriers, "point-edge active barriers copy");
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
  }
}

//==============================================================================
void evaluateEdgeEdgeBarrierHessiansCuda(
    const std::vector<EdgeEdgeBarrierInput>& inputs,
    EdgeEdgeBarrierHessianResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = EdgeEdgeBarrierHessianResult{};
  result.squaredDistances.resize(inputs.size(), 0.0);
  result.barrierValues.resize(inputs.size(), 0.0);
  result.barrierGradients.resize(12 * inputs.size(), 0.0);
  result.barrierHessians.resize(144 * inputs.size(), 0.0);
  result.activeBarriers.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<EdgeEdgeBarrierInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceSquaredDistances(inputs.size());
  DeviceBuffer<double> deviceBarrierValues(inputs.size());
  DeviceBuffer<double> deviceBarrierGradients(12 * inputs.size());
  DeviceBuffer<double> deviceBarrierHessians(144 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceActiveBarriers(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "edge-edge barrier inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchEdgeEdgeBarrierHessianKernel(
          deviceInputs.data(),
          deviceSquaredDistances.data(),
          deviceBarrierValues.data(),
          deviceBarrierGradients.data(),
          deviceBarrierHessians.data(),
          deviceActiveBarriers.data(),
          inputs.size()),
      "edge-edge barrier Hessian kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "edge-edge barrier Hessian synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances, "edge-edge squared distances copy");
  deviceBarrierValues.copyFromDevice(
      result.barrierValues, "edge-edge barrier values copy");
  deviceBarrierGradients.copyFromDevice(
      result.barrierGradients, "edge-edge barrier gradients copy");
  deviceBarrierHessians.copyFromDevice(
      result.barrierHessians, "edge-edge barrier Hessians copy");
  deviceActiveBarriers.copyFromDevice(
      result.activeBarriers, "edge-edge active barriers copy");
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
  }
}

//==============================================================================
void evaluatePointTriangleTangentStencilsCuda(
    const std::vector<PointTriangleTangentInput>& inputs,
    PointTriangleTangentStencilResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = PointTriangleTangentStencilResult{};
  result.basisValues.resize(6 * inputs.size(), 0.0);
  result.coordinates.resize(2 * inputs.size(), 0.0);
  result.projectionValues.resize(24 * inputs.size(), 0.0);
  result.fallbackBases.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<PointTriangleTangentInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceBasisValues(6 * inputs.size());
  DeviceBuffer<double> deviceCoordinates(2 * inputs.size());
  DeviceBuffer<double> deviceProjectionValues(24 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceFallbackBases(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "point-triangle tangent inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointTriangleTangentStencilKernel(
          deviceInputs.data(),
          deviceBasisValues.data(),
          deviceCoordinates.data(),
          deviceProjectionValues.data(),
          deviceFallbackBases.data(),
          inputs.size()),
      "point-triangle tangent stencil kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "point-triangle tangent stencil synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceBasisValues.copyFromDevice(
      result.basisValues, "point-triangle tangent basis copy");
  deviceCoordinates.copyFromDevice(
      result.coordinates, "point-triangle tangent coordinates copy");
  deviceProjectionValues.copyFromDevice(
      result.projectionValues, "point-triangle tangent projections copy");
  deviceFallbackBases.copyFromDevice(
      result.fallbackBases, "point-triangle tangent fallbacks copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (const std::uint8_t fallback : result.fallbackBases) {
    if (fallback != 0u) {
      ++result.fallbackBasisCount;
    }
  }
}

//==============================================================================
void evaluateEdgeEdgeTangentStencilsCuda(
    const std::vector<EdgeEdgeTangentInput>& inputs,
    EdgeEdgeTangentStencilResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = EdgeEdgeTangentStencilResult{};
  result.basisValues.resize(6 * inputs.size(), 0.0);
  result.coordinates.resize(2 * inputs.size(), 0.0);
  result.projectionValues.resize(24 * inputs.size(), 0.0);
  result.fallbackBases.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<EdgeEdgeTangentInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceBasisValues(6 * inputs.size());
  DeviceBuffer<double> deviceCoordinates(2 * inputs.size());
  DeviceBuffer<double> deviceProjectionValues(24 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceFallbackBases(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "edge-edge tangent inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchEdgeEdgeTangentStencilKernel(
          deviceInputs.data(),
          deviceBasisValues.data(),
          deviceCoordinates.data(),
          deviceProjectionValues.data(),
          deviceFallbackBases.data(),
          inputs.size()),
      "edge-edge tangent stencil kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "edge-edge tangent stencil synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceBasisValues.copyFromDevice(
      result.basisValues, "edge-edge tangent basis copy");
  deviceCoordinates.copyFromDevice(
      result.coordinates, "edge-edge tangent coordinates copy");
  deviceProjectionValues.copyFromDevice(
      result.projectionValues, "edge-edge tangent projections copy");
  deviceFallbackBases.copyFromDevice(
      result.fallbackBases, "edge-edge tangent fallbacks copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (const std::uint8_t fallback : result.fallbackBases) {
    if (fallback != 0u) {
      ++result.fallbackBasisCount;
    }
  }
}

//==============================================================================
void evaluatePointEdgeTangentStencilsCuda(
    const std::vector<PointEdgeTangentInput>& inputs,
    PointEdgeTangentStencilResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = PointEdgeTangentStencilResult{};
  result.basisValues.resize(6 * inputs.size(), 0.0);
  result.coordinates.resize(inputs.size(), 0.0);
  result.projectionValues.resize(18 * inputs.size(), 0.0);
  result.fallbackBases.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<PointEdgeTangentInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceBasisValues(6 * inputs.size());
  DeviceBuffer<double> deviceCoordinates(inputs.size());
  DeviceBuffer<double> deviceProjectionValues(18 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceFallbackBases(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "point-edge tangent inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointEdgeTangentStencilKernel(
          deviceInputs.data(),
          deviceBasisValues.data(),
          deviceCoordinates.data(),
          deviceProjectionValues.data(),
          deviceFallbackBases.data(),
          inputs.size()),
      "point-edge tangent stencil kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "point-edge tangent stencil synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceBasisValues.copyFromDevice(
      result.basisValues, "point-edge tangent basis copy");
  deviceCoordinates.copyFromDevice(
      result.coordinates, "point-edge tangent coordinates copy");
  deviceProjectionValues.copyFromDevice(
      result.projectionValues, "point-edge tangent projections copy");
  deviceFallbackBases.copyFromDevice(
      result.fallbackBases, "point-edge tangent fallbacks copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (const std::uint8_t fallback : result.fallbackBases) {
    if (fallback != 0u) {
      ++result.fallbackBasisCount;
    }
  }
}

//==============================================================================
void evaluatePointPointTangentStencilsCuda(
    const std::vector<PointPointTangentInput>& inputs,
    PointPointTangentStencilResult& result)
{
  const auto setupStart = Clock::now();
  validateInputs(inputs);

  result = PointPointTangentStencilResult{};
  result.basisValues.resize(6 * inputs.size(), 0.0);
  result.projectionValues.resize(12 * inputs.size(), 0.0);
  result.fallbackBases.resize(inputs.size(), 0u);

  if (inputs.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<PointPointTangentInput> deviceInputs(inputs.size());
  DeviceBuffer<double> deviceBasisValues(6 * inputs.size());
  DeviceBuffer<double> deviceProjectionValues(12 * inputs.size());
  DeviceBuffer<std::uint8_t> deviceFallbackBases(inputs.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceInputs.copyToDevice(inputs, "point-point tangent inputs copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointPointTangentStencilKernel(
          deviceInputs.data(),
          deviceBasisValues.data(),
          deviceProjectionValues.data(),
          deviceFallbackBases.data(),
          inputs.size()),
      "point-point tangent stencil kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "point-point tangent stencil synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceBasisValues.copyFromDevice(
      result.basisValues, "point-point tangent basis copy");
  deviceProjectionValues.copyFromDevice(
      result.projectionValues, "point-point tangent projections copy");
  deviceFallbackBases.copyFromDevice(
      result.fallbackBases, "point-point tangent fallbacks copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (const std::uint8_t fallback : result.fallbackBases) {
    if (fallback != 0u) {
      ++result.fallbackBasisCount;
    }
  }
}

} // namespace dart::simulation::compute::cuda
