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
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 *   THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *   SUCH DAMAGE.
 */

#include <cuda_runtime.h>
#include <dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::compute::cuda::detail {
namespace {

struct BarrierScalar
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  std::uint8_t active = 0u;
};

struct SmoothFriction
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  std::uint8_t active = 0u;
  std::uint8_t dynamic = 0u;
};

__device__ BarrierScalar c2ClampedLogBarrierDevice(
    const double squaredDistance, const double squaredActivationDistance)
{
  BarrierScalar result;
  if (!isfinite(squaredActivationDistance) || squaredActivationDistance <= 0.0
      || !isfinite(squaredDistance)) {
    return result;
  }

  if (squaredDistance >= squaredActivationDistance) {
    return result;
  }

  const double activeInteriorLimit = nextafter(squaredActivationDistance, 0.0);
  if (!(activeInteriorLimit > 0.0)) {
    return result;
  }

  constexpr double kDistanceFloorScale = 1e-16;
  constexpr double kDenormMin = 4.9406564584124654e-324;
  const double floor = fmin(
      fmax(kDenormMin, kDistanceFloorScale * squaredActivationDistance),
      activeInteriorLimit);
  const double d = fmax(squaredDistance, floor);
  const double dHat = squaredActivationDistance;
  const double offset = d - dHat;
  const double logRatio = log(d / dHat);
  const double offsetSquared = offset * offset;

  result.value = -offsetSquared * logRatio;
  result.firstDerivative = -2.0 * offset * logRatio - offsetSquared / d;
  result.secondDerivative
      = -2.0 * logRatio - 4.0 * offset / d + offsetSquared / (d * d);
  result.active = 1u;
  return result;
}

__device__ SmoothFriction
smoothFrictionNormDevice(const double norm, const double staticDisplacement)
{
  SmoothFriction result;
  if (!(norm >= 0.0) || !isfinite(norm) || !(staticDisplacement > 0.0)
      || !isfinite(staticDisplacement)) {
    return result;
  }

  result.active = 1u;
  if (norm > staticDisplacement) {
    result.value = norm;
    result.firstDerivative = 1.0;
    result.secondDerivative = 0.0;
    result.dynamic = 1u;
    return result;
  }

  const double invEps = 1.0 / staticDisplacement;
  const double invEpsSquared = invEps * invEps;
  result.value = norm * norm * (1.0 - norm * invEps / 3.0) * invEps
                 + staticDisplacement / 3.0;
  result.firstDerivative = 2.0 * norm * invEps - norm * norm * invEpsSquared;
  result.secondDerivative = 2.0 * invEps - 2.0 * norm * invEpsSquared;
  return result;
}

__global__ void barrierFrictionLocalKernel(
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
    const std::size_t inputCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= inputCount) {
    return;
  }

  barrierValues[index] = 0.0;
  barrierFirstDerivatives[index] = 0.0;
  barrierSecondDerivatives[index] = 0.0;
  frictionValues[index] = 0.0;
  frictionWorks[index] = 0.0;
  frictionFirstDerivatives[index] = 0.0;
  frictionSecondDerivatives[index] = 0.0;
  activeBarriers[index] = 0u;
  activeFrictions[index] = 0u;
  dynamicFrictions[index] = 0u;

  const BarrierFrictionLocalInput input = inputs[index];
  const double stiffness
      = isfinite(input.stiffness) ? fmax(0.0, input.stiffness) : 0.0;
  const BarrierScalar barrier = c2ClampedLogBarrierDevice(
      input.squaredDistance, input.squaredActivationDistance);
  if (barrier.active != 0u && stiffness > 0.0) {
    activeBarriers[index] = 1u;
    barrierValues[index] = stiffness * barrier.value;
    barrierFirstDerivatives[index] = stiffness * barrier.firstDerivative;
    barrierSecondDerivatives[index] = stiffness * barrier.secondDerivative;
  }

  const double weight
      = isfinite(input.frictionWeight) ? fmax(0.0, input.frictionWeight) : 0.0;
  const SmoothFriction friction = smoothFrictionNormDevice(
      input.tangentialDisplacementNorm, input.staticFrictionDisplacement);
  if (friction.active != 0u && weight > 0.0) {
    activeFrictions[index] = 1u;
    dynamicFrictions[index] = friction.dynamic;
    frictionValues[index] = weight * friction.value;
    frictionWorks[index]
        = weight * friction.firstDerivative * input.tangentialDisplacementNorm;
    frictionFirstDerivatives[index] = friction.firstDerivative;
    frictionSecondDerivatives[index] = friction.secondDerivative;
  }
}

} // namespace

//==============================================================================
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
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  barrierFrictionLocalKernel<<<gridSize, blockSize>>>(
      inputs,
      barrierValues,
      barrierFirstDerivatives,
      barrierSecondDerivatives,
      frictionValues,
      frictionWorks,
      frictionFirstDerivatives,
      frictionSecondDerivatives,
      activeBarriers,
      activeFrictions,
      dynamicFrictions,
      inputCount);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
