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

#include <cuda_runtime.h>

#include <cstddef>

namespace dart::simulation::experimental::compute::cuda::detail {
namespace {

__global__ void integrateRigidBodyStateBatchLinearKernel(
    double* position,
    double* linearVelocity,
    const double* force,
    const double* inverseMass,
    double timeStep,
    std::size_t bodies)
{
  const auto body
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (body >= bodies) {
    return;
  }

  const std::size_t base = 3 * body;
  const double velocityScale = inverseMass[body] * timeStep;

  linearVelocity[base] += force[base] * velocityScale;
  linearVelocity[base + 1] += force[base + 1] * velocityScale;
  linearVelocity[base + 2] += force[base + 2] * velocityScale;

  position[base] += linearVelocity[base] * timeStep;
  position[base + 1] += linearVelocity[base + 1] * timeStep;
  position[base + 2] += linearVelocity[base + 2] * timeStep;
}

} // namespace

//==============================================================================
cudaError_t launchRigidBodyStateBatchLinearKernel(
    double* position,
    double* linearVelocity,
    const double* force,
    const double* inverseMass,
    double timeStep,
    std::size_t bodies)
{
  if (bodies == 0) {
    return cudaSuccess;
  }

  constexpr int blockSize = 256;
  const auto gridSize = static_cast<unsigned int>(
      (bodies + static_cast<std::size_t>(blockSize) - 1)
      / static_cast<std::size_t>(blockSize));

  integrateRigidBodyStateBatchLinearKernel<<<gridSize, blockSize>>>(
      position, linearVelocity, force, inverseMass, timeStep, bodies);

  const auto launchStatus = cudaGetLastError();
  if (launchStatus != cudaSuccess) {
    return launchStatus;
  }

  return cudaDeviceSynchronize();
}

} // namespace dart::simulation::experimental::compute::cuda::detail
