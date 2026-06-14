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
 *     copyright notice, this list of conditions and the disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
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
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/newton_assembly_solve_cuda.cuh>

#include <cmath>
#include <cstddef>

namespace dart::simulation::compute::cuda::detail {
namespace {

__global__ void newtonAssemblyRowsKernel(
    const NewtonAssemblySolveRowInput* rows,
    double* assembledDiagonal,
    double* assembledGradient,
    const std::size_t rowCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t totalDofs = rowCount * kNewtonAssemblySolveDofsPerBody;
  if (index >= totalDofs) {
    return;
  }

  const std::size_t row = index / kNewtonAssemblySolveDofsPerBody;
  const std::size_t dof = index - row * kNewtonAssemblySolveDofsPerBody;
  const NewtonAssemblySolveRowInput input = rows[row];
  const std::size_t outputIndex = static_cast<std::size_t>(input.bodyIndex)
                                      * kNewtonAssemblySolveDofsPerBody
                                  + dof;
  atomicAdd(&assembledDiagonal[outputIndex], input.hessianDiagonal[dof]);
  atomicAdd(&assembledGradient[outputIndex], input.gradient[dof]);
}

__global__ void newtonOffDiagonalAssemblyRowsKernel(
    const NewtonOffDiagonalAssemblyRowInput* rows,
    double* assembledBlocks,
    const std::size_t rowCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t totalEntries = rowCount * kNewtonAssemblySolveBlockEntries;
  if (index >= totalEntries) {
    return;
  }

  const std::size_t row = index / kNewtonAssemblySolveBlockEntries;
  const std::size_t entry = index - row * kNewtonAssemblySolveBlockEntries;
  const NewtonOffDiagonalAssemblyRowInput input = rows[row];
  const std::size_t outputIndex = static_cast<std::size_t>(input.pairIndex)
                                      * kNewtonAssemblySolveBlockEntries
                                  + entry;
  atomicAdd(&assembledBlocks[outputIndex], input.hessianBlock[entry]);
}

__global__ void newtonDiagonalSolveKernel(
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* step,
    double* residual,
    const std::size_t dofCount,
    const double regularization,
    const double epsilonForDivision)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  const double effectiveDiagonal = assembledDiagonal[index] + regularization;
  double value = 0.0;
  if (isfinite(effectiveDiagonal)
      && fabs(effectiveDiagonal) > epsilonForDivision) {
    value = -assembledGradient[index] / effectiveDiagonal;
  }
  step[index] = value;
  residual[index] = effectiveDiagonal * value + assembledGradient[index];
}

} // namespace

//==============================================================================
cudaError_t launchNewtonAssemblyRowsKernel(
    const NewtonAssemblySolveRowInput* rows,
    double* assembledDiagonal,
    double* assembledGradient,
    const std::size_t rowCount)
{
  if (rowCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const std::size_t totalDofs = rowCount * kNewtonAssemblySolveDofsPerBody;
  const unsigned int gridSize = launchGrid1D(totalDofs, blockSize);
  newtonAssemblyRowsKernel<<<gridSize, blockSize>>>(
      rows, assembledDiagonal, assembledGradient, rowCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonOffDiagonalAssemblyRowsKernel(
    const NewtonOffDiagonalAssemblyRowInput* rows,
    double* assembledBlocks,
    const std::size_t rowCount)
{
  if (rowCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const std::size_t totalEntries = rowCount * kNewtonAssemblySolveBlockEntries;
  const unsigned int gridSize = launchGrid1D(totalEntries, blockSize);
  newtonOffDiagonalAssemblyRowsKernel<<<gridSize, blockSize>>>(
      rows, assembledBlocks, rowCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonDiagonalSolveKernel(
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* step,
    double* residual,
    const std::size_t dofCount,
    const double regularization,
    const double epsilonForDivision)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonDiagonalSolveKernel<<<gridSize, blockSize>>>(
      assembledDiagonal,
      assembledGradient,
      step,
      residual,
      dofCount,
      regularization,
      epsilonForDivision);
  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
