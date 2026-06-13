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

__global__ void newtonEqualityReductionKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* reducedDiagonal,
    double* reducedGradient,
    const std::size_t entryCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= entryCount) {
    return;
  }

  const NewtonEqualityReductionEntry entry = entries[index];
  const double basis = entry.basisValue;
  atomicAdd(
      &reducedDiagonal[entry.reducedDofIndex],
      basis * basis * assembledDiagonal[entry.fullDofIndex]);
  atomicAdd(
      &reducedGradient[entry.reducedDofIndex],
      basis * assembledGradient[entry.fullDofIndex]);
}

__global__ void newtonSparseDiagonalResidualKernel(
    const double* assembledDiagonal,
    const double* step,
    double* residual,
    const std::size_t dofCount,
    const double regularization)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  residual[index] += (assembledDiagonal[index] + regularization) * step[index];
}

__global__ void newtonSparseBlockResidualKernel(
    const NewtonSparseBlockEntry* blocks,
    const double* step,
    double* residual,
    const std::size_t blockCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t totalEntries
      = blockCount * kNewtonAssemblySolveBlockEntries;
  if (index >= totalEntries) {
    return;
  }

  const std::size_t block = index / kNewtonAssemblySolveBlockEntries;
  const std::size_t entry = index - block * kNewtonAssemblySolveBlockEntries;
  const std::size_t localRow = entry / kNewtonAssemblySolveDofsPerBody;
  const std::size_t localColumn
      = entry - localRow * kNewtonAssemblySolveDofsPerBody;
  const NewtonSparseBlockEntry input = blocks[block];
  const double value = input.hessianBlock[entry];
  const std::size_t rowDof = static_cast<std::size_t>(input.rowBodyIndex)
                                 * kNewtonAssemblySolveDofsPerBody
                             + localRow;
  const std::size_t columnDof = static_cast<std::size_t>(input.columnBodyIndex)
                                    * kNewtonAssemblySolveDofsPerBody
                                + localColumn;
  atomicAdd(&residual[rowDof], value * step[columnDof]);
  atomicAdd(&residual[columnDof], value * step[rowDof]);
}

__global__ void newtonSparseJacobiUpdateKernel(
    const double* assembledDiagonal,
    const double* residual,
    double* step,
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
  if (isfinite(effectiveDiagonal)
      && fabs(effectiveDiagonal) > epsilonForDivision) {
    step[index] -= residual[index] / effectiveDiagonal;
  }
}

__global__ void newtonSparseCgSeedKernel(
    const double* assembledGradient,
    double* step,
    double* residual,
    double* direction,
    const std::size_t dofCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  const double value = -assembledGradient[index];
  step[index] = 0.0;
  residual[index] = value;
  direction[index] = value;
}

__global__ void newtonVectorDotKernel(
    const double* lhs,
    const double* rhs,
    double* output,
    const std::size_t dofCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  atomicAdd(output, lhs[index] * rhs[index]);
}

__global__ void newtonSparseCgStepKernel(
    double* step,
    double* residual,
    const double* direction,
    const double* matrixDirection,
    const std::size_t dofCount,
    const double alpha)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  step[index] += alpha * direction[index];
  residual[index] -= alpha * matrixDirection[index];
}

__global__ void newtonSparseCgDirectionKernel(
    const double* residual,
    double* direction,
    const std::size_t dofCount,
    const double beta)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  direction[index] = residual[index] + beta * direction[index];
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

__global__ void newtonExpandEqualityReducedStepKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* reducedStep,
    double* fullStep,
    const std::size_t entryCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= entryCount) {
    return;
  }

  const NewtonEqualityReductionEntry entry = entries[index];
  atomicAdd(
      &fullStep[entry.fullDofIndex],
      entry.basisValue * reducedStep[entry.reducedDofIndex]);
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
cudaError_t launchNewtonEqualityReductionKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* reducedDiagonal,
    double* reducedGradient,
    const std::size_t entryCount)
{
  if (entryCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(entryCount, blockSize);
  newtonEqualityReductionKernel<<<gridSize, blockSize>>>(
      entries,
      assembledDiagonal,
      assembledGradient,
      reducedDiagonal,
      reducedGradient,
      entryCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseDiagonalResidualKernel(
    const double* assembledDiagonal,
    const double* step,
    double* residual,
    const std::size_t dofCount,
    const double regularization)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseDiagonalResidualKernel<<<gridSize, blockSize>>>(
      assembledDiagonal, step, residual, dofCount, regularization);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseBlockResidualKernel(
    const NewtonSparseBlockEntry* blocks,
    const double* step,
    double* residual,
    const std::size_t blockCount)
{
  if (blockCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const std::size_t totalEntries
      = blockCount * kNewtonAssemblySolveBlockEntries;
  const unsigned int gridSize = launchGrid1D(totalEntries, blockSize);
  newtonSparseBlockResidualKernel<<<gridSize, blockSize>>>(
      blocks, step, residual, blockCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseJacobiUpdateKernel(
    const double* assembledDiagonal,
    const double* residual,
    double* step,
    const std::size_t dofCount,
    const double regularization,
    const double epsilonForDivision)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseJacobiUpdateKernel<<<gridSize, blockSize>>>(
      assembledDiagonal,
      residual,
      step,
      dofCount,
      regularization,
      epsilonForDivision);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseCgSeedKernel(
    const double* assembledGradient,
    double* step,
    double* residual,
    double* direction,
    const std::size_t dofCount)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseCgSeedKernel<<<gridSize, blockSize>>>(
      assembledGradient, step, residual, direction, dofCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonVectorDotKernel(
    const double* lhs,
    const double* rhs,
    double* output,
    const std::size_t dofCount)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonVectorDotKernel<<<gridSize, blockSize>>>(lhs, rhs, output, dofCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseCgStepKernel(
    double* step,
    double* residual,
    const double* direction,
    const double* matrixDirection,
    const std::size_t dofCount,
    const double alpha)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseCgStepKernel<<<gridSize, blockSize>>>(
      step, residual, direction, matrixDirection, dofCount, alpha);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseCgDirectionKernel(
    const double* residual,
    double* direction,
    const std::size_t dofCount,
    const double beta)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseCgDirectionKernel<<<gridSize, blockSize>>>(
      residual, direction, dofCount, beta);
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

//==============================================================================
cudaError_t launchNewtonExpandEqualityReducedStepKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* reducedStep,
    double* fullStep,
    const std::size_t entryCount)
{
  if (entryCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(entryCount, blockSize);
  newtonExpandEqualityReducedStepKernel<<<gridSize, blockSize>>>(
      entries, reducedStep, fullStep, entryCount);
  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
