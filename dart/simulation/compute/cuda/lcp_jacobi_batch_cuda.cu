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
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <cmath>
#include <cstddef>

namespace dart::simulation::compute::cuda::detail {
namespace {

__global__ void solveBoxedLcpJacobiBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    const double* x,
    double* xNext,
    std::size_t problemSize,
    std::size_t problemCount,
    double relaxation,
    double epsilonForDivision)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t totalRows = problemSize * problemCount;
  if (index >= totalRows) {
    return;
  }

  const std::size_t problem = index / problemSize;
  const std::size_t row = index - problem * problemSize;
  const std::size_t vectorBase = problem * problemSize;
  const std::size_t matrixBase = problem * problemSize * problemSize;

  double Ax = 0.0;
  for (std::size_t col = 0; col < problemSize; ++col) {
    Ax += A[matrixBase + row * problemSize + col] * x[vectorBase + col];
  }

  double value = x[index];
  const double diag = A[matrixBase + row * problemSize + row];
  if (fabs(diag) >= epsilonForDivision) {
    const double w = Ax - b[index];
    const double step = x[index] - w / diag;
    value += relaxation * (step - x[index]);
  }

  double lower = lo[index];
  double upper = hi[index];
  const int ref = findex[index];
  if (ref >= 0) {
    const double mu = fabs(hi[index]);
    const double bound
        = mu * fabs(x[vectorBase + static_cast<std::size_t>(ref)]);
    lower = -bound;
    upper = bound;
  }

  if (value < lower) {
    value = lower;
  }
  if (value > upper) {
    value = upper;
  }
  xNext[index] = value;
}

__global__ void solveBoxedLcpPgsBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    double* x,
    std::size_t problemSize,
    std::size_t problemCount,
    std::size_t iterations,
    double relaxation,
    double epsilonForDivision)
{
  const auto problem
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (problem >= problemCount) {
    return;
  }

  const std::size_t vectorBase = problem * problemSize;
  const std::size_t matrixBase = problem * problemSize * problemSize;

  for (std::size_t iter = 0; iter < iterations; ++iter) {
    for (std::size_t row = 0; row < problemSize; ++row) {
      const std::size_t index = vectorBase + row;

      double Ax = 0.0;
      for (std::size_t col = 0; col < problemSize; ++col) {
        Ax += A[matrixBase + row * problemSize + col] * x[vectorBase + col];
      }

      double value = x[index];
      const double diag = A[matrixBase + row * problemSize + row];
      if (fabs(diag) >= epsilonForDivision) {
        const double w = Ax - b[index];
        const double step = x[index] - w / diag;
        value += relaxation * (step - x[index]);
      }

      double lower = lo[index];
      double upper = hi[index];
      const int ref = findex[index];
      if (ref >= 0) {
        const double mu = fabs(hi[index]);
        const double bound
            = mu * fabs(x[vectorBase + static_cast<std::size_t>(ref)]);
        lower = -bound;
        upper = bound;
      }

      if (value < lower) {
        value = lower;
      }
      if (value > upper) {
        value = upper;
      }
      x[index] = value;
    }
  }
}

__device__ double readRedBlackValue(
    const double* x,
    const double* xPrevious,
    std::size_t vectorBase,
    std::size_t col,
    int color)
{
  const std::size_t index = vectorBase + col;
  if (color == 0) {
    return xPrevious[index];
  }

  return (col % 2u == 0u) ? x[index] : xPrevious[index];
}

__global__ void solveBoxedLcpRedBlackGaussSeidelBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    double* x,
    const double* xPrevious,
    std::size_t problemSize,
    std::size_t problemCount,
    int color,
    double relaxation,
    double epsilonForDivision)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t totalRows = problemSize * problemCount;
  if (index >= totalRows) {
    return;
  }

  const std::size_t problem = index / problemSize;
  const std::size_t row = index - problem * problemSize;
  if (static_cast<int>(row % 2u) != color) {
    return;
  }

  const std::size_t vectorBase = problem * problemSize;
  const std::size_t matrixBase = problem * problemSize * problemSize;

  double Ax = 0.0;
  for (std::size_t col = 0; col < problemSize; ++col) {
    Ax += A[matrixBase + row * problemSize + col]
          * readRedBlackValue(x, xPrevious, vectorBase, col, color);
  }

  const double previous
      = readRedBlackValue(x, xPrevious, vectorBase, row, color);
  double value = previous;
  const double diag = A[matrixBase + row * problemSize + row];
  if (fabs(diag) >= epsilonForDivision) {
    const double w = Ax - b[index];
    const double step = previous - w / diag;
    value += relaxation * (step - previous);
  } else {
    value = 0.0;
  }

  double lower = lo[index];
  double upper = hi[index];
  const int ref = findex[index];
  if (ref >= 0) {
    const auto refIndex = static_cast<std::size_t>(ref);
    const double mu = fabs(hi[index]);
    const double bound
        = mu
          * fabs(readRedBlackValue(x, xPrevious, vectorBase, refIndex, color));
    lower = -bound;
    upper = bound;
  }

  if (value < lower) {
    value = lower;
  }
  if (value > upper) {
    value = upper;
  }
  x[index] = value;
}

} // namespace

//==============================================================================
cudaError_t launchBoxedLcpJacobiBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    const double* x,
    double* xNext,
    std::size_t problemSize,
    std::size_t problemCount,
    double relaxation,
    double epsilonForDivision)
{
  const std::size_t totalRows = problemSize * problemCount;
  if (totalRows == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(totalRows, blockSize);

  solveBoxedLcpJacobiBatchKernel<<<gridSize, blockSize>>>(
      A,
      b,
      lo,
      hi,
      findex,
      x,
      xNext,
      problemSize,
      problemCount,
      relaxation,
      epsilonForDivision);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchBoxedLcpRedBlackGaussSeidelBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    double* x,
    const double* xPrevious,
    std::size_t problemSize,
    std::size_t problemCount,
    int color,
    double relaxation,
    double epsilonForDivision)
{
  const std::size_t totalRows = problemSize * problemCount;
  if (totalRows == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(totalRows, blockSize);

  solveBoxedLcpRedBlackGaussSeidelBatchKernel<<<gridSize, blockSize>>>(
      A,
      b,
      lo,
      hi,
      findex,
      x,
      xPrevious,
      problemSize,
      problemCount,
      color,
      relaxation,
      epsilonForDivision);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchBoxedLcpPgsBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    double* x,
    std::size_t problemSize,
    std::size_t problemCount,
    std::size_t iterations,
    double relaxation,
    double epsilonForDivision)
{
  if (problemCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 128;
  const unsigned int gridSize = launchGrid1D(problemCount, blockSize);

  solveBoxedLcpPgsBatchKernel<<<gridSize, blockSize>>>(
      A,
      b,
      lo,
      hi,
      findex,
      x,
      problemSize,
      problemCount,
      iterations,
      relaxation,
      epsilonForDivision);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
