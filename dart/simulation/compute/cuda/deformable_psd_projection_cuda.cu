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

// Largest symmetric block edge length (mirrors kMaxPsdBlockDimension in the
// .cuh). The per-thread working storage is sized for this maximum.
constexpr int kMaxDim = 12;
// Cyclic Jacobi converges quadratically; a fixed sweep cap with an early-out is
// ample for symmetric matrices up to 12x12.
constexpr int kMaxSweeps = 30;

// Project one symmetric dim x dim matrix (row-major in `block`) onto the PSD
// cone in place: symmetric eigendecomposition by cyclic Jacobi, eigenvalues
// clamped to >= 0, recombined. Matches the CPU projectSymmetricToPsd.
__device__ void projectBlockToPsd(double* block, int dim)
{
  double a[kMaxDim * kMaxDim];
  double v[kMaxDim * kMaxDim];
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      a[i * dim + j] = block[i * dim + j];
      v[i * dim + j] = (i == j) ? 1.0 : 0.0;
    }
  }

  // Symmetrize defensively (the host passes symmetric blocks, but rounding in
  // assembly can leave tiny asymmetries).
  for (int i = 0; i < dim; ++i) {
    for (int j = i + 1; j < dim; ++j) {
      const double mean = 0.5 * (a[i * dim + j] + a[j * dim + i]);
      a[i * dim + j] = mean;
      a[j * dim + i] = mean;
    }
  }

  double scaleSquared = 0.0;
  for (int i = 0; i < dim * dim; ++i) {
    scaleSquared += a[i] * a[i];
  }
  const double threshold = 1e-30 * (scaleSquared + 1.0);

  for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
    double off = 0.0;
    for (int p = 0; p < dim; ++p) {
      for (int q = p + 1; q < dim; ++q) {
        off += a[p * dim + q] * a[p * dim + q];
      }
    }
    if (off <= threshold) {
      break;
    }

    for (int p = 0; p < dim; ++p) {
      for (int q = p + 1; q < dim; ++q) {
        const double apq = a[p * dim + q];
        if (apq == 0.0) {
          continue;
        }
        const double app = a[p * dim + p];
        const double aqq = a[q * dim + q];
        const double tau = (aqq - app) / (2.0 * apq);
        double t;
        if (tau >= 0.0) {
          t = 1.0 / (tau + sqrt(1.0 + tau * tau));
        } else {
          t = -1.0 / (-tau + sqrt(1.0 + tau * tau));
        }
        const double c = 1.0 / sqrt(1.0 + t * t);
        const double s = t * c;

        // A <- J^T A J: rotate columns p,q then rows p,q.
        for (int k = 0; k < dim; ++k) {
          const double akp = a[k * dim + p];
          const double akq = a[k * dim + q];
          a[k * dim + p] = c * akp - s * akq;
          a[k * dim + q] = s * akp + c * akq;
        }
        for (int k = 0; k < dim; ++k) {
          const double apk = a[p * dim + k];
          const double aqk = a[q * dim + k];
          a[p * dim + k] = c * apk - s * aqk;
          a[q * dim + k] = s * apk + c * aqk;
        }
        // V <- V J (accumulate eigenvectors as columns).
        for (int k = 0; k < dim; ++k) {
          const double vkp = v[k * dim + p];
          const double vkq = v[k * dim + q];
          v[k * dim + p] = c * vkp - s * vkq;
          v[k * dim + q] = s * vkp + c * vkq;
        }
      }
    }
  }

  double lambda[kMaxDim];
  for (int k = 0; k < dim; ++k) {
    lambda[k] = a[k * dim + k] > 0.0 ? a[k * dim + k] : 0.0;
  }

  // block <- V diag(lambda^+) V^T.
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      double sum = 0.0;
      for (int k = 0; k < dim; ++k) {
        sum += v[i * dim + k] * lambda[k] * v[j * dim + k];
      }
      block[i * dim + j] = sum;
    }
  }
}

__global__ void projectSymmetricBlocksToPsdKernel(
    double* blocks, int dim, std::size_t blockCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= blockCount) {
    return;
  }
  projectBlockToPsd(blocks + index * static_cast<std::size_t>(dim) * dim, dim);
}

} // namespace

//==============================================================================
cudaError_t launchProjectSymmetricBlocksToPsdKernel(
    double* blocks, std::size_t dimension, std::size_t blockCount)
{
  if (blockCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 128;
  const unsigned int gridSize = launchGrid1D(blockCount, blockSize);

  projectSymmetricBlocksToPsdKernel<<<gridSize, blockSize>>>(
      blocks, static_cast<int>(dimension), blockCount);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
