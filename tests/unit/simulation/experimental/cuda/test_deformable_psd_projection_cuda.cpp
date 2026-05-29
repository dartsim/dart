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

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/compute/deformable_psd_backend.hpp>

#include <dart/simulation/experimental/compute/cuda/deformable_psd_projection_cuda.cuh>
#include <gtest/gtest.h>

#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace cuda = dart::simulation::experimental::compute::cuda;
namespace sx = dart::simulation::experimental;

namespace {

// Build `count` deterministic symmetric `dim`x`dim` blocks (row-major, packed)
// with a mix of positive and negative eigenvalues, so PSD projection has real
// work to do. A small LCG keeps the data reproducible without <random>.
std::vector<double> makeSymmetricBlocks(std::size_t dim, std::size_t count)
{
  std::vector<double> blocks(dim * dim * count, 0.0);
  std::uint64_t seed = 0x9e3779b97f4a7c15ULL;
  const auto next = [&seed]() {
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    // Map to roughly [-1.5, 1.5].
    return (static_cast<double>(seed >> 11) / 9007199254740992.0) * 3.0 - 1.5;
  };

  for (std::size_t k = 0; k < count; ++k) {
    double* base = blocks.data() + k * dim * dim;
    for (std::size_t i = 0; i < dim; ++i) {
      for (std::size_t j = i; j < dim; ++j) {
        const double value = next();
        base[i * dim + j] = value;
        base[j * dim + i] = value;
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
    const double diff = std::abs(lhs[i] - rhs[i]);
    if (diff > maxDiff) {
      maxDiff = diff;
    }
  }
  return maxDiff;
}

} // namespace

//==============================================================================
TEST(CudaDeformablePsdProjection, RejectsUnsupportedDimensionBeforeRuntime)
{
  std::vector<double> blocks(4, 0.0);
  EXPECT_THROW(
      cuda::projectSymmetricBlocksToPsdReference(blocks, 0, 1),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      cuda::projectSymmetricBlocksToPsdReference(
          blocks, cuda::kMaxPsdBlockDimension + 1, 1),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(CudaDeformablePsdProjection, RejectsMismatchedBufferSizeBeforeRuntime)
{
  std::vector<double> blocks(5, 0.0); // not 2*2*1 = 4
  EXPECT_THROW(
      cuda::projectSymmetricBlocksToPsdReference(blocks, 2, 1),
      sx::InvalidArgumentException);
}

//==============================================================================
// The CPU reference clamps negative eigenvalues to zero (a diagonal matrix is
// its own eigendecomposition, so the answer is closed-form).
TEST(CudaDeformablePsdProjection, ReferenceClampsNegativeEigenvalues)
{
  std::vector<double> blocks = {2.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 3.0};
  cuda::projectSymmetricBlocksToPsdReference(blocks, 3, 1);

  const std::vector<double> expected
      = {2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0};
  EXPECT_LT(maxAbsDifference(blocks, expected), 1e-12);
}

//==============================================================================
TEST(CudaDeformablePsdProjection, MatchesCpuReferenceForSpringBlocks)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  constexpr std::size_t dim = 6; // spring stretch Hessian block size
  constexpr std::size_t count = 512;
  const auto reference = makeSymmetricBlocks(dim, count);

  std::vector<double> cpu = reference;
  cuda::projectSymmetricBlocksToPsdReference(cpu, dim, count);

  std::vector<double> gpu = reference;
  cuda::projectSymmetricBlocksToPsdCuda(gpu, dim, count);

  EXPECT_LT(maxAbsDifference(cpu, gpu), 1e-9);
}

//==============================================================================
TEST(CudaDeformablePsdProjection, MatchesCpuReferenceForBarrierBlocks)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  constexpr std::size_t dim = 12; // self-contact barrier Hessian block size
  constexpr std::size_t count = 512;
  const auto reference = makeSymmetricBlocks(dim, count);

  std::vector<double> cpu = reference;
  cuda::projectSymmetricBlocksToPsdReference(cpu, dim, count);

  std::vector<double> gpu = reference;
  cuda::projectSymmetricBlocksToPsdCuda(gpu, dim, count);

  EXPECT_LT(maxAbsDifference(cpu, gpu), 1e-9);
}

//==============================================================================
// The projected blocks are positive semidefinite: every diagonal entry is
// non-negative and the block is symmetric (a necessary PSD condition that is
// cheap to check element-wise here).
TEST(
    CudaDeformablePsdProjection, ProjectedBlocksAreSymmetricNonNegativeDiagonal)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  constexpr std::size_t dim = 12;
  constexpr std::size_t count = 128;
  std::vector<double> gpu = makeSymmetricBlocks(dim, count);
  cuda::projectSymmetricBlocksToPsdCuda(gpu, dim, count);

  for (std::size_t k = 0; k < count; ++k) {
    const double* base = gpu.data() + k * dim * dim;
    for (std::size_t i = 0; i < dim; ++i) {
      EXPECT_GE(base[i * dim + i], -1e-9);
      for (std::size_t j = i + 1; j < dim; ++j) {
        EXPECT_NEAR(base[i * dim + j], base[j * dim + i], 1e-9);
      }
    }
  }
}

//==============================================================================
// The CUDA backend installs into the core PSD-projection seam that the
// deformable projected-Newton assembly uses. Projecting a large batch through
// the solver-facing compute::projectSymmetricBlocksToPsd entry point routes to
// the GPU and matches the CPU reference, and restoring the default returns the
// seam to the CPU backend (a bit-identical no-op for already-CPU results).
TEST(CudaDeformablePsdProjection, BackendInjectionRoutesThroughCoreSeam)
{
  namespace compute = dart::simulation::experimental::compute;
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  constexpr std::size_t dim = 12;
  constexpr std::size_t count = 256; // above the GPU batch threshold
  const auto reference = makeSymmetricBlocks(dim, count);

  std::vector<double> cpu = reference;
  compute::projectSymmetricBlocksToPsdCpu(cpu.data(), dim, count);

  cuda::installCudaDeformablePsdBackend();
  std::vector<double> viaSeam = reference;
  compute::projectSymmetricBlocksToPsd(viaSeam.data(), dim, count);
  cuda::restoreDefaultDeformablePsdBackend();

  EXPECT_LT(maxAbsDifference(cpu, viaSeam), 1e-9);

  // After restore the seam is back on the CPU backend (identical to a direct
  // CPU projection).
  std::vector<double> afterRestore = reference;
  compute::projectSymmetricBlocksToPsd(afterRestore.data(), dim, count);
  EXPECT_LT(maxAbsDifference(cpu, afterRestore), 1e-12);
}
