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

#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/compute/deformable_psd_backend.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cuda_runtime_api.h>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/deformable_psd_projection_cuda.cuh>
#include <dart/simulation/compute/cuda/device_buffer.cuh>

#include <algorithm>
#include <string_view>
#include <vector>

#include <cstddef>

namespace sx = dart::simulation;

namespace dart::simulation::compute::cuda {
namespace detail {

cudaError_t launchProjectSymmetricBlocksToPsdKernel(
    double* blocks, std::size_t dimension, std::size_t blockCount);

} // namespace detail
namespace {

// CUDA runtime probing and error mapping now live in the shared substrate
// (cuda_runtime.cuh). The resident grow-and-reuse device scratch is the shared
// DeviceBuffer<double> with its opt-in ensure()/allocationCount()/release()
// path (device_buffer.cuh), so the per-call cudaMalloc/cudaFree round trip
// stays out of the projected-Newton hot path.

// The resident device buffer reused across GPU projections (a function-local
// static so it is constructed on first use and torn down at exit). Single-
// threaded accelerator use only until the later resident-device-owner packet
// moves this scratch behind a World-owned handle.
DeviceBuffer<double>& residentDeviceBuffer()
{
  static DeviceBuffer<double> buffer;
  return buffer;
}

void validateDimension(std::size_t dimension, std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      dimension == 0 || dimension > kMaxPsdBlockDimension,
      sx::InvalidArgumentException,
      "{} block dimension {} is out of the supported range [1, {}]",
      operation,
      dimension,
      kMaxPsdBlockDimension);
}

void validateBlocks(
    const std::vector<double>& blocks,
    std::size_t dimension,
    std::size_t blockCount,
    std::string_view operation)
{
  validateDimension(dimension, operation);

  const auto expected = dimension * dimension * blockCount;
  DART_SIMULATION_THROW_T_IF(
      blocks.size() != expected,
      sx::InvalidArgumentException,
      "{} expects {} doubles for {} blocks of {}x{} but received {}",
      operation,
      expected,
      blockCount,
      dimension,
      dimension,
      blocks.size());
}

// In-place GPU PSD projection over a raw packed block buffer, reusing the
// resident device buffer. Validates the dimension and device availability; the
// caller owns the element-count bookkeeping (so this serves both the public
// std::vector overload and the backend adapter without an extra host copy).
void projectSymmetricBlocksToPsdCudaInPlace(
    double* blocks, std::size_t dimension, std::size_t blockCount)
{
  validateDimension(dimension, "projectSymmetricBlocksToPsdCuda");
  if (blocks == nullptr || blockCount == 0) {
    return;
  }

  DART_SIMULATION_THROW_T_IF(
      !isCudaRuntimeAvailable(),
      sx::InvalidOperationException,
      "CUDA runtime has no available device");

  const std::size_t count = dimension * dimension * blockCount;
  const std::size_t byteSize = count * sizeof(double);
  double* device = residentDeviceBuffer().ensure(count);

  throwIfCudaError(
      cudaMemcpy(device, blocks, byteSize, cudaMemcpyHostToDevice),
      "PSD block copy to device");
  throwIfCudaError(
      detail::launchProjectSymmetricBlocksToPsdKernel(
          device, dimension, blockCount),
      "PSD projection kernel");
  throwIfCudaError(cudaDeviceSynchronize(), "PSD projection synchronize");
  throwIfCudaError(
      cudaMemcpy(blocks, device, byteSize, cudaMemcpyDeviceToHost),
      "PSD block copy from device");
}

} // namespace

//==============================================================================
void projectSymmetricBlocksToPsdReference(
    std::vector<double>& blocks, std::size_t dimension, std::size_t blockCount)
{
  validateBlocks(
      blocks, dimension, blockCount, "projectSymmetricBlocksToPsdReference");
  if (blockCount == 0) {
    return;
  }

  const auto dim = static_cast<Eigen::Index>(dimension);
  for (std::size_t k = 0; k < blockCount; ++k) {
    double* base = blocks.data() + k * dimension * dimension;
    // Symmetric blocks: row-major and column-major views coincide.
    Eigen::Map<Eigen::MatrixXd> block(base, dim, dim);
    const Eigen::MatrixXd symmetric = 0.5 * (block + block.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
    if (solver.info() != Eigen::Success) {
      block.setZero();
      continue;
    }
    Eigen::VectorXd eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    block = solver.eigenvectors() * eigenvalues.asDiagonal()
            * solver.eigenvectors().transpose();
  }
}

//==============================================================================
void projectSymmetricBlocksToPsdCuda(
    std::vector<double>& blocks, std::size_t dimension, std::size_t blockCount)
{
  validateBlocks(
      blocks, dimension, blockCount, "projectSymmetricBlocksToPsdCuda");
  if (blockCount == 0) {
    return;
  }
  projectSymmetricBlocksToPsdCudaInPlace(blocks.data(), dimension, blockCount);
}

namespace {

// Smallest batch the GPU path is worth: below this the host-to-device round
// trip dominates the per-block eigensolves, so the CPU backend is faster. The
// GpuVsCpuPerfGateAtSolverScale test measures the crossover for 12x12 blocks on
// an RTX 5000 Ada at ~1k blocks (256 blocks ran ~0.4x, 1024 ~1.4x, 4096 ~4x,
// 16384 ~9x), so the GPU path engages from roughly a thousand blocks up.
constexpr std::size_t kMinGpuBatchBlocks = 1024;

// Backend adapter matching compute::DeformablePsdBlockProjector. Offloads large
// batches to the GPU and defers small batches (or the no-device case) to the
// CPU backend, so installing this never changes results -- only where the
// arithmetic runs.
void cudaPsdBackendAdapter(
    double* blocks, std::size_t dimension, std::size_t blockCount)
{
  if (blocks == nullptr || dimension == 0 || blockCount == 0) {
    return;
  }
  if (blockCount < kMinGpuBatchBlocks || !isCudaRuntimeAvailable()) {
    projectSymmetricBlocksToPsdCpu(blocks, dimension, blockCount);
    return;
  }
  // Project in place on the caller's packed buffer through the resident device
  // buffer, so the offload adds no per-call host allocation or device
  // malloc/free -- only the host<->device copies the data movement requires.
  projectSymmetricBlocksToPsdCudaInPlace(blocks, dimension, blockCount);
}

} // namespace

//==============================================================================
void installCudaDeformablePsdBackend()
{
  setDeformablePsdBlockProjector(&cudaPsdBackendAdapter);
}

//==============================================================================
void restoreDefaultDeformablePsdBackend()
{
  setDeformablePsdBlockProjector(nullptr);
  // Free the device scratch when the GPU backend is uninstalled.
  residentDeviceBuffer().release();
}

//==============================================================================
std::size_t deformablePsdResidentBufferAllocationCount() noexcept
{
  return residentDeviceBuffer().allocationCount();
}

namespace {

// Register the CUDA PSD accelerator with the backend-neutral core control at
// load time, so the runtime (and the dartpy bindings) can toggle GPU offload
// without the public API naming CUDA. Consumers that want the accelerated path
// force-link this static library (see the dartpy whole-archive link) so this
// registrar runs.
struct CudaPsdAcceleratorRegistrar
{
  CudaPsdAcceleratorRegistrar()
  {
    setDeformablePsdAccelerator(
        {&isCudaRuntimeAvailable,
         &cudaPsdBackendAdapter,
         &restoreDefaultDeformablePsdBackend});
  }
};

const CudaPsdAcceleratorRegistrar g_cudaPsdAcceleratorRegistrar;

} // namespace

} // namespace dart::simulation::compute::cuda
