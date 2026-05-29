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

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cuda_runtime_api.h>
#include <dart/simulation/experimental/compute/cuda/deformable_psd_projection_cuda.cuh>

#include <string_view>
#include <vector>

#include <cstddef>

namespace sx = dart::simulation::experimental;

namespace dart::simulation::experimental::compute::cuda {
namespace detail {

cudaError_t launchProjectSymmetricBlocksToPsdKernel(
    double* blocks, std::size_t dimension, std::size_t blockCount);

} // namespace detail
namespace {

void throwIfCudaError(cudaError_t status, std::string_view operation)
{
  if (status == cudaSuccess) {
    return;
  }

  DART_EXPERIMENTAL_THROW_T(
      sx::InvalidOperationException,
      "CUDA {} failed: {}",
      operation,
      cudaGetErrorString(status));
}

class DeviceDoubleBuffer
{
public:
  explicit DeviceDoubleBuffer(std::size_t count) : m_count(count)
  {
    if (m_count == 0) {
      return;
    }

    throwIfCudaError(
        cudaMalloc(reinterpret_cast<void**>(&m_data), m_count * sizeof(double)),
        "allocation");
  }

  ~DeviceDoubleBuffer()
  {
    if (m_data != nullptr) {
      (void)cudaFree(m_data);
    }
  }

  DeviceDoubleBuffer(const DeviceDoubleBuffer&) = delete;
  DeviceDoubleBuffer& operator=(const DeviceDoubleBuffer&) = delete;

  [[nodiscard]] double* data() noexcept
  {
    return m_data;
  }

  [[nodiscard]] std::size_t byteSize() const noexcept
  {
    return m_count * sizeof(double);
  }

private:
  double* m_data = nullptr;
  std::size_t m_count = 0;
};

std::size_t validateBlocks(
    const std::vector<double>& blocks,
    std::size_t dimension,
    std::size_t blockCount,
    std::string_view operation)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      dimension == 0 || dimension > kMaxPsdBlockDimension,
      sx::InvalidArgumentException,
      "{} block dimension {} is out of the supported range [1, {}]",
      operation,
      dimension,
      kMaxPsdBlockDimension);

  const auto expected = dimension * dimension * blockCount;
  DART_EXPERIMENTAL_THROW_T_IF(
      blocks.size() != expected,
      sx::InvalidArgumentException,
      "{} expects {} doubles for {} blocks of {}x{} but received {}",
      operation,
      expected,
      blockCount,
      dimension,
      dimension,
      blocks.size());

  return expected;
}

} // namespace

// isCudaRuntimeAvailable() is defined once for the experimental CUDA target in
// rigid_body_state_batch_cuda.cpp; this file declares it via the .cuh and links
// against that single definition (both sources live in the same -cuda library).

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

  DART_EXPERIMENTAL_THROW_T_IF(
      !isCudaRuntimeAvailable(),
      sx::InvalidOperationException,
      "CUDA runtime has no available device");

  DeviceDoubleBuffer deviceBlocks(blocks.size());
  throwIfCudaError(
      cudaMemcpy(
          deviceBlocks.data(),
          blocks.data(),
          deviceBlocks.byteSize(),
          cudaMemcpyHostToDevice),
      "PSD block copy to device");

  throwIfCudaError(
      detail::launchProjectSymmetricBlocksToPsdKernel(
          deviceBlocks.data(), dimension, blockCount),
      "PSD projection kernel");
  throwIfCudaError(cudaDeviceSynchronize(), "PSD projection synchronize");

  throwIfCudaError(
      cudaMemcpy(
          blocks.data(),
          deviceBlocks.data(),
          deviceBlocks.byteSize(),
          cudaMemcpyDeviceToHost),
      "PSD block copy from device");
}

} // namespace dart::simulation::experimental::compute::cuda
