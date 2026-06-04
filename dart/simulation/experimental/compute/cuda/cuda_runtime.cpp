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

#include <cuda_runtime_api.h>
#include <dart/simulation/experimental/compute/cuda/cuda_runtime.cuh>

#include <string_view>

// Host translation unit (compiled by the host compiler, not nvcc) so the single
// throwIfCudaError definition can use DART_EXPERIMENTAL_THROW_T (std::format +
// std::source_location) without requiring every caller's .cu to compile it.

namespace sx = dart::simulation::experimental;

namespace dart::simulation::experimental::compute::cuda {

//==============================================================================
bool isCudaRuntimeAvailable() noexcept
{
  int deviceCount = 0;
  const auto status = cudaGetDeviceCount(&deviceCount);
  return status == cudaSuccess && deviceCount > 0;
}

//==============================================================================
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

//==============================================================================
void checkLastError(std::string_view operation)
{
  throwIfCudaError(cudaGetLastError(), operation);
}

} // namespace dart::simulation::experimental::compute::cuda
