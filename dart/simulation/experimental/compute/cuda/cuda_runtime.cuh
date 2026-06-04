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

#pragma once

#include <cuda_runtime_api.h>

#include <string_view>

#include <cstddef>

// Shared device-runtime substrate for the experimental CUDA solver modules
// (rigid-body batch, deformable PSD projection, vertex block descent). Each
// module previously redeclared the runtime probe, re-defined the
// CUDA-error-to-exception helper, and hand-rolled the 1-D launch-grid math;
// this header is the single owner so new GPU solvers reuse it instead of
// reinventing it (see docs/design/shared_cuda_device_substrate.md, PLAN-031).
//
// This is a build-tree-only header: it uses the .cuh suffix so the experimental
// install rule (which globs only *.hpp) never exposes CUDA names in installed
// headers, and the compute-backend boundary scanner (which reads only .h/.hpp)
// never sees it.

namespace dart::simulation::experimental::compute::cuda {

/// Return whether the CUDA runtime currently exposes at least one device.
///
/// Single declaration for the whole experimental CUDA target (the definition
/// lives in cuda_runtime.cpp), replacing the per-module declarations that each
/// solver .cuh used to carry.
[[nodiscard]] bool isCudaRuntimeAvailable() noexcept;

/// Throw @c sx::InvalidOperationException when @p status is not @c cudaSuccess,
/// embedding @p operation and the CUDA error string.
///
/// Single owner for the error-mapping idiom every module used to define locally
/// (two of them byte-identical, the third diverging to @c std::runtime_error).
void throwIfCudaError(cudaError_t status, std::string_view operation);

/// Check @c cudaGetLastError() after a kernel launch and throw via
/// @ref throwIfCudaError on failure.
void checkLastError(std::string_view operation);

/// Number of thread blocks for a 1-D launch covering @p count items with
/// @p blockSize threads per block (the `(n + blockSize - 1) / blockSize`
/// ceil-div every module hand-rolled). The block size stays a per-call argument
/// so each kernel keeps its tuned launch shape (rigid 256, PSD/VBD 128).
[[nodiscard]] inline unsigned int launchGrid1D(
    std::size_t count, unsigned int blockSize) noexcept
{
  if (blockSize == 0) {
    return 0;
  }
  return static_cast<unsigned int>(
      (count + static_cast<std::size_t>(blockSize) - 1)
      / static_cast<std::size_t>(blockSize));
}

} // namespace dart::simulation::experimental::compute::cuda
