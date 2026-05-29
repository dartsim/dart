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

#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::compute::cuda {

/// Largest symmetric block edge length the batched PSD projection supports.
///
/// The IPC deformable projected-Newton assembly PSD-projects per-element spring
/// blocks (6x6) and self-contact barrier blocks (12x12), so 12 covers every
/// block the solver emits today.
inline constexpr std::size_t kMaxPsdBlockDimension = 12;

/// Return whether the CUDA runtime currently exposes at least one device.
///
/// This build-tree-only header intentionally uses a .cuh suffix so the
/// experimental install rule does not expose CUDA names in installed headers.
[[nodiscard]] bool isCudaRuntimeAvailable() noexcept;

/// Project a batch of symmetric blocks onto the nearest positive-semidefinite
/// matrices on the GPU, in place.
///
/// Each block is a @p dimension x @p dimension symmetric matrix stored
/// row-major and packed contiguously in @p blocks (block k occupies
/// `[k*dimension*dimension, (k+1)*dimension*dimension)`). Each block is
/// replaced by its projection onto the PSD cone, i.e. its symmetric
/// eigendecomposition with negative eigenvalues clamped to zero and recombined.
/// This matches the CPU `projectSymmetricToPsd` used by the deformable solver's
/// per-element Hessian assembly; it is the data-parallel building block for
/// offloading that hotspot.
///
/// @p dimension must be in [1, kMaxPsdBlockDimension]. The wrapper validates
/// the host buffer, then performs host-to-device transfer, a batched per-block
/// Jacobi eigensolve, and device-to-host readback so smoke tests exercise the
/// end-to-end path.
void projectSymmetricBlocksToPsdCuda(
    std::vector<double>& blocks, std::size_t dimension, std::size_t blockCount);

/// CPU reference for @ref projectSymmetricBlocksToPsdCuda with identical
/// semantics (Eigen self-adjoint eigensolve + eigenvalue clamp per block).
///
/// Tests and benchmarks compare the GPU path against this reference in the same
/// packet, per the Phase 5 GPU-prototype gate (identical-semantics CPU fallback
/// alongside the device path).
void projectSymmetricBlocksToPsdReference(
    std::vector<double>& blocks, std::size_t dimension, std::size_t blockCount);

} // namespace dart::simulation::experimental::compute::cuda
