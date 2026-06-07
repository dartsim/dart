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

#include <dart/simulation/export.hpp>

#include <cstddef>

namespace dart::simulation::compute {

/// Signature for a batched symmetric-block PSD projection backend.
///
/// Projects @p blockCount packed @p dimension x @p dimension row-major
/// symmetric blocks onto the positive-semidefinite cone in place (each block's
/// negative eigenvalues clamped to zero), matching the per-element projection
/// used by the deformable projected-Newton Hessian assembly. Block k occupies
/// `[k*dimension*dimension, (k+1)*dimension*dimension)` of @p blocks.
using DeformablePsdBlockProjector
    = void (*)(double* blocks, std::size_t dimension, std::size_t blockCount);

/// Built-in CPU backend: a per-block Eigen self-adjoint eigensolve with
/// negative eigenvalues clamped to zero (identical semantics to the solver's
/// per-element `projectSymmetricToPsd`). A block whose eigensolve fails is
/// zeroed, the conservative IPC choice that keeps the assembled Hessian
/// positive definite.
DART_SIMULATION_API void projectSymmetricBlocksToPsdCpu(
    double* blocks, std::size_t dimension, std::size_t blockCount);

/// Project a batch of symmetric blocks in place via the active backend.
///
/// Defaults to @ref projectSymmetricBlocksToPsdCpu. The optional CUDA sidecar
/// can install a GPU backend through @ref setDeformablePsdBlockProjector; the
/// core runtime never links or depends on CUDA. A null @p blocks or a zero
/// @p blockCount is a no-op.
DART_SIMULATION_API void projectSymmetricBlocksToPsd(
    double* blocks, std::size_t dimension, std::size_t blockCount);

/// Install a batched PSD projection backend (e.g. a GPU implementation) and
/// return the previously installed one. Passing nullptr restores the built-in
/// CPU backend. Intended to be set once during setup, not concurrently with a
/// running solve.
DART_SIMULATION_API DeformablePsdBlockProjector
setDeformablePsdBlockProjector(DeformablePsdBlockProjector projector);

/// Return the currently installed backend (never null; the CPU backend is the
/// default).
[[nodiscard]] DART_SIMULATION_API DeformablePsdBlockProjector
deformablePsdBlockProjector();

/// A registered acceleration backend for the deformable PSD projection.
///
/// An optional sidecar (compiled only when an accelerated backend is enabled)
/// may register itself here so the runtime can switch the PSD projection
/// between the built-in CPU path and the accelerated path. Backend-neutral by
/// design: the public API names the capability, not the device technology.
struct DeformablePsdAccelerator
{
  /// True when the accelerator reports an available device at runtime.
  bool (*available)() = nullptr;
  /// Install the accelerated projector; returns whether it became active.
  bool (*enable)() = nullptr;
  /// Restore the built-in CPU projector and release accelerator resources.
  void (*disable)() = nullptr;
};

/// Register (or clear, with a default-constructed value) the optional
/// accelerated PSD projector. Intended to be called once during setup by an
/// accelerator sidecar, not concurrently with a running solve.
DART_SIMULATION_API void setDeformablePsdAccelerator(
    const DeformablePsdAccelerator& accelerator);

/// Whether a registered accelerator reports an available device at runtime.
[[nodiscard]] DART_SIMULATION_API bool isDeformablePsdAccelerationAvailable();

/// Switch the deformable PSD projection to the registered accelerator (when
/// @p enabled and available) or back to the CPU backend. Process-wide. Returns
/// whether acceleration is active after the call (false when unavailable, so
/// the call is a safe no-op that stays on the CPU backend).
DART_SIMULATION_API bool setDeformablePsdAccelerated(bool enabled);

/// Whether the accelerated PSD projector is currently installed.
[[nodiscard]] DART_SIMULATION_API bool isDeformablePsdAccelerated();

} // namespace dart::simulation::compute
