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

#include <dart/simulation/compute/rigid_body_state_batch.hpp>

#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <memory>
#include <string_view>
#include <vector>

#include <cstdint>

namespace dart::simulation::compute::cuda {

// isCudaRuntimeAvailable() is declared once in cuda_runtime.cuh (included
// above).

/// Integrate the linear SoA rigid-body batch path through CUDA.
///
/// The wrapper performs validation before allocating device memory. It includes
/// host-to-device transfer, kernel launch, synchronization, and device-to-host
/// transfer so smoke tests and benchmarks exercise the end-to-end path.
void integrateRigidBodyStateBatchLinearCuda(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep);

/// Roll out the linear rigid-body batch path on CUDA while keeping the working
/// state, model, and force buffers resident on the device for all steps.
///
/// This is intentionally private to the experimental CUDA target. It provides
/// the performance shape that a future device-resident backend should expose:
/// one host-to-device upload, many ordered kernel launches, and one
/// device-to-host download at the end.
void rolloutRigidBodyStateBatchLinearCuda(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep,
    std::size_t stepCount);

/// Integrate the full force-driven SoA rigid-body batch path through CUDA.
///
/// This matches @c integrateRigidBodyStateBatch without torque: linear
/// velocity, position, and orientation are updated from the same model, force,
/// and angular velocity buffers. The wrapper validates the host buffers before
/// touching the CUDA runtime and includes host/device transfer, kernel
/// execution, and readback.
void integrateRigidBodyStateBatchCuda(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep);

/// Run repeated full rigid-body batch steps through one CUDA transfer packet.
///
/// The state, model, and force buffers are copied to the device once, the
/// kernel runs @p stepCount times, and the final state is copied back.
/// Benchmarks use this path so Phase 5 packets measure setup, transfer,
/// compute, and readback for the full homogeneous-batch workload.
void rolloutRigidBodyStateBatchCuda(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep,
    std::size_t stepCount);

/// Copy/synchronization counters for the internal resident rigid-batch owner.
struct ResidentRigidBodyBatchCudaDiagnostics
{
  std::uint64_t modelUploadCount = 0;
  std::uint64_t stateUploadCount = 0;
  std::uint64_t controlUploadCount = 0;
  std::uint64_t stateDownloadCount = 0;
  std::uint64_t stepCount = 0;
  bool deviceStateValid = false;
  bool hostStateCoherent = false;
};

/// Internal owner for the CUDA rigid-batch residency slice.
///
/// This sidecar-only type makes the residency boundaries explicit: callers
/// upload immutable Model storage, upload mutable State, upload Control data,
/// run one or more resident steps, and download State only at a synchronization
/// boundary. It is deliberately kept under the optional CUDA target and never
/// appears in the public simulation facade.
class ResidentRigidBodyBatchCuda
{
public:
  ResidentRigidBodyBatchCuda();
  ~ResidentRigidBodyBatchCuda();

  ResidentRigidBodyBatchCuda(const ResidentRigidBodyBatchCuda&) = delete;
  ResidentRigidBodyBatchCuda& operator=(const ResidentRigidBodyBatchCuda&)
      = delete;

  ResidentRigidBodyBatchCuda(ResidentRigidBodyBatchCuda&&) noexcept;
  ResidentRigidBodyBatchCuda& operator=(ResidentRigidBodyBatchCuda&&) noexcept;

  void clear();

  /// Upload immutable Model storage to the device. The model shape becomes the
  /// resident shape and later State/Control uploads must match it.
  void uploadModel(const RigidBodyModelBatch& model);

  /// Upload mutable State storage to the device. Host State is coherent until a
  /// resident step succeeds or fails after kernel launch.
  void uploadState(const RigidBodyStateBatch& state);

  /// Upload one force slice used by subsequent resident steps.
  void uploadControl(const std::vector<double>& force);

  /// Run one resident CUDA step. This does not copy State back to the host.
  void step(double timeStep);

  /// Download resident State to @p state and mark host/device State coherent.
  void downloadState(RigidBodyStateBatch& state);

  /// Throws unless host State is coherent with the resident device State.
  void requireCoherentStateForHostFallback(std::string_view operation) const;

  [[nodiscard]] bool canFallbackToHost() const noexcept;
  [[nodiscard]] ResidentRigidBodyBatchCudaDiagnostics diagnostics()
      const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace dart::simulation::compute::cuda
