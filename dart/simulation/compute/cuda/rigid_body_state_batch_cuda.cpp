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

#include <cuda_runtime_api.h>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/device_buffer.cuh>
#include <dart/simulation/compute/cuda/rigid_body_state_batch_cuda.cuh>

#include <memory>
#include <string_view>
#include <vector>

#include <cstddef>

namespace sx = dart::simulation;

namespace dart::simulation::compute::cuda {
namespace detail {

cudaError_t launchRigidBodyStateBatchLinearKernel(
    double* position,
    double* linearVelocity,
    const double* force,
    const double* inverseMass,
    double timeStep,
    std::size_t bodies);

cudaError_t launchRigidBodyStateBatchKernel(
    double* position,
    double* linearVelocity,
    double* orientation,
    const double* angularVelocity,
    const double* force,
    const double* inverseMass,
    double timeStep,
    std::size_t bodies);

} // namespace detail
namespace {

constexpr std::size_t kLinearComponents = 3;
constexpr std::size_t kOrientationComponents = 4;

// CUDA runtime probing, error mapping, and the owning device buffer now live in
// the shared substrate (cuda_runtime.cuh / device_buffer.cuh).

std::size_t validateLinearInputs(
    const RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      model.worldCount != state.worldCount
          || model.bodyCount != state.bodyCount,
      sx::InvalidArgumentException,
      "{} model batch ({}x{}) does not match the state batch ({}x{})",
      operation,
      model.worldCount,
      model.bodyCount,
      state.worldCount,
      state.bodyCount);

  const auto bodies = state.worldCount * state.bodyCount;
  DART_SIMULATION_THROW_T_IF(
      state.position.size() != kLinearComponents * bodies
          || state.linearVelocity.size() != kLinearComponents * bodies
          || force.size() != kLinearComponents * bodies
          || model.inverseMass.size() != bodies,
      sx::InvalidArgumentException,
      "{} arrays are inconsistent with worldCount {} and bodyCount {}",
      operation,
      state.worldCount,
      state.bodyCount);

  return bodies;
}

std::size_t validateFullInputs(
    const RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    std::string_view operation)
{
  const auto bodies = validateLinearInputs(state, model, force, operation);

  DART_SIMULATION_THROW_T_IF(
      state.orientation.size() != kOrientationComponents * bodies
          || state.angularVelocity.size() != kLinearComponents * bodies,
      sx::InvalidArgumentException,
      "{} orientation/angular-velocity arrays are inconsistent with worldCount "
      "{} and bodyCount {}",
      operation,
      state.worldCount,
      state.bodyCount);

  return bodies;
}

void throwIfCudaRuntimeUnavailable()
{
  DART_SIMULATION_THROW_T_IF(
      !isCudaRuntimeAvailable(),
      sx::InvalidOperationException,
      "CUDA runtime has no available device");
}

std::size_t validateModelInputs(
    const RigidBodyModelBatch& model, std::string_view operation)
{
  const auto bodies = model.worldCount * model.bodyCount;
  DART_SIMULATION_THROW_T_IF(
      model.inverseMass.size() != bodies,
      sx::InvalidArgumentException,
      "{} inverse-mass array has size {}, expected {} "
      "(worldCount {}, bodyCount {})",
      operation,
      model.inverseMass.size(),
      bodies,
      model.worldCount,
      model.bodyCount);
  return bodies;
}

std::size_t validateStateInputs(
    const RigidBodyStateBatch& state, std::string_view operation)
{
  const auto bodies = state.worldCount * state.bodyCount;
  DART_SIMULATION_THROW_T_IF(
      state.position.size() != kLinearComponents * bodies
          || state.linearVelocity.size() != kLinearComponents * bodies
          || state.orientation.size() != kOrientationComponents * bodies
          || state.angularVelocity.size() != kLinearComponents * bodies,
      sx::InvalidArgumentException,
      "{} arrays are inconsistent with worldCount {} and bodyCount {}",
      operation,
      state.worldCount,
      state.bodyCount);
  return bodies;
}

void validateResidentShapeMatches(
    const RigidBodyModelBatch& model,
    std::size_t worldCount,
    std::size_t bodyCount,
    std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      model.worldCount != worldCount || model.bodyCount != bodyCount,
      sx::InvalidArgumentException,
      "{} shape ({}x{}) does not match resident Model shape ({}x{})",
      operation,
      worldCount,
      bodyCount,
      model.worldCount,
      model.bodyCount);
}

void validateControlInputs(
    const std::vector<double>& force,
    std::size_t worldCount,
    std::size_t bodyCount,
    std::string_view operation)
{
  const auto bodies = worldCount * bodyCount;
  DART_SIMULATION_THROW_T_IF(
      force.size() != kLinearComponents * bodies,
      sx::InvalidArgumentException,
      "{} force array has size {}, expected {} "
      "(worldCount {}, bodyCount {})",
      operation,
      force.size(),
      kLinearComponents * bodies,
      worldCount,
      bodyCount);
}

} // namespace

struct ResidentRigidBodyBatchCuda::Impl
{
  void clear() noexcept
  {
    devicePosition.release();
    deviceLinearVelocity.release();
    deviceOrientation.release();
    deviceAngularVelocity.release();
    deviceForce.release();
    deviceInverseMass.release();
    model = RigidBodyModelBatch{};
    diagnostics = ResidentRigidBodyBatchCudaDiagnostics{};
    modelUploaded = false;
    stateUploaded = false;
    controlUploaded = false;
  }

  [[nodiscard]] std::size_t bodies() const noexcept
  {
    return model.worldCount * model.bodyCount;
  }

  RigidBodyModelBatch model;
  DeviceBuffer<double> devicePosition;
  DeviceBuffer<double> deviceLinearVelocity;
  DeviceBuffer<double> deviceOrientation;
  DeviceBuffer<double> deviceAngularVelocity;
  DeviceBuffer<double> deviceForce;
  DeviceBuffer<double> deviceInverseMass;
  ResidentRigidBodyBatchCudaDiagnostics diagnostics;
  bool modelUploaded = false;
  bool stateUploaded = false;
  bool controlUploaded = false;
};

//==============================================================================
ResidentRigidBodyBatchCuda::ResidentRigidBodyBatchCuda()
  : m_impl(std::make_unique<Impl>())
{
  // Default construction intentionally leaves all residency boundaries empty.
}

//==============================================================================
ResidentRigidBodyBatchCuda::~ResidentRigidBodyBatchCuda() = default;

//==============================================================================
ResidentRigidBodyBatchCuda::ResidentRigidBodyBatchCuda(
    ResidentRigidBodyBatchCuda&&) noexcept = default;

//==============================================================================
ResidentRigidBodyBatchCuda& ResidentRigidBodyBatchCuda::operator=(
    ResidentRigidBodyBatchCuda&&) noexcept = default;

//==============================================================================
void ResidentRigidBodyBatchCuda::clear()
{
  m_impl->clear();
}

//==============================================================================
void ResidentRigidBodyBatchCuda::uploadModel(const RigidBodyModelBatch& model)
{
  const auto bodies = validateModelInputs(model, "ResidentRigidBodyBatchCuda");
  if (bodies > 0) {
    throwIfCudaRuntimeUnavailable();
  }

  m_impl->deviceInverseMass.ensure(model.inverseMass.size());
  m_impl->deviceInverseMass.copyToDevice(
      model.inverseMass, "resident rigid-body inverse mass upload");
  m_impl->model = model;
  m_impl->modelUploaded = true;
  m_impl->stateUploaded = false;
  m_impl->controlUploaded = false;
  m_impl->diagnostics.deviceStateValid = false;
  m_impl->diagnostics.hostStateCoherent = false;
  ++m_impl->diagnostics.modelUploadCount;
}

//==============================================================================
void ResidentRigidBodyBatchCuda::uploadState(const RigidBodyStateBatch& state)
{
  const auto bodies = validateStateInputs(state, "ResidentRigidBodyBatchCuda");
  DART_SIMULATION_THROW_T_IF(
      !m_impl->modelUploaded,
      sx::InvalidOperationException,
      "ResidentRigidBodyBatchCuda requires uploadModel() before uploadState()");
  validateResidentShapeMatches(
      m_impl->model,
      state.worldCount,
      state.bodyCount,
      "ResidentRigidBodyBatchCuda state upload");
  if (bodies > 0) {
    throwIfCudaRuntimeUnavailable();
  }

  m_impl->devicePosition.ensure(state.position.size());
  m_impl->deviceLinearVelocity.ensure(state.linearVelocity.size());
  m_impl->deviceOrientation.ensure(state.orientation.size());
  m_impl->deviceAngularVelocity.ensure(state.angularVelocity.size());
  m_impl->devicePosition.copyToDevice(
      state.position, "resident rigid-body position upload");
  m_impl->deviceLinearVelocity.copyToDevice(
      state.linearVelocity, "resident rigid-body linear velocity upload");
  m_impl->deviceOrientation.copyToDevice(
      state.orientation, "resident rigid-body orientation upload");
  m_impl->deviceAngularVelocity.copyToDevice(
      state.angularVelocity, "resident rigid-body angular velocity upload");
  m_impl->stateUploaded = true;
  m_impl->diagnostics.deviceStateValid = true;
  m_impl->diagnostics.hostStateCoherent = true;
  ++m_impl->diagnostics.stateUploadCount;
}

//==============================================================================
void ResidentRigidBodyBatchCuda::uploadControl(const std::vector<double>& force)
{
  DART_SIMULATION_THROW_T_IF(
      !m_impl->modelUploaded,
      sx::InvalidOperationException,
      "ResidentRigidBodyBatchCuda requires uploadModel() before "
      "uploadControl()");
  validateControlInputs(
      force,
      m_impl->model.worldCount,
      m_impl->model.bodyCount,
      "ResidentRigidBodyBatchCuda control upload");
  if (!force.empty()) {
    throwIfCudaRuntimeUnavailable();
  }

  m_impl->deviceForce.ensure(force.size());
  m_impl->deviceForce.copyToDevice(force, "resident rigid-body force upload");
  m_impl->controlUploaded = true;
  ++m_impl->diagnostics.controlUploadCount;
}

//==============================================================================
void ResidentRigidBodyBatchCuda::step(double timeStep)
{
  DART_SIMULATION_THROW_T_IF(
      !m_impl->modelUploaded || !m_impl->stateUploaded
          || !m_impl->controlUploaded,
      sx::InvalidOperationException,
      "ResidentRigidBodyBatchCuda requires uploaded Model, State, and Control "
      "before step()");
  const auto bodies = m_impl->bodies();
  if (bodies == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();
  m_impl->diagnostics.hostStateCoherent = false;
  throwIfCudaError(
      detail::launchRigidBodyStateBatchKernel(
          m_impl->devicePosition.data(),
          m_impl->deviceLinearVelocity.data(),
          m_impl->deviceOrientation.data(),
          m_impl->deviceAngularVelocity.data(),
          m_impl->deviceForce.data(),
          m_impl->deviceInverseMass.data(),
          timeStep,
          bodies),
      "resident rigid-body full kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "resident rigid-body full synchronize");
  ++m_impl->diagnostics.stepCount;
}

//==============================================================================
void ResidentRigidBodyBatchCuda::downloadState(RigidBodyStateBatch& state)
{
  DART_SIMULATION_THROW_T_IF(
      !m_impl->stateUploaded || !m_impl->diagnostics.deviceStateValid,
      sx::InvalidOperationException,
      "ResidentRigidBodyBatchCuda has no resident State to download");
  const auto bodies = m_impl->bodies();
  if (bodies > 0) {
    throwIfCudaRuntimeUnavailable();
  }

  state.worldCount = m_impl->model.worldCount;
  state.bodyCount = m_impl->model.bodyCount;
  state.position.resize(kLinearComponents * bodies);
  state.linearVelocity.resize(kLinearComponents * bodies);
  state.orientation.resize(kOrientationComponents * bodies);
  state.angularVelocity.resize(kLinearComponents * bodies);
  m_impl->devicePosition.copyFromDevice(
      state.position, "resident rigid-body position download");
  m_impl->deviceLinearVelocity.copyFromDevice(
      state.linearVelocity, "resident rigid-body linear velocity download");
  m_impl->deviceOrientation.copyFromDevice(
      state.orientation, "resident rigid-body orientation download");
  m_impl->deviceAngularVelocity.copyFromDevice(
      state.angularVelocity, "resident rigid-body angular velocity download");
  m_impl->diagnostics.hostStateCoherent = true;
  ++m_impl->diagnostics.stateDownloadCount;
}

//==============================================================================
void ResidentRigidBodyBatchCuda::requireCoherentStateForHostFallback(
    std::string_view operation) const
{
  DART_SIMULATION_THROW_T_IF(
      !canFallbackToHost(),
      sx::InvalidOperationException,
      "{} requires host-coherent rigid-batch State; call downloadState() at an "
      "explicit synchronization boundary before falling back to the host",
      operation);
}

//==============================================================================
bool ResidentRigidBodyBatchCuda::canFallbackToHost() const noexcept
{
  return m_impl->diagnostics.deviceStateValid
         && m_impl->diagnostics.hostStateCoherent;
}

//==============================================================================
ResidentRigidBodyBatchCudaDiagnostics ResidentRigidBodyBatchCuda::diagnostics()
    const noexcept
{
  return m_impl->diagnostics;
}

//==============================================================================
void integrateRigidBodyStateBatchLinearCuda(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep)
{
  const auto bodies = validateLinearInputs(
      state, model, force, "integrateRigidBodyStateBatchLinearCuda");

  if (bodies == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> devicePosition(state.position.size());
  DeviceBuffer<double> deviceLinearVelocity(state.linearVelocity.size());
  DeviceBuffer<double> deviceForce(force.size());
  DeviceBuffer<double> deviceInverseMass(model.inverseMass.size());

  devicePosition.copyToDevice(state.position, "position copy to device");
  deviceLinearVelocity.copyToDevice(
      state.linearVelocity, "linear velocity copy to device");
  deviceForce.copyToDevice(force, "force copy to device");
  deviceInverseMass.copyToDevice(
      model.inverseMass, "inverse mass copy to device");

  throwIfCudaError(
      detail::launchRigidBodyStateBatchLinearKernel(
          devicePosition.data(),
          deviceLinearVelocity.data(),
          deviceForce.data(),
          deviceInverseMass.data(),
          timeStep,
          bodies),
      "rigid-body linear kernel");
  throwIfCudaError(cudaDeviceSynchronize(), "rigid-body linear synchronize");

  devicePosition.copyFromDevice(state.position, "position copy from device");
  deviceLinearVelocity.copyFromDevice(
      state.linearVelocity, "linear velocity copy from device");
}

//==============================================================================
void rolloutRigidBodyStateBatchLinearCuda(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep,
    std::size_t stepCount)
{
  const auto bodies = validateLinearInputs(
      state, model, force, "rolloutRigidBodyStateBatchLinearCuda");

  if (bodies == 0 || stepCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> devicePosition(state.position.size());
  DeviceBuffer<double> deviceLinearVelocity(state.linearVelocity.size());
  DeviceBuffer<double> deviceForce(force.size());
  DeviceBuffer<double> deviceInverseMass(model.inverseMass.size());

  devicePosition.copyToDevice(state.position, "position copy to device");
  deviceLinearVelocity.copyToDevice(
      state.linearVelocity, "linear velocity copy to device");
  deviceForce.copyToDevice(force, "force copy to device");
  deviceInverseMass.copyToDevice(
      model.inverseMass, "inverse mass copy to device");

  for (std::size_t step = 0; step < stepCount; ++step) {
    throwIfCudaError(
        detail::launchRigidBodyStateBatchLinearKernel(
            devicePosition.data(),
            deviceLinearVelocity.data(),
            deviceForce.data(),
            deviceInverseMass.data(),
            timeStep,
            bodies),
        "rigid-body linear kernel");
  }
  throwIfCudaError(cudaDeviceSynchronize(), "rigid-body rollout synchronize");

  devicePosition.copyFromDevice(state.position, "position copy from device");
  deviceLinearVelocity.copyFromDevice(
      state.linearVelocity, "linear velocity copy from device");
}

//==============================================================================
void integrateRigidBodyStateBatchCuda(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep)
{
  rolloutRigidBodyStateBatchCuda(state, model, force, timeStep, 1);
}

//==============================================================================
void rolloutRigidBodyStateBatchCuda(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep,
    std::size_t stepCount)
{
  const auto bodies = validateFullInputs(
      state, model, force, "rolloutRigidBodyStateBatchCuda");

  if (bodies == 0 || stepCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> devicePosition(state.position.size());
  DeviceBuffer<double> deviceLinearVelocity(state.linearVelocity.size());
  DeviceBuffer<double> deviceOrientation(state.orientation.size());
  DeviceBuffer<double> deviceAngularVelocity(state.angularVelocity.size());
  DeviceBuffer<double> deviceForce(force.size());
  DeviceBuffer<double> deviceInverseMass(model.inverseMass.size());

  devicePosition.copyToDevice(state.position, "position copy to device");
  deviceLinearVelocity.copyToDevice(
      state.linearVelocity, "linear velocity copy to device");
  deviceOrientation.copyToDevice(
      state.orientation, "orientation copy to device");
  deviceAngularVelocity.copyToDevice(
      state.angularVelocity, "angular velocity copy to device");
  deviceForce.copyToDevice(force, "force copy to device");
  deviceInverseMass.copyToDevice(
      model.inverseMass, "inverse mass copy to device");

  for (std::size_t step = 0; step < stepCount; ++step) {
    throwIfCudaError(
        detail::launchRigidBodyStateBatchKernel(
            devicePosition.data(),
            deviceLinearVelocity.data(),
            deviceOrientation.data(),
            deviceAngularVelocity.data(),
            deviceForce.data(),
            deviceInverseMass.data(),
            timeStep,
            bodies),
        "rigid-body full kernel");
  }
  throwIfCudaError(cudaDeviceSynchronize(), "rigid-body full synchronize");

  devicePosition.copyFromDevice(state.position, "position copy from device");
  deviceLinearVelocity.copyFromDevice(
      state.linearVelocity, "linear velocity copy from device");
  deviceOrientation.copyFromDevice(
      state.orientation, "orientation copy from device");
}

} // namespace dart::simulation::compute::cuda
