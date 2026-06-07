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

} // namespace

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
