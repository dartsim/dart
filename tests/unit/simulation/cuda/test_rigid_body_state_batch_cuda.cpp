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
#include <dart/simulation/compute/rigid_body_state_batch.hpp>

#include <dart/simulation/compute/cuda/rigid_body_state_batch_cuda.cuh>
#include <gtest/gtest.h>

#include <vector>

#include <cstddef>

namespace compute = dart::simulation::compute;
namespace cuda = dart::simulation::compute::cuda;
namespace sx = dart::simulation;

namespace {

compute::RigidBodyStateBatch makeStateBatch()
{
  compute::RigidBodyStateBatch state;
  state.worldCount = 2;
  state.bodyCount = 3;
  state.position = {
      0.0,
      1.0,
      2.0,
      10.0,
      11.0,
      12.0,
      20.0,
      21.0,
      22.0,
      100.0,
      101.0,
      102.0,
      110.0,
      111.0,
      112.0,
      120.0,
      121.0,
      122.0,
  };
  state.orientation = {
      1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
  };
  state.linearVelocity = {
      1.0,
      0.5,
      -0.25,
      0.0,
      2.0,
      0.5,
      -1.0,
      0.0,
      1.0,
      3.0,
      -2.0,
      1.0,
      0.5,
      0.25,
      -0.75,
      -2.0,
      4.0,
      0.0,
  };
  state.angularVelocity = {
      0.01,
      0.02,
      0.03,
      -0.02,
      0.01,
      0.04,
      0.0,
      0.0,
      0.0,
      0.03,
      -0.01,
      0.02,
      0.04,
      0.02,
      -0.01,
      -0.03,
      0.01,
      0.02,
  };
  return state;
}

compute::RigidBodyModelBatch makeModelBatch()
{
  compute::RigidBodyModelBatch model;
  model.worldCount = 2;
  model.bodyCount = 3;
  model.inverseMass = {1.0, 0.5, 0.0, 2.0, 0.0, 1.5};
  return model;
}

std::vector<double> makeForceBatch()
{
  return {
      0.0,
      0.0,
      10.0,
      1.0,
      -2.0,
      3.0,
      8.0,
      9.0,
      10.0,
      -1.0,
      0.5,
      2.0,
      9.0,
      9.0,
      9.0,
      3.0,
      -6.0,
      12.0,
  };
}

} // namespace

//==============================================================================
TEST(CudaRigidBodyStateBatch, RejectsInvalidSizesBeforeCudaRuntime)
{
  auto state = makeStateBatch();
  const auto model = makeModelBatch();
  const auto force = makeForceBatch();

  state.position.pop_back();

  EXPECT_THROW(
      cuda::integrateRigidBodyStateBatchLinearCuda(state, model, force, 0.01),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, RejectsMismatchedModelBeforeCudaRuntime)
{
  auto state = makeStateBatch();
  auto model = makeModelBatch();
  const auto force = makeForceBatch();

  model.worldCount = 1;

  EXPECT_THROW(
      cuda::integrateRigidBodyStateBatchLinearCuda(state, model, force, 0.01),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, RejectsInvalidFullStateBeforeCudaRuntime)
{
  auto state = makeStateBatch();
  const auto model = makeModelBatch();
  const auto force = makeForceBatch();

  state.orientation.pop_back();

  EXPECT_THROW(
      cuda::integrateRigidBodyStateBatchCuda(state, model, force, 0.01),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, MatchesCpuLinearIntegration)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto cudaState = makeStateBatch();
  auto cpuState = cudaState;
  const auto model = makeModelBatch();
  const auto force = makeForceBatch();
  constexpr double dt = 0.125;

  compute::integrateRigidBodyStateBatchLinear(cpuState, model, force, dt);
  cuda::integrateRigidBodyStateBatchLinearCuda(cudaState, model, force, dt);

  ASSERT_EQ(cudaState.position.size(), cpuState.position.size());
  ASSERT_EQ(cudaState.linearVelocity.size(), cpuState.linearVelocity.size());

  for (std::size_t i = 0; i < cpuState.position.size(); ++i) {
    EXPECT_NEAR(cudaState.position[i], cpuState.position[i], 1e-12) << i;
    EXPECT_NEAR(cudaState.linearVelocity[i], cpuState.linearVelocity[i], 1e-12)
        << i;
  }
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, RolloutMatchesCpuLinearIntegration)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto cudaState = makeStateBatch();
  auto cpuState = cudaState;
  const auto model = makeModelBatch();
  const auto force = makeForceBatch();
  constexpr double dt = 0.01;
  constexpr std::size_t steps = 32;

  for (std::size_t step = 0; step < steps; ++step) {
    compute::integrateRigidBodyStateBatchLinear(cpuState, model, force, dt);
  }
  cuda::rolloutRigidBodyStateBatchLinearCuda(
      cudaState, model, force, dt, steps);

  ASSERT_EQ(cudaState.position.size(), cpuState.position.size());
  ASSERT_EQ(cudaState.linearVelocity.size(), cpuState.linearVelocity.size());

  for (std::size_t i = 0; i < cpuState.position.size(); ++i) {
    EXPECT_NEAR(cudaState.position[i], cpuState.position[i], 1e-12) << i;
    EXPECT_NEAR(cudaState.linearVelocity[i], cpuState.linearVelocity[i], 1e-12)
        << i;
  }
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, RolloutRejectsInvalidSizesBeforeCudaRuntime)
{
  auto state = makeStateBatch();
  const auto model = makeModelBatch();
  auto force = makeForceBatch();

  force.pop_back();

  EXPECT_THROW(
      cuda::rolloutRigidBodyStateBatchLinearCuda(state, model, force, 0.01, 4),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, ResidentOwnerRejectsInvalidModelBeforeCudaRuntime)
{
  cuda::ResidentRigidBodyBatchCuda resident;
  auto model = makeModelBatch();
  model.inverseMass.pop_back();

  EXPECT_THROW(resident.uploadModel(model), sx::InvalidArgumentException);
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, ResidentOwnerRequiresModelBeforeStateOrControl)
{
  cuda::ResidentRigidBodyBatchCuda resident;

  EXPECT_THROW(
      resident.uploadState(makeStateBatch()), sx::InvalidOperationException);
  EXPECT_THROW(
      resident.uploadControl(makeForceBatch()), sx::InvalidOperationException);
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, MatchesCpuFullIntegration)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto cudaState = makeStateBatch();
  auto cpuState = cudaState;
  const auto model = makeModelBatch();
  const auto force = makeForceBatch();
  constexpr double dt = 0.125;
  constexpr std::size_t steps = 4;

  for (std::size_t step = 0; step < steps; ++step) {
    compute::integrateRigidBodyStateBatch(cpuState, model, force, dt);
  }
  cuda::rolloutRigidBodyStateBatchCuda(cudaState, model, force, dt, steps);

  ASSERT_EQ(cudaState.position.size(), cpuState.position.size());
  ASSERT_EQ(cudaState.linearVelocity.size(), cpuState.linearVelocity.size());
  ASSERT_EQ(cudaState.orientation.size(), cpuState.orientation.size());

  for (std::size_t i = 0; i < cpuState.position.size(); ++i) {
    EXPECT_NEAR(cudaState.position[i], cpuState.position[i], 1e-12) << i;
    EXPECT_NEAR(cudaState.linearVelocity[i], cpuState.linearVelocity[i], 1e-12)
        << i;
  }
  for (std::size_t i = 0; i < cpuState.orientation.size(); ++i) {
    EXPECT_NEAR(cudaState.orientation[i], cpuState.orientation[i], 1e-12) << i;
  }
}

//==============================================================================
TEST(CudaRigidBodyStateBatch, ResidentOwnerDownloadsOnlyAtExplicitSync)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  cuda::ResidentRigidBodyBatchCuda resident;
  auto cudaState = makeStateBatch();
  auto cpuState = cudaState;
  const auto model = makeModelBatch();
  const auto force = makeForceBatch();
  constexpr double dt = 0.125;
  constexpr std::size_t steps = 4;

  resident.uploadModel(model);
  resident.uploadState(cudaState);
  resident.uploadControl(force);

  auto diagnostics = resident.diagnostics();
  EXPECT_EQ(diagnostics.modelUploadCount, 1u);
  EXPECT_EQ(diagnostics.stateUploadCount, 1u);
  EXPECT_EQ(diagnostics.controlUploadCount, 1u);
  EXPECT_EQ(diagnostics.stateDownloadCount, 0u);
  EXPECT_TRUE(diagnostics.deviceStateValid);
  EXPECT_TRUE(diagnostics.hostStateCoherent);
  EXPECT_TRUE(resident.canFallbackToHost());
  EXPECT_NO_THROW(
      resident.requireCoherentStateForHostFallback("initial fallback"));

  for (std::size_t step = 0; step < steps; ++step) {
    compute::integrateRigidBodyStateBatch(cpuState, model, force, dt);
    resident.step(dt);
  }

  diagnostics = resident.diagnostics();
  EXPECT_EQ(diagnostics.modelUploadCount, 1u);
  EXPECT_EQ(diagnostics.stateUploadCount, 1u);
  EXPECT_EQ(diagnostics.controlUploadCount, 1u);
  EXPECT_EQ(diagnostics.stateDownloadCount, 0u);
  EXPECT_EQ(diagnostics.stepCount, steps);
  EXPECT_TRUE(diagnostics.deviceStateValid);
  EXPECT_FALSE(diagnostics.hostStateCoherent);
  EXPECT_FALSE(resident.canFallbackToHost());
  EXPECT_THROW(
      resident.requireCoherentStateForHostFallback("mid-step fallback"),
      sx::InvalidOperationException);

  resident.downloadState(cudaState);
  diagnostics = resident.diagnostics();
  EXPECT_EQ(diagnostics.stateDownloadCount, 1u);
  EXPECT_TRUE(diagnostics.hostStateCoherent);
  EXPECT_TRUE(resident.canFallbackToHost());
  EXPECT_NO_THROW(
      resident.requireCoherentStateForHostFallback("post-sync fallback"));

  ASSERT_EQ(cudaState.position.size(), cpuState.position.size());
  ASSERT_EQ(cudaState.linearVelocity.size(), cpuState.linearVelocity.size());
  ASSERT_EQ(cudaState.orientation.size(), cpuState.orientation.size());

  for (std::size_t i = 0; i < cpuState.position.size(); ++i) {
    EXPECT_NEAR(cudaState.position[i], cpuState.position[i], 1e-12) << i;
    EXPECT_NEAR(cudaState.linearVelocity[i], cpuState.linearVelocity[i], 1e-12)
        << i;
  }
  for (std::size_t i = 0; i < cpuState.orientation.size(); ++i) {
    EXPECT_NEAR(cudaState.orientation[i], cpuState.orientation[i], 1e-12) << i;
  }
}
