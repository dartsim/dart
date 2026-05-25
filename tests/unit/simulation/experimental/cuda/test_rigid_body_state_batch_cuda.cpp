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
#include <dart/simulation/experimental/compute/rigid_body_state_batch.hpp>

#include <dart/simulation/experimental/compute/cuda/rigid_body_state_batch_cuda.cuh>
#include <gtest/gtest.h>

#include <vector>

#include <cstddef>

namespace compute = dart::simulation::experimental::compute;
namespace cuda = dart::simulation::experimental::compute::cuda;
namespace sx = dart::simulation::experimental;

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
  state.angularVelocity.assign(18, 0.0);
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
