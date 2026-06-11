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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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
#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>

#include <dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace cuda = dart::simulation::compute::cuda;
namespace nb = dart::simulation::detail::newton_barrier;

namespace {

std::vector<cuda::BarrierFrictionLocalInput> makeFixture()
{
  return {
      {
          .squaredDistance = 0.25,
          .squaredActivationDistance = 1.0,
          .stiffness = 2.0,
          .tangentialDisplacementNorm = 0.05,
          .frictionWeight = 4.0,
          .staticFrictionDisplacement = 0.2,
      },
      {
          .squaredDistance = 1.25,
          .squaredActivationDistance = 1.0,
          .stiffness = 2.0,
          .tangentialDisplacementNorm = 0.4,
          .frictionWeight = 4.0,
          .staticFrictionDisplacement = 0.2,
      },
      {
          .squaredDistance = 0.64,
          .squaredActivationDistance = 1.0,
          .stiffness = 3.0,
          .tangentialDisplacementNorm = 0.0,
          .frictionWeight = 5.0,
          .staticFrictionDisplacement = 0.25,
      },
  };
}

} // namespace

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuScalarContracts)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const auto inputs = makeFixture();

  cuda::BarrierFrictionLocalResult result;
  cuda::evaluateBarrierFrictionLocalKernelsCuda(inputs, result);

  ASSERT_EQ(result.barrierValues.size(), inputs.size());
  ASSERT_EQ(result.frictionWorks.size(), inputs.size());
  ASSERT_EQ(result.activeBarriers.size(), inputs.size());
  ASSERT_EQ(result.activeFrictions.size(), inputs.size());
  EXPECT_EQ(result.activeBarrierCount, 2u);
  EXPECT_EQ(result.activeFrictionCount, 3u);
  EXPECT_EQ(result.dynamicFrictionCount, 1u);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const double stiffness = std::max(0.0, input.stiffness);
    const auto barrier = nb::c2ClampedLogBarrier(
        input.squaredDistance, input.squaredActivationDistance);
    const auto friction = nb::smoothFrictionNorm(
        input.tangentialDisplacementNorm, input.staticFrictionDisplacement);
    const auto work = nb::frictionWorkContribution(
        input.tangentialDisplacementNorm,
        input.frictionWeight,
        input.staticFrictionDisplacement);

    EXPECT_EQ(result.activeBarriers[i] != 0u, barrier.active && stiffness > 0.0)
        << i;
    EXPECT_EQ(result.activeFrictions[i] != 0u, work.active) << i;
    EXPECT_EQ(result.dynamicFrictions[i] != 0u, friction.dynamicBranch) << i;

    EXPECT_NEAR(result.barrierValues[i], stiffness * barrier.value, 1e-12) << i;
    EXPECT_NEAR(
        result.barrierFirstDerivatives[i],
        stiffness * barrier.firstDerivative,
        1e-12)
        << i;
    EXPECT_NEAR(
        result.barrierSecondDerivatives[i],
        stiffness * barrier.secondDerivative,
        1e-11)
        << i;
    EXPECT_NEAR(
        result.frictionValues[i], input.frictionWeight * friction.value, 1e-12)
        << i;
    EXPECT_NEAR(result.frictionWorks[i], work.work, 1e-12) << i;
    EXPECT_NEAR(
        result.frictionFirstDerivatives[i], friction.firstDerivative, 1e-12)
        << i;
    EXPECT_NEAR(
        result.frictionSecondDerivatives[i], friction.secondDerivative, 1e-12)
        << i;
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, RejectsNonFiniteInput)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makeFixture();
  inputs.front().squaredDistance = std::numeric_limits<double>::quiet_NaN();

  cuda::BarrierFrictionLocalResult result;
  EXPECT_THROW(
      cuda::evaluateBarrierFrictionLocalKernelsCuda(inputs, result),
      dart::simulation::InvalidArgumentException);
}
