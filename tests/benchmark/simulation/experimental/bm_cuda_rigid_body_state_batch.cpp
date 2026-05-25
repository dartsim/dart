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

#include <dart/simulation/experimental/compute/rigid_body_state_batch.hpp>

#include <benchmark/benchmark.h>
#include <dart/simulation/experimental/compute/cuda/rigid_body_state_batch_cuda.cuh>

#include <vector>

#include <cstddef>

namespace compute = dart::simulation::experimental::compute;
namespace cuda = dart::simulation::experimental::compute::cuda;

namespace {

struct LinearBatchFixture
{
  compute::RigidBodyStateBatch state;
  compute::RigidBodyModelBatch model;
  std::vector<double> force;
};

LinearBatchFixture makeFixture(std::size_t bodies)
{
  LinearBatchFixture fixture;
  fixture.state.worldCount = 4;
  fixture.state.bodyCount = bodies;
  fixture.model.worldCount = fixture.state.worldCount;
  fixture.model.bodyCount = fixture.state.bodyCount;

  const auto totalBodies = fixture.state.worldCount * fixture.state.bodyCount;
  fixture.state.position.resize(3 * totalBodies);
  fixture.state.linearVelocity.resize(3 * totalBodies);
  fixture.state.orientation.resize(4 * totalBodies);
  fixture.state.angularVelocity.resize(3 * totalBodies);
  fixture.model.inverseMass.resize(totalBodies);
  fixture.force.resize(3 * totalBodies);

  for (std::size_t body = 0; body < totalBodies; ++body) {
    const auto base = 3 * body;
    fixture.state.position[base] = static_cast<double>(body % 17);
    fixture.state.position[base + 1] = static_cast<double>(body % 29);
    fixture.state.position[base + 2] = static_cast<double>(body % 31);
    fixture.state.linearVelocity[base] = 0.25;
    fixture.state.linearVelocity[base + 1] = -0.5;
    fixture.state.linearVelocity[base + 2] = 1.0;
    fixture.force[base] = 1.0;
    fixture.force[base + 1] = -2.0;
    fixture.force[base + 2] = 3.0;
    fixture.model.inverseMass[body] = body % 7 == 0 ? 0.0 : 1.0;

    const auto orientationBase = 4 * body;
    fixture.state.orientation[orientationBase] = 1.0;
  }

  return fixture;
}

void BM_CpuRigidBodyStateBatchLinear(benchmark::State& state)
{
  const auto fixture = makeFixture(static_cast<std::size_t>(state.range(0)));

  for (auto _ : state) {
    auto working = fixture.state;
    compute::integrateRigidBodyStateBatchLinear(
        working, fixture.model, fixture.force, 0.001);
    benchmark::DoNotOptimize(working.position.data());
  }
}

void BM_CudaRigidBodyStateBatchLinear(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeFixture(static_cast<std::size_t>(state.range(0)));

  for (auto _ : state) {
    auto working = fixture.state;
    cuda::integrateRigidBodyStateBatchLinearCuda(
        working, fixture.model, fixture.force, 0.001);
    benchmark::DoNotOptimize(working.position.data());
  }
}

} // namespace

BENCHMARK(BM_CpuRigidBodyStateBatchLinear)->Arg(256)->Arg(4096);
BENCHMARK(BM_CudaRigidBodyStateBatchLinear)->Arg(256)->Arg(4096);
