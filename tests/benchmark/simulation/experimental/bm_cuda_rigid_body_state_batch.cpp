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

struct Phase5RigidBodyBatchFixture
{
  compute::RigidBodyStateBatch state;
  compute::RigidBodyModelBatch model;
  std::vector<double> force;
};

Phase5RigidBodyBatchFixture makeFixture(
    std::size_t worldCount, std::size_t bodyCount)
{
  Phase5RigidBodyBatchFixture fixture;
  fixture.state.worldCount = worldCount;
  fixture.state.bodyCount = bodyCount;
  fixture.model.worldCount = fixture.state.worldCount;
  fixture.model.bodyCount = fixture.state.bodyCount;

  const auto totalBodies = fixture.state.worldCount * fixture.state.bodyCount;
  fixture.state.position.resize(3 * totalBodies);
  fixture.state.linearVelocity.resize(3 * totalBodies);
  fixture.state.orientation.resize(4 * totalBodies);
  fixture.state.angularVelocity.resize(3 * totalBodies);
  fixture.model.inverseMass.resize(totalBodies);
  fixture.force.resize(3 * totalBodies);

  for (std::size_t world = 0; world < worldCount; ++world) {
    for (std::size_t body = 0; body < bodyCount; ++body) {
      const auto index = world * bodyCount + body;
      const auto component = 3 * index;
      const auto quaternion = 4 * index;

      fixture.state.position[component + 0] = 0.001 * static_cast<double>(body);
      fixture.state.position[component + 1]
          = 0.002 * static_cast<double>(world);
      fixture.state.position[component + 2] = 0.0;

      fixture.state.orientation[quaternion + 0] = 1.0;
      fixture.state.orientation[quaternion + 1] = 0.0;
      fixture.state.orientation[quaternion + 2] = 0.0;
      fixture.state.orientation[quaternion + 3] = 0.0;

      fixture.state.linearVelocity[component + 0] = 0.5;
      fixture.state.linearVelocity[component + 1] = 0.25;
      fixture.state.linearVelocity[component + 2] = 0.125;

      fixture.state.angularVelocity[component + 0] = 0.01;
      fixture.state.angularVelocity[component + 1] = 0.02;
      fixture.state.angularVelocity[component + 2] = 0.03;

      fixture.model.inverseMass[index] = 1.0 / (1.0 + 0.001 * body);

      fixture.force[component + 0] = 0.05;
      fixture.force[component + 1] = 0.025;
      fixture.force[component + 2] = 0.0125;
    }
  }

  return fixture;
}

void BM_Phase5RigidBodyBatchCpuBaseline(benchmark::State& state)
{
  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodyCount = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  const auto fixture = makeFixture(worldCount, bodyCount);

  for (auto _ : state) {
    auto working = fixture.state;
    for (std::size_t step = 0; step < stepCount; ++step) {
      compute::integrateRigidBodyStateBatch(
          working, fixture.model, fixture.force, 0.001);
    }
    benchmark::DoNotOptimize(working.position.data());
    benchmark::DoNotOptimize(working.linearVelocity.data());
    benchmark::DoNotOptimize(working.orientation.data());
    benchmark::ClobberMemory();
  }

  state.counters["worlds"] = static_cast<double>(worldCount);
  state.counters["bodies_per_world"] = static_cast<double>(bodyCount);
  state.counters["steps"] = static_cast<double>(stepCount);
  state.SetItemsProcessed(
      state.iterations() * worldCount * bodyCount * stepCount);
}

void BM_Phase5RigidBodyBatchGpu(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodyCount = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  const auto fixture = makeFixture(worldCount, bodyCount);

  for (auto _ : state) {
    auto working = fixture.state;
    cuda::rolloutRigidBodyStateBatchCuda(
        working, fixture.model, fixture.force, 0.001, stepCount);
    benchmark::DoNotOptimize(working.position.data());
    benchmark::DoNotOptimize(working.linearVelocity.data());
    benchmark::DoNotOptimize(working.orientation.data());
    benchmark::ClobberMemory();
  }

  state.counters["worlds"] = static_cast<double>(worldCount);
  state.counters["bodies_per_world"] = static_cast<double>(bodyCount);
  state.counters["steps"] = static_cast<double>(stepCount);
  state.SetItemsProcessed(
      state.iterations() * worldCount * bodyCount * stepCount);
}

} // namespace

BENCHMARK(BM_Phase5RigidBodyBatchCpuBaseline)
    ->Args({1024, 128, 10})
    ->Args({4096, 128, 100});
BENCHMARK(BM_Phase5RigidBodyBatchGpu)
    ->Args({1024, 128, 10})
    ->Args({4096, 128, 100});
