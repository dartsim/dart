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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
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

#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/compute/compute_graph.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/rigid_body_state_batch_cuda.cuh>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

namespace sx = dart::simulation;
namespace compute = dart::simulation::compute;
namespace cuda = dart::simulation::compute::cuda;

namespace {

constexpr double kTimeStep = 0.001;
constexpr std::size_t kCpuChunkSize = 8192;

struct LinearBatchFixture
{
  compute::RigidBodyStateBatch state;
  compute::RigidBodyModelBatch model;
  std::vector<double> force;
};

struct WorldBatchFixture
{
  std::vector<std::unique_ptr<sx::World>> storage;
  std::vector<sx::World*> worlds;
  std::vector<const sx::World*> constWorlds;
  compute::RigidBodyStateBatch state;
  compute::RigidBodyModelBatch model;
  std::vector<double> force;
};

struct Phase5RigidBodyBatchFixture
{
  compute::RigidBodyStateBatch state;
  compute::RigidBodyModelBatch model;
  std::vector<double> force;
};

Eigen::Vector3d makePosition(std::size_t body)
{
  return {
      static_cast<double>(body % 17),
      static_cast<double>(body % 29),
      static_cast<double>(body % 31)};
}

Eigen::Vector3d makeLinearVelocity(std::size_t body)
{
  return {0.25 + 0.001 * static_cast<double>(body % 11), -0.5, 1.0};
}

Eigen::Vector3d makeForce(std::size_t body)
{
  return {
      1.0 + 0.01 * static_cast<double>(body % 13),
      -2.0,
      3.0 + 0.02 * static_cast<double>(body % 7)};
}

double makeMass(std::size_t body)
{
  return body % 7 == 0 ? 0.0 : 1.0 + 0.001 * static_cast<double>(body % 101);
}

void appendVector3(std::vector<double>& values, const Eigen::Vector3d& vector)
{
  values.push_back(vector.x());
  values.push_back(vector.y());
  values.push_back(vector.z());
}

void appendModel(
    compute::RigidBodyModelBatch& target,
    const compute::RigidBodyModelBatch& source)
{
  target.inverseMass.insert(
      target.inverseMass.end(),
      source.inverseMass.begin(),
      source.inverseMass.end());
  target.inertia.insert(
      target.inertia.end(), source.inertia.begin(), source.inertia.end());
}

LinearBatchFixture makeFixture(
    std::size_t worldCount, std::size_t bodiesPerWorld)
{
  LinearBatchFixture fixture;
  fixture.state.worldCount = worldCount;
  fixture.state.bodyCount = bodiesPerWorld;
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
    const auto position = makePosition(body);
    const auto velocity = makeLinearVelocity(body);
    const auto force = makeForce(body);

    fixture.state.position[base] = position.x();
    fixture.state.position[base + 1] = position.y();
    fixture.state.position[base + 2] = position.z();
    fixture.state.linearVelocity[base] = velocity.x();
    fixture.state.linearVelocity[base + 1] = velocity.y();
    fixture.state.linearVelocity[base + 2] = velocity.z();
    fixture.force[base] = force.x();
    fixture.force[base + 1] = force.y();
    fixture.force[base + 2] = force.z();

    const auto mass = makeMass(body);
    fixture.model.inverseMass[body] = mass > 0.0 ? 1.0 / mass : 0.0;

    const auto orientationBase = 4 * body;
    fixture.state.orientation[orientationBase] = 1.0;
  }

  return fixture;
}

Phase5RigidBodyBatchFixture makePhase5Fixture(
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

WorldBatchFixture makeWorldBatchFixture(
    std::size_t worldCount, std::size_t bodiesPerWorld)
{
  WorldBatchFixture fixture;
  fixture.storage.reserve(worldCount);
  fixture.worlds.reserve(worldCount);
  fixture.constWorlds.reserve(worldCount);
  fixture.force.reserve(3 * worldCount * bodiesPerWorld);
  fixture.model.worldCount = worldCount;
  fixture.model.bodyCount = bodiesPerWorld;

  for (std::size_t worldIndex = 0; worldIndex < worldCount; ++worldIndex) {
    auto world = std::make_unique<sx::World>();

    for (std::size_t bodyIndex = 0; bodyIndex < bodiesPerWorld; ++bodyIndex) {
      const auto globalBody = worldIndex * bodiesPerWorld + bodyIndex;
      sx::RigidBodyOptions options;
      options.mass = std::max(1.0, makeMass(globalBody));
      options.position = makePosition(globalBody);
      options.linearVelocity = makeLinearVelocity(globalBody);

      auto body
          = world->addRigidBody("body_" + std::to_string(bodyIndex), options);
      const auto force = makeForce(globalBody);
      body.setForce(force);
      appendVector3(fixture.force, force);
    }

    world->setTimeStep(kTimeStep);
    world->enterSimulationMode();
    appendModel(fixture.model, compute::extractRigidBodyModelBatch(*world));

    fixture.worlds.push_back(world.get());
    fixture.constWorlds.push_back(world.get());
    fixture.storage.push_back(std::move(world));
  }

  fixture.state = compute::extractRigidBodyStateBatch(fixture.constWorlds);
  return fixture;
}

void integrateLinearRange(
    compute::RigidBodyStateBatch& state,
    const compute::RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep,
    std::size_t begin,
    std::size_t end)
{
  for (std::size_t body = begin; body < end; ++body) {
    const auto base = 3 * body;
    const double velocityScale = model.inverseMass[body] * timeStep;

    state.linearVelocity[base] += force[base] * velocityScale;
    state.linearVelocity[base + 1] += force[base + 1] * velocityScale;
    state.linearVelocity[base + 2] += force[base + 2] * velocityScale;

    state.position[base] += state.linearVelocity[base] * timeStep;
    state.position[base + 1] += state.linearVelocity[base + 1] * timeStep;
    state.position[base + 2] += state.linearVelocity[base + 2] * timeStep;
  }
}

class ParallelLinearIntegrator
{
public:
  ParallelLinearIntegrator(
      compute::RigidBodyStateBatch& state,
      const compute::RigidBodyModelBatch& model,
      const std::vector<double>& force,
      double timeStep,
      std::size_t chunkSize)
    : m_state(state), m_model(model), m_force(force), m_timeStep(timeStep)
  {
    m_executor.setInlineThreshold(0);

    const auto bodies = m_state.worldCount * m_state.bodyCount;
    const auto chunk = std::max<std::size_t>(1, chunkSize);
    for (std::size_t begin = 0; begin < bodies; begin += chunk) {
      const auto end = std::min(begin + chunk, bodies);
      m_graph.addNode(
          "linear_chunk_" + std::to_string(begin), [this, begin, end]() {
            integrateLinearRange(
                m_state, m_model, m_force, m_timeStep, begin, end);
          });
    }
  }

  void step()
  {
    m_executor.execute(m_graph);
  }

  [[nodiscard]] std::size_t getWorkerCount() const
  {
    return m_executor.getWorkerCount();
  }

private:
  compute::RigidBodyStateBatch& m_state;
  const compute::RigidBodyModelBatch& m_model;
  const std::vector<double>& m_force;
  double m_timeStep = 0.0;
  compute::ComputeGraph m_graph;
  compute::ParallelExecutor m_executor;
};

void setRolloutCounters(
    benchmark::State& state,
    std::size_t worldCount,
    std::size_t bodiesPerWorld,
    std::size_t stepCount)
{
  state.counters["worlds"] = static_cast<double>(worldCount);
  state.counters["bodies_per_world"] = static_cast<double>(bodiesPerWorld);
  state.counters["total_bodies"]
      = static_cast<double>(worldCount * bodiesPerWorld);
  state.counters["steps"] = static_cast<double>(stepCount);
  state.SetItemsProcessed(
      state.iterations() * worldCount * bodiesPerWorld * stepCount);
}

void runCpuSingleThreadRollout(
    compute::RigidBodyStateBatch& state,
    const compute::RigidBodyModelBatch& model,
    const std::vector<double>& force,
    std::size_t stepCount)
{
  for (std::size_t step = 0; step < stepCount; ++step) {
    compute::integrateRigidBodyStateBatchLinear(state, model, force, kTimeStep);
  }
}

void runCpuMulticoreRollout(
    compute::RigidBodyStateBatch& state,
    const compute::RigidBodyModelBatch& model,
    const std::vector<double>& force,
    std::size_t stepCount,
    benchmark::State& benchmarkState)
{
  ParallelLinearIntegrator integrator(
      state, model, force, kTimeStep, kCpuChunkSize);
  benchmarkState.counters["cpu_workers"]
      = static_cast<double>(integrator.getWorkerCount());
  benchmarkState.counters["cpu_chunk_size"]
      = static_cast<double>(kCpuChunkSize);

  for (std::size_t step = 0; step < stepCount; ++step) {
    integrator.step();
  }
}

bool requireCudaRuntime(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return false;
  }
  return true;
}

void integrateFullCpuSteps(
    compute::RigidBodyStateBatch& state,
    const compute::RigidBodyModelBatch& model,
    const std::vector<double>& force,
    std::size_t stepCount)
{
  for (std::size_t step = 0; step < stepCount; ++step) {
    compute::integrateRigidBodyStateBatch(state, model, force, kTimeStep);
  }
}

double maxAbsDifference(
    const std::vector<double>& actual, const std::vector<double>& expected)
{
  if (actual.size() != expected.size()) {
    return std::numeric_limits<double>::infinity();
  }

  double maxError = 0.0;
  for (std::size_t i = 0; i < actual.size(); ++i) {
    maxError = std::max(maxError, std::abs(actual[i] - expected[i]));
  }
  return maxError;
}

double maxFinalStateAbsError(
    const compute::RigidBodyStateBatch& actual,
    const compute::RigidBodyStateBatch& expected)
{
  return std::max(
      {maxAbsDifference(actual.position, expected.position),
       maxAbsDifference(actual.linearVelocity, expected.linearVelocity),
       maxAbsDifference(actual.orientation, expected.orientation),
       maxAbsDifference(actual.angularVelocity, expected.angularVelocity)});
}

//==============================================================================
void BM_CpuSingleThreadRigidBodyStateBatchLinearRollout(benchmark::State& state)
{
  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodiesPerWorld = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  const auto fixture = makeFixture(worldCount, bodiesPerWorld);
  auto working = fixture.state;
  auto warmup = fixture.state;
  cuda::integrateRigidBodyStateBatchLinearCuda(
      warmup, fixture.model, fixture.force, kTimeStep);

  for (auto _ : state) {
    working = fixture.state;
    runCpuSingleThreadRollout(working, fixture.model, fixture.force, stepCount);
    benchmark::DoNotOptimize(working.position.data());
  }

  setRolloutCounters(state, worldCount, bodiesPerWorld, stepCount);
}

//==============================================================================
void BM_CpuMulticoreRigidBodyStateBatchLinearRollout(benchmark::State& state)
{
  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodiesPerWorld = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  const auto fixture = makeFixture(worldCount, bodiesPerWorld);
  auto working = fixture.state;
  auto warmup = fixture.state;
  cuda::rolloutRigidBodyStateBatchLinearCuda(
      warmup, fixture.model, fixture.force, kTimeStep, 1);

  for (auto _ : state) {
    working = fixture.state;
    runCpuMulticoreRollout(
        working, fixture.model, fixture.force, stepCount, state);
    benchmark::DoNotOptimize(working.position.data());
  }

  setRolloutCounters(state, worldCount, bodiesPerWorld, stepCount);
}

//==============================================================================
void BM_CudaRoundTripRigidBodyStateBatchLinearRollout(benchmark::State& state)
{
  if (!requireCudaRuntime(state)) {
    return;
  }

  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodiesPerWorld = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  const auto fixture = makeFixture(worldCount, bodiesPerWorld);
  auto working = fixture.state;

  for (auto _ : state) {
    working = fixture.state;
    for (std::size_t step = 0; step < stepCount; ++step) {
      cuda::integrateRigidBodyStateBatchLinearCuda(
          working, fixture.model, fixture.force, kTimeStep);
    }
    benchmark::DoNotOptimize(working.position.data());
  }

  setRolloutCounters(state, worldCount, bodiesPerWorld, stepCount);
}

//==============================================================================
void BM_CudaResidentRigidBodyStateBatchLinearRollout(benchmark::State& state)
{
  if (!requireCudaRuntime(state)) {
    return;
  }

  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodiesPerWorld = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  const auto fixture = makeFixture(worldCount, bodiesPerWorld);
  auto working = fixture.state;

  for (auto _ : state) {
    working = fixture.state;
    cuda::rolloutRigidBodyStateBatchLinearCuda(
        working, fixture.model, fixture.force, kTimeStep, stepCount);
    benchmark::DoNotOptimize(working.position.data());
  }

  setRolloutCounters(state, worldCount, bodiesPerWorld, stepCount);
}

//==============================================================================
void BM_WorldExtractedCpuSingleThreadLinearRollout(benchmark::State& state)
{
  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodiesPerWorld = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  auto fixture = makeWorldBatchFixture(worldCount, bodiesPerWorld);
  auto warmup = fixture.state;
  cuda::rolloutRigidBodyStateBatchLinearCuda(
      warmup, fixture.model, fixture.force, kTimeStep, 1);

  for (auto _ : state) {
    compute::applyRigidBodyStateBatch(fixture.worlds, fixture.state);
    auto working = compute::extractRigidBodyStateBatch(fixture.constWorlds);
    runCpuSingleThreadRollout(working, fixture.model, fixture.force, stepCount);
    compute::applyRigidBodyStateBatch(fixture.worlds, working);
    benchmark::DoNotOptimize(working.position.data());
  }

  setRolloutCounters(state, worldCount, bodiesPerWorld, stepCount);
}

//==============================================================================
void BM_WorldExtractedCpuMulticoreLinearRollout(benchmark::State& state)
{
  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodiesPerWorld = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  auto fixture = makeWorldBatchFixture(worldCount, bodiesPerWorld);

  for (auto _ : state) {
    compute::applyRigidBodyStateBatch(fixture.worlds, fixture.state);
    auto working = compute::extractRigidBodyStateBatch(fixture.constWorlds);
    runCpuMulticoreRollout(
        working, fixture.model, fixture.force, stepCount, state);
    compute::applyRigidBodyStateBatch(fixture.worlds, working);
    benchmark::DoNotOptimize(working.position.data());
  }

  setRolloutCounters(state, worldCount, bodiesPerWorld, stepCount);
}

//==============================================================================
void BM_WorldExtractedCudaResidentLinearRollout(benchmark::State& state)
{
  if (!requireCudaRuntime(state)) {
    return;
  }

  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodiesPerWorld = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  auto fixture = makeWorldBatchFixture(worldCount, bodiesPerWorld);

  for (auto _ : state) {
    compute::applyRigidBodyStateBatch(fixture.worlds, fixture.state);
    auto working = compute::extractRigidBodyStateBatch(fixture.constWorlds);
    cuda::rolloutRigidBodyStateBatchLinearCuda(
        working, fixture.model, fixture.force, kTimeStep, stepCount);
    compute::applyRigidBodyStateBatch(fixture.worlds, working);
    benchmark::DoNotOptimize(working.position.data());
  }

  setRolloutCounters(state, worldCount, bodiesPerWorld, stepCount);
}

//==============================================================================
void BM_Phase5RigidBodyBatchCpuBaseline(benchmark::State& state)
{
  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodyCount = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  const auto fixture = makePhase5Fixture(worldCount, bodyCount);

  for (auto _ : state) {
    auto working = fixture.state;
    integrateFullCpuSteps(working, fixture.model, fixture.force, stepCount);
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

//==============================================================================
void BM_Phase5RigidBodyBatchGpu(benchmark::State& state)
{
  if (!requireCudaRuntime(state)) {
    return;
  }

  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto bodyCount = static_cast<std::size_t>(state.range(1));
  const auto stepCount = static_cast<std::size_t>(state.range(2));
  const auto fixture = makePhase5Fixture(worldCount, bodyCount);
  auto expected = fixture.state;
  integrateFullCpuSteps(expected, fixture.model, fixture.force, stepCount);

  auto working = fixture.state;
  for (auto _ : state) {
    working = fixture.state;
    cuda::rolloutRigidBodyStateBatchCuda(
        working, fixture.model, fixture.force, kTimeStep, stepCount);
    benchmark::DoNotOptimize(working.position.data());
    benchmark::DoNotOptimize(working.linearVelocity.data());
    benchmark::DoNotOptimize(working.orientation.data());
    benchmark::ClobberMemory();
  }

  state.counters["worlds"] = static_cast<double>(worldCount);
  state.counters["bodies_per_world"] = static_cast<double>(bodyCount);
  state.counters["steps"] = static_cast<double>(stepCount);
  state.counters["max_final_state_abs_error"]
      = maxFinalStateAbsError(working, expected);
  state.SetItemsProcessed(
      state.iterations() * worldCount * bodyCount * stepCount);
}

void addSoAArgs(benchmark::Benchmark* benchmark)
{
  benchmark->Args({4, 256, 1});
  benchmark->Args({64, 256, 64});
  benchmark->Args({256, 256, 128});
  benchmark->Args({1024, 256, 256});
}

void addRoundTripArgs(benchmark::Benchmark* benchmark)
{
  benchmark->Args({4, 256, 1});
  benchmark->Args({64, 256, 8});
}

void addWorldExtractedArgs(benchmark::Benchmark* benchmark)
{
  benchmark->Args({16, 64, 64});
  benchmark->Args({64, 128, 128});
  benchmark->Args({256, 128, 256});
}

} // namespace

BENCHMARK(BM_CpuSingleThreadRigidBodyStateBatchLinearRollout)
    ->Apply(addSoAArgs);
BENCHMARK(BM_CpuMulticoreRigidBodyStateBatchLinearRollout)->Apply(addSoAArgs);
BENCHMARK(BM_CudaResidentRigidBodyStateBatchLinearRollout)->Apply(addSoAArgs);
BENCHMARK(BM_CudaRoundTripRigidBodyStateBatchLinearRollout)
    ->Apply(addRoundTripArgs);
BENCHMARK(BM_WorldExtractedCpuSingleThreadLinearRollout)
    ->Apply(addWorldExtractedArgs);
BENCHMARK(BM_WorldExtractedCpuMulticoreLinearRollout)
    ->Apply(addWorldExtractedArgs);
BENCHMARK(BM_WorldExtractedCudaResidentLinearRollout)
    ->Apply(addWorldExtractedArgs);
BENCHMARK(BM_Phase5RigidBodyBatchCpuBaseline)
    ->Args({1024, 128, 10})
    ->Args({4096, 128, 100});
BENCHMARK(BM_Phase5RigidBodyBatchGpu)
    ->Args({1024, 128, 10})
    ->Args({4096, 128, 100});

BENCHMARK_MAIN();
