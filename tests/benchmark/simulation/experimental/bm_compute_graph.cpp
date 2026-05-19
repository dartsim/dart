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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/compute/compute_graph.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/taskflow_executor.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <string>
#include <vector>

namespace sx = dart::simulation::experimental;
namespace compute = dart::simulation::experimental::compute;

namespace {

//==============================================================================
struct BatchedGraph
{
  BatchedGraph(int itemCount, int batchSize) : values(itemCount, 1.0)
  {
    auto& start = graph.addNode("start", []() {});
    std::vector<compute::ComputeNode*> batches;
    batches.reserve(
        static_cast<std::size_t>((itemCount + batchSize - 1) / batchSize));

    for (int begin = 0; begin < itemCount; begin += batchSize) {
      const auto end = std::min(begin + batchSize, itemCount);
      auto& batch = graph.addNode(
          "batch_" + std::to_string(begin), [this, begin, end]() {
            for (int i = begin; i < end; ++i) {
              values[static_cast<std::size_t>(i)]
                  = values[static_cast<std::size_t>(i)] * 1.000001 + 1.0;
            }
          });

      graph.addDependency(start, batch);
      batches.push_back(&batch);
    }

    auto& reduce = graph.addNode("reduce", [this]() {
      double sum = 0.0;
      for (const auto value : values) {
        sum += value;
      }
      total = sum;
      benchmark::DoNotOptimize(total);
    });

    for (auto* batch : batches) {
      graph.addDependency(*batch, reduce);
    }
  }

  compute::ComputeGraph graph;
  std::vector<double> values;
  double total = 0.0;
};

//==============================================================================
void BM_ComputeGraphBuild(benchmark::State& state)
{
  const auto itemCount = static_cast<int>(state.range(0));
  const auto batchSize = static_cast<int>(state.range(1));

  for (auto _ : state) {
    BatchedGraph graph(itemCount, batchSize);
    benchmark::DoNotOptimize(graph.graph.getNodeCount());
  }

  state.counters["items"] = itemCount;
  state.counters["batch_size"] = batchSize;
}

//==============================================================================
void BM_ComputeGraphSequential(benchmark::State& state)
{
  const auto itemCount = static_cast<int>(state.range(0));
  const auto batchSize = static_cast<int>(state.range(1));
  BatchedGraph graph(itemCount, batchSize);
  compute::SequentialExecutor executor;

  for (auto _ : state) {
    executor.execute(graph.graph);
  }

  state.counters["items"] = itemCount;
  state.counters["batch_size"] = batchSize;
}

//==============================================================================
void BM_ComputeGraphTaskflow(benchmark::State& state)
{
  const auto itemCount = static_cast<int>(state.range(0));
  const auto batchSize = static_cast<int>(state.range(1));
  BatchedGraph graph(itemCount, batchSize);
  compute::TaskflowExecutor executor;

  for (auto _ : state) {
    executor.execute(graph.graph);
  }

  state.counters["items"] = itemCount;
  state.counters["batch_size"] = batchSize;
}

//==============================================================================
struct KinematicsWorld
{
  KinematicsWorld(int parentCount, int childCount)
  {
    parents.reserve(static_cast<std::size_t>(parentCount));
    children.reserve(static_cast<std::size_t>(parentCount * childCount));

    for (int i = 0; i < parentCount; ++i) {
      auto parent = world.addFreeFrame("parent_" + std::to_string(i));

      Eigen::Isometry3d parentTransform = Eigen::Isometry3d::Identity();
      parentTransform.translate(Eigen::Vector3d(i, i * 0.5, i * 0.25));
      parent.setLocalTransform(parentTransform);

      for (int j = 0; j < childCount; ++j) {
        Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
        childOffset.translate(Eigen::Vector3d(0.0, j + 1.0, 0.0));
        children.push_back(world.addFixedFrame(
            "child_" + std::to_string(i) + "_" + std::to_string(j),
            parent,
            childOffset));
      }

      parents.push_back(parent);
    }

    world.enterSimulationMode();
  }

  void perturbParents(int iteration)
  {
    for (std::size_t i = 0; i < parents.size(); ++i) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.translate(
          Eigen::Vector3d(
              static_cast<double>(i) + iteration * 0.001,
              static_cast<double>(i) * 0.5,
              static_cast<double>(i) * 0.25));
      parents[i].setLocalTransform(transform);
    }
  }

  sx::World world;
  std::vector<sx::FreeFrame> parents;
  std::vector<sx::FixedFrame> children;
};

//==============================================================================
struct RigidBodyWorld
{
  explicit RigidBodyWorld(int bodyCount)
  {
    bodies.reserve(static_cast<std::size_t>(bodyCount));

    for (int i = 0; i < bodyCount; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + 0.001 * static_cast<double>(i);
      options.position = Eigen::Vector3d(i * 0.01, i * 0.02, i * 0.03);
      options.linearVelocity = Eigen::Vector3d(1.0, 0.5, 0.25);
      options.angularVelocity = Eigen::Vector3d(0.01, 0.02, 0.03);
      bodies.push_back(
          world.addRigidBody("body_" + std::to_string(i), options));
    }

    world.setTimeStep(0.001);
    world.enterSimulationMode();
  }

  sx::World world;
  std::vector<sx::RigidBody> bodies;
};

//==============================================================================
void BM_WorldUpdateKinematics(benchmark::State& state)
{
  const auto parentCount = static_cast<int>(state.range(0));
  const auto childCount = static_cast<int>(state.range(1));
  KinematicsWorld fixture(parentCount, childCount);
  int iteration = 0;

  for (auto _ : state) {
    fixture.perturbParents(iteration++);
    fixture.world.updateKinematics();
    benchmark::DoNotOptimize(fixture.children.back().getTranslation().x());
  }

  state.counters["parents"] = parentCount;
  state.counters["children_per_parent"] = childCount;
  state.SetItemsProcessed(state.iterations() * parentCount * childCount);
}

//==============================================================================
void BM_WorldStepSequential(benchmark::State& state)
{
  const auto parentCount = static_cast<int>(state.range(0));
  const auto childCount = static_cast<int>(state.range(1));
  KinematicsWorld fixture(parentCount, childCount);
  int iteration = 0;

  for (auto _ : state) {
    fixture.perturbParents(iteration++);
    fixture.world.step();
    benchmark::DoNotOptimize(fixture.children.back().getTranslation().x());
  }

  state.counters["parents"] = parentCount;
  state.counters["children_per_parent"] = childCount;
  state.SetItemsProcessed(state.iterations() * parentCount * childCount);
}

//==============================================================================
void BM_WorldStepTaskflow(benchmark::State& state)
{
  const auto parentCount = static_cast<int>(state.range(0));
  const auto childCount = static_cast<int>(state.range(1));
  KinematicsWorld fixture(parentCount, childCount);
  compute::TaskflowExecutor executor;
  int iteration = 0;

  for (auto _ : state) {
    fixture.perturbParents(iteration++);
    fixture.world.step(executor);
    benchmark::DoNotOptimize(fixture.children.back().getTranslation().x());
  }

  state.counters["parents"] = parentCount;
  state.counters["children_per_parent"] = childCount;
  state.SetItemsProcessed(state.iterations() * parentCount * childCount);
}

//==============================================================================
void BM_RigidBodyStepSequential(benchmark::State& state)
{
  const auto bodyCount = static_cast<int>(state.range(0));
  RigidBodyWorld fixture(bodyCount);

  for (auto _ : state) {
    fixture.world.step();
    benchmark::DoNotOptimize(fixture.bodies.back().getTranslation().x());
  }

  state.counters["bodies"] = bodyCount;
  state.SetItemsProcessed(state.iterations() * bodyCount);
}

//==============================================================================
void BM_RigidBodyStepTaskflow(benchmark::State& state)
{
  const auto bodyCount = static_cast<int>(state.range(0));
  RigidBodyWorld fixture(bodyCount);
  compute::TaskflowExecutor executor;

  for (auto _ : state) {
    fixture.world.step(executor);
    benchmark::DoNotOptimize(fixture.bodies.back().getTranslation().x());
  }

  state.counters["bodies"] = bodyCount;
  state.SetItemsProcessed(state.iterations() * bodyCount);
}

} // namespace

BENCHMARK(BM_ComputeGraphBuild)
    ->Args({1024, 1})
    ->Args({1024, 32})
    ->Args({4096, 64});
BENCHMARK(BM_ComputeGraphSequential)
    ->Args({1024, 1})
    ->Args({1024, 32})
    ->Args({4096, 64});
BENCHMARK(BM_ComputeGraphTaskflow)
    ->Args({1024, 1})
    ->Args({1024, 32})
    ->Args({4096, 64});
BENCHMARK(BM_WorldUpdateKinematics)
    ->Args({32, 8})
    ->Args({128, 8})
    ->Args({128, 32});
BENCHMARK(BM_WorldStepSequential)
    ->Args({32, 8})
    ->Args({128, 8})
    ->Args({128, 32});
BENCHMARK(BM_WorldStepTaskflow)->Args({32, 8})->Args({128, 8})->Args({128, 32});
BENCHMARK(BM_RigidBodyStepSequential)->Arg(128)->Arg(1024)->Arg(4096);
BENCHMARK(BM_RigidBodyStepTaskflow)->Arg(128)->Arg(1024)->Arg(4096);

BENCHMARK_MAIN();
