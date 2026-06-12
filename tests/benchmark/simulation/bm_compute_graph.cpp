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

#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/compute/compute_graph.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/frame/fixed_frame.hpp>
#include <dart/simulation/frame/free_frame.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <string>
#include <vector>

namespace sx = dart::simulation;
namespace compute = dart::simulation::compute;

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
void BM_ComputeGraphParallel(benchmark::State& state)
{
  const auto itemCount = static_cast<int>(state.range(0));
  const auto batchSize = static_cast<int>(state.range(1));
  BatchedGraph graph(itemCount, batchSize);
  compute::ParallelExecutor executor;

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
void BM_WorldStepParallel(benchmark::State& state)
{
  const auto parentCount = static_cast<int>(state.range(0));
  const auto childCount = static_cast<int>(state.range(1));
  KinematicsWorld fixture(parentCount, childCount);
  compute::ParallelExecutor executor;
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
void BM_RigidBodyStepParallel(benchmark::State& state)
{
  const auto bodyCount = static_cast<int>(state.range(0));
  RigidBodyWorld fixture(bodyCount);
  compute::ParallelExecutor executor;

  for (auto _ : state) {
    fixture.world.step(executor);
    benchmark::DoNotOptimize(fixture.bodies.back().getTranslation().x());
  }

  state.counters["bodies"] = bodyCount;
  state.SetItemsProcessed(state.iterations() * bodyCount);
}

//==============================================================================
// Contact/constraint-shaped proxy: an iterative, sequentially-coupled sweep.
//
// Unlike the embarrassingly-parallel fixtures above, each constraint node reads
// and writes shared body state in a fixed order (Gauss-Seidel style), so the
// graph has a long critical path and little exploitable parallelism, and it
// touches memory through an irregular index permutation. This is the "hard
// case" that any multi-core or GPU speedup claim must be measured against, so
// scaling numbers are not gathered only on easy embarrassingly-parallel work.
struct ContactShapedGraph
{
  ContactShapedGraph(int bodyCount, int iterations)
    : velocities(static_cast<std::size_t>(std::max(bodyCount, 1)), 1.0),
      indices(static_cast<std::size_t>(std::max(bodyCount, 1)))
  {
    for (std::size_t i = 0; i < indices.size(); ++i) {
      indices[i] = static_cast<int>((i * 2654435761u) % indices.size());
    }

    // One node per solver iteration. Each node runs a full Gauss-Seidel sweep
    // over all bodies (heavy, O(bodies) sequential work), and each iteration
    // depends on the previous one, so the graph is a serial chain of heavy
    // nodes. Task-graph parallelism cannot speed it up, unlike the
    // embarrassingly-parallel fixtures above; sequential and parallel times
    // should be comparable. This keeps the node count small so graph scheduling
    // overhead does not dominate the measured solver work.
    compute::ComputeNode* previous = nullptr;
    for (int iter = 0; iter < iterations; ++iter) {
      auto& node = graph.addNode("sweep_" + std::to_string(iter), [this]() {
        for (std::size_t c = 0; c + 1 < indices.size(); ++c) {
          const auto a = static_cast<std::size_t>(indices[c]);
          const auto b = static_cast<std::size_t>(indices[c + 1]);
          const double delta = 0.01 * (velocities[a] - velocities[b]);
          velocities[a] -= delta;
          velocities[b] += delta;
        }
        benchmark::DoNotOptimize(velocities[0]);
      });
      if (previous != nullptr) {
        graph.addDependency(*previous, node);
      }
      previous = &node;
    }
  }

  compute::ComputeGraph graph;
  std::vector<double> velocities;
  std::vector<int> indices;
};

//==============================================================================
// Contact/constraint-shaped proxy with independent solver islands.
//
// Each island is internally Gauss-Seidel ordered, but islands write disjoint
// body-state ranges and can run concurrently. This is the first checked
// compute-bound surface where the Taskflow executor is expected to have real
// work to amortize scheduling overhead, unlike the Euler-only rigid-body step
// or the serial contact-shaped chain above.
struct ContactIslandShapedGraph
{
  ContactIslandShapedGraph(int islandCount, int bodiesPerIsland, int iterations)
    : islandCount(std::max(islandCount, 1)),
      bodiesPerIsland(std::max(bodiesPerIsland, 2)),
      iterations(std::max(iterations, 1)),
      velocities(
          static_cast<std::size_t>(this->islandCount * this->bodiesPerIsland),
          1.0),
      indices(static_cast<std::size_t>(this->bodiesPerIsland))
  {
    for (std::size_t i = 0; i < indices.size(); ++i) {
      indices[i] = static_cast<int>((i * 2654435761u) % indices.size());
    }

    auto& start = graph.addNode("start", []() {});
    std::vector<compute::ComputeNode*> islandNodes;
    islandNodes.reserve(static_cast<std::size_t>(this->islandCount));

    for (int island = 0; island < this->islandCount; ++island) {
      const auto offset
          = static_cast<std::size_t>(island * this->bodiesPerIsland);
      auto& node
          = graph.addNode("island_" + std::to_string(island), [this, offset]() {
              for (int iter = 0; iter < this->iterations; ++iter) {
                for (std::size_t c = 0; c + 1 < indices.size(); ++c) {
                  const auto a = offset + static_cast<std::size_t>(indices[c]);
                  const auto b
                      = offset + static_cast<std::size_t>(indices[c + 1]);
                  const double delta = 0.01 * (velocities[a] - velocities[b]);
                  velocities[a] -= delta;
                  velocities[b] += delta;
                }
              }
              benchmark::DoNotOptimize(velocities[offset]);
            });

      graph.addDependency(start, node);
      islandNodes.push_back(&node);
    }

    auto& reduce = graph.addNode("reduce", [this]() {
      double sum = 0.0;
      for (const auto velocity : velocities) {
        sum += velocity;
      }
      total = sum;
      benchmark::DoNotOptimize(total);
    });

    for (auto* node : islandNodes) {
      graph.addDependency(*node, reduce);
    }
  }

  compute::ComputeGraph graph;
  int islandCount;
  int bodiesPerIsland;
  int iterations;
  std::vector<double> velocities;
  std::vector<int> indices;
  double total = 0.0;
};

//==============================================================================
struct Phase5RigidBodyBatchFixture
{
  Phase5RigidBodyBatchFixture(int worldCount, int bodyCount)
  {
    const auto worlds = static_cast<std::size_t>(worldCount);
    const auto bodies = static_cast<std::size_t>(bodyCount);
    const auto totalBodies = worlds * bodies;

    initial.worldCount = worlds;
    initial.bodyCount = bodies;
    initial.position.resize(3 * totalBodies);
    initial.orientation.resize(4 * totalBodies);
    initial.linearVelocity.resize(3 * totalBodies);
    initial.angularVelocity.resize(3 * totalBodies);

    model.worldCount = worlds;
    model.bodyCount = bodies;
    model.inverseMass.resize(totalBodies);

    force.resize(3 * totalBodies);

    for (std::size_t world = 0; world < worlds; ++world) {
      for (std::size_t body = 0; body < bodies; ++body) {
        const auto index = world * bodies + body;
        const auto component = 3 * index;
        const auto quaternion = 4 * index;

        initial.position[component + 0] = 0.001 * static_cast<double>(body);
        initial.position[component + 1] = 0.002 * static_cast<double>(world);
        initial.position[component + 2] = 0.0;

        initial.orientation[quaternion + 0] = 1.0;
        initial.orientation[quaternion + 1] = 0.0;
        initial.orientation[quaternion + 2] = 0.0;
        initial.orientation[quaternion + 3] = 0.0;

        initial.linearVelocity[component + 0] = 0.5;
        initial.linearVelocity[component + 1] = 0.25;
        initial.linearVelocity[component + 2] = 0.125;

        initial.angularVelocity[component + 0] = 0.01;
        initial.angularVelocity[component + 1] = 0.02;
        initial.angularVelocity[component + 2] = 0.03;

        model.inverseMass[index] = 1.0 / (1.0 + 0.001 * body);

        force[component + 0] = 0.05;
        force[component + 1] = 0.025;
        force[component + 2] = 0.0125;
      }
    }
  }

  compute::RigidBodyStateBatch initial;
  compute::RigidBodyModelBatch model;
  std::vector<double> force;
};

//==============================================================================
void BM_ContactShapedSequential(benchmark::State& state)
{
  const auto bodyCount = static_cast<int>(state.range(0));
  const auto iterations = static_cast<int>(state.range(1));
  ContactShapedGraph fixture(bodyCount, iterations);
  compute::SequentialExecutor executor;

  for (auto _ : state) {
    executor.execute(fixture.graph);
  }

  state.counters["bodies"] = bodyCount;
  state.counters["iterations"] = iterations;
}

//==============================================================================
void BM_ContactShapedParallel(benchmark::State& state)
{
  const auto bodyCount = static_cast<int>(state.range(0));
  const auto iterations = static_cast<int>(state.range(1));
  ContactShapedGraph fixture(bodyCount, iterations);
  compute::ParallelExecutor executor;

  for (auto _ : state) {
    executor.execute(fixture.graph);
  }

  state.counters["bodies"] = bodyCount;
  state.counters["iterations"] = iterations;
}

//==============================================================================
void BM_ContactIslandShapedSequential(benchmark::State& state)
{
  const auto islandCount = static_cast<int>(state.range(0));
  const auto bodiesPerIsland = static_cast<int>(state.range(1));
  const auto iterations = static_cast<int>(state.range(2));
  ContactIslandShapedGraph fixture(islandCount, bodiesPerIsland, iterations);
  compute::SequentialExecutor executor;

  for (auto _ : state) {
    executor.execute(fixture.graph);
  }

  state.counters["islands"] = islandCount;
  state.counters["bodies_per_island"] = bodiesPerIsland;
  state.counters["iterations"] = iterations;
  state.SetItemsProcessed(
      state.iterations() * islandCount * bodiesPerIsland * iterations);
}

//==============================================================================
void BM_ContactIslandShapedParallel(benchmark::State& state)
{
  const auto islandCount = static_cast<int>(state.range(0));
  const auto bodiesPerIsland = static_cast<int>(state.range(1));
  const auto iterations = static_cast<int>(state.range(2));
  ContactIslandShapedGraph fixture(islandCount, bodiesPerIsland, iterations);
  compute::ParallelExecutor executor;

  for (auto _ : state) {
    executor.execute(fixture.graph);
  }

  state.counters["islands"] = islandCount;
  state.counters["bodies_per_island"] = bodiesPerIsland;
  state.counters["iterations"] = iterations;
  state.SetItemsProcessed(
      state.iterations() * islandCount * bodiesPerIsland * iterations);
}

//==============================================================================
void BM_Phase5RigidBodyBatchCpuBaseline(benchmark::State& state)
{
  const auto worldCount = static_cast<int>(state.range(0));
  const auto bodyCount = static_cast<int>(state.range(1));
  const auto stepCount = static_cast<int>(state.range(2));
  Phase5RigidBodyBatchFixture fixture(worldCount, bodyCount);

  for (auto _ : state) {
    auto current = fixture.initial;
    for (int step = 0; step < stepCount; ++step) {
      compute::integrateRigidBodyStateBatch(
          current, fixture.model, fixture.force, 0.001);
    }

    benchmark::DoNotOptimize(current.position.data());
    benchmark::DoNotOptimize(current.linearVelocity.data());
    benchmark::ClobberMemory();
  }

  state.counters["worlds"] = worldCount;
  state.counters["bodies_per_world"] = bodyCount;
  state.counters["steps"] = stepCount;
  state.SetItemsProcessed(
      state.iterations() * worldCount * bodyCount * stepCount);
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
BENCHMARK(BM_ComputeGraphParallel)
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
BENCHMARK(BM_WorldStepParallel)->Args({32, 8})->Args({128, 8})->Args({128, 32});
BENCHMARK(BM_RigidBodyStepSequential)->Arg(128)->Arg(1024)->Arg(4096);
BENCHMARK(BM_RigidBodyStepParallel)->Arg(128)->Arg(1024)->Arg(4096);
BENCHMARK(BM_ContactShapedSequential)
    ->Args({1024, 16})
    ->Args({4096, 16})
    ->Args({1024, 64});
BENCHMARK(BM_ContactShapedParallel)
    ->Args({1024, 16})
    ->Args({4096, 16})
    ->Args({1024, 64});
BENCHMARK(BM_ContactIslandShapedSequential)
    ->Args({4, 512, 64})
    ->Args({8, 512, 64})
    ->Args({16, 512, 64});
BENCHMARK(BM_ContactIslandShapedParallel)
    ->Args({4, 512, 64})
    ->Args({8, 512, 64})
    ->Args({16, 512, 64});
BENCHMARK(BM_Phase5RigidBodyBatchCpuBaseline)
    ->Args({1024, 128, 10})
    ->Args({4096, 128, 100});

BENCHMARK_MAIN();
