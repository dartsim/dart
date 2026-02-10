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

#include <dart/simulation/World.hpp>
#include <dart/simulation/compute_graph/compute_graph.hpp>
#include <dart/simulation/compute_graph/graph_executor.hpp>
#include <dart/simulation/compute_graph/taskflow_executor.hpp>
#include <dart/simulation/compute_graph/world_step_graph.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <Eigen/Dense>
#include <benchmark/benchmark.h>

#include <atomic>
#include <thread>

using namespace dart::simulation;

namespace {

void buildLinearGraph(ComputeGraph& graph, std::size_t numNodes)
{
  NodeId prev = kInvalidNodeId;
  for (std::size_t i = 0; i < numNodes; ++i) {
    NodeId current = graph.addNode(
        [](const ExecutionContext&) {
          volatile int x = 0;
          for (int j = 0; j < 100; ++j) {
            x += j;
          }
        },
        "node_" + std::to_string(i));

    if (prev != kInvalidNodeId) {
      graph.addEdge(prev, current);
    }
    prev = current;
  }
  graph.finalize();
}

void buildParallelGraph(ComputeGraph& graph, std::size_t numParallel)
{
  NodeId start = graph.addNode([](const ExecutionContext&) {}, "start");

  std::vector<NodeId> parallelNodes;
  for (std::size_t i = 0; i < numParallel; ++i) {
    NodeId n = graph.addNode(
        [](const ExecutionContext&) {
          volatile int x = 0;
          for (int j = 0; j < 1000; ++j) {
            x += j;
          }
        },
        "parallel_" + std::to_string(i));
    graph.addEdge(start, n);
    parallelNodes.push_back(n);
  }

  NodeId end = graph.addNode([](const ExecutionContext&) {}, "end");
  for (NodeId n : parallelNodes) {
    graph.addEdge(n, end);
  }

  graph.finalize();
}

dart::dynamics::SkeletonPtr createBenchSkeleton(const std::string& name)
{
  using namespace dart::dynamics;
  auto skel = Skeleton::create(name);

  auto [j1, b1] = skel->createJointAndBodyNodePair<FreeJoint>();
  j1->setName(name + "_j1");
  b1->setMass(1.0);
  b1->setName(name + "_b1");

  auto [j2, b2] = skel->createJointAndBodyNodePair<RevoluteJoint>(b1);
  j2->setName(name + "_j2");
  b2->setMass(0.5);
  b2->setName(name + "_b2");
  j2->setAxis(Eigen::Vector3d::UnitY());

  auto [j3, b3] = skel->createJointAndBodyNodePair<RevoluteJoint>(b2);
  j3->setName(name + "_j3");
  b3->setMass(0.3);
  b3->setName(name + "_b3");
  j3->setAxis(Eigen::Vector3d::UnitZ());

  for (std::size_t i = 0; i < skel->getNumJoints(); ++i) {
    skel->getJoint(i)->setActuatorType(dart::dynamics::Joint::ACCELERATION);
  }

  Eigen::VectorXd pos
      = Eigen::VectorXd::LinSpaced(skel->getNumDofs(), 0.1, 0.5);
  Eigen::VectorXd vel
      = Eigen::VectorXd::LinSpaced(skel->getNumDofs(), -0.2, 0.2);
  skel->setPositions(pos);
  skel->setVelocities(vel);

  return skel;
}

} // namespace

static void BM_SequentialExecutor_LinearGraph(benchmark::State& state)
{
  const auto numNodes = static_cast<std::size_t>(state.range(0));

  ComputeGraph graph;
  buildLinearGraph(graph, numNodes);

  SequentialExecutor executor;
  ExecutionContext ctx;

  for (auto _ : state) {
    executor.execute(graph, ctx);
  }

  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * numNodes));
}
BENCHMARK(BM_SequentialExecutor_LinearGraph)->RangeMultiplier(2)->Range(8, 256);

static void BM_TaskflowExecutor_LinearGraph(benchmark::State& state)
{
  const auto numNodes = static_cast<std::size_t>(state.range(0));

  ComputeGraph graph;
  buildLinearGraph(graph, numNodes);

  ExecutorConfig config;
  config.numWorkers = std::thread::hardware_concurrency();
  TaskflowExecutor executor(config);
  ExecutionContext ctx;

  for (auto _ : state) {
    executor.execute(graph, ctx);
  }

  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * numNodes));
}
BENCHMARK(BM_TaskflowExecutor_LinearGraph)->RangeMultiplier(2)->Range(8, 256);

static void BM_SequentialExecutor_ParallelGraph(benchmark::State& state)
{
  const auto numParallel = static_cast<std::size_t>(state.range(0));

  ComputeGraph graph;
  buildParallelGraph(graph, numParallel);

  SequentialExecutor executor;
  ExecutionContext ctx;

  for (auto _ : state) {
    executor.execute(graph, ctx);
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * numParallel));
}
BENCHMARK(BM_SequentialExecutor_ParallelGraph)
    ->RangeMultiplier(2)
    ->Range(4, 64);

static void BM_TaskflowExecutor_ParallelGraph(benchmark::State& state)
{
  const auto numParallel = static_cast<std::size_t>(state.range(0));

  ComputeGraph graph;
  buildParallelGraph(graph, numParallel);

  ExecutorConfig config;
  config.numWorkers = std::thread::hardware_concurrency();
  TaskflowExecutor executor(config);
  ExecutionContext ctx;

  for (auto _ : state) {
    executor.execute(graph, ctx);
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * numParallel));
}
BENCHMARK(BM_TaskflowExecutor_ParallelGraph)->RangeMultiplier(2)->Range(4, 64);

static void BM_TaskflowExecutor_ThreadScaling(benchmark::State& state)
{
  const auto numWorkers = static_cast<std::size_t>(state.range(0));

  ComputeGraph graph;
  buildParallelGraph(graph, 32);

  ExecutorConfig config;
  config.numWorkers = numWorkers;
  TaskflowExecutor executor(config);
  ExecutionContext ctx;

  for (auto _ : state) {
    executor.execute(graph, ctx);
  }

  state.SetLabel(std::to_string(numWorkers) + " threads");
}
BENCHMARK(BM_TaskflowExecutor_ThreadScaling)->Arg(1)->Arg(2)->Arg(4)->Arg(8);

static void BM_WorldStep_Sequential(benchmark::State& state)
{
  const auto numSkeletons = static_cast<std::size_t>(state.range(0));

  auto world = dart::simulation::World::create();
  world->setTimeStep(1e-3);
  for (std::size_t i = 0; i < numSkeletons; ++i) {
    world->addSkeleton(createBenchSkeleton("skel_" + std::to_string(i)));
  }

  for (auto _ : state) {
    world->step(false);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * numSkeletons));
}
BENCHMARK(BM_WorldStep_Sequential)->RangeMultiplier(2)->Range(1, 64);

static void BM_WorldStep_Graph(benchmark::State& state)
{
  const auto numSkeletons = static_cast<std::size_t>(state.range(0));

  auto world = dart::simulation::World::create();
  world->setTimeStep(1e-3);
  for (std::size_t i = 0; i < numSkeletons; ++i) {
    world->addSkeleton(createBenchSkeleton("skel_" + std::to_string(i)));
  }

  dart::simulation::GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 0;
  graphConfig.batchSize = 0;
  world->setGraphExecutionConfig(graphConfig);

  for (auto _ : state) {
    world->step(false);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * numSkeletons));
}
BENCHMARK(BM_WorldStep_Graph)->RangeMultiplier(2)->Range(1, 64);

static void BM_WorldStep_Graph_BatchSize(benchmark::State& state)
{
  const auto numSkeletons = static_cast<std::size_t>(state.range(0));
  const auto batchSize = static_cast<std::size_t>(state.range(1));

  auto world = dart::simulation::World::create();
  world->setTimeStep(1e-3);
  for (std::size_t i = 0; i < numSkeletons; ++i) {
    world->addSkeleton(createBenchSkeleton("skel_" + std::to_string(i)));
  }

  dart::simulation::GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 0;
  graphConfig.batchSize = batchSize;
  world->setGraphExecutionConfig(graphConfig);

  for (auto _ : state) {
    world->step(false);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * numSkeletons));
}
BENCHMARK(BM_WorldStep_Graph_BatchSize)
    ->Args({32, 1})
    ->Args({32, 4})
    ->Args({32, 8})
    ->Args({64, 1})
    ->Args({64, 4})
    ->Args({64, 8});

BENCHMARK_MAIN();
