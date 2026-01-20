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

#include <dart/simulation/experimental/compute/compute_graph.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/taskflow_executor.hpp>

#include <benchmark/benchmark.h>

#include <atomic>
#include <cmath>
#include <thread>

namespace compute = dart::simulation::experimental::compute;

namespace {

void simulateWork(int microseconds)
{
  auto start = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::high_resolution_clock::now() - start)
             .count()
         < microseconds) {
  }
}

compute::ComputeGraph buildLinearChain(int nodeCount, int workMicroseconds)
{
  compute::ComputeGraph graph;

  compute::ComputeNode* prev = nullptr;
  for (int i = 0; i < nodeCount; ++i) {
    auto& node = graph.addNode(
        "node_" + std::to_string(i), [workMicroseconds]() {
          simulateWork(workMicroseconds);
        });
    if (prev) {
      graph.addDependency(*prev, node);
    }
    prev = &node;
  }

  return graph;
}

compute::ComputeGraph buildParallelFanOut(int nodeCount, int workMicroseconds)
{
  compute::ComputeGraph graph;

  auto& start = graph.addNode("start", []() {});
  auto& end = graph.addNode("end", []() {});

  for (int i = 0; i < nodeCount; ++i) {
    auto& node = graph.addNode(
        "worker_" + std::to_string(i), [workMicroseconds]() {
          simulateWork(workMicroseconds);
        });
    graph.addDependency(start, node);
    graph.addDependency(node, end);
  }

  return graph;
}

compute::ComputeGraph buildDiamond(int width, int depth, int workMicroseconds)
{
  compute::ComputeGraph graph;

  std::vector<compute::ComputeNode*> prevLayer;
  prevLayer.push_back(
      &graph.addNode("start", [workMicroseconds]() {
        simulateWork(workMicroseconds);
      }));

  for (int d = 0; d < depth; ++d) {
    std::vector<compute::ComputeNode*> currentLayer;
    for (int w = 0; w < width; ++w) {
      auto& node = graph.addNode(
          "layer" + std::to_string(d) + "_" + std::to_string(w),
          [workMicroseconds]() { simulateWork(workMicroseconds); });
      currentLayer.push_back(&node);

      for (auto* prev : prevLayer) {
        graph.addDependency(*prev, node);
      }
    }
    prevLayer = currentLayer;
  }

  auto& end = graph.addNode("end", [workMicroseconds]() {
    simulateWork(workMicroseconds);
  });
  for (auto* prev : prevLayer) {
    graph.addDependency(*prev, end);
  }

  return graph;
}

} // namespace

static void BM_SequentialExecutor_LinearChain(benchmark::State& state)
{
  const int nodeCount = state.range(0);
  const int workUs = 10;

  auto graph = buildLinearChain(nodeCount, workUs);
  compute::SequentialExecutor executor;

  for (auto _ : state) {
    executor.execute(graph);
  }

  state.SetItemsProcessed(state.iterations() * nodeCount);
}
BENCHMARK(BM_SequentialExecutor_LinearChain)->Arg(10)->Arg(100)->Arg(1000);

static void BM_TaskflowExecutor_LinearChain(benchmark::State& state)
{
  const int nodeCount = state.range(0);
  const int numThreads = state.range(1);
  const int workUs = 10;

  auto graph = buildLinearChain(nodeCount, workUs);
  compute::TaskflowExecutor executor(numThreads);

  for (auto _ : state) {
    executor.execute(graph);
  }

  state.SetItemsProcessed(state.iterations() * nodeCount);
  state.counters["threads"] = numThreads;
}
BENCHMARK(BM_TaskflowExecutor_LinearChain)
    ->Args({100, 1})
    ->Args({100, 2})
    ->Args({100, 4})
    ->Args({1000, 1})
    ->Args({1000, 2})
    ->Args({1000, 4});

static void BM_SequentialExecutor_ParallelFanOut(benchmark::State& state)
{
  const int nodeCount = state.range(0);
  const int workUs = 100;

  auto graph = buildParallelFanOut(nodeCount, workUs);
  compute::SequentialExecutor executor;

  for (auto _ : state) {
    executor.execute(graph);
  }

  state.SetItemsProcessed(state.iterations() * nodeCount);
}
BENCHMARK(BM_SequentialExecutor_ParallelFanOut)->Arg(10)->Arg(100)->Arg(500);

static void BM_TaskflowExecutor_ParallelFanOut(benchmark::State& state)
{
  const int nodeCount = state.range(0);
  const int numThreads = state.range(1);
  const int workUs = 100;

  auto graph = buildParallelFanOut(nodeCount, workUs);
  compute::TaskflowExecutor executor(numThreads);

  for (auto _ : state) {
    executor.execute(graph);
  }

  state.SetItemsProcessed(state.iterations() * nodeCount);
  state.counters["threads"] = numThreads;
  state.counters["expected_speedup"] = std::min(numThreads, nodeCount);
}
BENCHMARK(BM_TaskflowExecutor_ParallelFanOut)
    ->Args({10, 1})
    ->Args({10, 2})
    ->Args({10, 4})
    ->Args({100, 1})
    ->Args({100, 2})
    ->Args({100, 4})
    ->Args({100, 8})
    ->Args({500, 4})
    ->Args({500, 8});

static void BM_SequentialExecutor_Diamond(benchmark::State& state)
{
  const int width = state.range(0);
  const int depth = state.range(1);
  const int workUs = 50;

  auto graph = buildDiamond(width, depth, workUs);
  compute::SequentialExecutor executor;

  for (auto _ : state) {
    executor.execute(graph);
  }

  state.SetItemsProcessed(state.iterations() * (width * depth + 2));
}
BENCHMARK(BM_SequentialExecutor_Diamond)
    ->Args({4, 10})
    ->Args({8, 10})
    ->Args({16, 10});

static void BM_TaskflowExecutor_Diamond(benchmark::State& state)
{
  const int width = state.range(0);
  const int depth = state.range(1);
  const int numThreads = state.range(2);
  const int workUs = 50;

  auto graph = buildDiamond(width, depth, workUs);
  compute::TaskflowExecutor executor(numThreads);

  for (auto _ : state) {
    executor.execute(graph);
  }

  state.SetItemsProcessed(state.iterations() * (width * depth + 2));
  state.counters["threads"] = numThreads;
  state.counters["width"] = width;
  state.counters["depth"] = depth;
}
BENCHMARK(BM_TaskflowExecutor_Diamond)
    ->Args({4, 10, 1})
    ->Args({4, 10, 2})
    ->Args({4, 10, 4})
    ->Args({8, 10, 1})
    ->Args({8, 10, 4})
    ->Args({8, 10, 8})
    ->Args({16, 10, 4})
    ->Args({16, 10, 8});

static void BM_GraphConstruction(benchmark::State& state)
{
  const int nodeCount = state.range(0);

  for (auto _ : state) {
    compute::ComputeGraph graph;
    compute::ComputeNode* prev = nullptr;
    for (int i = 0; i < nodeCount; ++i) {
      auto& node = graph.addNode("node_" + std::to_string(i), []() {});
      if (prev) {
        graph.addDependency(*prev, node);
      }
      prev = &node;
    }
    benchmark::DoNotOptimize(graph.getNodeCount());
  }

  state.SetItemsProcessed(state.iterations() * nodeCount);
}
BENCHMARK(BM_GraphConstruction)->Arg(100)->Arg(1000)->Arg(10000);

static void BM_TopologicalSort(benchmark::State& state)
{
  const int nodeCount = state.range(0);

  auto graph = buildLinearChain(nodeCount, 0);

  for (auto _ : state) {
    auto order = graph.getTopologicalOrder();
    benchmark::DoNotOptimize(order.size());
  }

  state.SetItemsProcessed(state.iterations() * nodeCount);
}
BENCHMARK(BM_TopologicalSort)->Arg(100)->Arg(1000)->Arg(10000);

static void BM_ExecutorOverhead(benchmark::State& state)
{
  const int nodeCount = state.range(0);
  const bool useParallel = state.range(1) != 0;

  compute::ComputeGraph graph;
  for (int i = 0; i < nodeCount; ++i) {
    graph.addNode("node_" + std::to_string(i), []() {});
  }

  std::unique_ptr<compute::ComputeExecutor> executor;
  if (useParallel) {
    executor = std::make_unique<compute::TaskflowExecutor>(4);
  } else {
    executor = std::make_unique<compute::SequentialExecutor>();
  }

  for (auto _ : state) {
    executor->execute(graph);
  }

  state.SetItemsProcessed(state.iterations() * nodeCount);
  state.counters["parallel"] = useParallel;
}
BENCHMARK(BM_ExecutorOverhead)
    ->Args({1, 0})
    ->Args({1, 1})
    ->Args({10, 0})
    ->Args({10, 1})
    ->Args({100, 0})
    ->Args({100, 1});

BENCHMARK_MAIN();
