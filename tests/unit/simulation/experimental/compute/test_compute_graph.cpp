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
#include <dart/simulation/experimental/compute/compute_graph.hpp>
#include <dart/simulation/experimental/compute/compute_node.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/taskflow_executor.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

using namespace dart::simulation::experimental;
using namespace dart::simulation::experimental::compute;

class ComputeNodeTest : public ::testing::Test
{
};

TEST_F(ComputeNodeTest, ConstructionWithNameAndFunction)
{
  int counter = 0;
  ComputeNode node("test", [&counter]() { counter++; });

  EXPECT_EQ(node.getName(), "test");
  EXPECT_TRUE(node.isValid());
}

TEST_F(ComputeNodeTest, Execute)
{
  int counter = 0;
  ComputeNode node("increment", [&counter]() { counter++; });

  EXPECT_EQ(counter, 0);
  node.execute();
  EXPECT_EQ(counter, 1);
  node.execute();
  EXPECT_EQ(counter, 2);
}

TEST_F(ComputeNodeTest, ExecuteWithNullFunctionThrows)
{
  ComputeNode node("empty", nullptr);

  EXPECT_FALSE(node.isValid());
  EXPECT_THROW(node.execute(), InvalidOperationException);
}

TEST_F(ComputeNodeTest, MoveConstruction)
{
  int counter = 0;
  ComputeNode node1("movable", [&counter]() { counter++; });

  ComputeNode node2(std::move(node1));
  EXPECT_EQ(node2.getName(), "movable");
  node2.execute();
  EXPECT_EQ(counter, 1);
}

class ComputeGraphTest : public ::testing::Test
{
};

TEST_F(ComputeGraphTest, EmptyGraph)
{
  ComputeGraph graph;

  EXPECT_TRUE(graph.isEmpty());
  EXPECT_EQ(graph.getNodeCount(), 0);
  EXPECT_EQ(graph.getEdgeCount(), 0);
  EXPECT_TRUE(graph.validate());
}

TEST_F(ComputeGraphTest, AddSingleNode)
{
  ComputeGraph graph;
  auto& node = graph.addNode("A", []() {});

  EXPECT_FALSE(graph.isEmpty());
  EXPECT_EQ(graph.getNodeCount(), 1);
  EXPECT_EQ(node.getName(), "A");
}

TEST_F(ComputeGraphTest, AddMultipleNodes)
{
  ComputeGraph graph;
  graph.addNode("A", []() {});
  graph.addNode("B", []() {});
  graph.addNode("C", []() {});

  EXPECT_EQ(graph.getNodeCount(), 3);
}

TEST_F(ComputeGraphTest, AddDuplicateNameThrows)
{
  ComputeGraph graph;
  graph.addNode("A", []() {});

  EXPECT_THROW(graph.addNode("A", []() {}), InvalidArgumentException);
}

TEST_F(ComputeGraphTest, GetNodeByName)
{
  ComputeGraph graph;
  auto& original = graph.addNode("findme", []() {});

  auto* found = graph.getNode("findme");
  EXPECT_EQ(found, &original);

  auto* notFound = graph.getNode("nonexistent");
  EXPECT_EQ(notFound, nullptr);
}

TEST_F(ComputeGraphTest, AddDependency)
{
  ComputeGraph graph;
  auto& a = graph.addNode("A", []() {});
  auto& b = graph.addNode("B", []() {});

  graph.addDependency(a, b);

  EXPECT_EQ(graph.getEdgeCount(), 1);
}

TEST_F(ComputeGraphTest, AddDependencyWithExternalNodeThrows)
{
  ComputeGraph graph1;
  ComputeGraph graph2;
  auto& a = graph1.addNode("A", []() {});
  auto& b = graph2.addNode("B", []() {});

  EXPECT_THROW(graph1.addDependency(a, b), InvalidArgumentException);
}

TEST_F(ComputeGraphTest, AddSelfDependencyThrows)
{
  ComputeGraph graph;
  auto& a = graph.addNode("A", []() {});

  EXPECT_THROW(graph.addDependency(a, a), InvalidOperationException);
}

TEST_F(ComputeGraphTest, AddCyclicDependencyThrows)
{
  ComputeGraph graph;
  auto& a = graph.addNode("A", []() {});
  auto& b = graph.addNode("B", []() {});
  auto& c = graph.addNode("C", []() {});

  graph.addDependency(a, b);
  graph.addDependency(b, c);

  EXPECT_THROW(graph.addDependency(c, a), InvalidOperationException);
}

TEST_F(ComputeGraphTest, DuplicateDependencyIsNoOp)
{
  ComputeGraph graph;
  auto& a = graph.addNode("A", []() {});
  auto& b = graph.addNode("B", []() {});

  graph.addDependency(a, b);
  graph.addDependency(a, b);

  EXPECT_EQ(graph.getEdgeCount(), 1);
}

TEST_F(ComputeGraphTest, TopologicalOrderLinearChain)
{
  ComputeGraph graph;
  auto& a = graph.addNode("A", []() {});
  auto& b = graph.addNode("B", []() {});
  auto& c = graph.addNode("C", []() {});

  graph.addDependency(a, b);
  graph.addDependency(b, c);

  auto order = graph.getTopologicalOrder();
  ASSERT_EQ(order.size(), 3);

  auto indexOf = [&order](const std::string& name) {
    for (std::size_t i = 0; i < order.size(); ++i) {
      if (order[i]->getName() == name)
        return i;
    }
    return std::string::npos;
  };

  EXPECT_LT(indexOf("A"), indexOf("B"));
  EXPECT_LT(indexOf("B"), indexOf("C"));
}

TEST_F(ComputeGraphTest, TopologicalOrderDiamond)
{
  ComputeGraph graph;
  auto& a = graph.addNode("A", []() {});
  auto& b = graph.addNode("B", []() {});
  auto& c = graph.addNode("C", []() {});
  auto& d = graph.addNode("D", []() {});

  graph.addDependency(a, b);
  graph.addDependency(a, c);
  graph.addDependency(b, d);
  graph.addDependency(c, d);

  auto order = graph.getTopologicalOrder();
  ASSERT_EQ(order.size(), 4);

  auto indexOf = [&order](const std::string& name) {
    for (std::size_t i = 0; i < order.size(); ++i) {
      if (order[i]->getName() == name)
        return i;
    }
    return std::string::npos;
  };

  EXPECT_LT(indexOf("A"), indexOf("B"));
  EXPECT_LT(indexOf("A"), indexOf("C"));
  EXPECT_LT(indexOf("B"), indexOf("D"));
  EXPECT_LT(indexOf("C"), indexOf("D"));
}

TEST_F(ComputeGraphTest, GetDependencies)
{
  ComputeGraph graph;
  auto& a = graph.addNode("A", []() {});
  auto& b = graph.addNode("B", []() {});
  auto& c = graph.addNode("C", []() {});

  graph.addDependency(a, c);
  graph.addDependency(b, c);

  auto deps = graph.getDependencies(c);
  EXPECT_EQ(deps.size(), 2);
}

TEST_F(ComputeGraphTest, GetDependents)
{
  ComputeGraph graph;
  auto& a = graph.addNode("A", []() {});
  auto& b = graph.addNode("B", []() {});
  auto& c = graph.addNode("C", []() {});

  graph.addDependency(a, b);
  graph.addDependency(a, c);

  auto deps = graph.getDependents(a);
  EXPECT_EQ(deps.size(), 2);
}

TEST_F(ComputeGraphTest, Clear)
{
  ComputeGraph graph;
  graph.addNode("A", []() {});
  graph.addNode("B", []() {});

  graph.clear();

  EXPECT_TRUE(graph.isEmpty());
  EXPECT_EQ(graph.getNodeCount(), 0);
  EXPECT_EQ(graph.getEdgeCount(), 0);
}

TEST_F(ComputeGraphTest, Validate)
{
  ComputeGraph graph;
  graph.addNode("A", []() {});
  graph.addNode("B", []() {});

  EXPECT_TRUE(graph.validate());
}

TEST_F(ComputeGraphTest, ValidateWithInvalidNode)
{
  ComputeGraph graph;
  graph.addNode("A", nullptr);

  EXPECT_FALSE(graph.validate());
}

class SequentialExecutorTest : public ::testing::Test
{
};

TEST_F(SequentialExecutorTest, WorkerCount)
{
  SequentialExecutor executor;
  EXPECT_EQ(executor.getWorkerCount(), 1);
}

TEST_F(SequentialExecutorTest, ExecuteEmptyGraph)
{
  ComputeGraph graph;
  SequentialExecutor executor;

  executor.execute(graph);
}

TEST_F(SequentialExecutorTest, ExecuteLinearChain)
{
  std::vector<std::string> order;

  ComputeGraph graph;
  auto& a = graph.addNode("A", [&order]() { order.push_back("A"); });
  auto& b = graph.addNode("B", [&order]() { order.push_back("B"); });
  auto& c = graph.addNode("C", [&order]() { order.push_back("C"); });

  graph.addDependency(a, b);
  graph.addDependency(b, c);

  SequentialExecutor executor;
  executor.execute(graph);

  ASSERT_EQ(order.size(), 3);
  EXPECT_EQ(order[0], "A");
  EXPECT_EQ(order[1], "B");
  EXPECT_EQ(order[2], "C");
}

TEST_F(SequentialExecutorTest, ExecuteDeterministic)
{
  std::vector<std::string> order1;
  std::vector<std::string> order2;

  auto buildGraph = [](std::vector<std::string>& order) {
    ComputeGraph graph;
    auto& a = graph.addNode("A", [&order]() { order.push_back("A"); });
    auto& b = graph.addNode("B", [&order]() { order.push_back("B"); });
    auto& c = graph.addNode("C", [&order]() { order.push_back("C"); });
    auto& d = graph.addNode("D", [&order]() { order.push_back("D"); });

    graph.addDependency(a, b);
    graph.addDependency(a, c);
    graph.addDependency(b, d);
    graph.addDependency(c, d);

    return graph;
  };

  auto graph1 = buildGraph(order1);
  auto graph2 = buildGraph(order2);

  SequentialExecutor executor;
  executor.execute(graph1);
  executor.execute(graph2);

  EXPECT_EQ(order1, order2);
}

class TaskflowExecutorTest : public ::testing::Test
{
};

TEST_F(TaskflowExecutorTest, DefaultWorkerCount)
{
  TaskflowExecutor executor;
  EXPECT_GE(executor.getWorkerCount(), 1);
}

TEST_F(TaskflowExecutorTest, ExplicitWorkerCount)
{
  TaskflowExecutor executor(4);
  EXPECT_EQ(executor.getWorkerCount(), 4);
}

TEST_F(TaskflowExecutorTest, ExecuteEmptyGraph)
{
  ComputeGraph graph;
  TaskflowExecutor executor(2);

  executor.execute(graph);
}

TEST_F(TaskflowExecutorTest, ExecuteLinearChain)
{
  std::vector<std::string> order;
  std::mutex mutex;

  ComputeGraph graph;
  auto& a = graph.addNode("A", [&]() {
    std::lock_guard lock(mutex);
    order.push_back("A");
  });
  auto& b = graph.addNode("B", [&]() {
    std::lock_guard lock(mutex);
    order.push_back("B");
  });
  auto& c = graph.addNode("C", [&]() {
    std::lock_guard lock(mutex);
    order.push_back("C");
  });

  graph.addDependency(a, b);
  graph.addDependency(b, c);

  TaskflowExecutor executor(2);
  executor.execute(graph);

  ASSERT_EQ(order.size(), 3);

  auto indexOf = [&order](const std::string& name) {
    for (std::size_t i = 0; i < order.size(); ++i) {
      if (order[i] == name)
        return i;
    }
    return std::string::npos;
  };

  EXPECT_LT(indexOf("A"), indexOf("B"));
  EXPECT_LT(indexOf("B"), indexOf("C"));
}

TEST_F(TaskflowExecutorTest, ExecuteParallelNodes)
{
  std::atomic<int> counter{0};
  std::atomic<int> maxConcurrent{0};

  ComputeGraph graph;
  auto& start = graph.addNode("start", []() {});

  for (int i = 0; i < 10; ++i) {
    auto& node = graph.addNode(
        "parallel_" + std::to_string(i), [&counter, &maxConcurrent]() {
          int current = ++counter;
          int expected = maxConcurrent.load();
          while (current > expected
                 && !maxConcurrent.compare_exchange_weak(expected, current)) {
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          --counter;
        });
    graph.addDependency(start, node);
  }

  TaskflowExecutor executor(4);
  executor.execute(graph);

  EXPECT_GE(maxConcurrent.load(), 1);
}

TEST_F(TaskflowExecutorTest, ResultsMatchSequential)
{
  std::vector<int> seqResults;
  std::vector<int> parResults;
  std::mutex mutex;

  auto buildGraph = [](std::vector<int>& results, std::mutex* m = nullptr) {
    ComputeGraph graph;
    auto& a = graph.addNode("A", [&results, m]() {
      if (m) {
        std::lock_guard lock(*m);
        results.push_back(1);
      } else {
        results.push_back(1);
      }
    });
    auto& b = graph.addNode("B", [&results, m]() {
      if (m) {
        std::lock_guard lock(*m);
        results.push_back(2);
      } else {
        results.push_back(2);
      }
    });
    auto& c = graph.addNode("C", [&results, m]() {
      if (m) {
        std::lock_guard lock(*m);
        results.push_back(3);
      } else {
        results.push_back(3);
      }
    });

    graph.addDependency(a, b);
    graph.addDependency(b, c);

    return graph;
  };

  auto seqGraph = buildGraph(seqResults);
  auto parGraph = buildGraph(parResults, &mutex);

  SequentialExecutor seqExec;
  TaskflowExecutor parExec(4);

  seqExec.execute(seqGraph);
  parExec.execute(parGraph);

  EXPECT_EQ(seqResults, parResults);
}

TEST_F(TaskflowExecutorTest, MoveConstruction)
{
  TaskflowExecutor exec1(2);
  auto workerCount = exec1.getWorkerCount();

  TaskflowExecutor exec2(std::move(exec1));
  EXPECT_EQ(exec2.getWorkerCount(), workerCount);
}
