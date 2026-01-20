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

#include <dart/simulation/compute_graph/compute_graph.hpp>
#include <dart/simulation/compute_graph/compute_node.hpp>
#include <dart/simulation/compute_graph/graph_executor.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <vector>

using namespace dart::simulation;

TEST(ComputeGraph, AddNodes)
{
  ComputeGraph graph;

  int callCount = 0;
  NodeId id1
      = graph.addNode([&](const ExecutionContext&) { ++callCount; }, "node1");
  NodeId id2
      = graph.addNode([&](const ExecutionContext&) { ++callCount; }, "node2");

  EXPECT_NE(id1, kInvalidNodeId);
  EXPECT_NE(id2, kInvalidNodeId);
  EXPECT_NE(id1, id2);
  EXPECT_EQ(graph.getNodeCount(), 2u);
}

TEST(ComputeGraph, AddEdges)
{
  ComputeGraph graph;

  NodeId n1 = graph.addNode([](const ExecutionContext&) {}, "n1");
  NodeId n2 = graph.addNode([](const ExecutionContext&) {}, "n2");
  NodeId n3 = graph.addNode([](const ExecutionContext&) {}, "n3");

  EXPECT_TRUE(graph.addEdge(n1, n2));
  EXPECT_TRUE(graph.addEdge(n2, n3));
  EXPECT_TRUE(graph.addEdge(n1, n3));

  auto preds = graph.getPredecessors(n3);
  EXPECT_EQ(preds.size(), 2u);

  auto succs = graph.getSuccessors(n1);
  EXPECT_EQ(succs.size(), 2u);
}

TEST(ComputeGraph, RejectCycle)
{
  ComputeGraph graph;

  NodeId n1 = graph.addNode([](const ExecutionContext&) {}, "n1");
  NodeId n2 = graph.addNode([](const ExecutionContext&) {}, "n2");
  NodeId n3 = graph.addNode([](const ExecutionContext&) {}, "n3");

  EXPECT_TRUE(graph.addEdge(n1, n2));
  EXPECT_TRUE(graph.addEdge(n2, n3));
  EXPECT_FALSE(graph.addEdge(n3, n1));
}

TEST(ComputeGraph, RejectSelfLoop)
{
  ComputeGraph graph;

  NodeId n1 = graph.addNode([](const ExecutionContext&) {}, "n1");
  EXPECT_FALSE(graph.addEdge(n1, n1));
}

TEST(ComputeGraph, TopologicalOrder)
{
  ComputeGraph graph;

  NodeId n1 = graph.addNode([](const ExecutionContext&) {}, "n1");
  NodeId n2 = graph.addNode([](const ExecutionContext&) {}, "n2");
  NodeId n3 = graph.addNode([](const ExecutionContext&) {}, "n3");

  graph.addEdge(n1, n2);
  graph.addEdge(n2, n3);

  EXPECT_TRUE(graph.finalize());
  EXPECT_TRUE(graph.isFinalized());

  auto order = graph.getTopologicalOrder();
  EXPECT_EQ(order.size(), 3u);

  std::size_t pos1 = 0, pos2 = 0, pos3 = 0;
  for (std::size_t i = 0; i < order.size(); ++i) {
    if (order[i] == n1)
      pos1 = i;
    if (order[i] == n2)
      pos2 = i;
    if (order[i] == n3)
      pos3 = i;
  }

  EXPECT_LT(pos1, pos2);
  EXPECT_LT(pos2, pos3);
}

TEST(ComputeGraph, ExecutionOrderDeterministic)
{
  std::vector<int> executionOrder;

  ComputeGraph graph;
  NodeId n1 = graph.addNode(
      [&](const ExecutionContext&) { executionOrder.push_back(1); }, "n1");
  NodeId n2 = graph.addNode(
      [&](const ExecutionContext&) { executionOrder.push_back(2); }, "n2");
  NodeId n3 = graph.addNode(
      [&](const ExecutionContext&) { executionOrder.push_back(3); }, "n3");

  graph.addEdge(n1, n2);
  graph.addEdge(n2, n3);
  graph.finalize();

  SequentialExecutor executor;
  ExecutionContext ctx;

  for (int trial = 0; trial < 10; ++trial) {
    executionOrder.clear();
    executor.execute(graph, ctx);

    ASSERT_EQ(executionOrder.size(), 3u);
    EXPECT_EQ(executionOrder[0], 1);
    EXPECT_EQ(executionOrder[1], 2);
    EXPECT_EQ(executionOrder[2], 3);
  }
}

TEST(ComputeGraph, LambdaNode)
{
  int value = 0;
  auto node = std::make_shared<LambdaNode>(
      [&](const ExecutionContext& ctx) {
        value = static_cast<int>(ctx.frameNumber);
      },
      "lambda_test");

  ExecutionContext ctx;
  ctx.frameNumber = 42;
  node->execute(ctx);

  EXPECT_EQ(value, 42);
}

TEST(ComputeGraph, Clear)
{
  ComputeGraph graph;
  graph.addNode([](const ExecutionContext&) {}, "n1");
  graph.addNode([](const ExecutionContext&) {}, "n2");

  EXPECT_EQ(graph.getNodeCount(), 2u);

  graph.clear();

  EXPECT_EQ(graph.getNodeCount(), 0u);
}
