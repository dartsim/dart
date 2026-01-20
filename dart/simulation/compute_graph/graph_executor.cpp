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

#include "dart/simulation/compute_graph/graph_executor.hpp"

#include "dart/simulation/compute_graph/taskflow_executor.hpp"

#include <chrono>
#include <thread>

namespace dart::simulation {

GraphExecutorPtr GraphExecutor::create(const ExecutorConfig& config)
{
  if (config.forceSequential || config.numWorkers == 1) {
    return std::make_shared<SequentialExecutor>();
  }
  return std::make_shared<TaskflowExecutor>(config);
}

void SequentialExecutor::execute(
    ComputeGraph& graph, const ExecutionContext& ctx)
{
  if (!graph.isFinalized() && !graph.finalize()) {
    return;
  }

  for (NodeId nodeId : graph.getTopologicalOrder()) {
    auto node = graph.getNode(nodeId);
    if (node) {
      node->execute(ctx);
    }
  }
}

std::future<void> SequentialExecutor::executeAsync(
    ComputeGraph& graph, const ExecutionContext& ctx)
{
  return std::async(
      std::launch::deferred, [this, &graph, ctx]() { execute(graph, ctx); });
}

} // namespace dart::simulation
