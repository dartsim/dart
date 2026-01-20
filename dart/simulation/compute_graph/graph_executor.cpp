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

#include "dart/common/Profile.hpp"
#include "dart/simulation/compute_graph/taskflow_executor.hpp"

#include <chrono>
#include <thread>

namespace dart::simulation {

GraphExecutorPtr GraphExecutor::create(const ExecutorConfig& config)
{
  if (config.forceSequential || config.numWorkers == 1) {
    return std::make_shared<SequentialExecutor>(config);
  }
  return std::make_shared<TaskflowExecutor>(config);
}

SequentialExecutor::SequentialExecutor(const ExecutorConfig& config)
  : mConfig(config)
{
}

void SequentialExecutor::execute(
    ComputeGraph& graph, const ExecutionContext& ctx)
{
  if (!graph.isFinalized() && !graph.finalize()) {
    return;
  }

  const bool enableProfiling = mConfig.enableProfiling;
  const auto onNodeComplete = mConfig.onNodeComplete;
  const bool reportTiming = static_cast<bool>(onNodeComplete);

  for (NodeId nodeId : graph.getTopologicalOrder()) {
    auto node = graph.getNode(nodeId);
    if (node) {
      if (enableProfiling || reportTiming) {
        auto start = std::chrono::steady_clock::time_point{};
        if (reportTiming) {
          start = std::chrono::steady_clock::now();
        }

        if (enableProfiling) {
#if defined(DART_PROFILE_HAS_TRACY) && DART_PROFILE_HAS_TRACY
          ZoneScoped;
          const auto& name = node->getName();
          ZoneName(name.c_str(), name.size());
#endif
          DART_PROFILE_TEXT_SCOPED(node->getName());
          node->execute(ctx);
        } else {
          node->execute(ctx);
        }

        if (reportTiming) {
          const auto end = std::chrono::steady_clock::now();
          const auto elapsedMs
              = std::chrono::duration<double, std::milli>(end - start).count();
          onNodeComplete(nodeId, elapsedMs);
        }
      } else {
        node->execute(ctx);
      }
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
