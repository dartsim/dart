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

#include "dart/simulation/compute/parallel_executor.hpp"

#include "dart/simulation/common/assert.hpp"
#include "dart/simulation/compute/compute_graph.hpp"
#include "dart/simulation/compute/execution_profile.hpp"

#include <dart/config.hpp>

// clang-format off
// Taskflow's work-stealing queue uses std::bit_ceil/std::bit_width without
// including <bit>. libc++ (unlike libstdc++) does not pull <bit> in transitively
// under C++23, so include it explicitly before Taskflow.
#include <bit>

#include <taskflow/taskflow.hpp>
// clang-format on

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

namespace dart::simulation::compute {

//==============================================================================
class ParallelExecutor::Impl
{
public:
  explicit Impl(std::size_t workerCount)
  {
    if (workerCount == 0) {
      executor = std::make_unique<tf::Executor>();
    } else {
      executor = std::make_unique<tf::Executor>(workerCount);
    }
  }

  // Dispatch a cached task: profile through the per-run profiler when one is
  // active, otherwise execute the node directly. Captured by the cached tasks
  // via the stable `this` pointer so a single Taskflow serves both execute()
  // and executeProfiled().
  void runNode(ComputeNode* node)
  {
#if DART_BUILD_PROFILE
    if (activeProfiler != nullptr) {
      activeProfiler->executeNode(*node);
      return;
    }
#endif
    node->execute();
  }

  // Reuse the Taskflow built for the most recent same-shape graph so warmed
  // same-shape parallel executions skip the per-run task/edge rebuild. The
  // shape is identified by the (alloc-free) topological-order and edge views;
  // any node-set or dependency change rebuilds the cache.
  bool cacheMatches(const ComputeGraph& graph) const
  {
    if (!cacheValid) {
      return false;
    }
    const auto order = graph.getTopologicalOrderView();
    if (order.size() != cachedNodes.size()
        || !std::equal(order.begin(), order.end(), cachedNodes.begin())) {
      return false;
    }
    const auto edges = graph.getEdges();
    return edges.size() == cachedEdges.size()
           && std::equal(
               edges.begin(),
               edges.end(),
               cachedEdges.begin(),
               [](const ComputeEdge& a, const ComputeEdge& b) {
                 return a.from == b.from && a.to == b.to;
               });
  }

  void ensureTaskflow(const ComputeGraph& graph)
  {
    if (cacheMatches(graph)) {
      return;
    }

    cachedTaskflow.clear();
    cachedTasks.clear();
    const auto order = graph.getTopologicalOrderView();
    for (auto* node : order) {
      cachedTasks.emplace(
          node,
          cachedTaskflow.emplace([this, node]() { runNode(node); })
              .name(node->getName()));
    }
    for (const auto& edge : graph.getEdges()) {
      cachedTasks.at(edge.from).precede(cachedTasks.at(edge.to));
    }

    cachedNodes.assign(order.begin(), order.end());
    const auto edges = graph.getEdges();
    cachedEdges.assign(edges.begin(), edges.end());
    cacheValid = true;
  }

  std::unique_ptr<tf::Executor> executor;
  std::size_t inlineThreshold = 1;
  std::vector<ComputeExecutionProfile::Duration> inlineProfilePathTimes;

  tf::Taskflow cachedTaskflow;
  std::unordered_map<ComputeNode*, tf::Task> cachedTasks;
  std::vector<ComputeNode*> cachedNodes;
  std::vector<ComputeEdge> cachedEdges;
  bool cacheValid = false;
#if DART_BUILD_PROFILE
  ComputeExecutionProfiler* activeProfiler = nullptr;
#endif
};

//==============================================================================
ParallelExecutor::ParallelExecutor(std::size_t workerCount)
  : m_impl(std::make_unique<Impl>(workerCount))
{
}

//==============================================================================
ParallelExecutor::~ParallelExecutor() = default;

//==============================================================================
ParallelExecutor::ParallelExecutor(ParallelExecutor&&) noexcept = default;

//==============================================================================
ParallelExecutor& ParallelExecutor::operator=(ParallelExecutor&&) noexcept
    = default;

//==============================================================================
void ParallelExecutor::execute(const ComputeGraph& graph)
{
  // Debug-only diagnostic aid: explicit graph edges remain the correctness
  // source of truth, so stage authors must encode every cross-node data
  // dependency as an edge rather than relying on this check (it is compiled out
  // under NDEBUG).
  DART_SIMULATION_ASSERT_MSG(
      graph.findResourceHazards().empty(),
      "Parallel execution found unordered resource-access hazards; add an "
      "explicit dependency or declare a reduction");

  // Cost gate: a graph at or below the inline threshold runs in topological
  // order on the calling thread, skipping Taskflow build/scheduling overhead.
  if (graph.getNodeCount() <= m_impl->inlineThreshold) {
    for (auto* node : graph.getTopologicalOrderView()) {
      node->execute();
    }
    return;
  }

  m_impl->ensureTaskflow(graph);
#if DART_BUILD_PROFILE
  m_impl->activeProfiler = nullptr;
#endif
  m_impl->executor->run(m_impl->cachedTaskflow).get();
}

//==============================================================================
ComputeExecutionProfile ParallelExecutor::executeProfiled(
    const ComputeGraph& graph)
{
  ComputeExecutionProfile profile;
  executeProfiled(graph, profile);
  return profile;
}

//==============================================================================
void ParallelExecutor::executeProfiled(
    const ComputeGraph& graph, ComputeExecutionProfile& profile)
{
#if DART_BUILD_PROFILE
  DART_SIMULATION_ASSERT_MSG(
      graph.findResourceHazards().empty(),
      "Parallel execution found unordered resource-access hazards; add an "
      "explicit dependency or declare a reduction");

  // Cost gate: mirror execute(); a sub-threshold graph profiles its inline
  // (sequential) run, which is what actually executes.
  if (graph.getNodeCount() <= m_impl->inlineThreshold) {
    ComputeExecutionProfiler::executeInline(
        graph, getWorkerCount(), profile, m_impl->inlineProfilePathTimes);
    return;
  }

  m_impl->ensureTaskflow(graph);
  ComputeExecutionProfiler profiler(graph, getWorkerCount());
  m_impl->activeProfiler = &profiler;
  profiler.start();
  m_impl->executor->run(m_impl->cachedTaskflow).get();
  m_impl->activeProfiler = nullptr;
  profile = profiler.finish();
#else
  execute(graph);
  profile = {};
#endif
}

//==============================================================================
std::size_t ParallelExecutor::getWorkerCount() const
{
  return m_impl && m_impl->executor ? m_impl->executor->num_workers() : 0;
}

//==============================================================================
void ParallelExecutor::setInlineThreshold(std::size_t threshold) noexcept
{
  m_impl->inlineThreshold = threshold;
}

//==============================================================================
std::size_t ParallelExecutor::getInlineThreshold() const noexcept
{
  return m_impl->inlineThreshold;
}

} // namespace dart::simulation::compute
