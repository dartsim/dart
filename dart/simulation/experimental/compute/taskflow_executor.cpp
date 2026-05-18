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

#include "dart/simulation/experimental/compute/taskflow_executor.hpp"

#include "dart/simulation/experimental/compute/compute_graph.hpp"
#include "dart/simulation/experimental/compute/execution_profile.hpp"

#include <taskflow/taskflow.hpp>

#include <unordered_map>

namespace dart::simulation::experimental::compute {

//==============================================================================
TaskflowExecutor::TaskflowExecutor(std::size_t workerCount)
{
  if (workerCount == 0) {
    m_executor = std::make_unique<tf::Executor>();
  } else {
    m_executor = std::make_unique<tf::Executor>(workerCount);
  }
}

//==============================================================================
TaskflowExecutor::~TaskflowExecutor() = default;

//==============================================================================
TaskflowExecutor::TaskflowExecutor(TaskflowExecutor&&) noexcept = default;

//==============================================================================
TaskflowExecutor& TaskflowExecutor::operator=(TaskflowExecutor&&) noexcept
    = default;

//==============================================================================
void TaskflowExecutor::execute(const ComputeGraph& graph)
{
  tf::Taskflow taskflow;
  std::unordered_map<ComputeNode*, tf::Task> tasks;

  for (auto* node : graph.getNodes()) {
    tasks.emplace(
        node,
        taskflow.emplace([node]() { node->execute(); }).name(node->getName()));
  }

  for (const auto& edge : graph.getEdges()) {
    tasks.at(edge.from).precede(tasks.at(edge.to));
  }

  m_executor->run(taskflow).wait();
}

//==============================================================================
ComputeExecutionProfile TaskflowExecutor::executeProfiled(
    const ComputeGraph& graph)
{
  tf::Taskflow taskflow;
  std::unordered_map<ComputeNode*, tf::Task> tasks;
  ComputeExecutionProfiler profiler(graph, getWorkerCount());

  for (auto* node : graph.getNodes()) {
    tasks.emplace(
        node,
        taskflow.emplace([&profiler, node]() { profiler.executeNode(*node); })
            .name(node->getName()));
  }

  for (const auto& edge : graph.getEdges()) {
    tasks.at(edge.from).precede(tasks.at(edge.to));
  }

  profiler.start();
  m_executor->run(taskflow).wait();
  return profiler.finish();
}

//==============================================================================
std::size_t TaskflowExecutor::getWorkerCount() const
{
  return m_executor ? m_executor->num_workers() : 0;
}

} // namespace dart::simulation::experimental::compute
