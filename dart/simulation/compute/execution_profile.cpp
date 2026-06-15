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

#include "dart/simulation/compute/execution_profile.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/compute/compute_graph.hpp"
#include "dart/simulation/compute/compute_node.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <utility>

namespace dart::simulation::compute {

namespace {

using Duration = ComputeExecutionProfile::Duration;

//==============================================================================
double toMilliseconds(Duration duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

//==============================================================================
std::vector<std::size_t> computeNodeLevels(
    const ComputeGraph& graph, const std::vector<ComputeNode*>& nodes)
{
  std::unordered_map<const ComputeNode*, std::size_t> nodeIndices;
  nodeIndices.reserve(nodes.size());
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    nodeIndices.emplace(nodes[i], i);
  }

  std::vector<std::size_t> levels(nodes.size(), 0);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (auto* dependency : graph.getDependencies(*nodes[i])) {
      const auto it = nodeIndices.find(dependency);
      if (it != nodeIndices.end()) {
        levels[i] = std::max(levels[i], levels[it->second] + 1);
      }
    }
  }

  return levels;
}

//==============================================================================
std::size_t computeMaxParallelism(
    const std::vector<ComputeNodeExecutionProfile>& nodes)
{
  struct Event
  {
    Duration time{};
    int delta = 0;
  };

  std::vector<Event> events;
  events.reserve(nodes.size() * 2);
  for (const auto& node : nodes) {
    events.push_back({node.startTime, 1});
    events.push_back({node.endTime, -1});
  }

  std::ranges::sort(events, [](const auto& a, const auto& b) {
    if (a.time == b.time) {
      return a.delta < b.delta;
    }
    return a.time < b.time;
  });

  std::size_t maxParallelism = 0;
  int currentParallelism = 0;
  for (const auto& event : events) {
    currentParallelism += event.delta;
    if (currentParallelism > 0) {
      maxParallelism = std::max(
          maxParallelism, static_cast<std::size_t>(currentParallelism));
    }
  }

  return maxParallelism;
}

//==============================================================================
std::size_t findNodeIndex(
    std::span<ComputeNode* const> nodes, const ComputeNode* node)
{
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i] == node) {
      return i;
    }
  }
  return nodes.size();
}

//==============================================================================
void resetNodeProfile(
    ComputeNodeExecutionProfile& profile,
    const ComputeGraph& graph,
    std::span<ComputeNode* const> order,
    std::size_t index)
{
  auto* node = order[index];
  profile.name = node->getName();
  profile.topologicalIndex = index;
  profile.dependencyCount = 0;
  profile.dependentCount = 0;
  profile.level = 0;
  profile.workerIndex = 0;
  profile.startTime = {};
  profile.endTime = {};
  profile.duration = {};

  for (const auto& edge : graph.getEdges()) {
    if (edge.to == node) {
      ++profile.dependencyCount;
    }
    if (edge.from == node) {
      ++profile.dependentCount;
    }
  }
}

} // namespace

//==============================================================================
bool ComputeExecutionProfile::isEmpty() const noexcept
{
  return nodes.empty();
}

//==============================================================================
double ComputeExecutionProfile::getAverageParallelism() const noexcept
{
  const auto wallCount = wallTime.count();
  if (wallCount <= 0) {
    return 0.0;
  }

  return static_cast<double>(totalNodeTime.count())
         / static_cast<double>(wallCount);
}

//==============================================================================
const ComputeNodeExecutionProfile* ComputeExecutionProfile::getNode(
    std::string_view name) const
{
  const auto it = std::ranges::find_if(
      nodes, [&](const auto& node) { return node.name == name; });

  if (it == nodes.end()) {
    return nullptr;
  }

  return &*it;
}

//==============================================================================
std::string ComputeExecutionProfile::toSummaryText() const
{
  if (nodes.empty()) {
    return "No compute execution profile captured.\n";
  }

  const double wallMs = toMilliseconds(wallTime);
  const auto sharePercent = [wallMs](double ms) {
    return wallMs > 0.0 ? 100.0 * ms / wallMs : 0.0;
  };

  std::vector<const ComputeNodeExecutionProfile*> ordered;
  ordered.reserve(nodes.size());
  for (const auto& node : nodes) {
    ordered.push_back(&node);
  }
  std::ranges::sort(ordered, [](const auto* lhs, const auto* rhs) {
    return lhs->duration > rhs->duration;
  });

  std::ostringstream out;
  out << "=== Compute Execution Profile ===\n";
  out << "workers=" << workerCount << "  wall=" << std::fixed
      << std::setprecision(3) << wallMs << " ms"
      << "  total_node=" << toMilliseconds(totalNodeTime) << " ms"
      << "  critical_path=" << toMilliseconds(criticalPathTime) << " ms\n";
  out << "max_parallelism=" << maxParallelism
      << "  average_parallelism=" << std::fixed << std::setprecision(2)
      << getAverageParallelism() << '\n';
  out << '\n';
  out << std::left << std::setw(34) << "Node" << std::right << std::setw(8)
      << "Worker" << std::setw(8) << "Level" << std::setw(8) << "Deps"
      << std::setw(14) << "Time (ms)" << std::setw(10) << "% wall" << '\n';
  out << std::string(82, '-') << '\n';
  for (const auto* node : ordered) {
    const double ms = toMilliseconds(node->duration);
    out << std::left << std::setw(34) << node->name << std::right
        << std::setw(8) << node->workerIndex << std::setw(8) << node->level
        << std::setw(8) << node->dependencyCount << std::setw(14) << std::fixed
        << std::setprecision(3) << ms << std::setw(9) << std::fixed
        << std::setprecision(1) << sharePercent(ms) << '%' << '\n';
  }

  return out.str();
}

//==============================================================================
ComputeExecutionProfiler::ComputeExecutionProfiler(
    const ComputeGraph& graph, std::size_t workerCount)
  : m_graph(graph), m_nodes(graph.getTopologicalOrder())
{
  m_profile.workerCount = workerCount;
  m_profile.nodes.reserve(m_nodes.size());
  m_nodeIndices.reserve(m_nodes.size());

  const auto levels = computeNodeLevels(m_graph, m_nodes);
  for (std::size_t i = 0; i < m_nodes.size(); ++i) {
    auto* node = m_nodes[i];
    m_nodeIndices.emplace(node, i);

    auto& nodeProfile = m_profile.nodes.emplace_back();
    nodeProfile.name = node->getName();
    nodeProfile.topologicalIndex = i;
    nodeProfile.dependencyCount = m_graph.getDependencies(*node).size();
    nodeProfile.dependentCount = m_graph.getDependents(*node).size();
    nodeProfile.level = levels[i];
  }
}

//==============================================================================
void ComputeExecutionProfiler::executeInline(
    const ComputeGraph& graph,
    std::size_t workerCount,
    ComputeExecutionProfile& profile,
    std::vector<ComputeExecutionProfile::Duration>& pathTimes)
{
  const auto order = graph.getTopologicalOrderView();

  profile.workerCount = workerCount;
  profile.wallTime = {};
  profile.totalNodeTime = {};
  profile.criticalPathTime = {};
  profile.maxParallelism = 0;
  profile.nodes.resize(order.size());

  for (std::size_t i = 0; i < order.size(); ++i) {
    resetNodeProfile(profile.nodes[i], graph, order, i);
    for (const auto& edge : graph.getEdges()) {
      if (edge.to != order[i]) {
        continue;
      }

      const auto dependencyIndex = findNodeIndex(order, edge.from);
      if (dependencyIndex < i) {
        profile.nodes[i].level = std::max(
            profile.nodes[i].level, profile.nodes[dependencyIndex].level + 1);
      }
    }
  }

  const auto start = Clock::now();
  for (std::size_t i = 0; i < order.size(); ++i) {
    auto& nodeProfile = profile.nodes[i];
    nodeProfile.workerIndex = 0;

    const auto nodeStart = Clock::now();
    nodeProfile.startTime = nodeStart - start;
    try {
      order[i]->execute();
    } catch (...) {
      const auto nodeEnd = Clock::now();
      nodeProfile.endTime = nodeEnd - start;
      nodeProfile.duration = nodeProfile.endTime - nodeProfile.startTime;
      throw;
    }

    const auto nodeEnd = Clock::now();
    nodeProfile.endTime = nodeEnd - start;
    nodeProfile.duration = nodeProfile.endTime - nodeProfile.startTime;
  }

  profile.wallTime = Clock::now() - start;

  pathTimes.assign(order.size(), Duration{});
  for (std::size_t i = 0; i < order.size(); ++i) {
    profile.totalNodeTime += profile.nodes[i].duration;

    Duration dependencyPath{};
    for (const auto& edge : graph.getEdges()) {
      if (edge.to != order[i]) {
        continue;
      }

      const auto dependencyIndex = findNodeIndex(order, edge.from);
      if (dependencyIndex < i) {
        dependencyPath = std::max(dependencyPath, pathTimes[dependencyIndex]);
      }
    }

    pathTimes[i] = dependencyPath + profile.nodes[i].duration;
    profile.criticalPathTime = std::max(profile.criticalPathTime, pathTimes[i]);
  }

  profile.maxParallelism = profile.nodes.empty() ? 0u : 1u;
}

//==============================================================================
void ComputeExecutionProfiler::start()
{
  m_startTime = Clock::now();
  m_started = true;
  m_finished = false;
}

//==============================================================================
void ComputeExecutionProfiler::executeNode(ComputeNode& node)
{
  if (!m_started) {
    start();
  }

  const auto nodeIndex = getNodeIndex(node);
  auto& nodeProfile = m_profile.nodes[nodeIndex];
  nodeProfile.workerIndex = getWorkerIndex();
  const auto start = Clock::now();
  nodeProfile.startTime = start - m_startTime;

  try {
    node.execute();
  } catch (...) {
    finishNode(nodeIndex, Clock::now());
    throw;
  }

  finishNode(nodeIndex, Clock::now());
}

//==============================================================================
ComputeExecutionProfile ComputeExecutionProfiler::finish()
{
  if (!m_started) {
    start();
  }

  if (!m_finished) {
    m_profile.wallTime = Clock::now() - m_startTime;
    finalizeProfile();
    m_finished = true;
  }

  return m_profile;
}

//==============================================================================
std::size_t ComputeExecutionProfiler::getNodeIndex(
    const ComputeNode& node) const
{
  const auto it = m_nodeIndices.find(&node);
  DART_SIMULATION_THROW_T_IF(
      it == m_nodeIndices.end(),
      InvalidArgumentException,
      "Compute node '{}' does not belong to this profiled graph",
      node.getName());

  return it->second;
}

//==============================================================================
std::size_t ComputeExecutionProfiler::getWorkerIndex()
{
  const auto threadId = std::this_thread::get_id();
  std::scoped_lock lock(m_workerMutex);

  const auto it = m_workerIndices.find(threadId);
  if (it != m_workerIndices.end()) {
    return it->second;
  }

  const auto workerIndex = m_workerIndices.size();
  m_workerIndices.emplace(threadId, workerIndex);
  return workerIndex;
}

//==============================================================================
void ComputeExecutionProfiler::finishNode(
    std::size_t nodeIndex, Clock::time_point end)
{
  auto& nodeProfile = m_profile.nodes[nodeIndex];
  nodeProfile.endTime = end - m_startTime;
  nodeProfile.duration = nodeProfile.endTime - nodeProfile.startTime;
}

//==============================================================================
void ComputeExecutionProfiler::finalizeProfile()
{
  m_profile.totalNodeTime = Duration{};
  for (const auto& node : m_profile.nodes) {
    m_profile.totalNodeTime += node.duration;
  }

  std::vector<Duration> pathTimes(m_profile.nodes.size(), Duration{});
  for (std::size_t i = 0; i < m_nodes.size(); ++i) {
    Duration dependencyPath{};
    for (auto* dependency : m_graph.getDependencies(*m_nodes[i])) {
      dependencyPath
          = std::max(dependencyPath, pathTimes[getNodeIndex(*dependency)]);
    }

    pathTimes[i] = dependencyPath + m_profile.nodes[i].duration;
    m_profile.criticalPathTime
        = std::max(m_profile.criticalPathTime, pathTimes[i]);
  }

  m_profile.maxParallelism = computeMaxParallelism(m_profile.nodes);
}

} // namespace dart::simulation::compute
