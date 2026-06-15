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

#pragma once

#include <dart/simulation/export.hpp>

#include <chrono>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include <cstddef>

namespace dart::simulation::compute {

class ComputeGraph;
class ComputeNode;

/// Runtime timing and scheduling data for one compute node execution.
struct DART_SIMULATION_API ComputeNodeExecutionProfile
{
  using Duration = std::chrono::steady_clock::duration;

  /// Node name at the time profiling started.
  std::string name;

  /// Index in the graph's topological order.
  std::size_t topologicalIndex = 0;

  /// Number of incoming dependencies.
  std::size_t dependencyCount = 0;

  /// Number of outgoing dependents.
  std::size_t dependentCount = 0;

  /// Longest dependency depth from any source node.
  std::size_t level = 0;

  /// Compact index for the worker thread that ran this node.
  std::size_t workerIndex = 0;

  /// Node start time relative to the beginning of graph execution.
  Duration startTime{};

  /// Node finish time relative to the beginning of graph execution.
  Duration endTime{};

  /// Time spent inside this node's callable.
  Duration duration{};
};

/// Runtime profile for one compute graph execution.
struct DART_SIMULATION_API ComputeExecutionProfile
{
  using Duration = ComputeNodeExecutionProfile::Duration;

  /// Number of workers exposed by the executor.
  std::size_t workerCount = 0;

  /// End-to-end graph execution time, including scheduling overhead.
  Duration wallTime{};

  /// Sum of all node callable durations.
  Duration totalNodeTime{};

  /// Longest measured dependency path through the graph.
  Duration criticalPathTime{};

  /// Largest number of node callables observed running at the same time.
  std::size_t maxParallelism = 0;

  /// Per-node execution records, sorted by topological index.
  std::vector<ComputeNodeExecutionProfile> nodes;

  [[nodiscard]] bool isEmpty() const noexcept;
  [[nodiscard]] double getAverageParallelism() const noexcept;
  [[nodiscard]] const ComputeNodeExecutionProfile* getNode(
      std::string_view name) const;
  [[nodiscard]] std::string toSummaryText() const;
};

/// Helper used by executors to collect one profiled graph execution.
///
/// Custom executors can use this helper to report profiles in the same format
/// as the built-in sequential and parallel executors.
class DART_SIMULATION_API ComputeExecutionProfiler
{
public:
  explicit ComputeExecutionProfiler(
      const ComputeGraph& graph, std::size_t workerCount);

  static void executeInline(
      const ComputeGraph& graph,
      std::size_t workerCount,
      ComputeExecutionProfile& profile,
      std::vector<ComputeExecutionProfile::Duration>& pathTimes);

  void start();
  void executeNode(ComputeNode& node);
  [[nodiscard]] ComputeExecutionProfile finish();

private:
  using Clock = std::chrono::steady_clock;

  [[nodiscard]] std::size_t getNodeIndex(const ComputeNode& node) const;
  [[nodiscard]] std::size_t getWorkerIndex();
  void finishNode(std::size_t nodeIndex, Clock::time_point end);
  void finalizeProfile();

  const ComputeGraph& m_graph;
  std::vector<ComputeNode*> m_nodes;
  std::unordered_map<const ComputeNode*, std::size_t> m_nodeIndices;
  std::unordered_map<std::thread::id, std::size_t> m_workerIndices;
  std::mutex m_workerMutex;
  Clock::time_point m_startTime;
  bool m_started = false;
  bool m_finished = false;
  ComputeExecutionProfile m_profile;
};

} // namespace dart::simulation::compute
