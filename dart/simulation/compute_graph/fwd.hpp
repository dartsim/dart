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

#include <dart/Export.hpp>

#include <functional>
#include <memory>
#include <string>

#include <cstdint>

namespace dart::simulation {

// Forward declarations
class ComputeNode;
class ComputeGraph;
class GraphExecutor;
class SequentialExecutor;
class TaskflowExecutor;

// Smart pointer types
using ComputeNodePtr = std::shared_ptr<ComputeNode>;
using ComputeGraphPtr = std::shared_ptr<ComputeGraph>;
using GraphExecutorPtr = std::shared_ptr<GraphExecutor>;

/// Unique identifier for nodes within a graph
using NodeId = std::uint64_t;

/// Invalid node ID constant
inline constexpr NodeId kInvalidNodeId = 0;

/// Execution context passed to nodes during execution
///
/// This structure provides all necessary information for a compute node
/// to execute its computation. It is passed by const reference to ensure
/// thread-safety during parallel execution.
struct DART_API ExecutionContext
{
  /// Time step for the simulation (seconds)
  double timeStep{0.001};

  /// Current simulation time (seconds)
  double currentTime{0.0};

  /// Current frame number
  std::uint64_t frameNumber{0};

  /// Whether to reset commands after integration
  bool resetCommand{true};

  /// Opaque pointer to World for nodes that need full access
  /// This allows gradual migration - nodes can access World directly initially
  void* worldPtr{nullptr};
};

/// Configuration for graph execution
struct DART_API ExecutorConfig
{
  /// Number of worker threads
  /// - 0: Use hardware concurrency (std::thread::hardware_concurrency())
  /// - 1: Single-threaded execution (use SequentialExecutor internally)
  /// - N: Use N worker threads
  std::size_t numWorkers{0};

  /// Force sequential execution regardless of numWorkers
  /// Useful for debugging and determinism verification
  bool forceSequential{false};

  /// Enable profiling/tracing of node execution
  bool enableProfiling{false};

  /// Optional callback invoked when a node completes
  /// @param nodeId The ID of the completed node
  /// @param elapsedMs Execution time in milliseconds
  std::function<void(NodeId nodeId, double elapsedMs)> onNodeComplete;
};

/// Threading policy for compute graph execution
enum class ThreadingPolicy
{
  /// Single-threaded execution, deterministic ordering
  Sequential,

  /// Automatic thread count (hardware_concurrency)
  Auto,

  /// Static thread count specified via ExecutorConfig::numWorkers
  Static
};

} // namespace dart::simulation
