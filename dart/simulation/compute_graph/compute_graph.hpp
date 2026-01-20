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

#include <dart/simulation/compute_graph/compute_node.hpp>

#include <memory>
#include <span>
#include <unordered_map>
#include <vector>

namespace dart::simulation {

/// Directed acyclic graph (DAG) of compute nodes with explicit dependencies.
/// Thread-safety: Graph construction is NOT thread-safe. Build completely
/// before executing.
class DART_API ComputeGraph
{
public:
  ComputeGraph() = default;
  ~ComputeGraph() = default;

  ComputeGraph(const ComputeGraph&) = delete;
  ComputeGraph& operator=(const ComputeGraph&) = delete;
  ComputeGraph(ComputeGraph&&) = default;
  ComputeGraph& operator=(ComputeGraph&&) = default;

  NodeId addNode(ComputeNodePtr node);
  NodeId addNode(LambdaNode::Callable func, std::string_view name = "lambda");

  [[nodiscard]] ComputeNodePtr getNode(NodeId id) const;
  [[nodiscard]] std::span<const ComputeNodePtr> getNodes() const;
  [[nodiscard]] std::size_t getNodeCount() const;

  /// Add dependency: predecessor must complete before successor starts
  /// @return true if edge added, false if would create cycle
  bool addEdge(NodeId predecessor, NodeId successor);
  bool addEdge(
      const ComputeNodePtr& predecessor, const ComputeNodePtr& successor);

  [[nodiscard]] std::span<const NodeId> getPredecessors(NodeId nodeId) const;
  [[nodiscard]] std::span<const NodeId> getSuccessors(NodeId nodeId) const;

  [[nodiscard]] bool isValid() const;

  /// Finalize graph, computing topological order. Must call before execution.
  /// @return false if graph has cycles
  bool finalize();

  [[nodiscard]] bool isFinalized() const;
  [[nodiscard]] std::span<const NodeId> getTopologicalOrder() const;

  void clear();
  void debugPrint() const;

private:
  bool detectCycle(NodeId from, NodeId to) const;
  void computeTopologicalOrder();

  NodeId mNextId{1};
  bool mFinalized{false};

  std::vector<ComputeNodePtr> mNodes;
  std::unordered_map<NodeId, ComputeNodePtr> mNodeMap;
  std::unordered_map<NodeId, std::vector<NodeId>> mPredecessors;
  std::unordered_map<NodeId, std::vector<NodeId>> mSuccessors;
  std::vector<NodeId> mTopologicalOrder;
};

} // namespace dart::simulation
