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

#include <dart/simulation/experimental/compute/compute_node.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::compute {

/// @brief Edge representing a dependency between nodes.
///
/// If an edge exists from node A to node B, then A must complete
/// execution before B can start.
struct DART_EXPERIMENTAL_API ComputeEdge
{
  ComputeNode* from; ///< Node that must complete first
  ComputeNode* to;   ///< Node that depends on 'from'
};

/// @brief A directed acyclic graph representing a computation pipeline.
///
/// ComputeGraph manages a collection of ComputeNodes and their dependencies.
/// It provides topological ordering for sequential execution and validates
/// that the graph is acyclic.
///
/// Usage:
/// @code
/// ComputeGraph graph;
/// auto& a = graph.addNode("A", []() { /* work */ });
/// auto& b = graph.addNode("B", []() { /* work */ });
/// graph.addDependency(a, b);  // A must complete before B
///
/// for (auto* node : graph.getTopologicalOrder()) {
///   node->execute();
/// }
/// @endcode
///
/// @note Once nodes are added, they should not be removed or moved.
/// The graph owns all nodes via unique_ptr.
class DART_EXPERIMENTAL_API ComputeGraph
{
public:
  ComputeGraph() = default;
  ~ComputeGraph() = default;

  /// Non-copyable
  ComputeGraph(const ComputeGraph&) = delete;
  ComputeGraph& operator=(const ComputeGraph&) = delete;

  /// Movable
  ComputeGraph(ComputeGraph&&) noexcept = default;
  ComputeGraph& operator=(ComputeGraph&&) noexcept = default;

  /// @brief Adds a node to the graph.
  /// @param name Unique name for the node
  /// @param fn Work function to execute
  /// @return Reference to the created node
  /// @throws InvalidArgumentException if a node with the same name exists
  ComputeNode& addNode(std::string name, ComputeNode::ExecuteFn fn);

  /// @brief Adds a dependency edge: 'from' must complete before 'to'.
  /// @param from Node that must complete first
  /// @param to Node that depends on 'from'
  /// @throws InvalidArgumentException if either node is not in this graph
  /// @throws InvalidOperationException if adding this edge would create a cycle
  void addDependency(ComputeNode& from, ComputeNode& to);

  /// @brief Gets a node by name.
  /// @param name The node name
  /// @return Pointer to the node, or nullptr if not found
  [[nodiscard]] ComputeNode* getNode(std::string_view name) const;

  /// @brief Gets all nodes in topological order.
  ///
  /// The returned order guarantees that for every edge (A, B), node A
  /// appears before node B in the sequence.
  ///
  /// @return Vector of node pointers in execution order
  /// @note The order is deterministic for deterministic graph construction
  [[nodiscard]] std::vector<ComputeNode*> getTopologicalOrder() const;

  /// @brief Gets all nodes (unordered).
  /// @return Vector of pointers to all nodes
  [[nodiscard]] std::vector<ComputeNode*> getNodes() const;

  /// @brief Gets all edges.
  /// @return Vector of all dependency edges
  [[nodiscard]] const std::vector<ComputeEdge>& getEdges() const noexcept
  {
    return m_edges;
  }

  /// @brief Gets the number of nodes.
  /// @return Node count
  [[nodiscard]] std::size_t getNodeCount() const noexcept
  {
    return m_nodes.size();
  }

  /// @brief Gets the number of edges.
  /// @return Edge count
  [[nodiscard]] std::size_t getEdgeCount() const noexcept
  {
    return m_edges.size();
  }

  /// @brief Checks if the graph is empty.
  /// @return True if there are no nodes
  [[nodiscard]] bool isEmpty() const noexcept
  {
    return m_nodes.empty();
  }

  /// @brief Validates the graph structure.
  ///
  /// Checks that:
  /// - All nodes have valid work functions
  /// - All edge endpoints are in this graph
  /// - The graph is acyclic
  ///
  /// @return True if valid, false otherwise
  [[nodiscard]] bool validate() const;

  /// @brief Clears all nodes and edges.
  void clear();

  /// @brief Gets the dependencies (predecessors) of a node.
  /// @param node The node to query
  /// @return Vector of nodes that must complete before this node
  [[nodiscard]] std::vector<ComputeNode*> getDependencies(
      const ComputeNode& node) const;

  /// @brief Gets the dependents (successors) of a node.
  /// @param node The node to query
  /// @return Vector of nodes that depend on this node
  [[nodiscard]] std::vector<ComputeNode*> getDependents(
      const ComputeNode& node) const;

private:
  /// Checks if adding edge (from, to) would create a cycle
  [[nodiscard]] bool wouldCreateCycle(
      const ComputeNode& from, const ComputeNode& to) const;

  std::vector<std::unique_ptr<ComputeNode>> m_nodes;
  std::vector<ComputeEdge> m_edges;
  std::unordered_map<std::string, ComputeNode*> m_nodesByName;
};

} // namespace dart::simulation::experimental::compute
