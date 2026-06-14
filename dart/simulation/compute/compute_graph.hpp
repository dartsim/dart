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

#include <dart/simulation/compute/compute_node.hpp>
#include <dart/simulation/export.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <cstddef>

namespace dart::simulation::compute {

/// Dependency edge from @c from to @c to.
///
/// The dependent node may execute only after its dependency completes.
struct DART_SIMULATION_API ComputeEdge
{
  ComputeNode* from;
  ComputeNode* to;
};

/// A descriptive resource-access hazard between two unordered nodes.
///
/// Reported when two nodes that are not ordered by an explicit dependency
/// declare conflicting accesses to the same resource. This does not mutate the
/// graph; explicit dependencies remain the correctness source of truth.
struct DART_SIMULATION_API ComputeResourceHazard
{
  const ComputeNode* first;
  const ComputeNode* second;
  std::string resource;
  ComputeAccessMode firstMode;
  ComputeAccessMode secondMode;
};

/// Directed acyclic graph for experimental compute pipeline work.
class DART_SIMULATION_API ComputeGraph
{
public:
  ComputeGraph();
  explicit ComputeGraph(dart::common::MemoryAllocator& allocator);
  ~ComputeGraph() = default;

  ComputeGraph(const ComputeGraph&) = delete;
  ComputeGraph& operator=(const ComputeGraph&) = delete;
  ComputeGraph(ComputeGraph&&) noexcept = default;
  ComputeGraph& operator=(ComputeGraph&&) noexcept = default;

  ComputeNode& addNode(
      std::string_view name,
      ComputeNode::ExecuteFn fn,
      ComputeStageMetadata metadata = {});
  void addDependency(ComputeNode& from, ComputeNode& to);

  [[nodiscard]] ComputeNode* getNode(std::string_view name) const;
  [[nodiscard]] std::vector<ComputeNode*> getNodes() const;
  [[nodiscard]] std::vector<ComputeNode*> getTopologicalOrder() const;
  [[nodiscard]] std::span<ComputeNode* const> getTopologicalOrderView() const;
  [[nodiscard]] std::vector<std::vector<ComputeNode*>> getParallelLevels()
      const;

  [[nodiscard]] std::span<const ComputeEdge> getEdges() const noexcept
  {
    return std::span<const ComputeEdge>{m_edges.data(), m_edges.size()};
  }

  [[nodiscard]] std::vector<ComputeNode*> getDependencies(
      const ComputeNode& node) const;
  [[nodiscard]] std::vector<ComputeNode*> getDependents(
      const ComputeNode& node) const;

  [[nodiscard]] std::size_t getNodeCount() const noexcept
  {
    return m_nodes.size();
  }

  [[nodiscard]] std::size_t getEdgeCount() const noexcept
  {
    return m_edges.size();
  }

  [[nodiscard]] bool isEmpty() const noexcept
  {
    return m_nodes.empty();
  }

  [[nodiscard]] bool validate() const;

  /// Reports conservative resource-access hazards between unordered nodes.
  ///
  /// Two nodes conflict when they are not connected by any dependency path and
  /// declare conflicting accesses to the same resource (see @c
  /// accessesConflict). This is diagnostic only and never mutates the graph.
  [[nodiscard]] std::vector<ComputeResourceHazard> findResourceHazards() const;

  void clear();

private:
  struct ComputeNodeDeleter
  {
    dart::common::MemoryAllocator* allocator = nullptr;
    void operator()(ComputeNode* node) const noexcept;
  };

  using ComputeNodePtr = std::unique_ptr<ComputeNode, ComputeNodeDeleter>;
  using ComputeNodePtrAllocator = dart::common::StlAllocator<ComputeNodePtr>;
  using EdgeAllocator = dart::common::StlAllocator<ComputeEdge>;
  using TopologicalOrderAllocator = dart::common::StlAllocator<ComputeNode*>;
  using EdgeVector = std::vector<ComputeEdge, EdgeAllocator>;
  using TopologicalOrderVector
      = std::vector<ComputeNode*, TopologicalOrderAllocator>;
  using NodeNameLookupAllocator
      = dart::common::StlAllocator<std::pair<const std::string, ComputeNode*>>;

  void invalidateTraversalCache() noexcept;

  [[nodiscard]] bool ownsNode(const ComputeNode& node) const;
  [[nodiscard]] bool wouldCreateCycle(
      const ComputeNode& from, const ComputeNode& to) const;
  [[nodiscard]] TopologicalOrderVector buildTopologicalOrder(
      bool throwOnCycle) const;

  dart::common::MemoryAllocator* m_allocator = nullptr;
  std::vector<ComputeNodePtr, ComputeNodePtrAllocator> m_nodes;
  EdgeVector m_edges;
  std::unordered_map<
      std::string,
      ComputeNode*,
      std::hash<std::string>,
      std::equal_to<std::string>,
      NodeNameLookupAllocator>
      m_nodesByName;
  mutable TopologicalOrderVector m_topologicalOrderCache;
  mutable bool m_topologicalOrderCacheValid = false;
};

} // namespace dart::simulation::compute
