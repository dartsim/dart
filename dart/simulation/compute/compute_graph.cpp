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

#include "dart/simulation/compute/compute_graph.hpp"

#include "dart/simulation/common/exceptions.hpp"

#include <algorithm>
#include <functional>
#include <iterator>
#include <new>
#include <queue>
#include <unordered_map>
#include <utility>

#include <cstdint>

namespace dart::simulation::compute {

namespace {

using NodePointerAllocator = dart::common::StlAllocator<ComputeNode*>;
using NodePointerVector = std::vector<ComputeNode*, NodePointerAllocator>;
using IndexAllocator = dart::common::StlAllocator<std::size_t>;
using IndexVector = std::vector<std::size_t, IndexAllocator>;
using IndexVectorAllocator = dart::common::StlAllocator<IndexVector>;
using IndexVectorVector = std::vector<IndexVector, IndexVectorAllocator>;
using ByteAllocator = dart::common::StlAllocator<std::uint8_t>;
using ByteVector = std::vector<std::uint8_t, ByteAllocator>;
using NodeIndexLookupAllocator = dart::common::StlAllocator<
    std::pair<const ComputeNode* const, std::size_t>>;
using NodeIndexLookup = std::unordered_map<
    const ComputeNode*,
    std::size_t,
    std::hash<const ComputeNode*>,
    std::equal_to<const ComputeNode*>,
    NodeIndexLookupAllocator>;

//==============================================================================
template <typename Nodes>
NodePointerVector collectNodePointers(
    const Nodes& nodes, dart::common::MemoryAllocator& allocator)
{
  NodePointerVector result(NodePointerAllocator{allocator});
  result.reserve(nodes.size());
  for (const auto& node : nodes) {
    result.push_back(node.get());
  }
  return result;
}

//==============================================================================
NodeIndexLookup makeNodeIndexLookup(
    dart::common::MemoryAllocator& allocator, std::size_t capacity)
{
  NodeIndexLookup lookup(
      0,
      std::hash<const ComputeNode*>{},
      std::equal_to<const ComputeNode*>{},
      NodeIndexLookupAllocator{allocator});
  lookup.reserve(capacity);
  return lookup;
}

//==============================================================================
IndexVectorVector makeIndexAdjacency(
    dart::common::MemoryAllocator& allocator, std::size_t count)
{
  IndexVectorVector adjacency(IndexVectorAllocator{allocator});
  adjacency.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    adjacency.emplace_back(IndexAllocator{allocator});
  }
  return adjacency;
}

} // namespace

//==============================================================================
ComputeGraph::ComputeGraph()
  : ComputeGraph(dart::common::MemoryAllocator::GetDefault())
{
}

//==============================================================================
ComputeGraph::ComputeGraph(dart::common::MemoryAllocator& allocator)
  : m_allocator(&allocator),
    m_nodes(ComputeNodePtrAllocator{allocator}),
    m_edges(EdgeAllocator{allocator}),
    m_nodesByName(
        0,
        std::hash<std::string>{},
        std::equal_to<std::string>{},
        NodeNameLookupAllocator{allocator}),
    m_topologicalOrderCache(TopologicalOrderAllocator{allocator})
{
}

//==============================================================================
void ComputeGraph::ComputeNodeDeleter::operator()(
    ComputeNode* node) const noexcept
{
  if (node == nullptr) {
    return;
  }
  auto& targetAllocator = allocator != nullptr
                              ? *allocator
                              : dart::common::MemoryAllocator::GetDefault();
  targetAllocator.destroy(node);
}

//==============================================================================
ComputeNode& ComputeGraph::addNode(
    std::string_view name,
    ComputeNode::ExecuteFn fn,
    ComputeStageMetadata metadata)
{
  DART_SIMULATION_THROW_T_IF(
      name.empty(),
      InvalidArgumentException,
      "Compute node names must not be empty");

  const auto nameString = std::string(name);
  DART_SIMULATION_THROW_T_IF(
      m_nodesByName.contains(nameString),
      InvalidArgumentException,
      "A compute node named '{}' already exists",
      nameString);

  auto node = ComputeNodePtr(
      m_allocator->construct<ComputeNode>(
          nameString, std::move(fn), std::move(metadata)),
      ComputeNodeDeleter{m_allocator});
  if (node == nullptr) {
    throw std::bad_alloc();
  }
  auto* nodePtr = node.get();
  m_nodes.push_back(std::move(node));
  try {
    m_nodesByName.emplace(nameString, nodePtr);
  } catch (...) {
    m_nodes.pop_back();
    throw;
  }
  invalidateTraversalCache();
  return *nodePtr;
}

//==============================================================================
void ComputeGraph::addDependency(ComputeNode& from, ComputeNode& to)
{
  DART_SIMULATION_THROW_T_IF(
      !ownsNode(from) || !ownsNode(to),
      InvalidArgumentException,
      "Both compute nodes must belong to this graph");

  DART_SIMULATION_THROW_T_IF(
      &from == &to,
      InvalidOperationException,
      "Compute node '{}' cannot depend on itself",
      from.getName());

  const auto duplicate = std::ranges::any_of(m_edges, [&](const auto& edge) {
    return edge.from == &from && edge.to == &to;
  });
  if (duplicate) {
    return;
  }

  DART_SIMULATION_THROW_T_IF(
      wouldCreateCycle(from, to),
      InvalidOperationException,
      "Adding dependency '{}' -> '{}' would create a cycle",
      from.getName(),
      to.getName());

  m_edges.push_back({&from, &to});
  invalidateTraversalCache();
}

//==============================================================================
ComputeNode* ComputeGraph::getNode(std::string_view name) const
{
  const auto it = m_nodesByName.find(std::string(name));
  if (it == m_nodesByName.end()) {
    return nullptr;
  }

  return it->second;
}

//==============================================================================
std::vector<ComputeNode*> ComputeGraph::getNodes() const
{
  std::vector<ComputeNode*> nodes;
  nodes.reserve(m_nodes.size());
  for (const auto& node : m_nodes) {
    nodes.push_back(node.get());
  }
  return nodes;
}

//==============================================================================
std::vector<ComputeNode*> ComputeGraph::getTopologicalOrder() const
{
  const auto order = getTopologicalOrderView();
  return std::vector<ComputeNode*>{order.begin(), order.end()};
}

//==============================================================================
std::span<ComputeNode* const> ComputeGraph::getTopologicalOrderView() const
{
  if (!m_topologicalOrderCacheValid) {
    m_topologicalOrderCache = buildTopologicalOrder(true);
    m_topologicalOrderCacheValid = true;
  }
  return std::span<ComputeNode* const>{
      m_topologicalOrderCache.data(), m_topologicalOrderCache.size()};
}

//==============================================================================
std::vector<std::vector<ComputeNode*>> ComputeGraph::getParallelLevels() const
{
  const auto order = getTopologicalOrderView();
  if (order.empty()) {
    return {};
  }

  std::unordered_map<const ComputeNode*, std::size_t> nodeLevels;
  nodeLevels.reserve(order.size());

  std::size_t maxLevel = 0;
  for (auto* node : order) {
    std::size_t level = 0;
    for (auto* dependency : getDependencies(*node)) {
      level = std::max(level, nodeLevels.at(dependency) + 1);
    }

    nodeLevels.emplace(node, level);
    maxLevel = std::max(maxLevel, level);
  }

  std::vector<std::vector<ComputeNode*>> levels(maxLevel + 1);
  for (auto* node : order) {
    levels[nodeLevels.at(node)].push_back(node);
  }

  return levels;
}

//==============================================================================
std::vector<ComputeNode*> ComputeGraph::getDependencies(
    const ComputeNode& node) const
{
  DART_SIMULATION_THROW_T_IF(
      !ownsNode(node),
      InvalidArgumentException,
      "Compute node '{}' does not belong to this graph",
      node.getName());

  std::vector<ComputeNode*> dependencies;
  for (const auto& edge : m_edges) {
    if (edge.to == &node) {
      dependencies.push_back(edge.from);
    }
  }
  return dependencies;
}

//==============================================================================
std::vector<ComputeNode*> ComputeGraph::getDependents(
    const ComputeNode& node) const
{
  DART_SIMULATION_THROW_T_IF(
      !ownsNode(node),
      InvalidArgumentException,
      "Compute node '{}' does not belong to this graph",
      node.getName());

  std::vector<ComputeNode*> dependents;
  for (const auto& edge : m_edges) {
    if (edge.from == &node) {
      dependents.push_back(edge.to);
    }
  }
  return dependents;
}

//==============================================================================
bool ComputeGraph::validate() const
{
  for (const auto& node : m_nodes) {
    if (!node || !node->isValid()) {
      return false;
    }
  }

  for (const auto& edge : m_edges) {
    if (!edge.from || !edge.to || !ownsNode(*edge.from)
        || !ownsNode(*edge.to)) {
      return false;
    }
  }

  return buildTopologicalOrder(false).size() == m_nodes.size();
}

//==============================================================================
std::vector<ComputeResourceHazard> ComputeGraph::findResourceHazards() const
{
  std::vector<ComputeResourceHazard> hazards;
  NodePointerVector nodes = collectNodePointers(m_nodes, *m_allocator);
  const auto count = nodes.size();
  if (count < 2) {
    return hazards;
  }

  NodeIndexLookup indexByNode = makeNodeIndexLookup(*m_allocator, count);
  for (std::size_t i = 0; i < count; ++i) {
    indexByNode.emplace(nodes[i], i);
  }

  IndexVectorVector dependents = makeIndexAdjacency(*m_allocator, count);
  for (const auto& edge : m_edges) {
    dependents[indexByNode.at(edge.from)].push_back(indexByNode.at(edge.to));
  }

  // reachable[i][j] is true when node j runs strictly after node i, i.e. an
  // explicit dependency path orders them. Unordered pairs may run concurrently.
  // The precomputed index map and adjacency keep this O(N * (N + E)) instead of
  // rescanning all edges and linear-searching for each visited node.
  ByteVector reachable(count * count, 0u, ByteAllocator{*m_allocator});
  const auto isReachable = [&](std::size_t from, std::size_t to) -> auto& {
    return reachable[from * count + to];
  };
  IndexVector stack(IndexAllocator{*m_allocator});
  stack.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    stack.assign(dependents[i].begin(), dependents[i].end());
    while (!stack.empty()) {
      const auto j = stack.back();
      stack.pop_back();
      if (isReachable(i, j) != 0u) {
        continue;
      }
      isReachable(i, j) = 1u;
      for (const auto to : dependents[j]) {
        stack.push_back(to);
      }
    }
  }

  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      if (isReachable(i, j) != 0u || isReachable(j, i) != 0u) {
        continue;
      }

      for (const auto& first : nodes[i]->getMetadata().resources) {
        for (const auto& second : nodes[j]->getMetadata().resources) {
          if (first.resource != second.resource
              || !accessesConflict(first.mode, second.mode)) {
            continue;
          }
          hazards.push_back(
              ComputeResourceHazard{
                  nodes[i],
                  nodes[j],
                  std::string(first.resource.begin(), first.resource.end()),
                  first.mode,
                  second.mode});
        }
      }
    }
  }

  return hazards;
}

//==============================================================================
void ComputeGraph::clear()
{
  m_edges.clear();
  m_nodesByName.clear();
  m_nodes.clear();
  invalidateTraversalCache();
}

//==============================================================================
void ComputeGraph::invalidateTraversalCache() noexcept
{
  m_topologicalOrderCache.clear();
  m_topologicalOrderCacheValid = false;
}

//==============================================================================
bool ComputeGraph::ownsNode(const ComputeNode& node) const
{
  return std::ranges::any_of(
      m_nodes, [&](const auto& ownedNode) { return ownedNode.get() == &node; });
}

//==============================================================================
bool ComputeGraph::wouldCreateCycle(
    const ComputeNode& from, const ComputeNode& to) const
{
  NodePointerVector stack(NodePointerAllocator{*m_allocator});
  stack.push_back(const_cast<ComputeNode*>(&to));
  NodePointerVector visited(NodePointerAllocator{*m_allocator});
  visited.reserve(m_nodes.size());

  while (!stack.empty()) {
    auto* current = stack.back();
    stack.pop_back();

    if (current == &from) {
      return true;
    }

    if (std::ranges::find(visited, current) != visited.end()) {
      continue;
    }
    visited.push_back(current);

    for (const auto& edge : m_edges) {
      if (edge.from == current) {
        stack.push_back(edge.to);
      }
    }
  }

  return false;
}

//==============================================================================
ComputeGraph::TopologicalOrderVector ComputeGraph::buildTopologicalOrder(
    bool throwOnCycle) const
{
  NodePointerVector nodes = collectNodePointers(m_nodes, *m_allocator);
  const auto count = nodes.size();

  NodeIndexLookup indexByNode = makeNodeIndexLookup(*m_allocator, count);
  for (std::size_t i = 0; i < count; ++i) {
    indexByNode.emplace(nodes[i], i);
  }

  IndexVectorVector dependents = makeIndexAdjacency(*m_allocator, count);
  IndexVector indegree(count, 0, IndexAllocator{*m_allocator});
  for (const auto& edge : m_edges) {
    if (!edge.from || !edge.to || !ownsNode(*edge.from)
        || !ownsNode(*edge.to)) {
      if (throwOnCycle) {
        DART_SIMULATION_THROW_T(
            InvalidOperationException,
            "Compute graph contains an edge with nodes outside the graph");
      }
      return TopologicalOrderVector(TopologicalOrderAllocator{*m_allocator});
    }
    dependents[indexByNode.at(edge.from)].push_back(indexByNode.at(edge.to));
    ++indegree[indexByNode.at(edge.to)];
  }

  // Kahn's algorithm with a min-heap on node index so ties break by node
  // construction order, matching the previous linear-scan behavior in
  // O((N + E) log N) instead of O(N^2).
  std::priority_queue<std::size_t, IndexVector, std::greater<>> ready{
      std::greater<>{}, IndexVector(IndexAllocator{*m_allocator})};
  for (std::size_t i = 0; i < count; ++i) {
    if (indegree[i] == 0) {
      ready.push(i);
    }
  }

  TopologicalOrderVector order(TopologicalOrderAllocator{*m_allocator});
  order.reserve(count);
  while (!ready.empty()) {
    const auto i = ready.top();
    ready.pop();
    order.push_back(nodes[i]);
    for (const auto to : dependents[i]) {
      if (--indegree[to] == 0) {
        ready.push(to);
      }
    }
  }

  if (order.size() != count && throwOnCycle) {
    DART_SIMULATION_THROW_T(
        InvalidOperationException, "Compute graph contains a cycle");
  }

  return order;
}

} // namespace dart::simulation::compute
