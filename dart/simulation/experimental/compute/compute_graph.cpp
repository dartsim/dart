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

#include "dart/simulation/experimental/compute/compute_graph.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

namespace dart::simulation::experimental::compute {

namespace {

//==============================================================================
bool containsPointer(
    const std::vector<ComputeNode*>& nodes, const ComputeNode* node)
{
  return std::ranges::find(nodes, node) != nodes.end();
}

} // namespace

//==============================================================================
ComputeNode& ComputeGraph::addNode(
    std::string_view name,
    ComputeNode::ExecuteFn fn,
    ComputeStageMetadata metadata)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      name.empty(),
      InvalidArgumentException,
      "Compute node names must not be empty");

  const auto nameString = std::string(name);
  DART_EXPERIMENTAL_THROW_T_IF(
      m_nodesByName.contains(nameString),
      InvalidArgumentException,
      "A compute node named '{}' already exists",
      nameString);

  auto node
      = std::make_unique<ComputeNode>(nameString, std::move(fn), metadata);
  auto* nodePtr = node.get();
  m_nodes.push_back(std::move(node));
  m_nodesByName.emplace(nameString, nodePtr);
  return *nodePtr;
}

//==============================================================================
void ComputeGraph::addDependency(ComputeNode& from, ComputeNode& to)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !ownsNode(from) || !ownsNode(to),
      InvalidArgumentException,
      "Both compute nodes must belong to this graph");

  DART_EXPERIMENTAL_THROW_T_IF(
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

  DART_EXPERIMENTAL_THROW_T_IF(
      wouldCreateCycle(from, to),
      InvalidOperationException,
      "Adding dependency '{}' -> '{}' would create a cycle",
      from.getName(),
      to.getName());

  m_edges.push_back({&from, &to});
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
  return buildTopologicalOrder(true);
}

//==============================================================================
std::vector<std::vector<ComputeNode*>> ComputeGraph::getParallelLevels() const
{
  const auto order = getTopologicalOrder();
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
  DART_EXPERIMENTAL_THROW_T_IF(
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
  DART_EXPERIMENTAL_THROW_T_IF(
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
  std::vector<ComputeNode*> nodes = getNodes();
  const auto count = nodes.size();
  if (count < 2) {
    return hazards;
  }

  auto indexOf = [&](const ComputeNode* node) -> std::size_t {
    const auto it = std::ranges::find(nodes, node);
    return static_cast<std::size_t>(std::distance(nodes.begin(), it));
  };

  // reachable[i][j] is true when node j runs strictly after node i, i.e. an
  // explicit dependency path orders them. Unordered pairs may run concurrently.
  std::vector<std::vector<bool>> reachable(
      count, std::vector<bool>(count, false));
  for (std::size_t i = 0; i < count; ++i) {
    std::vector<ComputeNode*> stack;
    for (const auto& edge : m_edges) {
      if (edge.from == nodes[i]) {
        stack.push_back(edge.to);
      }
    }
    while (!stack.empty()) {
      auto* current = stack.back();
      stack.pop_back();
      const auto j = indexOf(current);
      if (j >= count || reachable[i][j]) {
        continue;
      }
      reachable[i][j] = true;
      for (const auto& edge : m_edges) {
        if (edge.from == current) {
          stack.push_back(edge.to);
        }
      }
    }
  }

  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      if (reachable[i][j] || reachable[j][i]) {
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
                  nodes[i], nodes[j], first.resource, first.mode, second.mode});
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
  std::vector<ComputeNode*> stack{const_cast<ComputeNode*>(&to)};
  std::vector<ComputeNode*> visited;

  while (!stack.empty()) {
    auto* current = stack.back();
    stack.pop_back();

    if (current == &from) {
      return true;
    }

    if (containsPointer(visited, current)) {
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
std::vector<ComputeNode*> ComputeGraph::buildTopologicalOrder(
    bool throwOnCycle) const
{
  std::vector<ComputeNode*> nodes = getNodes();
  std::vector<std::size_t> indegree(nodes.size(), 0);
  std::vector<bool> consumed(nodes.size(), false);

  auto nodeIndex = [&](const ComputeNode* node) -> std::size_t {
    const auto it = std::ranges::find(nodes, node);
    return static_cast<std::size_t>(std::distance(nodes.begin(), it));
  };

  for (const auto& edge : m_edges) {
    if (!edge.from || !edge.to || !ownsNode(*edge.from)
        || !ownsNode(*edge.to)) {
      if (throwOnCycle) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidOperationException,
            "Compute graph contains an edge with nodes outside the graph");
      }
      return {};
    }
    ++indegree[nodeIndex(edge.to)];
  }

  std::vector<ComputeNode*> order;
  order.reserve(nodes.size());

  while (order.size() < nodes.size()) {
    bool consumedNode = false;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (consumed[i] || indegree[i] != 0) {
        continue;
      }

      auto* node = nodes[i];
      consumed[i] = true;
      order.push_back(node);
      consumedNode = true;

      for (const auto& edge : m_edges) {
        if (edge.from == node) {
          --indegree[nodeIndex(edge.to)];
        }
      }
      break;
    }

    if (!consumedNode) {
      if (throwOnCycle) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidOperationException, "Compute graph contains a cycle");
      }
      return order;
    }
  }

  return order;
}

} // namespace dart::simulation::experimental::compute
