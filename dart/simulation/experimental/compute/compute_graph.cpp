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

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/compute/compute_graph.hpp>

#include <algorithm>
#include <queue>
#include <unordered_set>

namespace dart::simulation::experimental::compute {

ComputeNode& ComputeGraph::addNode(std::string name, ComputeNode::ExecuteFn fn)
{
  if (m_nodesByName.contains(name)) {
    throw InvalidArgumentException(
        "Node with name '" + name + "' already exists in graph");
  }

  auto node = std::make_unique<ComputeNode>(std::move(name), std::move(fn));
  auto* ptr = node.get();
  m_nodesByName[ptr->getName()] = ptr;
  m_nodes.push_back(std::move(node));
  return *ptr;
}

void ComputeGraph::addDependency(ComputeNode& from, ComputeNode& to)
{
  auto fromIt = m_nodesByName.find(from.getName());
  auto toIt = m_nodesByName.find(to.getName());

  if (fromIt == m_nodesByName.end() || fromIt->second != &from) {
    throw InvalidArgumentException(
        "Node '" + from.getName() + "' is not in this graph");
  }
  if (toIt == m_nodesByName.end() || toIt->second != &to) {
    throw InvalidArgumentException(
        "Node '" + to.getName() + "' is not in this graph");
  }

  if (&from == &to) {
    throw InvalidOperationException(
        "Cannot add self-dependency for node '" + from.getName() + "'");
  }

  for (const auto& edge : m_edges) {
    if (edge.from == &from && edge.to == &to) {
      return;
    }
  }

  if (wouldCreateCycle(from, to)) {
    throw InvalidOperationException(
        "Adding dependency from '" + from.getName() + "' to '" + to.getName()
        + "' would create a cycle");
  }

  m_edges.push_back({&from, &to});
}

ComputeNode* ComputeGraph::getNode(std::string_view name) const
{
  auto it = m_nodesByName.find(std::string(name));
  return it != m_nodesByName.end() ? it->second : nullptr;
}

std::vector<ComputeNode*> ComputeGraph::getTopologicalOrder() const
{
  if (m_nodes.empty()) {
    return {};
  }

  // Kahn's algorithm: compute in-degree for each node
  std::unordered_map<ComputeNode*, std::size_t> inDegree;
  for (const auto& node : m_nodes) {
    inDegree[node.get()] = 0;
  }
  for (const auto& edge : m_edges) {
    inDegree[edge.to]++;
  }

  // Process nodes with zero in-degree in insertion order for determinism
  std::queue<ComputeNode*> queue;
  for (const auto& node : m_nodes) {
    if (inDegree[node.get()] == 0) {
      queue.push(node.get());
    }
  }

  std::vector<ComputeNode*> result;
  result.reserve(m_nodes.size());

  while (!queue.empty()) {
    auto* node = queue.front();
    queue.pop();
    result.push_back(node);

    for (const auto& edge : m_edges) {
      if (edge.from == node) {
        if (--inDegree[edge.to] == 0) {
          queue.push(edge.to);
        }
      }
    }
  }

  if (result.size() != m_nodes.size()) {
    throw InvalidOperationException(
        "Graph contains a cycle - topological sort impossible");
  }

  return result;
}

std::vector<ComputeNode*> ComputeGraph::getNodes() const
{
  std::vector<ComputeNode*> result;
  result.reserve(m_nodes.size());
  for (const auto& node : m_nodes) {
    result.push_back(node.get());
  }
  return result;
}

bool ComputeGraph::validate() const
{
  for (const auto& node : m_nodes) {
    if (!node->isValid()) {
      return false;
    }
  }

  for (const auto& edge : m_edges) {
    bool fromFound = false;
    bool toFound = false;
    for (const auto& node : m_nodes) {
      if (node.get() == edge.from)
        fromFound = true;
      if (node.get() == edge.to)
        toFound = true;
    }
    if (!fromFound || !toFound) {
      return false;
    }
  }

  try {
    [[maybe_unused]] auto order = getTopologicalOrder();
    return true;
  } catch (const InvalidOperationException&) {
    return false;
  }
}

void ComputeGraph::clear()
{
  m_edges.clear();
  m_nodesByName.clear();
  m_nodes.clear();
}

std::vector<ComputeNode*> ComputeGraph::getDependencies(
    const ComputeNode& node) const
{
  std::vector<ComputeNode*> deps;
  for (const auto& edge : m_edges) {
    if (edge.to == &node) {
      deps.push_back(edge.from);
    }
  }
  return deps;
}

std::vector<ComputeNode*> ComputeGraph::getDependents(
    const ComputeNode& node) const
{
  std::vector<ComputeNode*> deps;
  for (const auto& edge : m_edges) {
    if (edge.from == &node) {
      deps.push_back(edge.to);
    }
  }
  return deps;
}

bool ComputeGraph::wouldCreateCycle(
    const ComputeNode& from, const ComputeNode& to) const
{
  // BFS from 'to' to check if 'from' is reachable (would create cycle)
  std::unordered_set<const ComputeNode*> visited;
  std::queue<const ComputeNode*> queue;
  queue.push(&to);

  while (!queue.empty()) {
    const auto* current = queue.front();
    queue.pop();

    if (current == &from) {
      return true;
    }

    if (visited.contains(current)) {
      continue;
    }
    visited.insert(current);

    for (const auto& edge : m_edges) {
      if (edge.from == current) {
        queue.push(edge.to);
      }
    }
  }

  return false;
}

} // namespace dart::simulation::experimental::compute
