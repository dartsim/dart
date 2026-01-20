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

#include "dart/simulation/compute_graph/compute_graph.hpp"

#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_set>

namespace dart::simulation {

NodeId ComputeGraph::addNode(ComputeNodePtr node)
{
  if (!node) {
    return kInvalidNodeId;
  }

  NodeId id = mNextId++;
  node->mId = id;

  mNodes.push_back(node);
  mNodeMap[id] = node;
  mPredecessors[id] = {};
  mSuccessors[id] = {};
  mFinalized = false;

  return id;
}

NodeId ComputeGraph::addNode(LambdaNode::Callable func, std::string_view name)
{
  return addNode(std::make_shared<LambdaNode>(std::move(func), name));
}

ComputeNodePtr ComputeGraph::getNode(NodeId id) const
{
  auto it = mNodeMap.find(id);
  return (it != mNodeMap.end()) ? it->second : nullptr;
}

std::span<const ComputeNodePtr> ComputeGraph::getNodes() const
{
  return mNodes;
}

std::size_t ComputeGraph::getNodeCount() const
{
  return mNodes.size();
}

bool ComputeGraph::addEdge(NodeId predecessor, NodeId successor)
{
  if (mNodeMap.find(predecessor) == mNodeMap.end()
      || mNodeMap.find(successor) == mNodeMap.end()) {
    return false;
  }

  if (predecessor == successor) {
    return false;
  }

  if (detectCycle(predecessor, successor)) {
    return false;
  }

  mSuccessors[predecessor].push_back(successor);
  mPredecessors[successor].push_back(predecessor);
  mFinalized = false;

  return true;
}

bool ComputeGraph::addEdge(
    const ComputeNodePtr& predecessor, const ComputeNodePtr& successor)
{
  if (!predecessor || !successor) {
    return false;
  }
  return addEdge(predecessor->getId(), successor->getId());
}

std::span<const NodeId> ComputeGraph::getPredecessors(NodeId nodeId) const
{
  auto it = mPredecessors.find(nodeId);
  if (it == mPredecessors.end()) {
    return {};
  }
  return it->second;
}

std::span<const NodeId> ComputeGraph::getSuccessors(NodeId nodeId) const
{
  auto it = mSuccessors.find(nodeId);
  if (it == mSuccessors.end()) {
    return {};
  }
  return it->second;
}

bool ComputeGraph::isValid() const
{
  for (const auto& node : mNodes) {
    if (!node || node->getId() == kInvalidNodeId) {
      return false;
    }
  }

  for (const auto& [nodeId, preds] : mPredecessors) {
    for (NodeId predId : preds) {
      if (mNodeMap.find(predId) == mNodeMap.end()) {
        return false;
      }
    }
  }

  return true;
}

bool ComputeGraph::finalize()
{
  if (mFinalized) {
    return true;
  }

  if (!isValid()) {
    return false;
  }

  computeTopologicalOrder();

  if (mTopologicalOrder.size() != mNodes.size()) {
    return false;
  }

  mFinalized = true;
  return true;
}

bool ComputeGraph::isFinalized() const
{
  return mFinalized;
}

std::span<const NodeId> ComputeGraph::getTopologicalOrder() const
{
  return mTopologicalOrder;
}

void ComputeGraph::clear()
{
  mNodes.clear();
  mNodeMap.clear();
  mPredecessors.clear();
  mSuccessors.clear();
  mTopologicalOrder.clear();
  mNextId = 1;
  mFinalized = false;
}

void ComputeGraph::debugPrint() const
{
  std::cout << "ComputeGraph with " << mNodes.size() << " nodes:\n";
  for (const auto& node : mNodes) {
    std::cout << "  Node " << node->getId() << " (" << node->getName() << ")";
    auto preds = getPredecessors(node->getId());
    if (!preds.empty()) {
      std::cout << " <- [";
      for (std::size_t i = 0; i < preds.size(); ++i) {
        if (i > 0)
          std::cout << ", ";
        std::cout << preds[i];
      }
      std::cout << "]";
    }
    std::cout << "\n";
  }

  if (mFinalized) {
    std::cout << "Topological order: [";
    for (std::size_t i = 0; i < mTopologicalOrder.size(); ++i) {
      if (i > 0)
        std::cout << ", ";
      std::cout << mTopologicalOrder[i];
    }
    std::cout << "]\n";
  }
}

bool ComputeGraph::detectCycle(NodeId from, NodeId to) const
{
  // BFS from 'to' to check if we can reach 'from'
  // If adding edge from->to would create a cycle, 'from' is reachable from 'to'
  std::unordered_set<NodeId> visited;
  std::queue<NodeId> queue;
  queue.push(to);

  while (!queue.empty()) {
    NodeId current = queue.front();
    queue.pop();

    if (current == from) {
      return true;
    }

    if (visited.count(current) > 0) {
      continue;
    }
    visited.insert(current);

    auto it = mSuccessors.find(current);
    if (it != mSuccessors.end()) {
      for (NodeId succ : it->second) {
        queue.push(succ);
      }
    }
  }

  return false;
}

void ComputeGraph::computeTopologicalOrder()
{
  // Kahn's algorithm for topological sort
  mTopologicalOrder.clear();

  std::unordered_map<NodeId, std::size_t> inDegree;
  for (const auto& node : mNodes) {
    inDegree[node->getId()] = 0;
  }
  for (const auto& [nodeId, preds] : mPredecessors) {
    inDegree[nodeId] = preds.size();
  }

  std::queue<NodeId> ready;
  for (const auto& node : mNodes) {
    if (inDegree[node->getId()] == 0) {
      ready.push(node->getId());
    }
  }

  while (!ready.empty()) {
    NodeId nodeId = ready.front();
    ready.pop();
    mTopologicalOrder.push_back(nodeId);

    auto it = mSuccessors.find(nodeId);
    if (it != mSuccessors.end()) {
      for (NodeId succ : it->second) {
        if (--inDegree[succ] == 0) {
          ready.push(succ);
        }
      }
    }
  }
}

} // namespace dart::simulation
