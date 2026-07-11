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

#include <dart/collision/native/broad_phase/AabbTree.hpp>

#include <algorithm>
#include <unordered_set>

#include <cassert>

namespace dart::collision::native {

AabbTreeBroadPhase::AabbTreeBroadPhase(double fatAabbMargin)
  : fatAabbMargin_(fatAabbMargin)
{
}

void AabbTreeBroadPhase::clear()
{
  nodes_.clear();
  root_ = kNullNode;
  nodeCount_ = 0;
  freeList_ = kNullNode;
  objectToNode_.clear();
}

void AabbTreeBroadPhase::add(std::size_t id, const Aabb& aabb)
{
  if (objectToNode_.find(id) != objectToNode_.end()) {
    update(id, aabb);
    return;
  }

  const std::size_t leafIndex = allocateNode();
  Node& leaf = nodes_[leafIndex];

  leaf.tightAabb = aabb;
  leaf.fatAabb = aabb;
  leaf.fatAabb.expand(fatAabbMargin_);
  leaf.objectId = id;
  leaf.height = 0;

  objectToNode_[id] = leafIndex;

  insertLeaf(leafIndex);
}

void AabbTreeBroadPhase::update(std::size_t id, const Aabb& aabb)
{
  const auto it = objectToNode_.find(id);
  if (it == objectToNode_.end()) {
    return;
  }

  const std::size_t leafIndex = it->second;
  Node& leaf = nodes_[leafIndex];

  if (leaf.fatAabb.contains(aabb)) {
    leaf.tightAabb = aabb;
    return;
  }

  removeLeaf(leafIndex);

  leaf.tightAabb = aabb;
  leaf.fatAabb = aabb;
  leaf.fatAabb.expand(fatAabbMargin_);

  insertLeaf(leafIndex);
}

void AabbTreeBroadPhase::remove(std::size_t id)
{
  const auto it = objectToNode_.find(id);
  if (it == objectToNode_.end()) {
    return;
  }

  const std::size_t leafIndex = it->second;
  objectToNode_.erase(it);
  removeLeaf(leafIndex);
  freeNode(leafIndex);
}

std::vector<BroadPhasePair> AabbTreeBroadPhase::queryPairs() const
{
  std::vector<BroadPhasePair> pairs;
  queryPairs(pairs);
  return pairs;
}

void AabbTreeBroadPhase::queryPairs(std::vector<BroadPhasePair>& out) const
{
  out.clear();

  if (root_ == kNullNode) {
    return;
  }

  queryPairsRecursive(root_, root_, out);

  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
}

bool AabbTreeBroadPhase::visitPairsAnyOrder(
    const BroadPhasePairVisitor& visitor) const
{
  // Streams candidates straight out of the tree walk with no sort/dedup, so
  // an early visitor rejection (e.g. a boolean query's first hit) aborts the
  // traversal without materializing the pair set. The tree self-query may
  // visit a pair more than once; callers must be idempotent and
  // order-independent (see BroadPhase::visitPairsAnyOrder).
  if (root_ == kNullNode) {
    return true;
  }

  return visitPairsRecursiveAnyOrder(root_, root_, visitor);
}

bool AabbTreeBroadPhase::visitPairsRecursiveAnyOrder(
    std::size_t nodeA,
    std::size_t nodeB,
    const BroadPhasePairVisitor& visitor) const
{
  if (nodeA == kNullNode || nodeB == kNullNode) {
    return true;
  }

  if (nodeA == nodeB) {
    if (nodes_[nodeA].isLeaf()) {
      return true;
    }

    return visitPairsRecursiveAnyOrder(
               nodes_[nodeA].left, nodes_[nodeA].right, visitor)
           && visitPairsRecursiveAnyOrder(
               nodes_[nodeA].left, nodes_[nodeA].left, visitor)
           && visitPairsRecursiveAnyOrder(
               nodes_[nodeA].right, nodes_[nodeA].right, visitor);
  }

  if (!nodes_[nodeA].fatAabb.overlaps(nodes_[nodeB].fatAabb)) {
    return true;
  }

  if (nodes_[nodeA].isLeaf() && nodes_[nodeB].isLeaf()) {
    if (nodes_[nodeA].tightAabb.overlaps(nodes_[nodeB].tightAabb)) {
      const std::size_t id1 = nodes_[nodeA].objectId;
      const std::size_t id2 = nodes_[nodeB].objectId;
      return visitor(std::min(id1, id2), std::max(id1, id2));
    }
    return true;
  }

  if (nodes_[nodeB].isLeaf()
      || (!nodes_[nodeA].isLeaf()
          && nodes_[nodeA].height > nodes_[nodeB].height)) {
    return visitPairsRecursiveAnyOrder(nodes_[nodeA].left, nodeB, visitor)
           && visitPairsRecursiveAnyOrder(nodes_[nodeA].right, nodeB, visitor);
  }

  return visitPairsRecursiveAnyOrder(nodeA, nodes_[nodeB].left, visitor)
         && visitPairsRecursiveAnyOrder(nodeA, nodes_[nodeB].right, visitor);
}

bool AabbTreeBroadPhase::visitPairs(const BroadPhasePairVisitor& visitor) const
{
  // Materializing and sorting the candidate pairs before visiting is
  // load-bearing, not incidental: callers that stop early (contact caps,
  // boolean queries) terminate on whichever pair they see first, so the
  // visitation order is behavior. Sorted (min,max) order reproduces
  // BruteForceBroadPhase's ordered scan bit-for-bit, which is what keeps the
  // native detector's results independent of tree topology. Collection costs
  // O(k log k) in the number of overlapping candidates; the previous
  // BruteForce scan was O(n^2) in object count regardless of overlap, so
  // even early-exit queries do not regress asymptotically.
  const auto pairs = queryPairs();
  for (const auto& pair : pairs) {
    if (!visitor(pair.first, pair.second)) {
      return false;
    }
  }

  return true;
}

void AabbTreeBroadPhase::buildDebugSnapshot(BroadPhaseDebugSnapshot& out) const
{
  out.clear();
  out.candidatePairs = queryPairs();
  out.numObjects = size();
  out.hasTreeTopology = true;
  out.rootNode = root_;

  if (root_ == kNullNode) {
    return;
  }

  std::vector<std::size_t> stack{root_};
  std::unordered_set<std::size_t> visited;
  visited.reserve(nodeCount_);

  while (!stack.empty()) {
    const std::size_t nodeIndex = stack.back();
    stack.pop_back();

    if (nodeIndex == kNullNode || nodeIndex >= nodes_.size()) {
      continue;
    }
    if (!visited.insert(nodeIndex).second) {
      continue;
    }

    const Node& node = nodes_[nodeIndex];
    BroadPhaseDebugNode debugNode;
    debugNode.nodeId = nodeIndex;
    debugNode.parent = node.parent;
    debugNode.left = node.left;
    debugNode.right = node.right;
    debugNode.objectId = node.objectId;
    debugNode.aabb = node.fatAabb;
    debugNode.tightAabb = node.isLeaf() ? node.tightAabb : node.fatAabb;
    debugNode.height = node.height;
    out.nodes.push_back(debugNode);

    if (!node.isLeaf()) {
      stack.push_back(node.left);
      stack.push_back(node.right);
    }
  }

  std::sort(
      out.nodes.begin(),
      out.nodes.end(),
      [](const BroadPhaseDebugNode& lhs, const BroadPhaseDebugNode& rhs) {
        return lhs.nodeId < rhs.nodeId;
      });
}

void AabbTreeBroadPhase::build(
    span<const std::size_t> ids, span<const Aabb> aabbs)
{
  clear();

  const std::size_t n = std::min(ids.size(), aabbs.size());
  if (n == 0u) {
    return;
  }

  nodes_.reserve(2u * n);

  for (std::size_t i = 0; i < n; ++i) {
    add(ids[i], aabbs[i]);
  }
}

void AabbTreeBroadPhase::updateRange(
    span<const std::size_t> ids, span<const Aabb> aabbs)
{
  const std::size_t n = std::min(ids.size(), aabbs.size());

  for (std::size_t i = 0; i < n; ++i) {
    update(ids[i], aabbs[i]);
  }
}

std::vector<std::size_t> AabbTreeBroadPhase::queryOverlapping(
    const Aabb& aabb) const
{
  std::vector<std::size_t> results;

  if (root_ == kNullNode) {
    return results;
  }

  queryOverlappingRecursive(root_, aabb, results);

  std::sort(results.begin(), results.end());

  return results;
}

std::size_t AabbTreeBroadPhase::size() const
{
  return objectToNode_.size();
}

double AabbTreeBroadPhase::getFatAabbMargin() const
{
  return fatAabbMargin_;
}

void AabbTreeBroadPhase::setFatAabbMargin(double margin)
{
  fatAabbMargin_ = margin;
}

std::size_t AabbTreeBroadPhase::getHeight() const
{
  if (root_ == kNullNode) {
    return 0;
  }

  return nodes_[root_].height;
}

bool AabbTreeBroadPhase::validate() const
{
  if (root_ == kNullNode) {
    return nodeCount_ == 0u;
  }

  return validateStructure(root_);
}

std::size_t AabbTreeBroadPhase::allocateNode()
{
  if (freeList_ != kNullNode) {
    const std::size_t nodeIndex = freeList_;
    freeList_ = nodes_[nodeIndex].parent;
    nodes_[nodeIndex] = Node{};
    ++nodeCount_;
    return nodeIndex;
  }

  const std::size_t nodeIndex = nodes_.size();
  nodes_.emplace_back();
  ++nodeCount_;
  return nodeIndex;
}

void AabbTreeBroadPhase::freeNode(std::size_t nodeIndex)
{
  assert(nodeIndex < nodes_.size());
  nodes_[nodeIndex].parent = freeList_;
  nodes_[nodeIndex].height = kNullNode;
  freeList_ = nodeIndex;
  --nodeCount_;
}

void AabbTreeBroadPhase::insertLeaf(std::size_t leafIndex)
{
  if (root_ == kNullNode) {
    root_ = leafIndex;
    nodes_[leafIndex].parent = kNullNode;
    return;
  }

  // Copy values before allocateNode(), which may reallocate nodes_.
  const Aabb leafAabb = nodes_[leafIndex].fatAabb;
  const std::size_t siblingIndex = findBestSibling(leafAabb);
  const Aabb siblingAabb = nodes_[siblingIndex].fatAabb;
  const std::size_t oldParent = nodes_[siblingIndex].parent;
  const std::size_t newParent = allocateNode();

  nodes_[newParent].parent = oldParent;
  nodes_[newParent].fatAabb = combine(leafAabb, siblingAabb);
  nodes_[newParent].height = nodes_[siblingIndex].height + 1u;

  if (oldParent != kNullNode) {
    if (nodes_[oldParent].left == siblingIndex) {
      nodes_[oldParent].left = newParent;
    } else {
      nodes_[oldParent].right = newParent;
    }
    nodes_[newParent].left = siblingIndex;
    nodes_[newParent].right = leafIndex;
    nodes_[siblingIndex].parent = newParent;
    nodes_[leafIndex].parent = newParent;
  } else {
    nodes_[newParent].left = siblingIndex;
    nodes_[newParent].right = leafIndex;
    nodes_[siblingIndex].parent = newParent;
    nodes_[leafIndex].parent = newParent;
    root_ = newParent;
  }

  rebalance(nodes_[leafIndex].parent);
}

void AabbTreeBroadPhase::removeLeaf(std::size_t leafIndex)
{
  if (leafIndex == root_) {
    root_ = kNullNode;
    return;
  }

  const std::size_t parent = nodes_[leafIndex].parent;
  const std::size_t grandParent = nodes_[parent].parent;
  const std::size_t sibling = (nodes_[parent].left == leafIndex)
                                  ? nodes_[parent].right
                                  : nodes_[parent].left;

  if (grandParent != kNullNode) {
    if (nodes_[grandParent].left == parent) {
      nodes_[grandParent].left = sibling;
    } else {
      nodes_[grandParent].right = sibling;
    }
    nodes_[sibling].parent = grandParent;
    freeNode(parent);
    rebalance(grandParent);
  } else {
    root_ = sibling;
    nodes_[sibling].parent = kNullNode;
    freeNode(parent);
  }
}

std::size_t AabbTreeBroadPhase::findBestSibling(const Aabb& aabb) const
{
  std::size_t index = root_;

  while (!nodes_[index].isLeaf()) {
    const std::size_t left = nodes_[index].left;
    const std::size_t right = nodes_[index].right;

    const double area = surfaceArea(nodes_[index].fatAabb);
    const double combinedArea
        = surfaceArea(combine(nodes_[index].fatAabb, aabb));
    const double cost = 2.0 * combinedArea;
    const double inheritanceCost = 2.0 * (combinedArea - area);

    double costLeft;
    if (nodes_[left].isLeaf()) {
      costLeft
          = surfaceArea(combine(aabb, nodes_[left].fatAabb)) + inheritanceCost;
    } else {
      const double oldArea = surfaceArea(nodes_[left].fatAabb);
      const double newArea = surfaceArea(combine(aabb, nodes_[left].fatAabb));
      costLeft = (newArea - oldArea) + inheritanceCost;
    }

    double costRight;
    if (nodes_[right].isLeaf()) {
      costRight
          = surfaceArea(combine(aabb, nodes_[right].fatAabb)) + inheritanceCost;
    } else {
      const double oldArea = surfaceArea(nodes_[right].fatAabb);
      const double newArea = surfaceArea(combine(aabb, nodes_[right].fatAabb));
      costRight = (newArea - oldArea) + inheritanceCost;
    }

    if (cost < costLeft && cost < costRight) {
      break;
    }

    index = (costLeft < costRight) ? left : right;
  }

  return index;
}

void AabbTreeBroadPhase::rebalance(std::size_t nodeIndex)
{
  while (nodeIndex != kNullNode) {
    nodeIndex = balance(nodeIndex);

    const std::size_t left = nodes_[nodeIndex].left;
    const std::size_t right = nodes_[nodeIndex].right;

    assert(left != kNullNode);
    assert(right != kNullNode);

    nodes_[nodeIndex].height
        = 1u + std::max(nodes_[left].height, nodes_[right].height);
    nodes_[nodeIndex].fatAabb
        = combine(nodes_[left].fatAabb, nodes_[right].fatAabb);

    nodeIndex = nodes_[nodeIndex].parent;
  }
}

std::size_t AabbTreeBroadPhase::balance(std::size_t nodeIndex)
{
  assert(nodeIndex != kNullNode);

  Node& A = nodes_[nodeIndex];
  if (A.isLeaf() || A.height < 2u) {
    return nodeIndex;
  }

  const std::size_t iB = A.left;
  const std::size_t iC = A.right;
  assert(iB < nodes_.size());
  assert(iC < nodes_.size());

  Node& B = nodes_[iB];
  Node& C = nodes_[iC];

  const int balance = static_cast<int>(C.height) - static_cast<int>(B.height);

  if (balance > 1) {
    const std::size_t iF = C.left;
    const std::size_t iG = C.right;
    assert(iF < nodes_.size());
    assert(iG < nodes_.size());
    Node& F = nodes_[iF];
    Node& G = nodes_[iG];

    C.left = nodeIndex;
    C.parent = A.parent;
    A.parent = iC;

    if (C.parent != kNullNode) {
      if (nodes_[C.parent].left == nodeIndex) {
        nodes_[C.parent].left = iC;
      } else {
        assert(nodes_[C.parent].right == nodeIndex);
        nodes_[C.parent].right = iC;
      }
    } else {
      root_ = iC;
    }

    if (F.height > G.height) {
      C.right = iF;
      A.right = iG;
      G.parent = nodeIndex;
      A.fatAabb = combine(B.fatAabb, G.fatAabb);
      C.fatAabb = combine(A.fatAabb, F.fatAabb);
      A.height = 1u + std::max(B.height, G.height);
      C.height = 1u + std::max(A.height, F.height);
    } else {
      C.right = iG;
      A.right = iF;
      F.parent = nodeIndex;
      A.fatAabb = combine(B.fatAabb, F.fatAabb);
      C.fatAabb = combine(A.fatAabb, G.fatAabb);
      A.height = 1u + std::max(B.height, F.height);
      C.height = 1u + std::max(A.height, G.height);
    }

    return iC;
  }

  if (balance < -1) {
    const std::size_t iD = B.left;
    const std::size_t iE = B.right;
    assert(iD < nodes_.size());
    assert(iE < nodes_.size());
    Node& D = nodes_[iD];
    Node& E = nodes_[iE];

    B.left = nodeIndex;
    B.parent = A.parent;
    A.parent = iB;

    if (B.parent != kNullNode) {
      if (nodes_[B.parent].left == nodeIndex) {
        nodes_[B.parent].left = iB;
      } else {
        assert(nodes_[B.parent].right == nodeIndex);
        nodes_[B.parent].right = iB;
      }
    } else {
      root_ = iB;
    }

    if (D.height > E.height) {
      B.right = iD;
      A.left = iE;
      E.parent = nodeIndex;
      A.fatAabb = combine(C.fatAabb, E.fatAabb);
      B.fatAabb = combine(A.fatAabb, D.fatAabb);
      A.height = 1u + std::max(C.height, E.height);
      B.height = 1u + std::max(A.height, D.height);
    } else {
      B.right = iE;
      A.left = iD;
      D.parent = nodeIndex;
      A.fatAabb = combine(C.fatAabb, D.fatAabb);
      B.fatAabb = combine(A.fatAabb, E.fatAabb);
      A.height = 1u + std::max(C.height, D.height);
      B.height = 1u + std::max(A.height, E.height);
    }

    return iB;
  }

  return nodeIndex;
}

Aabb AabbTreeBroadPhase::combine(const Aabb& a, const Aabb& b)
{
  return Aabb(a.min.cwiseMin(b.min), a.max.cwiseMax(b.max));
}

double AabbTreeBroadPhase::surfaceArea(const Aabb& aabb)
{
  const Eigen::Vector3d d = aabb.max - aabb.min;
  return 2.0 * (d.x() * d.y() + d.y() * d.z() + d.z() * d.x());
}

void AabbTreeBroadPhase::queryPairsRecursive(
    std::size_t nodeA,
    std::size_t nodeB,
    std::vector<BroadPhasePair>& pairs) const
{
  if (nodeA == kNullNode || nodeB == kNullNode) {
    return;
  }

  if (nodeA == nodeB) {
    if (nodes_[nodeA].isLeaf()) {
      return;
    }

    queryPairsRecursive(nodes_[nodeA].left, nodes_[nodeA].right, pairs);
    queryPairsRecursive(nodes_[nodeA].left, nodes_[nodeA].left, pairs);
    queryPairsRecursive(nodes_[nodeA].right, nodes_[nodeA].right, pairs);
    return;
  }

  if (!nodes_[nodeA].fatAabb.overlaps(nodes_[nodeB].fatAabb)) {
    return;
  }

  if (nodes_[nodeA].isLeaf() && nodes_[nodeB].isLeaf()) {
    if (nodes_[nodeA].tightAabb.overlaps(nodes_[nodeB].tightAabb)) {
      const std::size_t id1 = nodes_[nodeA].objectId;
      const std::size_t id2 = nodes_[nodeB].objectId;
      pairs.emplace_back(std::min(id1, id2), std::max(id1, id2));
    }
    return;
  }

  if (nodes_[nodeB].isLeaf()
      || (!nodes_[nodeA].isLeaf()
          && nodes_[nodeA].height > nodes_[nodeB].height)) {
    queryPairsRecursive(nodes_[nodeA].left, nodeB, pairs);
    queryPairsRecursive(nodes_[nodeA].right, nodeB, pairs);
  } else {
    queryPairsRecursive(nodeA, nodes_[nodeB].left, pairs);
    queryPairsRecursive(nodeA, nodes_[nodeB].right, pairs);
  }
}

void AabbTreeBroadPhase::queryOverlappingRecursive(
    std::size_t nodeIndex,
    const Aabb& aabb,
    std::vector<std::size_t>& results) const
{
  if (nodeIndex == kNullNode) {
    return;
  }

  if (!nodes_[nodeIndex].fatAabb.overlaps(aabb)) {
    return;
  }

  if (nodes_[nodeIndex].isLeaf()) {
    if (nodes_[nodeIndex].tightAabb.overlaps(aabb)) {
      results.push_back(nodes_[nodeIndex].objectId);
    }
    return;
  }

  queryOverlappingRecursive(nodes_[nodeIndex].left, aabb, results);
  queryOverlappingRecursive(nodes_[nodeIndex].right, aabb, results);
}

bool AabbTreeBroadPhase::validateStructure(std::size_t nodeIndex) const
{
  if (nodeIndex == kNullNode) {
    return true;
  }

  if (nodeIndex == root_ && nodes_[nodeIndex].parent != kNullNode) {
    return false;
  }

  const Node& node = nodes_[nodeIndex];

  if (node.isLeaf()) {
    if (node.left != kNullNode || node.right != kNullNode) {
      return false;
    }
    if (node.height != 0u) {
      return false;
    }
    return true;
  }

  if (node.left == kNullNode || node.right == kNullNode) {
    return false;
  }

  if (nodes_[node.left].parent != nodeIndex) {
    return false;
  }
  if (nodes_[node.right].parent != nodeIndex) {
    return false;
  }

  const std::size_t expectedHeight
      = 1u + std::max(nodes_[node.left].height, nodes_[node.right].height);
  if (node.height != expectedHeight) {
    return false;
  }

  return validateStructure(node.left) && validateStructure(node.right);
}

} // namespace dart::collision::native
