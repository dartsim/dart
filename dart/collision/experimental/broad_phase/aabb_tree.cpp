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

#include <dart/collision/experimental/broad_phase/aabb_tree.hpp>

#include <algorithm>
#include <stack>

#include <cassert>
#include <cmath>

namespace dart::collision::experimental {

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

  std::size_t leafIndex = allocateNode();
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
  auto it = objectToNode_.find(id);
  if (it == objectToNode_.end()) {
    add(id, aabb);
    return;
  }

  std::size_t leafIndex = it->second;
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
  auto it = objectToNode_.find(id);
  if (it == objectToNode_.end()) {
    return;
  }

  std::size_t leafIndex = it->second;
  objectToNode_.erase(it);
  removeLeaf(leafIndex);
  freeNode(leafIndex);
}

std::vector<BroadPhasePair> AabbTreeBroadPhase::queryPairs() const
{
  std::vector<BroadPhasePair> pairs;

  if (root_ == kNullNode) {
    return pairs;
  }

  queryPairsRecursive(root_, root_, pairs);

  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());

  return pairs;
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
    return nodeCount_ == 0;
  }
  return validateStructure(root_);
}

std::size_t AabbTreeBroadPhase::allocateNode()
{
  if (freeList_ != kNullNode) {
    std::size_t nodeIndex = freeList_;
    freeList_ = nodes_[nodeIndex].parent;
    nodes_[nodeIndex] = Node{};
    ++nodeCount_;
    return nodeIndex;
  }

  std::size_t nodeIndex = nodes_.size();
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

  const Aabb leafAabb = nodes_[leafIndex].fatAabb;
  std::size_t siblingIndex = findBestSibling(leafAabb);

  std::size_t oldParent = nodes_[siblingIndex].parent;
  std::size_t newParent = allocateNode();

  nodes_[newParent].parent = oldParent;
  nodes_[newParent].fatAabb = combine(leafAabb, nodes_[siblingIndex].fatAabb);
  nodes_[newParent].height = nodes_[siblingIndex].height + 1;

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

  std::size_t parent = nodes_[leafIndex].parent;
  std::size_t grandParent = nodes_[parent].parent;
  std::size_t sibling = (nodes_[parent].left == leafIndex)
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
    std::size_t left = nodes_[index].left;
    std::size_t right = nodes_[index].right;

    double area = surfaceArea(nodes_[index].fatAabb);
    double combinedArea = surfaceArea(combine(nodes_[index].fatAabb, aabb));
    double cost = 2.0 * combinedArea;
    double inheritanceCost = 2.0 * (combinedArea - area);

    double costLeft;
    if (nodes_[left].isLeaf()) {
      costLeft
          = surfaceArea(combine(aabb, nodes_[left].fatAabb)) + inheritanceCost;
    } else {
      double oldArea = surfaceArea(nodes_[left].fatAabb);
      double newArea = surfaceArea(combine(aabb, nodes_[left].fatAabb));
      costLeft = (newArea - oldArea) + inheritanceCost;
    }

    double costRight;
    if (nodes_[right].isLeaf()) {
      costRight
          = surfaceArea(combine(aabb, nodes_[right].fatAabb)) + inheritanceCost;
    } else {
      double oldArea = surfaceArea(nodes_[right].fatAabb);
      double newArea = surfaceArea(combine(aabb, nodes_[right].fatAabb));
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

    std::size_t left = nodes_[nodeIndex].left;
    std::size_t right = nodes_[nodeIndex].right;

    assert(left != kNullNode);
    assert(right != kNullNode);

    nodes_[nodeIndex].height
        = 1 + std::max(nodes_[left].height, nodes_[right].height);
    nodes_[nodeIndex].fatAabb
        = combine(nodes_[left].fatAabb, nodes_[right].fatAabb);

    nodeIndex = nodes_[nodeIndex].parent;
  }
}

std::size_t AabbTreeBroadPhase::balance(std::size_t nodeIndex)
{
  assert(nodeIndex != kNullNode);

  Node& A = nodes_[nodeIndex];
  if (A.isLeaf() || A.height < 2) {
    return nodeIndex;
  }

  std::size_t iB = A.left;
  std::size_t iC = A.right;
  assert(iB < nodes_.size());
  assert(iC < nodes_.size());

  Node& B = nodes_[iB];
  Node& C = nodes_[iC];

  int balance = static_cast<int>(C.height) - static_cast<int>(B.height);

  if (balance > 1) {
    std::size_t iF = C.left;
    std::size_t iG = C.right;
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
      A.height = 1 + std::max(B.height, G.height);
      C.height = 1 + std::max(A.height, F.height);
    } else {
      C.right = iG;
      A.right = iF;
      F.parent = nodeIndex;
      A.fatAabb = combine(B.fatAabb, F.fatAabb);
      C.fatAabb = combine(A.fatAabb, G.fatAabb);
      A.height = 1 + std::max(B.height, F.height);
      C.height = 1 + std::max(A.height, G.height);
    }

    return iC;
  }

  if (balance < -1) {
    std::size_t iD = B.left;
    std::size_t iE = B.right;
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
      A.height = 1 + std::max(C.height, E.height);
      B.height = 1 + std::max(A.height, D.height);
    } else {
      B.right = iE;
      A.left = iD;
      D.parent = nodeIndex;
      A.fatAabb = combine(C.fatAabb, D.fatAabb);
      B.fatAabb = combine(A.fatAabb, E.fatAabb);
      A.height = 1 + std::max(C.height, D.height);
      B.height = 1 + std::max(A.height, E.height);
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
  Eigen::Vector3d d = aabb.max - aabb.min;
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
    std::size_t id1 = nodes_[nodeA].objectId;
    std::size_t id2 = nodes_[nodeB].objectId;
    if (nodes_[nodeA].tightAabb.overlaps(nodes_[nodeB].tightAabb)) {
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

std::size_t AabbTreeBroadPhase::computeHeight(std::size_t nodeIndex) const
{
  if (nodeIndex == kNullNode) {
    return 0;
  }

  if (nodes_[nodeIndex].isLeaf()) {
    return 0;
  }

  std::size_t leftHeight = computeHeight(nodes_[nodeIndex].left);
  std::size_t rightHeight = computeHeight(nodes_[nodeIndex].right);

  return 1 + std::max(leftHeight, rightHeight);
}

bool AabbTreeBroadPhase::validateStructure(std::size_t nodeIndex) const
{
  if (nodeIndex == kNullNode) {
    return true;
  }

  if (nodeIndex == root_) {
    if (nodes_[nodeIndex].parent != kNullNode) {
      return false;
    }
  }

  const Node& node = nodes_[nodeIndex];

  if (node.isLeaf()) {
    if (node.left != kNullNode || node.right != kNullNode) {
      return false;
    }
    if (node.height != 0) {
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

  std::size_t expectedHeight
      = 1 + std::max(nodes_[node.left].height, nodes_[node.right].height);
  if (node.height != expectedHeight) {
    return false;
  }

  return validateStructure(node.left) && validateStructure(node.right);
}

} // namespace dart::collision::experimental
