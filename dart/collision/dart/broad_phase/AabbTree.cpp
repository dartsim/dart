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

#include <dart/collision/dart/broad_phase/AabbTree.hpp>

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

#include <cassert>
#include <cmath>

namespace dart::collision::native {

namespace {

double validateFatAabbMargin(double margin)
{
  if (!std::isfinite(margin) || margin < 0.0) {
    throw std::invalid_argument(
        "AabbTreeBroadPhase fat AABB margin must be finite and non-negative");
  }

  return margin;
}

} // namespace

AabbTreeBroadPhase::AabbTreeBroadPhase(double fatAabbMargin)
  : fatAabbMargin_(validateFatAabbMargin(fatAabbMargin))
{
}

void AabbTreeBroadPhase::clear()
{
  nodes_.clear();
  parents_.clear();
  root_ = kInvalidNode;
  nodeCount_ = 0;
  freeList_ = kInvalidNode;
  objectToNode_.clear();
  for (auto& values : tightMin_) {
    values.clear();
  }
  for (auto& values : tightMax_) {
    values.clear();
  }
  orderedIds_.clear();
  idsSorted_ = true;
}

void AabbTreeBroadPhase::setTightAabb(std::size_t id, const Aabb& aabb)
{
  for (std::size_t axis = 0; axis < 3; ++axis) {
    tightMin_[axis][id] = aabb.min[axis];
    tightMax_[axis][id] = aabb.max[axis];
  }
}

Aabb AabbTreeBroadPhase::tightAabb(std::size_t id) const
{
  return Aabb(
      Eigen::Vector3d(tightMin_[0][id], tightMin_[1][id], tightMin_[2][id]),
      Eigen::Vector3d(tightMax_[0][id], tightMax_[1][id], tightMax_[2][id]));
}

bool AabbTreeBroadPhase::overlapsTight(std::size_t id, const Aabb& aabb) const
{
  // Keep the same double comparisons, including touching bounds, as Aabb.
  return (tightMin_[0][id] <= aabb.max.x() && tightMax_[0][id] >= aabb.min.x())
         && (tightMin_[1][id] <= aabb.max.y()
             && tightMax_[1][id] >= aabb.min.y())
         && (tightMin_[2][id] <= aabb.max.z()
             && tightMax_[2][id] >= aabb.min.z());
}

void AabbTreeBroadPhase::add(std::size_t id, const Aabb& aabb)
{
  if (id < objectToNode_.size() && objectToNode_[id] != kInvalidNode) {
    update(id, aabb);
    return;
  }

  if (id >= std::numeric_limits<std::uint32_t>::max()) {
    throw std::length_error("AABB tree object id exceeds compact storage");
  }
  // Grow each id-indexed array against its own size: if one resize throws,
  // a later add() still grows the rest before writing to them.
  const std::size_t size = id + 1u;
  if (objectToNode_.size() < size) {
    objectToNode_.resize(size, kInvalidNode);
  }
  for (auto* bounds : {&tightMin_, &tightMax_}) {
    for (auto& values : *bounds) {
      if (values.size() < size) {
        values.resize(size);
      }
    }
  }

  // Reserve every allocation this insertion makes before changing the tree, so
  // a failure leaves it unchanged: the leaf, at most one internal node from
  // insertLeaf(), and the ordered id.
  reserveNodes(2u);
  if (orderedIds_.size() == orderedIds_.capacity()) {
    orderedIds_.reserve(std::max<std::size_t>(1u, 2u * orderedIds_.capacity()));
  }

  const NodeIndex leafIndex = allocateNode();
  Node& leaf = nodes_[leafIndex];
  setTightAabb(id, aabb);
  leaf.fatAabb = aabb;
  leaf.fatAabb.expand(fatAabbMargin_);
  leaf.maxObjectId = static_cast<std::uint32_t>(id);
  objectToNode_[id] = leafIndex;
  orderedIds_.push_back(id);
  idsSorted_ = false;

  insertLeaf(leafIndex);
}

void AabbTreeBroadPhase::update(std::size_t id, const Aabb& aabb)
{
  if (id >= objectToNode_.size() || objectToNode_[id] == kInvalidNode) {
    return;
  }

  const NodeIndex leafIndex = objectToNode_[id];
  Node& leaf = nodes_[leafIndex];
  setTightAabb(id, aabb);
  if (leaf.fatAabb.contains(aabb)) {
    return;
  }

  removeLeaf(leafIndex);
  leaf.fatAabb = aabb;
  leaf.fatAabb.expand(fatAabbMargin_);
  insertLeaf(leafIndex);
}

void AabbTreeBroadPhase::remove(std::size_t id)
{
  if (id >= objectToNode_.size() || objectToNode_[id] == kInvalidNode) {
    return;
  }

  const NodeIndex leafIndex = objectToNode_[id];
  objectToNode_[id] = kInvalidNode;
  orderedIds_.erase(std::find(orderedIds_.begin(), orderedIds_.end(), id));
  idsSorted_ = false;
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
  // The tree self-query rejects each disjoint subtree pair once, which
  // per-object queries cannot do in sparse trees. It may repeat a pair.
  visitPairsAnyOrder([&out](std::size_t id1, std::size_t id2) {
    out.emplace_back(id1, id2);
    return true;
  });
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
}

bool AabbTreeBroadPhase::visitPairsAnyOrder(
    const BroadPhasePairVisitor& visitor) const
{
  return root_ == kInvalidNode || visitPairsRecursive(root_, root_, visitor);
}

bool AabbTreeBroadPhase::visitPairsRecursive(
    NodeIndex nodeA,
    NodeIndex nodeB,
    const BroadPhasePairVisitor& visitor) const
{
  // Recursion depth is bounded by twice the tree height; it is about twice as
  // fast as an explicit stack on sparse trees.
  const Node& a = nodes_[nodeA];
  const Node& b = nodes_[nodeB];
  if (nodeA == nodeB) {
    return a.isLeaf()
           || (visitPairsRecursive(a.left, a.right, visitor)
               && visitPairsRecursive(a.left, a.left, visitor)
               && visitPairsRecursive(a.right, a.right, visitor));
  }
  if (!a.fatAabb.overlaps(b.fatAabb)) {
    return true;
  }
  if (a.isLeaf() && b.isLeaf()) {
    const std::size_t id1 = a.maxObjectId;
    const std::size_t id2 = b.maxObjectId;
    return !overlapsTight(id1, tightAabb(id2))
           || visitor(std::min(id1, id2), std::max(id1, id2));
  }
  if (b.isLeaf() || (!a.isLeaf() && a.height > b.height)) {
    return visitPairsRecursive(a.left, nodeB, visitor)
           && visitPairsRecursive(a.right, nodeB, visitor);
  }
  return visitPairsRecursive(nodeA, b.left, visitor)
         && visitPairsRecursive(nodeA, b.right, visitor);
}

bool AabbTreeBroadPhase::visitPairs(const BroadPhasePairVisitor& visitor) const
{
  // Match BruteForceBroadPhase's lexicographic order while materializing only
  // one object's higher-id overlaps, retaining early rejection for capped
  // queries. Updates preserve membership, so sorting ids is only needed after
  // add/remove. All query buffers are reused for allocation-free stepping.
  if (!idsSorted_) {
    std::sort(orderedIds_.begin(), orderedIds_.end());
    idsSorted_ = true;
  }

  auto& overlaps = mOverlapScratch;
  for (const std::size_t id : orderedIds_) {
    overlaps.clear();
    queryOverlappingImpl(tightAabb(id), overlaps, id, true);
    std::sort(overlaps.begin(), overlaps.end());
    for (const std::size_t other : overlaps) {
      if (!visitor(id, other)) {
        return false;
      }
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
  const auto debugIndex = [](NodeIndex index) -> std::size_t {
    return index == kInvalidNode ? kNullNode : static_cast<std::size_t>(index);
  };
  out.rootNode = debugIndex(root_);

  if (root_ == kInvalidNode) {
    return;
  }

  std::vector<NodeIndex> stack{root_};
  std::unordered_set<std::size_t> visited;
  visited.reserve(nodeCount_);

  while (!stack.empty()) {
    const NodeIndex nodeIndex = stack.back();
    stack.pop_back();

    if (nodeIndex < 0 || static_cast<std::size_t>(nodeIndex) >= nodes_.size()) {
      continue;
    }
    if (!visited.insert(nodeIndex).second) {
      continue;
    }

    const Node& node = nodes_[nodeIndex];
    BroadPhaseDebugNode debugNode;
    debugNode.nodeId = nodeIndex;
    debugNode.parent = debugIndex(parents_[nodeIndex]);
    debugNode.left = debugIndex(node.left);
    debugNode.right = debugIndex(node.right);
    debugNode.objectId = node.isLeaf() ? node.maxObjectId : kNullNode;
    debugNode.aabb = node.fatAabb;
    debugNode.tightAabb
        = node.isLeaf() ? tightAabb(node.maxObjectId) : node.fatAabb;
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
  parents_.reserve(2u * n);
  orderedIds_.reserve(n);

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

  if (root_ == kInvalidNode) {
    return results;
  }

  queryOverlappingImpl(aabb, results, 0u, false);

  std::sort(results.begin(), results.end());

  return results;
}

std::size_t AabbTreeBroadPhase::size() const
{
  return orderedIds_.size();
}

double AabbTreeBroadPhase::getFatAabbMargin() const
{
  return fatAabbMargin_;
}

void AabbTreeBroadPhase::setFatAabbMargin(double margin)
{
  fatAabbMargin_ = validateFatAabbMargin(margin);
}

std::size_t AabbTreeBroadPhase::getHeight() const
{
  if (root_ == kInvalidNode) {
    return 0;
  }

  return nodes_[root_].height;
}

bool AabbTreeBroadPhase::validate() const
{
  if (root_ == kInvalidNode) {
    return nodeCount_ == 0u && orderedIds_.empty();
  }

  return validateStructure(root_);
}

AabbTreeBroadPhase::NodeIndex AabbTreeBroadPhase::allocateNode()
{
  if (freeList_ != kInvalidNode) {
    const NodeIndex nodeIndex = freeList_;
    freeList_ = parents_[nodeIndex];
    nodes_[nodeIndex] = Node{};
    parents_[nodeIndex] = kInvalidNode;
    ++nodeCount_;
    return nodeIndex;
  }

  if (nodes_.size()
      >= static_cast<std::size_t>(std::numeric_limits<NodeIndex>::max())) {
    throw std::length_error("AABB tree node index exceeds compact storage");
  }
  reserveNodes(1u);
  const NodeIndex nodeIndex = static_cast<NodeIndex>(nodes_.size());
  nodes_.emplace_back();
  parents_.push_back(kInvalidNode);
  ++nodeCount_;
  return nodeIndex;
}

void AabbTreeBroadPhase::reserveNodes(std::size_t extra)
{
  // Grow the parallel node arrays before any size changes, so a failed
  // allocation leaves them in step. The query stack is sized here, outside
  // queries, so a deeper walk cannot allocate during a prepared step.
  const std::size_t needed = nodes_.size() + extra;
  if (nodes_.capacity() < needed) {
    nodes_.reserve(std::max(needed, 2u * nodes_.capacity()));
  }
  parents_.reserve(nodes_.capacity());
  mQueryStack.reserve(nodes_.capacity());
}

void AabbTreeBroadPhase::freeNode(NodeIndex nodeIndex)
{
  assert(nodeIndex >= 0 && static_cast<std::size_t>(nodeIndex) < nodes_.size());
  parents_[nodeIndex] = freeList_;
  nodes_[nodeIndex].height = -1;
  freeList_ = nodeIndex;
  --nodeCount_;
}

void AabbTreeBroadPhase::insertLeaf(NodeIndex leafIndex)
{
  if (root_ == kInvalidNode) {
    root_ = leafIndex;
    parents_[leafIndex] = kInvalidNode;
    return;
  }

  // Copy values before allocateNode(), which may reallocate nodes_.
  const Aabb leafAabb = nodes_[leafIndex].fatAabb;
  const NodeIndex siblingIndex = findBestSibling(leafAabb);
  const Aabb siblingAabb = nodes_[siblingIndex].fatAabb;
  const NodeIndex oldParent = parents_[siblingIndex];
  const NodeIndex newParent = allocateNode();

  parents_[newParent] = oldParent;
  nodes_[newParent].fatAabb = combine(leafAabb, siblingAabb);
  nodes_[newParent].height = nodes_[siblingIndex].height + 1;
  nodes_[newParent].maxObjectId = std::max(
      nodes_[siblingIndex].maxObjectId, nodes_[leafIndex].maxObjectId);

  if (oldParent != kInvalidNode) {
    if (nodes_[oldParent].left == siblingIndex) {
      nodes_[oldParent].left = newParent;
    } else {
      nodes_[oldParent].right = newParent;
    }
    nodes_[newParent].left = siblingIndex;
    nodes_[newParent].right = leafIndex;
    parents_[siblingIndex] = newParent;
    parents_[leafIndex] = newParent;
  } else {
    nodes_[newParent].left = siblingIndex;
    nodes_[newParent].right = leafIndex;
    parents_[siblingIndex] = newParent;
    parents_[leafIndex] = newParent;
    root_ = newParent;
  }

  rebalance(parents_[leafIndex]);
}

void AabbTreeBroadPhase::removeLeaf(NodeIndex leafIndex)
{
  if (leafIndex == root_) {
    root_ = kInvalidNode;
    return;
  }

  const NodeIndex parent = parents_[leafIndex];
  const NodeIndex grandParent = parents_[parent];
  const NodeIndex sibling = (nodes_[parent].left == leafIndex)
                                ? nodes_[parent].right
                                : nodes_[parent].left;

  if (grandParent != kInvalidNode) {
    if (nodes_[grandParent].left == parent) {
      nodes_[grandParent].left = sibling;
    } else {
      nodes_[grandParent].right = sibling;
    }
    parents_[sibling] = grandParent;
    freeNode(parent);
    rebalance(grandParent);
  } else {
    root_ = sibling;
    parents_[sibling] = kInvalidNode;
    freeNode(parent);
  }
}

AabbTreeBroadPhase::NodeIndex AabbTreeBroadPhase::findBestSibling(
    const Aabb& aabb) const
{
  NodeIndex index = root_;

  while (!nodes_[index].isLeaf()) {
    const NodeIndex left = nodes_[index].left;
    const NodeIndex right = nodes_[index].right;

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

void AabbTreeBroadPhase::rebalance(NodeIndex nodeIndex)
{
  while (nodeIndex != kInvalidNode) {
    nodeIndex = balance(nodeIndex);

    const NodeIndex left = nodes_[nodeIndex].left;
    const NodeIndex right = nodes_[nodeIndex].right;

    assert(left != kInvalidNode);
    assert(right != kInvalidNode);

    nodes_[nodeIndex].height
        = 1 + std::max(nodes_[left].height, nodes_[right].height);
    nodes_[nodeIndex].fatAabb
        = combine(nodes_[left].fatAabb, nodes_[right].fatAabb);

    nodes_[nodeIndex].maxObjectId
        = std::max(nodes_[left].maxObjectId, nodes_[right].maxObjectId);
    nodeIndex = parents_[nodeIndex];
  }
}

AabbTreeBroadPhase::NodeIndex AabbTreeBroadPhase::balance(NodeIndex nodeIndex)
{
  assert(nodeIndex != kInvalidNode);

  Node& A = nodes_[nodeIndex];
  if (A.isLeaf() || A.height < 2) {
    return nodeIndex;
  }

  const NodeIndex iB = A.left;
  const NodeIndex iC = A.right;
  assert(iB >= 0 && static_cast<std::size_t>(iB) < nodes_.size());
  assert(iC >= 0 && static_cast<std::size_t>(iC) < nodes_.size());

  Node& B = nodes_[iB];
  Node& C = nodes_[iC];

  const int balance = static_cast<int>(C.height) - static_cast<int>(B.height);

  if (balance > 1) {
    const NodeIndex iF = C.left;
    const NodeIndex iG = C.right;
    assert(iF >= 0 && static_cast<std::size_t>(iF) < nodes_.size());
    assert(iG >= 0 && static_cast<std::size_t>(iG) < nodes_.size());
    Node& F = nodes_[iF];
    Node& G = nodes_[iG];

    C.left = nodeIndex;
    parents_[iC] = parents_[nodeIndex];
    parents_[nodeIndex] = iC;

    if (parents_[iC] != kInvalidNode) {
      if (nodes_[parents_[iC]].left == nodeIndex) {
        nodes_[parents_[iC]].left = iC;
      } else {
        assert(nodes_[parents_[iC]].right == nodeIndex);
        nodes_[parents_[iC]].right = iC;
      }
    } else {
      root_ = iC;
    }

    if (F.height > G.height) {
      C.right = iF;
      A.right = iG;
      parents_[iG] = nodeIndex;
      A.fatAabb = combine(B.fatAabb, G.fatAabb);
      C.fatAabb = combine(A.fatAabb, F.fatAabb);
      A.height = 1 + std::max(B.height, G.height);
      A.maxObjectId = std::max(B.maxObjectId, G.maxObjectId);
      C.height = 1 + std::max(A.height, F.height);
      C.maxObjectId = std::max(A.maxObjectId, F.maxObjectId);
    } else {
      C.right = iG;
      A.right = iF;
      parents_[iF] = nodeIndex;
      A.fatAabb = combine(B.fatAabb, F.fatAabb);
      C.fatAabb = combine(A.fatAabb, G.fatAabb);
      A.height = 1 + std::max(B.height, F.height);
      A.maxObjectId = std::max(B.maxObjectId, F.maxObjectId);
      C.height = 1 + std::max(A.height, G.height);
      C.maxObjectId = std::max(A.maxObjectId, G.maxObjectId);
    }

    return iC;
  }

  if (balance < -1) {
    const NodeIndex iD = B.left;
    const NodeIndex iE = B.right;
    assert(iD >= 0 && static_cast<std::size_t>(iD) < nodes_.size());
    assert(iE >= 0 && static_cast<std::size_t>(iE) < nodes_.size());
    Node& D = nodes_[iD];
    Node& E = nodes_[iE];

    B.left = nodeIndex;
    parents_[iB] = parents_[nodeIndex];
    parents_[nodeIndex] = iB;

    if (parents_[iB] != kInvalidNode) {
      if (nodes_[parents_[iB]].left == nodeIndex) {
        nodes_[parents_[iB]].left = iB;
      } else {
        assert(nodes_[parents_[iB]].right == nodeIndex);
        nodes_[parents_[iB]].right = iB;
      }
    } else {
      root_ = iB;
    }

    if (D.height > E.height) {
      B.right = iD;
      A.left = iE;
      parents_[iE] = nodeIndex;
      A.fatAabb = combine(C.fatAabb, E.fatAabb);
      B.fatAabb = combine(A.fatAabb, D.fatAabb);
      A.height = 1 + std::max(C.height, E.height);
      A.maxObjectId = std::max(C.maxObjectId, E.maxObjectId);
      B.height = 1 + std::max(A.height, D.height);
      B.maxObjectId = std::max(A.maxObjectId, D.maxObjectId);
    } else {
      B.right = iE;
      A.left = iD;
      parents_[iD] = nodeIndex;
      A.fatAabb = combine(C.fatAabb, D.fatAabb);
      B.fatAabb = combine(A.fatAabb, E.fatAabb);
      A.height = 1 + std::max(C.height, D.height);
      A.maxObjectId = std::max(C.maxObjectId, D.maxObjectId);
      B.height = 1 + std::max(A.height, E.height);
      B.maxObjectId = std::max(A.maxObjectId, E.maxObjectId);
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

void AabbTreeBroadPhase::queryOverlappingImpl(
    const Aabb& aabb,
    std::vector<std::size_t>& results,
    std::size_t minId,
    bool higherIdsOnly) const
{
  if (root_ == kInvalidNode) {
    return;
  }

  auto& stack = mQueryStack;
  stack.clear();
  stack.push_back(root_);
  while (!stack.empty()) {
    const NodeIndex nodeIndex = stack.back();
    stack.pop_back();
    const Node& node = nodes_[nodeIndex];
    if ((higherIdsOnly && node.maxObjectId <= minId)
        || !node.fatAabb.overlaps(aabb)) {
      continue;
    }
    if (node.isLeaf()) {
      if (overlapsTight(node.maxObjectId, aabb)) {
        results.push_back(node.maxObjectId);
      }
    } else {
      stack.push_back(node.right);
      stack.push_back(node.left);
    }
  }
}

bool AabbTreeBroadPhase::validateStructure(NodeIndex nodeIndex) const
{
  if (nodeIndex == kInvalidNode) {
    return true;
  }

  if (nodeIndex == root_ && parents_[nodeIndex] != kInvalidNode) {
    return false;
  }

  const Node& node = nodes_[nodeIndex];

  if (node.isLeaf()) {
    if (node.left != kInvalidNode || node.right != kInvalidNode) {
      return false;
    }
    if (node.height != 0) {
      return false;
    }
    const std::size_t id = node.maxObjectId;
    return id < objectToNode_.size() && objectToNode_[id] == nodeIndex;
  }

  if (node.left == kInvalidNode || node.right == kInvalidNode) {
    return false;
  }

  if (parents_[node.left] != nodeIndex) {
    return false;
  }
  if (parents_[node.right] != nodeIndex) {
    return false;
  }

  const std::int32_t expectedHeight
      = 1 + std::max(nodes_[node.left].height, nodes_[node.right].height);
  if (node.maxObjectId
          != std::max(
              nodes_[node.left].maxObjectId, nodes_[node.right].maxObjectId)
      || node.height != expectedHeight) {
    return false;
  }

  return validateStructure(node.left) && validateStructure(node.right);
}

} // namespace dart::collision::native
