/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_COLLISION_DART_DYNAMICBVTREE_HPP_
#define DART_COLLISION_DART_DYNAMICBVTREE_HPP_

#include <vector>
#include <queue>
#include <numeric>
#include <stack>

#include "dart/common/Console.hpp"
#include "dart/math/Aabb.hpp"
#include "dart/math/Ray.hpp"
#include "dart/collision/dart/DynamicBvNode.hpp"

namespace dart {
namespace collision {

class DynamicAabbTreeOverlapCallback
{
public:
  virtual void notifyOverlappingNode(int nodeId) = 0;
};

class DynamicBvTreeRaycastCallback
{
public:
  virtual double notifyOverlappingNode(int nodeIndex, const math::Ray& ray) = 0;
};

/// DynamicBvTree encodes bounding volume hierarchy. This class is also a
/// container of shapes. The tree adds and removes objects in incremental
/// manner. The incremental tree modification is suitable when the objects are
/// added and removed frequently.
template <typename Bv = math::Aabb>
class DynamicBvTree
{
public:
  using NodeType = DynamicBvNode<Bv>;

  /// Constructor
  DynamicBvTree(std::size_t reserveObjects = 16u, double bvMargin = double(0.0));

  //----------------------------------------------------------------------------
  /// \{ \name Properties
  //----------------------------------------------------------------------------

  /// Sets bounding volume margin that allows the object to move within the
  /// margin without triggering the tree updating which might be cost.
  void setBvMargin(double margin);
  // TODO: add a flag whether updating the bounding volumes of existing objects

  /// Returns bounding volume.
  double getBvMargin() const;

  /// Reserve
  void reserve(std::size_t size);

  /// \}

  /// Adds an object to this tree.
  std::size_t addObject(const Bv& bv, void* data);

  /// Removes an object from this tree.
  ///
  /// This class doesn't check the validity of the node index so passing valid
  /// index is your responsiblity.
  void removeObject(std::size_t node);

  /// Returns true if the tree is empty
  bool isEmpty() const;

  /// Returns the number of nodes in this tree.
  std::size_t getNumPopulatedNodes() const;

  /// Returns the number of leaf nodes in this tree.
  std::size_t getNumLeafNodes() const;

  /// Returns the number of objects added to this tree. Identical to
  /// getNumLeafNodes().
  std::size_t getNumObjects() const;

  /// Clears this tree.
  void clear();

  const Bv& getBv(std::size_t node) const;

  //----------------------------------------------------------------------------
  /// \{ Balancing
  //----------------------------------------------------------------------------

  void balanceBottomUp();

  void balanceBottomUpBruteForce();

  std::size_t balanceBottomUpBruteForceWorker(
      const std::vector<std::size_t>::iterator& begin,
      const std::vector<std::size_t>::iterator& end);

  void balanceTopDown();

  /// \}

  void collide(const math::Aabb& aabb,
               DynamicAabbTreeOverlapCallback& callback) const;

  /// Ray casting method
  void raycast(const math::Ray& ray, DynamicBvTreeRaycastCallback& callback) const;

protected:
  //----------------------------------------------------------------------------
  /// \{ \name Node manipulation
  //
  // Dev note: The node is designed to holds the indices of the children for
  // better performance rather than holding their pointers. For this reason,
  // most node manipulation should be done by tree class (i.e.,
  // DynamicBvTree<Bv>) since nodes are unable to access the children node
  // directly.
  //----------------------------------------------------------------------------

  /// Claims an internal node and return the index of the node.
  ///
  /// This function claim a node from the lastly reclaimed node back. If there
  /// is no reclaimed node, then create a new node.
  std::size_t claimInternalNode(std::size_t parent,
                                std::size_t child1,
                                std::size_t child2,
                                const Bv& bv);

  /// Claims an internal node and return the index of the node.
  ///
  /// This function claim a node from the lastly reclaimed node back. If there
  /// is no reclaimed node, then create a new node.
  std::size_t claimInternalNode(std::size_t parent,
                                std::size_t child1,
                                std::size_t child2,
                                Bv&& bv);

  /// Claims an leaf node and return the index of the node.
  ///
  /// This function claim a node from the lastly reclaimed node back. If there
  /// is no reclaimed node, then create a new node.
  std::size_t claimLeafNode(std::size_t parent, const Bv& bv, void* data);

  /// Claims an leaf node and return the index of the node.
  ///
  /// This function claim a node from the lastly reclaimed node back. If there
  /// is no reclaimed node, then create a new node.
  std::size_t claimLeafNode(std::size_t parent, Bv&& bv, void* data);

  /// Reclaims an internal node.
  ///
  /// The reclaimed node is not actually removed from mNodes. Instead,
  /// mFreeNodeIndex points the reclaimed node's index so it can be claimed
  /// later. If already mFreeNodeIndex points other reclaimed node, then the
  /// most newly reclaimed node stores the previously reclaimed node index in
  /// `next`. This allows to keep track all the reclaimed nodes sequently.
  void reclaimInternalNode(std::size_t node);

  /// Reclaims a leaf node.
  void reclaimLeafNode(std::size_t node);

  /// Inserts a leaf node to this tree.
  ///
  /// More specifically, this function
  void insertLeaf(std::size_t newLeaf, std::size_t searchPoint);

  /// Removes a leaf node from the tree.
  ///
  /// More specifically, this function removes the leaf's parent and replaces it
  /// with the leaf's sibling. After this operation, the bounding volume needs
  /// to be updated. See also updateAncenstorBvsDueToRemoval().
  void removeLeaf(std::size_t leaf);

  /// Create an internal node whose parent is `parent` and children are `child1`
  /// and `child2`.
  std::size_t branchAt(std::size_t branching, std::size_t newLeaf);

  /// Returns the sibling node index of `node`.
  std::size_t getSiblingOf(std::size_t node) const;

  /// Returns true if `node` is the root node.
  bool isRootNode(std::size_t node) const;

  /// Updates the bounding volumes of the ancestors of `target` after a new leaf
  /// node is added to this tree. `updatedChild` is used to get a hint how the
  /// bounding volumes are changed.
  void updateAncenstorBvsDueToAddition(std::size_t firstAncestor,
                                       std::size_t updatedChild);

  /// Updates the bounding volumes of the ancestors of `target` after a leaf
  /// node is removed from to this tree.
  void updateAncenstorBvsDueToRemoval(std::size_t firstAncestor);

  /// Moves all the leaf nodes in the node array to the front without free
  /// nodes, and reclaims all the free nodes.
  void removeAllInternalNodes();

  void runTopDownWorker();

  /// \}

protected:
  static constexpr std::size_t DefaultNumReservingNodes{16u};

  double mBvMargin;

  /// Node array
  std::vector<NodeType> mNodes;

  /// Index to the root node. Initially it is invalid index (-1).
  std::size_t mRoot;

  /// Index to a free node if it exists, or InvalidNodeIndex.
  ///
  /// Free node is a allocated node before as one of leaf or internal node, but
  /// now it's reclaimed.
  std::size_t mFreeNodeIndex;

  /// Number of leaf nodes
  std::size_t mNumLeafNodes;

private:
  /// Cach data for balancing.
  std::queue<std::size_t> mQueue;
};

//==============================================================================
/// Returns node1 if query is closer to node1 than node2.
template <typename Bv>
std::size_t selectCloser(std::size_t /*query*/,
                         std::size_t /*node1*/,
                         std::size_t /*node2*/,
                         const std::vector<DynamicBvNode<Bv>>& /*nodes*/)
{
  // TODO: warning?
  return 0;
}

//==============================================================================
template <>
inline std::size_t selectCloser(std::size_t query,
                                std::size_t node1,
                                std::size_t node2,
                                const std::vector<DynamicBvNode<math::Aabb>>& nodes)
{
  const auto& bv = nodes[query].bv;
  const auto& bv1 = nodes[node1].bv;
  const auto& bv2 = nodes[node2].bv;

  const Eigen::Vector3d v = bv.getMin() + bv.getMax();
  const Eigen::Vector3d v1 = v - (bv1.getMin() + bv1.getMax());
  const Eigen::Vector3d v2 = v - (bv2.getMin() + bv2.getMax());

  const auto d1 = v1.cwiseAbs().sum();
  const auto d2 = v2.cwiseAbs().sum();

  return (d1 < d2) ? node1 : node2;
}

//==============================================================================
template <typename Bv>
DynamicBvTree<Bv>::DynamicBvTree(std::size_t reserveObjects, double bvMargin)
  : mBvMargin{bvMargin}, mRoot{DynamicBvNode<Bv>::invalidIndex()},
    mFreeNodeIndex{DynamicBvNode<Bv>::invalidIndex()}, mNumLeafNodes{0u}
{
  mNodes.reserve(reserveObjects);
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::DynamicBvTree::setBvMargin(double margin)
{
  if (margin < double(0.0))
  {
    dtwarn << "[DynamicBvTree::setBvMargin] Attempting to set the bounding "
              "volume marge to a negative value, which doesn't make sense. "
              "Ignoring the action.\n";
    return;
  }

  mBvMargin = margin;
}

//==============================================================================
template <typename Bv>
double DynamicBvTree<Bv>::getBvMargin() const
{
  return mBvMargin;
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::reserve(std::size_t size)
{
  if (size < 2u)
    return;

  // Assume worst case
  mNodes.reserve(2u * size - 1u);
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::addObject(const Bv& bv, void* data)
{
  // TODO: check duplicates

  assert(data);

  // Create a leaf node whoes parent is not specified yet
  const auto newLeaf =
      claimLeafNode(DynamicBvNode<Bv>::invalidIndex(), bv, data);

  // Inflate the AABB
  mNodes[newLeaf].bv.inflate(mBvMargin);

  // Add the leaf node to the tree. The parent should be determined here.
  insertLeaf(newLeaf, mRoot);

  assert(isRootNode(newLeaf) ||
         mNodes[newLeaf].parent != DynamicBvNode<Bv>::invalidIndex());

  return newLeaf;
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::removeObject(std::size_t node)
{
  assert(mNodes[node].isLeaf());

  removeLeaf(node);

  reclaimLeafNode(node);
}

//==============================================================================
template <typename Bv>
bool DynamicBvTree<Bv>::isEmpty() const
{
  return (mNumLeafNodes == 0u);
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::getNumPopulatedNodes() const
{
  return mNodes.size();
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::getNumLeafNodes() const
{
  return mNumLeafNodes;
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::getNumObjects() const
{
  return getNumLeafNodes();
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::clear()
{
  mNodes.clear();
  mNodes.reserve(DefaultNumReservingNodes); // TODO: Necessary?
  mNumLeafNodes = 0u;
  mFreeNodeIndex = DynamicBvNode<Bv>::invalidIndex();
  mRoot = DynamicBvNode<Bv>::invalidIndex();
}

//==============================================================================
template <typename Bv>
const Bv& DynamicBvTree<Bv>::getBv(std::size_t node) const
{
  return mNodes[node].bv;
}

//==============================================================================
template <typename Bv>
inline Bv computeBv(std::size_t begin,
                    std::size_t end,
                    const std::vector<DynamicBvNode<Bv>>& nodes)
{
  assert(end > begin + 1u); // the number of nodes should be greater than 2.
  assert(!nodes.isEmpty());

  Bv vol = nodes[begin].bv;

  for (auto i = begin + 1u; i < end; ++i)
    vol += nodes[i].bv;

  return vol;
}

//================================================================================
template <typename Bv>
class nodeLess final
{
public:
  nodeLess(const std::vector<DynamicBvNode<Bv>>& nodes, size_t axisIndex)
    : mNodes(nodes), mAxisIndex(axisIndex)
  {
  }

  bool operator()(const DynamicBvNode<Bv>& i, const DynamicBvNode<Bv>& j) const
  {
    if (i.bv.getCenter()[mAxisIndex] < j.bv.getCenter()[mAxisIndex])
      return true;

    return false;
  }

private:
  const std::vector<DynamicBvNode<Bv>>& mNodes;
  std::size_t mAxisIndex;
};

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::balanceBottomUp()
{
  if (mNumLeafNodes < 2u)
    return;

  balanceBottomUpBruteForce();
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::balanceBottomUpBruteForce()
{
  assert(mNumLeafNodes > 1u);

  removeAllInternalNodes();

  std::vector<std::size_t> indices(mNumLeafNodes);
  std::iota(indices.begin(), indices.end(), 0u);

  balanceBottomUpBruteForceWorker(indices.begin(), indices.end());
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::balanceBottomUpBruteForceWorker(
    const typename std::vector<std::size_t>::iterator& begin,
    const typename std::vector<std::size_t>::iterator& end)
{
  assert(begin + 1u < end);

  auto currEnd = end;

  // While there are at least two leaf nodes to balance
  while (begin + 1u < currEnd)
  {
    std::vector<std::size_t>::iterator leafA;
    std::vector<std::size_t>::iterator leafB;
    auto minSize = std::numeric_limits<double>::max();

    for (auto itrA = begin; itrA < currEnd; ++itrA)
    {
      for (auto itrB = itrA + 1u; itrB < currEnd; ++itrB)
      {
        const auto size = (mNodes[*itrA].bv + mNodes[*itrB].bv).getSize();
        if (size < minSize)
        {
          minSize = size;
          leafA = itrA;
          leafB = itrB;
        }
      }
    }
  }

  // return the index to the root node
  return 0; // TODO:
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::balanceTopDown()
{
  if (mNumLeafNodes < 2u)
    return;

  removeAllInternalNodes();
  mRoot = DynamicBvNode<Bv>::invalidIndex();

  //  auto begin = 0u;
  //  auto end = mNumLeafNodes;
  //  auto numLeafNodes = end;

  auto curr = 0u;

  //  Bv vol = computeBv(begin, end, mNodes);

  assert(mQueue.empty());

  mQueue.push(mNodes[curr].left);
  mQueue.push(mNodes[curr].right);

  //  nodeLess<Bv> comp(mNodes, 0); // TODO: pass best axis index

  //  auto center = begin + numLeafNodes / 2;
  //  mNodes.sortFor(begin, center, end, comp);

  // Choice of partitioning axis
  // TODO: make this a template function for BV type and policy (space mean,
  // object mean, object median)

  // Implement splitter

  // For now we implement space mean
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::collide(const math::Aabb& aabb,
                                DynamicAabbTreeOverlapCallback& callback) const
{
  if (isEmpty())
    return;

  // Create a stack with the nodes to visit
  std::stack<int> stack;
  auto nodeIndexToVisit = mRoot;

  // While there are still nodes to visit
  while (true)
  {
    const auto* nodeToVisit = mNodes[nodeIndexToVisit];

    if (nodeToVisit->overlapsWith(aabb))
    {
      if (nodeToVisit->isLeaf())
      {
        // Notify the broad-phase about a new potential overlapping pair
        callback.notifyOverlappingNode(nodeIndexToVisit);
      }
      else
      {
        nodeIndexToVisit = nodeToVisit->children[0];

        // Push its children in the stack of nodes to visit
        stack.push(nodeToVisit->children[1]);

        continue;
      }
    }

    if (stack.empty())
      break;

    nodeIndexToVisit = stack.top();
    stack.pop();
  }

  // TODO(JS): recursive version could be faster. need profiling for sure.
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::raycast(const math::Ray& ray,
                                DynamicBvTreeRaycastCallback& callback) const
{
  // Implementation of RTCD 6.3.2.

  if (isEmpty())
    return;

  std::stack<int> stack;
  auto maxFraction = ray.fraction;
  auto nodeIndex = mRoot;

  while (true)
  {
    const auto* node = mNodes[nodeIndex];

    math::Ray rayTemp(ray.from, ray.to, maxFraction);

    if (node->aabb.overlapsWith(ray))
    {
      if (node->isLeaf())
      {
        auto hitFraction = callback.notifyOverlappingNode(nodeIndex, rayTemp);

        if (hitFraction == double(0.0))
          return;

        if (hitFraction > double(0.0))
        {
          if (hitFraction < maxFraction)
            maxFraction = hitFraction;
        }
      }
      else
      {
        nodeIndex = node->children[0];

        // Push its children in the stack of nodes to visit
        stack.push(node->children[1]);

        continue;
      }
    }

    if (stack.empty())
      break;

    nodeIndex = stack.top();
    stack.pop();
  }
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::branchAt(std::size_t branching,
                                        std::size_t newLeaf)
{
  const auto grandParent = mNodes[branching].parent;

  // Creat an new internal node to be placed at the branching node. The left
  // and right childeren of the created node are branching node and new leaf
  // node, respectively.
  const auto branched =
      claimInternalNode(grandParent,
                        branching,
                        newLeaf,
                        mNodes[branching].bv + mNodes[newLeaf].bv);

  // Connect the created node to the children.
  mNodes[branching].parent = branched;
  mNodes[newLeaf].parent = branched;

  // Update the bounding volumes of the ancesters of the parent node.
  if (grandParent == DynamicBvNode<Bv>::invalidIndex())
  {
    // The branching node was the root.
    mRoot = branched;
  }
  else
  {
    // Connect branched node to the parent of branching node.
    if (mNodes[grandParent].left == branching)
      mNodes[grandParent].left = branched;
    else
      mNodes[grandParent].right = branched;

    // Update the bounding volumes from the parent of branching node to the
    // root if neccessary.
    updateAncenstorBvsDueToAddition(grandParent, branched);
  }

  return branched;
}

//==============================================================================
template <typename Bv>
inline std::size_t findBestSibling(std::size_t node,
                                   std::size_t searchPoint,
                                   const std::vector<DynamicBvNode<Bv>>& nodes)
{
  assert(DynamicBvNode<Bv>::invalidIndex() != searchPoint);

  auto sibling = searchPoint;

  while (nodes[sibling].isInternal())
  {
    sibling =
        selectCloser(node, nodes[sibling].left, nodes[sibling].right, nodes);
  }

  return sibling;
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::claimInternalNode(std::size_t parent,
                                                 std::size_t left,
                                                 std::size_t right,
                                                 const Bv& bv)
{
  std::size_t claimedNodeIndex;

  const auto hasFreeNode =
      (DynamicBvNode<Bv>::invalidIndex() != mFreeNodeIndex);

  if (!hasFreeNode)
  {
    claimedNodeIndex = mNodes.size();
    mNodes.emplace_back(parent, left, right, bv);
  }
  else
  {
    claimedNodeIndex = mFreeNodeIndex;
    mFreeNodeIndex = mNodes[claimedNodeIndex].next;
    mNodes[claimedNodeIndex].setAsInternalNode(parent, left, right, bv);
  }

  return claimedNodeIndex;
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::claimInternalNode(std::size_t parent,
                                                 std::size_t left,
                                                 std::size_t right,
                                                 Bv&& bv)
{
  std::size_t claimedNodeIndex;

  const auto hasFreeNode =
      (DynamicBvNode<Bv>::invalidIndex() != mFreeNodeIndex);

  if (!hasFreeNode)
  {
    claimedNodeIndex = mNodes.size();
    mNodes.emplace_back(parent, left, right, std::forward<Bv>(bv));
  }
  else
  {
    claimedNodeIndex = mFreeNodeIndex;
    mFreeNodeIndex = mNodes[claimedNodeIndex].next;
    mNodes[claimedNodeIndex].setAsInternalNode(
        parent, left, right, std::forward<Bv>(bv));
  }

  return claimedNodeIndex;
}

//==============================================================================
template <typename Bv>
std::size_t
DynamicBvTree<Bv>::claimLeafNode(std::size_t parent, const Bv& bv, void* data)
{
  std::size_t claimedNodeIndex;

  const auto hasFreeNode =
      (DynamicBvNode<Bv>::invalidIndex() != mFreeNodeIndex);

  if (!hasFreeNode)
  {
    claimedNodeIndex = mNodes.size();
    mNodes.emplace_back(parent, bv, data);
  }
  else
  {
    claimedNodeIndex = mFreeNodeIndex;
    mFreeNodeIndex = mNodes[claimedNodeIndex].next;
    mNodes[claimedNodeIndex].setAsLeafNode(parent, bv, data);
  }

  ++mNumLeafNodes;

  return claimedNodeIndex;
}

//==============================================================================
template <typename Bv>
std::size_t
DynamicBvTree<Bv>::claimLeafNode(std::size_t parent, Bv&& bv, void* data)
{
  std::size_t claimedNodeIndex;

  const auto hasFreeNode =
      (DynamicBvNode<Bv>::invalidIndex() != mFreeNodeIndex);

  if (!hasFreeNode)
  {
    claimedNodeIndex = mNodes.size();
    mNodes.emplace_back(parent, std::forward<Bv>(bv), data);
  }
  else
  {
    claimedNodeIndex = mFreeNodeIndex;
    mFreeNodeIndex = mNodes[claimedNodeIndex].next;
    mNodes[claimedNodeIndex].setAsLeafNode(parent, std::forward<Bv>(bv), data);
  }

  ++mNumLeafNodes;

  return claimedNodeIndex;
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::reclaimInternalNode(std::size_t node)
{
  mNodes[node].next = mFreeNodeIndex;

  assert(mNodes[node].right != DynamicBvNode<Bv>::invalidIndex());

  mFreeNodeIndex = node;

  // isLeaf() shouldn't return true for reclaimed node (free node). This is
  // necessary condition for extracting leaf node efficiently
  assert(!mNodes[node].isLeaf());
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::reclaimLeafNode(std::size_t node)
{
  mNodes[node].next = mFreeNodeIndex;

  assert(mNodes[node].right == DynamicBvNode<Bv>::invalidIndex());
  mNodes[node].right = 0u; // any value not InvalidenodeIndex

  mFreeNodeIndex = node;

  --mNumLeafNodes;

  // isLeaf() shouldn't return true for reclaimed node (free node). This is
  // necessary condition for extracting leaf node efficiently
  assert(!mNodes[node].isLeaf());
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::insertLeaf(std::size_t newLeaf, std::size_t searchPoint)
{
  // If the root hasn't been added yet, just make the new leaf node to the root.
  if (DynamicBvNode<Bv>::invalidIndex() == mRoot)
  {
    assert(mNodes[newLeaf].parent == DynamicBvNode<Bv>::invalidIndex());

    mRoot = newLeaf;
    return;
  }

  // Find the closest leaf node to the new leaf node. The closest leaf node will
  // become the sibling of the new leaf node.
  const auto sibling = findBestSibling(newLeaf, searchPoint, mNodes);

  // Create a new parent node of the sibling and a new node. The parent node's
  // bounding volume is the sum of the sibling and new node.
  //
  //      [g]          [g]          g: grand parent
  //      / \          / \          p: parent
  //    [s] [o]  =>  [p] [o]        n: new leaf
  //                 / \            o: other
  //     [n]       [s] [n]
  //
  branchAt(sibling, newLeaf);
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::removeLeaf(std::size_t leaf)
{
  if (isRootNode(leaf))
  {
    mRoot = DynamicBvNode<Bv>::invalidIndex();
    return;
  }

  // Setup parent, grandParent and sibling
  auto parent = mNodes[leaf].parent;
  auto grandParent = mNodes[parent].parent;
  auto sibling = getSiblingOf(leaf);

  // Replace parent with sibling
  if (isRootNode(parent))
  {
    //
    //      [p]            [x]           p: parent
    //      / \    =>                    r: removing
    //    [s] [r]        [s] [r]         s: sibling
    //    / \            / \             o: others
    //  [o] [o]        [o] [o]
    //
    // [s] becomes the new root, and [p] will be removed at the end of this
    // function. [r] is still alive.

    mRoot = sibling;
    mNodes[sibling].parent = DynamicBvNode<Bv>::invalidIndex();
  }
  else
  {
    //
    //      [g]             [g]          g: grand parent
    //      / \             / \          p: parent
    //    [p] [o]  =>   [x]/  [o]        r: removing
    //    / \             /              s: sibling
    //  [s] [r]         [s] [r]          o: others
    //
    // [s] is now the new child of [g], and [p] is being removed at the end of
    // this function. [r] is still alive.

    // Connect grand parent to sibling
    if (mNodes[grandParent].left == parent)
      mNodes[grandParent].left = sibling;
    else
      mNodes[grandParent].right = sibling;

    // Connect sibling to grand parent
    mNodes[sibling].parent = grandParent;

    updateAncenstorBvsDueToRemoval(grandParent);
  }

  // Destroy parent node
  reclaimInternalNode(parent);
}

//==============================================================================
template <typename Bv>
std::size_t DynamicBvTree<Bv>::getSiblingOf(std::size_t node) const
{
  const auto& parentNode = mNodes[mNodes[node].parent];

  if (parentNode.left == node)
    return parentNode.right;
  else
    return parentNode.left;
}

//==============================================================================
template <typename Bv>
bool DynamicBvTree<Bv>::isRootNode(std::size_t node) const
{
  assert((node == mRoot) ==
         (mNodes[node].parent == DynamicBvNode<Bv>::invalidIndex()));

  return (mNodes[node].parent == DynamicBvNode<Bv>::invalidIndex());
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::updateAncenstorBvsDueToAddition(
    std::size_t firstAncestor, std::size_t updatedChild)
{
  assert(mNodes[updatedChild].parent == firstAncestor);
  assert(updatedChild == mNodes[firstAncestor].left ||
         updatedChild == mNodes[firstAncestor].right);

  do
  {
    auto& targetNode = mNodes[firstAncestor];
    auto& updatedChildNode = mNodes[updatedChild];

    // Target has no possibility to get smaller because the target still holds
    // the original decendants and just got the newly added node. So checking if
    // it contains the updated child is a sufficient condition for early stop.
    if (targetNode.bv.contains(updatedChildNode.bv))
      break;

    auto& childNode1 = mNodes[targetNode.left];
    auto& childNode2 = mNodes[targetNode.right];

    targetNode.bv = childNode1.bv + childNode2.bv;

    updatedChild = firstAncestor;
    firstAncestor = mNodes[updatedChild].parent;

  } while (DynamicBvNode<Bv>::invalidIndex() != firstAncestor);
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::updateAncenstorBvsDueToRemoval(
    std::size_t firstAncestor)
{
  // Assume the target validity was already checked by the caller. If it's not
  // true then the do-while statement should be switched to while-do statement.
  assert(DynamicBvNode<Bv>::invalidIndex() != firstAncestor);

  do
  {
    auto& targetNode = mNodes[firstAncestor];

    auto& childNode1 = mNodes[targetNode.left];
    auto& childNode2 = mNodes[targetNode.right];

    const auto newBv = childNode1.bv + childNode2.bv;

    // Target has possibility to get smaller because it lost one decentant node.
    // So the updating can stop only when either of that the target has exactly
    // same with the children sum or that the target is larger than the children
    // sum with a small tolerance (but not smaller that the children sum).
    if (false) // TODO: Add the check
      break;

    targetNode.bv = newBv;

    firstAncestor = mNodes[firstAncestor].parent;

  } while (DynamicBvNode<Bv>::invalidIndex() != firstAncestor);
}

//==============================================================================
template <typename Bv>
void DynamicBvTree<Bv>::removeAllInternalNodes()
{
  std::vector<NodeType> newNodes;
  newNodes.reserve(mNodes.capacity());

  for (auto& node : mNodes)
  {
    if (node.isLeaf())
      newNodes.emplace_back(std::move(node));
  }

  assert(mNodes.size() >= newNodes.size());

  mNodes = std::move(newNodes);

  assert(getNumLeafNodes() == getNumPopulatedNodes());
}

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_DYNAMICBVTREE_HPP_
