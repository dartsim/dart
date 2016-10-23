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

#ifndef DART_COLLISION_DART_DYNAMICBVNODE_HPP_
#define DART_COLLISION_DART_DYNAMICBVNODE_HPP_

#include <vector>

#include "dart/math/Aabb.hpp"

namespace dart {
namespace collision {

/// Node class for DynamicBvTree.
///
/// A node can be one of following state:
///
/// - State (1): free node
///
///   - next/parent: index to the next free node (parent shouldn't be used)
///   - left/data  : undefined
///   - right      : 0
///
/// - State (2): root + leaf
///
///   - next/parent: the invalid index (-1) (next shouldn't be used)
///   - left/data  : pointer to the object (left shouldn't be used)
///   - right      : the invalid index (-1) (this is used for leaf check)
///
/// - State (3): root + internal node
///
///   - next/parent: the invalid index (-1) (next shouldn't be used)
///   - left/data  : index to the left child (data shound't be used)
///   - right      : index to the right child
///
/// - State (4): leaf node
///
///   - next/parent: index to the parent node (next shouldn't be used)
///   - left/data  : pointer to the object (left shouldn't be used)
///   - right      : the invalid index (-1) (this is used for leaf check)
///
/// - State (5): internal node
///
///   - next/parent: index to the parent node (next shouldn't be used)
///   - left/data  : index to the left child (data shound't be used)
///   - right      : index to the right child
///
/// `right` should be zero (or any value but not (-1)) for free node. This
/// restriction is useful when extracting leaf nodes from NodeVector because
/// `right` will be (-1) only when the node is non-free and leaf node.
template <typename Bv>
struct DynamicBvNode final
{
  union
  {
    /// - State (1): index to the next free node
    /// - State (2): undefined
    /// - State (3): undefined
    /// - State (4): undefined
    /// - State (5): undefined
    std::size_t next;

    /// - State (1): undefined
    /// - State (2): the invalid index (-1)
    /// - State (3): the invalid index (-1)
    /// - State (4): index to the parent node
    /// - State (5): index to the parent node
    std::size_t parent;
  };

  union
  {
    struct
    {
      /// - State (1): undefined
      /// - State (2): undefined
      /// - State (3): index to the left child
      /// - State (4): undefined
      /// - State (5): index to the left child
      std::size_t left;

      /// - State (1): undefined
      /// - State (2): the invalid index (-1)
      /// - State (3): index to the right child
      /// - State (4): the invalid index (-1)
      /// - State (5): index to the right child
      std::size_t right;
    };

    /// - State (1): undefined
    /// - State (2): pointer to the object
    /// - State (3): undefined
    /// - State (4): pointer to the object
    /// - State (5): undefined
    void* data;
  };

  /// Bounding volume that encloses all the decendants
  Bv bv;

  std::size_t code;

  /// Constructs as an internal node
  DynamicBvNode(std::size_t parent,
                std::size_t child1,
                std::size_t child2,
                const Bv& bv);

  /// Constructs as an internal node
  DynamicBvNode(std::size_t parent,
                std::size_t child1,
                std::size_t child2,
                Bv&& bv);

  /// Constructs as a leaf node
  DynamicBvNode(std::size_t parent = 10u,
                const Bv& bv = Bv(),
                void* data = nullptr);

  /// Constructs as a leaf node
  DynamicBvNode(std::size_t parent, Bv&& bv, void* data);

  /// Destructor
  ~DynamicBvNode();

  /// Copy constructor
  DynamicBvNode(const DynamicBvNode<Bv>& other);

  /// Move constructor
  DynamicBvNode(DynamicBvNode<Bv>&& other);

  /// Copy assignment
  DynamicBvNode& operator=(const DynamicBvNode<Bv>& other);

  /// Move assignment
  DynamicBvNode& operator=(DynamicBvNode<Bv>&& other);

  static constexpr std::size_t invalidIndex();

  /// Helper function to switch to an internal node
  void setAsInternalNode(std::size_t parent,
                         std::size_t child1,
                         std::size_t child2,
                         const Bv& newBv);

  /// Helper function to switch to an internal node
  void setAsInternalNode(std::size_t parent,
                         std::size_t child1,
                         std::size_t child2,
                         Bv&& newBv);

  /// Helper function to switch to a leaf node
  void setAsLeafNode(std::size_t parent, const Bv& bv, void* newData);

  /// Helper function to switch to a leaf node
  void setAsLeafNode(std::size_t parent, Bv&& bv, void* newData);

  /// Return true if this node is a leaf. If right has the invalid index,
  /// then this node is a leaf.
  bool isLeaf() const;

  /// Return true if this node is not a leaf. This function is identical to
  /// !isLeaf().
  bool isInternal() const;
};

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>::DynamicBvNode(std::size_t parent,
                                 const Bv& newBv,
                                 void* newData)
  : parent(parent), data(newData), bv(newBv)
{
  static_assert(sizeof(std::size_t) == sizeof(void*),
                "The size of `std::size_t` and `void*` are not the same. "
                "Please report this because `BvNode` is implemented based on "
                "the assumption of that they are the same size.");

  // The result of this constructor should be the same with `setAsLeafNode(~)`.

  // `next` is overwritten by parent so don't need to be initialized.

  // `left` is overwritten by data so don't need to be initialized.

  // `right` should be `InvalidNodeIndex` to mark this node is a leaf
  // node. see also `isLeaf()`
  right = invalidIndex();
}

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>::DynamicBvNode(std::size_t parent, Bv&& newBv, void* newData)
  : parent(parent), data(newData), bv(std::forward<Bv>(newBv))
{
  static_assert(sizeof(std::size_t) == sizeof(void*),
                "The size of `std::size_t` and `void*` are not the same. "
                "Please report this because `BvNode` is implemented based on "
                "the assumption of that they are the same size.");

  // The result of this constructor should be the same with `setAsLeafNode(~)`.

  // `next` is overwritten by parent so don't need to be initialized.

  // `left` is overwritten by data so don't need to be initialized.

  // `right` should be `InvalidNodeIndex` to mark this node is a leaf
  // node. see also `isLeaf()`
  right = invalidIndex();
}

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>::DynamicBvNode(std::size_t parent,
                                 std::size_t child1,
                                 std::size_t child2,
                                 const Bv& newBv)
  : parent(parent), left(child1), right(child2), bv(newBv)
{
  // This should be the same with setAsInternalNode(~).

  // `data` is overwritten by left so don't need to be initialized.
}

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>::DynamicBvNode(std::size_t parent,
                                 std::size_t child1,
                                 std::size_t child2,
                                 Bv&& newBv)
  : parent(parent), left(child1), right(child2), bv(std::forward<Bv>(newBv))
{
  // This should be the same with setAsInternalNode(~).

  // `data` is overwritten by left so don't need to be initialized.
}

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>::DynamicBvNode(const DynamicBvNode<Bv>& other)
  : parent(other.parent), left(other.left), right(other.right), bv(other.bv),
    code(other.code)
{
  // Do nothing
}

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>::DynamicBvNode(DynamicBvNode<Bv>&& other)
  : parent(std::move(other.parent)), left(std::move(other.left)),
    right(std::move(other.right)), bv(std::forward<Bv>(other.bv)),
    code(std::move(other.code))
{
  // Do nothing
}

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>::~DynamicBvNode()
{
  // Do nothing
}

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>& DynamicBvNode<Bv>::operator=(const DynamicBvNode<Bv>& other)
{
  // check for self-assignment
  if (&other == this)
    return *this;

  parent = other.parent;
  left = other.left;
  right = other.right;
  bv = other.bv;
  code = other.code;

  return *this;
}

//==============================================================================
template <typename Bv>
DynamicBvNode<Bv>& DynamicBvNode<Bv>::operator=(DynamicBvNode<Bv>&& other)
{
  // check for self-assignment
  if (&other == this)
    return *this;

  parent = std::move(other.parent);
  left = std::move(other.left);
  right = std::move(other.right);
  bv = std::forward<Bv>(other.bv);
  code = std::move(other.code);

  return *this;
}

//==============================================================================
template <typename Bv>
constexpr std::size_t DynamicBvNode<Bv>::invalidIndex()
{
  return static_cast<std::size_t>(-1);
}

//==============================================================================
template <typename Bv>
void DynamicBvNode<Bv>::setAsInternalNode(std::size_t parentNodeIndex,
                                          std::size_t child1,
                                          std::size_t child2,
                                          const Bv& newBv)
{
  parent = parentNodeIndex;
  left = child1;
  right = child2;
  bv = newBv;
}

//==============================================================================
template <typename Bv>
void DynamicBvNode<Bv>::setAsInternalNode(std::size_t parentNodeIndex,
                                          std::size_t child1,
                                          std::size_t child2,
                                          Bv&& newBv)
{
  parent = parentNodeIndex;
  left = child1;
  right = child2;
  bv = std::forward<Bv>(newBv);
}

//==============================================================================
template <typename Bv>
void DynamicBvNode<Bv>::setAsLeafNode(std::size_t parent,
                                      const Bv& newBv,
                                      void* newData)
{
  parent = parent;
  data = newData;
  // `left` doesn't need to be initialized since it's overwritten by
  // `data`
  right = invalidIndex();
  bv = newBv;
}

//==============================================================================
template <typename Bv>
void DynamicBvNode<Bv>::setAsLeafNode(std::size_t parent,
                                      Bv&& newBv,
                                      void* newData)
{
  parent = parent;
  data = newData;
  // `left` doesn't need to be initialized since it's overwritten by
  // `data`
  right = invalidIndex();
  bv = std::forward<Bv>(newBv);
}

//==============================================================================
template <typename Bv>
bool DynamicBvNode<Bv>::isLeaf() const
{
  return (invalidIndex() == right);
}

//==============================================================================
template <typename Bv>
bool DynamicBvNode<Bv>::isInternal() const
{
  return !isLeaf();
}

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_DYNAMICBVNODE_HPP_
