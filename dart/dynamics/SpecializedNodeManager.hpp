/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_SPECIALIZEDNODEMANAGER_HPP_
#define DART_DYNAMICS_SPECIALIZEDNODEMANAGER_HPP_

#include "dart/common/Virtual.hpp"
#include "dart/dynamics/detail/BasicNodeManager.hpp"
#include "dart/dynamics/NodeManagerJoiner.hpp"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

//==============================================================================
/// Declaration of the variadic template
template <class... OtherSpecNodes>
class BodyNodeSpecializedFor { };

//==============================================================================
/// BodyNodeSpecializedFor allows classes that inherit BodyNode to
/// have constant-time access to a specific type of Node
template <class SpecNode>
class BodyNodeSpecializedFor<SpecNode> :
    public virtual detail::BasicNodeManagerForBodyNode
{
public:

  /// Default constructor
  BodyNodeSpecializedFor();

  /// Get the number of Nodes corresponding to the specified type
  template <class NodeType>
  std::size_t getNumNodes() const;

  /// Get the Node of the specified type and the specified index
  template <class NodeType>
  NodeType* getNode(std::size_t index);

  /// Get the Node of the specified type and the specified index
  template <class NodeType>
  const NodeType* getNode(std::size_t index) const;

  /// Check if this Manager is specialized for a specific type of Node
  template <class NodeType>
  static constexpr bool isSpecializedForNode();

protected:

  /// Redirect to BasicNodeManagerForBodyNode::getNumNodes()
  template <class NodeType>
  std::size_t _getNumNodes(type<NodeType>) const;

  /// Specialized implementation of getNumNodes()
  std::size_t _getNumNodes(type<SpecNode>) const;

  /// Redirect to BasicNodeManagerForBodyNode::getNode(std::size_t)
  template <class NodeType>
  NodeType* _getNode(type<NodeType>, std::size_t index);

  /// Specialized implementation of getNode(std::size_t)
  SpecNode* _getNode(type<SpecNode>, std::size_t index);

  /// Return false
  template <class NodeType>
  static constexpr bool _isSpecializedForNode(type<NodeType>);

  /// Return true
  static constexpr bool _isSpecializedForNode(type<SpecNode>);

  /// Iterator that allows direct access to the specialized Nodes
  BasicNodeManagerForBodyNode::NodeMap::iterator mSpecNodeIterator;

};

//==============================================================================
/// This is the variadic version of the BodyNodeSpecializedFor class
/// which allows you to include arbitrarily many specialized types in the
/// specialization.
template <class SpecNode1, class... OtherSpecNodes>
class BodyNodeSpecializedFor<SpecNode1, OtherSpecNodes...> :
    public NodeManagerJoinerForBodyNode<
      common::Virtual< BodyNodeSpecializedFor<SpecNode1> >,
      common::Virtual< BodyNodeSpecializedFor<OtherSpecNodes...> > > { };

//==============================================================================
/// Declaration of the variadic template
template <class... OtherSpecNodes>
class SkeletonSpecializedFor { };

//==============================================================================
/// SkeletonSpecializedForNode allows classes that inherit Skeleton to
/// have constant-time access to a specific type of Node
template <class SpecNode>
class SkeletonSpecializedFor<SpecNode> :
    public virtual detail::BasicNodeManagerForSkeleton,
    public virtual BodyNodeSpecializedFor<SpecNode>
{
public:

  using BodyNodeSpecializedFor<SpecNode>::getNode;
  using BodyNodeSpecializedFor<SpecNode>::getNumNodes;
  using BodyNodeSpecializedFor<SpecNode>::isSpecializedForNode;

  SkeletonSpecializedFor();

  /// Get the number of Nodes of the specified type that are in the treeIndexth
  /// tree of this Skeleton
  template <class NodeType>
  std::size_t getNumNodes(std::size_t treeIndex) const;

  /// Get the nodeIndexth Node of the specified type within the tree of
  /// treeIndex.
  template <class NodeType>
  NodeType* getNode(std::size_t treeIndex, std::size_t nodeIndex);

  /// Get the nodeIndexth Node of the specified type within the tree of
  /// treeIndex.
  template <class NodeType>
  const NodeType* getNode(std::size_t treeIndex, std::size_t nodeIndex) const;

  /// Get the Node of the specified type with the given name.
  template <class NodeType>
  NodeType* getNode(const std::string& name);

  /// Get the Node of the specified type with the given name.
  template <class NodeType>
  const NodeType* getNode(const std::string& name) const;

  /// Check if this Manager is specialized for a specific type of Node
  template <class T>
  static constexpr bool isSpecializedForNode();

protected:

  /// Redirect to BasicNodeManagerForSkeleton::getNumNodes(std::size_t)
  template <class NodeType>
  std::size_t _getNumNodes(type<NodeType>, std::size_t treeIndex) const;

  /// Specialized implementation of getNumNodes(std::size_t)
  std::size_t _getNumNodes(type<SpecNode>, std::size_t treeIndex) const;

  /// Redirect to BasicNodeManagerForSkeleton::getNode(std::size_t, std::size_t)
  template <class NodeType>
  NodeType* _getNode(type<NodeType>, std::size_t treeIndex, std::size_t nodeIndex);

  /// Specialized implementation of getNode(std::size_t, std::size_t)
  SpecNode* _getNode(type<SpecNode>, std::size_t treeIndex, std::size_t nodeIndex);

  /// Redirect to BasicNodeManagerForSkeleton::getNode(const std::string&)
  template <class NodeType>
  NodeType* _getNode(type<NodeType>, const std::string& name);

  /// Specialized implementation of getNode(const std::string&)
  SpecNode* _getNode(type<SpecNode>, const std::string& name);

  /// Return false
  template <class T>
  static constexpr bool _isSpecializedForNode(type<T>);

  /// Return true
  static constexpr bool _isSpecializedForNode(type<SpecNode>);

  /// std::vector of iterators that allow direct access to the specialized Nodes
  /// of each tree
  std::vector<BasicNodeManagerForBodyNode::NodeMap::iterator> mTreeSpecNodeIterators;

  /// Iterator that gives direct access to the name manager of the specialized
  /// Nodes
  NodeNameMgrMap::iterator mSpecNodeNameMgrIterator;

};

//==============================================================================
/// This is the variadic version of the SkeletonSpecializedForNode class
/// which allows you to include arbitrarily many specialized types in the
/// specialization.
template <class SpecNode1, class... OtherSpecNodes>
class SkeletonSpecializedFor<SpecNode1, OtherSpecNodes...> :
    public NodeManagerJoinerForSkeleton<
      common::Virtual< SkeletonSpecializedFor<SpecNode1> >,
      common::Virtual< SkeletonSpecializedFor<OtherSpecNodes...> > > { };

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/SpecializedNodeManager.hpp"

#endif // DART_DYNAMICS_SPECIALIZEDNODEMANAGER_HPP_
