/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_DYNAMICS_SPECIALIZEDNODEMANAGER_H_
#define DART_DYNAMICS_SPECIALIZEDNODEMANAGER_H_

#include "dart/dynamics/detail/BasicNodeManager.h"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

/// Declaration of the variadic template
template <bool amSkeleton, class... OtherSpecNodes>
class SpecializedNodeManager { };

/// SpecializedNodeManager allows classes that inherit BodyNode to have
/// constant-time access to a specific type of Node
template <bool amSkeleton, class SpecNode>
class SpecializedNodeManager<amSkeleton, SpecNode> :
    public virtual detail::BasicNodeManager<amSkeleton>
{
public:

  static constexpr bool isSkeleton = amSkeleton;
  using Basic = typename detail::BasicNodeManager<isSkeleton>;
  template <class T> using type = typename Basic::template type<T>;

  /// Default constructor
  SpecializedNodeManager();

  /// Get the number of Nodes corresponding to the specified type
  template <class NodeType>
  size_t getNumNodes() const;

  /// Get the Node of the specified type and the specified index
  template <class NodeType>
  NodeType* getNode(size_t index);

  /// Get the Node of the specified type and the specified index
  template <class NodeType>
  const NodeType* getNode(size_t index) const;

  /// THIS FUNCTION IS ONLY AVAILABLE FOR SKELETONS. Get the number of Nodes of
  /// the specified type that are in the treeIndexth tree of this Skeleton
  template <class NodeType>
  typename std::enable_if<amSkeleton, size_t>::type getNumNodes(
      size_t treeIndex) const;

  /// THIS FUNCTION IS ONLY AVAILABLE FOR SKELETONS. Get the nodeIndexth Node
  /// of the specified type within the tree of treeIndex.
  template <class NodeType>
  typename std::enable_if<amSkeleton, NodeType*>::type getNode(
      size_t treeIndex, size_t nodeIndex);

  /// THIS FUNCTION IS ONLY AVAILABLE FOR SKELETONS. Get the nodeIndexth Node
  /// of the specified type within the tree of treeIndex.
  template <class NodeType>
  typename std::enable_if<amSkeleton, NodeType*>::type getNode(
      size_t treeIndex, size_t nodeIndex) const;

  /// THIS FUNCTION IS ONLY AVAILABLE FOR SKELETONS. Get the Node of the
  /// specified type with the given name.
  template <class NodeType>
  typename std::enable_if<amSkeleton, NodeType*>::type getNode(
      const std::string& name);

  /// THIS FUNCTION IS ONLY AVAILABLE FOR SKELETONS. Get the Node of the
  /// specified type with the given name.
  template <class NodeType>
  typename std::enable_if<amSkeleton, const NodeType*>::type getNode(
      const std::string& name) const;

protected:

  /// Redirect to BasicNodeManager::getNumNodes()
  template <class NodeType>
  size_t _getNumNodes(type<NodeType>) const;

  /// Specialized implementation of getNumNodes()
  size_t _getNumNodes(type<SpecNode>) const;

  /// Redirect to BasicNodeManager::getNode(size_t)
  template <class NodeType>
  NodeType* _getNode(type<NodeType>, size_t index);

  /// Specialized implementation of getNode(size_t)
  SpecNode* _getNode(type<SpecNode>, size_t index);

  /// Iterator that points to the map location of the specialized Node type
  typename Basic::NodeMap::iterator mSpecNodeIterator;


};

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/SpecializedNodeManager.h"

#endif // DART_DYNAMICS_SPECIALIZEDNODEMANAGER_H_
