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

#ifndef DART_DYNAMICS_DETAIL_SPECIALIZEDNODEMANAGER_HPP_
#define DART_DYNAMICS_DETAIL_SPECIALIZEDNODEMANAGER_HPP_

#include "dart/dynamics/SpecializedNodeManager.hpp"

namespace dart {
namespace dynamics {

// This preprocessor token should only be used by the unittest that is
// responsible for checking that the specialized routines are being used to
// access specialized Aspects
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
bool usedSpecializedNodeAccess;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

//==============================================================================
template <class SpecNode>
BodyNodeSpecializedFor<SpecNode>::BodyNodeSpecializedFor()
{
  mNodeMap[typeid( SpecNode )] = std::vector<Node*>();
  mSpecNodeIterator = mNodeMap.find(typeid( SpecNode ));
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
std::size_t BodyNodeSpecializedFor<SpecNode>::getNumNodes() const
{
  return _getNumNodes(type<NodeType>());
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* BodyNodeSpecializedFor<SpecNode>::getNode(std::size_t index)
{
  return _getNode(type<NodeType>(), index);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
const NodeType* BodyNodeSpecializedFor<SpecNode>::getNode(std::size_t index) const
{
  return const_cast<BodyNodeSpecializedFor<SpecNode>*>(this)->
      _getNode(type<NodeType>(), index);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
constexpr bool BodyNodeSpecializedFor<SpecNode>::isSpecializedForNode()
{
  return _isSpecializedForNode(type<NodeType>());
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
std::size_t BodyNodeSpecializedFor<SpecNode>::_getNumNodes(type<NodeType>) const
{
  return detail::BasicNodeManagerForBodyNode::getNumNodes<NodeType>();
}

//==============================================================================
template <class SpecNode>
std::size_t BodyNodeSpecializedFor<SpecNode>::_getNumNodes(type<SpecNode>) const
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  return mSpecNodeIterator->second.size();
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* BodyNodeSpecializedFor<SpecNode>::_getNode(type<NodeType>, std::size_t index)
{
  return detail::BasicNodeManagerForBodyNode::getNode<NodeType>(index);
}

//==============================================================================
template <class SpecNode>
SpecNode* BodyNodeSpecializedFor<SpecNode>::_getNode(type<SpecNode>, std::size_t index)
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  return static_cast<SpecNode*>(
        getVectorObjectIfAvailable(index, mSpecNodeIterator->second));
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
constexpr bool BodyNodeSpecializedFor<SpecNode>::_isSpecializedForNode(type<NodeType>)
{
  return false;
}

//==============================================================================
template <class SpecNode>
constexpr bool BodyNodeSpecializedFor<SpecNode>::_isSpecializedForNode(type<SpecNode>)
{
  return true;
}

//==============================================================================
template <class SpecNode>
SkeletonSpecializedFor<SpecNode>::SkeletonSpecializedFor()
{
  mSpecializedTreeNodes[typeid( SpecNode )] = &mTreeSpecNodeIterators;

  mNodeNameMgrMap[typeid( SpecNode )] = common::NameManager<Node*>();
  mSpecNodeNameMgrIterator = mNodeNameMgrMap.find(typeid( SpecNode ));
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
std::size_t SkeletonSpecializedFor<SpecNode>::getNumNodes(
    std::size_t treeIndex) const
{
  return _getNumNodes(type<NodeType>(), treeIndex);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SkeletonSpecializedFor<SpecNode>::getNode(
    std::size_t treeIndex, std::size_t nodeIndex)
{
  return _getNode(type<NodeType>(), treeIndex, nodeIndex);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
const NodeType* SkeletonSpecializedFor<SpecNode>::getNode(
    std::size_t treeIndex, std::size_t nodeIndex) const
{
  return const_cast<SkeletonSpecializedFor<SpecNode>*>(this)->
        _getNode(type<NodeType>(), treeIndex, nodeIndex);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SkeletonSpecializedFor<SpecNode>::getNode(
    const std::string& name)
{
  return _getNode(type<NodeType>(), name);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
const NodeType* SkeletonSpecializedFor<SpecNode>::getNode(
    const std::string& name) const
{
  return const_cast<SkeletonSpecializedFor<SpecNode>*>(this)->
        _getNode(type<NodeType>(), name);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
constexpr bool SkeletonSpecializedFor<SpecNode>::isSpecializedForNode()
{
  return _isSpecializedForNode(type<NodeType>());
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
std::size_t SkeletonSpecializedFor<SpecNode>::_getNumNodes(
    type<NodeType>, std::size_t treeIndex) const
{
  return detail::BasicNodeManagerForSkeleton::getNumNodes<NodeType>(treeIndex);
}

//==============================================================================
template <class SpecNode>
std::size_t SkeletonSpecializedFor<SpecNode>::_getNumNodes(
    type<SpecNode>, std::size_t treeIndex) const
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  if(treeIndex >= mTreeNodeMaps.size())
  {
    dterr << "[Skeleton::getNumNodes<" << typeid(SpecNode).name() << ">] "
          << "Requested tree index (" << treeIndex << "), but there are only ("
          << mTreeNodeMaps.size() << ") trees available\n";
    assert(false);
    return 0;
  }

  return mTreeSpecNodeIterators[treeIndex]->second.size();
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SkeletonSpecializedFor<SpecNode>::_getNode(
    type<NodeType>, std::size_t treeIndex, std::size_t nodeIndex)
{
  return detail::BasicNodeManagerForSkeleton::getNode<NodeType>(
        treeIndex, nodeIndex);
}

//==============================================================================
template <class SpecNode>
SpecNode* SkeletonSpecializedFor<SpecNode>::_getNode(
    type<SpecNode>, std::size_t treeIndex, std::size_t nodeIndex)
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  if(treeIndex >= mTreeNodeMaps.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(SpecNode).name() << ">] "
          << "Requested tree index (" << treeIndex << "), but there are only ("
          << mTreeNodeMaps.size() << ") trees available\n";
    assert(false);
    return nullptr;
  }

  NodeMap::iterator& it = mTreeSpecNodeIterators[treeIndex];

  if(nodeIndex >= it->second.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(SpecNode).name() << ">] "
          << "Requested index (" << nodeIndex << ") within tree (" << treeIndex
          << "), but there are only (" << it->second.size() << ") Nodes of the "
          << "requested type within that tree\n";
    assert(false);
    return nullptr;
  }

  return static_cast<SpecNode*>(it->second[nodeIndex]);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SkeletonSpecializedFor<SpecNode>::_getNode(
    type<NodeType>, const std::string& name)
{
  return detail::BasicNodeManagerForSkeleton::getNode<NodeType>(name);
}

//==============================================================================
template <class SpecNode>
SpecNode* SkeletonSpecializedFor<SpecNode>::_getNode(
    type<SpecNode>, const std::string& name)
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  return static_cast<SpecNode*>(
      mSpecNodeNameMgrIterator->second.getObject(name));
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
constexpr bool SkeletonSpecializedFor<SpecNode>::_isSpecializedForNode(
    type<NodeType>)
{
  return false;
}

//==============================================================================
template <class SpecNode>
constexpr bool SkeletonSpecializedFor<SpecNode>::_isSpecializedForNode(
    type<SpecNode>)
{
  return true;
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SPECIALIZEDNODEMANAGER_HPP_
