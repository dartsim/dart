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

#ifndef DART_DYNAMICS_DETAIL_SKELETON_H_
#define DART_DYNAMICS_DETAIL_SKELETON_H_

#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
template <class NodeType>
size_t Skeleton::getNumNodes() const
{
  NodeMap::const_iterator it = mSkelCache.mNodeMap.find(typeid(NodeType));
  if(mSkelCache.mNodeMap.end() == it)
    return 0;

  return it->second.size();
}

//==============================================================================
template <class NodeType>
size_t Skeleton::getNumNodes(size_t treeIndex) const
{
  if(treeIndex >= mTreeCache.size())
  {
    dterr << "[Skeleton::getNumNodes<" << typeid(NodeType).name() << ">] "
          << "Requested tree index (" << treeIndex << "), but there are only ("
          << mTreeCache.size() << ") trees available\n";
    assert(false);
    return 0;
  }

  const DataCache& cache = mTreeCache[treeIndex];

  NodeMap::const_iterator it = cache.mNodeMap.find(typeid(NodeType));

  if(cache.mNodeMap.end() == it)
    return 0;

  return it->second.size();
}

//==============================================================================
template <class NodeType>
NodeType* Skeleton::getNode(size_t index)
{
  NodeMap::iterator it = mSkelCache.mNodeMap.find(typeid(NodeType));
  if(mSkelCache.mNodeMap.end() == it)
  {
    dterr << "[Skeleton::getNode<" << typeid(NodeType).name() << ">] "
          << "Requested index (" << index << "), but there are no Nodes of the "
          << "requested type in this Skeleton\n";
    assert(false);
    return nullptr;
  }

  if(index >= it->second.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(NodeType).name() << ">] "
          << "Requested index (" << index << "), but there are only ("
          << it->second.size() << ") Nodes of the requested type in this "
          << "Skeleton\n";
    assert(false);
    return nullptr;
  }

  return static_cast<NodeType*>(it->second[index]);
}

//==============================================================================
template <class NodeType>
NodeType* Skeleton::getNode(size_t nodeIndex, size_t treeIndex)
{
  if(treeIndex >= mTreeCache.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(NodeType).name() << ">] "
          << "Requested tree index (" << treeIndex << "), but there are only ("
          << mTreeCache.size() << ") trees available\n";
    assert(false);
    return nullptr;
  }

  DataCache& cache = mTreeCache[treeIndex];
  NodeMap::iterator it = cache.mNodeMap.find(typeid(NodeType));
  if(cache.mNodeMap.end() == it)
  {
    dterr << "[Skeleton::getNode<" << typeid(NodeType).name() << ">] "
          << "Requested index (" << nodeIndex << ") within tree (" << treeIndex
          << "), but there are no Nodes of the requested type in this tree\n";
    assert(false);
    return nullptr;
  }

  if(nodeIndex >= it->second.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(NodeType).name() << ">] "
          << "Requested index (" << nodeIndex << ") within tree (" << treeIndex
          << "), but there are only (" << it->second.size() << ") Nodes of the "
          << "requested type within that tree\n";
    assert(false);
    return nullptr;
  }

  return static_cast<NodeType*>(it->second[nodeIndex]);
}

//==============================================================================
template <class NodeType>
const NodeType* Skeleton::getNode(size_t index) const
{
  return const_cast<Skeleton*>(this)->getNode<NodeType>(index);
}

//==============================================================================
template <class NodeType>
const NodeType* Skeleton::getNode(size_t nodeIndex, size_t treeIndex) const
{
  return const_cast<Skeleton*>(this)->getNode<NodeType>(nodeIndex, treeIndex);
}

//==============================================================================
template <class NodeType>
NodeType* Skeleton::getNode(const std::string& name)
{
  NodeNameMgrMap::iterator it = mNodeNameMgrMap.find(typeid(NodeType));
  if(mNodeNameMgrMap.end() == it)
    return nullptr;

  return static_cast<NodeType*>(it->second.getObject(name));
}

//==============================================================================
template <class NodeType>
const NodeType* Skeleton::getNode(const std::string& name) const
{
  return const_cast<Skeleton*>(this)->getNode<NodeType>(name);
}

//==============================================================================
template <class JointType>
JointType* Skeleton::moveBodyNodeTree(
    BodyNode* _bodyNode,
    const SkeletonPtr& _newSkeleton,
    BodyNode* _parentNode,
    const typename JointType::Properties& _joint)
{
  JointType* parentJoint = new JointType(_joint);

  if(moveBodyNodeTree(parentJoint, _bodyNode, _newSkeleton, _parentNode))
    return parentJoint;

  // If the move failed, we should delete the Joint that we created and return
  // a nullptr.
  delete parentJoint;
  return nullptr;
}

//==============================================================================
template <class JointType>
std::pair<JointType*, BodyNode*> Skeleton::cloneBodyNodeTree(
    const BodyNode* _bodyNode,
    const SkeletonPtr& _newSkeleton,
    BodyNode* _parentNode,
    const typename JointType::Properties& _joint, bool _recursive) const
{
  JointType* parentJoint = new JointType(_joint);
  std::pair<Joint*, BodyNode*> root =
      cloneBodyNodeTree(parentJoint, _bodyNode, _newSkeleton, _parentNode,
                        _recursive);
  return std::pair<JointType*, BodyNode*>(parentJoint, root.second);
}

//==============================================================================
template <class JointType, class NodeType>
std::pair<JointType*, NodeType*> Skeleton::createJointAndBodyNodePair(
    BodyNode* _parent,
    const typename JointType::Properties& _jointProperties,
    const typename NodeType::Properties& _bodyProperties)
{
  JointType* joint = new JointType(_jointProperties);
  NodeType* node = new NodeType(_parent, joint, _bodyProperties);
  registerBodyNode(node);

  return std::pair<JointType*, NodeType*>(joint, node);
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SKELETON_H_
