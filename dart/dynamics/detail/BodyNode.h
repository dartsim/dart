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

#ifndef DART_DYNAMICS_DETAIL_BODYNODE_H_
#define DART_DYNAMICS_DETAIL_BODYNODE_H_

#include <utility>

#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
template <class JointType>
JointType* BodyNode::moveTo(BodyNode* _newParent,
    const typename JointType::Properties& _joint)
{
  if(nullptr == _newParent)
    return getSkeleton()->moveBodyNodeTree<JointType>(
          this, getSkeleton(), nullptr, _joint);
  else
    return getSkeleton()->moveBodyNodeTree<JointType>(
          this, _newParent->getSkeleton(), _newParent, _joint);
}

//==============================================================================
template <class JointType>
JointType* BodyNode::moveTo(
    const SkeletonPtr& _newSkeleton, BodyNode* _newParent,
    const typename JointType::Properties& _joint)
{
  return getSkeleton()->moveBodyNodeTree<JointType>(
        this, _newSkeleton, _newParent, _joint);
}

//==============================================================================
template <class JointType>
SkeletonPtr BodyNode::split(const std::string& _skeletonName,
      const typename JointType::Properties& _joint)
{
  SkeletonPtr skel = Skeleton::create(getSkeleton()->getSkeletonProperties());
  skel->setName(_skeletonName);
  moveTo<JointType>(skel, nullptr, _joint);
  return skel;
}

//==============================================================================
template <class JointType>
JointType* BodyNode::changeParentJointType(
    const typename JointType::Properties& _joint)
{
  return moveTo<JointType>(getParentBodyNode(), _joint);
}

//==============================================================================
template <class JointType>
std::pair<JointType*, BodyNode*> BodyNode::copyTo(
    BodyNode* _newParent,
    const typename JointType::Properties& _joint,
    bool _recursive)
{
  if(nullptr == _newParent)
    return getSkeleton()->cloneBodyNodeTree<JointType>(
          this, getSkeleton(), nullptr, _joint, _recursive);
  else
    return getSkeleton()->cloneBodyNodeTree<JointType>(
          this, _newParent->getSkeleton(), _newParent, _joint, _recursive);
}

//==============================================================================
template <class JointType>
std::pair<JointType*, BodyNode*> BodyNode::copyTo(
    const SkeletonPtr& _newSkeleton, BodyNode* _newParent,
    const typename JointType::Properties& _joint,
    bool _recursive) const
{
  return getSkeleton()->cloneBodyNodeTree<JointType>(
        this, _newSkeleton, _newParent, _joint, _recursive);
}

//==============================================================================
template <class JointType>
SkeletonPtr BodyNode::copyAs(const std::string& _skeletonName,
    const typename JointType::Properties& _joint, bool _recursive) const
{
  SkeletonPtr skel = Skeleton::create(getSkeleton()->getSkeletonProperties());
  skel->setName(_skeletonName);
  copyTo<JointType>(skel, nullptr, _joint, _recursive);
  return skel;
}

//==============================================================================
template <class JointType, class NodeType>
std::pair<JointType*, NodeType*> BodyNode::createChildJointAndBodyNodePair(
    const typename JointType::Properties& _jointProperties,
    const typename NodeType::Properties& _bodyProperties)
{
  return getSkeleton()->createJointAndBodyNodePair<JointType, NodeType>(
        this, _jointProperties, _bodyProperties);
}

////==============================================================================
//template <class NodeType>
//size_t BodyNode::getNumNodes() const
//{
//  NodeMap::const_iterator it = mNodeMap.find(typeid(NodeType));
//  if(mNodeMap.end() == it)
//    return 0;

//  return it->second.size();
//}

////==============================================================================
//template <class NodeType>
//NodeType* BodyNode::getNode(size_t index)
//{
//  NodeMap::const_iterator it = mNodeMap.find(typeid(NodeType));
//  if(mNodeMap.end() == it)
//    return nullptr;

//  return static_cast<NodeType*>(
//        getVectorObjectIfAvailable(index, it->second));
//}

////==============================================================================
//template <class NodeType>
//const NodeType* BodyNode::getNode(size_t index) const
//{
//  return const_cast<BodyNode*>(this)->getNode<NodeType>(index);
//}

//==============================================================================
template <class NodeType, typename ...Args>
NodeType* BodyNode::createNode(Args&&... args)
{
  NodeType* node = new NodeType(this, std::forward<Args>(args)...);
  node->attach();

  return node;
}

//==============================================================================
template <class EndEffectorProperties>
EndEffector* BodyNode::createEndEffector(
    const EndEffectorProperties& _properties)
{
  return createNode<EndEffector>(_properties);
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_BODYNODE_H_
