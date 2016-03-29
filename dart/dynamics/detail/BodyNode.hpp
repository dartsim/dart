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

#ifndef DART_DYNAMICS_DETAIL_BODYNODE_HPP_
#define DART_DYNAMICS_DETAIL_BODYNODE_HPP_

#include <utility>

#include "dart/dynamics/Skeleton.hpp"

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

//==============================================================================
template <class NodeType, typename ...Args>
NodeType* BodyNode::createNode(Args&&... args)
{
  NodeType* node = new NodeType(this, std::forward<Args>(args)...);
  node->attach();

  return node;
}

//==============================================================================
template <class ShapeNodeProperties>
ShapeNode* BodyNode::createShapeNode(ShapeNodeProperties properties,
                                     bool automaticName)
{
  if(automaticName)
  {
    properties.mName = getName()+"_ShapeNode_"
        +std::to_string(getNumShapeNodes());
  }

  return createNode<ShapeNode>(properties);
}

//==============================================================================
template <class... Addons>
ShapeNode* BodyNode::createShapeNodeWith(const ShapePtr& shape)
{
  return createShapeNodeWith<Addons...>(shape, getName()+"_ShapeNode_"
                                        +std::to_string(getNumShapeNodes()));
}

//==============================================================================
template <class... Addons>
ShapeNode* BodyNode::createShapeNodeWith(
    const ShapePtr& shape, const std::string& name)
{
  auto shapeNode = createShapeNode(shape, name);

  common::createAddons<ShapeNode, Addons...>(shapeNode);

  return shapeNode;
}

//==============================================================================
template <class Addon>
size_t BodyNode::getNumShapeNodesWith() const
{
  auto count = 0u;
  auto numShapeNode = getNumShapeNodes();

  for (auto i = 0u; i < numShapeNode; ++i)
  {
    if (getShapeNode(i)->has<Addon>())
      ++count;
  }

  return count;
}

//==============================================================================
template <class Addon>
const std::vector<ShapeNode*> BodyNode::getShapeNodesWith()
{
  std::vector<ShapeNode*> shapeNodes;

  auto numShapeNode = getNumShapeNodes();

  for (auto i = 0u; i < numShapeNode; ++i)
  {
    auto shapeNode = getShapeNode(i);

    if (shapeNode->has<Addon>())
      shapeNodes.push_back(shapeNode);
  }

  return shapeNodes;
}

//==============================================================================
template <class Addon>
const std::vector<const ShapeNode*> BodyNode::getShapeNodesWith() const
{
  std::vector<const ShapeNode*> shapeNodes;

  auto numShapeNode = getNumShapeNodes();

  for (auto i = 0u; i < numShapeNode; ++i)
  {
    const auto shapeNode = getShapeNode(i);

    if (shapeNode->has<Addon>())
      shapeNodes.push_back(shapeNode);
  }

  return shapeNodes;
}

//==============================================================================
template <class Addon>
void BodyNode::removeAllShapeNodesWith()
{
  auto shapeNodes = getShapeNodesWith<Addon>();
  for (auto shapeNode : shapeNodes)
    shapeNode->remove();
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

#endif // DART_DYNAMICS_DETAIL_BODYNODE_HPP_
