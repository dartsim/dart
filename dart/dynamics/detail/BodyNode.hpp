/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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
  SkeletonPtr skel = Skeleton::create(getSkeleton()->getAspectProperties());
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
  SkeletonPtr skel = Skeleton::create(getSkeleton()->getAspectProperties());
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
template <class ShapeType>
ShapeNode* BodyNode::createShapeNode(const std::shared_ptr<ShapeType>& shape)
{
  ShapeNode::BasicProperties properties;
  properties.mShape = shape;

  return createShapeNode(properties, true);
}

//==============================================================================
template <class ShapeType, class StringType>
ShapeNode* BodyNode::createShapeNode(
    const std::shared_ptr<ShapeType>& shape,
    StringType&& name)
{
  ShapeNode::BasicProperties properties;
  properties.mShape = shape;
  properties.mName = std::forward<StringType>(name);

  return createShapeNode(properties, false);
}

//==============================================================================
template <class... Aspects>
ShapeNode* BodyNode::createShapeNodeWith(const ShapePtr& shape)
{
  return createShapeNodeWith<Aspects...>(shape, getName()+"_ShapeNode_"
                                        +std::to_string(getNumShapeNodes()));
}

//==============================================================================
template <class... Aspects>
ShapeNode* BodyNode::createShapeNodeWith(
    const ShapePtr& shape, const std::string& name)
{
  auto shapeNode = createShapeNode(shape, name);

  common::createAspects<ShapeNode, Aspects...>(shapeNode);

  return shapeNode;
}

//==============================================================================
template <class AspectT>
std::size_t BodyNode::getNumShapeNodesWith() const
{
  auto count = 0u;
  auto numShapeNode = getNumShapeNodes();

  for (auto i = 0u; i < numShapeNode; ++i)
  {
    if (getShapeNode(i)->has<AspectT>())
      ++count;
  }

  return count;
}

//==============================================================================
template <class AspectT>
const std::vector<ShapeNode*> BodyNode::getShapeNodesWith()
{
  std::vector<ShapeNode*> shapeNodes;

  auto numShapeNode = getNumShapeNodes();

  for (auto i = 0u; i < numShapeNode; ++i)
  {
    auto shapeNode = getShapeNode(i);

    if (shapeNode->has<AspectT>())
      shapeNodes.push_back(shapeNode);
  }

  return shapeNodes;
}

//==============================================================================
template <class AspectT>
const std::vector<const ShapeNode*> BodyNode::getShapeNodesWith() const
{
  std::vector<const ShapeNode*> shapeNodes;

  auto numShapeNode = getNumShapeNodes();

  for (auto i = 0u; i < numShapeNode; ++i)
  {
    const auto shapeNode = getShapeNode(i);

    if (shapeNode->has<AspectT>())
      shapeNodes.push_back(shapeNode);
  }

  return shapeNodes;
}

//==============================================================================
template <class AspectT>
void BodyNode::removeAllShapeNodesWith()
{
  auto shapeNodes = getShapeNodesWith<AspectT>();
  for (auto shapeNode : shapeNodes)
    shapeNode->remove();
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_BODYNODE_HPP_
