/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/ShapeNode.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

//==============================================================================
ShapeNode::Properties::Properties(
    const ShapeFrame::AspectProperties& shapeFrameProperties,
    const FixedFrame::AspectProperties& fixedFrameProperties,
    const NameAspect::Properties& name,
    const CompositeProperties& compositeProperties)
  : ShapeFrame::AspectProperties(shapeFrameProperties),
    FixedFrame::AspectProperties(fixedFrameProperties),
    NameAspect::Properties(name),
    mCompositeProperties(compositeProperties)
{
  // Do nothing
}

//==============================================================================
void ShapeNode::setProperties(const Properties& properties)
{
  setCompositeProperties(properties.mCompositeProperties);
  ShapeFrame::setAspectProperties(properties);
  FixedFrame::setAspectProperties(
        static_cast<const FixedFrame::AspectProperties&>(properties));
  get<NameAspect>()->setProperties(properties);
}

//==============================================================================
const ShapeNode::Properties ShapeNode::getShapeNodeProperties() const
{
  return Properties(ShapeFrame::getAspectProperties(),
                    FixedFrame::getAspectProperties(),
                    getName(),
                    getCompositeProperties());
}

//==============================================================================
void ShapeNode::copy(const ShapeNode& other)
{
  if (this == &other)
    return;

  setProperties(other.getShapeNodeProperties());
}

//==============================================================================
void ShapeNode::copy(const ShapeNode* other)
{
  if (nullptr == other)
    return;

  copy(*other);
}

//==============================================================================
ShapeNode& ShapeNode::operator=(const ShapeNode& other)
{
  copy(other);
  return *this;
}

//==============================================================================
void ShapeNode::setRelativeTransform(const Eigen::Isometry3d& transform)
{
  if(transform.matrix() == FixedFrame::mAspectProperties.mRelativeTf.matrix())
    return;

  const Eigen::Isometry3d oldTransform = getRelativeTransform();

  FixedFrame::setRelativeTransform(transform);
  notifyJacobianUpdate();
  notifyJacobianDerivUpdate();

  mRelativeTransformUpdatedSignal.raise(
        this, oldTransform, getRelativeTransform());
}

//==============================================================================
void ShapeNode::setRelativeRotation(const Eigen::Matrix3d& rotation)
{
  Eigen::Isometry3d transform = getRelativeTransform();
  transform.linear() = rotation;

  setRelativeTransform(transform);
}

//==============================================================================
Eigen::Matrix3d ShapeNode::getRelativeRotation() const
{
  return getRelativeTransform().linear();
}

//==============================================================================
void ShapeNode::setRelativeTranslation(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d transform = getRelativeTransform();
  transform.translation() = translation;

  setRelativeTransform(transform);
}

//==============================================================================
void ShapeNode::setOffset(const Eigen::Vector3d& offset)
{
  setRelativeTranslation(offset);
}

//==============================================================================
Eigen::Vector3d ShapeNode::getRelativeTranslation() const
{
  return getRelativeTransform().translation();
}

//==============================================================================
Eigen::Vector3d ShapeNode::getOffset() const
{
  return getRelativeTranslation();
}

//==============================================================================
ShapeNode::ShapeNode(BodyNode* bodyNode, const Properties& properties)
  : Entity(ConstructFrame),
    Frame(bodyNode),
    FixedFrame(bodyNode),
    detail::ShapeNodeCompositeBase(
      std::make_tuple(bodyNode, properties.mRelativeTf), bodyNode)
{
  setProperties(properties);
}

//==============================================================================
ShapeNode::ShapeNode(BodyNode* bodyNode,
                     const ShapePtr& shape,
                     const std::string& name)
  : Entity(ConstructFrame),
    Frame(bodyNode),
    FixedFrame(bodyNode),
    detail::ShapeNodeCompositeBase(
      std::make_tuple(bodyNode, Eigen::Isometry3d::Identity()), bodyNode)
{
  // TODO(MXG): Consider changing this to a delegating constructor instead
  Properties prop;
  prop.mShape = shape;
  prop.mName = name;

  setProperties(prop);
}

//==============================================================================
Node* ShapeNode::cloneNode(BodyNode* parent) const
{
  ShapeNode* shapeNode = new ShapeNode(parent, Properties());
  shapeNode->duplicateAspects(this);

  shapeNode->copy(this);

  if(mIK)
    shapeNode->mIK = mIK->clone(shapeNode);

  return shapeNode;
}

} // namespace dynamics
} // namespace dart

