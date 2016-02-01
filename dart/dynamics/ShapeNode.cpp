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

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
void VisualUpdate(VisualData* /*visualData*/)
{
  // Do nothing
}

//==============================================================================
void CollisionUpdate(CollisionData* /*collisionData*/)
{
  // Do nothing
}

//==============================================================================
void DynamicsUpdate(DynamicsData* /*dynamicsData*/)
{
  // Do nothing
}

//==============================================================================
VisualDataProperties::VisualDataProperties(const Eigen::Vector4d& color)
  : mRGBA(color)
{
  // Do nothing
}

//==============================================================================
CollisionDataProperties::CollisionDataProperties(
    const bool collisionMode)
  : mCollisionMode(collisionMode)
{
  // Do nothing
}

//==============================================================================
DynamicsDataProperties::DynamicsDataProperties(
    const double frictionCoeff,
    const double restitutionCoeff)
  :
    mFrictionCoeff(frictionCoeff),
    mRestitutionCoeff(restitutionCoeff)
{
  // Do nothing
}

} // namespace detail

//==============================================================================
//VisualData::VisualData(
//    common::AddonManager* mgr,
//    const AddonWithProtectedPropertiesInSkeleton::Properties& properties)
//  : AddonWithProtectedPropertiesInSkeleton<VisualData, VisualDataProperties>(
//      mgr, properties)
//{
//  // Do nothing
//}

//==============================================================================
void VisualData::setColor(const Eigen::Vector3d& color)
{
  setRGB(color);
}

//==============================================================================
void VisualData::setColor(const Eigen::Vector4d& color)
{
  setRGBA(color);
}

//==============================================================================
void VisualData::setRGB(const Eigen::Vector3d& rgb)
{
  Eigen::Vector4d rgba = getRGBA();
  rgba.head<3>() = rgb;

  setRGBA(rgba);
}

//==============================================================================
void VisualData::setAlpha(const double alpha)
{
  Eigen::Vector4d rgba = getRGBA();
  rgba[3] = alpha;

  setRGBA(rgba);
}

//==============================================================================
Eigen::Vector3d VisualData::getColor() const
{
  return getRGB();
}

//==============================================================================
Eigen::Vector3d VisualData::getRGB() const
{
  return getRGBA().head<3>();
}

//==============================================================================
const double VisualData::getAlpha() const
{
  return getRGBA()[3];
}

//==============================================================================
ShapeNode::ExtendedProperties::ExtendedProperties(
    const ShapeNode::Properties &standardProperties,
    const ShapeNode::AddonProperties &addonProperties)
  : Properties(standardProperties),
    mAddonProperties(addonProperties)
{
  // Do nothing
}

//==============================================================================
ShapeNode::ExtendedProperties::ExtendedProperties(
    const ShapeNode::Properties&& standardProperties,
    const ShapeNode::AddonProperties&& addonProperties)
  : Properties(std::move(standardProperties)),
    mAddonProperties(std::move(addonProperties))
{
  // Do nothing
}

//==============================================================================
ShapeNode::Properties::Properties(const ShapePtr& shape,
                                  const Eigen::Isometry3d tf,
                                  const bool hidden)
  : mShape(shape),
    mTransform(tf),
    mHidden(hidden)
{
  // Do nothing
}

//==============================================================================
ShapeNode::ShapeNode(BodyNode* bodyNode)
  : //Node(bn),
    common::AddonManager(),
    mBodyNode(bodyNode),
    mShapeNodeProp(),
    mShapeUpdatedSignal(),
    mRelativeTransformUpdatedSignal(),
    onShapeUpdated(mShapeUpdatedSignal),
    onRelativeTransformUpdated(mRelativeTransformUpdatedSignal)
{
  // Do nothing
}

//==============================================================================
void ShapeNode::setProperties(const ShapeNode::Properties& properties)
{
  setShape(properties.mShape);
  setRelativeTransform(properties.mTransform);
  setHidden(properties.mHidden);
}

//==============================================================================
const ShapeNode::Properties ShapeNode::getProperties() const
{
  return mShapeNodeProp;
}

//==============================================================================
void ShapeNode::copy(const ShapeNode& other)
{
  if (this == &other)
    return;

  setProperties(other.getProperties());
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
void ShapeNode::setShape(const ShapePtr& shape)
{
  if (shape == mShapeNodeProp.mShape)
    return;

  ShapePtr oldShape = mShapeNodeProp.mShape;

  mShapeNodeProp.mShape = shape;

  mShapeUpdatedSignal.raise(this, oldShape, mShapeNodeProp.mShape);
}

//==============================================================================
ShapePtr ShapeNode::getShape()
{
  return mShapeNodeProp.mShape;
}

//==============================================================================
ConstShapePtr ShapeNode::getShape() const
{
  return mShapeNodeProp.mShape;
}

//==============================================================================
void ShapeNode::setRelativeTransform(const Eigen::Isometry3d& transform)
{
  const Eigen::Isometry3d& oldTransform = mShapeNodeProp.mTransform;

  mShapeNodeProp.mTransform = transform;

  mRelativeTransformUpdatedSignal.raise(
        this, oldTransform, mShapeNodeProp.mTransform);
}

//==============================================================================
const Eigen::Isometry3d& ShapeNode::getRelativeTransform() const
{
  return mShapeNodeProp.mTransform;
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
  return mShapeNodeProp.mTransform.linear();
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
  return mShapeNodeProp.mTransform.translation();
}

//==============================================================================
Eigen::Vector3d ShapeNode::getOffset() const
{
  return getRelativeTranslation();
}

//==============================================================================
void ShapeNode::setHidden(const bool hide)
{
  mShapeNodeProp.mHidden = hide;
}

//==============================================================================
bool ShapeNode::isHidden() const
{
  return mShapeNodeProp.mHidden;
}

//==============================================================================
std::shared_ptr<Skeleton> ShapeNode::getSkeleton()
{
  return mBodyNode->getSkeleton();
}

//==============================================================================
std::shared_ptr<const Skeleton> ShapeNode::getSkeleton() const
{
  return mBodyNode->getSkeleton();
}

//==============================================================================
VisualData* ShapeNode::getVisualData(const bool createIfNull)
{
  VisualData* visualData = getVisualData();

  if (createIfNull && nullptr == visualData)
    return createVisualData();

  return visualData;
}

//==============================================================================
CollisionData* ShapeNode::getCollisionData(const bool createIfNull)
{
  CollisionData* collisionData = getCollisionData();

  if (createIfNull && nullptr == collisionData)
    return createCollisionData();

  return collisionData;
}

//==============================================================================
DynamicsData* ShapeNode::getDynamicsData(const bool createIfNull)
{
  DynamicsData* dynamicsData = getDynamicsData();

  if (createIfNull && nullptr == dynamicsData)
    return createDynamicsData();

  return dynamicsData;
}

} // namespace dynamics
} // namespace dart

