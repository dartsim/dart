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
VisualDataProperties::VisualDataProperties(const Eigen::Vector4d& color,
                                           const bool hidden)
  : mRGBA(color),
    mHidden(hidden)
{
  // Do nothing
}

//==============================================================================
CollisionDataProperties::CollisionDataProperties(
    const bool collidable)
  : mCollidable(collidable)
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
void VisualData::hide()
{
  setHidden(false);
}

//==============================================================================
void VisualData::show()
{
  setHidden(true);
}

//==============================================================================
bool VisualData::isHidden() const
{
  return getHidden();
}

//==============================================================================
bool CollisionData::isCollidable() const
{
  return getCollidable();
}

//==============================================================================
ShapeNode::Properties::Properties(
    const ShapeNode::UniqueProperties &standardProperties,
    const ShapeNode::AddonProperties &addonProperties)
  : UniqueProperties(standardProperties),
    mAddonProperties(addonProperties)
{
  // Do nothing
}

//==============================================================================
ShapeNode::Properties::Properties(
    const ShapeNode::UniqueProperties&& standardProperties,
    const ShapeNode::AddonProperties&& addonProperties)
  : UniqueProperties(std::move(standardProperties)),
    mAddonProperties(std::move(addonProperties))
{
  // Do nothing
}

//==============================================================================
ShapeNode::UniqueProperties::UniqueProperties(const ShapePtr& shape,
                                  const Eigen::Isometry3d tf,
                                  const bool hidden)
  : mShape(shape),
    mTransform(tf)
{
  // Do nothing
}

//==============================================================================
void ShapeNode::setProperties(const ShapeNode::UniqueProperties& properties)
{
  setShape(properties.mShape);
  setRelativeTransform(properties.mTransform);
}

//==============================================================================
const ShapeNode::UniqueProperties ShapeNode::getProperties() const
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
const std::string& ShapeNode::setName(const std::string& newName)
{
  // If it already has the requested name, do nothing
  if(mEntityP.mName == newName && !newName.empty())
    return mEntityP.mName;

  mEntityP.mName = registerNameChange(newName);

  // Return the resulting name, after it has been checked for uniqueness
  return mEntityP.mName;
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
bool ShapeNode::dependsOn(size_t genCoordIndex) const
{
  return mBodyNode->dependsOn(genCoordIndex);
}

//==============================================================================
size_t ShapeNode::getNumDependentGenCoords() const
{
  return mBodyNode->getNumDependentGenCoords();
}

//==============================================================================
size_t ShapeNode::getDependentGenCoordIndex(size_t arrayIndex) const
{
  return mBodyNode->getDependentGenCoordIndex(arrayIndex);
}

//==============================================================================
const std::vector<size_t>& ShapeNode::getDependentGenCoordIndices() const
{
  return mBodyNode->getDependentGenCoordIndices();
}

//==============================================================================
size_t ShapeNode::getNumDependentDofs() const
{
  return mBodyNode->getNumDependentDofs();
}

//==============================================================================
DegreeOfFreedom* ShapeNode::getDependentDof(size_t index)
{
  return mBodyNode->getDependentDof(index);
}

//==============================================================================
const DegreeOfFreedom* ShapeNode::getDependentDof(size_t index) const
{
  return mBodyNode->getDependentDof(index);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& ShapeNode::getDependentDofs()
{
  return mBodyNode->getDependentDofs();
}

//==============================================================================
const std::vector<const DegreeOfFreedom*>& ShapeNode::getDependentDofs() const
{
  return static_cast<const BodyNode*>(mBodyNode)->getDependentDofs();
}

//==============================================================================
const std::vector<const DegreeOfFreedom*> ShapeNode::getChainDofs() const
{
  return mBodyNode->getChainDofs();
}

//==============================================================================
const math::Jacobian& ShapeNode::getJacobian() const
{
  if (mIsBodyJacobianDirty)
    updateShapeNodeJacobian();

  return mShapeNodeJacobian;
}

//==============================================================================
const math::Jacobian& ShapeNode::getWorldJacobian() const
{
  if(mIsWorldJacobianDirty)
    updateWorldJacobian();

  return mWorldJacobian;
}

//==============================================================================
const math::Jacobian& ShapeNode::getJacobianSpatialDeriv() const
{
  if(mIsBodyJacobianSpatialDerivDirty)
    updateShapeNodeJacobianSpatialDeriv();

  return mShapeNodeJacobianSpatialDeriv;
}

//==============================================================================
const math::Jacobian& ShapeNode::getJacobianClassicDeriv() const
{
  if(mIsWorldJacobianClassicDerivDirty)
    updateWorldJacobianClassicDeriv();

  return mWorldJacobianClassicDeriv;
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

//==============================================================================
void ShapeNode::draw(renderer::RenderInterface* ri,
                     const Eigen::Vector4d& color,
                     bool useDefaultColor) const
{
  auto visualData = getVisualData();

  if (!visualData && visualData->isHidden())
    return;

  if (useDefaultColor)
    mShapeNodeProp.mShape->draw(ri, visualData->getRGBA(), false);
  else
    mShapeNodeProp.mShape->draw(ri, color, false);
}

//==============================================================================
ShapeNode::ShapeNode(BodyNode* bodyNode, const Properties& properties)
  : common::AddonManager(),
    Entity(ConstructFrame),
    Frame(bodyNode, ""),
    FixedFrame(bodyNode, "", properties.mTransform),
    TemplatedJacobianNode<ShapeNode>(bodyNode),
    mShapeNodeProp(),
    mShapeUpdatedSignal(),
    mRelativeTransformUpdatedSignal(),
    onShapeUpdated(mShapeUpdatedSignal),
    onRelativeTransformUpdated(mRelativeTransformUpdatedSignal)
{
  // Do nothing
}

//==============================================================================
Node* ShapeNode::cloneNode(BodyNode* parent) const
{
  ShapeNode* shapeNode = new ShapeNode(parent, Properties());
  shapeNode->duplicateAddons(this);

  shapeNode->copy(this);

  if(mIK)
    shapeNode->mIK = mIK->clone(shapeNode);

  return shapeNode;
}

//==============================================================================
void ShapeNode::updateShapeNodeJacobian() const
{
  mShapeNodeJacobian = math::AdInvTJac(getRelativeTransform(),
                                       mBodyNode->getJacobian());
  mIsBodyJacobianDirty = false;
}

//==============================================================================
void ShapeNode::updateWorldJacobian() const
{
  mWorldJacobian = math::AdRJac(getWorldTransform(), getJacobian());

  mIsWorldJacobianDirty = false;
}

//==============================================================================
void ShapeNode::updateShapeNodeJacobianSpatialDeriv() const
{
  mShapeNodeJacobianSpatialDeriv =
      math::AdInvTJac(getRelativeTransform(),
                      mBodyNode->getJacobianSpatialDeriv());

  mIsBodyJacobianSpatialDerivDirty = false;
}

//==============================================================================
void ShapeNode::updateWorldJacobianClassicDeriv() const
{
  const math::Jacobian& dJ_parent = mBodyNode->getJacobianClassicDeriv();
  const math::Jacobian& J_parent = mBodyNode->getWorldJacobian();

  const Eigen::Vector3d& v_local =
      getLinearVelocity(mBodyNode, Frame::World());

  const Eigen::Vector3d& w_parent = mBodyNode->getAngularVelocity();
  const Eigen::Vector3d& p = (getWorldTransform().translation()
                  - mBodyNode->getWorldTransform().translation()).eval();

  mWorldJacobianClassicDeriv = dJ_parent;
  mWorldJacobianClassicDeriv.bottomRows<3>().noalias() +=
      J_parent.topRows<3>().colwise().cross(v_local + w_parent.cross(p))
      + dJ_parent.topRows<3>().colwise().cross(p);

  mIsWorldJacobianClassicDerivDirty = false;
}

} // namespace dynamics
} // namespace dart

