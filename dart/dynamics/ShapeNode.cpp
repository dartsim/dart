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
ShapeNode::UniqueProperties::UniqueProperties(
    const Eigen::Isometry3d& relativeTransform)
  : mRelativeTransform(relativeTransform)
{
  // Do nothing
}

//==============================================================================
ShapeNode::Properties::Properties(
    const ShapeFrame::Properties& shapeFrameProperties,
    const ShapeNode::UniqueProperties& shapeNodeProperties,
    const ShapeNode::AspectProperties& aspectProperties)
  : ShapeFrame::Properties(shapeFrameProperties),
    ShapeNode::UniqueProperties(shapeNodeProperties),
    mAspectProperties(aspectProperties)
{
  // Do nothing
}

//==============================================================================
ShapeNode::Properties::Properties(
    ShapeFrame::Properties&& shapeFrameProperties,
    ShapeNode::UniqueProperties&& shapeNodeProperties,
    ShapeNode::AspectProperties&& aspectProperties)
  : ShapeFrame::Properties(std::move(shapeFrameProperties)),
    ShapeNode::UniqueProperties(std::move(shapeNodeProperties)),
    mAspectProperties(std::move(aspectProperties))
{
  // Do nothing
}

//==============================================================================
void ShapeNode::setProperties(const Properties& properties)
{
  ShapeFrame::setProperties(
        static_cast<const ShapeFrame::Properties&>(properties));
  setProperties(static_cast<const ShapeNode::UniqueProperties&>(properties));
  setAspectProperties(properties.mAspectProperties);
}

//==============================================================================
void ShapeNode::setProperties(const ShapeNode::UniqueProperties& properties)
{
  setRelativeTransform(properties.mRelativeTransform);
}

//==============================================================================
const ShapeNode::Properties ShapeNode::getShapeNodeProperties() const
{
  return Properties(getShapeFrameProperties(), mShapeNodeP,
                    getAspectProperties());
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
const std::string& ShapeNode::setName(const std::string& name)
{
  // If it already has the requested name, do nothing
  if(mEntityP.mName == name && !name.empty())
    return mEntityP.mName;

  mEntityP.mName = registerNameChange(name);

  // Return the resulting name, after it has been checked for uniqueness
  return mEntityP.mName;
}

//==============================================================================
size_t ShapeNode::incrementVersion()
{
  ++mShapeFrameP.mVersion;
  if(const SkeletonPtr& skel = getSkeleton())
    skel->incrementVersion();

  return mShapeFrameP.mVersion;
}

//==============================================================================
size_t ShapeNode::getVersion() const
{
  return mShapeFrameP.mVersion;
}

//==============================================================================
void ShapeNode::setRelativeTransform(const Eigen::Isometry3d& transform)
{
  const Eigen::Isometry3d oldTransform = mRelativeTf;

  mRelativeTf = transform;
  mShapeNodeP.mRelativeTransform = transform;
  notifyTransformUpdate();
  notifyJacobianUpdate();
  notifyJacobianDerivUpdate();

  mRelativeTransformUpdatedSignal.raise(this, oldTransform,
                                        mShapeNodeP.mRelativeTransform);
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
ShapeNode::ShapeNode(BodyNode* bodyNode, const Properties& properties)
  : Entity(ConstructFrame),
    Frame(bodyNode, ""),
    FixedFrame(bodyNode, ""),
    ShapeFrame(bodyNode),
    TemplatedJacobianNode<ShapeNode>(bodyNode),
    mShapeUpdatedSignal(ShapeUpdatedSignal()),
    mRelativeTransformUpdatedSignal(RelativeTransformUpdatedSignal()),
    onShapeUpdated(mShapeUpdatedSignal),
    onRelativeTransformUpdated(mRelativeTransformUpdatedSignal)
{
  setProperties(properties);
}

//==============================================================================
ShapeNode::ShapeNode(BodyNode* bodyNode,
                     const ShapePtr& shape,
                     const std::string& name)
  : Entity(ConstructFrame),
    Frame(bodyNode, ""),
    FixedFrame(bodyNode, ""),
    ShapeFrame(bodyNode),
    TemplatedJacobianNode<ShapeNode>(bodyNode),
    mShapeUpdatedSignal(ShapeUpdatedSignal()),
    mRelativeTransformUpdatedSignal(RelativeTransformUpdatedSignal()),
    onShapeUpdated(mShapeUpdatedSignal),
    onRelativeTransformUpdated(mRelativeTransformUpdatedSignal)
{
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

