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

#include "dart/common/Console.h"
#include "dart/dynamics/EndEffector.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
EndEffector::UniqueProperties::UniqueProperties(
    const Eigen::Isometry3d& _defaultTransform)
  : mDefaultTransform(_defaultTransform)
{
  // Do nothing
}

//==============================================================================
EndEffector::Properties::Properties(
    const Entity::Properties& _entityProperties,
    const UniqueProperties& _effectorProperties)
  : Entity::Properties(_entityProperties),
    UniqueProperties(_effectorProperties)
{
  // Do nothing
}

//==============================================================================
EndEffector::~EndEffector()
{
  // Do nothing
}

//==============================================================================
void EndEffector::remove()
{
  size_t index = mIndexInBodyNode;
  assert(mParentBodyNode->mEndEffectors[index] == this);

  mParentBodyNode->mEndEffectors.erase(
        mParentBodyNode->mEndEffectors.begin()+index);

  for(size_t i=index; i<mParentBodyNode->mEndEffectors.size(); ++i)
  {
    EndEffector* ee = mParentBodyNode->mEndEffectors[i];
    ee->mIndexInBodyNode = i;
  }

  getSkeleton()->unregisterEndEffector(this);

  delete this;
}

//==============================================================================
void EndEffector::setProperties(const Properties& _properties, bool _useNow)
{
  Entity::setProperties(_properties);
  setProperties(static_cast<const UniqueProperties&>(_properties), _useNow);
}

//==============================================================================
void EndEffector::setProperties(const UniqueProperties& _properties,
                                bool _useNow)
{
  setDefaultRelativeTransform(_properties.mDefaultTransform, _useNow);
}

//==============================================================================
EndEffector::Properties EndEffector::getEndEffectorProperties() const
{
  return Properties(getEntityProperties(), mEndEffectorP);
}

//==============================================================================
void EndEffector::copy(const EndEffector& _otherEndEffector)
{
  if(this == &_otherEndEffector)
    return;

  setProperties(_otherEndEffector.getEndEffectorProperties());
  setRelativeTransform(_otherEndEffector.getRelativeTransform());
}

//==============================================================================
void EndEffector::copy(const EndEffector* _otherEndEffector)
{
  if(nullptr == _otherEndEffector)
    return;

  copy(*_otherEndEffector);
}

//==============================================================================
EndEffector& EndEffector::operator=(const EndEffector& _otherEndEffector)
{
  copy(_otherEndEffector);
  return *this;
}

//==============================================================================
const std::string& EndEffector::setName(const std::string& _name)
{
  // If it already has the requested name, do nothing
  if(mEntityP.mName == _name)
    return mEntityP.mName;

  // Remove the current name entry and add a new name entry
  getSkeleton()->mNameMgrForEndEffectors.removeName(mEntityP.mName);
  mEntityP.mName = _name;
  getSkeleton()->addEntryToEndEffectorNameMgr(this);

  // Return the resulting name, after it has been checked for uniqueness
  return mEntityP.mName;
}

//==============================================================================
void EndEffector::setRelativeTransform(const Eigen::Isometry3d& _newRelativeTf)
{
  mRelativeTf = _newRelativeTf;
  notifyTransformUpdate();
}

//==============================================================================
void EndEffector::setDefaultRelativeTransform(
    const Eigen::Isometry3d& _newDefaultTf, bool _useNow)
{
  mEndEffectorP.mDefaultTransform = _newDefaultTf;

  if(_useNow)
    resetRelativeTransform();
}

//==============================================================================
void EndEffector::resetRelativeTransform()
{
  setRelativeTransform(mEndEffectorP.mDefaultTransform);
}

//==============================================================================
std::shared_ptr<Skeleton> EndEffector::getSkeleton()
{
  return mParentBodyNode->getSkeleton();
}

//==============================================================================
std::shared_ptr<const Skeleton> EndEffector::getSkeleton() const
{
  return mParentBodyNode->getSkeleton();
}

//==============================================================================
size_t EndEffector::getNumDependentGenCoords() const
{
  return mParentBodyNode->getNumDependentGenCoords();
}

//==============================================================================
const std::vector<size_t>& EndEffector::getDependentGenCoordIndices() const
{
  return mParentBodyNode->getDependentGenCoordIndices();
}

//==============================================================================
size_t EndEffector::getNumDependentDofs() const
{
  return mParentBodyNode->getNumDependentDofs();
}

//==============================================================================
const std::vector<const DegreeOfFreedom*> EndEffector::getChainDofs() const
{
  return mParentBodyNode->getChainDofs();
}

//==============================================================================
BodyNode* EndEffector::getParentBodyNode()
{
  return mParentBodyNode;
}

//==============================================================================
const BodyNode* EndEffector::getParentBodyNode() const
{
  return mParentBodyNode;
}

//==============================================================================
size_t EndEffector::getIndexInSkeleton() const
{
  return mIndexInSkeleton;
}

//==============================================================================
const math::Jacobian& EndEffector::getJacobian() const
{
  if (mIsEffectorJacobianDirty)
    updateEffectorJacobian();

  return mEffectorJacobian;
}

//==============================================================================
const math::Jacobian& EndEffector::getWorldJacobian() const
{
  if(mIsWorldJacobianDirty)
    updateWorldJacobian();

  return mWorldJacobian;
}

//==============================================================================
const math::Jacobian& EndEffector::getJacobianSpatialDeriv() const
{
  if(mIsEffectorJacobianSpatialDerivDirty)
    updateEffectorJacobianSpatialDeriv();

  return mEffectorJacobianSpatialDeriv;
}

//==============================================================================
const math::Jacobian& EndEffector::getJacobianClassicDeriv() const
{
  if(mIsWorldJacobianClassicDerivDirty)
    updateWorldJacobianClassicDeriv();

  return mWorldJacobianClassicDeriv;
}

//==============================================================================
void EndEffector::notifyTransformUpdate()
{
  mIsEffectorJacobianDirty = true;
  mIsWorldJacobianDirty = true;
  mIsEffectorJacobianSpatialDerivDirty = true;
  mIsWorldJacobianClassicDerivDirty = true;

  Frame::notifyTransformUpdate();
}

//==============================================================================
void EndEffector::notifyVelocityUpdate()
{
  mIsEffectorJacobianSpatialDerivDirty = true;
  mIsWorldJacobianClassicDerivDirty = true;

  Frame::notifyVelocityUpdate();
}

//==============================================================================
EndEffector::EndEffector(BodyNode* _parent, const Properties& _properties)
  : Entity(nullptr, "", false),
    Frame(_parent, ""),
    FixedFrame(_parent, "", _properties.mDefaultTransform),
    mParentBodyNode(_parent),
    mIndexInSkeleton(0),
    mIndexInBodyNode(0)
{
  setProperties(_properties);

  _parent->mEndEffectors.push_back(this);
  mIndexInBodyNode = _parent->mEndEffectors.size()-1;
}

//==============================================================================
EndEffector* EndEffector::clone(BodyNode* _parent) const
{
  EndEffector* ee = new EndEffector(_parent, Properties());
  ee->copy(this);

  return ee;
}

//==============================================================================
void EndEffector::updateEffectorJacobian() const
{
  mEffectorJacobian = math::AdInvTJac(getRelativeTransform(),
                                      mParentBodyNode->getJacobian());
  mIsEffectorJacobianDirty = false;
}

//==============================================================================
void EndEffector::updateWorldJacobian() const
{
  mWorldJacobian = math::AdRJac(getWorldTransform(), getJacobian());

  mIsWorldJacobianDirty = false;
}

//==============================================================================
void EndEffector::updateEffectorJacobianSpatialDeriv() const
{
  mEffectorJacobianSpatialDeriv =
      math::AdInvTJac(getRelativeTransform(),
                      mParentBodyNode->getJacobianSpatialDeriv());

  mIsEffectorJacobianSpatialDerivDirty = false;
}

//==============================================================================
void EndEffector::updateWorldJacobianClassicDeriv() const
{
  const math::Jacobian& dJ_parent = mParentBodyNode->getJacobianClassicDeriv();
  const math::Jacobian& J_parent = mParentBodyNode->getWorldJacobian();

  const Eigen::Vector3d& v_local =
      getLinearVelocity(mParentBodyNode, Frame::World());

  const Eigen::Vector3d& w_parent = mParentBodyNode->getAngularVelocity();
  const Eigen::Vector3d& p = (getWorldTransform().translation()
                  - mParentBodyNode->getWorldTransform().translation()).eval();

  mWorldJacobianClassicDeriv = dJ_parent;
  mWorldJacobianClassicDeriv.bottomRows<3>().noalias() +=
      J_parent.topRows<3>().colwise().cross(v_local + w_parent.cross(p))
      + dJ_parent.topRows<3>().colwise().cross(p);

  mIsWorldJacobianClassicDerivDirty = false;
}

} // namespace dynamics
} // namespace dart
