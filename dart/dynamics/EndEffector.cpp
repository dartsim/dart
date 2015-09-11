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
Support::StateData::StateData(bool active)
  : mActive(active)
{
  // Do nothing
}

//==============================================================================
Support::State::State(bool active)
  : common::Addon::StateMixer<StateData>(StateData(active))
{
  // Do nothing
}

//==============================================================================
Support::Support(EndEffector* _ee)
  : common::Addon(_ee, "Support"),
    mEndEffector(_ee)
{
  if(nullptr == mEndEffector)
  {
    dterr << "[Support::constructor] It is not permissible to construct a "
          << "Support with a nullptr EndEffector!\n";
    assert(false);
  }

  setStatePtr(&mState);
  setPropertiesPtr(&mProperties);
}

//==============================================================================
Support::Support(EndEffector *_ee, const Support& otherSupport)
  : common::Addon(_ee, "Support"),
    mEndEffector(_ee)
{
  if(nullptr == mEndEffector)
  {
    dterr << "[Support::constructor] It is not permissible to construct a "
          << "Support with a nullptr EndEffector!\n";
    assert(false);
  }

  mState = otherSupport.mState;
  mProperties = otherSupport.mProperties;

  setStatePtr(&mState);
  setPropertiesPtr(&mProperties);
}

//==============================================================================
std::unique_ptr<common::Addon> Support::clone(
    common::AddonManager* newManager) const
{
  EndEffector* ee = dynamic_cast<EndEffector*>(newManager);
  if(nullptr == ee)
  {
    dterr << "[Support::clone] Attempting to clone a Support class into an "
          << "AddonManager which is not an EndEffector. This is not allowed!\n";
    assert(false);
    return nullptr;
  }

  return std::unique_ptr<common::Addon>(new Support(ee, *this));
}

//==============================================================================
void Support::setState(const std::unique_ptr<common::Addon::State>& otherState)
{
  if(otherState)
    mState = *static_cast<const State*>(otherState.get());
}

//==============================================================================
void Support::setProperties(
    const std::unique_ptr<common::Addon::Properties>& otherProperties)
{
  if(otherProperties)
    mProperties = *static_cast<const Properties*>(otherProperties.get());
}

//==============================================================================
void Support::setGeometry(const math::SupportGeometry& _newSupport)
{
  mProperties.mGeometry = _newSupport;
  mEndEffector->getSkeleton()->notifySupportUpdate(
        mEndEffector->getTreeIndex());
}

//==============================================================================
const math::SupportGeometry& Support::getGeometry() const
{
  return mProperties.mGeometry;
}

//==============================================================================
void Support::setActive(bool _supporting)
{
  if(mState.mActive == _supporting)
    return;

  mState.mActive = _supporting;
  mEndEffector->getSkeleton()->notifySupportUpdate(
        mEndEffector->getTreeIndex());
}

//==============================================================================
bool Support::isActive() const
{
  return mState.mActive;
}

//==============================================================================
EndEffector::StateData::StateData(const Eigen::Isometry3d& relativeTransform)
  : mRelativeTransform(relativeTransform)
{
  // Do nothing
}

//==============================================================================
EndEffector::UniqueProperties::UniqueProperties(
    const Eigen::Isometry3d& _defaultTransform)
  : mDefaultTransform(_defaultTransform)
{
  // Do nothing
}

//==============================================================================
EndEffector::PropertiesData::PropertiesData(
    const Entity::Properties& _entityProperties,
    const UniqueProperties& _effectorProperties)
  : Entity::Properties(_entityProperties),
    UniqueProperties(_effectorProperties)
{
  // Do nothing
}

//==============================================================================
void EndEffector::setProperties(const PropertiesData& _properties, bool _useNow)
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
EndEffector::PropertiesData EndEffector::getEndEffectorProperties() const
{
  return PropertiesData(getEntityProperties(), mEndEffectorP);
}

//==============================================================================
void EndEffector::copy(const EndEffector& _otherEndEffector)
{
  if(this == &_otherEndEffector)
    return;

  setProperties(_otherEndEffector.getEndEffectorProperties());

  // We should also copy the relative transform, because it could be different
  // than the default relative transform
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
  if(mEntityP.mName == _name && !_name.empty())
    return mEntityP.mName;

  mEntityP.mName = registerNameChange(_name);

  // Return the resulting name, after it has been checked for uniqueness
  return mEntityP.mName;
}

//==============================================================================
void EndEffector::setNodeState(
    const std::unique_ptr<Node::State>& otherState)
{
  const State* state = static_cast<const State*>(otherState.get());

  setRelativeTransform(state->mRelativeTransform);
  setAddonStates(state->mAddonStates);
}

//==============================================================================
const Node::State* EndEffector::getNodeState() const
{
  mStateCache.mRelativeTransform = getRelativeTransform();
  getAddonStates(mStateCache.mAddonStates);

  return &mStateCache;
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
Support* EndEffector::getSupport(bool _createIfNull)
{
  if(nullptr == getSupport() && _createIfNull)
    createSupport();

  return getSupport();
}

//==============================================================================
std::shared_ptr<Skeleton> EndEffector::getSkeleton()
{
  return mBodyNode->getSkeleton();
}

//==============================================================================
std::shared_ptr<const Skeleton> EndEffector::getSkeleton() const
{
  return mBodyNode->getSkeleton();
}

//==============================================================================
bool EndEffector::dependsOn(size_t _genCoordIndex) const
{
  return mBodyNode->dependsOn(_genCoordIndex);
}

//==============================================================================
size_t EndEffector::getNumDependentGenCoords() const
{
  return mBodyNode->getNumDependentGenCoords();
}

//==============================================================================
size_t EndEffector::getDependentGenCoordIndex(size_t _arrayIndex) const
{
  return mBodyNode->getDependentGenCoordIndex(_arrayIndex);
}

//==============================================================================
const std::vector<size_t>& EndEffector::getDependentGenCoordIndices() const
{
  return mBodyNode->getDependentGenCoordIndices();
}

//==============================================================================
size_t EndEffector::getNumDependentDofs() const
{
  return mBodyNode->getNumDependentDofs();
}

//==============================================================================
DegreeOfFreedom* EndEffector::getDependentDof(size_t _index)
{
  return mBodyNode->getDependentDof(_index);
}

//==============================================================================
const DegreeOfFreedom* EndEffector::getDependentDof(size_t _index) const
{
  return mBodyNode->getDependentDof(_index);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& EndEffector::getDependentDofs()
{
  return mBodyNode->getDependentDofs();
}

//==============================================================================
const std::vector<const DegreeOfFreedom*>& EndEffector::getDependentDofs() const
{
  return static_cast<const BodyNode*>(mBodyNode)->getDependentDofs();
}

//==============================================================================
const std::vector<const DegreeOfFreedom*> EndEffector::getChainDofs() const
{
  return mBodyNode->getChainDofs();
}

//==============================================================================
BodyNode* EndEffector::getParentBodyNode()
{
  return mBodyNode;
}

//==============================================================================
const BodyNode* EndEffector::getParentBodyNode() const
{
  return mBodyNode;
}

//==============================================================================
size_t EndEffector::getIndexInSkeleton() const
{
  return mIndexInSkeleton;
}

//==============================================================================
size_t EndEffector::getTreeIndex() const
{
  return mBodyNode->getTreeIndex();
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
  if(!mNeedTransformUpdate)
  {
    mIsEffectorJacobianDirty = true;
    mIsWorldJacobianDirty = true;
    mIsEffectorJacobianSpatialDerivDirty = true;
    mIsWorldJacobianClassicDerivDirty = true;

    const SkeletonPtr& skel = getSkeleton();
    if(skel)
      skel->notifySupportUpdate(getTreeIndex());
  }

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
EndEffector::EndEffector(BodyNode* _parent, const PropertiesData& _properties)
  : Entity(ConstructFrame),
    Frame(_parent, ""),
    FixedFrame(_parent, "", _properties.mDefaultTransform),
    TemplatedJacobianNode<EndEffector>(_parent),
    mIndexInSkeleton(0),
    mIsEffectorJacobianDirty(true),
    mIsWorldJacobianDirty(true),
    mIsEffectorJacobianSpatialDerivDirty(true),
    mIsWorldJacobianClassicDerivDirty(true)
{
  DART_INSTANTIATE_SPECIALIZED_ADDON(Support)
  setProperties(_properties);
}

//==============================================================================
Node* EndEffector::cloneNode(BodyNode* _parent) const
{
  EndEffector* ee = new EndEffector(_parent, PropertiesData());
  ee->copy(this);

  ee->duplicateAddons(this);

  if(mIK)
    ee->mIK = mIK->clone(ee);

  return ee;
}

//==============================================================================
void EndEffector::updateEffectorJacobian() const
{
  mEffectorJacobian = math::AdInvTJac(getRelativeTransform(),
                                      mBodyNode->getJacobian());
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
                      mBodyNode->getJacobianSpatialDeriv());

  mIsEffectorJacobianSpatialDerivDirty = false;
}

//==============================================================================
void EndEffector::updateWorldJacobianClassicDeriv() const
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
