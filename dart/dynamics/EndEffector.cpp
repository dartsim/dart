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

namespace dart {
namespace dynamics {

namespace detail {

void SupportUpdate(Support* support)
{
  if(EndEffector* ee = support->getComposite())
    ee->getSkeleton()->notifySupportUpdate(ee->getTreeIndex());
}

} // namespace detail

//==============================================================================
void Support::setActive(bool _supporting)
{
  if(mState.mActive == _supporting)
    return;

  mState.mActive = _supporting;
  UpdateState(this);
}

//==============================================================================
bool Support::isActive() const
{
  return mState.mActive;
}

//==============================================================================
EndEffector::StateData::StateData(
    const Eigen::Isometry3d& relativeTransform,
    const common::Composite::State& aspectStates)
  : mRelativeTransform(relativeTransform),
    mAspectStates(aspectStates)
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
    const UniqueProperties& _effectorProperties,
    const common::Composite::Properties& _aspectProperties)
  : Entity::Properties(_entityProperties),
    UniqueProperties(_effectorProperties),
    mAspectProperties(_aspectProperties)
{
  // Do nothing
}

//==============================================================================
void EndEffector::setState(const StateData& _state)
{
  setRelativeTransform(_state.mRelativeTransform);
  setAspectStates(_state.mAspectStates);
}

//==============================================================================
void EndEffector::setState(StateData&& _state)
{
  setRelativeTransform(_state.mRelativeTransform);
  setAspectStates(std::move(_state.mAspectStates));
}

//==============================================================================
EndEffector::StateData EndEffector::getEndEffectorState() const
{
  return StateData(mRelativeTf, getAspectStates());
}

//==============================================================================
void EndEffector::setProperties(const PropertiesData& _properties, bool _useNow)
{
  Entity::setProperties(_properties);
  setProperties(static_cast<const UniqueProperties&>(_properties), _useNow);
  setAspectProperties(_properties.mAspectProperties);
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
  return PropertiesData(getEntityProperties(), mEndEffectorP,
                        getAspectProperties());
}

//==============================================================================
void EndEffector::copy(const EndEffector& _otherEndEffector)
{
  if(this == &_otherEndEffector)
    return;

  setState(_otherEndEffector.getEndEffectorState());
  setProperties(_otherEndEffector.getEndEffectorProperties());
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
void EndEffector::setNodeState(const Node::State& otherState)
{
  setState(static_cast<const State&>(otherState));
}

//==============================================================================
std::unique_ptr<Node::State> EndEffector::getNodeState() const
{
  return common::make_unique<EndEffector::State>(getEndEffectorState());
}

//==============================================================================
void EndEffector::copyNodeStateTo(std::unique_ptr<Node::State>& outputState) const
{
  if(outputState)
  {
    EndEffector::State* state =
        static_cast<EndEffector::State*>(outputState.get());

    state->mRelativeTransform = mRelativeTf;
    copyAspectStatesTo(state->mAspectStates);
  }
  else
  {
    outputState = getNodeState();
  }
}

//==============================================================================
void EndEffector::setNodeProperties(const Node::Properties& otherProperties)
{
  setProperties(static_cast<const Properties&>(otherProperties));
}

//==============================================================================
std::unique_ptr<Node::Properties> EndEffector::getNodeProperties() const
{
  return common::make_unique<EndEffector::Properties>(
      getEndEffectorProperties());
}

//==============================================================================
void EndEffector::copyNodePropertiesTo(
    std::unique_ptr<Node::Properties>& outputProperties) const
{
  if(outputProperties)
  {
    EndEffector::Properties* properties =
        static_cast<EndEffector::Properties*>(outputProperties.get());

    static_cast<Entity::Properties&>(*properties) = getEntityProperties();
    static_cast<UniqueProperties&>(*properties) = mEndEffectorP;
    copyAspectPropertiesTo(properties->mAspectProperties);
  }
  else
  {
    outputProperties = getNodeProperties();
  }
}

//==============================================================================
void EndEffector::setRelativeTransform(const Eigen::Isometry3d& _newRelativeTf)
{
  mRelativeTf = _newRelativeTf;
  notifyTransformUpdate();
  notifyJacobianUpdate();
  notifyJacobianDerivUpdate();
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
const math::Jacobian& EndEffector::getJacobian() const
{
  if (mIsBodyJacobianDirty)
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
  if(mIsBodyJacobianSpatialDerivDirty)
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
    const SkeletonPtr& skel = getSkeleton();
    if(skel)
      skel->notifySupportUpdate(getTreeIndex());
  }

  Frame::notifyTransformUpdate();
}

//==============================================================================
void EndEffector::notifyVelocityUpdate()
{
  Frame::notifyVelocityUpdate();
}

//==============================================================================
EndEffector::EndEffector(BodyNode* _parent, const PropertiesData& _properties)
  : Entity(ConstructFrame),
    Frame(_parent, ""),
    FixedFrame(_parent, "", _properties.mDefaultTransform),
    TemplatedJacobianNode<EndEffector>(_parent)
{
  setProperties(_properties);
}

//==============================================================================
Node* EndEffector::cloneNode(BodyNode* _parent) const
{
  EndEffector* ee = new EndEffector(_parent, PropertiesData());
  ee->duplicateAspects(this);

  ee->copy(this);

  if(mIK)
    ee->mIK = mIK->clone(ee);

  return ee;
}

//==============================================================================
void EndEffector::updateEffectorJacobian() const
{
  mEffectorJacobian = math::AdInvTJac(getRelativeTransform(),
                                      mBodyNode->getJacobian());
  mIsBodyJacobianDirty = false;
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

  mIsBodyJacobianSpatialDerivDirty = false;
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
