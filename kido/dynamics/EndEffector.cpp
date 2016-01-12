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

#include "kido/common/Console.h"
#include "kido/dynamics/EndEffector.h"
#include "kido/dynamics/BodyNode.h"
#include "kido/dynamics/Skeleton.h"

namespace kido {
namespace dynamics {

//==============================================================================
Support::Support(EndEffector* _ee)
  : mActive(false),
    mEndEffector(_ee)
{
  if(nullptr == mEndEffector)
  {
    dterr << "[Support::Support] It is not permissible to construct a Support "
          << "with a nullptr EndEffector!\n";
    assert(false);
  }
}

//==============================================================================
void Support::setGeometry(const math::SupportGeometry& _newSupport)
{
  mGeometry = _newSupport;
  mEndEffector->getSkeleton()->notifySupportUpdate(
        mEndEffector->getTreeIndex());
}

//==============================================================================
const math::SupportGeometry& Support::getGeometry() const
{
  return mGeometry;
}

//==============================================================================
void Support::setActive(bool _supporting)
{
  if(mActive == _supporting)
    return;

  mActive = _supporting;
  mEndEffector->getSkeleton()->notifySupportUpdate(
        mEndEffector->getTreeIndex());
}

//==============================================================================
bool Support::isActive() const
{
  return mActive;
}

//==============================================================================
EndEffector::UniqueProperties::UniqueProperties(const Eigen::Isometry3d& _defaultTransform,
    const math::SupportGeometry& _supportGeometry, bool _supporting)
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
  size_t index = mIndexInBodyNode;
  assert(mBodyNode->mEndEffectors[index] == this);
  mBodyNode->mEndEffectors.erase(mBodyNode->mEndEffectors.begin() + index);

  for(size_t i=index; i<mBodyNode->mEndEffectors.size(); ++i)
  {
    EndEffector* ee = mBodyNode->mEndEffectors[i];
    ee->mIndexInBodyNode = i;
  }

  SkeletonPtr skel = getSkeleton();
  if(skel)
    skel->unregisterEndEffector(this);
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
Support* EndEffector::getSupport(bool _createIfNull)
{
  if(nullptr == mSupport && _createIfNull)
    createSupport();

  return mSupport.get();
}

//==============================================================================
const Support* EndEffector::getSupport() const
{
  return mSupport.get();
}

//==============================================================================
Support* EndEffector::createSupport()
{
  mSupport = std::unique_ptr<Support>(new Support(this));
  return mSupport.get();
}

//==============================================================================
void EndEffector::eraseSupport()
{
  mSupport = nullptr;
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
EndEffector::EndEffector(BodyNode* _parent, const Properties& _properties)
  : Entity(ConstructFrame),
    Frame(_parent, ""),
    Node(ConstructNode, _parent),
    FixedFrame(_parent, "", _properties.mDefaultTransform),
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
} // namespace kido
