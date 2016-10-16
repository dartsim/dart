/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/Skeleton.h"

#include <algorithm>
#include <queue>
#include <string>
#include <vector>

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/EndEffector.h"
#include "dart/dynamics/InverseKinematics.h"
#include "dart/dynamics/Marker.h"
#include "dart/dynamics/PointMass.h"
#include "dart/dynamics/SoftBodyNode.h"

namespace dart {
namespace dynamics {

#define SET_ALL_FLAGS( X ) for(auto& cache : mTreeCache) cache.mDirty. X = true;\
                           mSkelCache.mDirty. X = true;

#define SET_FLAG( Y, X ) mTreeCache[ Y ].mDirty. X = true;                      \
                         mSkelCache.mDirty. X = true;

#define ON_ALL_TREES( X ) for(size_t i=0; i < mTreeCache.size(); ++i) X (i);

//==============================================================================
Skeleton::Properties::Properties(
    const std::string& _name,
    bool _isMobile,
    const Eigen::Vector3d& _gravity,
    double _timeStep,
    bool _enabledSelfCollisionCheck,
    bool _enableAdjacentBodyCheck)
  : mName(_name),
    mIsMobile(_isMobile),
    mGravity(_gravity),
    mTimeStep(_timeStep),
    mEnabledSelfCollisionCheck(_enabledSelfCollisionCheck),
    mEnabledAdjacentBodyCheck(_enableAdjacentBodyCheck)
{
  // Do nothing
}

//==============================================================================
SkeletonPtr Skeleton::create(const std::string& _name)
{
  return create(Properties(_name));
}

//==============================================================================
SkeletonPtr Skeleton::create(const Properties& _properties)
{
  SkeletonPtr skel(new Skeleton(_properties));
  skel->setPtr(skel);
  return skel;
}

//==============================================================================
SkeletonPtr Skeleton::getPtr()
{
  return mPtr.lock();
}

//==============================================================================
ConstSkeletonPtr Skeleton::getPtr() const
{
  return mPtr.lock();
}

//==============================================================================
std::mutex& Skeleton::getMutex() const
{
  return mMutex;
}

//==============================================================================
Skeleton::~Skeleton()
{
  for (BodyNode* bn : mSkelCache.mBodyNodes)
    delete bn;
}

//==============================================================================
SkeletonPtr Skeleton::clone() const
{
  SkeletonPtr skelClone = Skeleton::create(getName());

  for(size_t i=0; i<getNumBodyNodes(); ++i)
  {
    // Create a clone of the parent Joint
    Joint* joint = getJoint(i)->clone();

    // Identify the original parent BodyNode
    const BodyNode* originalParent = getBodyNode(i)->getParentBodyNode();

    // Grab the parent BodyNode clone (using its name, which is guaranteed to be
    // unique), or use nullptr if this is a root BodyNode
    BodyNode* parentClone = (originalParent == nullptr)? nullptr :
          skelClone->getBodyNode(originalParent->getName());

    if( (nullptr != originalParent) && (nullptr == parentClone) )
    {
      dterr << "[Skeleton::clone] Failed to find a clone of BodyNode named ["
            << originalParent->getName() << "] which is needed as the parent "
            << "of the BodyNode named [" << getBodyNode(i)->getName()
            << "] and should already have been created. Please report this as "
            << "a bug!\n";
    }

    BodyNode* newBody = getBodyNode(i)->clone(parentClone, joint);

    // The IK module gets cloned by the Skeleton and not by the BodyNode,
    // because IK modules rely on the Skeleton's structure and indexing. If the
    // IK was cloned by the BodyNode into a Skeleton that has a different
    // structure, then there is no guarantee that it will continue to work
    // correctly.
    if(getBodyNode(i)->getIK())
      newBody->mIK = getBodyNode(i)->getIK()->clone(newBody);

    skelClone->registerBodyNode(newBody);
  }

  for(size_t i=0; i<getNumEndEffectors(); ++i)
  {
    // Grab the EndEffector we want to clone
    const EndEffector* originalEE = getEndEffector(i);

    // Identify the original parent BodyNode
    const BodyNode* originalParent = originalEE->getParentBodyNode();

    // Grab the clone of the original parent
    BodyNode* parentClone = skelClone->getBodyNode(originalParent->getName());

    EndEffector* newEE = originalEE->clone(parentClone);

    if(originalEE->getIK())
      newEE->mIK = originalEE->getIK()->clone(newEE);

    skelClone->registerEndEffector(newEE);
  }

  skelClone->setProperties(getSkeletonProperties());

  return skelClone;
}

//==============================================================================
void Skeleton::setProperties(const Properties& _properties)
{
  setName(_properties.mName);
  setMobile(_properties.mIsMobile);
  setGravity(_properties.mGravity);
  setTimeStep(_properties.mTimeStep);

  if(_properties.mEnabledSelfCollisionCheck)
    enableSelfCollision(_properties.mEnabledAdjacentBodyCheck);
  else
    disableSelfCollision();
}

//==============================================================================
const Skeleton::Properties& Skeleton::getSkeletonProperties() const
{
  return mSkeletonP;
}

//==============================================================================
const std::string& Skeleton::setName(const std::string& _name)
{
  if(_name == mSkeletonP.mName && !_name.empty())
    return mSkeletonP.mName;

  const std::string oldName = mSkeletonP.mName;
  mSkeletonP.mName = _name;

  mNameMgrForBodyNodes.setManagerName(
        "Skeleton::BodyNode | "+mSkeletonP.mName);
  mNameMgrForSoftBodyNodes.setManagerName(
        "Skeleton::SoftBodyNode | "+mSkeletonP.mName);
  mNameMgrForJoints.setManagerName(
        "Skeleton::Joint | "+mSkeletonP.mName);
  mNameMgrForDofs.setManagerName(
        "Skeleton::DegreeOfFreedom | "+mSkeletonP.mName);
  mNameMgrForMarkers.setManagerName(
        "Skeleton::Marker | "+mSkeletonP.mName);
  mNameMgrForEndEffectors.setManagerName(
        "Skeleton::EndEffector | "+mSkeletonP.mName);

  ConstMetaSkeletonPtr me = mPtr.lock();
  mNameChangedSignal.raise(me, oldName, mSkeletonP.mName);

  return mSkeletonP.mName;
}

//==============================================================================
const std::string& Skeleton::getName() const
{
  return mSkeletonP.mName;
}

//==============================================================================
const std::string& Skeleton::addEntryToBodyNodeNameMgr(BodyNode* _newNode)
{
  _newNode->mEntityP.mName =
      mNameMgrForBodyNodes.issueNewNameAndAdd(_newNode->getName(), _newNode);

  return _newNode->mEntityP.mName;
}

//==============================================================================
const std::string& Skeleton::addEntryToJointNameMgr(Joint* _newJoint,
                                                    bool _updateDofNames)
{
  _newJoint->mJointP.mName =
      mNameMgrForJoints.issueNewNameAndAdd(_newJoint->getName(), _newJoint);

  if(_updateDofNames)
    _newJoint->updateDegreeOfFreedomNames();

  return _newJoint->mJointP.mName;
}

//==============================================================================
void Skeleton::addEntryToEndEffectorNameMgr(EndEffector* _ee)
{
  _ee->mEntityP.mName =
      mNameMgrForEndEffectors.issueNewNameAndAdd(_ee->getName(), _ee);
}

//==============================================================================
void Skeleton::addEntryToSoftBodyNodeNameMgr(SoftBodyNode* _newNode)
{
  // Note: This doesn't need the same checks as BodyNode and Joint, because
  // its name has already been resolved against all the BodyNodes, which includes
  // all SoftBodyNodes.
  mNameMgrForSoftBodyNodes.addName(_newNode->getName(), _newNode);
}

//==============================================================================
void Skeleton::addMarkersOfBodyNode(BodyNode* _node)
{
  for (size_t i=0; i<_node->getNumMarkers(); ++i)
    addEntryToMarkerNameMgr(_node->getMarker(i));
}

//==============================================================================
void Skeleton::removeMarkersOfBodyNode(BodyNode* _node)
{
  for (size_t i=0; i<_node->getNumMarkers(); ++i)
    mNameMgrForMarkers.removeName(_node->getMarker(i)->getName());
}

//==============================================================================
const std::string& Skeleton::addEntryToMarkerNameMgr(Marker* _newMarker)
{
  _newMarker->mProperties.mName = mNameMgrForMarkers.issueNewNameAndAdd(
      _newMarker->getName(), _newMarker);
  return _newMarker->mProperties.mName;
}

//==============================================================================
void Skeleton::enableSelfCollision(bool _enableAdjacentBodyCheck)
{
  mSkeletonP.mEnabledSelfCollisionCheck = true;
  mSkeletonP.mEnabledAdjacentBodyCheck = _enableAdjacentBodyCheck;
}

//==============================================================================
void Skeleton::disableSelfCollision()
{
  mSkeletonP.mEnabledSelfCollisionCheck = false;
  mSkeletonP.mEnabledAdjacentBodyCheck = false;
}

//==============================================================================
bool Skeleton::isEnabledSelfCollisionCheck() const
{
  return mSkeletonP.mEnabledSelfCollisionCheck;
}

//==============================================================================
bool Skeleton::isEnabledAdjacentBodyCheck() const
{
  return mSkeletonP.mEnabledAdjacentBodyCheck;
}

//==============================================================================
void Skeleton::setMobile(bool _isMobile)
{
  mSkeletonP.mIsMobile = _isMobile;
}

//==============================================================================
bool Skeleton::isMobile() const
{
  return mSkeletonP.mIsMobile;
}

//==============================================================================
void Skeleton::setTimeStep(double _timeStep)
{
  assert(_timeStep > 0.0);
  mSkeletonP.mTimeStep = _timeStep;

  for(size_t i=0; i<mTreeCache.size(); ++i)
    notifyArticulatedInertiaUpdate(i);
}

//==============================================================================
double Skeleton::getTimeStep() const
{
  return mSkeletonP.mTimeStep;
}

//==============================================================================
void Skeleton::setGravity(const Eigen::Vector3d& _gravity)
{
  mSkeletonP.mGravity = _gravity;
  SET_ALL_FLAGS(mGravityForces);
  SET_ALL_FLAGS(mCoriolisAndGravityForces);
  ON_ALL_TREES(notifySupportUpdate);
}

//==============================================================================
const Eigen::Vector3d& Skeleton::getGravity() const
{
  return mSkeletonP.mGravity;
}

//==============================================================================
size_t Skeleton::getNumBodyNodes() const
{
  return mSkelCache.mBodyNodes.size();
}

//==============================================================================
size_t Skeleton::getNumRigidBodyNodes() const
{
  return mSkelCache.mBodyNodes.size() - mSoftBodyNodes.size();
}

//==============================================================================
size_t Skeleton::getNumSoftBodyNodes() const
{
  return mSoftBodyNodes.size();
}

//==============================================================================
size_t Skeleton::getNumTrees() const
{
  return mTreeCache.size();
}

//==============================================================================
template<typename T>
static T getVectorObjectIfAvailable(size_t _idx, const std::vector<T>& _vec)
{
  if (_idx < _vec.size())
    return _vec[_idx];

  assert( _idx < _vec.size() );
  return nullptr;
}

//==============================================================================
BodyNode* Skeleton::getRootBodyNode(size_t _treeIdx)
{
  if( mTreeCache.size() > _treeIdx)
    return mTreeCache[_treeIdx].mBodyNodes[0];

  if(mTreeCache.size() == 0)
  {
    dterr << "[Skeleton::getRootBodyNode] Requested a root BodyNode from a "
          << "Skeleton with no BodyNodes!\n";
    assert(false);
  }
  else
  {
    dterr << "[Skeleton::getRootBodyNode] Requested invalid root BodyNode "
          << "index (" << _treeIdx << ")! Must be less than "
          << mTreeCache.size() << ".\n";
    assert(false);
  }

  return nullptr;
}

//==============================================================================
const BodyNode* Skeleton::getRootBodyNode(size_t _treeIdx) const
{
  return const_cast<Skeleton*>(this)->getRootBodyNode(_treeIdx);
}

//==============================================================================
BodyNode* Skeleton::getBodyNode(size_t _idx)
{
  return getVectorObjectIfAvailable<BodyNode*>(_idx, mSkelCache.mBodyNodes);
}

//==============================================================================
const BodyNode* Skeleton::getBodyNode(size_t _idx) const
{
  return getVectorObjectIfAvailable<BodyNode*>(_idx, mSkelCache.mBodyNodes);
}

//==============================================================================
SoftBodyNode* Skeleton::getSoftBodyNode(size_t _idx)
{
  return getVectorObjectIfAvailable<SoftBodyNode*>(_idx, mSoftBodyNodes);
}

//==============================================================================
const SoftBodyNode* Skeleton::getSoftBodyNode(size_t _idx) const
{
  return getVectorObjectIfAvailable<SoftBodyNode*>(_idx, mSoftBodyNodes);
}

//==============================================================================
BodyNode* Skeleton::getBodyNode(const std::string& _name)
{
  return mNameMgrForBodyNodes.getObject(_name);
}

//==============================================================================
const BodyNode* Skeleton::getBodyNode(const std::string& _name) const
{
  return mNameMgrForBodyNodes.getObject(_name);
}

//==============================================================================
SoftBodyNode* Skeleton::getSoftBodyNode(const std::string& _name)
{
  return mNameMgrForSoftBodyNodes.getObject(_name);
}

//==============================================================================
const SoftBodyNode* Skeleton::getSoftBodyNode(const std::string& _name) const
{
  return mNameMgrForSoftBodyNodes.getObject(_name);
}

//==============================================================================
template <class T>
static std::vector<const T*>& convertToConstPtrVector(
    const std::vector<T*>& vec, std::vector<const T*>& const_vec)
{
  const_vec.resize(vec.size());
  for(size_t i=0; i<vec.size(); ++i)
    const_vec[i] = vec[i];
  return const_vec;
}

//==============================================================================
const std::vector<BodyNode*>& Skeleton::getBodyNodes()
{
  return mSkelCache.mBodyNodes;
}

//==============================================================================
const std::vector<const BodyNode*>& Skeleton::getBodyNodes() const
{
  return convertToConstPtrVector<BodyNode>(
        mSkelCache.mBodyNodes, mSkelCache.mConstBodyNodes);
}

//==============================================================================
template <class ObjectT, size_t (ObjectT::*getIndexInSkeleton)() const>
static size_t templatedGetIndexOf(const Skeleton* _skel, const ObjectT* _obj,
                                  const std::string& _type, bool _warning)
{
  if(nullptr == _obj)
  {
    if(_warning)
    {
      dterr << "[Skeleton::getIndexOf] Requesting the index of a nullptr "
            << _type << " within the Skeleton [" << _skel->getName() << "] ("
            << _skel << ")!\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  if(_skel == _obj->getSkeleton().get())
    return (_obj->*getIndexInSkeleton)();

  if(_warning)
  {
    dterr << "[Skeleton::getIndexOf] Requesting the index of a " << _type << " ["
          << _obj->getName() << "] (" << _obj << ") from a Skeleton that it does "
          << "not belong to!\n";
    assert(false);
  }

  return INVALID_INDEX;
}

//==============================================================================
size_t Skeleton::getIndexOf(const BodyNode* _bn, bool _warning) const
{
  return templatedGetIndexOf<BodyNode, &BodyNode::getIndexInSkeleton>(
        this, _bn, "BodyNode", _warning);
}

//==============================================================================
const std::vector<BodyNode*>& Skeleton::getTreeBodyNodes(size_t _treeIdx)
{
  return mTreeCache[_treeIdx].mBodyNodes;
}

//==============================================================================
std::vector<const BodyNode*> Skeleton::getTreeBodyNodes(size_t _treeIdx) const
{
  return convertToConstPtrVector<BodyNode>(
        mTreeCache[_treeIdx].mBodyNodes, mTreeCache[_treeIdx].mConstBodyNodes);
}

//==============================================================================
size_t Skeleton::getNumJoints() const
{
  // The number of joints and body nodes are identical
  return getNumBodyNodes();
}

//==============================================================================
Joint* Skeleton::getJoint(size_t _idx)
{
  BodyNode* bn = getVectorObjectIfAvailable<BodyNode*>(
                   _idx, mSkelCache.mBodyNodes);
  if (bn)
    return bn->getParentJoint();

  return nullptr;
}

//==============================================================================
const Joint* Skeleton::getJoint(size_t _idx) const
{
  return const_cast<Skeleton*>(this)->getJoint(_idx);
}

//==============================================================================
Joint* Skeleton::getJoint(const std::string& _name)
{
  return mNameMgrForJoints.getObject(_name);
}

//==============================================================================
const Joint* Skeleton::getJoint(const std::string& _name) const
{
  return mNameMgrForJoints.getObject(_name);
}

//==============================================================================
size_t Skeleton::getIndexOf(const Joint* _joint, bool _warning) const
{
  return templatedGetIndexOf<Joint, &Joint::getJointIndexInSkeleton>(
        this, _joint, "Joint", _warning);
}

//==============================================================================
size_t Skeleton::getNumDofs() const
{
  return mSkelCache.mDofs.size();
}

//==============================================================================
DegreeOfFreedom* Skeleton::getDof(size_t _idx)
{
  return getVectorObjectIfAvailable<DegreeOfFreedom*>(_idx, mSkelCache.mDofs);
}

//==============================================================================
const DegreeOfFreedom* Skeleton::getDof(size_t _idx) const
{
  return getVectorObjectIfAvailable<DegreeOfFreedom*>(_idx, mSkelCache.mDofs);
}

//==============================================================================
DegreeOfFreedom* Skeleton::getDof(const std::string& _name)
{
  return mNameMgrForDofs.getObject(_name);
}

//==============================================================================
const DegreeOfFreedom* Skeleton::getDof(const std::string& _name) const
{
  return mNameMgrForDofs.getObject(_name);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& Skeleton::getDofs()
{
  return mSkelCache.mDofs;
}

//==============================================================================
std::vector<const DegreeOfFreedom*> Skeleton::getDofs() const
{
  return convertToConstPtrVector<DegreeOfFreedom>(
        mSkelCache.mDofs, mSkelCache.mConstDofs);
}

//==============================================================================
size_t Skeleton::getIndexOf(const DegreeOfFreedom* _dof, bool _warning) const
{
  return templatedGetIndexOf<DegreeOfFreedom,
      &DegreeOfFreedom::getIndexInSkeleton>(
        this, _dof, "DegreeOfFreedom", _warning);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& Skeleton::getTreeDofs(size_t _treeIdx)
{
  return mTreeCache[_treeIdx].mDofs;
}

//==============================================================================
const std::vector<const DegreeOfFreedom*>& Skeleton::getTreeDofs(
    size_t _treeIdx) const
{
  return convertToConstPtrVector<DegreeOfFreedom>(
        mTreeCache[_treeIdx].mDofs, mTreeCache[_treeIdx].mConstDofs);
}

//==============================================================================
size_t Skeleton::getNumEndEffectors() const
{
  return mEndEffectors.size();
}

//==============================================================================
EndEffector* Skeleton::getEndEffector(size_t _idx)
{
  return getVectorObjectIfAvailable<EndEffector*>(_idx, mEndEffectors);
}

//==============================================================================
const EndEffector* Skeleton::getEndEffector(size_t _idx) const
{
  return getVectorObjectIfAvailable<EndEffector*>(_idx, mEndEffectors);
}

//==============================================================================
EndEffector* Skeleton::getEndEffector(const std::string& _name)
{
  return mNameMgrForEndEffectors.getObject(_name);
}

//==============================================================================
const EndEffector* Skeleton::getEndEffector(const std::string& _name) const
{
  return mNameMgrForEndEffectors.getObject(_name);
}

//==============================================================================
const std::shared_ptr<WholeBodyIK>& Skeleton::getIK(bool _createIfNull)
{
  if(nullptr == mWholeBodyIK && _createIfNull)
    createIK();

  return mWholeBodyIK;
}

//==============================================================================
const std::shared_ptr<WholeBodyIK>& Skeleton::getOrCreateIK()
{
  return getIK(true);
}

//==============================================================================
std::shared_ptr<const WholeBodyIK> Skeleton::getIK() const
{
  return mWholeBodyIK;
}

//==============================================================================
const std::shared_ptr<WholeBodyIK>& Skeleton::createIK()
{
  mWholeBodyIK = WholeBodyIK::create(mPtr.lock());
  return mWholeBodyIK;
}

//==============================================================================
void Skeleton::clearIK()
{
  mWholeBodyIK = nullptr;
}

//==============================================================================
Marker* Skeleton::getMarker(const std::string& _name)
{
  return mNameMgrForMarkers.getObject(_name);
}

//==============================================================================
const Marker* Skeleton::getMarker(const std::string& _name) const
{
  return const_cast<Skeleton*>(this)->getMarker(_name);
}

//==============================================================================
void Skeleton::setState(const Eigen::VectorXd& _state)
{
  assert(_state.size() % 2 == 0);

  size_t index = 0;
  size_t halfSize = _state.size() / 2;
  Joint* joint;

  for (size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
  {
    joint = mSkelCache.mBodyNodes[i]->getParentJoint();

    const size_t dof = joint->getNumDofs();

    if (dof)
    {
      joint->setPositions(_state.segment(index, dof));
      joint->setVelocities(_state.segment(index + halfSize, dof));

      index += dof;
    }
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getState() const
{
  Eigen::VectorXd state(2 * getNumDofs());

  state << getPositions(), getVelocities();

  return state;
}

//==============================================================================
void Skeleton::integratePositions(double _dt)
{
  for (size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    mSkelCache.mBodyNodes[i]->getParentJoint()->integratePositions(_dt);

  for (size_t i = 0; i < mSoftBodyNodes.size(); ++i)
  {
    for (size_t j = 0; j < mSoftBodyNodes[i]->getNumPointMasses(); ++j)
      mSoftBodyNodes[i]->getPointMass(j)->integratePositions(_dt);
  }
}

//==============================================================================
void Skeleton::integrateVelocities(double _dt)
{
  for (size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    mSkelCache.mBodyNodes[i]->getParentJoint()->integrateVelocities(_dt);

  for (size_t i = 0; i < mSoftBodyNodes.size(); ++i)
  {
    for (size_t j = 0; j < mSoftBodyNodes[i]->getNumPointMasses(); ++j)
      mSoftBodyNodes[i]->getPointMass(j)->integrateVelocities(_dt);
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getPositionDifferences(
    const Eigen::VectorXd& _q2, const Eigen::VectorXd& _q1) const
{
  if (static_cast<size_t>(_q2.size()) != getNumDofs()
      || static_cast<size_t>(_q1.size()) != getNumDofs())
  {
    dterr << "Skeleton::getPositionsDifference: q1's size[" << _q1.size()
          << "] or q2's size[" << _q2.size() << "is different with the dof ["
          << getNumDofs() << "]." << std::endl;
    return Eigen::VectorXd::Zero(getNumDofs());
  }

  Eigen::VectorXd dq(getNumDofs());

  for (const auto& bodyNode : mSkelCache.mBodyNodes)
  {
    const Joint* joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      const Eigen::VectorXd& q2Seg = _q2.segment(index, dof);
      const Eigen::VectorXd& q1Seg = _q1.segment(index, dof);
      dq.segment(index, dof) = joint->getPositionDifferences(q2Seg, q1Seg);
    }
  }

  return dq;
}

//==============================================================================
Eigen::VectorXd Skeleton::getVelocityDifferences(
    const Eigen::VectorXd& _dq2, const Eigen::VectorXd& _dq1) const
{
  if (static_cast<size_t>(_dq2.size()) != getNumDofs()
      || static_cast<size_t>(_dq1.size()) != getNumDofs())
  {
    dterr << "Skeleton::getPositionsDifference: dq1's size[" << _dq1.size()
          << "] or dq2's size[" << _dq2.size() << "is different with the dof ["
          << getNumDofs() << "]." << std::endl;
    return Eigen::VectorXd::Zero(getNumDofs());
  }

  // All the tangent spaces of Joint's configuration spaces are vector spaces.
  return _dq2 - _dq1;
}

//==============================================================================
static bool isValidBodyNode(const Skeleton* _skeleton,
                            const JacobianNode* _node,
                            const std::string& _fname)
{
  if (nullptr == _node)
  {
    dtwarn << "[Skeleton::" << _fname << "] Invalid BodyNode pointer: "
           << "nullptr. Returning zero Jacobian.\n";
    assert(false);
    return false;
  }

  // The given BodyNode should be in the Skeleton
  if (_node->getSkeleton().get() != _skeleton)
  {
    dtwarn << "[Skeleton::" << _fname << "] Attempting to get a Jacobian for a "
           "BodyNode [" << _node->getName() << "] (" << _node
           << ") that is not in this Skeleton [" << _skeleton->getName()
           << "] (" << _skeleton << "). Returning zero Jacobian.\n";
    assert(false);
    return false;
  }

  return true;
}

//==============================================================================
template <typename JacobianType>
void assignJacobian(JacobianType& _J,
                    const JacobianNode* _node,
                    const JacobianType& _JBodyNode)
{
  // Assign the BodyNode's Jacobian to the result Jacobian.
  size_t localIndex = 0;
  const auto& indices = _node->getDependentGenCoordIndices();
  for (const auto& index : indices)
  {
    // Each index should be less than the number of dofs of this Skeleton.
    assert(index < _node->getSkeleton()->getNumDofs());

    _J.col(index) = _JBodyNode.col(localIndex++);
  }
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobian(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::Jacobian J = math::Jacobian::Zero(6, _skel->getNumDofs());

  if ( !isValidBodyNode(_skel, _node, "getJacobian") )
    return J;

  const math::Jacobian JBodyNode = _node->getJacobian(args...);

  assignJacobian<math::Jacobian>(J, _node, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const JacobianNode* _node) const
{
  return variadicGetJacobian(this, _node);
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const JacobianNode* _node,
                                     const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const JacobianNode* _node,
                                     const Eigen::Vector3d& _localOffset) const
{
  return variadicGetJacobian(this, _node, _localOffset);
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const JacobianNode* _node,
                                     const Eigen::Vector3d& _localOffset,
                                     const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobian(this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetWorldJacobian(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::Jacobian J = math::Jacobian::Zero(6, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getWorldJacobian") )
    return J;

  const math::Jacobian JBodyNode = _node->getWorldJacobian(args...);

  assignJacobian<math::Jacobian>(J, _node, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getWorldJacobian(const JacobianNode* _node) const
{
  return variadicGetWorldJacobian(this, _node);
}

//==============================================================================
math::Jacobian Skeleton::getWorldJacobian(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset) const
{
  return variadicGetWorldJacobian(this, _node, _localOffset);
}

//==============================================================================
template <typename ...Args>
math::LinearJacobian variadicGetLinearJacobian(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::LinearJacobian J =
      math::LinearJacobian::Zero(3, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getLinearJacobian") )
    return J;

  const math::LinearJacobian JBodyNode = _node->getLinearJacobian(args...);

  assignJacobian<math::LinearJacobian>(J, _node, JBodyNode);

  return J;
}


//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobian(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobian(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobian(
        this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::AngularJacobian variadicGetAngularJacobian(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::AngularJacobian J =
      math::AngularJacobian::Zero(3, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getAngularJacobian") )
    return J;

  const math::AngularJacobian JBodyNode =
      _node->getAngularJacobian(args...);

  assignJacobian<math::AngularJacobian>(J, _node, JBodyNode);

  return J;
}

//==============================================================================
math::AngularJacobian Skeleton::getAngularJacobian(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetAngularJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobianSpatialDeriv(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::Jacobian dJ = math::Jacobian::Zero(6, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getJacobianSpatialDeriv") )
    return dJ;

  const math::Jacobian dJBodyNode = _node->getJacobianSpatialDeriv(args...);

  assignJacobian<math::Jacobian>(dJ, _node, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node) const
{
  return variadicGetJacobianSpatialDeriv(this, _node);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianSpatialDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset) const
{
  return variadicGetJacobianSpatialDeriv(this, _node, _localOffset);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianSpatialDeriv(
        this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobianClassicDeriv(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::Jacobian dJ = math::Jacobian::Zero(6, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getJacobianClassicDeriv") )
    return dJ;

  const math::Jacobian dJBodyNode = _node->getJacobianClassicDeriv(args...);

  assignJacobian<math::Jacobian>(dJ, _node, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const JacobianNode* _node) const
{
  return variadicGetJacobianClassicDeriv(this, _node);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianClassicDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianClassicDeriv(
        this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::LinearJacobian variadicGetLinearJacobianDeriv(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::LinearJacobian dJv =
      math::LinearJacobian::Zero(3, _skel->getNumDofs());

  if ( !isValidBodyNode(_skel, _node, "getLinearJacobianDeriv") )
    return dJv;

  const math::LinearJacobian dJvBodyNode =
      _node->getLinearJacobianDeriv(args...);

  assignJacobian<math::LinearJacobian>(dJv, _node, dJvBodyNode);

  return dJv;
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobianDeriv(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobianDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobianDeriv(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobianDeriv(
        this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::AngularJacobian variadicGetAngularJacobianDeriv(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::AngularJacobian dJw =
      math::AngularJacobian::Zero(3, _skel->getNumDofs());

  if ( !isValidBodyNode(_skel, _node, "getAngularJacobianDeriv") )
    return dJw;

  const math::AngularJacobian dJwBodyNode =
      _node->getAngularJacobianDeriv(args...);

  assignJacobian<math::AngularJacobian>(dJw, _node, dJwBodyNode);

  return dJw;
}

//==============================================================================
math::AngularJacobian Skeleton::getAngularJacobianDeriv(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetAngularJacobianDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
double Skeleton::getMass() const
{
  return mTotalMass;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getMassMatrix(size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mMassMatrix)
    updateMassMatrix(_treeIdx);
  return mTreeCache[_treeIdx].mM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getMassMatrix() const
{
  if (mSkelCache.mDirty.mMassMatrix)
    updateMassMatrix();
  return mSkelCache.mM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getAugMassMatrix(size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mAugMassMatrix)
    updateAugMassMatrix(_treeIdx);

  return mTreeCache[_treeIdx].mAugM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getAugMassMatrix() const
{
  if (mSkelCache.mDirty.mAugMassMatrix)
    updateAugMassMatrix();

  return mSkelCache.mAugM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvMassMatrix(size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mInvMassMatrix)
    updateInvMassMatrix(_treeIdx);

  return mTreeCache[_treeIdx].mInvM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvMassMatrix() const
{
  if (mSkelCache.mDirty.mInvMassMatrix)
    updateInvMassMatrix();

  return mSkelCache.mInvM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvAugMassMatrix(size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mInvAugMassMatrix)
    updateInvAugMassMatrix(_treeIdx);

  return mTreeCache[_treeIdx].mInvAugM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvAugMassMatrix() const
{
  if (mSkelCache.mDirty.mInvAugMassMatrix)
    updateInvAugMassMatrix();

  return mSkelCache.mInvAugM;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisForces(size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mCoriolisForces)
    updateCoriolisForces(_treeIdx);

  return mTreeCache[_treeIdx].mCvec;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisForces() const
{
  if (mSkelCache.mDirty.mCoriolisForces)
    updateCoriolisForces();

  return mSkelCache.mCvec;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getGravityForces(size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mGravityForces)
    updateGravityForces(_treeIdx);

  return mTreeCache[_treeIdx].mG;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getGravityForces() const
{
  if (mSkelCache.mDirty.mGravityForces)
    updateGravityForces();

  return mSkelCache.mG;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisAndGravityForces(size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mCoriolisAndGravityForces)
    updateCoriolisAndGravityForces(_treeIdx);

  return mTreeCache[_treeIdx].mCg;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisAndGravityForces() const
{
  if (mSkelCache.mDirty.mCoriolisAndGravityForces)
    updateCoriolisAndGravityForces();

  return mSkelCache.mCg;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getExternalForces(size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mExternalForces)
    updateExternalForces(_treeIdx);

  return mTreeCache[_treeIdx].mFext;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getExternalForces() const
{
  if (mSkelCache.mDirty.mExternalForces)
    updateExternalForces();

  return mSkelCache.mFext;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getConstraintForces(size_t _treeIdx) const
{
  return computeConstraintForces(mTreeCache[_treeIdx]);
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getConstraintForces() const
{
  return computeConstraintForces(mSkelCache);
}

//==============================================================================
//const Eigen::VectorXd& Skeleton::getDampingForceVector() {
//  if (mIsDampingForceVectorDirty)
//    updateDampingForceVector();
//  return mFd;
//}

//==============================================================================
void Skeleton::draw(renderer::RenderInterface* _ri, const Eigen::Vector4d& _color,
                    bool _useDefaultColor) const
{
  for(size_t i=0; i<getNumTrees(); ++i)
    getRootBodyNode(i)->draw(_ri, _color, _useDefaultColor);
}

//==============================================================================
void Skeleton::drawMarkers(renderer::RenderInterface* _ri,
                           const Eigen::Vector4d& _color,
                           bool _useDefaultColor) const
{
  getRootBodyNode()->drawMarkers(_ri, _color, _useDefaultColor);
}

//==============================================================================
Skeleton::Skeleton(const Properties& _properties)
  : mSkeletonP(""),
    mTotalMass(0.0),
    mIsImpulseApplied(false),
    mUnionSize(1)
{
  setProperties(_properties);
}

//==============================================================================
void Skeleton::setPtr(const SkeletonPtr& _ptr)
{
  mPtr = _ptr;
  resetUnion();
}

//==============================================================================
void Skeleton::registerBodyNode(BodyNode* _newBodyNode)
{
#ifndef NDEBUG  // Debug mode
  std::vector<BodyNode*>::iterator repeat =
      std::find(mSkelCache.mBodyNodes.begin(), mSkelCache.mBodyNodes.end(),
                _newBodyNode);
  if(repeat != mSkelCache.mBodyNodes.end())
  {
    dterr << "[Skeleton::registerBodyNode] Attempting to double-register the "
          << "BodyNode named [" << _newBodyNode->getName() << "] in the "
          << "Skeleton named [" << getName() << "]. Please report this as a "
          << "bug!\n";
    assert(false);
    return;
  }
#endif // -------- Debug mode

  mSkelCache.mBodyNodes.push_back(_newBodyNode);
  if(nullptr == _newBodyNode->getParentBodyNode())
  {
    _newBodyNode->mIndexInTree = 0;
    mTreeCache.push_back(DataCache());
    mTreeCache.back().mBodyNodes.push_back(_newBodyNode);
    _newBodyNode->mTreeIndex = mTreeCache.size()-1;
  }
  else
  {
    size_t tree = _newBodyNode->getParentBodyNode()->getTreeIndex();
    _newBodyNode->mTreeIndex = tree;
    DataCache& cache = mTreeCache[tree];
    cache.mBodyNodes.push_back(_newBodyNode);
    _newBodyNode->mIndexInTree = cache.mBodyNodes.size()-1;
  }

  _newBodyNode->mSkeleton = getPtr();
  _newBodyNode->mIndexInSkeleton = mSkelCache.mBodyNodes.size()-1;
  addEntryToBodyNodeNameMgr(_newBodyNode);
  registerJoint(_newBodyNode->getParentJoint());

  SoftBodyNode* softBodyNode = dynamic_cast<SoftBodyNode*>(_newBodyNode);
  if (softBodyNode)
  {
    mSoftBodyNodes.push_back(softBodyNode);
    addEntryToSoftBodyNodeNameMgr(softBodyNode);
  }

  _newBodyNode->init(getPtr());

  std::vector<EndEffector*>& endEffectors = _newBodyNode->mEndEffectors;
  for(EndEffector* ee : endEffectors)
    registerEndEffector(ee);

  updateTotalMass();
  updateCacheDimensions(_newBodyNode->mTreeIndex);

#ifndef NDEBUG // Debug mode
  for(size_t i=0; i<mSkelCache.mBodyNodes.size(); ++i)
  {
    if(mSkelCache.mBodyNodes[i]->mIndexInSkeleton != i)
    {
      dterr << "[Skeleton::registerBodyNode] BodyNode named ["
            << mSkelCache.mBodyNodes[i]->getName() << "] in Skeleton ["
            << getName() << "] is mistaken about its index in the Skeleton ( "
            << i << " : " << mSkelCache.mBodyNodes[i]->mIndexInSkeleton
            << "). Please report this as a bug!\n";
      assert(false);
    }
  }

  for(size_t i=0; i<mTreeCache.size(); ++i)
  {
    const DataCache& cache = mTreeCache[i];
    for(size_t j=0; j<cache.mBodyNodes.size(); ++j)
    {
      BodyNode* bn = cache.mBodyNodes[j];
      if(bn->mTreeIndex != i)
      {
        dterr << "[Skeleton::registerBodyNode] BodyNode named ["
              << bn->getName() << "] in Skeleton [" << getName() << "] is "
              << "mistaken about its tree's index (" << i << " : "
              << bn->mTreeIndex << "). Please report this as a bug!\n";
        assert(false);
      }

      if(bn->mIndexInTree != j)
      {
        dterr << "[Skeleton::registerBodyNode] BodyNode named ["
              << bn->getName() << "] in Skeleton [" << getName() << "] is "
              << "mistaken about its index in the tree (" << j << " : "
              << bn->mIndexInTree << "). Please report this as a bug!\n";
        assert(false);
      }
    }
  }
#endif // ------- Debug mode

  _newBodyNode->mStructuralChangeSignal.raise(_newBodyNode);
}

//==============================================================================
void Skeleton::registerJoint(Joint* _newJoint)
{
  if (nullptr == _newJoint)
  {
    dterr << "[Skeleton::registerJoint] Error: Attempting to add a nullptr "
             "Joint to the Skeleton named [" << mSkeletonP.mName << "]. Report "
             "this as a bug!\n";
    assert(false);
    return;
  }

  addEntryToJointNameMgr(_newJoint);
  _newJoint->registerDofs();

  size_t tree = _newJoint->getChildBodyNode()->getTreeIndex();
  std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
  for(size_t i = 0; i < _newJoint->getNumDofs(); ++i)
  {
    mSkelCache.mDofs.push_back(_newJoint->getDof(i));
    _newJoint->getDof(i)->mIndexInSkeleton = mSkelCache.mDofs.size()-1;

    treeDofs.push_back(_newJoint->getDof(i));
    _newJoint->getDof(i)->mIndexInTree = treeDofs.size()-1;
  }
}

//==============================================================================
void Skeleton::registerEndEffector(EndEffector* _newEndEffector)
{
#ifndef NDEBUG // Debug mode
  std::vector<EndEffector*>::iterator it = find(mEndEffectors.begin(),
                                                mEndEffectors.end(),
                                                _newEndEffector);

  if(it != mEndEffectors.end())
  {
    dterr << "[Skeleton::registerEndEffector] Attempting to double-register "
           << "an EndEffector named [" << _newEndEffector->getName() << "] ("
           << _newEndEffector << ") in the Skeleton named [" << getName()
           << "] (" << this << "). This is most likely a bug. Please report "
           << "this!\n";
    assert(false);
    return;
  }
#endif // ------- Debug mode

  mEndEffectors.push_back(_newEndEffector);
  _newEndEffector->mIndexInSkeleton = mEndEffectors.size()-1;

  // We sneak this in here to patch the issue where child JacobianNodes were
  // going unrecognized. This only works for EndEffector, but EndEffector is
  // the only JacobianNode type besides BodyNode in v5.1.
  _newEndEffector->mBodyNode->mChildJacobianNodes.insert(_newEndEffector);

  // The EndEffector name gets added when the EndEffector is constructed, so we
  // don't need to add it here.
}

//==============================================================================
void Skeleton::unregisterBodyNode(BodyNode* _oldBodyNode)
{
  unregisterJoint(_oldBodyNode->getParentJoint());

  std::vector<EndEffector*>& endEffectors = _oldBodyNode->mEndEffectors;
  for(EndEffector* ee : endEffectors)
    unregisterEndEffector(ee);

  mNameMgrForBodyNodes.removeName(_oldBodyNode->getName());

  size_t index = _oldBodyNode->getIndexInSkeleton();
  assert(mSkelCache.mBodyNodes[index] == _oldBodyNode);
  mSkelCache.mBodyNodes.erase(mSkelCache.mBodyNodes.begin()+index);
  for(size_t i=index; i < mSkelCache.mBodyNodes.size(); ++i)
  {
    BodyNode* bn = mSkelCache.mBodyNodes[i];
    bn->mIndexInSkeleton = i;
  }

  if(nullptr == _oldBodyNode->getParentBodyNode())
  {
    // If the parent of this BodyNode is a nullptr, then this is the root of its
    // tree. If the root of the tree is being removed, then the tree itself
    // should be destroyed.

    // There is no way that any child BodyNodes of this root BodyNode are still
    // registered, because the BodyNodes always get unregistered from leaf to
    // root.

    size_t tree = _oldBodyNode->getTreeIndex();
    assert(mTreeCache[tree].mBodyNodes.size() == 1);
    assert(mTreeCache[tree].mBodyNodes[0] == _oldBodyNode);
    mTreeCache.erase(mTreeCache.begin() + tree);

    // Decrease the tree index of every BodyNode whose tree index is higher than
    // the one which is being removed. None of the BodyNodes that predate the
    // current one can have a higher tree index, so they can be ignored.
    for(size_t i=index; i < mSkelCache.mBodyNodes.size(); ++i)
    {
      BodyNode* bn = mSkelCache.mBodyNodes[i];
      if(bn->mTreeIndex > tree)
        --bn->mTreeIndex;
    }

    updateCacheDimensions(mSkelCache);
  }
  else
  {
    size_t tree = _oldBodyNode->getTreeIndex();
    size_t indexInTree = _oldBodyNode->getIndexInTree();
    assert(mTreeCache[tree].mBodyNodes[indexInTree] == _oldBodyNode);
    mTreeCache[tree].mBodyNodes.erase(
          mTreeCache[tree].mBodyNodes.begin() + indexInTree);

    for(size_t i=indexInTree; i<mTreeCache[tree].mBodyNodes.size(); ++i)
      mTreeCache[tree].mBodyNodes[i]->mIndexInTree = i;

    updateCacheDimensions(tree);
  }

  SoftBodyNode* soft = dynamic_cast<SoftBodyNode*>(_oldBodyNode);
  if(soft)
  {
    mNameMgrForSoftBodyNodes.removeName(soft->getName());

    mSoftBodyNodes.erase(std::remove(mSoftBodyNodes.begin(),
                                     mSoftBodyNodes.end(), soft),
                         mSoftBodyNodes.end());
  }
}

//==============================================================================
void Skeleton::unregisterJoint(Joint* _oldJoint)
{
  if (nullptr == _oldJoint)
  {
    dterr << "[Skeleton::unregisterJoint] Attempting to unregister nullptr "
          << "Joint from Skeleton named [" << getName() << "]. Report this as "
          << "a bug!\n";
    assert(false);
    return;
  }

  mNameMgrForJoints.removeName(_oldJoint->getName());

  size_t tree = _oldJoint->getChildBodyNode()->getTreeIndex();
  std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
  std::vector<DegreeOfFreedom*>& skelDofs = mSkelCache.mDofs;

  size_t firstSkelIndex = INVALID_INDEX;
  size_t firstTreeIndex = INVALID_INDEX;
  for (size_t i = 0; i < _oldJoint->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = _oldJoint->getDof(i);
    mNameMgrForDofs.removeObject(dof);

    firstSkelIndex = std::min(firstSkelIndex, dof->getIndexInSkeleton());
    skelDofs.erase(
          std::remove(skelDofs.begin(), skelDofs.end(), dof), skelDofs.end());

    firstTreeIndex = std::min(firstTreeIndex, dof->getIndexInTree());
    treeDofs.erase(
          std::remove(treeDofs.begin(), treeDofs.end(), dof), treeDofs.end());
  }

  for (size_t i = firstSkelIndex; i < skelDofs.size(); ++i)
  {
    DegreeOfFreedom* dof = skelDofs[i];
    dof->mIndexInSkeleton = i;
  }

  for (size_t i = firstTreeIndex; i < treeDofs.size(); ++i)
  {
    DegreeOfFreedom* dof = treeDofs[i];
    dof->mIndexInTree = i;
  }
}

//==============================================================================
void Skeleton::unregisterEndEffector(EndEffector* _oldEndEffector)
{
  size_t index = _oldEndEffector->getIndexInSkeleton();
  assert(mEndEffectors[index] == _oldEndEffector);
  mEndEffectors.erase(mEndEffectors.begin() + index);

  for(size_t i=index; i < mEndEffectors.size(); ++i)
  {
    EndEffector* ee = mEndEffectors[i];
    ee->mIndexInSkeleton = i;
  }

  mNameMgrForEndEffectors.removeName(_oldEndEffector->getName());
}

//==============================================================================
bool Skeleton::moveBodyNodeTree(Joint* _parentJoint, BodyNode* _bodyNode,
                                SkeletonPtr _newSkeleton,
                                BodyNode* _parentNode)
{
  if(nullptr == _bodyNode)
  {
    dterr << "[Skeleton::moveBodyNodeTree] Skeleton named [" << getName()
          << "] (" << this << ") is attempting to move a nullptr BodyNode. "
          << "Please report this as a bug!\n";
    assert(false);
    return false;
  }

  if(this != _bodyNode->getSkeleton().get())
  {
    dterr << "[Skeleton::moveBodyNodeTree] Skeleton named [" << getName()
          << "] (" << this << ") is attempting to move a BodyNode named ["
          << _bodyNode->getName() << "] even though it belongs to another "
          << "Skeleton [" << _bodyNode->getSkeleton()->getName() << "] ("
          << _bodyNode->getSkeleton() << "). Please report this as a bug!\n";
    assert(false);
    return false;
  }

  if( (nullptr == _parentJoint)
      && (_bodyNode->getParentBodyNode() == _parentNode)
      && (this == _newSkeleton.get()) )
  {
    // Short-circuit if the BodyNode is already in the requested place, and its
    // Joint does not need to be changed
    return false;
  }

  if(_bodyNode == _parentNode)
  {
    dterr << "[Skeleton::moveBodyNodeTree] Attempting to move BodyNode named ["
          << _bodyNode->getName() << "] (" << _bodyNode << ") to be its own "
          << "parent. This is not permitted!\n";
    return false;
  }

  if(_parentNode && _parentNode->descendsFrom(_bodyNode))
  {
    dterr << "[Skeleton::moveBodyNodeTree] Attempting to move BodyNode named ["
          << _bodyNode->getName() << "] of Skeleton [" << getName() << "] ("
          << this << ") to be a child of BodyNode [" << _parentNode->getName()
          << "] in Skeleton [" << _newSkeleton->getName() << "] ("
          << _newSkeleton << "), but that would create a closed kinematic "
          << "chain, which is not permitted! Nothing will be moved.\n";
    return false;
  }

  if(nullptr == _newSkeleton)
  {
    if(nullptr == _parentNode)
    {
      dterr << "[Skeleton::moveBodyNodeTree] Attempting to move a BodyNode "
            << "tree starting from [" << _bodyNode->getName() << "] in "
            << "Skeleton [" << getName() << "] into a nullptr Skeleton. This "
            << "is not permitted!\n";
      return false;
    }

    _newSkeleton = _parentNode->getSkeleton();
  }

  if(_parentNode && _newSkeleton != _parentNode->getSkeleton())
  {
    dterr << "[Skeleton::moveBodyNodeTree] Mismatch between the specified "
          << "Skeleton [" << _newSkeleton->getName() << "] (" << _newSkeleton
          << ") and the specified new parent BodyNode ["
          << _parentNode->getName() << "] whose actual Skeleton is named ["
          << _parentNode->getSkeleton()->getName() << "] ("
          << _parentNode->getSkeleton() << ") while attempting to move a "
          << "BodyNode tree starting from [" << _bodyNode->getName() << "] in "
          << "Skeleton [" << getName() << "] (" << this << ")\n";
    return false;
  }

  std::vector<BodyNode*> tree = extractBodyNodeTree(_bodyNode);

  Joint* originalParent = _bodyNode->getParentJoint();
  if(originalParent != _parentJoint)
  {
    _bodyNode->mParentJoint = _parentJoint;
    _parentJoint->mChildBodyNode = _bodyNode;
    delete originalParent;
  }

  if(_parentNode != _bodyNode->getParentBodyNode())
  {
    _bodyNode->mParentBodyNode = _parentNode;
    if(_parentNode)
    {
      _parentNode->mChildBodyNodes.push_back(_bodyNode);
      _bodyNode->changeParentFrame(_parentNode);
    }
    else
    {
      _bodyNode->changeParentFrame(Frame::World());
    }
  }
  _newSkeleton->receiveBodyNodeTree(tree);

  return true;
}

//==============================================================================
std::pair<Joint*, BodyNode*> Skeleton::cloneBodyNodeTree(
    Joint* _parentJoint, const BodyNode* _bodyNode,
    const SkeletonPtr& _newSkeleton, BodyNode* _parentNode,
    bool _recursive) const
{
  std::pair<Joint*, BodyNode*> root(nullptr, nullptr);
  std::vector<const BodyNode*> tree;
  if(_recursive)
    tree = constructBodyNodeTree(_bodyNode);
  else
    tree.push_back(_bodyNode);

  std::map<std::string, BodyNode*> nameMap;
  std::vector<BodyNode*> clones;
  clones.reserve(tree.size());

  for(size_t i=0; i<tree.size(); ++i)
  {
    const BodyNode* original = tree[i];
    // If this is the root of the tree, and the user has requested a change in
    // its parent Joint, use the specified parent Joint instead of created a
    // clone
    Joint* joint = (i==0 && _parentJoint != nullptr) ? _parentJoint :
        original->getParentJoint()->clone();

    BodyNode* newParent = i==0 ? _parentNode :
        nameMap[original->getParentBodyNode()->getName()];

    BodyNode* clone = original->clone(newParent, joint);
    clones.push_back(clone);
    nameMap[clone->getName()] = clone;

    if(0==i)
    {
      root.first = joint;
      root.second = clone;
    }
  }

  _newSkeleton->receiveBodyNodeTree(clones);
  return root;
}

//==============================================================================
template <typename BodyNodeT>
static void recursiveConstructBodyNodeTree(
    std::vector<BodyNodeT*>& tree, BodyNodeT* _currentBodyNode)
{
  tree.push_back(_currentBodyNode);
  for(size_t i=0; i<_currentBodyNode->getNumChildBodyNodes(); ++i)
    recursiveConstructBodyNodeTree(tree, _currentBodyNode->getChildBodyNode(i));
}

//==============================================================================
std::vector<const BodyNode*> Skeleton::constructBodyNodeTree(
    const BodyNode* _bodyNode) const
{
  std::vector<const BodyNode*> tree;
  recursiveConstructBodyNodeTree<const BodyNode>(tree, _bodyNode);

  return tree;
}

//==============================================================================
std::vector<BodyNode*> Skeleton::constructBodyNodeTree(BodyNode *_bodyNode)
{
  std::vector<BodyNode*> tree;
  recursiveConstructBodyNodeTree<BodyNode>(tree, _bodyNode);

  return tree;
}

//==============================================================================
std::vector<BodyNode*> Skeleton::extractBodyNodeTree(BodyNode* _bodyNode)
{
  std::vector<BodyNode*> tree = constructBodyNodeTree(_bodyNode);

  // Go backwards to minimize the number of shifts needed
  std::vector<BodyNode*>::reverse_iterator rit;
  // Go backwards to minimize the amount of element shifting in the vectors
  for(rit = tree.rbegin(); rit != tree.rend(); ++rit)
    unregisterBodyNode(*rit);

  for(size_t i=0; i<mSkelCache.mBodyNodes.size(); ++i)
    mSkelCache.mBodyNodes[i]->init(getPtr());

  return tree;
}

//==============================================================================
void Skeleton::receiveBodyNodeTree(const std::vector<BodyNode*>& _tree)
{
  for(BodyNode* bn : _tree)
    registerBodyNode(bn);
}

//==============================================================================
void Skeleton::updateTotalMass()
{
  mTotalMass = 0.0;
  for(size_t i=0; i<getNumBodyNodes(); ++i)
    mTotalMass += getBodyNode(i)->getMass();
}

//==============================================================================
void Skeleton::updateCacheDimensions(Skeleton::DataCache& _cache)
{
  size_t dof = _cache.mDofs.size();
  _cache.mM        = Eigen::MatrixXd::Zero(dof, dof);
  _cache.mAugM     = Eigen::MatrixXd::Zero(dof, dof);
  _cache.mInvM     = Eigen::MatrixXd::Zero(dof, dof);
  _cache.mInvAugM  = Eigen::MatrixXd::Zero(dof, dof);
  _cache.mCvec     = Eigen::VectorXd::Zero(dof);
  _cache.mG        = Eigen::VectorXd::Zero(dof);
  _cache.mCg       = Eigen::VectorXd::Zero(dof);
  _cache.mFext     = Eigen::VectorXd::Zero(dof);
  _cache.mFc       = Eigen::VectorXd::Zero(dof);
}

//==============================================================================
void Skeleton::updateCacheDimensions(size_t _treeIdx)
{
  updateCacheDimensions(mTreeCache[_treeIdx]);
  updateCacheDimensions(mSkelCache);

  notifyArticulatedInertiaUpdate(_treeIdx);
}

//==============================================================================
void Skeleton::updateArticulatedInertia(size_t _tree) const
{
  DataCache& cache = mTreeCache[_tree];
  for (std::vector<BodyNode*>::const_reverse_iterator it = cache.mBodyNodes.rbegin();
       it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->updateArtInertia(mSkeletonP.mTimeStep);
  }

  cache.mDirty.mArticulatedInertia = false;
}

//==============================================================================
void Skeleton::updateArticulatedInertia() const
{
  for(size_t i=0; i<mTreeCache.size(); ++i)
  {
    DataCache& cache = mTreeCache[i];
    if(cache.mDirty.mArticulatedInertia)
      updateArticulatedInertia(i);
  }

  mSkelCache.mDirty.mArticulatedInertia = false;
}

//==============================================================================
void Skeleton::updateMassMatrix(size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mM.cols()) == dof
         && static_cast<size_t>(cache.mM.rows()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mMassMatrix = false;
    return;
  }

  cache.mM.setZero();

  // Backup the original internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  // Clear out the accelerations of the dofs in this tree so that we can set
  // them to 1.0 one at a time to build up the mass matrix
  for (size_t i = 0; i < dof; ++i)
    cache.mDofs[i]->setAcceleration(0.0);

  for (size_t j = 0; j < dof; ++j)
  {
    // Set the acceleration of this DOF to 1.0 while all the rest are 0.0
    cache.mDofs[j]->setAcceleration(1.0);

    // Prepare cache data
    for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
         it != cache.mBodyNodes.end(); ++it)
    {
      (*it)->updateMassMatrix();
    }

    // Mass matrix
    for (std::vector<BodyNode*>::const_reverse_iterator it =
         cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
    {
      (*it)->aggregateMassMatrix(cache.mM, j);
      size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        size_t iStart = (*it)->mParentJoint->getIndexInTree(0);

        if (iStart + localDof < j)
          break;
      }
    }

    // Set the acceleration of this DOF back to 0.0
    cache.mDofs[j]->setAcceleration(0.0);
  }
  cache.mM.triangularView<Eigen::StrictlyUpper>() = cache.mM.transpose();

  // Restore the original generalized accelerations
  const_cast<Skeleton*>(this)->setAccelerations(originalGenAcceleration);

  cache.mDirty.mMassMatrix = false;
}

//==============================================================================
void Skeleton::updateMassMatrix() const
{
  size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<size_t>(mSkelCache.mM.cols()) == dof
         && static_cast<size_t>(mSkelCache.mM.rows()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mMassMatrix = false;
    return;
  }

  mSkelCache.mM.setZero();

  for(size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::MatrixXd& treeM = getMassMatrix(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    size_t nTreeDofs = treeDofs.size();
    for(size_t i=0; i<nTreeDofs; ++i)
    {
      for(size_t j=0; j<nTreeDofs; ++j)
      {
        size_t ki = treeDofs[i]->getIndexInSkeleton();
        size_t kj = treeDofs[j]->getIndexInSkeleton();

        mSkelCache.mM(ki,kj) = treeM(i,j);
      }
    }
  }

  mSkelCache.mDirty.mMassMatrix = false;
}

//==============================================================================
void Skeleton::updateAugMassMatrix(size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mAugM.cols()) == dof
         && static_cast<size_t>(cache.mAugM.rows()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mAugMassMatrix = false;
    return;
  }

  cache.mAugM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  // Clear out the accelerations of the DOFs in this tree so that we can set
  // them to 1.0 one at a time to build up the augmented mass matrix
  for (size_t i = 0; i < dof; ++i)
    cache.mDofs[i]->setAcceleration(0.0);

  for (size_t j = 0; j < dof; ++j)
  {
    // Set the acceleration of this DOF to 1.0 while all the rest are 0.0
    cache.mDofs[j]->setAcceleration(1.0);

    // Prepare cache data
    for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
         it != cache.mBodyNodes.end(); ++it)
    {
      (*it)->updateMassMatrix();
    }

    // Augmented Mass matrix
    for (std::vector<BodyNode*>::const_reverse_iterator it =
         cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
    {
      (*it)->aggregateAugMassMatrix(cache.mAugM, j, mSkeletonP.mTimeStep);
      size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        size_t iStart = (*it)->mParentJoint->getIndexInTree(0);

        if (iStart + localDof < j)
          break;
      }
    }

    // Set the acceleration of this DOF back to 0.0
    cache.mDofs[j]->setAcceleration(0.0);
  }
  cache.mAugM.triangularView<Eigen::StrictlyUpper>() = cache.mAugM.transpose();

  // Restore the origianl internal force
  const_cast<Skeleton*>(this)->setAccelerations(originalGenAcceleration);

  cache.mDirty.mAugMassMatrix = false;
}

//==============================================================================
void Skeleton::updateAugMassMatrix() const
{
  size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<size_t>(mSkelCache.mAugM.cols()) == dof
         && static_cast<size_t>(mSkelCache.mAugM.rows()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mMassMatrix = false;
    return;
  }

  mSkelCache.mAugM.setZero();

  for(size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::MatrixXd& treeAugM = getAugMassMatrix(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    size_t nTreeDofs = treeDofs.size();
    for(size_t i=0; i<nTreeDofs; ++i)
    {
      for(size_t j=0; j<nTreeDofs; ++j)
      {
        size_t ki = treeDofs[i]->getIndexInSkeleton();
        size_t kj = treeDofs[j]->getIndexInSkeleton();

        mSkelCache.mAugM(ki,kj) = treeAugM(i,j);
      }
    }
  }

  mSkelCache.mDirty.mAugMassMatrix = false;
}

//==============================================================================
void Skeleton::updateInvMassMatrix(size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mInvM.cols()) == dof
         && static_cast<size_t>(cache.mInvM.rows()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mInvMassMatrix = false;
    return;
  }

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // cache.mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = getForces();

  // Clear out the forces of the dofs in this tree so that we can set them to
  // 1.0 one at a time to build up the inverse mass matrix
  for (size_t i = 0; i < dof; ++i)
    cache.mDofs[i]->setForce(0.0);

  for (size_t j = 0; j < dof; ++j)
  {
    // Set the force of this DOF to 1.0 while all the rest are 0.0
    cache.mDofs[j]->setForce(1.0);

    // Prepare cache data
    for (std::vector<BodyNode*>::const_reverse_iterator it =
         cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
    {
      (*it)->updateInvMassMatrix();
    }

    // Inverse of mass matrix
    for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
         it != cache.mBodyNodes.end(); ++it)
    {
      (*it)->aggregateInvMassMatrix(cache.mInvM, j);
      size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        size_t iStart = (*it)->mParentJoint->getIndexInTree(0);

        if (iStart + localDof > j)
          break;
      }
    }

    // Set the force of this DOF back to 0.0
    cache.mDofs[j]->setForce(0.0);
  }
  cache.mInvM.triangularView<Eigen::StrictlyLower>() = cache.mInvM.transpose();

  // Restore the original internal force
  const_cast<Skeleton*>(this)->setForces(originalInternalForce);

  cache.mDirty.mInvMassMatrix = false;
}

//==============================================================================
void Skeleton::updateInvMassMatrix() const
{
  size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<size_t>(mSkelCache.mInvM.cols()) == dof
         && static_cast<size_t>(mSkelCache.mInvM.rows()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mInvMassMatrix = false;
    return;
  }

  mSkelCache.mInvM.setZero();

  for(size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::MatrixXd& treeInvM = getInvMassMatrix(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    size_t nTreeDofs = treeDofs.size();
    for(size_t i=0; i<nTreeDofs; ++i)
    {
      for(size_t j=0; j<nTreeDofs; ++j)
      {
        size_t ki = treeDofs[i]->getIndexInSkeleton();
        size_t kj = treeDofs[j]->getIndexInSkeleton();

        mSkelCache.mInvM(ki,kj) = treeInvM(i,j);
      }
    }
  }

  mSkelCache.mDirty.mInvMassMatrix = false;
}

//==============================================================================
void Skeleton::updateInvAugMassMatrix(size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mInvAugM.cols()) == dof
         && static_cast<size_t>(cache.mInvAugM.rows()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mInvAugMassMatrix = false;
    return;
  }

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = getForces();

  // Clear out the forces of the dofs in this tree so that we can set them to
  // 1.0 one at a time to build up the inverse augmented mass matrix
  for (size_t i = 0; i < dof; ++i)
    cache.mDofs[i]->setForce(0.0);

  for (size_t j = 0; j < dof; ++j)
  {
    // Set the force of this DOF to 1.0 while all the rest are 0.0
    cache.mDofs[j]->setForce(1.0);

    // Prepare cache data
    for (std::vector<BodyNode*>::const_reverse_iterator it =
         cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
    {
      (*it)->updateInvAugMassMatrix();
    }

    // Inverse of augmented mass matrix
    for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
         it != cache.mBodyNodes.end(); ++it)
    {
      (*it)->aggregateInvAugMassMatrix(cache.mInvAugM, j, mSkeletonP.mTimeStep);
      size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        size_t iStart = (*it)->mParentJoint->getIndexInTree(0);

        if (iStart + localDof > j)
          break;
      }
    }

    // Set the force of this DOF back to 0.0
    cache.mDofs[j]->setForce(0.0);
  }
  cache.mInvAugM.triangularView<Eigen::StrictlyLower>() =
      cache.mInvAugM.transpose();

  // Restore the original internal force
  const_cast<Skeleton*>(this)->setForces(originalInternalForce);

  cache.mDirty.mInvAugMassMatrix = false;
}

//==============================================================================
void Skeleton::updateInvAugMassMatrix() const
{
  size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<size_t>(mSkelCache.mInvAugM.cols()) == dof
         && static_cast<size_t>(mSkelCache.mInvAugM.rows()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mInvAugMassMatrix = false;
    return;
  }

  mSkelCache.mInvAugM.setZero();

  for(size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::MatrixXd& treeInvAugM = getInvAugMassMatrix(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    size_t nTreeDofs = treeDofs.size();
    for(size_t i=0; i<nTreeDofs; ++i)
    {
      for(size_t j=0; j<nTreeDofs; ++j)
      {
        size_t ki = treeDofs[i]->getIndexInSkeleton();
        size_t kj = treeDofs[j]->getIndexInSkeleton();

        mSkelCache.mInvAugM(ki,kj) = treeInvAugM(i,j);
      }
    }
  }

  mSkelCache.mDirty.mInvAugMassMatrix = false;
}

//==============================================================================
void Skeleton::updateCoriolisForces(size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mCvec.size()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mCoriolisForces = false;
    return;
  }

  cache.mCvec.setZero();

  for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
       it != cache.mBodyNodes.end(); ++it)
  {
    (*it)->updateCombinedVector();
  }

  for (std::vector<BodyNode*>::const_reverse_iterator it =
       cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateCoriolisForceVector(cache.mCvec);
  }

  cache.mDirty.mCoriolisForces = false;
}

//==============================================================================
void Skeleton::updateCoriolisForces() const
{
  size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<size_t>(mSkelCache.mCvec.size()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mCoriolisForces = false;
    return;
  }

  mSkelCache.mCvec.setZero();

  for(size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::VectorXd& treeCvec = getCoriolisForces(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    size_t nTreeDofs = treeDofs.size();
    for(size_t i=0; i<nTreeDofs; ++i)
    {
      size_t k = treeDofs[i]->getIndexInSkeleton();
      mSkelCache.mCvec[k] = treeCvec[i];
    }
  }

  mSkelCache.mDirty.mCoriolisForces = false;
}

//==============================================================================
void Skeleton::updateGravityForces(size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mG.size()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mGravityForces = false;
    return;
  }

  cache.mG.setZero();

  for (std::vector<BodyNode*>::const_reverse_iterator it =
       cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateGravityForceVector(cache.mG, mSkeletonP.mGravity);
  }

  cache.mDirty.mGravityForces = false;
}

//==============================================================================
void Skeleton::updateGravityForces() const
{
  size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<size_t>(mSkelCache.mG.size()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mGravityForces = false;
    return;
  }

  mSkelCache.mG.setZero();

  for(size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::VectorXd& treeG = getGravityForces(tree);
    std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    size_t nTreeDofs = treeDofs.size();
    for(size_t i=0; i<nTreeDofs; ++i)
    {
      size_t k = treeDofs[i]->getIndexInSkeleton();
      mSkelCache.mG[k] = treeG[i];
    }
  }

  mSkelCache.mDirty.mGravityForces = false;
}

//==============================================================================
void Skeleton::updateCoriolisAndGravityForces(size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mCg.size()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mCoriolisAndGravityForces = false;
    return;
  }

  cache.mCg.setZero();

  for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
       it != cache.mBodyNodes.end(); ++it)
  {
    (*it)->updateCombinedVector();
  }

  for (std::vector<BodyNode*>::const_reverse_iterator it =
       cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateCombinedVector(cache.mCg, mSkeletonP.mGravity);
  }

  cache.mDirty.mCoriolisAndGravityForces = false;
}

//==============================================================================
void Skeleton::updateCoriolisAndGravityForces() const
{
  size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<size_t>(mSkelCache.mCg.size()) == dof);
  if (dof == 0)
  {
    mSkelCache.mDirty.mCoriolisAndGravityForces = false;
    return;
  }

  mSkelCache.mCg.setZero();

  for(size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::VectorXd& treeCg = getCoriolisAndGravityForces(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    size_t nTreeDofs = treeDofs.size();
    for(size_t i=0; i<nTreeDofs; ++i)
    {
      size_t k = treeDofs[i]->getIndexInSkeleton();
      mSkelCache.mCg[k] = treeCg[i];
    }
  }

  mSkelCache.mDirty.mCoriolisAndGravityForces = false;
}

//==============================================================================
void Skeleton::updateExternalForces(size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mFext.size()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mExternalForces = false;
    return;
  }

  // Clear external force.
  cache.mFext.setZero();

  for (std::vector<BodyNode*>::const_reverse_iterator itr =
       cache.mBodyNodes.rbegin(); itr != cache.mBodyNodes.rend(); ++itr)
  {
    (*itr)->aggregateExternalForces(cache.mFext);
  }

  // TODO(JS): Not implemented yet
//  for (std::vector<SoftBodyNode*>::iterator it = mSoftBodyNodes.begin();
//       it != mSoftBodyNodes.end(); ++it)
//  {
//    double kv = (*it)->getVertexSpringStiffness();
//    double ke = (*it)->getEdgeSpringStiffness();

//    for (int i = 0; i < (*it)->getNumPointMasses(); ++i)
//    {
//      PointMass* pm = (*it)->getPointMass(i);
//      int nN = pm->getNumConnectedPointMasses();

//      // Vertex restoring force
//      Eigen::Vector3d Fext = -(kv + nN * ke) * pm->getPositions()
//                             - (mTimeStep * (kv + nN*ke)) * pm->getVelocities();

//      // Edge restoring force
//      for (int j = 0; j < nN; ++j)
//      {
//        Fext += ke * (pm->getConnectedPointMass(j)->getPositions()
//                      + mTimeStep
//                        * pm->getConnectedPointMass(j)->getVelocities());
//      }

//      // Assign
//      int iStart = pm->getIndexInSkeleton(0);
//      mFext.segment<3>(iStart) = Fext;
//    }
//  }

  cache.mDirty.mExternalForces = false;
}

//==============================================================================
void Skeleton::updateExternalForces() const
{
  size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<size_t>(mSkelCache.mFext.size()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mExternalForces = false;
    return;
  }

  mSkelCache.mFext.setZero();

  for(size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::VectorXd& treeFext = getExternalForces(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    size_t nTreeDofs = treeDofs.size();
    for(size_t i=0; i<nTreeDofs; ++i)
    {
      size_t k = treeDofs[i]->getIndexInSkeleton();
      mSkelCache.mFext[k] = treeFext[i];
    }
  }

  mSkelCache.mDirty.mExternalForces = false;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::computeConstraintForces(DataCache& cache) const
{
  const size_t dof = cache.mDofs.size();
  assert(static_cast<size_t>(cache.mFc.size()) == dof);

  // Body constraint impulses
  for (std::vector<BodyNode*>::reverse_iterator it =
       cache.mBodyNodes.rbegin();
       it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateSpatialToGeneralized(
          cache.mFc, (*it)->getConstraintImpulse());
  }

  // Joint constraint impulses
  for (size_t i = 0; i < dof; ++i)
    cache.mFc[i] += cache.mDofs[i]->getConstraintImpulse();

  // Get force by dividing the impulse by the time step
  cache.mFc = cache.mFc / mSkeletonP.mTimeStep;

  return cache.mFc;
}

//==============================================================================
static void computeSupportPolygon(
    const Skeleton* skel, math::SupportPolygon& polygon,
    math::SupportGeometry& geometry,  std::vector<size_t>& ee_indices,
    Eigen::Vector3d& axis1, Eigen::Vector3d& axis2, Eigen::Vector2d& centroid,
    size_t treeIndex)
{
  polygon.clear();
  geometry.clear();
  ee_indices.clear();

  const Eigen::Vector3d& up = -skel->getGravity();
  if(up.norm() == 0.0)
  {
    dtwarn << "[computeSupportPolygon] Requesting support polygon of a "
           << "Skeleton with no gravity. The result will only be an empty "
           << "set!\n";
    axis1.setZero();
    axis2.setZero();
    centroid = Eigen::Vector2d::Constant(std::nan(""));
    return;
  }

  std::vector<size_t> originalEE_map;
  originalEE_map.reserve(skel->getNumEndEffectors());
  for(size_t i=0; i < skel->getNumEndEffectors(); ++i)
  {
    const EndEffector* ee = skel->getEndEffector(i);
    if(ee->getSupport() && ee->getSupport()->isActive()
       && (INVALID_INDEX == treeIndex || ee->getTreeIndex() == treeIndex))
    {
      const math::SupportGeometry& eeGeom = ee->getSupport()->getGeometry();
      for(const Eigen::Vector3d& v : eeGeom)
      {
        geometry.push_back(ee->getWorldTransform()*v);
        originalEE_map.push_back(ee->getIndexInSkeleton());
      }
    }
  }

  axis1 = (up-Eigen::Vector3d::UnitX()).norm() > 1e-6 ?
        Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();

  axis1 = axis1 - up.dot(axis1)*up/up.dot(up);
  axis1.normalize();

  axis2 = up.normalized().cross(axis1);

  std::vector<size_t> vertex_indices;
  polygon = math::computeSupportPolgyon(vertex_indices, geometry, axis1, axis2);

  ee_indices.reserve(vertex_indices.size());
  for(size_t i=0; i < vertex_indices.size(); ++i)
    ee_indices[i] = originalEE_map[vertex_indices[i]];

  if(polygon.size() > 0)
    centroid = math::computeCentroidOfHull(polygon);
  else
    centroid = Eigen::Vector2d::Constant(std::nan(""));
}

//==============================================================================
const math::SupportPolygon& Skeleton::getSupportPolygon() const
{
  math::SupportPolygon& polygon = mSkelCache.mSupportPolygon;

  if(!mSkelCache.mDirty.mSupport)
    return polygon;

  computeSupportPolygon(this, polygon, mSkelCache.mSupportGeometry,
                        mSkelCache.mSupportIndices,
                        mSkelCache.mSupportAxes.first,
                        mSkelCache.mSupportAxes.second,
                        mSkelCache.mSupportCentroid, INVALID_INDEX);

  mSkelCache.mDirty.mSupport = false;
  ++mSkelCache.mDirty.mSupportVersion;
  return polygon;
}

//==============================================================================
const math::SupportPolygon& Skeleton::getSupportPolygon(size_t _treeIdx) const
{
  math::SupportPolygon& polygon = mTreeCache[_treeIdx].mSupportPolygon;

  if(!mTreeCache[_treeIdx].mDirty.mSupport)
    return polygon;

  computeSupportPolygon(this, polygon, mTreeCache[_treeIdx].mSupportGeometry,
                        mTreeCache[_treeIdx].mSupportIndices,
                        mTreeCache[_treeIdx].mSupportAxes.first,
                        mTreeCache[_treeIdx].mSupportAxes.second,
                        mTreeCache[_treeIdx].mSupportCentroid, _treeIdx);

  mTreeCache[_treeIdx].mDirty.mSupport = false;
  ++mTreeCache[_treeIdx].mDirty.mSupportVersion;
  return polygon;
}

//==============================================================================
const std::vector<size_t>& Skeleton::getSupportIndices() const
{
  getSupportPolygon();
  return mSkelCache.mSupportIndices;
}

//==============================================================================
const std::vector<size_t>& Skeleton::getSupportIndices(size_t _treeIdx) const
{
  getSupportPolygon(_treeIdx);
  return mTreeCache[_treeIdx].mSupportIndices;
}

//==============================================================================
const std::pair<Eigen::Vector3d, Eigen::Vector3d>&
Skeleton::getSupportAxes() const
{
  getSupportPolygon();
  return mSkelCache.mSupportAxes;
}

//==============================================================================
const std::pair<Eigen::Vector3d, Eigen::Vector3d>&
Skeleton::getSupportAxes(size_t _treeIdx) const
{
  getSupportPolygon(_treeIdx);
  return mTreeCache[_treeIdx].mSupportAxes;
}

//==============================================================================
const Eigen::Vector2d& Skeleton::getSupportCentroid() const
{
  getSupportPolygon();
  return mSkelCache.mSupportCentroid;
}

//==============================================================================
const Eigen::Vector2d& Skeleton::getSupportCentroid(size_t _treeIdx) const
{
  getSupportPolygon(_treeIdx);
  return mTreeCache[_treeIdx].mSupportCentroid;
}

//==============================================================================
size_t Skeleton::getSupportVersion() const
{
  if(mSkelCache.mDirty.mSupport)
    return mSkelCache.mDirty.mSupportVersion + 1;

  return mSkelCache.mDirty.mSupportVersion;
}

//==============================================================================
size_t Skeleton::getSupportVersion(size_t _treeIdx) const
{
  if(mTreeCache[_treeIdx].mDirty.mSupport)
    return mTreeCache[_treeIdx].mDirty.mSupportVersion + 1;

  return mTreeCache[_treeIdx].mDirty.mSupportVersion;
}

//==============================================================================
void Skeleton::computeForwardKinematics(bool _updateTransforms,
                                        bool _updateVels,
                                        bool _updateAccs)
{
  if (_updateTransforms)
  {
    for (std::vector<BodyNode*>::iterator it = mSkelCache.mBodyNodes.begin();
         it != mSkelCache.mBodyNodes.end(); ++it)
    {
      (*it)->updateTransform();
    }
  }

  if (_updateVels)
  {
    for (std::vector<BodyNode*>::iterator it = mSkelCache.mBodyNodes.begin();
         it != mSkelCache.mBodyNodes.end(); ++it)
    {
      (*it)->updateVelocity();
      (*it)->updatePartialAcceleration();
    }
  }

  if (_updateAccs)
  {
    for (std::vector<BodyNode*>::iterator it = mSkelCache.mBodyNodes.begin();
         it != mSkelCache.mBodyNodes.end(); ++it)
    {
      (*it)->updateAccelerationID();
    }
  }
}

//==============================================================================
void Skeleton::computeForwardDynamics()
{
  // Note: Articulated Inertias will be updated automatically when
  // getArtInertiaImplicit() is called in BodyNode::updateBiasForce()

  for (auto it = mSkelCache.mBodyNodes.rbegin();
       it != mSkelCache.mBodyNodes.rend(); ++it)
    (*it)->updateBiasForce(mSkeletonP.mGravity, mSkeletonP.mTimeStep);

  // Forward recursion
  for (auto& bodyNode : mSkelCache.mBodyNodes)
  {
    bodyNode->updateAccelerationFD();
    bodyNode->updateTransmittedForceFD();
    bodyNode->updateJointForceFD(mSkeletonP.mTimeStep, true, true);
  }
}

//==============================================================================
void Skeleton::computeInverseDynamics(bool _withExternalForces,
                                      bool _withDampingForces,
                                      bool _withSpringForces)
{
  // Skip immobile or 0-dof skeleton
  if (getNumDofs() == 0)
    return;

  // Backward recursion
  for (auto it = mSkelCache.mBodyNodes.rbegin();
       it != mSkelCache.mBodyNodes.rend(); ++it)
  {
    (*it)->updateTransmittedForceID(mSkeletonP.mGravity, _withExternalForces);
    (*it)->updateJointForceID(mSkeletonP.mTimeStep,
                              _withDampingForces,
                              _withSpringForces);
  }
}

//==============================================================================
void Skeleton::clearExternalForces()
{
  for (auto& bodyNode : mSkelCache.mBodyNodes)
    bodyNode->clearExternalForces();
}

//==============================================================================
void Skeleton::clearInternalForces()
{
  for (auto& bodyNode : mSkelCache.mBodyNodes)
    bodyNode->clearInternalForces();
}

//==============================================================================
void Skeleton::notifyArticulatedInertiaUpdate(size_t _treeIdx)
{
  SET_FLAG(_treeIdx, mArticulatedInertia);
  SET_FLAG(_treeIdx, mMassMatrix);
  SET_FLAG(_treeIdx, mAugMassMatrix);
  SET_FLAG(_treeIdx, mInvMassMatrix);
  SET_FLAG(_treeIdx, mInvAugMassMatrix);
  SET_FLAG(_treeIdx, mCoriolisForces);
  SET_FLAG(_treeIdx, mGravityForces);
  SET_FLAG(_treeIdx, mCoriolisAndGravityForces);
}

//==============================================================================
void Skeleton::notifySupportUpdate(size_t _treeIdx)
{
  SET_FLAG(_treeIdx, mSupport);
}

//==============================================================================
void Skeleton::clearConstraintImpulses()
{
  for (auto& bodyNode : mSkelCache.mBodyNodes)
    bodyNode->clearConstraintImpulse();
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode)
{
  if(nullptr == _bodyNode)
  {
    dterr << "[Skeleton::updateBiasImpulse] Passed in a nullptr!\n";
    assert(false);
    return;
  }

  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(_bodyNode->getSkeleton().get() == this);

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    assert(mSkelCache.mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Prepare cache data
  BodyNode* it = _bodyNode;
  while (it != nullptr)
  {
    it->updateBiasImpulse();
    it = it->getParentBodyNode();
  }
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode,
                                 const Eigen::Vector6d& _imp)
{
  if(nullptr == _bodyNode)
  {
    dterr << "[Skeleton::updateBiasImpulse] Passed in a nullptr!\n";
    assert(false);
    return;
  }

  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(_bodyNode->getSkeleton().get() == this);

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    assert(mSkelCache.mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Set impulse of _bodyNode
  _bodyNode->mConstraintImpulse = _imp;

  // Prepare cache data
  BodyNode* it = _bodyNode;
  while (it != nullptr)
  {
    it->updateBiasImpulse();
    it = it->getParentBodyNode();
  }

  _bodyNode->mConstraintImpulse.setZero();
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode1,
                                 const Eigen::Vector6d& _imp1,
                                 BodyNode* _bodyNode2,
                                 const Eigen::Vector6d& _imp2)
{
  // Assertions
  if(nullptr == _bodyNode1)
  {
    dterr << "[Skeleton::updateBiasImpulse] Passed in nullptr for BodyNode1!\n";
    assert(false);
    return;
  }

  if(nullptr == _bodyNode2)
  {
    dterr << "[Skeleton::updateBiasImpulse] Passed in nullptr for BodyNode2!\n";
    assert(false);
    return;
  }

  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(_bodyNode1->getSkeleton().get() == this);
  assert(_bodyNode2->getSkeleton().get() == this);

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    assert(mSkelCache.mBodyNodes[i]->mConstraintImpulse ==
           Eigen::Vector6d::Zero());
#endif

  // Set impulse to _bodyNode
  _bodyNode1->mConstraintImpulse = _imp1;
  _bodyNode2->mConstraintImpulse = _imp2;

  // Find which body is placed later in the list of body nodes in this skeleton
  size_t index1 = _bodyNode1->getIndexInSkeleton();
  size_t index2 = _bodyNode2->getIndexInSkeleton();

  size_t index = std::max(index1, index2);

  // Prepare cache data
  for (int i = index; 0 <= i; --i)
    mSkelCache.mBodyNodes[i]->updateBiasImpulse();

  _bodyNode1->mConstraintImpulse.setZero();
  _bodyNode2->mConstraintImpulse.setZero();
}

//==============================================================================
void Skeleton::updateBiasImpulse(SoftBodyNode* _softBodyNode,
                                 PointMass* _pointMass,
                                 const Eigen::Vector3d& _imp)
{
  // Assertions
  assert(_softBodyNode != nullptr);
  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(std::find(mSoftBodyNodes.begin(), mSoftBodyNodes.end(), _softBodyNode)
         != mSoftBodyNodes.end());

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    assert(mSkelCache.mBodyNodes[i]->mConstraintImpulse ==
           Eigen::Vector6d::Zero());
#endif

  // Set impulse to _bodyNode
  Eigen::Vector3d oldConstraintImpulse =_pointMass->getConstraintImpulses();
  _pointMass->setConstraintImpulse(_imp, true);

  // Prepare cache data
  BodyNode* it = _softBodyNode;
  while (it != nullptr)
  {
    it->updateBiasImpulse();
    it = it->getParentBodyNode();
  }

  // TODO(JS): Do we need to backup and restore the original value?
  _pointMass->setConstraintImpulse(oldConstraintImpulse);
}

//==============================================================================
void Skeleton::updateVelocityChange()
{
  for (auto& bodyNode : mSkelCache.mBodyNodes)
    bodyNode->updateVelocityChangeFD();
}

//==============================================================================
void Skeleton::setImpulseApplied(bool _val)
{
  mIsImpulseApplied = _val;
}

//==============================================================================
bool Skeleton::isImpulseApplied() const
{
  return mIsImpulseApplied;
}

//==============================================================================
void Skeleton::computeImpulseForwardDynamics()
{
  // Skip immobile or 0-dof skeleton
  if (!isMobile() || getNumDofs() == 0)
    return;

  // Note: we do not need to update articulated inertias here, because they will
  // be updated when BodyNode::updateBiasImpulse() calls
  // BodyNode::getArticulatedInertia()

  // Backward recursion
  for (auto it = mSkelCache.mBodyNodes.rbegin();
       it != mSkelCache.mBodyNodes.rend(); ++it)
    (*it)->updateBiasImpulse();

  // Forward recursion
  for (auto& bodyNode : mSkelCache.mBodyNodes)
  {
    bodyNode->updateVelocityChangeFD();
    bodyNode->updateTransmittedImpulse();
    bodyNode->updateJointImpulseFD();
    bodyNode->updateConstrainedTerms(mSkeletonP.mTimeStep);
  }
}

//==============================================================================
double Skeleton::getKineticEnergy() const
{
  double KE = 0.0;

  for (std::vector<BodyNode*>::const_iterator it = mSkelCache.mBodyNodes.begin();
       it != mSkelCache.mBodyNodes.end(); ++it)
  {
    KE += (*it)->getKineticEnergy();
  }

  assert(KE >= 0.0 && "Kinetic energy should be positive value.");
  return KE;
}

//==============================================================================
double Skeleton::getPotentialEnergy() const
{
  double PE = 0.0;

  for (std::vector<BodyNode*>::const_iterator it = mSkelCache.mBodyNodes.begin();
       it != mSkelCache.mBodyNodes.end(); ++it)
  {
    PE += (*it)->getPotentialEnergy(mSkeletonP.mGravity);
    PE += (*it)->getParentJoint()->getPotentialEnergy();
  }

  return PE;
}

//==============================================================================
Eigen::Vector3d Skeleton::getCOM(const Frame* _withRespectTo) const
{
  Eigen::Vector3d com = Eigen::Vector3d::Zero();

  const size_t numBodies = getNumBodyNodes();
  for (size_t i = 0; i < numBodies; ++i)
  {
    const BodyNode* bodyNode = getBodyNode(i);
    com += bodyNode->getMass() * bodyNode->getCOM(_withRespectTo);
  }

  assert(mTotalMass != 0.0);
  return com / mTotalMass;
}

//==============================================================================
// Templated function for computing different kinds of COM properties, like
// velocities and accelerations
template <
    typename PropertyType,
    PropertyType (BodyNode::*getPropertyFn)(const Frame*, const Frame*) const>
PropertyType getCOMPropertyTemplate(const Skeleton* _skel,
                                    const Frame* _relativeTo,
                                    const Frame* _inCoordinatesOf)
{
  PropertyType result(PropertyType::Zero());

  const size_t numBodies = _skel->getNumBodyNodes();
  for (size_t i = 0; i < numBodies; ++i)
  {
    const BodyNode* bodyNode = _skel->getBodyNode(i);
    result += bodyNode->getMass()
              * (bodyNode->*getPropertyFn)(_relativeTo, _inCoordinatesOf);
  }

  assert(_skel->getMass() != 0.0);
  return result / _skel->getMass();
}

//==============================================================================
Eigen::Vector6d Skeleton::getCOMSpatialVelocity(const Frame* _relativeTo,
    const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector6d,
      &BodyNode::getCOMSpatialVelocity>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Skeleton::getCOMLinearVelocity(const Frame* _relativeTo,
    const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector3d,
      &BodyNode::getCOMLinearVelocity>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d Skeleton::getCOMSpatialAcceleration(const Frame* _relativeTo,
    const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector6d,
      &BodyNode::getCOMSpatialAcceleration>(
        this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Skeleton::getCOMLinearAcceleration(const Frame* _relativeTo,
    const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector3d,
      &BodyNode::getCOMLinearAcceleration>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
// Templated function for computing different kinds of COM Jacobians and their
// derivatives
template <
    typename JacType, // JacType is the type of Jacobian we're computing
    JacType (TemplatedJacobianNode<BodyNode>::*getJacFn)(
        const Eigen::Vector3d&, const Frame*) const>
JacType getCOMJacobianTemplate(const Skeleton* _skel,
                               const Frame* _inCoordinatesOf)
{
  // Initialize the Jacobian to zero
  JacType J = JacType::Zero(JacType::RowsAtCompileTime, _skel->getNumDofs());

  // Iterate through each of the Skeleton's BodyNodes
  const size_t numBodies = _skel->getNumBodyNodes();
  for (size_t i = 0; i < numBodies; ++i)
  {
    const BodyNode* bn = _skel->getBodyNode(i);

    // (bn->*getJacFn) is a function pointer to the function that gives us the
    // kind of Jacobian we want from the BodyNodes. Calling it will give us the
    // relevant Jacobian for this BodyNode
    JacType bnJ = bn->getMass() * (bn->*getJacFn)(bn->getLocalCOM(),
                                                  _inCoordinatesOf);

    // For each column in the Jacobian of this BodyNode, we add it to the
    // appropriate column of the overall BodyNode
    for (size_t j=0, end=bn->getNumDependentGenCoords(); j < end; ++j)
    {
      size_t idx = bn->getDependentGenCoordIndex(j);
      J.col(idx) += bnJ.col(j);
    }
  }

  assert(_skel->getMass() != 0.0);
  return J / _skel->getMass();
}

//==============================================================================
math::Jacobian Skeleton::getCOMJacobian(const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::Jacobian,
          &TemplatedJacobianNode<BodyNode>::getJacobian>(
        this, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getCOMLinearJacobian(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::LinearJacobian,
           &TemplatedJacobianNode<BodyNode>::getLinearJacobian>(
              this, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getCOMJacobianSpatialDeriv(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::Jacobian,
      &TemplatedJacobianNode<BodyNode>::getJacobianSpatialDeriv>(
              this, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getCOMLinearJacobianDeriv(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::LinearJacobian,
      &TemplatedJacobianNode<BodyNode>::getLinearJacobianDeriv>(
              this, _inCoordinatesOf);
}

//==============================================================================
Skeleton::DirtyFlags::DirtyFlags()
  : mArticulatedInertia(true),
    mMassMatrix(true),
    mAugMassMatrix(true),
    mInvMassMatrix(true),
    mInvAugMassMatrix(true),
    mGravityForces(true),
    mCoriolisForces(true),
    mCoriolisAndGravityForces(true),
    mExternalForces(true),
    mDampingForces(true),
    mSupport(true),
    mSupportVersion(0)
{
  // Do nothing
}

}  // namespace dynamics
}  // namespace dart
