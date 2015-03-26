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
#include "dart/dynamics/Marker.h"
#include "dart/dynamics/PointMass.h"
#include "dart/dynamics/SoftBodyNode.h"

namespace dart {
namespace dynamics {

//==============================================================================
Skeleton::Skeleton(const std::string& _name)
  : mName(_name),
    mNumDofs(0),
    mEnabledSelfCollisionCheck(false),
    mEnabledAdjacentBodyCheck(false),
    mNameMgrForBodyNodes("BodyNode"),
    mNameMgrForJoints("Joint"),
    mNameMgrForSoftBodyNodes("BodyNode"),
    mIsMobile(true),
    mTimeStep(0.001),
    mGravity(Eigen::Vector3d(0.0, 0.0, -9.81)),
    mTotalMass(0.0),
    mIsArticulatedInertiaDirty(true),
    mIsMassMatrixDirty(true),
    mIsAugMassMatrixDirty(true),
    mIsInvMassMatrixDirty(true),
    mIsInvAugMassMatrixDirty(true),
    mIsCoriolisForcesDirty(true),
    mIsGravityForcesDirty(true),
    mIsCoriolisAndGravityForcesDirty(true),
    mIsExternalForcesDirty(true),
    mIsDampingForcesDirty(true),
    mIsImpulseApplied(false),
    mUnionRootSkeleton(this),
    mUnionSize(1)
{
}

//==============================================================================
Skeleton::~Skeleton()
{
  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    delete (*it);
  }
}

//==============================================================================
void Skeleton::setName(const std::string& _name)
{
  mName = _name;
}

//==============================================================================
const std::string& Skeleton::getName() const
{
  return mName;
}

//==============================================================================
const std::string& Skeleton::addEntryToBodyNodeNameMgr(BodyNode* _newNode)
{
  _newNode->mEntityP.mName = mNameMgrForBodyNodes.issueNewNameAndAdd(
        _newNode->getName(),_newNode);
  return _newNode->mEntityP.mName;
}

//==============================================================================
const std::string& Skeleton::addEntryToJointNameMgr(Joint* _newJoint)
{
  _newJoint->mJointP.mName = mNameMgrForJoints.issueNewNameAndAdd(
        _newJoint->getName(), _newJoint);
  _newJoint->updateDegreeOfFreedomNames();
  return _newJoint->mJointP.mName;
}

//==============================================================================
const std::string& Skeleton::addEntryToDofNameMgr(DegreeOfFreedom* _newDof)
{
  _newDof->mName = mNameMgrForDofs.issueNewNameAndAdd(_newDof->getName(),
                                                      _newDof);
  return _newDof->mName;
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
  _newMarker->mName = mNameMgrForMarkers.issueNewNameAndAdd(
      _newMarker->getName(), _newMarker);
  return _newMarker->mName;
}

//==============================================================================
void Skeleton::enableSelfCollision(bool _enableAdjecentBodyCheck)
{
  mEnabledSelfCollisionCheck = true;
  mEnabledAdjacentBodyCheck = _enableAdjecentBodyCheck;
}

//==============================================================================
void Skeleton::disableSelfCollision()
{
  mEnabledSelfCollisionCheck = false;
  mEnabledAdjacentBodyCheck = false;
}

//==============================================================================
bool Skeleton::isEnabledSelfCollisionCheck() const
{
  return mEnabledSelfCollisionCheck;
}

//==============================================================================
bool Skeleton::isEnabledAdjacentBodyCheck() const
{
  return mEnabledAdjacentBodyCheck;
}

//==============================================================================
void Skeleton::setMobile(bool _isMobile)
{
  mIsMobile = _isMobile;
}

//==============================================================================
bool Skeleton::isMobile() const
{
  return mIsMobile;
}

//==============================================================================
void Skeleton::setTimeStep(double _timeStep)
{
  assert(_timeStep > 0.0);
  mTimeStep = _timeStep;
  notifyArticulatedInertiaUpdate();
}

//==============================================================================
double Skeleton::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void Skeleton::setGravity(const Eigen::Vector3d& _gravity)
{
  mGravity = _gravity;
  mIsGravityForcesDirty = true;
  mIsCoriolisAndGravityForcesDirty = true;
}

//==============================================================================
const Eigen::Vector3d& Skeleton::getGravity() const
{
  return mGravity;
}

//==============================================================================
double Skeleton::getMass() const
{
  return mTotalMass;
}

//==============================================================================
void Skeleton::addBodyNode(BodyNode* _body)
{
  assert(_body && _body->getParentJoint());

  mBodyNodes.push_back(_body);
  addEntryToBodyNodeNameMgr(_body);
  addMarkersOfBodyNode(_body);
  _body->mSkeleton = this;
  registerJoint(_body->getParentJoint());

  SoftBodyNode* softBodyNode = dynamic_cast<SoftBodyNode*>(_body);
  if (softBodyNode)
  {
    mSoftBodyNodes.push_back(softBodyNode);
    addEntryToSoftBodyNodeNameMgr(softBodyNode);
  }
}

//==============================================================================
size_t Skeleton::getNumBodyNodes() const
{
  return mBodyNodes.size();
}

//==============================================================================
size_t Skeleton::getNumRigidBodyNodes() const
{
  return mBodyNodes.size() - mSoftBodyNodes.size();
}

//==============================================================================
size_t Skeleton::getNumSoftBodyNodes() const
{
  return mSoftBodyNodes.size();
}

//==============================================================================
BodyNode* Skeleton::getRootBodyNode()
{
  if (mBodyNodes.size()==0)
    return NULL;

  // We assume that the first element of body nodes is root.
  return mBodyNodes[0];
}

//==============================================================================
const BodyNode* Skeleton::getRootBodyNode() const
{
  return const_cast<Skeleton*>(this)->getRootBodyNode();
}

//==============================================================================
template<typename T>
static T getVectorObjectIfAvailable(size_t _idx, const std::vector<T>& _vec)
{
  // TODO: Should we have an out-of-bounds assertion or throw here?
  if (_idx < _vec.size())
    return _vec[_idx];

  return NULL;
}

//==============================================================================
BodyNode* Skeleton::getBodyNode(size_t _idx)
{
  return getVectorObjectIfAvailable<BodyNode*>(_idx, mBodyNodes);
}

//==============================================================================
const BodyNode* Skeleton::getBodyNode(size_t _idx) const
{
  return getVectorObjectIfAvailable<BodyNode*>(_idx, mBodyNodes);
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

const SoftBodyNode* Skeleton::getSoftBodyNode(const std::string& _name) const
{
  return mNameMgrForSoftBodyNodes.getObject(_name);
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
  BodyNode* bn = getVectorObjectIfAvailable<BodyNode*>(_idx, mBodyNodes);
  if (bn)
    return bn->getParentJoint();

  return NULL;
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
DegreeOfFreedom* Skeleton::getDof(size_t _idx)
{
  assert(_idx < getNumDofs());

  if (_idx >= getNumDofs())
    return NULL;

  return mDofs[_idx];
}

//==============================================================================
const DegreeOfFreedom* Skeleton::getDof(size_t _idx) const
{
  assert(_idx < getNumDofs());

  if (_idx >= getNumDofs())
    return NULL;

  return mDofs[_idx];
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
void Skeleton::init(double _timeStep, const Eigen::Vector3d& _gravity)
{
  // Set timestep and gravity
  setTimeStep(_timeStep);
  setGravity(_gravity);

  // Get root bodynodes that don't have parent bodynode
  std::vector<BodyNode*> rootBodyNodes;
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
  {
    if (mBodyNodes[i]->getParentBodyNode() == NULL)
      rootBodyNodes.push_back(mBodyNodes[i]);
  }

  // Rearrange the list of body nodes with BFS (Breadth First Search)
  std::queue<BodyNode*> queue;
  mBodyNodes.clear();
  for (size_t i = 0; i < rootBodyNodes.size(); ++i)
  {
    queue.push(rootBodyNodes[i]);

    while (!queue.empty())
    {
      BodyNode* itBodyNode = queue.front();
      queue.pop();
      mBodyNodes.push_back(itBodyNode);
      for (size_t j = 0; j < itBodyNode->getNumChildBodyNodes(); ++j)
        queue.push(itBodyNode->getChildBodyNode(j));
    }
  }

  // Initialize body nodes and generalized coordinates
  mDofs.clear();
  mNumDofs = 0;
  const size_t numBodyNodes = getNumBodyNodes();
  for (size_t i = 0; i < numBodyNodes; ++i)
  {
    BodyNode* bodyNode = mBodyNodes[i];
    Joint*    joint    = bodyNode->getParentJoint();

    const size_t numDofsOfJoint = joint->getNumDofs();
    for (size_t j = 0; j < numDofsOfJoint; ++j)
    {
      mDofs.push_back(joint->getDof(j));
      joint->setIndexInSkeleton(j, mNumDofs + j);
    }

    bodyNode->init(this);
    mNumDofs += joint->getNumDofs();
  }

  // Compute transformations, velocities, and partial accelerations
//  computeForwardDynamicsRecursionPartA(); // No longer needed with auto-update

  // Set dimension of dynamics quantities
  size_t dof = getNumDofs();
  mM    = Eigen::MatrixXd::Zero(dof, dof);
  mAugM = Eigen::MatrixXd::Zero(dof, dof);
  mInvM = Eigen::MatrixXd::Zero(dof, dof);
  mInvAugM = Eigen::MatrixXd::Zero(dof, dof);
  mCvec = Eigen::VectorXd::Zero(dof);
  mG    = Eigen::VectorXd::Zero(dof);
  mCg   = Eigen::VectorXd::Zero(dof);
  mFext = Eigen::VectorXd::Zero(dof);
  mFc   = Eigen::VectorXd::Zero(dof);
  mFd   = Eigen::VectorXd::Zero(dof);

  // Clear external/internal force
  clearExternalForces();
  resetForces();

  // Calculate mass
  mTotalMass = 0.0;
  for (size_t i = 0; i < getNumBodyNodes(); i++)
    mTotalMass += getBodyNode(i)->getMass();
}

//==============================================================================
size_t Skeleton::getDof() const
{
  return getNumDofs();
}

//==============================================================================
size_t Skeleton::getNumDofs() const
{
  return mNumDofs;
}

//==============================================================================
GenCoordInfo Skeleton::getGenCoordInfo(size_t _index) const
{
  assert(_index < getNumDofs());

  return mGenCoordInfos[_index];
}

//==============================================================================
void Skeleton::setCommand(size_t _index, double _command)
{
  assert(_index < getNumDofs());

  Joint* joint = mDofs[_index]->getJoint();
  const size_t localIndex = mDofs[_index]->getIndexInJoint();

  return joint->setCommand(localIndex, _command);
}

//==============================================================================
double Skeleton::getCommand(size_t _index) const
{
  assert(_index <getNumDofs());

  const Joint* joint = mDofs[_index]->getJoint();
  const size_t localIndex = mDofs[_index]->getIndexInJoint();

  return joint->getCommand(localIndex);
}

//==============================================================================
void Skeleton::setCommands(const Eigen::VectorXd& _commands)
{
  for (const auto& bodyNode : mBodyNodes)
  {
    Joint*       joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      joint->setCommands(_commands.segment(index, dof));
    }
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getCommands() const
{
  Eigen::VectorXd commands(getNumDofs());

  for (const auto& bodyNode : mBodyNodes)
  {
    const Joint* joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      commands.segment(index, dof) = joint->getCommands();
    }
  }

  return commands;
}

//==============================================================================
void Skeleton::resetCommands()
{
  for (auto& bodyNode : mBodyNodes)
    bodyNode->getParentJoint()->resetCommands();
}

//==============================================================================
void Skeleton::setPosition(size_t _index, double _position)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setPosition(_position);
}

//==============================================================================
double Skeleton::getPosition(size_t _index) const
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getPosition();
}

//==============================================================================
void Skeleton::setPositions(const Eigen::VectorXd& _positions)
{
  for (const auto& bodyNode : mBodyNodes)
  {
    Joint*       joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      joint->setPositions(_positions.segment(index, dof));
    }
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getPositions() const
{
  Eigen::VectorXd q(getNumDofs());

  for (const auto& bodyNode : mBodyNodes)
  {
    const Joint* joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      q.segment(index, dof) = joint->getPositions();
    }
  }

  return q;
}

//==============================================================================
Eigen::VectorXd Skeleton::getPositionSegment(
    const std::vector<size_t>& _id) const
{
  Eigen::VectorXd q(_id.size());

  for (size_t i = 0; i < _id.size(); ++i)
    q[i] = mDofs[_id[i]]->getPosition();

  return q;
}

//==============================================================================
void Skeleton::setPositionSegment(const std::vector<size_t>& _id,
                                  const Eigen::VectorXd& _positions)
{
  assert((int)_id.size() == _positions.size());
  if((int)_id.size() != _positions.size())
  {
    dterr << "[Skeleton::setPositionSegment] Mismatch between _id size ("
          << _id.size() << ") and _positions size (" << _positions.size()
          << "). Positions will NOT be set!\n";
    return;
  }

  for (size_t i = 0; i < _id.size(); ++i)
  {
    DegreeOfFreedom* dof = getDof(_id[i]);
    dof->setPosition(_positions[i]);
  }
}

//==============================================================================
void Skeleton::resetPositions()
{
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    mBodyNodes[i]->getParentJoint()->resetPositions();
}

//==============================================================================
void Skeleton::setPositionLowerLimit(size_t _index, double _position)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setPositionLowerLimit(_position);
}

//==============================================================================
double Skeleton::getPositionLowerLimit(size_t _index)
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getPositionLowerLimit();
}

//==============================================================================
void Skeleton::setPositionUpperLimit(size_t _index, double _position)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setPositionUpperLimit(_position);
}

//==============================================================================
double Skeleton::getPositionUpperLimit(size_t _index)
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getPositionUpperLimit();
}

//==============================================================================
void Skeleton::setVelocity(size_t _index, double _velocity)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setVelocity(_velocity);
}

//==============================================================================
double Skeleton::getVelocity(size_t _index) const
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getVelocity();
}

//==============================================================================
void Skeleton::setVelocities(const Eigen::VectorXd& _velocities)
{
  for (const auto& bodyNode : mBodyNodes)
  {
    Joint*       joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      joint->setVelocities(_velocities.segment(index, dof));
    }
  }
}

//==============================================================================
void Skeleton::setVelocitySegment(const std::vector<size_t>& _id,
                                  const Eigen::VectorXd& _velocities)
{
  assert((int)_id.size() == _velocities.size());
  if((int)_id.size() != _velocities.size())
  {
    dterr << "[Skeleton::setVelocitySegment] Mismatch between _id size ("
          << _id.size() << ") and _velocity size (" << _velocities.size()
          << "). Velocities will NOT be set!\n";
    return;
  }

  for (size_t i=0; i<_id.size(); ++i)
  {
    DegreeOfFreedom* dof = getDof(_id[i]);
    dof->setVelocity(_velocities[i]);
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getVelocities() const
{
  Eigen::VectorXd dq(getNumDofs());

  for (const auto& bodyNode : mBodyNodes)
  {
    const Joint* joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      dq.segment(index, dof) = joint->getVelocities();
    }
  }

  return dq;
}

//==============================================================================
Eigen::VectorXd Skeleton::getVelocitySegment(const std::vector<size_t>& _id) const
{
  Eigen::VectorXd dq(_id.size());

  for(size_t i=0; i<_id.size(); ++i)
  {
    const DegreeOfFreedom* dof = getDof(_id[i]);
    dq[i] = dof->getVelocity();
  }

  return dq;
}

//==============================================================================
void Skeleton::resetVelocities()
{
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    mBodyNodes[i]->getParentJoint()->resetVelocities();
}

//==============================================================================
void Skeleton::setVelocityLowerLimit(size_t _index, double _velocity)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setVelocityLowerLimit(_velocity);
}

//==============================================================================
double Skeleton::getVelocityLowerLimit(size_t _index)
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getVelocityLowerLimit();
}

//==============================================================================
void Skeleton::setVelocityUpperLimit(size_t _index, double _velocity)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setVelocityUpperLimit(_velocity);
}

//==============================================================================
double Skeleton::getVelocityUpperLimit(size_t _index)
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getVelocityUpperLimit();
}

//==============================================================================
void Skeleton::setAcceleration(size_t _index, double _acceleration)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setAcceleration(_acceleration);
}

//==============================================================================
double Skeleton::getAcceleration(size_t _index) const
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getAcceleration();
}

//==============================================================================
void Skeleton::setAccelerations(const Eigen::VectorXd& _accelerations)
{
  for (const auto& bodyNode : mBodyNodes)
  {
    Joint*       joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      joint->setAccelerations(_accelerations.segment(index, dof));
    }
  }
}

//==============================================================================
void Skeleton::setAccelerationSegment(const std::vector<size_t>& _id,
                                      const Eigen::VectorXd& _accelerations)
{
  assert((int)_id.size() == _accelerations.size());
  if((int)_id.size() != _accelerations.size())
  {
    dterr << "[Skeleton::setAccelerationSegment] Mismatch between _id size ("
          << _id.size() << ") and _acceleration size (" << _accelerations.size()
          << "). Accelerations will NOT be set!\n";
    return;
  }

  for (size_t i=0; i<_id.size(); ++i)
  {
    DegreeOfFreedom* dof = getDof(_id[i]);
    dof->setAcceleration(_accelerations[i]);
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getAccelerations() const
{
  Eigen::VectorXd ddq(getNumDofs());

  for (const auto& bodyNode : mBodyNodes)
  {
    const Joint* joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      ddq.segment(index, dof) = joint->getAccelerations();
    }
  }

  return ddq;
}

//==============================================================================
Eigen::VectorXd Skeleton::getAccelerationSegment(const std::vector<size_t>& _id) const
{
  Eigen::VectorXd ddq(_id.size());

  for (size_t i=0; i<_id.size(); ++i)
  {
    const DegreeOfFreedom* dof = getDof(_id[i]);
    ddq[i] = dof->getAcceleration();
  }

  return ddq;
}

//==============================================================================
void Skeleton::resetAccelerations()
{
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    mBodyNodes[i]->getParentJoint()->resetAccelerations();
}

//==============================================================================
void Skeleton::setForce(size_t _index, double _force)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setForce(_force);
}

//==============================================================================
double Skeleton::getForce(size_t _index)
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getForce();
}

//==============================================================================
void Skeleton::setForces(const Eigen::VectorXd& _forces)
{
  for (const auto& bodyNode : mBodyNodes)
  {
    Joint*       joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      joint->setForces(_forces.segment(index, dof));
    }
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getForces() const
{
  Eigen::VectorXd forces(getNumDofs());

  for (const auto& bodyNode : mBodyNodes)
  {
    const Joint* joint = bodyNode->getParentJoint();
    const size_t dof   = joint->getNumDofs();

    if (dof)
    {
      size_t index = joint->getDof(0)->getIndexInSkeleton();
      forces.segment(index, dof) = joint->getForces();
    }
  }

  return forces;
}

//==============================================================================
void Skeleton::resetForces()
{
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    mBodyNodes[i]->getParentJoint()->resetForces();

  // TODO(JS): Find better place
  for (size_t i = 0; i < mSoftBodyNodes.size(); ++i)
  {
    for (size_t j = 0; j < mSoftBodyNodes[i]->getNumPointMasses(); ++j)
      mSoftBodyNodes[i]->getPointMass(j)->resetForces();
  }
}

//==============================================================================
void Skeleton::setForceLowerLimit(size_t _index, double _force)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setForceLowerLimit(_force);
}

//==============================================================================
double Skeleton::getForceLowerLimit(size_t _index)
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getForceLowerLimit();
}

//==============================================================================
void Skeleton::setForceUpperLimit(size_t _index, double _force)
{
  assert(_index < getNumDofs());

  mDofs[_index]->setForceUpperLimit(_force);
}

//==============================================================================
double Skeleton::getForceUpperLimit(size_t _index)
{
  assert(_index <getNumDofs());

  return mDofs[_index]->getVelocityUpperLimit();
}

//==============================================================================
Eigen::VectorXd Skeleton::getVelocityChanges() const
{
  const size_t dof = getNumDofs();
  Eigen::VectorXd velChange(dof);

  size_t index = 0;
  for (size_t i = 0; i < dof; ++i)
  {
    Joint* joint      = mDofs[i]->getJoint();
    size_t localIndex = mDofs[i]->getIndexInJoint();

    velChange[index++] = joint->getVelocityChange(localIndex);
  }

  assert(index == dof);

  return velChange;
}

//==============================================================================
void Skeleton::setConstraintImpulses(const Eigen::VectorXd& _impulses)
{
  setJointConstraintImpulses(_impulses);
}

//==============================================================================
void Skeleton::setJointConstraintImpulses(const Eigen::VectorXd& _impulses)
{
  const size_t dof = getNumDofs();

  size_t index = 0;
  for (size_t i = 0; i < dof; ++i)
  {
    Joint* joint      = mDofs[i]->getJoint();
    size_t localIndex = mDofs[i]->getIndexInJoint();

    joint->setConstraintImpulse(localIndex, _impulses[index++]);
  }

  assert(index == dof);
}

//==============================================================================
Eigen::VectorXd Skeleton::getConstraintImpulses() const
{
  return getJointConstraintImpulses();
}

//==============================================================================
Eigen::VectorXd Skeleton::getJointConstraintImpulses() const
{
  const size_t dof = getNumDofs();
  Eigen::VectorXd impulse(dof);

  size_t index = 0;
  for (size_t i = 0; i < dof; ++i)
  {
    Joint* joint      = mDofs[i]->getJoint();
    size_t localIndex = mDofs[i]->getIndexInJoint();

    impulse[index++] = joint->getConstraintImpulse(localIndex);
  }

  assert(index == dof);

  return impulse;
}

//==============================================================================
void Skeleton::setState(const Eigen::VectorXd& _state)
{
  assert(_state.size() % 2 == 0);

  size_t index = 0;
  size_t dof = 0;
  size_t halfSize = _state.size() / 2;
  Joint* joint;

  for (size_t i = 0; i < mBodyNodes.size(); ++i)
  {
    joint = mBodyNodes[i]->getParentJoint();

    dof = joint->getNumDofs();

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
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    mBodyNodes[i]->getParentJoint()->integratePositions(_dt);

  for (size_t i = 0; i < mSoftBodyNodes.size(); ++i)
  {
    for (size_t j = 0; j < mSoftBodyNodes[i]->getNumPointMasses(); ++j)
      mSoftBodyNodes[i]->getPointMass(j)->integratePositions(_dt);
  }
}

//==============================================================================
void Skeleton::integrateVelocities(double _dt)
{
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    mBodyNodes[i]->getParentJoint()->integrateVelocities(_dt);

  for (size_t i = 0; i < mSoftBodyNodes.size(); ++i)
  {
    for (size_t j = 0; j < mSoftBodyNodes[i]->getNumPointMasses(); ++j)
      mSoftBodyNodes[i]->getPointMass(j)->integrateVelocities(_dt);
  }
}

//==============================================================================
void Skeleton::computeForwardKinematics(bool _updateTransforms,
                                        bool _updateVels,
                                        bool _updateAccs)
{
  if (_updateTransforms)
  {
    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it)
    {
      (*it)->updateTransform();
    }
  }

  if (_updateVels)
  {
    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it)
    {
      (*it)->updateVelocity();
      (*it)->updatePartialAcceleration();
    }
  }

  if (_updateAccs)
  {
    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it)
    {
      (*it)->updateAccelerationID();
    }
  }
}

//==============================================================================
bool isValidBodyNode(const Skeleton* _skeleton,
                     const BodyNode* _bodyNode,
                     const std::string& _jacobianType)
{
  if (nullptr == _bodyNode)
  {
    dtwarn << "[Skeleton::getJacobian] Invalid BodyNode pointer, 'nullptr'. "
           << "Returning zero Jacobian." << std::endl;
    return false;
  }

  // The given BodyNode should be in the Skeleton.
  if (_bodyNode->getSkeleton() != _skeleton)
  {
    dtwarn << "[Skeleton::getJacobian] Attempting to get a "
           << _jacobianType << " of a BodyNode '"
           << _bodyNode->getName() << "' that is not in this Skeleton '"
           << _skeleton->getName() << ". Returning zero Jacobian." << std::endl;
    return false;
  }

  return true;
}

//==============================================================================
template <typename JacobianType>
void assignJacobian(JacobianType& _J,
                    const BodyNode* _bodyNode,
                    const JacobianType& _JBodyNode)
{
  // Assign the BodyNode's Jacobian to the result Jacobian.
  size_t localIndex = 0;
  const auto& indices = _bodyNode->getDependentGenCoordIndices();
  for (const auto& index : indices)
  {
    // Each index should be less than the number of dofs of this Skeleton.
    assert(index < _bodyNode->getSkeleton()->getNumDofs());

    _J.col(index) = _JBodyNode.col(localIndex++);
  }
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const BodyNode* _bodyNode) const
{
  math::Jacobian J = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian"))
    return J;

  // Get the spatial Jacobian of the targeting BodyNode
  const math::Jacobian JBodyNode = _bodyNode->getJacobian();

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(J, _bodyNode, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const BodyNode* _bodyNode,
                                     const Frame* _inCoordinatesOf) const
{
  math::Jacobian J = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian"))
    return J;

  // Get the spatial Jacobian of the targeting BodyNode
  const math::Jacobian JBodyNode = _bodyNode->getJacobian(_inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(J, _bodyNode, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const BodyNode* _bodyNode,
                                     const Eigen::Vector3d& _localOffset) const
{
  math::Jacobian J = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian"))
    return J;

  // Get the spatial Jacobian of the targeting BodyNode
  const math::Jacobian JBodyNode = _bodyNode->getJacobian(_localOffset);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(J, _bodyNode, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const BodyNode* _bodyNode,
                                     const Eigen::Vector3d& _localOffset,
                                     const Frame* _inCoordinatesOf) const
{
  math::Jacobian J = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian"))
    return J;

  // Get the spatial Jacobian of the targeting BodyNode
  const math::Jacobian JBodyNode = _bodyNode->getJacobian(_localOffset,
                                                          _inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(J, _bodyNode, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getWorldJacobian(const BodyNode* _bodyNode) const
{
  math::Jacobian J = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian"))
    return J;

  // Get the spatial Jacobian of the targeting BodyNode
  const math::Jacobian JBodyNode = _bodyNode->getWorldJacobian();

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(J, _bodyNode, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getWorldJacobian(
    const BodyNode* _bodyNode,
    const Eigen::Vector3d& _localOffset) const
{
  math::Jacobian J = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian"))
    return J;

  // Get the spatial Jacobian of the targeting BodyNode
  const math::Jacobian JBodyNode = _bodyNode->getWorldJacobian(_localOffset);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(J, _bodyNode, JBodyNode);

  return J;
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobian(
    const BodyNode* _bodyNode,
    const Frame* _inCoordinatesOf) const
{
  math::LinearJacobian Jv = math::LinearJacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "linear Jacobian"))
    return Jv;

  // Get the linear Jacobian of the targeting BodyNode
  math::LinearJacobian JvBodyNode
      = _bodyNode->getLinearJacobian(_inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::LinearJacobian>(Jv, _bodyNode, JvBodyNode);

  return Jv;
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobian(
    const BodyNode* _bodyNode,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  math::LinearJacobian Jv = math::LinearJacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "linear Jacobian"))
    return Jv;

  // Get the linear Jacobian of the targeting BodyNode
  math::LinearJacobian JvBodyNode
      = _bodyNode->getLinearJacobian(_localOffset, _inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::LinearJacobian>(Jv, _bodyNode, JvBodyNode);

  return Jv;
}

//==============================================================================
math::AngularJacobian Skeleton::getAngularJacobian(
    const BodyNode* _bodyNode,
    const Frame* _inCoordinatesOf) const
{
  math::AngularJacobian Jw = math::AngularJacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "angular Jacobian"))
    return Jw;

  // Get the angular Jacobian of the targeting BodyNode
  math::AngularJacobian JwBodyNode
      = _bodyNode->getAngularJacobian(_inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::AngularJacobian>(Jw, _bodyNode, JwBodyNode);

  return Jw;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const BodyNode* _bodyNode) const
{
  math::Jacobian dJ = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian time derivative"))
    return dJ;

  // Get the spatial Jacobian time derivative of the targeting BodyNode
  math::Jacobian dJBodyNode = _bodyNode->getJacobianSpatialDeriv();

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(dJ, _bodyNode, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const BodyNode* _bodyNode,
    const Frame* _inCoordinatesOf) const
{
  math::Jacobian dJ = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian time derivative"))
    return dJ;

  // Get the spatial Jacobian time derivative of the targeting BodyNode
  math::Jacobian dJBodyNode
      = _bodyNode->getJacobianSpatialDeriv(_inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(dJ, _bodyNode, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const BodyNode* _bodyNode,
    const Eigen::Vector3d& _localOffset) const
{
  math::Jacobian dJ = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian time derivative"))
    return dJ;

  // Get the spatial Jacobian time derivative of the targeting BodyNode
  math::Jacobian dJBodyNode
      = _bodyNode->getJacobianSpatialDeriv(_localOffset);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(dJ, _bodyNode, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const BodyNode* _bodyNode,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  math::Jacobian dJ = math::Jacobian::Zero(6, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "spatial Jacobian time derivative"))
    return dJ;

  // Get the spatial Jacobian time derivative of the targeting BodyNode
  math::Jacobian dJBodyNode
      = _bodyNode->getJacobianSpatialDeriv(_localOffset, _inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(dJ, _bodyNode, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const BodyNode* _bodyNode) const
{
  math::Jacobian dJ = math::Jacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode,
                       "spatial Jacobian (classical) time derivative"))
    return dJ;

  // Get the spatial Jacobian time derivative of the targeting BodyNode
  math::Jacobian dJBodyNode = _bodyNode->getJacobianClassicDeriv();

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(dJ, _bodyNode, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const BodyNode* _bodyNode,
    const Frame* _inCoordinatesOf) const
{
  math::Jacobian dJ = math::Jacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode,
                       "spatial Jacobian (classical) time derivative"))
    return dJ;

  // Get the spatial Jacobian time derivative of the targeting BodyNode
  math::Jacobian dJBodyNode
      = _bodyNode->getJacobianClassicDeriv(_inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(dJ, _bodyNode, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const BodyNode* _bodyNode,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  math::Jacobian dJ = math::Jacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode,
                       "spatial Jacobian (classical) time derivative"))
    return dJ;

  // Get the spatial Jacobian time derivative of the targeting BodyNode
  math::Jacobian dJBodyNode
      = _bodyNode->getJacobianClassicDeriv(_localOffset, _inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::Jacobian>(dJ, _bodyNode, dJBodyNode);

  return dJ;
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobianDeriv(
    const BodyNode* _bodyNode,
    const Frame* _inCoordinatesOf) const
{
  math::LinearJacobian dJv = math::LinearJacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "linear Jacobian time derivative"))
    return dJv;

  // Get the linear Jacobian time derivative of the targeting BodyNode
  math::LinearJacobian JvBodyNode
      = _bodyNode->getLinearJacobianDeriv(_inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::LinearJacobian>(dJv, _bodyNode, JvBodyNode);

  return dJv;
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobianDeriv(
    const BodyNode* _bodyNode,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  math::LinearJacobian dJv = math::LinearJacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "linear Jacobian time derivative"))
    return dJv;

  // Get the linear Jacobian time derivative of the targeting BodyNode
  math::LinearJacobian JvBodyNode
      = _bodyNode->getLinearJacobianDeriv(_localOffset, _inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::LinearJacobian>(dJv, _bodyNode, JvBodyNode);

  return dJv;
}

//==============================================================================
math::AngularJacobian Skeleton::getAngularJacobianDeriv(
    const BodyNode* _bodyNode, const Frame* _inCoordinatesOf) const
{
  math::AngularJacobian dJw = math::AngularJacobian::Zero(3, getNumDofs());

  // If _bodyNode is nullptr or not in this Skeleton, return zero Jacobian
  if (!isValidBodyNode(this, _bodyNode, "angular Jacobian time derivative"))
    return dJw;

  // Get the angular Jacobian time derivative of the targeting BodyNode
  math::AngularJacobian JwBodyNode
      = _bodyNode->getAngularJacobianDeriv(_inCoordinatesOf);

  // Assign the BodyNode's Jacobian to the full-sized Jacobian
  assignJacobian<math::AngularJacobian>(dJw, _bodyNode, JwBodyNode);

  return dJw;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getMassMatrix()
{
  if (mIsMassMatrixDirty)
    updateMassMatrix();
  return mM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getAugMassMatrix()
{
  if (mIsAugMassMatrixDirty)
    updateAugMassMatrix();

  return mAugM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvMassMatrix()
{
  if (mIsInvMassMatrixDirty)
    updateInvMassMatrix();

  return mInvM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvAugMassMatrix()
{
  if (mIsInvAugMassMatrixDirty)
    updateInvAugMassMatrix();

  return mInvAugM;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisForces()
{
  if (mIsCoriolisForcesDirty)
    updateCoriolisForces();

  return mCvec;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getGravityForces()
{
  if (mIsGravityForcesDirty)
    updateGravityForces();

  return mG;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisAndGravityForces()
{
  if (mIsCoriolisAndGravityForcesDirty)
    updateCoriolisAndGravityForces();

  return mCg;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getExternalForces()
{
  if (mIsExternalForcesDirty)
    updateExternalForces();

  return mFext;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getConstraintForces()
{
  const size_t dof = getNumDofs();
  mFc = Eigen::VectorXd::Zero(dof);

  // Body constraint impulses
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateSpatialToGeneralized(&mFc, (*it)->getConstraintImpulse());
  }

  // Joint constraint impulses
  size_t index = 0;
  for (size_t i = 0; i < dof; ++i)
  {
    Joint* joint      = mDofs[i]->getJoint();
    size_t localIndex = mDofs[i]->getIndexInJoint();

    mFc[index++] += joint->getConstraintImpulse(localIndex);
  }
  assert(index == dof);

  // Get force by devide impulse by time step
  mFc = mFc / mTimeStep;

  return mFc;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisForceVector()
{
  return getCoriolisForces();
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getGravityForceVector()
{
  return getGravityForces();
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCombinedVector()
{
  return getCoriolisAndGravityForces();
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getExternalForceVector()
{
  return getExternalForces();
}

//==============================================================================
//const Eigen::VectorXd& Skeleton::getDampingForceVector() {
//  if (mIsDampingForceVectorDirty)
//    updateDampingForceVector();
//  return mFd;
//}

//==============================================================================
const Eigen::VectorXd& Skeleton::getConstraintForceVector()
{
  return getConstraintForces();
}

//==============================================================================
void Skeleton::draw(renderer::RenderInterface* _ri, const Eigen::Vector4d& _color,
                    bool _useDefaultColor) const
{
  getRootBodyNode()->draw(_ri, _color, _useDefaultColor);
}

//==============================================================================
void Skeleton::drawMarkers(renderer::RenderInterface* _ri,
                           const Eigen::Vector4d& _color,
                           bool _useDefaultColor) const
{
  getRootBodyNode()->drawMarkers(_ri, _color, _useDefaultColor);
}

//==============================================================================
void Skeleton::registerJoint(Joint* _newJoint)
{
  if (NULL == _newJoint)
  {
    dterr << "[Skeleton::registerJoint] Error: Attempting to add a NULL joint "
             "to the Skeleton named '" << mName << "'!\n";
    return;
  }

  addEntryToJointNameMgr(_newJoint);
  _newJoint->mSkeleton = this;

  for (size_t i = 0; i < _newJoint->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = _newJoint->getDof(i);
    dof->mName = mNameMgrForDofs.issueNewNameAndAdd(dof->getName(), dof);
  }
}

//==============================================================================
void Skeleton::unregisterJoint(Joint* _oldJoint)
{
  if (NULL == _oldJoint)
    return;

  mNameMgrForJoints.removeName(_oldJoint->getName());

  for (size_t i = 0; i < _oldJoint->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = _oldJoint->getDof(i);
    mNameMgrForDofs.removeName(dof->getName());
  }
}

//==============================================================================
void Skeleton::notifyArticulatedInertiaUpdate()
{
  if(mIsArticulatedInertiaDirty)
    return;

  mIsArticulatedInertiaDirty = true;
  mIsMassMatrixDirty = true;
  mIsAugMassMatrixDirty = true;
  mIsInvMassMatrixDirty = true;
  mIsInvAugMassMatrixDirty = true;
  mIsCoriolisForcesDirty = true;
  mIsGravityForcesDirty = true;
  mIsCoriolisAndGravityForcesDirty = true;
}

//==============================================================================
void Skeleton::updateArticulatedInertia() const
{
  for (std::vector<BodyNode*>::const_reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it)
  {
    (*it)->updateArtInertia(mTimeStep);
  }

  mIsArticulatedInertiaDirty = false;
}

//==============================================================================
void Skeleton::updateMassMatrix()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mM.cols()) == getNumDofs()
         && static_cast<size_t>(mM.rows()) == getNumDofs());

  mM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  size_t dof = getNumDofs();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (size_t j = 0; j < dof; ++j)
  {
    e[j] = 1.0;
    setAccelerations(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it)
    {
      (*it)->updateMassMatrix();
    }

    // Mass matrix
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it)
    {
      (*it)->aggregateMassMatrix(&mM, j);
      size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        size_t iStart = (*it)->mParentJoint->getIndexInSkeleton(0);

        if (iStart + localDof < j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mM.triangularView<Eigen::StrictlyUpper>() = mM.transpose();

  // Restore the origianl generalized accelerations
  setAccelerations(originalGenAcceleration);

  mIsMassMatrixDirty = false;
}

//==============================================================================
void Skeleton::updateAugMassMatrix()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mAugM.cols()) == getNumDofs()
         && static_cast<size_t>(mAugM.rows()) == getNumDofs());

  mAugM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  int dof = getNumDofs();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (int j = 0; j < dof; ++j)
  {
    e[j] = 1.0;
    setAccelerations(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it)
    {
      (*it)->updateMassMatrix();
    }

    // Mass matrix
    //    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
    //         it != mBodyNodes.end(); ++it)
    for (int i = mBodyNodes.size() - 1; i > -1 ; --i)
    {
      mBodyNodes[i]->aggregateAugMassMatrix(&mAugM, j, mTimeStep);
      int localDof = mBodyNodes[i]->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        int iStart =
            mBodyNodes[i]->mParentJoint->getIndexInSkeleton(0);
        if (iStart + localDof < j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mAugM.triangularView<Eigen::StrictlyUpper>() = mAugM.transpose();

  // Restore the origianl internal force
  setAccelerations(originalGenAcceleration);

  mIsAugMassMatrixDirty = false;
}

//==============================================================================
void Skeleton::updateInvMassMatrix()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mInvM.cols()) == getNumDofs()
         && static_cast<size_t>(mInvM.rows()) == getNumDofs());

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = getForces();

  // Note: we do not need to update articulated inertias here, because they will
  // be updated when BodyNode::updateInvMassMatrix() calls
  // BodyNode::getArticulatedInertia()

  int dof = getNumDofs();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (int j = 0; j < dof; ++j)
  {
    e[j] = 1.0;
    setForces(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it)
    {
      (*it)->updateInvMassMatrix();
    }

    // Inverse of mass matrix
    //    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
    //         it != mBodyNodes.end(); ++it)
    for (size_t i = 0; i < mBodyNodes.size(); ++i)
    {
      mBodyNodes[i]->aggregateInvMassMatrix(&mInvM, j);
      int localDof = mBodyNodes[i]->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        int iStart =
            mBodyNodes[i]->mParentJoint->getIndexInSkeleton(0);
        if (iStart + localDof > j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mInvM.triangularView<Eigen::StrictlyLower>() = mInvM.transpose();

  // Restore the origianl internal force
  setForces(originalInternalForce);

  mIsInvMassMatrixDirty = false;
}

//==============================================================================
void Skeleton::updateInvAugMassMatrix()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mInvAugM.cols()) == getNumDofs()
         && static_cast<size_t>(mInvAugM.rows()) == getNumDofs());

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = getForces();

  int dof = getNumDofs();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (int j = 0; j < dof; ++j)
  {
    e[j] = 1.0;
    setForces(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it)
    {
      (*it)->updateInvAugMassMatrix();
    }

    // Inverse of mass matrix
    //    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
    //         it != mBodyNodes.end(); ++it)
    for (size_t i = 0; i < mBodyNodes.size(); ++i)
    {
      mBodyNodes[i]->aggregateInvAugMassMatrix(&mInvAugM, j, mTimeStep);
      int localDof = mBodyNodes[i]->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        int iStart =
            mBodyNodes[i]->mParentJoint->getIndexInSkeleton(0);
        if (iStart + localDof > j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mInvAugM.triangularView<Eigen::StrictlyLower>() = mInvAugM.transpose();

  // Restore the origianl internal force
  setForces(originalInternalForce);

  mIsInvAugMassMatrixDirty = false;
}

//==============================================================================
void Skeleton::updateCoriolisForceVector()
{
  updateCoriolisForces();
}

//==============================================================================
void Skeleton::updateCoriolisForces()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mCvec.size()) == getNumDofs());

  mCvec.setZero();

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->updateCombinedVector();
  }

  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateCoriolisForceVector(&mCvec);
  }

  mIsCoriolisForcesDirty = false;
}

//==============================================================================
void Skeleton::updateGravityForceVector()
{
  updateGravityForces();
}

//==============================================================================
void Skeleton::updateGravityForces()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mG.size()) == getNumDofs());

  // Calcualtion mass matrix, M
  mG.setZero();
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateGravityForceVector(&mG, mGravity);
  }

  mIsGravityForcesDirty = false;
}

//==============================================================================
void Skeleton::updateCombinedVector()
{
  updateCoriolisAndGravityForces();
}

//==============================================================================
void Skeleton::updateCoriolisAndGravityForces()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mCg.size()) == getNumDofs());

  mCg.setZero();
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->updateCombinedVector();
  }

  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateCombinedVector(&mCg, mGravity);
  }

  mIsCoriolisAndGravityForcesDirty = false;
}

//==============================================================================
void Skeleton::updateExternalForceVector()
{
  updateExternalForces();
}

//==============================================================================
void Skeleton::updateExternalForces()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mFext.size()) == getNumDofs());

  // Clear external force.
  mFext.setZero();

  for (std::vector<BodyNode*>::reverse_iterator itr = mBodyNodes.rbegin();
       itr != mBodyNodes.rend(); ++itr)
  {
    (*itr)->aggregateExternalForces(&mFext);
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

  mIsExternalForcesDirty = false;
}

//==============================================================================
//void Skeleton::updateDampingForceVector() {
//  assert(mFd.size() == getNumDofs());
//  assert(getNumDofs() > 0);

//  // Clear external force.
//  mFd.setZero();

//  for (std::vector<BodyNode*>::iterator itr = mBodyNodes.begin();
//       itr != mBodyNodes.end(); ++itr) {
//    Eigen::VectorXd jointDampingForce =
//        (*itr)->getParentJoint()->getDampingForces();
//    for (int i = 0; i < jointDampingForce.size(); i++) {
//      mFd((*itr)->getParentJoint()->getIndexInSkeleton(0)) =
//          jointDampingForce(i);
//    }
//  }

//  for (std::vector<SoftBodyNode*>::iterator it = mSoftBodyNodes.begin();
//       it != mSoftBodyNodes.end(); ++it) {
//    for (int i = 0; i < (*it)->getNumPointMasses(); ++i) {
//      PointMass* pm = (*it)->getPointMass(i);
//      int iStart = pm->getGenCoord(0)->getIndexInSkeleton();
//      mFd.segment<3>(iStart) = -(*it)->getDampingCoefficient() * pm->getVelocities();
//    }
//  }
//}

//==============================================================================
void Skeleton::computeForwardDynamics()
{
  //
//  computeForwardDynamicsRecursionPartA(); // No longer needed with auto-update

  //
  computeForwardDynamicsRecursionPartB();
}

//==============================================================================
void Skeleton::computeForwardDynamicsRecursionPartA()
{
  // Update body transformations, velocities, and partial acceleration due to
  // parent joint's velocity
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->updateTransform();
    (*it)->updateVelocity();
    (*it)->updatePartialAcceleration();
  }

  mIsArticulatedInertiaDirty = true;
  mIsMassMatrixDirty = true;
  mIsAugMassMatrixDirty = true;
  mIsInvMassMatrixDirty = true;
  mIsInvAugMassMatrixDirty = true;
  mIsCoriolisForcesDirty = true;
  mIsGravityForcesDirty = true;
  mIsCoriolisAndGravityForcesDirty = true;
  mIsExternalForcesDirty = true;
//  mIsDampingForceVectorDirty = true;

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->mIsBodyJacobianDirty = true;
    (*it)->mIsWorldJacobianDirty = true;
    (*it)->mIsBodyJacobianSpatialDerivDirty = true;
    (*it)->mIsWorldJacobianClassicDerivDirty = true;
  }
}

//==============================================================================
void Skeleton::computeForwardDynamicsRecursionPartB()
{
  // Note: Articulated Inertias will be updated automatically when
  // getArtInertiaImplicit() is called in BodyNode::updateBiasForce()

  for (auto it = mBodyNodes.rbegin(); it != mBodyNodes.rend(); ++it)
    (*it)->updateBiasForce(mGravity, mTimeStep);

  // Forward recursion
  for (auto& bodyNode : mBodyNodes)
  {
    bodyNode->updateAccelerationFD();
    bodyNode->updateTransmittedForceFD();
    bodyNode->updateJointForceFD(mTimeStep, true, true);
  }
}

//==============================================================================
void Skeleton::computeInverseDynamics(bool _withExternalForces,
                                      bool _withDampingForces)
{
  //
  computeInverseDynamicsRecursionB(_withExternalForces, _withDampingForces);
}

//==============================================================================
void Skeleton::computeInverseDynamicsRecursionA()
{
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->updateTransform();
    (*it)->updateVelocity();
    (*it)->updatePartialAcceleration();
    (*it)->updateAccelerationID();
  }

  mIsArticulatedInertiaDirty = true;
  mIsMassMatrixDirty = true;
  mIsAugMassMatrixDirty = true;
  mIsInvMassMatrixDirty = true;
  mIsInvAugMassMatrixDirty = true;
  mIsCoriolisForcesDirty = true;
  mIsGravityForcesDirty = true;
  mIsCoriolisAndGravityForcesDirty = true;
  mIsExternalForcesDirty = true;
//  mIsDampingForceVectorDirty = true;

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->mIsBodyJacobianDirty = true;
    (*it)->mIsWorldJacobianDirty = true;
    (*it)->mIsBodyJacobianSpatialDerivDirty = true;
    (*it)->mIsWorldJacobianClassicDerivDirty = true;
  }
}

//==============================================================================
void Skeleton::computeInverseDynamicsRecursionB(bool _withExternalForces,
                                                bool _withDampingForces,
                                                bool _withSpringForces)
{
  // Skip immobile or 0-dof skeleton
  if (getNumDofs() == 0)
    return;

  // Backward recursion
  for (auto it = mBodyNodes.rbegin(); it != mBodyNodes.rend(); ++it)
  {
    (*it)->updateTransmittedForceID(mGravity, _withExternalForces);
    (*it)->updateJointForceID(mTimeStep,
                              _withDampingForces,
                              _withSpringForces);
  }
}

//==============================================================================
void Skeleton::clearExternalForces()
{
  for (auto& bodyNode : mBodyNodes)
    bodyNode->clearExternalForces();
}

//==============================================================================
void Skeleton::clearConstraintImpulses()
{
  for (auto& bodyNode : mBodyNodes)
    bodyNode->clearConstraintImpulse();
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode)
{
  // Assertions
  assert(_bodyNode != NULL);
  assert(getNumDofs() > 0);

  // This skeleton should contains _bodyNode
  assert(std::find(mBodyNodes.begin(), mBodyNodes.end(), _bodyNode)
         != mBodyNodes.end());

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    assert(mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Prepare cache data
  BodyNode* it = _bodyNode;
  while (it != NULL)
  {
    it->updateBiasImpulse();
    it = it->getParentBodyNode();
  }
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode,
                                 const Eigen::Vector6d& _imp)
{
  // Assertions
  assert(_bodyNode != NULL);
  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(std::find(mBodyNodes.begin(), mBodyNodes.end(), _bodyNode)
         != mBodyNodes.end());

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    assert(mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Set impulse to _bodyNode
  _bodyNode->mConstraintImpulse = _imp;

  // Prepare cache data
  BodyNode* it = _bodyNode;
  while (it != NULL)
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
  assert(_bodyNode1 != NULL);
  assert(_bodyNode2 != NULL);
  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(std::find(mBodyNodes.begin(), mBodyNodes.end(), _bodyNode1)
         != mBodyNodes.end());
  assert(std::find(mBodyNodes.begin(), mBodyNodes.end(), _bodyNode2)
         != mBodyNodes.end());

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    assert(mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Set impulse to _bodyNode
  _bodyNode1->mConstraintImpulse = _imp1;
  _bodyNode2->mConstraintImpulse = _imp2;

  // Find which body is placed later in the list of body nodes in this skeleton
  std::vector<BodyNode*>::reverse_iterator it1
      = std::find(mBodyNodes.rbegin(), mBodyNodes.rend(), _bodyNode1);
  std::vector<BodyNode*>::reverse_iterator it2
      = std::find(mBodyNodes.rbegin(), mBodyNodes.rend(), _bodyNode2);

  std::vector<BodyNode*>::reverse_iterator it = std::min(it1, it2);

  // Prepare cache data
  for (; it != mBodyNodes.rend(); ++it)
    (*it)->updateBiasImpulse();

  _bodyNode1->mConstraintImpulse.setZero();
  _bodyNode2->mConstraintImpulse.setZero();
}

//==============================================================================
void Skeleton::updateBiasImpulse(SoftBodyNode* _softBodyNode,
                                 PointMass* _pointMass,
                                 const Eigen::Vector3d& _imp)
{
  // Assertions
  assert(_softBodyNode != NULL);
  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(std::find(mSoftBodyNodes.begin(), mSoftBodyNodes.end(), _softBodyNode)
         != mSoftBodyNodes.end());

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    assert(mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Set impulse to _bodyNode
  Eigen::Vector3d oldConstraintImpulse =_pointMass->getConstraintImpulses();
  _pointMass->setConstraintImpulse(_imp, true);

  // Prepare cache data
  BodyNode* it = _softBodyNode;
  while (it != NULL)
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
  for (auto& bodyNode : mBodyNodes)
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
  for (auto it = mBodyNodes.rbegin(); it != mBodyNodes.rend(); ++it)
    (*it)->updateBiasImpulse();

  // Forward recursion
  for (auto& bodyNode : mBodyNodes)
  {
    bodyNode->updateVelocityChangeFD();
    bodyNode->updateTransmittedImpulse();
    bodyNode->updateJointImpulseFD();
    bodyNode->updateConstrainedTerms(mTimeStep);
  }
}

//==============================================================================
void Skeleton::setConstraintForceVector(const Eigen::VectorXd& _Fc)
{
  mFc = _Fc;
}

//==============================================================================
Eigen::Vector3d Skeleton::getCOM(const Frame* _withRespectTo) const
{
  Eigen::Vector3d com(0.0, 0.0, 0.0);

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
      &BodyNode::getCOMSpatialAcceleration>(this, _relativeTo, _inCoordinatesOf);
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
    JacType (BodyNode::*getJacFn)(const Eigen::Vector3d&, const Frame*) const>
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
  return getCOMJacobianTemplate<math::Jacobian, &BodyNode::getJacobian>(
        this, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getCOMLinearJacobian(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::LinearJacobian,
           &BodyNode::getLinearJacobian>(this, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getCOMJacobianSpatialDeriv(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::Jacobian,
      &BodyNode::getJacobianSpatialDeriv>(this, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getCOMLinearJacobianDeriv(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::LinearJacobian,
      &BodyNode::getLinearJacobianDeriv>(this, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Skeleton::getWorldCOM()
{
  return getCOM(Frame::World());
}

//==============================================================================
Eigen::Vector3d Skeleton::getWorldCOMVelocity()
{
  return getCOMLinearVelocity();
}

//==============================================================================
Eigen::Vector3d Skeleton::getWorldCOMAcceleration()
{
  return getCOMLinearAcceleration();
}

//==============================================================================
Eigen::MatrixXd Skeleton::getWorldCOMJacobian()
{
  return getCOMLinearJacobian();
}

//==============================================================================
Eigen::MatrixXd Skeleton::getWorldCOMJacobianTimeDeriv()
{
  return getCOMLinearJacobianDeriv();
}

//==============================================================================
double Skeleton::getKineticEnergy() const
{
  double KE = 0.0;

  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
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

  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    PE += (*it)->getPotentialEnergy(mGravity);
    PE += (*it)->getParentJoint()->getPotentialEnergy();
  }

  return PE;
}

}  // namespace dynamics
}  // namespace dart
