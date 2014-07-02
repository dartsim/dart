/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
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
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/PointMass.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Marker.h"

namespace dart {
namespace dynamics {

//==============================================================================
Skeleton::Skeleton(const std::string& _name)
  : mName(_name),
    mDof(0),
    mEnabledSelfCollisionCheck(false),
    mEnabledAdjacentBodyCheck(false),
    mIsMobile(true),
    mTimeStep(0.001),
    mGravity(Eigen::Vector3d(0.0, 0.0, -9.81)),
    mTotalMass(0.0),
    mIsArticulatedInertiaDirty(true),
    mIsMassMatrixDirty(true),
    mIsAugMassMatrixDirty(true),
    mIsInvMassMatrixDirty(true),
    mIsInvAugMassMatrixDirty(true),
    mIsCoriolisVectorDirty(true),
    mIsGravityForceVectorDirty(true),
    mIsCombinedVectorDirty(true),
    mIsExternalForceVectorDirty(true),
    mIsDampingForceVectorDirty(true),
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

  SoftBodyNode* softBodyNode = dynamic_cast<SoftBodyNode*>(_body);
  if (softBodyNode)
    mSoftBodyNodes.push_back(softBodyNode);
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
BodyNode* Skeleton::getRootBodyNode() const
{
  // We assume that the first element of body nodes is root.
  return mBodyNodes[0];
}

//==============================================================================
BodyNode* Skeleton::getBodyNode(size_t _idx) const
{
  return mBodyNodes[_idx];
}

//==============================================================================
SoftBodyNode* Skeleton::getSoftBodyNode(size_t _idx) const
{
  assert(0 <= _idx && _idx < mSoftBodyNodes.size());
  return mSoftBodyNodes[_idx];
}

//==============================================================================
BodyNode* Skeleton::getBodyNode(const std::string& _name) const
{
  assert(!_name.empty());

  for (std::vector<BodyNode*>::const_iterator itrBody = mBodyNodes.begin();
       itrBody != mBodyNodes.end(); ++itrBody)
  {
    if ((*itrBody)->getName() == _name)
      return *itrBody;
  }

  return NULL;
}

//==============================================================================
SoftBodyNode* Skeleton::getSoftBodyNode(const std::string& _name) const
{
  assert(!_name.empty());
  for (std::vector<SoftBodyNode*>::const_iterator itrSoftBodyNode =
       mSoftBodyNodes.begin();
       itrSoftBodyNode != mSoftBodyNodes.end(); ++itrSoftBodyNode)
  {
    if ((*itrSoftBodyNode)->getName() == _name)
      return *itrSoftBodyNode;
  }

  return NULL;
}

//==============================================================================
Joint* Skeleton::getJoint(size_t _idx) const
{
  return mBodyNodes[_idx]->getParentJoint();
}

//==============================================================================
Joint* Skeleton::getJoint(const std::string& _name) const
{
  assert(!_name.empty());

  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    if ((*it)->getParentJoint()->getName() == _name)
      return (*it)->getParentJoint();
  }

  return NULL;
}

//==============================================================================
Marker* Skeleton::getMarker(const std::string& _name) const
{
  assert(!_name.empty());

  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    for (size_t i = 0; i < (*it)->getNumMarkers(); ++i)
    {
      if ((*it)->getMarker(i)->getName() == _name)
        return (*it)->getMarker(i);
    }
  }

  return NULL;
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
  mGenCoordInfos.clear();
  mDof = 0;
  for (size_t i = 0; i < getNumBodyNodes(); ++i)
  {
    Joint* joint = mBodyNodes[i]->getParentJoint();

    for (size_t j = 0; j < joint->getDof(); ++j)
    {
      GenCoordInfo genCoord;
      genCoord.joint = joint;
      genCoord.localIndex = j;

      mGenCoordInfos.push_back(genCoord);

      joint->setIndexInSkeleton(j, mGenCoordInfos.size() - 1);
    }

    mBodyNodes[i]->init(this);
    mDof += mBodyNodes[i]->getParentJoint()->getDof();
  }

  // Compute transformations, velocities, and partial accelerations
  computeForwardDynamicsRecursionPartA();

  // Set dimension of dynamics quantities
  int dof = getDof();
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
  return mDof;
}

//==============================================================================
GenCoordInfo Skeleton::getGenCoordInfo(size_t _index) const
{
  assert(_index < getDof());

  return mGenCoordInfos[_index];
}

//==============================================================================
void Skeleton::setPosition(size_t _index, double _position)
{
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setPosition(mGenCoordInfos[_index].localIndex,
                                            _position);
}

//==============================================================================
double Skeleton::getPosition(size_t _index) const
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getPosition(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setPositions(const Eigen::VectorXd& _configs)
{
  size_t index = 0;
  size_t dof = 0;
  Joint* joint;

  for (size_t i = 0; i < mBodyNodes.size(); ++i)
  {
    joint = mBodyNodes[i]->getParentJoint();

    dof = joint->getDof();

    if (dof)
    {
      joint->setPositions(_configs.segment(index, dof));

      index += dof;
    }
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getPositions() const
{
  size_t index = 0;
  size_t dof = getDof();
  Eigen::VectorXd pos(dof);

  for (size_t i = 0; i < dof; ++i)
  {
    pos[index++]
        = mGenCoordInfos[i].joint->getPosition(mGenCoordInfos[i].localIndex);
  }

  assert(index == dof);

  return pos;
}

//==============================================================================
Eigen::VectorXd Skeleton::getPositionSegment(
    const std::vector<size_t>& _id) const
{
  Eigen::VectorXd q(_id.size());

  Joint* joint;
  size_t localIndex;

  for (size_t i = 0; i < _id.size(); ++i)
  {
    joint = mGenCoordInfos[_id[i]].joint;
    localIndex = mGenCoordInfos[_id[i]].localIndex;

    q[i] = joint->getPosition(localIndex);
  }

  return q;
}

//==============================================================================
void Skeleton::setPositionSegment(const std::vector<size_t>& _id,
                                  const Eigen::VectorXd& _positions)
{
  Joint* joint;
  size_t localIndex;

  for (size_t i = 0; i < _id.size(); ++i)
  {
    joint = mGenCoordInfos[_id[i]].joint;
    localIndex = mGenCoordInfos[_id[i]].localIndex;

    joint->setPosition(localIndex, _positions[i]);
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
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setPositionLowerLimit(
        mGenCoordInfos[_index].localIndex, _position);
}

//==============================================================================
double Skeleton::getPositionLowerLimit(size_t _index)
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getPositionLowerLimit(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setPositionUpperLimit(size_t _index, double _position)
{
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setPositionUpperLimit(
        mGenCoordInfos[_index].localIndex, _position);
}

//==============================================================================
double Skeleton::getPositionUpperLimit(size_t _index)
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getPositionUpperLimit(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setVelocity(size_t _index, double _velocity)
{
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setVelocity(mGenCoordInfos[_index].localIndex,
                                            _velocity);
}

//==============================================================================
double Skeleton::getVelocity(size_t _index) const
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getVelocity(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setVelocities(const Eigen::VectorXd& _genVels)
{
  size_t index = 0;
  size_t dof = 0;
  Joint* joint;

  for (size_t i = 0; i < mBodyNodes.size(); ++i)
  {
    joint = mBodyNodes[i]->getParentJoint();

    dof = joint->getDof();

    if (dof)
    {
      joint->setVelocities(_genVels.segment(index, dof));

      index += dof;
    }
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getVelocities() const
{
  size_t index = 0;
  size_t dof = getDof();
  Eigen::VectorXd vel(dof);

  for (size_t i = 0; i < dof; ++i)
  {
    vel[index++]
        = mGenCoordInfos[i].joint->getVelocity(mGenCoordInfos[i].localIndex);
  }

  assert(index == dof);

  return vel;
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
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setVelocityLowerLimit(
        mGenCoordInfos[_index].localIndex, _velocity);
}

//==============================================================================
double Skeleton::getVelocityLowerLimit(size_t _index)
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getVelocityLowerLimit(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setVelocityUpperLimit(size_t _index, double _velocity)
{
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setVelocityUpperLimit(
        mGenCoordInfos[_index].localIndex, _velocity);
}

//==============================================================================
double Skeleton::getVelocityUpperLimit(size_t _index)
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getVelocityUpperLimit(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setAcceleration(size_t _index, double _acceleration)
{
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setAcceleration(
        mGenCoordInfos[_index].localIndex, _acceleration);
}

//==============================================================================
double Skeleton::getAcceleration(size_t _index) const
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getAcceleration(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setAccelerations(const Eigen::VectorXd& _accelerations)
{
  size_t index = 0;
  size_t dof = 0;
  Joint* joint;

  for (size_t i = 0; i < mBodyNodes.size(); ++i)
  {
    joint = mBodyNodes[i]->getParentJoint();

    dof = joint->getDof();

    if (dof)
    {
      joint->setAccelerations(_accelerations.segment(index, dof));

      index += dof;
    }
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getAccelerations() const
{
  size_t index = 0;
  size_t dof = getDof();
  Eigen::VectorXd acc(dof);

  for (size_t i = 0; i < dof; ++i)
  {
    acc[index++]
        = mGenCoordInfos[i].joint->getAcceleration(mGenCoordInfos[i].localIndex);
  }

  assert(index == dof);

  return acc;
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
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setForce(
        mGenCoordInfos[_index].localIndex, _force);
}

//==============================================================================
double Skeleton::getForce(size_t _index)
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getForce(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setForces(const Eigen::VectorXd& _forces)
{
  size_t index = 0;
  size_t dof = 0;
  Joint* joint;

  for (size_t i = 0; i < mBodyNodes.size(); ++i)
  {
    joint = mBodyNodes[i]->getParentJoint();

    dof = joint->getDof();

    if (dof)
    {
      joint->setForces(_forces.segment(index, dof));

      index += dof;
    }
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getForces() const
{
  size_t index = 0;
  size_t dof = getDof();
  Eigen::VectorXd force(dof);

  for (size_t i = 0; i < dof; ++i)
  {
    force[index++]
        = mGenCoordInfos[i].joint->getForce(mGenCoordInfos[i].localIndex);
  }

  assert(index == dof);

  return force;
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
    {
      mSoftBodyNodes[i]->getPointMass(j)->resetForces();
    }
  }
}

//==============================================================================
void Skeleton::setForceLowerLimit(size_t _index, double _force)
{
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setForceLowerLimit(
        mGenCoordInfos[_index].localIndex, _force);
}

//==============================================================================
double Skeleton::getForceLowerLimit(size_t _index)
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getForceLowerLimit(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
void Skeleton::setForceUpperLimit(size_t _index, double _force)
{
  assert(_index < getDof());

  mGenCoordInfos[_index].joint->setForceUpperLimit(
        mGenCoordInfos[_index].localIndex, _force);
}

//==============================================================================
double Skeleton::getForceUpperLimit(size_t _index)
{
  assert(_index <getDof());

  return mGenCoordInfos[_index].joint->getForceUpperLimit(
        mGenCoordInfos[_index].localIndex);
}

//==============================================================================
Eigen::VectorXd Skeleton::getVelocityChanges() const
{
  size_t index = 0;
  size_t dof = getDof();
  Eigen::VectorXd velChange(dof);

  for (size_t i = 0; i < dof; ++i)
  {
    velChange[index++] = mGenCoordInfos[i].joint->getVelocityChange(
                           mGenCoordInfos[i].localIndex);
  }

  assert(index == dof);

  return velChange;
}

//==============================================================================
void Skeleton::setConstraintImpulses(const Eigen::VectorXd& _impulses)
{
  size_t index = 0;
  size_t dof = getDof();

  for (size_t i = 0; i < dof; ++i)
  {
    mGenCoordInfos[i].joint->setConstraintImpulse(
          mGenCoordInfos[i].localIndex, _impulses[index++]);
  }

  assert(index == getDof());
}

//==============================================================================
Eigen::VectorXd Skeleton::getConstraintImpulses() const
{
  size_t index = 0;
  size_t dof = getDof();
  Eigen::VectorXd impulse(dof);

  for (size_t i = 0; i < dof; ++i)
  {
    impulse[index++]
        = mGenCoordInfos[i].joint->getConstraintImpulse(
            mGenCoordInfos[i].localIndex);
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

    dof = joint->getDof();

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
  Eigen::VectorXd state(2 * getDof());

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
      (*it)->updateAcceleration();
    }
  }

  mIsArticulatedInertiaDirty = true;
  mIsMassMatrixDirty = true;
  mIsAugMassMatrixDirty = true;
  mIsInvMassMatrixDirty = true;
  mIsInvAugMassMatrixDirty = true;
  mIsCoriolisVectorDirty = true;
  mIsGravityForceVectorDirty = true;
  mIsCombinedVectorDirty = true;
  mIsExternalForceVectorDirty = true;
//  mIsDampingForceVectorDirty = true;

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->mIsBodyJacobianDirty = true;
    (*it)->mIsBodyJacobianDerivDirty = true;
  }
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
const Eigen::VectorXd& Skeleton::getCoriolisForceVector()
{
  if (mIsCoriolisVectorDirty)
    updateCoriolisForceVector();

  return mCvec;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getGravityForceVector()
{
  if (mIsGravityForceVectorDirty)
    updateGravityForceVector();

  return mG;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCombinedVector()
{
  if (mIsCombinedVectorDirty)
    updateCombinedVector();

  return mCg;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getExternalForceVector()
{
  if (mIsExternalForceVectorDirty)
    updateExternalForceVector();
  return mFext;
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
  return mFc;
}

//==============================================================================
void Skeleton::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
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
void Skeleton::updateMassMatrix()
{
  if (getDof() == 0)
    return;

  assert(mM.cols() == math::castUIntToInt(getDof())
         && mM.rows() == math::castUIntToInt(getDof()));

  mM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  size_t dof = getDof();
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
      size_t localDof = (*it)->mParentJoint->getDof();
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
  if (getDof() == 0)
    return;

  assert(mAugM.cols() == math::castUIntToInt(getDof())
         && mAugM.rows() == math::castUIntToInt(getDof()));

  mAugM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  int dof = getDof();
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
      int localDof = mBodyNodes[i]->mParentJoint->getDof();
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
  if (getDof() == 0)
    return;

  assert(mInvM.cols() == math::castUIntToInt(getDof())
         && mInvM.rows() == math::castUIntToInt(getDof()));

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = getForces();

  if (mIsArticulatedInertiaDirty)
  {
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it)
    {
      (*it)->updateArtInertia(mTimeStep);
    }

    mIsArticulatedInertiaDirty = false;
  }

  int dof = getDof();
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
      int localDof = mBodyNodes[i]->mParentJoint->getDof();
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
  if (getDof() == 0)
    return;

  assert(mInvAugM.cols() == math::castUIntToInt(getDof())
         && mInvAugM.rows() == math::castUIntToInt(getDof()));

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = getForces();

  int dof = getDof();
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
      int localDof = mBodyNodes[i]->mParentJoint->getDof();
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
  if (getDof() == 0)
    return;

  assert(mCvec.size() == math::castUIntToInt(getDof()));

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

  mIsCoriolisVectorDirty = false;
}

//==============================================================================
void Skeleton::updateGravityForceVector()
{
  if (getDof() == 0)
    return;

  assert(mG.size() == math::castUIntToInt(getDof()));

  // Calcualtion mass matrix, M
  mG.setZero();
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateGravityForceVector(&mG, mGravity);
  }

  mIsGravityForceVectorDirty = false;
}

//==============================================================================
void Skeleton::updateCombinedVector()
{
  if (getDof() == 0)
    return;

  assert(mCg.size() == math::castUIntToInt(getDof()));

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

  mIsCombinedVectorDirty = false;
}

//==============================================================================
void Skeleton::updateExternalForceVector()
{
  if (getDof() == 0)
    return;

  assert(mFext.size() == math::castUIntToInt(getDof()));

  // Clear external force.
  mFext.setZero();

  for (std::vector<BodyNode*>::reverse_iterator itr = mBodyNodes.rbegin();
       itr != mBodyNodes.rend(); ++itr)
  {
    (*itr)->aggregateExternalForces(&mFext);
  }

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

  mIsExternalForceVectorDirty = false;
}

//==============================================================================
//void Skeleton::updateDampingForceVector() {
//  assert(mFd.size() == getDof());
//  assert(getDof() > 0);

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
  computeForwardDynamicsRecursionPartA();

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
  mIsCoriolisVectorDirty = true;
  mIsGravityForceVectorDirty = true;
  mIsCombinedVectorDirty = true;
  mIsExternalForceVectorDirty = true;
//  mIsDampingForceVectorDirty = true;

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->mIsBodyJacobianDirty = true;
    (*it)->mIsBodyJacobianDerivDirty = true;
  }
}

//==============================================================================
void Skeleton::computeForwardDynamicsRecursionPartB()
{
  // Backward recursion
//  if (mIsArticulatedInertiaDirty)
  {
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it)
    {
      (*it)->updateArtInertia(mTimeStep);
      (*it)->updateBiasForce(mGravity, mTimeStep);
    }

    mIsArticulatedInertiaDirty = false;
  }
//  else
//  {
//    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
//         it != mBodyNodes.rend(); ++it)
//    {
//      (*it)->updateBiasForce(mGravity, mTimeStep);
//    }
//  }

  // Forward recursion
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->updateJointAndBodyAcceleration();
    (*it)->updateTransmittedWrench();
  }
}

//==============================================================================
void Skeleton::computeInverseDynamics(bool _withExternalForces,
                                      bool _withDampingForces)
{
  //
  computeInverseDynamicsRecursionA();

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
    (*it)->updateAcceleration();
  }

  mIsArticulatedInertiaDirty = true;
  mIsMassMatrixDirty = true;
  mIsAugMassMatrixDirty = true;
  mIsInvMassMatrixDirty = true;
  mIsInvAugMassMatrixDirty = true;
  mIsCoriolisVectorDirty = true;
  mIsGravityForceVectorDirty = true;
  mIsCombinedVectorDirty = true;
  mIsExternalForceVectorDirty = true;
//  mIsDampingForceVectorDirty = true;

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->mIsBodyJacobianDirty = true;
    (*it)->mIsBodyJacobianDerivDirty = true;
  }
}

//==============================================================================
void Skeleton::computeInverseDynamicsRecursionB(bool _withExternalForces,
                                                bool _withDampingForces)
{
  // Skip immobile or 0-dof skeleton
  if (getDof() == 0)
    return;

  // Backward recursion
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it)
  {
    (*it)->updateBodyWrench(mGravity, _withExternalForces);
    (*it)->updateGeneralizedForce(_withDampingForces);
  }
}

//==============================================================================
//void Skeleton::computeHybridDynamics()
//{
//  dterr << "Not implemented yet.\n";
//}

//==============================================================================
//void Skeleton::computeHybridDynamicsRecursionA()
//{
//  dterr << "Not implemented yet.\n";
//}

//==============================================================================
//void Skeleton::computeHybridDynamicsRecursionB()
//{
//  dterr << "Not implemented yet.\n";
//}

//==============================================================================
void Skeleton::clearExternalForces()
{
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->clearExternalForces();
  }
}

//==============================================================================
void Skeleton::clearConstraintImpulses()
{
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->clearConstraintImpulse();
  }
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode)
{
  // Assertions
  assert(_bodyNode != NULL);
  assert(getDof() > 0);

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
  assert(getDof() > 0);

  // This skeleton should contain _bodyNode
  assert(std::find(mBodyNodes.begin(), mBodyNodes.end(), _bodyNode)
         != mBodyNodes.end());

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (size_t i = 0; i < mBodyNodes.size(); ++i)
    assert(mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Set impulse to _bodyNode
//  Eigen::Vector6d oldConstraintImpulse =_bodyNode->mConstraintImpulse;
  _bodyNode->mConstraintImpulse = _imp;

  // Prepare cache data
  BodyNode* it = _bodyNode;
  while (it != NULL)
  {
    it->updateBiasImpulse();
    it = it->getParentBodyNode();
  }

  // TODO(JS): Do we need to backup and restore the original value?
//  _bodyNode->mConstraintImpulse = oldConstraintImpulse;
  _bodyNode->mConstraintImpulse.setZero();
}

//==============================================================================
void Skeleton::updateBiasImpulse(SoftBodyNode* _softBodyNode,
                                 PointMass* _pointMass,
                                 const Eigen::Vector3d& _imp)
{
  // Assertions
  assert(_softBodyNode != NULL);
  assert(getDof() > 0);

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
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->updateJointVelocityChange();
  }
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
  if (!isMobile() || getDof() == 0)
    return;

  // Backward recursion
  if (mIsArticulatedInertiaDirty)
  {
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it)
    {
      (*it)->updateArtInertia(mTimeStep);
      (*it)->updateBiasImpulse();
    }

    mIsArticulatedInertiaDirty = false;
  }
  else
  {
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it)
    {
      (*it)->updateBiasImpulse();
    }
  }

  // Forward recursion
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    (*it)->updateJointVelocityChange();
//    (*it)->updateBodyVelocityChange();
    (*it)->updateBodyImpForceFwdDyn();
  }

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    // 1. dq = dq + del_dq
    // 2. ddq = ddq + del_dq / dt
    // 3. tau = tau + imp / dt
    (*it)->updateConstrainedJointAndBodyAcceleration(mTimeStep);

    // 4. F(+) = F(-) + ImpF / dt
    (*it)->updateConstrainedTransmittedForce(mTimeStep);
  }
}

//==============================================================================
void Skeleton::setConstraintForceVector(const Eigen::VectorXd& _Fc)
{
  mFc = _Fc;
}

//==============================================================================
Eigen::Vector3d Skeleton::getWorldCOM()
{
  // COM
  Eigen::Vector3d com(0.0, 0.0, 0.0);

  // Compute sum of each body's COM multiplied by body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++)
  {
    BodyNode* bodyNode = getBodyNode(i);
    com += bodyNode->getMass() * bodyNode->getWorldCOM();
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return com / mTotalMass;
}

//==============================================================================
Eigen::Vector3d Skeleton::getWorldCOMVelocity()
{
  // Velocity of COM
  Eigen::Vector3d comVel(0.0, 0.0, 0.0);

  // Compute sum of each body's COM velocities multiplied by body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++)
  {
    BodyNode* bodyNode = getBodyNode(i);
    comVel += bodyNode->getMass() * bodyNode->getWorldCOMVelocity();
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return comVel / mTotalMass;
}

//==============================================================================
Eigen::Vector3d Skeleton::getWorldCOMAcceleration()
{
  // Acceleration of COM
  Eigen::Vector3d comAcc(0.0, 0.0, 0.0);

  // Compute sum of each body's COM accelerations multiplied by body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++) {
    BodyNode* bodyNode = getBodyNode(i);
    comAcc += bodyNode->getMass() * bodyNode->getWorldCOMAcceleration();
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return comAcc / mTotalMass;
}

//==============================================================================
Eigen::MatrixXd Skeleton::getWorldCOMJacobian()
{
  // Jacobian of COM
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, getDof());

  // Compute sum of each body's Jacobian of COM accelerations multiplied by
  // body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++)
  {
    // BodyNode iterator
    BodyNode* bodyNode = getBodyNode(i);

    // Compute weighted Jacobian
    Eigen::MatrixXd localJ
        = bodyNode->getMass()
          * bodyNode->getWorldLinearJacobian(bodyNode->getLocalCOM());

    // Assign the weighted Jacobian to total Jacobian
    for (size_t j = 0; j < bodyNode->getNumDependentGenCoords(); ++j)
    {
      int idx = bodyNode->getDependentGenCoordIndex(j);
      J.col(idx) += localJ.col(j);
    }
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return J / mTotalMass;
}

//==============================================================================
Eigen::MatrixXd Skeleton::getWorldCOMJacobianTimeDeriv()
{
  // Jacobian time derivative of COM
  Eigen::MatrixXd dJ = Eigen::MatrixXd::Zero(3, getDof());

  // Compute sum of each body's Jacobian time derivative of COM accelerations
  // multiplied by body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++)
  {
    // BodyNode iterator
    BodyNode* bodyNode = getBodyNode(i);

    // Compute weighted Jacobian time derivative
    Eigen::MatrixXd localJ
        = bodyNode->getMass()
          * bodyNode->getWorldLinearJacobianDeriv(bodyNode->getLocalCOM());

    // Assign the weighted Jacobian to total Jacobian time derivative
    for (size_t j = 0; j < bodyNode->getNumDependentGenCoords(); ++j)
    {
      int idx = bodyNode->getDependentGenCoordIndex(j);
      dJ.col(idx) += localJ.col(j);
    }
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return dJ / mTotalMass;
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
