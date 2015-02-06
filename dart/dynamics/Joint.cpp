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

#include "dart/dynamics/Joint.h"

#include <string>

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/renderer/RenderInterface.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
const Joint::ActuatorType Joint::DefaultActuatorType = Joint::FORCE;

//==============================================================================
Joint::Joint(const std::string& _name)
  : mName(_name),
    mActuatorType(FORCE),
    mChildBodyNode(NULL),
    mSkeleton(NULL),
    mT_ParentBodyToJoint(Eigen::Isometry3d::Identity()),
    mT_ChildBodyToJoint(Eigen::Isometry3d::Identity()),
    mT(Eigen::Isometry3d::Identity()),
    mSpatialVelocity(Eigen::Vector6d::Zero()),
    mNeedTransformUpdate(true),
    mNeedSpatialVelocityUpdate(true),
    mNeedSpatialAccelerationUpdate(true),
    mNeedPrimaryAccelerationUpdate(true),
    mIsLocalJacobianDirty(true),
    mIsLocalJacobianTimeDerivDirty(true),
    mIsPositionLimited(true)
{
}

//==============================================================================
Joint::~Joint()
{
}

//==============================================================================
const std::string& Joint::setName(const std::string& _name, bool _renameDofs)
{
  if (mName == _name)
  {
    if (_renameDofs)
      updateDegreeOfFreedomNames();
    return mName;
  }

  if (mSkeleton)
  {
    mSkeleton->mNameMgrForJoints.removeName(mName);
    mName = _name;
    mSkeleton->addEntryToJointNameMgr(this);
  }
  else
  {
    mName = _name;
  }

  if (_renameDofs)
    updateDegreeOfFreedomNames();

  return mName;
}

//==============================================================================
const std::string& Joint::getName() const
{
  return mName;
}

//==============================================================================
void Joint::setActuatorType(Joint::ActuatorType _actuatorType)
{
  mActuatorType = _actuatorType;
}

//==============================================================================
Joint::ActuatorType Joint::getActuatorType() const
{
  return mActuatorType;
}

//==============================================================================
bool Joint::isKinematic() const
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      return false;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      return true;
    default:
    {
      dterr << "Unsupported actuator type." << std::endl;
      return false;
    }
  }
}

//==============================================================================
bool Joint::isDynamic() const
{
  return !isKinematic();
}

//==============================================================================
BodyNode* Joint::getChildBodyNode()
{
  return mChildBodyNode;
}

//==============================================================================
const BodyNode* Joint::getChildBodyNode() const
{
  return mChildBodyNode;
}

//==============================================================================
BodyNode* Joint::getParentBodyNode()
{
  if (mChildBodyNode)
    return mChildBodyNode->getParentBodyNode();

  return NULL;
}

//==============================================================================
const BodyNode* Joint::getParentBodyNode() const
{
  return const_cast<Joint*>(this)->getParentBodyNode();
}

//==============================================================================
Skeleton* Joint::getSkeleton()
{
  return mSkeleton;
}

//==============================================================================
const Skeleton* Joint::getSkeleton() const
{
  return mSkeleton;
}

//==============================================================================
const Eigen::Isometry3d& Joint::getLocalTransform() const
{
  if(mNeedTransformUpdate)
  {
    updateLocalTransform();
    mNeedTransformUpdate = false;
  }

  return mT;
}

//==============================================================================
const Eigen::Vector6d& Joint::getLocalSpatialVelocity() const
{
  if(mNeedSpatialVelocityUpdate)
  {
    updateLocalSpatialVelocity();
    mNeedSpatialVelocityUpdate = false;
  }

  return mSpatialVelocity;
}

//==============================================================================
const Eigen::Vector6d& Joint::getLocalSpatialAcceleration() const
{
  if(mNeedSpatialAccelerationUpdate)
  {
    updateLocalSpatialAcceleration();
    mNeedSpatialAccelerationUpdate = false;
  }

  return mSpatialAcceleration;
}

//==============================================================================
const Eigen::Vector6d& Joint::getLocalPrimaryAcceleration() const
{
  if(mNeedPrimaryAccelerationUpdate)
  {
    updateLocalPrimaryAcceleration();
    mNeedPrimaryAccelerationUpdate = false;
  }

  return mPrimaryAcceleration;
}

//==============================================================================
//bool Joint::contains(const GenCoord* _genCoord) const {
//  return find(mGenCoords.begin(), mGenCoords.end(), _genCoord) !=
//      mGenCoords.end() ? true : false;
//}

//==============================================================================
//int Joint::getGenCoordLocalIndex(int _dofSkelIndex) const
//{
//  for (unsigned int i = 0; i < mGenCoords.size(); i++)
//    if (mGenCoords[i]->getIndexInSkeleton() == _dofSkelIndex)
//      return i;
//  return -1;
//}

//==============================================================================
void Joint::setPositionLimited(bool _isPositionLimited)
{
  mIsPositionLimited = _isPositionLimited;
}

//==============================================================================
bool Joint::isPositionLimited() const
{
  return mIsPositionLimited;
}

//==============================================================================
void Joint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
{
  assert(math::verifyTransform(_T));
  mT_ParentBodyToJoint = _T;
  notifyPositionUpdate();
}

//==============================================================================
void Joint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  assert(math::verifyTransform(_T));
  mT_ChildBodyToJoint = _T;
  updateLocalJacobian();
  notifyPositionUpdate();
}

//==============================================================================
const Eigen::Isometry3d&Joint::getTransformFromParentBodyNode() const
{
  return mT_ParentBodyToJoint;
}

//==============================================================================
const Eigen::Isometry3d&Joint::getTransformFromChildBodyNode() const
{
  return mT_ChildBodyToJoint;
}

//==============================================================================
void Joint::applyGLTransform(renderer::RenderInterface* _ri)
{
  _ri->transform(getLocalTransform());
}

//==============================================================================
void Joint::init(Skeleton* _skel)
{
  mSkeleton = _skel;
}

//==============================================================================
DegreeOfFreedom* Joint::createDofPointer(const std::string &_name,
                                         size_t _indexInJoint)
{
  return new DegreeOfFreedom(this, _name, _indexInJoint);
}

//==============================================================================
void Joint::updateArticulatedInertia() const
{
  if(mSkeleton && mSkeleton->mIsArticulatedInertiaDirty)
      mSkeleton->updateArticulatedInertia();
}

//==============================================================================
//Eigen::VectorXd Joint::getDampingForces() const
//{
//  int numDofs = getNumDofs();
//  Eigen::VectorXd dampingForce(numDofs);

//  for (int i = 0; i < numDofs; ++i)
//    dampingForce(i) = -mDampingCoefficient[i] * getGenCoord(i)->getVel();

//  return dampingForce;
//}

//==============================================================================
//Eigen::VectorXd Joint::getSpringForces(double _timeStep) const
//{
//  int dof = getNumDofs();
//  Eigen::VectorXd springForce(dof);
//  for (int i = 0; i < dof; ++i)
//  {
//    springForce(i) =
//        -mSpringStiffness[i] * (getGenCoord(i)->getPos()
//                                + getGenCoord(i)->getVel() * _timeStep
//                                - mRestPosition[i]);
//  }
//  assert(!math::isNan(springForce));
//  return springForce;
//}

//==============================================================================
void Joint::notifyPositionUpdate()
{
  if(mChildBodyNode)
  {
    mChildBodyNode->notifyTransformUpdate();
    mChildBodyNode->mIsBodyJacobianDirty = true;
    mChildBodyNode->mIsWorldJacobianDirty = true;
    mChildBodyNode->mIsBodyJacobianSpatialDerivDirty = true;
    mChildBodyNode->mIsWorldJacobianClassicDerivDirty = true;
  }

  mIsLocalJacobianDirty = true;
  mIsLocalJacobianTimeDerivDirty = true;
  mNeedPrimaryAccelerationUpdate = true;

  mNeedTransformUpdate = true;
  mNeedSpatialVelocityUpdate = true;
  mNeedSpatialAccelerationUpdate = true;

  if(mSkeleton)
  {
    mSkeleton->notifyArticulatedInertiaUpdate();
    mSkeleton->mIsExternalForcesDirty = true;
  }
}

//==============================================================================
void Joint::notifyVelocityUpdate()
{
  if(mChildBodyNode)
    mChildBodyNode->notifyVelocityUpdate();

  mIsLocalJacobianTimeDerivDirty = true;

  mNeedSpatialVelocityUpdate = true;
  mNeedSpatialAccelerationUpdate = true;
}

//==============================================================================
void Joint::notifyAccelerationUpdate()
{
  if(mChildBodyNode)
    mChildBodyNode->notifyAccelerationUpdate();

  mNeedSpatialAccelerationUpdate = true;
  mNeedPrimaryAccelerationUpdate = true;
}

}  // namespace dynamics
}  // namespace dart
