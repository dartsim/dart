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

#include "kido/dynamics/Joint.h"

#include <string>

#include "kido/common/Console.h"
#include "kido/math/Helpers.h"
#include "kido/renderer/RenderInterface.h"
#include "kido/dynamics/BodyNode.h"
#include "kido/dynamics/DegreeOfFreedom.h"
#include "kido/dynamics/Skeleton.h"

namespace kido {
namespace dynamics {

//==============================================================================
const Joint::ActuatorType Joint::DefaultActuatorType = Joint::FORCE;

//==============================================================================
Joint::Properties::Properties(const std::string& _name,
                              const Eigen::Isometry3d& _T_ParentBodyToJoint,
                              const Eigen::Isometry3d& _T_ChildBodyToJoint,
                              bool _isPositionLimited,
                              ActuatorType _actuatorType)
  : mName(_name),
    mT_ParentBodyToJoint(_T_ParentBodyToJoint),
    mT_ChildBodyToJoint(_T_ChildBodyToJoint),
    mIsPositionLimited(_isPositionLimited),
    mActuatorType(_actuatorType)
{
  // Do nothing
}

//==============================================================================
Joint::~Joint()
{
  // Do nothing
}

//==============================================================================
void Joint::setProperties(const Properties& _properties)
{
  setName(_properties.mName);
  setTransformFromParentBodyNode(_properties.mT_ParentBodyToJoint);
  setTransformFromChildBodyNode(_properties.mT_ChildBodyToJoint);
  setPositionLimitEnforced(_properties.mIsPositionLimited);
  setActuatorType(_properties.mActuatorType);
}

//==============================================================================
const Joint::Properties& Joint::getJointProperties() const
{
  return mJointP;
}

//==============================================================================
void Joint::copy(const Joint& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getJointProperties());
}

//==============================================================================
void Joint::copy(const Joint* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*_otherJoint);
}

//==============================================================================
Joint& Joint::operator=(const Joint& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
const std::string& Joint::setName(const std::string& _name, bool _renameDofs)
{
  if (mJointP.mName == _name)
  {
    if (_renameDofs)
      updateDegreeOfFreedomNames();
    return mJointP.mName;
  }

  const SkeletonPtr& skel = mChildBodyNode?
        mChildBodyNode->getSkeleton() : nullptr;
  if (skel)
  {
    skel->mNameMgrForJoints.removeName(mJointP.mName);
    mJointP.mName = _name;

    skel->addEntryToJointNameMgr(this, _renameDofs);
  }
  else
  {
    mJointP.mName = _name;

    if (_renameDofs)
      updateDegreeOfFreedomNames();
  }

  return mJointP.mName;
}

//==============================================================================
const std::string& Joint::getName() const
{
  return mJointP.mName;
}

//==============================================================================
void Joint::setActuatorType(Joint::ActuatorType _actuatorType)
{
  mJointP.mActuatorType = _actuatorType;
}

//==============================================================================
Joint::ActuatorType Joint::getActuatorType() const
{
  return mJointP.mActuatorType;
}

//==============================================================================
bool Joint::isKinematic() const
{
  switch (mJointP.mActuatorType)
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

  return nullptr;
}

//==============================================================================
const BodyNode* Joint::getParentBodyNode() const
{
  return const_cast<Joint*>(this)->getParentBodyNode();
}

//==============================================================================
SkeletonPtr Joint::getSkeleton()
{
  return mChildBodyNode? mChildBodyNode->getSkeleton() : nullptr;
}

//==============================================================================
std::shared_ptr<const Skeleton> Joint::getSkeleton() const
{
  return mChildBodyNode? mChildBodyNode->getSkeleton() : nullptr;
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
void Joint::setPositionLimitEnforced(bool _isPositionLimited)
{
  mJointP.mIsPositionLimited = _isPositionLimited;
}

//==============================================================================
void Joint::setPositionLimited(bool _isPositionLimited)
{
  setPositionLimitEnforced(_isPositionLimited);
}

//==============================================================================
bool Joint::isPositionLimitEnforced() const
{
  return mJointP.mIsPositionLimited;
}

//==============================================================================
bool Joint::isPositionLimited() const
{
  return isPositionLimitEnforced();
}

//==============================================================================
size_t Joint::getJointIndexInSkeleton() const
{
  return mChildBodyNode->getIndexInSkeleton();
}

//==============================================================================
size_t Joint::getJointIndexInTree() const
{
  return mChildBodyNode->getIndexInTree();
}

//==============================================================================
size_t Joint::getTreeIndex() const
{
  return mChildBodyNode->getTreeIndex();
}

//==============================================================================
bool Joint::checkSanity(bool _printWarnings) const
{
  bool sane = true;
  for(size_t i=0; i < getNumDofs(); ++i)
  {
    if(getInitialPosition(i) < getPositionLowerLimit(i)
       || getPositionUpperLimit(i) < getInitialPosition(i))
    {
      if(_printWarnings)
      {
        dtwarn << "[Joint::checkSanity] Initial position of index " << i << " ["
               << getDofName(i) << "] in Joint [" << getName() << "] is "
               << "outside of its position limits\n"
               << " -- Initial Position: " << getInitialPosition(i) << "\n"
               << " -- Limits: [" << getPositionLowerLimit(i) << ", "
               << getPositionUpperLimit(i) << "]\n";
      }
      else
      {
        return false;
      }

      sane = false;
    }

    if(getInitialVelocity(i) < getVelocityLowerLimit(i)
       || getVelocityUpperLimit(i) < getInitialVelocity(i))
    {
      if(_printWarnings)
      {
        dtwarn << "[Joint::checkSanity] Initial velocity of index " << i << " ["
               << getDofName(i) << "] is Joint [" << getName() << "] is "
               << "outside of its velocity limits\n"
               << " -- Initial Velocity: " << getInitialVelocity(i) << "\n"
               << " -- Limits: [" << getVelocityLowerLimit(i) << ", "
               << getVelocityUpperLimit(i) << "]\n";
      }
      else
      {
        return false;
      }

      sane = false;
    }
  }

  return sane;
}

//==============================================================================
void Joint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
{
  assert(math::verifyTransform(_T));
  mJointP.mT_ParentBodyToJoint = _T;
  notifyPositionUpdate();
}

//==============================================================================
void Joint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  assert(math::verifyTransform(_T));
  mJointP.mT_ChildBodyToJoint = _T;
  updateLocalJacobian();
  notifyPositionUpdate();
}

//==============================================================================
const Eigen::Isometry3d& Joint::getTransformFromParentBodyNode() const
{
  return mJointP.mT_ParentBodyToJoint;
}

//==============================================================================
const Eigen::Isometry3d& Joint::getTransformFromChildBodyNode() const
{
  return mJointP.mT_ChildBodyToJoint;
}

//==============================================================================
void Joint::applyGLTransform(renderer::RenderInterface* _ri)
{
  _ri->transform(getLocalTransform());
}

//==============================================================================
Joint::Joint(const Properties& _properties)
  : mJointP(_properties),
    mChildBodyNode(nullptr),
    mT(Eigen::Isometry3d::Identity()),
    mSpatialVelocity(Eigen::Vector6d::Zero()),
    mSpatialAcceleration(Eigen::Vector6d::Zero()),
    mPrimaryAcceleration(Eigen::Vector6d::Zero()),
    mNeedTransformUpdate(true),
    mNeedSpatialVelocityUpdate(true),
    mNeedSpatialAccelerationUpdate(true),
    mNeedPrimaryAccelerationUpdate(true),
    mIsLocalJacobianDirty(true),
    mIsLocalJacobianTimeDerivDirty(true)
{
  // Do nothing
}

//==============================================================================
DegreeOfFreedom* Joint::createDofPointer(size_t _indexInJoint)
{
  return new DegreeOfFreedom(this, _indexInJoint);
}

//==============================================================================
void Joint::updateArticulatedInertia() const
{
  mChildBodyNode->getArticulatedInertia();
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
    mChildBodyNode->notifyJacobianUpdate();
    mChildBodyNode->notifyJacobianDerivUpdate();
  }

  mIsLocalJacobianDirty = true;
  mIsLocalJacobianTimeDerivDirty = true;
  mNeedPrimaryAccelerationUpdate = true;

  mNeedTransformUpdate = true;
  mNeedSpatialVelocityUpdate = true;
  mNeedSpatialAccelerationUpdate = true;

  SkeletonPtr skel = getSkeleton();
  if(skel)
  {
    size_t tree = mChildBodyNode->mTreeIndex;
    skel->notifyArticulatedInertiaUpdate(tree);
    skel->mTreeCache[tree].mDirty.mExternalForces = true;
    skel->mSkelCache.mDirty.mExternalForces = true;
  }
}

//==============================================================================
void Joint::notifyVelocityUpdate()
{
  if(mChildBodyNode)
  {
    mChildBodyNode->notifyVelocityUpdate();
    mChildBodyNode->notifyJacobianDerivUpdate();
  }

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
}  // namespace kido
