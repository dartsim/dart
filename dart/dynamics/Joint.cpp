/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/dynamics/Joint.hpp"

#include <string>

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/Helpers.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
const Joint::ActuatorType Joint::DefaultActuatorType
    = detail::DefaultActuatorType;
// These declarations are needed for linking to work
constexpr Joint::ActuatorType Joint::FORCE;
constexpr Joint::ActuatorType Joint::PASSIVE;
constexpr Joint::ActuatorType Joint::SERVO;
constexpr Joint::ActuatorType Joint::MIMIC;
constexpr Joint::ActuatorType Joint::ACCELERATION;
constexpr Joint::ActuatorType Joint::VELOCITY;
constexpr Joint::ActuatorType Joint::LOCKED;

namespace detail {

//==============================================================================
JointProperties::JointProperties(
    const std::string& _name,
    const Eigen::Isometry3d& _T_ParentBodyToJoint,
    const Eigen::Isometry3d& _T_ChildBodyToJoint,
    bool _isPositionLimitEnforced,
    ActuatorType _actuatorType,
    const Joint* _mimicJoint,
    double _mimicMultiplier,
    double _mimicOffset)
  : mName(_name),
    mT_ParentBodyToJoint(_T_ParentBodyToJoint),
    mT_ChildBodyToJoint(_T_ChildBodyToJoint),
    mIsPositionLimitEnforced(_isPositionLimitEnforced),
    mActuatorType(_actuatorType),
    mMimicJoint(_mimicJoint),
    mMimicMultiplier(_mimicMultiplier),
    mMimicOffset(_mimicOffset)
{
  // Do nothing
}

} // namespace detail

//==============================================================================
Joint::ExtendedProperties::ExtendedProperties(
    const Properties& standardProperties,
    const CompositeProperties& aspectProperties)
  : Properties(standardProperties), mCompositeProperties(aspectProperties)
{
  // Do nothing
}

//==============================================================================
Joint::ExtendedProperties::ExtendedProperties(
    Properties&& standardProperties, CompositeProperties&& aspectProperties)
  : Properties(std::move(standardProperties)),
    mCompositeProperties(std::move(aspectProperties))
{
  // Do nothing
}

//==============================================================================
Joint::~Joint()
{
  // Do nothing
}

//==============================================================================
void Joint::setProperties(const Properties& properties)
{
  setAspectProperties(properties);
}

//==============================================================================
void Joint::setAspectProperties(const AspectProperties& properties)
{
  setName(properties.mName);
  setTransformFromParentBodyNode(properties.mT_ParentBodyToJoint);
  setTransformFromChildBodyNode(properties.mT_ChildBodyToJoint);
  setLimitEnforcement(properties.mIsPositionLimitEnforced);
  setActuatorType(properties.mActuatorType);
  setMimicJoint(
      properties.mMimicJoint,
      properties.mMimicMultiplier,
      properties.mMimicOffset);
}

//==============================================================================
const Joint::Properties& Joint::getJointProperties() const
{
  return mAspectProperties;
}

//==============================================================================
void Joint::copy(const Joint& _otherJoint)
{
  if (this == &_otherJoint)
    return;

  setProperties(_otherJoint.getJointProperties());
}

//==============================================================================
void Joint::copy(const Joint* _otherJoint)
{
  if (nullptr == _otherJoint)
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
  if (mAspectProperties.mName == _name)
  {
    if (_renameDofs)
      updateDegreeOfFreedomNames();
    return mAspectProperties.mName;
  }

  const SkeletonPtr& skel
      = mChildBodyNode ? mChildBodyNode->getSkeleton() : nullptr;
  if (skel)
  {
    skel->mNameMgrForJoints.removeName(mAspectProperties.mName);
    mAspectProperties.mName = _name;

    skel->addEntryToJointNameMgr(this, _renameDofs);
  }
  else
  {
    mAspectProperties.mName = _name;

    if (_renameDofs)
      updateDegreeOfFreedomNames();
  }

  return mAspectProperties.mName;
}

//==============================================================================
const std::string& Joint::getName() const
{
  return mAspectProperties.mName;
}

//==============================================================================
void Joint::setActuatorType(Joint::ActuatorType _actuatorType)
{
  mAspectProperties.mActuatorType = _actuatorType;
}

//==============================================================================
Joint::ActuatorType Joint::getActuatorType() const
{
  return mAspectProperties.mActuatorType;
}

//==============================================================================
void Joint::setMimicJoint(
    const Joint* _mimicJoint, double _mimicMultiplier, double _mimicOffset)
{
  mAspectProperties.mMimicJoint = _mimicJoint;
  mAspectProperties.mMimicMultiplier = _mimicMultiplier;
  mAspectProperties.mMimicOffset = _mimicOffset;
}

//==============================================================================
const Joint* Joint::getMimicJoint() const
{
  return mAspectProperties.mMimicJoint;
}

//==============================================================================
double Joint::getMimicMultiplier() const
{
  return mAspectProperties.mMimicMultiplier;
}

//==============================================================================
double Joint::getMimicOffset() const
{
  return mAspectProperties.mMimicOffset;
}

//==============================================================================
bool Joint::isKinematic() const
{
  switch (mAspectProperties.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
    case MIMIC:
      return false;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      return true;
    default: {
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
  return mChildBodyNode ? mChildBodyNode->getSkeleton() : nullptr;
}

//==============================================================================
std::shared_ptr<const Skeleton> Joint::getSkeleton() const
{
  return mChildBodyNode ? mChildBodyNode->getSkeleton() : nullptr;
}

//==============================================================================
const Eigen::Isometry3d& Joint::getLocalTransform() const
{
  return getRelativeTransform();
}

//==============================================================================
const Eigen::Vector6d& Joint::getLocalSpatialVelocity() const
{
  return getRelativeSpatialVelocity();
}

//==============================================================================
const Eigen::Vector6d& Joint::getLocalSpatialAcceleration() const
{
  return getRelativeSpatialAcceleration();
}

//==============================================================================
const Eigen::Vector6d& Joint::getLocalPrimaryAcceleration() const
{
  return getRelativePrimaryAcceleration();
}

//==============================================================================
const math::Jacobian Joint::getLocalJacobian() const
{
  return getRelativeJacobian();
}

//==============================================================================
math::Jacobian Joint::getLocalJacobian(const Eigen::VectorXd& positions) const
{
  return getRelativeJacobian(positions);
}

//==============================================================================
const math::Jacobian Joint::getLocalJacobianTimeDeriv() const
{
  return getRelativeJacobianTimeDeriv();
}

//==============================================================================
const Eigen::Isometry3d& Joint::getRelativeTransform() const
{
  if (mNeedTransformUpdate)
  {
    updateRelativeTransform();
    mNeedTransformUpdate = false;
  }

  return mT;
}

//==============================================================================
const Eigen::Vector6d& Joint::getRelativeSpatialVelocity() const
{
  if (mNeedSpatialVelocityUpdate)
  {
    updateRelativeSpatialVelocity();
    mNeedSpatialVelocityUpdate = false;
  }

  return mSpatialVelocity;
}

//==============================================================================
const Eigen::Vector6d& Joint::getRelativeSpatialAcceleration() const
{
  if (mNeedSpatialAccelerationUpdate)
  {
    updateRelativeSpatialAcceleration();
    mNeedSpatialAccelerationUpdate = false;
  }

  return mSpatialAcceleration;
}

//==============================================================================
const Eigen::Vector6d& Joint::getRelativePrimaryAcceleration() const
{
  if (mNeedPrimaryAccelerationUpdate)
  {
    updateRelativePrimaryAcceleration();
    mNeedPrimaryAccelerationUpdate = false;
  }

  return mPrimaryAcceleration;
}

//==============================================================================
Eigen::Vector6d Joint::getWrenchToChildBodyNode(
    const Frame* withRespectTo) const
{
  const BodyNode* childBodyNode = getChildBodyNode();
  if (!childBodyNode)
  {
    return Eigen::Vector6d::Zero();
  }

  const Eigen::Vector6d& F2 = childBodyNode->getBodyForce();
  const BodyNode* parentBodyNode = getParentBodyNode();

  if (withRespectTo == nullptr)
  {
    // (Default) Wrench applying to the child body node, where the reference
    // frame is the joint frame
    return math::dAdT(getTransformFromChildBodyNode(), -F2);
  }
  else if (withRespectTo == childBodyNode)
  {
    // Wrench applying to the child body node, where the reference frame is the
    // child body frame
    return -F2;
  }
  else if (withRespectTo == parentBodyNode)
  {
    // Wrench applying to the child body node, where the reference frame is the
    // parent body frame
    return math::dAdInvT(getRelativeTransform(), -F2);
  }
  else
  {
    // Wrench applying to the child body node, where the reference frame is an
    // arbitrary frame
    return math::dAdT(withRespectTo->getTransform(childBodyNode), -F2);
  }
}

//==============================================================================
Eigen::Vector6d Joint::getWrenchToParentBodyNode(
    const Frame* withRespectTo) const
{
  return -getWrenchToChildBodyNode(withRespectTo);
}

//==============================================================================
void Joint::setPositionLimitEnforced(bool enforced)
{
  setLimitEnforcement(enforced);
}

//==============================================================================
void Joint::setLimitEnforcement(bool enforced)
{
  mAspectProperties.mIsPositionLimitEnforced = enforced;
}

//==============================================================================
bool Joint::isPositionLimitEnforced() const
{
  return areLimitsEnforced();
}

//==============================================================================
bool Joint::areLimitsEnforced() const
{
  return mAspectProperties.mIsPositionLimitEnforced;
}

//==============================================================================
std::size_t Joint::getJointIndexInSkeleton() const
{
  return mChildBodyNode->getIndexInSkeleton();
}

//==============================================================================
std::size_t Joint::getJointIndexInTree() const
{
  return mChildBodyNode->getIndexInTree();
}

//==============================================================================
std::size_t Joint::getTreeIndex() const
{
  return mChildBodyNode->getTreeIndex();
}

//==============================================================================
bool Joint::checkSanity(bool _printWarnings) const
{
  bool sane = true;
  for (std::size_t i = 0; i < getNumDofs(); ++i)
  {
    if (getInitialPosition(i) < getPositionLowerLimit(i)
        || getPositionUpperLimit(i) < getInitialPosition(i))
    {
      if (_printWarnings)
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

    if (getInitialVelocity(i) < getVelocityLowerLimit(i)
        || getVelocityUpperLimit(i) < getInitialVelocity(i))
    {
      if (_printWarnings)
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
double Joint::getPotentialEnergy() const
{
  return computePotentialEnergy();
}

//==============================================================================
void Joint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
{
  assert(math::verifyTransform(_T));
  mAspectProperties.mT_ParentBodyToJoint = _T;
  notifyPositionUpdated();
}

//==============================================================================
void Joint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  assert(math::verifyTransform(_T));
  mAspectProperties.mT_ChildBodyToJoint = _T;
  updateRelativeJacobian();
  notifyPositionUpdated();
}

//==============================================================================
const Eigen::Isometry3d& Joint::getTransformFromParentBodyNode() const
{
  return mAspectProperties.mT_ParentBodyToJoint;
}

//==============================================================================
const Eigen::Isometry3d& Joint::getTransformFromChildBodyNode() const
{
  return mAspectProperties.mT_ChildBodyToJoint;
}

//==============================================================================
Joint::Joint()
  : mChildBodyNode(nullptr),
    mT(Eigen::Isometry3d::Identity()),
    mSpatialVelocity(Eigen::Vector6d::Zero()),
    mSpatialAcceleration(Eigen::Vector6d::Zero()),
    mPrimaryAcceleration(Eigen::Vector6d::Zero()),
    mNeedTransformUpdate(true),
    mNeedSpatialVelocityUpdate(true),
    mNeedSpatialAccelerationUpdate(true),
    mNeedPrimaryAccelerationUpdate(true),
    mIsRelativeJacobianDirty(true),
    mIsRelativeJacobianTimeDerivDirty(true)
{
  // Do nothing. The Joint::Aspect must be created by a derived class.
}

//==============================================================================
DegreeOfFreedom* Joint::createDofPointer(std::size_t _indexInJoint)
{
  return new DegreeOfFreedom(this, _indexInJoint);
}

//==============================================================================
void Joint::updateLocalTransform() const
{
  updateRelativeTransform();
}

//==============================================================================
void Joint::updateLocalSpatialVelocity() const
{
  updateRelativeSpatialVelocity();
}

//==============================================================================
void Joint::updateLocalSpatialAcceleration() const
{
  updateRelativeSpatialAcceleration();
}

//==============================================================================
void Joint::updateLocalPrimaryAcceleration() const
{
  updateRelativePrimaryAcceleration();
}

//==============================================================================
void Joint::updateLocalJacobian(bool mandatory) const
{
  updateRelativeJacobian(mandatory);
}

//==============================================================================
void Joint::updateLocalJacobianTimeDeriv() const
{
  updateRelativeJacobianTimeDeriv();
}

//==============================================================================
void Joint::updateArticulatedInertia() const
{
  mChildBodyNode->getArticulatedInertia();
}

//==============================================================================
// Eigen::VectorXd Joint::getDampingForces() const
//{
//  int numDofs = getNumDofs();
//  Eigen::VectorXd dampingForce(numDofs);

//  for (int i = 0; i < numDofs; ++i)
//    dampingForce(i) = -mDampingCoefficient[i] * getGenCoord(i)->getVel();

//  return dampingForce;
//}

//==============================================================================
// Eigen::VectorXd Joint::getSpringForces(double _timeStep) const
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
  notifyPositionUpdated();
}

//==============================================================================
void Joint::notifyPositionUpdated()
{
  if (mChildBodyNode)
  {
    mChildBodyNode->dirtyTransform();
    mChildBodyNode->dirtyJacobian();
    mChildBodyNode->dirtyJacobianDeriv();
  }

  mIsRelativeJacobianDirty = true;
  mIsRelativeJacobianTimeDerivDirty = true;
  mNeedPrimaryAccelerationUpdate = true;

  mNeedTransformUpdate = true;
  mNeedSpatialVelocityUpdate = true;
  mNeedSpatialAccelerationUpdate = true;

  SkeletonPtr skel = getSkeleton();
  if (skel)
  {
    std::size_t tree = mChildBodyNode->mTreeIndex;
    skel->dirtyArticulatedInertia(tree);
    skel->mTreeCache[tree].mDirty.mExternalForces = true;
    skel->mSkelCache.mDirty.mExternalForces = true;
  }
}

//==============================================================================
void Joint::notifyVelocityUpdate()
{
  notifyVelocityUpdated();
}

//==============================================================================
void Joint::notifyVelocityUpdated()
{
  if (mChildBodyNode)
  {
    mChildBodyNode->dirtyVelocity();
    mChildBodyNode->dirtyJacobianDeriv();
  }

  mIsRelativeJacobianTimeDerivDirty = true;

  mNeedSpatialVelocityUpdate = true;
  mNeedSpatialAccelerationUpdate = true;
}

//==============================================================================
void Joint::notifyAccelerationUpdate()
{
  notifyAccelerationUpdated();
}

//==============================================================================
void Joint::notifyAccelerationUpdated()
{
  if (mChildBodyNode)
    mChildBodyNode->dirtyAcceleration();

  mNeedSpatialAccelerationUpdate = true;
  mNeedPrimaryAccelerationUpdate = true;
}

} // namespace dynamics
} // namespace dart
