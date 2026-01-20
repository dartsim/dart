/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/dynamics/joint.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/degree_of_freedom.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/helpers.hpp"

#include <string>

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
    mActuatorType(_actuatorType)
{
  mMimicDofProps.resize(6);
  // TODO: Dof 6, which is the max value at the moment, is used because
  // JointProperties doesn't have the Dof information

  for (auto i = 0u; i < mMimicDofProps.size(); ++i) {
    auto& prop = mMimicDofProps[i];
    prop.mReferenceJoint = _mimicJoint;
    prop.mReferenceDofIndex = i;
    prop.mMultiplier = _mimicMultiplier;
    prop.mOffset = _mimicOffset;
    prop.mConstraintType = MimicConstraintType::Motor;
  }
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
  for (const auto& [index, actuatorType] : properties.mActuatorTypes)
    setActuatorType(index, actuatorType);
  setMimicJointDofs(
      std::span<const MimicDofProperties>(properties.mMimicDofProps));
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
  if (mAspectProperties.mName == _name) {
    if (_renameDofs)
      updateDegreeOfFreedomNames();
    return mAspectProperties.mName;
  }

  const SkeletonPtr& skel
      = mChildBodyNode ? mChildBodyNode->getSkeleton() : nullptr;
  if (skel) {
    skel->mNameMgrForJoints.removeName(mAspectProperties.mName);
    mAspectProperties.mName = _name;

    skel->addEntryToJointNameMgr(this, _renameDofs);
  } else {
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
  if (mAspectProperties.mActuatorType == _actuatorType
      && mAspectProperties.mActuatorTypes.empty())
    return;

  mAspectProperties.mActuatorType = _actuatorType;
  mAspectProperties.mActuatorTypes.clear();
  resetCommands();
}

//==============================================================================
void Joint::setActuatorType(std::size_t index, ActuatorType actuatorType)
{
  if (index >= getNumDofs()) {
    DART_ERROR(
        "Attempted to set actuator type for invalid index {} on Joint [{}] "
        "with {} DoFs.",
        index,
        getName(),
        getNumDofs());
    return;
  }

  const bool defaultDynamic
      = isDynamicActuatorType(mAspectProperties.mActuatorType);
  const bool requestedDynamic = isDynamicActuatorType(actuatorType);
  if (requestedDynamic != defaultDynamic) {
    DART_ERROR(
        "Cannot assign actuator type {} to DoF {} of Joint [{}] because it "
        "does not match the dynamic/kinematic classification of the "
        "joint-wide actuator type {}.",
        static_cast<int>(actuatorType),
        index,
        getName(),
        static_cast<int>(mAspectProperties.mActuatorType));
    return;
  }

  if (actuatorType != mAspectProperties.mActuatorType
      && actuatorType != Joint::MIMIC) {
    DART_ERROR(
        "Only per-DoF overrides to Joint::MIMIC are supported. Requested "
        "actuator type {} for DoF {} on Joint [{}].",
        static_cast<int>(actuatorType),
        index,
        getName());
    return;
  }

  auto& overrides = mAspectProperties.mActuatorTypes;

  if (actuatorType == mAspectProperties.mActuatorType) {
    const auto it = overrides.find(index);
    if (it == overrides.end())
      return;

    overrides.erase(it);
    resetCommands();
    return;
  }

  const auto it = overrides.find(index);
  if (it != overrides.end() && it->second == actuatorType)
    return;

  overrides[index] = actuatorType;
  resetCommands();
}

//==============================================================================
void Joint::setActuatorTypes(std::span<const ActuatorType> actuatorTypes)
{
  if (actuatorTypes.size() != getNumDofs()) {
    DART_ERROR(
        "Actuator type vector size ({}) does not match the number of DoFs ({}) "
        "for Joint [{}].",
        actuatorTypes.size(),
        getNumDofs(),
        getName());
    return;
  }

  if (actuatorTypes.empty()) {
    if (!mAspectProperties.mActuatorTypes.empty()) {
      mAspectProperties.mActuatorTypes.clear();
      resetCommands();
    }
    return;
  }

  ActuatorType newDefault = actuatorTypes.front();
  std::map<std::size_t, ActuatorType> newOverrides;
  const bool newDefaultDynamic = isDynamicActuatorType(newDefault);

  for (std::size_t i = 1; i < actuatorTypes.size(); ++i) {
    const auto type = actuatorTypes[i];
    if (isDynamicActuatorType(type) != newDefaultDynamic) {
      DART_ERROR(
          "Actuator type {} for DoF {} of Joint [{}] does not match the "
          "dynamic/kinematic classification of the joint-wide actuator type "
          "{}.",
          static_cast<int>(type),
          i,
          getName(),
          static_cast<int>(newDefault));
      return;
    }
    if (type != newDefault) {
      if (type != Joint::MIMIC) {
        DART_ERROR(
            "Only per-DoF overrides to Joint::MIMIC are supported. Requested "
            "actuator type {} for DoF {} on Joint [{}].",
            static_cast<int>(type),
            i,
            getName());
        return;
      }
      newOverrides.emplace(i, type);
    }
  }

  if (mAspectProperties.mActuatorType == newDefault
      && mAspectProperties.mActuatorTypes == newOverrides)
    return;

  mAspectProperties.mActuatorType = newDefault;
  mAspectProperties.mActuatorTypes = std::move(newOverrides);
  resetCommands();
}

//==============================================================================
Joint::ActuatorType Joint::getActuatorType() const
{
  return mAspectProperties.mActuatorType;
}

//==============================================================================
Joint::ActuatorType Joint::getActuatorType(std::size_t index) const
{
  if (index >= getNumDofs()) {
    DART_ERROR(
        "Requested actuator type for invalid index {} on Joint [{}] with {} "
        "DoFs.",
        index,
        getName(),
        getNumDofs());
    return mAspectProperties.mActuatorType;
  }

  const auto it = mAspectProperties.mActuatorTypes.find(index);
  if (it != mAspectProperties.mActuatorTypes.end())
    return it->second;

  return mAspectProperties.mActuatorType;
}

//==============================================================================
std::vector<Joint::ActuatorType> Joint::getActuatorTypes() const
{
  const auto numDofs = getNumDofs();
  std::vector<ActuatorType> actuatorTypes(
      numDofs, mAspectProperties.mActuatorType);

  for (const auto& [index, type] : mAspectProperties.mActuatorTypes) {
    if (index < numDofs)
      actuatorTypes[index] = type;
  }

  return actuatorTypes;
}

//==============================================================================
bool Joint::hasActuatorType(ActuatorType actuatorType) const
{
  const auto numDofs = getNumDofs();
  for (std::size_t i = 0; i < numDofs; ++i) {
    if (getActuatorType(i) == actuatorType)
      return true;
  }
  return false;
}

//==============================================================================
bool Joint::isKinematicActuatorType(ActuatorType actuatorType)
{
  switch (actuatorType) {
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      return true;
    case FORCE:
    case PASSIVE:
    case SERVO:
    case MIMIC:
      return false;
    default:
      DART_ERROR(
          "Unsupported actuator type: {}.", static_cast<int>(actuatorType));
      return false;
  }
}

//==============================================================================
bool Joint::isDynamicActuatorType(ActuatorType actuatorType)
{
  switch (actuatorType) {
    case FORCE:
    case PASSIVE:
    case SERVO:
    case MIMIC:
      return true;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      return false;
    default:
      DART_ERROR(
          "Unsupported actuator type: {}.", static_cast<int>(actuatorType));
      return false;
  }
}

//==============================================================================
void Joint::setMimicJoint(
    const Joint* referenceJoint, double mimicMultiplier, double mimicOffset)
{
  const auto constraintType = getMimicConstraintType();
  std::size_t numDofs = getNumDofs();
  mAspectProperties.mMimicDofProps.resize(numDofs);

  for (std::size_t i = 0; i < numDofs; ++i) {
    MimicDofProperties prop;
    prop.mReferenceJoint = referenceJoint;
    prop.mReferenceDofIndex = i;
    prop.mMultiplier = mimicMultiplier;
    prop.mOffset = mimicOffset;
    prop.mConstraintType = constraintType;
    setMimicJointDof(i, prop);
  }
}

//==============================================================================
void Joint::setMimicJointDof(
    std::size_t index, const MimicDofProperties& mimicProp)
{
  mAspectProperties.mMimicDofProps[index] = mimicProp;
}

//==============================================================================
void Joint::setMimicJointDofs(std::span<const MimicDofProperties> mimicProps)
{
  mAspectProperties.mMimicDofProps.assign(mimicProps.begin(), mimicProps.end());
}

//==============================================================================
void Joint::setMimicJointDofs(
    const std::map<std::size_t, MimicDofProperties>& mimicPropMap)
{
  for (const auto& pair : mimicPropMap) {
    const auto& index = pair.first;
    const auto& prop = pair.second;
    setMimicJointDof(index, prop);
  }
}

//==============================================================================
const Joint* Joint::getMimicJoint(std::size_t index) const
{
  return mAspectProperties.mMimicDofProps[index].mReferenceJoint;
}

//==============================================================================
double Joint::getMimicMultiplier(std::size_t index) const
{
  return mAspectProperties.mMimicDofProps[index].mMultiplier;
}

//==============================================================================
double Joint::getMimicOffset(std::size_t index) const
{
  return mAspectProperties.mMimicDofProps[index].mOffset;
}

//==============================================================================
std::span<const MimicDofProperties> Joint::getMimicDofProperties() const
{
  return std::span<const MimicDofProperties>(mAspectProperties.mMimicDofProps);
}

//==============================================================================
void Joint::setMimicConstraintType(MimicConstraintType type)
{
  for (auto& prop : mAspectProperties.mMimicDofProps)
    prop.mConstraintType = type;
}

//==============================================================================
MimicConstraintType Joint::getMimicConstraintType() const
{
  if (mAspectProperties.mMimicDofProps.empty())
    return MimicConstraintType::Motor;
  return mAspectProperties.mMimicDofProps.front().mConstraintType;
}

//==============================================================================
void Joint::setUseCouplerConstraint(bool enable)
{
  setMimicConstraintType(
      enable ? MimicConstraintType::Coupler : MimicConstraintType::Motor);
}

//==============================================================================
bool Joint::isUsingCouplerConstraint() const
{
  return getMimicConstraintType() == MimicConstraintType::Coupler;
}

//==============================================================================
bool Joint::isKinematic() const
{
  const auto numDofs = getNumDofs();
  if (numDofs == 0)
    return isKinematicActuatorType(mAspectProperties.mActuatorType);

  for (std::size_t i = 0; i < numDofs; ++i) {
    if (!isKinematicActuatorType(getActuatorType(i)))
      return false;
  }

  return true;
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
//==============================================================================
const Eigen::Isometry3d& Joint::getRelativeTransform() const
{
  if (mNeedTransformUpdate) {
    updateRelativeTransform();
    mNeedTransformUpdate = false;
  }

  return mT;
}

//==============================================================================
const Eigen::Vector6d& Joint::getRelativeSpatialVelocity() const
{
  if (mNeedSpatialVelocityUpdate) {
    updateRelativeSpatialVelocity();
    mNeedSpatialVelocityUpdate = false;
  }

  return mSpatialVelocity;
}

//==============================================================================
const Eigen::Vector6d& Joint::getRelativeSpatialAcceleration() const
{
  if (mNeedSpatialAccelerationUpdate) {
    updateRelativeSpatialAcceleration();
    mNeedSpatialAccelerationUpdate = false;
  }

  return mSpatialAcceleration;
}

//==============================================================================
const Eigen::Vector6d& Joint::getRelativePrimaryAcceleration() const
{
  if (mNeedPrimaryAccelerationUpdate) {
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
  if (!childBodyNode) {
    return Eigen::Vector6d::Zero();
  }

  const Eigen::Vector6d& F2 = childBodyNode->getBodyForce();
  const BodyNode* parentBodyNode = getParentBodyNode();

  if (withRespectTo == nullptr) {
    // (Default) Wrench applying to the child body node, where the reference
    // frame is the joint frame
    return math::dAdT(getTransformFromChildBodyNode(), -F2);
  } else if (withRespectTo == childBodyNode) {
    // Wrench applying to the child body node, where the reference frame is the
    // child body frame
    return -F2;
  } else if (withRespectTo == parentBodyNode) {
    // Wrench applying to the child body node, where the reference frame is the
    // parent body frame
    return math::dAdInvT(getRelativeTransform(), -F2);
  } else {
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
void Joint::setLimitEnforcement(bool enforced)
{
  mAspectProperties.mIsPositionLimitEnforced = enforced;
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
  for (std::size_t i = 0; i < getNumDofs(); ++i) {
    if (getInitialPosition(i) < getPositionLowerLimit(i)
        || getPositionUpperLimit(i) < getInitialPosition(i)) {
      if (_printWarnings) {
        DART_WARN(
            "Initial position of index {} [{}] in Joint [{}] is outside of its "
            "position limits\\n -- Initial Position: {}\\n -- Limits: [{}, {}]",
            i,
            getDofName(i),
            getName(),
            getInitialPosition(i),
            getPositionLowerLimit(i),
            getPositionUpperLimit(i));
      } else {
        return false;
      }

      sane = false;
    }

    if (getInitialVelocity(i) < getVelocityLowerLimit(i)
        || getVelocityUpperLimit(i) < getInitialVelocity(i)) {
      if (_printWarnings) {
        DART_WARN(
            "Initial velocity of index {} [{}] is Joint [{}] is outside of its "
            "velocity limits\\n -- Initial Velocity: {}\\n -- Limits: [{}, {}]",
            i,
            getDofName(i),
            getName(),
            getInitialVelocity(i),
            getVelocityLowerLimit(i),
            getVelocityUpperLimit(i));
      } else {
        return false;
      }

      sane = false;
    }
  }

  return sane;
}

//==============================================================================
Eigen::VectorXd Joint::integratePositions(
    const Eigen::VectorXd& q0, const Eigen::VectorXd& v, double dt) const
{
  Eigen::VectorXd result(getNumDofs());
  integratePositions(q0, v, dt, result);
  return result;
}

//==============================================================================
void Joint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
{
  DART_ASSERT(math::verifyTransform(_T));
  mAspectProperties.mT_ParentBodyToJoint = _T;
  notifyPositionUpdated();
}

//==============================================================================
void Joint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  DART_ASSERT(math::verifyTransform(_T));
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
void Joint::notifyPositionUpdated()
{
  if (mChildBodyNode) {
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
  if (skel) {
    std::size_t tree = mChildBodyNode->mTreeIndex;
    skel->dirtyArticulatedInertia(tree);
    skel->mTreeCache[tree].mDirty.mExternalForces = true;
    skel->mSkelCache.mDirty.mExternalForces = true;
  }
}

//==============================================================================
void Joint::notifyVelocityUpdated()
{
  if (mChildBodyNode) {
    mChildBodyNode->dirtyVelocity();
    mChildBodyNode->dirtyJacobianDeriv();
  }

  mIsRelativeJacobianTimeDerivDirty = true;

  mNeedSpatialVelocityUpdate = true;
  mNeedSpatialAccelerationUpdate = true;
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
