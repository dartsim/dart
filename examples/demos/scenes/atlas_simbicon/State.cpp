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

// Ported wholesale from examples/atlas_simbicon/State.cpp. Deviation: the
// original also declared (but only ever stubbed out with a "Not implemented
// yet" printout) string-keyed overloads of getDesiredJointPosition,
// setProportionalGain, and setDerivativeGain; Controller.cpp -- the only
// caller in this subsystem -- never calls them, so they are dropped here as
// genuine dead code rather than ported as non-functional stubs.

#include "State.hpp"

#include "TerminalCondition.hpp"
#include "dart/common/Macros.hpp"

namespace dart_demos {
namespace atlas_simbicon {

//==============================================================================
State::State(dart::dynamics::SkeletonPtr skeleton, const std::string& name)
  : mName(name),
    mSkeleton(skeleton),
    mNextState(this),
    mTerminalCondition(nullptr),
    mBeginTime(0.0),
    mEndTime(0.0),
    mFrame(0),
    mElapsedTime(0.0),
    mDesiredGlobalSwingLegAngleOnSagital(0.0),
    mDesiredGlobalSwingLegAngleOnCoronal(0.0),
    mDesiredGlobalPelvisAngleOnSagital(0.0),
    mDesiredGlobalPelvisAngleOnCoronal(0.0)
{
  const int dof = static_cast<int>(mSkeleton->getNumDofs());

  mDesiredJointPositions = Eigen::VectorXd::Zero(dof);
  mDesiredJointPositionsBalance = Eigen::VectorXd::Zero(dof);
  mKp = Eigen::VectorXd::Zero(dof);
  mKd = Eigen::VectorXd::Zero(dof);
  mSagitalCd = Eigen::VectorXd::Zero(dof);
  mSagitalCv = Eigen::VectorXd::Zero(dof);
  mCoronalCd = Eigen::VectorXd::Zero(dof);
  mCoronalCv = Eigen::VectorXd::Zero(dof);
  mTorque = Eigen::VectorXd::Zero(dof);

  for (int i = 0; i < dof; ++i) {
    mKp[i] = kAtlasDefaultKp;
    mKd[i] = kAtlasDefaultKd;
  }

  mPelvis = mSkeleton->getBodyNode("pelvis");
  mLeftFoot = mSkeleton->getBodyNode("l_foot");
  mRightFoot = mSkeleton->getBodyNode("r_foot");
  mLeftThigh = mSkeleton->getBodyNode("l_uleg");
  mRightThigh = mSkeleton->getBodyNode("r_uleg");
  mStanceFoot = nullptr;

  DART_ASSERT(mPelvis != nullptr);
  DART_ASSERT(mLeftFoot != nullptr);
  DART_ASSERT(mRightFoot != nullptr);
  DART_ASSERT(mLeftThigh != nullptr);
  DART_ASSERT(mRightThigh != nullptr);

  mCoronalLeftHip = mSkeleton->getDof("l_leg_hpx")->getIndexInSkeleton();
  mCoronalRightHip = mSkeleton->getDof("r_leg_hpx")->getIndexInSkeleton();
  mSagitalLeftHip = mSkeleton->getDof("l_leg_hpy")->getIndexInSkeleton();
  mSagitalRightHip = mSkeleton->getDof("r_leg_hpy")->getIndexInSkeleton();
}

//==============================================================================
State::~State() {}

//==============================================================================
void State::setName(std::string& name)
{
  mName = name;
}

//==============================================================================
const std::string& State::getName() const
{
  return mName;
}

//==============================================================================
void State::setNextState(State* nextState)
{
  mNextState = nextState;
}

//==============================================================================
void State::setTerminalCondition(TerminalCondition* condition)
{
  DART_ASSERT(condition != nullptr);
  mTerminalCondition = condition;
}

//==============================================================================
void State::begin(double currentTime)
{
  mBeginTime = currentTime;
  mFrame = 0;
  mElapsedTime = 0.0;
}

//==============================================================================
void State::computeControlForce(double timestep)
{
  DART_ASSERT(mNextState != nullptr && "Next state should be set.");

  const int dof = static_cast<int>(mSkeleton->getNumDofs());
  const Eigen::VectorXd q = mSkeleton->getPositions();
  const Eigen::VectorXd dq = mSkeleton->getVelocities();

  // Update desired joint angles with balance feedback. Equation (1) in the
  // Simbicon paper.
  mDesiredJointPositionsBalance = mDesiredJointPositions
                                  + getSagitalCOMDistance() * mSagitalCd
                                  + getSagitalCOMVelocity() * mSagitalCv
                                  + getCoronalCOMDistance() * mCoronalCd
                                  + getCoronalCOMVelocity() * mCoronalCv;

  // The first 6 dofs are the free-floating base; leave them unactuated.
  mTorque.head<6>() = Eigen::Vector6d::Zero();
  for (int i = 6; i < dof; ++i) {
    mTorque[i]
        = -mKp[i] * (q[i] - mDesiredJointPositionsBalance[i]) - mKd[i] * dq[i];
  }

  _updateTorqueForStanceLeg();

  mSkeleton->setForces(mTorque);

  mElapsedTime += timestep;
  mFrame++;
}

//==============================================================================
bool State::isTerminalConditionSatisfied() const
{
  DART_ASSERT(mTerminalCondition != nullptr && "Invalid terminal condition.");
  return mTerminalCondition->isSatisfied();
}

//==============================================================================
void State::end(double currentTime)
{
  mEndTime = currentTime;
}

//==============================================================================
Eigen::Vector3d State::getCOM() const
{
  return mSkeleton->getCOM();
}

//==============================================================================
Eigen::Vector3d State::getCOMVelocity() const
{
  return mSkeleton->getCOMLinearVelocity();
}

//==============================================================================
Eigen::Isometry3d State::getCOMFrame() const
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d yAxis = Eigen::Vector3d::UnitY();

  Eigen::Vector3d pelvisXAxis = mPelvis->getTransform().linear().col(0);
  const double mag = yAxis.dot(pelvisXAxis);
  pelvisXAxis -= mag * yAxis;
  const Eigen::Vector3d xAxis = pelvisXAxis.normalized();

  const Eigen::Vector3d zAxis = xAxis.cross(yAxis);

  T.translation() = getCOM();
  T.linear().col(0) = xAxis;
  T.linear().col(1) = yAxis;
  T.linear().col(2) = zAxis;

  return T;
}

//==============================================================================
double State::getSagitalCOMDistance()
{
  const Eigen::Vector3d xAxis = getCOMFrame().linear().col(0);
  const Eigen::Vector3d d = getCOM() - getStanceAnklePosition();
  return d.dot(xAxis);
}

//==============================================================================
double State::getSagitalCOMVelocity()
{
  const Eigen::Vector3d xAxis = getCOMFrame().linear().col(0);
  const Eigen::Vector3d v = getCOMVelocity();
  return v.dot(xAxis);
}

//==============================================================================
double State::getCoronalCOMDistance()
{
  const Eigen::Vector3d zAxis = getCOMFrame().linear().col(2);
  const Eigen::Vector3d d = getCOM() - getStanceAnklePosition();
  return d.dot(zAxis);
}

//==============================================================================
double State::getCoronalCOMVelocity()
{
  const Eigen::Vector3d zAxis = getCOMFrame().linear().col(2);
  const Eigen::Vector3d v = getCOMVelocity();
  return v.dot(zAxis);
}

//==============================================================================
Eigen::Vector3d State::getStanceAnklePosition() const
{
  if (mStanceFoot == nullptr)
    return getCOM();
  return _getJointPosition(mStanceFoot);
}

//==============================================================================
Eigen::Vector3d State::getLeftAnklePosition() const
{
  return _getJointPosition(mLeftFoot);
}

//==============================================================================
Eigen::Vector3d State::getRightAnklePosition() const
{
  return _getJointPosition(mRightFoot);
}

//==============================================================================
double State::getSagitalPelvisAngle() const
{
  const Eigen::Matrix3d comR = getCOMFrame().linear();
  const Eigen::Vector3d comY = comR.col(1);

  Eigen::Vector3d pelvisZ = mPelvis->getTransform().linear().col(2);
  Eigen::Vector3d projPelvisZ = comR.transpose() * pelvisZ;
  projPelvisZ[2] = 0.0;
  projPelvisZ.normalize();
  const double angle = _getAngleBetweenTwoVectors(projPelvisZ, comY);

  const Eigen::Vector3d cross = comY.cross(projPelvisZ);
  return cross[2] > 0.0 ? angle : -angle;
}

//==============================================================================
double State::getCoronalPelvisAngle() const
{
  const Eigen::Matrix3d comR = getCOMFrame().linear();
  const Eigen::Vector3d comY = comR.col(1);
  Eigen::Vector3d pelvisZ = mPelvis->getTransform().linear().col(2);
  Eigen::Vector3d projPelvisZ = comR.transpose() * pelvisZ;
  projPelvisZ[0] = 0.0;
  projPelvisZ.normalize();
  const double angle = _getAngleBetweenTwoVectors(projPelvisZ, comY);

  const Eigen::Vector3d cross = comY.cross(projPelvisZ);
  return cross[0] > 0.0 ? angle : -angle;
}

//==============================================================================
double State::getSagitalLeftLegAngle() const
{
  const Eigen::Matrix3d comR = getCOMFrame().linear();
  const Eigen::Vector3d comY = comR.col(1);
  Eigen::Vector3d thighAxisZ = mLeftThigh->getTransform().linear().col(2);
  Eigen::Vector3d projThighAZ = comR.transpose() * thighAxisZ;
  projThighAZ[2] = 0.0;
  projThighAZ.normalize();
  const double angle = _getAngleBetweenTwoVectors(projThighAZ, comY);

  const Eigen::Vector3d cross = comY.cross(projThighAZ);
  return cross[2] > 0.0 ? angle : -angle;
}

//==============================================================================
double State::getSagitalRightLegAngle() const
{
  const Eigen::Matrix3d comR = getCOMFrame().linear();
  const Eigen::Vector3d comY = comR.col(1);
  Eigen::Vector3d thighAxisZ = mRightThigh->getTransform().linear().col(2);
  Eigen::Vector3d projThighAZ = comR.transpose() * thighAxisZ;
  projThighAZ[2] = 0.0;
  projThighAZ.normalize();
  const double angle = _getAngleBetweenTwoVectors(projThighAZ, comY);

  const Eigen::Vector3d cross = comY.cross(projThighAZ);
  return cross[2] > 0.0 ? angle : -angle;
}

//==============================================================================
double State::getCoronalLeftLegAngle() const
{
  const Eigen::Matrix3d comR = getCOMFrame().linear();
  const Eigen::Vector3d comY = comR.col(1);
  Eigen::Vector3d thighAxisZ = mLeftThigh->getTransform().linear().col(2);
  Eigen::Vector3d projThighAZ = comR.transpose() * thighAxisZ;
  projThighAZ[0] = 0.0;
  projThighAZ.normalize();
  const double angle = _getAngleBetweenTwoVectors(projThighAZ, comY);

  const Eigen::Vector3d cross = comY.cross(projThighAZ);
  return cross[0] > 0.0 ? angle : -angle;
}

//==============================================================================
double State::getCoronalRightLegAngle() const
{
  const Eigen::Matrix3d comR = getCOMFrame().linear();
  const Eigen::Vector3d comY = comR.col(1);
  Eigen::Vector3d thighAxisZ = mRightThigh->getTransform().linear().col(2);
  Eigen::Vector3d projThighAZ = comR.transpose() * thighAxisZ;
  projThighAZ[0] = 0.0;
  projThighAZ.normalize();
  const double angle = _getAngleBetweenTwoVectors(projThighAZ, comY);

  const Eigen::Vector3d cross = comY.cross(projThighAZ);
  return cross[0] > 0.0 ? angle : -angle;
}

//==============================================================================
Eigen::Vector3d State::_getJointPosition(
    dart::dynamics::BodyNode* bodyNode) const
{
  auto* parentJoint = bodyNode->getParentJoint();
  const Eigen::Vector3d localJointPosition
      = parentJoint->getTransformFromChildBodyNode().translation();
  return bodyNode->getTransform() * localJointPosition;
}

//==============================================================================
double State::_getAngleBetweenTwoVectors(
    const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const
{
  return std::acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

//==============================================================================
void State::_updateTorqueForStanceLeg()
{
  if (mStanceFoot == mLeftFoot) {
    const double pelvisSagitalAngle = getSagitalPelvisAngle();
    const double tauTorsoSagital
        = -5000.0 * (pelvisSagitalAngle + mDesiredGlobalPelvisAngleOnSagital);
    mTorque[mSagitalLeftHip] = tauTorsoSagital - mTorque[mSagitalRightHip];

    const double pelvisCoronalAngle = getCoronalPelvisAngle();
    const double tauTorsoCoronal
        = -5000.0 * (pelvisCoronalAngle - mDesiredGlobalPelvisAngleOnCoronal);
    mTorque[mCoronalLeftHip] = -tauTorsoCoronal - mTorque[mCoronalRightHip];
  } else if (mStanceFoot == mRightFoot) {
    const double pelvisSagitalAngle = getSagitalPelvisAngle();
    const double tauTorsoSagital
        = -5000.0 * (pelvisSagitalAngle + mDesiredGlobalPelvisAngleOnSagital);
    mTorque[mSagitalRightHip] = tauTorsoSagital - mTorque[mSagitalLeftHip];

    const double pelvisCoronalAngle = getCoronalPelvisAngle();
    const double tauTorsoCoronal
        = -5000.0 * (pelvisCoronalAngle - mDesiredGlobalPelvisAngleOnCoronal);
    mTorque[mCoronalRightHip] = -tauTorsoCoronal - mTorque[mCoronalLeftHip];
  }
  // else: no foot is touching the ground -- no torso/swing-hip compensation.
}

//==============================================================================
State* State::getNextState() const
{
  return mNextState;
}

//==============================================================================
double State::getElapsedTime() const
{
  return mElapsedTime;
}

//==============================================================================
void State::setDesiredJointPosition(const std::string& jointName, double val)
{
  const std::size_t index = mSkeleton->getDof(jointName)->getIndexInSkeleton();
  mDesiredJointPositions[index] = val;
}

//==============================================================================
double State::getDesiredJointPosition(int idx) const
{
  DART_ASSERT(0 <= idx && idx <= mDesiredJointPositions.size());
  return mDesiredJointPositions[idx];
}

//==============================================================================
void State::setDesiredSwingLegGlobalAngleOnSagital(double val)
{
  mDesiredGlobalSwingLegAngleOnSagital = val;
}

//==============================================================================
void State::setDesiredSwingLegGlobalAngleOnCoronal(double val)
{
  mDesiredGlobalSwingLegAngleOnCoronal = val;
}

//==============================================================================
void State::setDesiredPelvisGlobalAngleOnSagital(double val)
{
  mDesiredGlobalPelvisAngleOnSagital = val;
}

//==============================================================================
void State::setDesiredPelvisGlobalAngleOnCoronal(double val)
{
  mDesiredGlobalPelvisAngleOnCoronal = val;
}

//==============================================================================
void State::setProportionalGain(int idx, double val)
{
  DART_ASSERT(0 <= idx && idx <= mKp.size());
  mKp[idx] = val;
}

//==============================================================================
double State::getProportionalGain(int idx) const
{
  DART_ASSERT(0 <= idx && idx <= mKp.size());
  return mKp[idx];
}

//==============================================================================
void State::setDerivativeGain(int idx, double val)
{
  DART_ASSERT(0 <= idx && idx <= mKd.size());
  mKd[idx] = val;
}

//==============================================================================
double State::getDerivativeGain(int idx) const
{
  DART_ASSERT(0 <= idx && idx <= mKd.size());
  return mKd[idx];
}

//==============================================================================
void State::setFeedbackSagitalCOMDistance(std::size_t index, double val)
{
  DART_ASSERT(static_cast<int>(index) <= mSagitalCd.size());
  mSagitalCd[index] = val;
}

//==============================================================================
void State::setFeedbackSagitalCOMVelocity(std::size_t index, double val)
{
  DART_ASSERT(static_cast<int>(index) <= mSagitalCv.size());
  mSagitalCv[index] = val;
}

//==============================================================================
void State::setFeedbackCoronalCOMDistance(std::size_t index, double val)
{
  DART_ASSERT(static_cast<int>(index) <= mCoronalCd.size());
  mCoronalCd[index] = val;
}

//==============================================================================
void State::setFeedbackCoronalCOMVelocity(std::size_t index, double val)
{
  DART_ASSERT(static_cast<int>(index) <= mCoronalCv.size());
  mCoronalCv[index] = val;
}

//==============================================================================
void State::setStanceFootToLeftFoot()
{
  mStanceFoot = mLeftFoot;
}

//==============================================================================
void State::setStanceFootToRightFoot()
{
  mStanceFoot = mRightFoot;
}

} // namespace atlas_simbicon
} // namespace dart_demos
