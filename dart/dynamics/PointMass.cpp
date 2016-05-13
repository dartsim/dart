/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/dynamics/PointMass.hpp"

#include "dart/common/Console.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

using namespace Eigen;

namespace dart {
namespace dynamics {

#define RETURN_FALSE_IF_OTHER_IS_EQUAL( X )\
  if( other. X != X )\
    return false;

//==============================================================================
PointMass::State::State(
    const Vector3d& positions,
    const Vector3d& velocities,
    const Vector3d& accelerations,
    const Vector3d& forces)
  : mPositions(positions),
    mVelocities(velocities),
    mAccelerations(accelerations),
    mForces(forces)
{
  // Do nothing
}

//==============================================================================
bool PointMass::State::operator ==(const PointMass::State& other) const
{
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mPositions);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mVelocities);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mAccelerations);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mForces);

  return true;
}

//==============================================================================
PointMass::Properties::Properties(
    const Vector3d& _X0,
    double _mass,
    const std::vector<std::size_t>& _connections,
    const Vector3d& _positionLowerLimits,
    const Vector3d& _positionUpperLimits,
    const Vector3d& _velocityLowerLimits,
    const Vector3d& _velocityUpperLimits,
    const Vector3d& _accelerationLowerLimits,
    const Vector3d& _accelerationUpperLimits,
    const Vector3d& _forceLowerLimits,
    const Vector3d& _forceUpperLimits)
  : mX0(_X0),
    mMass(_mass),
    mConnectedPointMassIndices(_connections),
    mPositionLowerLimits(_positionLowerLimits),
    mPositionUpperLimits(_positionUpperLimits),
    mVelocityLowerLimits(_velocityLowerLimits),
    mVelocityUpperLimits(_velocityUpperLimits),
    mAccelerationLowerLimits(_accelerationLowerLimits),
    mAccelerationUpperLimits(_accelerationUpperLimits),
    mForceLowerLimits(_forceLowerLimits),
    mForceUpperLimits(_forceUpperLimits)
{
  // Do nothing
}

//==============================================================================
void PointMass::Properties::setRestingPosition(const Vector3d &_x)
{
  mX0 = _x;
}

//==============================================================================
void PointMass::Properties::setMass(double _mass)
{
  mMass = _mass;
}

//==============================================================================
bool PointMass::Properties::operator ==(const PointMass::Properties& other) const
{
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mX0);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mMass);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mConnectedPointMassIndices);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mPositionLowerLimits);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mPositionUpperLimits);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mVelocityLowerLimits);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mVelocityUpperLimits);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mAccelerationLowerLimits);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mAccelerationUpperLimits);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mForceLowerLimits);
  RETURN_FALSE_IF_OTHER_IS_EQUAL(mForceUpperLimits);

  // Nothing was inequal, so we return true
  return true;
}

//==============================================================================
bool PointMass::Properties::operator !=(const PointMass::Properties& other) const
{
  return !(other == *this);
}

//==============================================================================
PointMass::PointMass(SoftBodyNode* _softBodyNode)
  : // mIndexInSkeleton(Eigen::Matrix<std::size_t, 3, 1>::Zero()),
    mParentSoftBodyNode(_softBodyNode),
    mPositionDeriv(Eigen::Vector3d::Zero()),
    mVelocitiesDeriv(Eigen::Vector3d::Zero()),
    mAccelerationsDeriv(Eigen::Vector3d::Zero()),
    mForcesDeriv(Eigen::Vector3d::Zero()),
    mVelocityChanges(Eigen::Vector3d::Zero()),
    // mImpulse(Eigen::Vector3d::Zero()),
    mConstraintImpulses(Eigen::Vector3d::Zero()),
    mW(Eigen::Vector3d::Zero()),
    mX(Eigen::Vector3d::Zero()),
    mV(Eigen::Vector3d::Zero()),
    mEta(Eigen::Vector3d::Zero()),
    mAlpha(Eigen::Vector3d::Zero()),
    mBeta(Eigen::Vector3d::Zero()),
    mA(Eigen::Vector3d::Zero()),
    mF(Eigen::Vector3d::Zero()),
    mPsi(0.0),
    mImplicitPsi(0.0),
    mPi(0.0),
    mImplicitPi(0.0),
    mB(Eigen::Vector3d::Zero()),
    mFext(Eigen::Vector3d::Zero()),
    mIsColliding(false),
    mDelV(Eigen::Vector3d::Zero()),
    mImpB(Eigen::Vector3d::Zero()),
    mImpAlpha(Eigen::Vector3d::Zero()),
    mImpBeta(Eigen::Vector3d::Zero()),
    mImpF(Eigen::Vector3d::Zero()),
    mNotifier(_softBodyNode->mNotifier)
{
  assert(mParentSoftBodyNode != nullptr);
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
PointMass::~PointMass()
{
  // Do nothing
}

//==============================================================================
PointMass::State& PointMass::getState()
{
  return mParentSoftBodyNode->mAspectState.mPointStates[mIndex];
}

//==============================================================================
const PointMass::State& PointMass::getState() const
{
  return mParentSoftBodyNode->mAspectState.mPointStates[mIndex];
}

//==============================================================================
std::size_t PointMass::getIndexInSoftBodyNode() const
{
  return mIndex;
}

//==============================================================================
void PointMass::setMass(double _mass)
{
  assert(0.0 < _mass);
  double& mMass = mParentSoftBodyNode->mAspectProperties.mPointProps[mIndex].mMass;
  if(_mass == mMass)
    return;

  mMass = _mass;
  mParentSoftBodyNode->incrementVersion();
}

//==============================================================================
double PointMass::getMass() const
{
  return mParentSoftBodyNode->mAspectProperties.mPointProps[mIndex].mMass;
}

//==============================================================================
double PointMass::getPsi() const
{
  mParentSoftBodyNode->checkArticulatedInertiaUpdate();
  return mPsi;
}

//==============================================================================
double PointMass::getImplicitPsi() const
{
  mParentSoftBodyNode->checkArticulatedInertiaUpdate();
  return mImplicitPsi;
}

//==============================================================================
double PointMass::getPi() const
{
  mParentSoftBodyNode->checkArticulatedInertiaUpdate();
  return mPi;
}

//==============================================================================
double PointMass::getImplicitPi() const
{
  mParentSoftBodyNode->checkArticulatedInertiaUpdate();
  return mImplicitPi;
}

//==============================================================================
void PointMass::addConnectedPointMass(PointMass* _pointMass)
{
  assert(_pointMass != nullptr);

  mParentSoftBodyNode->mAspectProperties.mPointProps[mIndex].
      mConnectedPointMassIndices.push_back(_pointMass->mIndex);
  mParentSoftBodyNode->incrementVersion();
}

//==============================================================================
std::size_t PointMass::getNumConnectedPointMasses() const
{
  return mParentSoftBodyNode->mAspectProperties.mPointProps[mIndex].
      mConnectedPointMassIndices.size();
}

//==============================================================================
PointMass* PointMass::getConnectedPointMass(std::size_t _idx)
{
  assert(_idx < getNumConnectedPointMasses());

  return mParentSoftBodyNode->mPointMasses[
      mParentSoftBodyNode->mAspectProperties.mPointProps[mIndex].
      mConnectedPointMassIndices[_idx]];
}

//==============================================================================
const PointMass* PointMass::getConnectedPointMass(std::size_t _idx) const
{
  return const_cast<PointMass*>(this)->getConnectedPointMass(_idx);
}

//==============================================================================
void PointMass::setColliding(bool _isColliding)
{
  mIsColliding = _isColliding;
}

//==============================================================================
bool PointMass::isColliding()
{
  return mIsColliding;
}

//==============================================================================
std::size_t PointMass::getNumDofs() const
{
  return 3;
}

////==============================================================================
//void PointMass::setIndexInSkeleton(std::size_t _index, std::size_t _indexInSkeleton)
//{
//  assert(_index < 3);

//  mIndexInSkeleton[_index] = _indexInSkeleton;
//}

////==============================================================================
//std::size_t PointMass::getIndexInSkeleton(std::size_t _index) const
//{
//  assert(_index < 3);

//  return mIndexInSkeleton[_index];
//}

//==============================================================================
void PointMass::setPosition(std::size_t _index, double _position)
{
  assert(_index < 3);

  getState().mPositions[_index] = _position;
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
double PointMass::getPosition(std::size_t _index) const
{
  assert(_index < 3);

  return getState().mPositions[_index];
}

//==============================================================================
void PointMass::setPositions(const Vector3d& _positions)
{
  getState().mPositions = _positions;
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
const Vector3d& PointMass::getPositions() const
{
  return getState().mPositions;
}

//==============================================================================
void PointMass::resetPositions()
{
  getState().mPositions.setZero();
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
void PointMass::setVelocity(std::size_t _index, double _velocity)
{
  assert(_index < 3);

  getState().mVelocities[_index] = _velocity;
  mNotifier->notifyVelocityUpdate();
}

//==============================================================================
double PointMass::getVelocity(std::size_t _index) const
{
  assert(_index < 3);

  return getState().mVelocities[_index];
}

//==============================================================================
void PointMass::setVelocities(const Vector3d& _velocities)
{
  getState().mVelocities = _velocities;
  mNotifier->notifyVelocityUpdate();
}

//==============================================================================
const Vector3d& PointMass::getVelocities() const
{
  return getState().mVelocities;
}

//==============================================================================
void PointMass::resetVelocities()
{
  getState().mVelocities.setZero();
  mNotifier->notifyVelocityUpdate();
}

//==============================================================================
void PointMass::setAcceleration(std::size_t _index, double _acceleration)
{
  assert(_index < 3);

  getState().mAccelerations[_index] = _acceleration;
  mNotifier->notifyAccelerationUpdate();
}

//==============================================================================
double PointMass::getAcceleration(std::size_t _index) const
{
 assert(_index < 3);

 return getState().mAccelerations[_index];
}

//==============================================================================
void PointMass::setAccelerations(const Eigen::Vector3d& _accelerations)
{
  getState().mAccelerations = _accelerations;
  mNotifier->notifyAccelerationUpdate();
}

//==============================================================================
const Vector3d& PointMass::getAccelerations() const
{
  return getState().mAccelerations;
}

//==============================================================================
const Vector3d& PointMass::getPartialAccelerations() const
{
  if(mNotifier->needsPartialAccelerationUpdate())
    mParentSoftBodyNode->updatePartialAcceleration();
  return mEta;
}

//==============================================================================
void PointMass::resetAccelerations()
{
  getState().mAccelerations.setZero();
  mNotifier->notifyAccelerationUpdate();
}

//==============================================================================
void PointMass::setForce(std::size_t _index, double _force)
{
  assert(_index < 3);

  getState().mForces[_index] = _force;
}

//==============================================================================
double PointMass::getForce(std::size_t _index)
{
  assert(_index < 3);

  return getState().mForces[_index];
}

//==============================================================================
void PointMass::setForces(const Vector3d& _forces)
{
  getState().mForces = _forces;
}

//==============================================================================
const Vector3d& PointMass::getForces() const
{
  return getState().mForces;
}

//==============================================================================
void PointMass::resetForces()
{
  getState().mForces.setZero();
}

//==============================================================================
void PointMass::setVelocityChange(std::size_t _index, double _velocityChange)
{
  assert(_index < 3);

  mVelocityChanges[_index] = _velocityChange;
}

//==============================================================================
double PointMass::getVelocityChange(std::size_t _index)
{
  assert(_index < 3);

  return mVelocityChanges[_index];
}

//==============================================================================
void PointMass::resetVelocityChanges()
{
  mVelocityChanges.setZero();
}

//==============================================================================
void PointMass::setConstraintImpulse(std::size_t _index, double _impulse)
{
  assert(_index < 3);

  mConstraintImpulses[_index] = _impulse;
}

//==============================================================================
double PointMass::getConstraintImpulse(std::size_t _index)
{
  assert(_index < 3);

  return mConstraintImpulses[_index];
}

//==============================================================================
void PointMass::resetConstraintImpulses()
{
  mConstraintImpulses.setZero();
}

//==============================================================================
void PointMass::integratePositions(double _dt)
{
  setPositions( getPositions() + getVelocities() * _dt );
}

//==============================================================================
void PointMass::integrateVelocities(double _dt)
{
  setVelocities( getVelocities() + getAccelerations() * _dt );
}

//==============================================================================
void PointMass::addExtForce(const Eigen::Vector3d& _force, bool _isForceLocal)
{
  if (_isForceLocal)
  {
    mFext += _force;
  }
  else
  {
    mFext += mParentSoftBodyNode->getWorldTransform().linear().transpose()
             * _force;
  }
}

//==============================================================================
void PointMass::clearExtForce()
{
  mFext.setZero();
}

//==============================================================================
void PointMass::setConstraintImpulse(const Eigen::Vector3d& _constImp,
                                     bool _isLocal)
{
  if (_isLocal)
  {
    mConstraintImpulses = _constImp;
  }
  else
  {
    const Matrix3d Rt
        = mParentSoftBodyNode->getWorldTransform().linear().transpose();
    mConstraintImpulses = Rt * _constImp;
  }
}

//==============================================================================
void PointMass::addConstraintImpulse(const Eigen::Vector3d& _constImp,
                                     bool _isLocal)
{
  if (_isLocal)
  {
    mConstraintImpulses += _constImp;
  }
  else
  {
    const Matrix3d Rt
        = mParentSoftBodyNode->getWorldTransform().linear().transpose();
    mConstraintImpulses.noalias() += Rt * _constImp;
  }
}

//==============================================================================
Eigen::Vector3d PointMass::getConstraintImpulses() const
{
  return mConstraintImpulses;
}

//==============================================================================
void PointMass::clearConstraintImpulse()
{
  assert(getNumDofs() == 3);
  mConstraintImpulses.setZero();
  mDelV.setZero();
  mImpB.setZero();
  mImpAlpha.setZero();
  mImpBeta.setZero();
  mImpF.setZero();
}

//==============================================================================
void PointMass::setRestingPosition(const Eigen::Vector3d& _p)
{

  Eigen::Vector3d& mRest =
      mParentSoftBodyNode->mAspectProperties.mPointProps[mIndex].mX0;
  if(_p == mRest)
    return;

  mRest = _p;
  mParentSoftBodyNode->incrementVersion();
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
const Eigen::Vector3d& PointMass::getRestingPosition() const
{
  return mParentSoftBodyNode->mAspectProperties.mPointProps[mIndex].mX0;
}

//==============================================================================
const Eigen::Vector3d& PointMass::getLocalPosition() const
{
  if(mNotifier->needsTransformUpdate())
    mParentSoftBodyNode->updateTransform();
  return mX;
}

//==============================================================================
const Eigen::Vector3d& PointMass::getWorldPosition() const
{
  if(mNotifier && mNotifier->needsTransformUpdate())
    mParentSoftBodyNode->updateTransform();
  return mW;
}

//==============================================================================
Eigen::Matrix<double, 3, Eigen::Dynamic> PointMass::getBodyJacobian()
{
  assert(mParentSoftBodyNode != nullptr);

  int dof = mParentSoftBodyNode->getNumDependentGenCoords();
  int totalDof = mParentSoftBodyNode->getNumDependentGenCoords() + 3;

  Eigen::Matrix<double, 3, Eigen::Dynamic> J
      = Eigen::MatrixXd::Zero(3, totalDof);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = getLocalPosition();

  J.leftCols(dof)
      = math::AdInvTJac(
          T, mParentSoftBodyNode->getJacobian()).bottomRows<3>();
  J.rightCols<3>() = Eigen::Matrix3d::Identity();

  return J;
}

//==============================================================================
Eigen::Matrix<double, 3, Eigen::Dynamic> PointMass::getWorldJacobian()
{
  return mParentSoftBodyNode->getWorldTransform().linear()
      * getBodyJacobian();
}

//==============================================================================
const Eigen::Vector3d& PointMass::getBodyVelocityChange() const
{
  return mDelV;
}

//==============================================================================
SoftBodyNode* PointMass::getParentSoftBodyNode()
{
  return mParentSoftBodyNode;
}

//==============================================================================
const SoftBodyNode* PointMass::getParentSoftBodyNode() const
{
  return mParentSoftBodyNode;
}

//==============================================================================
//int PointMass::getNumDependentGenCoords() const
//{
//  return mDependentGenCoordIndices.size();
//}

//==============================================================================
//int PointMass::getDependentGenCoord(int _arrayIndex) const
//{
//  assert(0 <= _arrayIndex && _arrayIndex < mDependentGenCoordIndices.size());
//  return mDependentGenCoordIndices[_arrayIndex];
//}

//==============================================================================
const Eigen::Vector3d&PointMass::getBodyVelocity() const
{
  if(mNotifier->needsVelocityUpdate())
    mParentSoftBodyNode->updateVelocity();
  return mV;
}

//==============================================================================
Eigen::Vector3d PointMass::getWorldVelocity() const
{
  return mParentSoftBodyNode->getWorldTransform().linear() * getBodyVelocity();
}

//==============================================================================
const Eigen::Vector3d& PointMass::getBodyAcceleration() const
{
  if(mNotifier->needsAccelerationUpdate())
    mParentSoftBodyNode->updateAccelerationID();
  return mA;
}

//==============================================================================
Eigen::Vector3d PointMass::getWorldAcceleration() const
{
  return mParentSoftBodyNode->getWorldTransform().linear() * getBodyAcceleration();
}

//==============================================================================
void PointMass::init()
{
  mDependentGenCoordIndices = mParentSoftBodyNode->getDependentGenCoordIndices();
}

//==============================================================================
void PointMass::updateTransform() const
{
  // Local translation
  mX = getPositions() + getRestingPosition();
  assert(!math::isNan(mX));

  // World translation
  const Eigen::Isometry3d& parentW = mParentSoftBodyNode->getWorldTransform();
  mW = parentW.translation() + parentW.linear() * mX;
  assert(!math::isNan(mW));
}

//==============================================================================
void PointMass::updateVelocity() const
{
  // v = w(parent) x mX + v(parent) + dq
  const Eigen::Vector6d& v_parent = mParentSoftBodyNode->getSpatialVelocity();
  mV = v_parent.head<3>().cross(getLocalPosition()) + v_parent.tail<3>()
       + getVelocities();
  assert(!math::isNan(mV));
}

//==============================================================================
void PointMass::updatePartialAcceleration() const
{
  // eta = w(parent) x dq
  const Eigen::Vector3d& dq = getVelocities();
  mEta = mParentSoftBodyNode->getSpatialVelocity().head<3>().cross(dq);
  assert(!math::isNan(mEta));
}

//==============================================================================
void PointMass::updateAccelerationID() const
{
  // dv = dw(parent) x mX + dv(parent) + eata + ddq
  const Eigen::Vector6d& a_parent = mParentSoftBodyNode->getSpatialAcceleration();
  mA = a_parent.head<3>().cross(getLocalPosition()) + a_parent.tail<3>()
       + getPartialAccelerations() + getAccelerations();
  assert(!math::isNan(mA));
}

//==============================================================================
void PointMass::updateTransmittedForceID(const Eigen::Vector3d& _gravity,
                                         bool /*_withExternalForces*/)
{
  // f = m*dv + w(parent) x m*v - fext
  mF.noalias() = getMass() * getBodyAcceleration();
  mF += mParentSoftBodyNode->getSpatialVelocity().head<3>().cross(
        getMass() * getBodyVelocity()) - mFext;
  if (mParentSoftBodyNode->getGravityMode() == true)
  {
    mF -= getMass() * (mParentSoftBodyNode->getWorldTransform().linear().transpose()
                   * _gravity);
  }
  assert(!math::isNan(mF));
}

//==============================================================================
void PointMass::updateArtInertiaFD(double _timeStep) const
{
  // Articulated inertia
  // - Do nothing

  // Cache data: PsiK and Psi
  mPsi = 1.0 / getMass();
  mImplicitPsi
      = 1.0 / (getMass()
               + _timeStep * mParentSoftBodyNode->getDampingCoefficient()
               + _timeStep * _timeStep
                 * mParentSoftBodyNode->getVertexSpringStiffness());
  assert(!math::isNan(mImplicitPsi));

  // Cache data: AI_S_Psi
  // - Do nothing

  // Cache data: Pi
  mPi         = getMass() - getMass() * getMass() * mPsi;
  mImplicitPi = getMass() - getMass() * getMass() * mImplicitPsi;
  assert(!math::isNan(mPi));
  assert(!math::isNan(mImplicitPi));
}

//==============================================================================
void PointMass::updateJointForceID(double /*_timeStep*/,
                                   double /*_withDampingForces*/,
                                   double /*_withSpringForces*/)
{
  // tau = f
  getState().mForces = mF;
  // TODO: need to add spring and damping forces
}

//==============================================================================
void PointMass::updateBiasForceFD(double _dt, const Eigen::Vector3d& _gravity)
{
  // B = w(parent) x m*v - fext - fgravity
  // - w(parent) x m*v - fext
  mB = mParentSoftBodyNode->getSpatialVelocity().head<3>().cross(
        getMass() * getBodyVelocity()) - mFext;
  // - fgravity
  if (mParentSoftBodyNode->getGravityMode() == true)
  {
    mB -= getMass()
          * (mParentSoftBodyNode->getWorldTransform().linear().transpose()
             * _gravity);
  }
  assert(!math::isNan(mB));

  const State& state = getState();

  // Cache data: alpha
  double kv = mParentSoftBodyNode->getVertexSpringStiffness();
  double ke = mParentSoftBodyNode->getEdgeSpringStiffness();
  double kd = mParentSoftBodyNode->getDampingCoefficient();
  int nN = getNumConnectedPointMasses();
  mAlpha = state.mForces
           - (kv + nN * ke) * getPositions()
           - (_dt * (kv + nN * ke) + kd) * getVelocities()
           - getMass() * getPartialAccelerations()
           - mB;
  for (std::size_t i = 0; i < getNumConnectedPointMasses(); ++i)
  {
    const State& i_state = getConnectedPointMass(i)->getState();
    mAlpha += ke * (i_state.mPositions + _dt * i_state.mVelocities);
  }
  assert(!math::isNan(mAlpha));

  // Cache data: beta
  mBeta = mB;
  mBeta.noalias() += getMass() * (getPartialAccelerations() + getImplicitPsi() * mAlpha);
  assert(!math::isNan(mBeta));
}

//==============================================================================
void PointMass::updateAccelerationFD()
{
  // ddq = imp_psi*(alpha - m*(dw(parent) x mX + dv(parent))
  const Eigen::Vector3d& X = getLocalPosition();
  const Eigen::Vector6d& a_parent = mParentSoftBodyNode->getSpatialAcceleration();
  Eigen::Vector3d ddq =
      getImplicitPsi()
      * (mAlpha - getMass() * (a_parent.head<3>().cross(X) + a_parent.tail<3>()));
  setAccelerations(ddq);
  assert(!math::isNan(ddq));

  // dv = dw(parent) x mX + dv(parent) + eata + ddq
  mA = a_parent.head<3>().cross(X) + a_parent.tail<3>()
       + getPartialAccelerations() + getAccelerations();
  assert(!math::isNan(mA));
}

//==============================================================================
void PointMass::updateTransmittedForce()
{
  // f = m*dv + B
  mF = mB;
  mF.noalias() += getMass() * getBodyAcceleration();
  assert(!math::isNan(mF));
}

//==============================================================================
void PointMass::updateMassMatrix()
{
  mM_dV = getAccelerations()
          + mParentSoftBodyNode->mM_dV.head<3>().cross(getLocalPosition())
          + mParentSoftBodyNode->mM_dV.tail<3>();
  assert(!math::isNan(mM_dV));
}

//==============================================================================
void PointMass::updateBiasImpulseFD()
{
  mImpB = -mConstraintImpulses;
  assert(!math::isNan(mImpB));

  // Cache data: alpha
  mImpAlpha = -mImpB;
  assert(!math::isNan(mImpAlpha));

  // Cache data: beta
  mImpBeta.setZero();
  assert(!math::isNan(mImpBeta));
}

//==============================================================================
void PointMass::updateVelocityChangeFD()
{
  //  Eigen::Vector3d del_dq
  //      = mPsi
  //        * (mImpAlpha - mMass
  //           * (mParentSoftBodyNode->getBodyVelocityChange().head<3>().cross(mX)
  //              + mParentSoftBodyNode->getBodyVelocityChange().tail<3>()));

  const Eigen::Vector3d& X = getLocalPosition();
  Eigen::Vector3d del_dq
      = getPsi() * mImpAlpha
        - mParentSoftBodyNode->getBodyVelocityChange().head<3>().cross(X)
        - mParentSoftBodyNode->getBodyVelocityChange().tail<3>();

  //  del_dq = Eigen::Vector3d::Zero();

  mVelocityChanges = del_dq;
  assert(!math::isNan(del_dq));

  mDelV = mParentSoftBodyNode->getBodyVelocityChange().head<3>().cross(X)
          + mParentSoftBodyNode->getBodyVelocityChange().tail<3>()
          + mVelocityChanges;
  assert(!math::isNan(mDelV));
}


//==============================================================================
void PointMass::updateTransmittedImpulse()
{
  mImpF = mImpB;
  mImpF.noalias() += getMass() * mDelV;
  assert(!math::isNan(mImpF));
}

//==============================================================================
void PointMass::updateConstrainedTermsFD(double _timeStep)
{
  // 1. dq = dq + del_dq
  setVelocities( getVelocities() + mVelocityChanges );

  // 2. ddq = ddq + del_dq / dt
  setAccelerations( getAccelerations() + mVelocityChanges / _timeStep );

  // 3. tau = tau + imp / dt
  getState().mForces.noalias() += mConstraintImpulses / _timeStep;

  ///
//  mA += mDelV / _timeStep;
  setAccelerations( getAccelerations() + mDelV / _timeStep );

  ///
  mF += _timeStep * mImpF;
}

//==============================================================================
void PointMass::aggregateMassMatrix(MatrixXd& /*_MCol*/, int /*_col*/)
{
  // TODO(JS): Not implemented
//  // Assign
//  // We assume that the three generalized coordinates are in a row.
//  int iStart = mIndexInSkeleton[0];
//  mM_F.noalias() = mMass * mM_dV;
//  _MCol->block<3, 1>(iStart, _col).noalias() = mM_F;
}

//==============================================================================
void PointMass::aggregateAugMassMatrix(Eigen::MatrixXd& /*_MCol*/, int /*_col*/,
                                       double /*_timeStep*/)
{
  // TODO(JS): Not implemented
//  // Assign
//  // We assume that the three generalized coordinates are in a row.
//  int iStart = mIndexInSkeleton[0];
//  mM_F.noalias() = mMass * mM_dV;

//  double d = mParentSoftBodyNode->getDampingCoefficient();
//  double kv = mParentSoftBodyNode->getVertexSpringStiffness();
//  _MCol->block<3, 1>(iStart, _col).noalias()
//      = mM_F + (_timeStep * _timeStep * kv + _timeStep * d) * mAccelerations;
}

//==============================================================================
void PointMass::updateInvMassMatrix()
{
  mBiasForceForInvMeta = getState().mForces;
}

//==============================================================================
void PointMass::updateInvAugMassMatrix()
{
//  mBiasForceForInvMeta = mMass * mImplicitPsi * mForces;
}

//==============================================================================
void PointMass::aggregateInvMassMatrix(Eigen::MatrixXd& /*_MInvCol*/,
                                       int /*_col*/)
{
  // TODO(JS): Not implemented
//  // Assign
//  // We assume that the three generalized coordinates are in a row.
//  int iStart = mIndexInSkeleton[0];
//  _MInvCol->block<3, 1>(iStart, _col)
//      = mPsi * mForces
//        - mParentSoftBodyNode->mInvM_U.head<3>().cross(mX)
//        - mParentSoftBodyNode->mInvM_U.tail<3>();
}

//==============================================================================
void PointMass::aggregateInvAugMassMatrix(Eigen::MatrixXd& /*_MInvCol*/,
                                          int /*_col*/,
                                          double /*_timeStep*/)
{
  // TODO(JS): Not implemented
//  // Assign
//  // We assume that the three generalized coordinates are in a row.
//  int iStart = mIndexInSkeleton[0];
//  _MInvCol->block<3, 1>(iStart, _col)
//      = mImplicitPsi
//        * (mForces
//           - mMass * (mParentSoftBodyNode->mInvM_U.head<3>().cross(mX)
//                      + mParentSoftBodyNode->mInvM_U.tail<3>()));
}

//==============================================================================
void PointMass::aggregateGravityForceVector(VectorXd& /*_g*/,
                                            const Eigen::Vector3d& /*_gravity*/)
{
  // TODO(JS): Not implemented
//  mG_F = mMass * (mParentSoftBodyNode->getWorldTransform().linear().transpose()
//                  * _gravity);

//  // Assign
//  // We assume that the three generalized coordinates are in a row.
//  int iStart = mIndexInSkeleton[0];
//  _g->segment<3>(iStart) = mG_F;
}

//==============================================================================
void PointMass::updateCombinedVector()
{
  mCg_dV = getPartialAccelerations()
           + mParentSoftBodyNode->mCg_dV.head<3>().cross(getLocalPosition())
           + mParentSoftBodyNode->mCg_dV.tail<3>();
}

//==============================================================================
void PointMass::aggregateCombinedVector(Eigen::VectorXd& /*_Cg*/,
                                        const Eigen::Vector3d& /*_gravity*/)
{
  // TODO(JS): Not implemented
//  mCg_F.noalias() = mMass * mCg_dV;
//  mCg_F -= mMass
//           * (mParentSoftBodyNode->getWorldTransform().linear().transpose()
//              * _gravity);
//  mCg_F += mParentSoftBodyNode->getBodyVelocity().head<3>().cross(mMass * mV);

//  // Assign
//  // We assume that the three generalized coordinates are in a row.
//  int iStart = mIndexInSkeleton[0];
//  _Cg->segment<3>(iStart) = mCg_F;
}

//==============================================================================
void PointMass::aggregateExternalForces(VectorXd& /*_Fext*/)
{
  // TODO(JS): Not implemented
//  int iStart = mIndexInSkeleton[0];
//  _Fext->segment<3>(iStart) = mFext;
}

//==============================================================================
PointMassNotifier::PointMassNotifier(SoftBodyNode* _parentSoftBody,
                                     const std::string& _name)
  : Entity(_parentSoftBody, false),
    mNeedPartialAccelerationUpdate(true),
    mParentSoftBodyNode(_parentSoftBody)
{
  setName(_name);
}

//==============================================================================
bool PointMassNotifier::needsPartialAccelerationUpdate() const
{
  return mNeedPartialAccelerationUpdate;
}

//==============================================================================
void PointMassNotifier::clearTransformNotice()
{
  mNeedTransformUpdate = false;
}

//==============================================================================
void PointMassNotifier::clearVelocityNotice()
{
  mNeedVelocityUpdate = false;
}

//==============================================================================
void PointMassNotifier::clearPartialAccelerationNotice()
{
  mNeedPartialAccelerationUpdate = false;
}

//==============================================================================
void PointMassNotifier::clearAccelerationNotice()
{
  mNeedAccelerationUpdate = false;
}

//==============================================================================
void PointMassNotifier::notifyTransformUpdate()
{
  mNeedTransformUpdate = true;
  mNeedVelocityUpdate = true;
  mNeedPartialAccelerationUpdate = true;
  mNeedAccelerationUpdate = true;

  mParentSoftBodyNode->notifyArticulatedInertiaUpdate();
  mParentSoftBodyNode->notifyExternalForcesUpdate();
}

//==============================================================================
void PointMassNotifier::notifyVelocityUpdate()
{
  mNeedVelocityUpdate = true;
  mNeedPartialAccelerationUpdate = true;
  mNeedAccelerationUpdate = true;

  mParentSoftBodyNode->notifyCoriolisUpdate();
}

//==============================================================================
void PointMassNotifier::notifyAccelerationUpdate()
{
  mNeedAccelerationUpdate = true;
}

//==============================================================================
const std::string& PointMassNotifier::setName(const std::string& _name)
{
  if(_name == mName)
    return mName;

  const std::string oldName = mName;

  mName = _name;

  Entity::mNameChangedSignal.raise(this, oldName, mName);

  return mName;
}

//==============================================================================
const std::string& PointMassNotifier::getName() const
{
  return mName;
}

}  // namespace dynamics
}  // namespace dart
