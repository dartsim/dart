/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/PointMass.h"

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/renderer/RenderInterface.h"

#include "dart/dynamics/SoftBodyNode.h"

using namespace Eigen;

namespace dart {
namespace dynamics {

//==============================================================================
PointMass::PointMass(SoftBodyNode* _softBodyNode)
  : // mIndexInSkeleton(Eigen::Matrix<size_t, 3, 1>::Zero()),
    mPositions(Eigen::Vector3d::Zero()),
    mPositionLowerLimits(Eigen::Vector3d::Constant(-DART_DBL_INF)),
    mPositionUpperLimits(Eigen::Vector3d::Constant(DART_DBL_INF)),
    mPositionDeriv(Eigen::Vector3d::Zero()),
    mVelocities(Eigen::Vector3d::Zero()),
    mVelocityLowerLimits(Eigen::Vector3d::Constant(-DART_DBL_INF)),
    mVelocityUpperLimits(Eigen::Vector3d::Constant(DART_DBL_INF)),
    mVelocitiesDeriv(Eigen::Vector3d::Zero()),
    mAccelerations(Eigen::Vector3d::Zero()),
    mAccelerationLowerLimits(Eigen::Vector3d::Constant(-DART_DBL_INF)),
    mAccelerationUpperLimits(Eigen::Vector3d::Constant(DART_DBL_INF)),
    mAccelerationsDeriv(Eigen::Vector3d::Zero()),
    mForces(Eigen::Vector3d::Zero()),
    mForceLowerLimits(Eigen::Vector3d::Constant(-DART_DBL_INF)),
    mForceUpperLimits(Eigen::Vector3d::Constant(DART_DBL_INF)),
    mForcesDeriv(Eigen::Vector3d::Zero()),
    mVelocityChanges(Eigen::Vector3d::Zero()),
    // mImpulse(Eigen::Vector3d::Zero()),
    mConstraintImpulses(Eigen::Vector3d::Zero()),
    mMass(0.0005),
    mW(Eigen::Vector3d::Zero()),
    mX(Eigen::Vector3d::Zero()),
    mX0(Eigen::Vector3d::Zero()),
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
    mParentSoftBodyNode(_softBodyNode),
    mFext(Eigen::Vector3d::Zero()),
    mIsColliding(false),
    mDelV(Eigen::Vector3d::Zero()),
    mImpB(Eigen::Vector3d::Zero()),
    mImpAlpha(Eigen::Vector3d::Zero()),
    mImpBeta(Eigen::Vector3d::Zero()),
    mImpF(Eigen::Vector3d::Zero()),
    mShape(new EllipsoidShape(Eigen::Vector3d(0.01, 0.01, 0.01)))
{
  assert(mParentSoftBodyNode != NULL);
}

//==============================================================================
PointMass::~PointMass()
{
  delete mShape;
}

//==============================================================================
void PointMass::setMass(double _mass)
{
  assert(0.0 < _mass);
  mMass = _mass;
}

//==============================================================================
double PointMass::getMass() const
{
  return mMass;
}

//==============================================================================
void PointMass::addConnectedPointMass(PointMass* _pointMass)
{
  assert(_pointMass != NULL);

  mConnectedPointMasses.push_back(_pointMass);
}

//==============================================================================
int PointMass::getNumConnectedPointMasses() const
{
  return mConnectedPointMasses.size();
}

//==============================================================================
PointMass* PointMass::getConnectedPointMass(size_t _idx)
{
  assert(0 <= _idx && _idx < mConnectedPointMasses.size());

  return mConnectedPointMasses[_idx];
}

//==============================================================================
const PointMass* PointMass::getConnectedPointMass(size_t _idx) const
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
size_t PointMass::getNumDofs() const
{
  return 3;
}

////==============================================================================
//void PointMass::setIndexInSkeleton(size_t _index, size_t _indexInSkeleton)
//{
//  assert(_index < 3);

//  mIndexInSkeleton[_index] = _indexInSkeleton;
//}

////==============================================================================
//size_t PointMass::getIndexInSkeleton(size_t _index) const
//{
//  assert(_index < 3);

//  return mIndexInSkeleton[_index];
//}

//==============================================================================
void PointMass::setPosition(size_t _index, double _position)
{
  assert(_index < 3);

  mPositions[_index] = _position;
}

//==============================================================================
double PointMass::getPosition(size_t _index) const
{
  assert(_index < 3);

  return mPositions[_index];
}

//==============================================================================
void PointMass::setPositions(const Vector3d& _positions)
{
  mPositions = _positions;
}

//==============================================================================
const Vector3d& PointMass::getPositions() const
{
  return mPositions;
}

//==============================================================================
void PointMass::resetPositions()
{
  mPositions.setZero();
}

//==============================================================================
void PointMass::setVelocity(size_t _index, double _velocity)
{
  assert(_index < 3);

  mVelocities[_index] = _velocity;
}

//==============================================================================
double PointMass::getVelocity(size_t _index) const
{
  assert(_index < 3);

  return mVelocities[_index];
}

//==============================================================================
void PointMass::setVelocities(const Vector3d& _velocities)
{
  mVelocities = _velocities;
}

//==============================================================================
const Vector3d& PointMass::getVelocities() const
{
  return mVelocities;
}

//==============================================================================
void PointMass::resetVelocities()
{
  mVelocities.setZero();
}

//==============================================================================
void PointMass::setAcceleration(size_t _index, double _acceleration)
{
  assert(_index < 3);

  mAccelerations[_index] = _acceleration;
}

//==============================================================================
double PointMass::getAcceleration(size_t _index) const
{
 assert(_index < 3);

 return mAccelerations[_index];
}

//==============================================================================
void PointMass::setAccelerations(const Eigen::Vector3d& _accelerations)
{
  mAccelerations = _accelerations;
}

//==============================================================================
const Vector3d& PointMass::getAccelerations() const
{
  return mAccelerations;
}

//==============================================================================
void PointMass::resetAccelerations()
{
  mAccelerations.setZero();
}

//==============================================================================
void PointMass::setForce(size_t _index, double _force)
{
  assert(_index < 3);

  mForces[_index] = _force;
}

//==============================================================================
double PointMass::getForce(size_t _index)
{
  assert(_index < 3);

  return mForces[_index];
}

//==============================================================================
void PointMass::setForces(const Vector3d& _forces)
{
  mForces = _forces;
}

//==============================================================================
const Vector3d& PointMass::getForces() const
{
  return mForces;
}

//==============================================================================
void PointMass::resetForces()
{
  mForces.setZero();
}

//==============================================================================
void PointMass::setVelocityChange(size_t _index, double _velocityChange)
{
  assert(_index < 3);

  mVelocityChanges[_index] = _velocityChange;
}

//==============================================================================
double PointMass::getVelocityChange(size_t _index)
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
void PointMass::setConstraintImpulse(size_t _index, double _impulse)
{
  assert(_index < 3);

  mConstraintImpulses[_index] = _impulse;
}

//==============================================================================
double PointMass::getConstraintImpulse(size_t _index)
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
  mPositions += mVelocities * _dt;
}

//==============================================================================
void PointMass::integrateVelocities(double _dt)
{
  mVelocities += mAccelerations * _dt;
}

//==============================================================================
void PointMass::updateVelocityWithVelocityChange()
{
  mVelocities += mVelocityChanges;
}

//==============================================================================
void PointMass::updateAccelerationWithVelocityChange(double _timeStep)
{
  mAccelerations.noalias() += mVelocityChanges / _timeStep;
}

//==============================================================================
void PointMass::updateForceWithImpulse(double _timeStep)
{
  mForces.noalias() += mConstraintImpulses / _timeStep;
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
    mFext += mParentSoftBodyNode->getTransform().linear().transpose()
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
        = mParentSoftBodyNode->getTransform().linear().transpose();
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
        = mParentSoftBodyNode->getTransform().linear().transpose();
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
  mX0 = _p;
}

//==============================================================================
const Eigen::Vector3d& PointMass::getRestingPosition() const
{
  return mX0;
}

//==============================================================================
const Eigen::Vector3d& PointMass::getLocalPosition() const
{
  return mX;
}

//==============================================================================
const Eigen::Vector3d& PointMass::getWorldPosition() const
{
  return mW;
}

//==============================================================================
Eigen::Matrix<double, 3, Eigen::Dynamic> PointMass::getBodyJacobian()
{
  assert(mParentSoftBodyNode != NULL);

  int dof = mParentSoftBodyNode->getNumDependentGenCoords();
  int totalDof = mParentSoftBodyNode->getNumDependentGenCoords() + 3;

  Eigen::Matrix<double, 3, Eigen::Dynamic> J
      = Eigen::MatrixXd::Zero(3, totalDof);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = mX;

  J.leftCols(dof)
      = math::AdInvTJac(
          T, mParentSoftBodyNode->getBodyJacobian()).bottomRows<3>();
  J.rightCols<3>() = Eigen::Matrix3d::Identity();

  return J;
}

//==============================================================================
Eigen::Matrix<double, 3, Eigen::Dynamic> PointMass::getWorldJacobian()
{
  return mParentSoftBodyNode->getTransform().linear()
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
  return mV;
}

//==============================================================================
Eigen::Vector3d PointMass::getWorldVelocity() const
{
  return mParentSoftBodyNode->getTransform().linear() * mV;
}

//==============================================================================
const Eigen::Vector3d& PointMass::getBodyAcceleration() const
{
  return mA;
}

//==============================================================================
Eigen::Vector3d PointMass::getWorldAcceleration() const
{
  return mParentSoftBodyNode->getTransform().linear() * mA;
}

//==============================================================================
void PointMass::init()
{
  // Dependen generalized coordinate setting
  int parentDof = mParentSoftBodyNode->getNumDependentGenCoords();

  mDependentGenCoordIndices.resize(parentDof + 3);
  for (int i = 0; i < parentDof; ++i)
  {
    mDependentGenCoordIndices[i]
        = mParentSoftBodyNode->getDependentGenCoordIndex(i);
  }

//  mDependentGenCoordIndices[parentDof]     = mIndexInSkeleton[0];
//  mDependentGenCoordIndices[parentDof + 1] = mIndexInSkeleton[1];
//  mDependentGenCoordIndices[parentDof + 2] = mIndexInSkeleton[2];
}

//==============================================================================
void PointMass::updateTransform()
{
  // Local transpose
  mX = mPositions + mX0;
  assert(!math::isNan(mX));

  // World transpose
  mW = mParentSoftBodyNode->getTransform().translation()
       + mParentSoftBodyNode->getTransform().linear() * mX;
  assert(!math::isNan(mW));
}

//==============================================================================
void PointMass::updateVelocity()
{
  // v = w(parent) x mX + v(parent) + dq
  mV = mParentSoftBodyNode->getBodyVelocity().head<3>().cross(mX)
       + mParentSoftBodyNode->getBodyVelocity().tail<3>()
       + mVelocities;
  assert(!math::isNan(mV));
}

//==============================================================================
void PointMass::updatePartialAcceleration()
{
  // eta = w(parent) x dq
  Eigen::Vector3d dq = mVelocities;
  mEta = mParentSoftBodyNode->getBodyVelocity().head<3>().cross(dq);
  assert(!math::isNan(mEta));
}

//==============================================================================
void PointMass::updateAcceleration()
{
  updateAccelerationID();
}

//==============================================================================
void PointMass::updateAccelerationID()
{
  // dv = dw(parent) x mX + dv(parent) + eata + ddq
  mA = mParentSoftBodyNode->getBodyAcceleration().head<3>().cross(mX) +
        mParentSoftBodyNode->getBodyAcceleration().tail<3>() +
        mEta + mAccelerations;
  assert(!math::isNan(mA));
}

//==============================================================================
void PointMass::updateBodyForce(const Eigen::Vector3d& _gravity,
                                bool _withExternalForces)
{
  updateTransmittedForceID(_gravity, _withExternalForces);
}

//==============================================================================
void PointMass::updateTransmittedForceID(const Eigen::Vector3d& _gravity,
                                bool _withExternalForces)
{
  // f = m*dv + w(parent) x m*v - fext
  mF.noalias() = mMass * mA;
  mF += mParentSoftBodyNode->getBodyVelocity().head<3>().cross(mMass * mV)
        - mFext;
  if (mParentSoftBodyNode->getGravityMode() == true)
  {
    mF -= mMass * (mParentSoftBodyNode->getTransform().linear().transpose()
                   * _gravity);
  }
  assert(!math::isNan(mF));
}

//==============================================================================
void PointMass::updateArticulatedInertia(double _dt)
{
  updateArtInertiaFD(_dt);
}

//==============================================================================
void PointMass::updateArtInertiaFD(double _timeStep)
{
  // Articulated inertia
  // - Do nothing

  // Cache data: PsiK and Psi
  mPsi = 1.0 / mMass;
  mImplicitPsi
      = 1.0 / (mMass
               + _timeStep * mParentSoftBodyNode->getDampingCoefficient()
               + _timeStep * _timeStep
                 * mParentSoftBodyNode->getVertexSpringStiffness());
  assert(!math::isNan(mImplicitPsi));

  // Cache data: AI_S_Psi
  // - Do nothing

  // Cache data: Pi
  mPi         = mMass - mMass * mMass * mPsi;
  mImplicitPi = mMass - mMass * mMass * mImplicitPsi;
  assert(!math::isNan(mPi));
  assert(!math::isNan(mImplicitPi));
}

//==============================================================================
void PointMass::updateGeneralizedForce(bool /*_withDampingForces*/)
{
  updateJointForceID(false, false, false);
}

//==============================================================================
void PointMass::updateJointForceID(double /*_timeStep*/,
                                   double /*_withDampingForces*/,
                                   double /*_withSpringForces*/)
{
  // tau = f
  mForces = mF;
  // TODO: need to add spring and damping forces
}

//==============================================================================
void PointMass::updateBiasForceFD(double _dt, const Eigen::Vector3d& _gravity)
{
  // B = w(parent) x m*v - fext - fgravity
  // - w(parent) x m*v - fext
  mB = mParentSoftBodyNode->getBodyVelocity().head<3>().cross(mMass*mV) - mFext;
  // - fgravity
  if (mParentSoftBodyNode->getGravityMode() == true)
  {
    mB -= mMass
          * (mParentSoftBodyNode->getTransform().linear().transpose()
             * _gravity);
  }
  assert(!math::isNan(mB));

  // Cache data: alpha
  double kv = mParentSoftBodyNode->getVertexSpringStiffness();
  double ke = mParentSoftBodyNode->getEdgeSpringStiffness();
  double kd = mParentSoftBodyNode->getDampingCoefficient();
  int nN = mConnectedPointMasses.size();
  mAlpha = mForces
           - (kv + nN * ke) * mPositions
           - (_dt * (kv + nN * ke) + kd) * mVelocities
           - mMass * mEta
           - mB;
  for (size_t i = 0; i < mConnectedPointMasses.size(); ++i)
  {
    mAlpha += ke * (mConnectedPointMasses[i]->mPositions
                    + _dt * mConnectedPointMasses[i]->mVelocities);
  }
  assert(!math::isNan(mAlpha));

  // Cache data: beta
  mBeta = mB;
  mBeta.noalias() += mMass * (mEta + mImplicitPsi * mAlpha);
  assert(!math::isNan(mBeta));
}

//==============================================================================
void PointMass::updateJointAndBodyAcceleration()
{
  updateAccelerationFD();
}

//==============================================================================
void PointMass::updateAccelerationFD()
{
  // ddq = imp_psi*(alpha - m*(dw(parent) x mX + dv(parent))
  Eigen::Vector3d ddq =
      mImplicitPsi
      * (mAlpha - mMass
         * (mParentSoftBodyNode->getBodyAcceleration().head<3>().cross(mX)
            + mParentSoftBodyNode->getBodyAcceleration().tail<3>()));
  setAccelerations(ddq);
  assert(!math::isNan(ddq));

  // dv = dw(parent) x mX + dv(parent) + eata + ddq
  mA = mParentSoftBodyNode->getBodyAcceleration().head<3>().cross(mX) +
        mParentSoftBodyNode->getBodyAcceleration().tail<3>() +
        mEta + mAccelerations;
  assert(!math::isNan(mA));
}

//==============================================================================
void PointMass::updateTransmittedForce()
{
  // f = m*dv + B
  mF = mB;
  mF.noalias() += mMass * mA;
  assert(!math::isNan(mF));
}

//==============================================================================
void PointMass::updateMassMatrix()
{
  mM_dV = mAccelerations
          + mParentSoftBodyNode->mM_dV.head<3>().cross(mX)
          + mParentSoftBodyNode->mM_dV.tail<3>();
  assert(!math::isNan(mM_dV));
}

//==============================================================================
void PointMass::updateBiasImpulse()
{
  updateBiasImpulseFD();
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
void PointMass::updateJointVelocityChange()
{
  updateVelocityChangeFD();
}

//==============================================================================
void PointMass::updateVelocityChangeFD()
{
  //  Eigen::Vector3d del_dq
  //      = mPsi
  //        * (mImpAlpha - mMass
  //           * (mParentSoftBodyNode->getBodyVelocityChange().head<3>().cross(mX)
  //              + mParentSoftBodyNode->getBodyVelocityChange().tail<3>()));

  Eigen::Vector3d del_dq
      = mPsi * mImpAlpha
        - mParentSoftBodyNode->getBodyVelocityChange().head<3>().cross(mX)
        - mParentSoftBodyNode->getBodyVelocityChange().tail<3>();

  //  del_dq = Eigen::Vector3d::Zero();

  mVelocityChanges = del_dq;
  assert(!math::isNan(del_dq));

  mDelV = mParentSoftBodyNode->getBodyVelocityChange().head<3>().cross(mX)
          + mParentSoftBodyNode->getBodyVelocityChange().tail<3>()
          + mVelocityChanges;
  assert(!math::isNan(mDelV));
}

//==============================================================================
void PointMass::updateBodyVelocityChange()
{
  mDelV = mParentSoftBodyNode->getBodyVelocityChange().head<3>().cross(mX)
          + mParentSoftBodyNode->getBodyVelocityChange().tail<3>()
          + mVelocityChanges;
  assert(!math::isNan(mDelV));
}

//==============================================================================
void PointMass::updateBodyImpForceFwdDyn()
{
  updateTransmittedImpulse();
}

//==============================================================================
void PointMass::updateTransmittedImpulse()
{
  mImpF = mImpB;
  mImpF.noalias() += mMass * mDelV;
  assert(!math::isNan(mImpF));
}

//==============================================================================
void PointMass::updateConstrainedJointAndBodyAcceleration(double _timeStep)
{
//  // 1. dq = dq + del_dq
//  updateVelocityWithVelocityChange();

//  // 2. ddq = ddq + del_dq / dt
//  updateAccelerationWithVelocityChange(_timeStep);

//  // 3. tau = tau + imp / dt
//  updateForceWithImpulse(_timeStep);
}

//==============================================================================
void PointMass::updateConstrainedTransmittedForce(double _timeStep)
{
  ///
  mA += mDelV / _timeStep;

  ///
  mF += _timeStep * mImpF;
}

//==============================================================================
void PointMass::updateConstrainedTermsFD(double _timeStep)
{
  // 1. dq = dq + del_dq
  mVelocities += mVelocityChanges;

  // 2. ddq = ddq + del_dq / dt
  mAccelerations.noalias() += mVelocityChanges / _timeStep;

  // 3. tau = tau + imp / dt
  mForces.noalias() += mConstraintImpulses / _timeStep;

  ///
  mA += mDelV / _timeStep;

  ///
  mF += _timeStep * mImpF;
}

//==============================================================================
void PointMass::aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col)
{
//  // Assign
//  // We assume that the three generalized coordinates are in a row.
//  int iStart = mIndexInSkeleton[0];
//  mM_F.noalias() = mMass * mM_dV;
//  _MCol->block<3, 1>(iStart, _col).noalias() = mM_F;
}

//==============================================================================
void PointMass::aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                       double _timeStep)
{
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
  mBiasForceForInvMeta = mForces;
}

//==============================================================================
void PointMass::updateInvAugMassMatrix()
{
//  mBiasForceForInvMeta = mMass * mImplicitPsi * mForces;
}

//==============================================================================
void PointMass::aggregateInvMassMatrix(Eigen::MatrixXd* _MInvCol, int _col)
{
//  // Assign
//  // We assume that the three generalized coordinates are in a row.
//  int iStart = mIndexInSkeleton[0];
//  _MInvCol->block<3, 1>(iStart, _col)
//      = mPsi * mForces
//        - mParentSoftBodyNode->mInvM_U.head<3>().cross(mX)
//        - mParentSoftBodyNode->mInvM_U.tail<3>();
}

//==============================================================================
void PointMass::aggregateInvAugMassMatrix(Eigen::MatrixXd* _MInvCol, int _col,
                                          double _timeStep)
{
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
void PointMass::aggregateGravityForceVector(Eigen::VectorXd* _g,
                                            const Eigen::Vector3d& _gravity)
{
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
  mCg_dV = mEta
           + mParentSoftBodyNode->mCg_dV.head<3>().cross(mX)
           + mParentSoftBodyNode->mCg_dV.tail<3>();
}

//==============================================================================
void PointMass::aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                        const Eigen::Vector3d& _gravity)
{
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
void PointMass::aggregateExternalForces(Eigen::VectorXd* _Fext)
{
//  int iStart = mIndexInSkeleton[0];
//  _Fext->segment<3>(iStart) = mFext;
}

//==============================================================================
void PointMass::draw(renderer::RenderInterface* _ri,
                     const Eigen::Vector4d& _color,
                     bool _useDefaultColor) const
{
  if (_ri == NULL)
    return;

  _ri->pushMatrix();

  // render the self geometry
  //  mParentJoint->applyGLTransform(_ri);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = mX;
  _ri->transform(T);
  Eigen::Vector4d color1;
  color1 << 0.8, 0.3, 0.3, 1.0;
  mShape->draw(_ri, color1, false);
  _ri->popMatrix();

  //  _ri->pushName((unsigned)mID);
  _ri->pushMatrix();
  T.translation() = mX0;
  _ri->transform(T);
  Eigen::Vector4d color2;
  color2 << 0.3, 0.8, 0.3, 1.0;
  mShape->draw(_ri, color2, false);
  _ri->popMatrix();
  //  _ri->popName();

}

}  // namespace dynamics
}  // namespace dart
