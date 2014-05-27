/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include "dart/dynamics/SingleDofJoint.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
SingleDofJoint::SingleDofJoint(const std::string& _name)
  : Joint(_name),
    mJacobian(Eigen::Vector6d::Zero()),
    mJacobianDeriv(Eigen::Vector6d::Zero()),
    mInvProjArtInertia(0.0),
    mInvProjArtInertiaImplicit(0.0),
    mTotalForce(0.0),
    mTotalImpulse(0.0)
{
  mGenCoords.push_back(&mCoordinate);

  // TODO(JS): Deprecated
  mdS = mJacobianDeriv;

  mSpringStiffness.resize(1, 0.0);
  mDampingCoefficient.resize(1, 0.0);
  mRestPosition.resize(1, 0.0);
}

//==============================================================================
SingleDofJoint::~SingleDofJoint()
{
}

//==============================================================================
void SingleDofJoint::addVelocityTo(Eigen::Vector6d& _vel)
{
  // Add joint velocity to _vel
  _vel += mJacobian * mCoordinate.getVel();

  // Verification
  assert(!math::isNan(_vel));
}

//==============================================================================
void SingleDofJoint::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& _childVelocity)
{
  // ad(V, S * dq)
  _partialAcceleration
      = math::ad(_childVelocity, mJacobian * mCoordinate.getVel());

  // Verification
  assert(!math::isNan(_partialAcceleration));

  // Add joint acceleration
  _partialAcceleration.noalias() += mJacobianDeriv * mCoordinate.getVel();

  // Verification
  assert(!math::isNan(_partialAcceleration));
}


//==============================================================================
void SingleDofJoint::addAccelerationTo(Eigen::Vector6d& _acc)
{
  //
  _acc += mJacobian * mCoordinate.getAcc();
}

//==============================================================================
void SingleDofJoint::addVelocityChangeTo(Eigen::Vector6d& _velocityChange)
{
  //
  _velocityChange += mJacobian * mCoordinate.getVelChange();
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Vector6d AIS = _childArtInertia * mJacobian;
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= mInvProjArtInertia * AIS * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(mT.inverse(), PI);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Vector6d AIS = _childArtInertia * mJacobian;
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= mInvProjArtInertiaImplicit * AIS * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(mT.inverse(), PI);
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertia(const Eigen::Matrix6d& _artInertia)
{
  // Projected articulated inertia
  double projAI = mJacobian.dot(_artInertia * mJacobian);

  // Inversion of projected articulated inertia
  mInvProjArtInertia = 1.0 / projAI;

  // Verification
  assert(!math::isNan(mInvProjArtInertia));
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  // Projected articulated inertia
  double projAI = mJacobian.dot(_artInertia * mJacobian);

  // Add additional inertia for implicit damping and spring force
  projAI += _timeStep * mDampingCoefficient[0]
            + _timeStep * _timeStep * mSpringStiffness[0];

  // Inversion of the projected articulated inertia for implicit damping and
  // spring force
  mInvProjArtInertiaImplicit = 1.0 / projAI;

  // Verification
  assert(!math::isNan(mInvProjArtInertiaImplicit));
}

//==============================================================================
void SingleDofJoint::addChildBiasForceTo(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
//  Eigen::Vector6d beta
//      = _childBiasForce
//        + _childArtInertia
//          * (_childPartialAcc
//             + mJacobian * mInvProjArtInertiaImplicit * mTotalForce);

  Eigen::Vector6d beta
      = _childBiasForce;
  beta.noalias() += _childArtInertia * _childPartialAcc;
  beta.noalias() += _childArtInertia *  mJacobian * mInvProjArtInertiaImplicit * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Compute beta
  Eigen::Vector6d beta
      = _childBiasImpulse
        + _childArtInertia * mJacobian * mInvProjArtInertia * mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::updateTotalForce(
    const Eigen::Vector6d& _bodyForce,
    double _timeStep)
{
  // Spring force
  double springForce
      = -mSpringStiffness[0] * (mCoordinate.getPos()
                                + mCoordinate.getVel() * _timeStep
                                - mRestPosition[0]);

  // Damping force
  double dampingForce = -mDampingCoefficient[0] * mCoordinate.getVel();

  // Compute alpha
  mTotalForce = mCoordinate.getForce()
                + springForce
                + dampingForce
                - mJacobian.dot(_bodyForce);
}

//==============================================================================
void SingleDofJoint::updateTotalImpulse(const Eigen::Vector6d& _bodyImpulse)
{
  //
  mTotalImpulse = mCoordinate.getConstraintImpulse()
                  - mJacobian.dot(_bodyImpulse);
}

//==============================================================================
void SingleDofJoint::updateAcceleration(const Eigen::Matrix6d& _artInertia,
                                       const Eigen::Vector6d& _spatialAcc)
{
  //
  double acc
      = mInvProjArtInertiaImplicit
        * (mTotalForce
           - mJacobian.dot(_artInertia * math::AdInvT(mT, _spatialAcc)));

  // Verification
  assert(!math::isNan(acc));

  //
  mCoordinate.setAcc(acc);
}

//==============================================================================
void SingleDofJoint::updateVelocityChange(const Eigen::Matrix6d& _artInertia,
                                         const Eigen::Vector6d& _velocityChange)
{
  //
  double delVel
      = mInvProjArtInertia
        * (mTotalImpulse
           - mJacobian.dot(_artInertia * math::AdInvT(mT, _velocityChange)));

  // Verification
  assert(!math::isNan(delVel));

  //
  mCoordinate.setVelChange(delVel);
}

//==============================================================================
void SingleDofJoint::clearConstraintImpulse()
{
  mTotalImpulse = 0.0;
}

//==============================================================================
void SingleDofJoint::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  std::cout << "!! " << std::endl;
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia *  mJacobian * mInvProjArtInertia
                    * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  std::cout << "aa " << std::endl;
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia *  mJacobian * mInvProjArtInertiaImplicit
                    * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& _bodyForce)
{
  // Compute alpha
  mInvM_a = mCoordinate.getForce() - mJacobian.dot(_bodyForce);
}

//==============================================================================
void SingleDofJoint::getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                             const size_t _col,
                                             const Eigen::Matrix6d& _artInertia,
                                             const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = mInvProjArtInertia
        * (mInvM_a
           - mJacobian.dot(_artInertia * math::AdInvT(mT, _spatialAcc)));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mCoordinate.getSkeletonIndex();

  // Assign
  _invMassMat(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SingleDofJoint::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = mInvProjArtInertiaImplicit
        * (mInvM_a
           - mJacobian.dot(_artInertia * math::AdInvT(mT, _spatialAcc)));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mCoordinate.getSkeletonIndex();

  // Assign
  _invMassMat(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SingleDofJoint::addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
{
  //
  _acc += mJacobian * mInvMassMatrixSegment;
}

}  // namespace dynamics
}  // namespace dart
