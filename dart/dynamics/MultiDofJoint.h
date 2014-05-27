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

#ifndef DART_DYNAMICS_MULTIDOFJOINT_H_
#define DART_DYNAMICS_MULTIDOFJOINT_H_

#include <iostream>
#include <string>

#include "dart/math/Helpers.h"
#include "dart/dynamics/Joint.h"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

/// class MultiDofJoint
template<size_t DOF>
class MultiDofJoint : public Joint
{
public:
  /// Constructor
  MultiDofJoint(const std::string& _name)
    : Joint(_name),
      mJacobian(Eigen::Matrix<double, 6, DOF>::Zero()),
      mJacobianDeriv(Eigen::Matrix<double, 6, DOF>::Zero()),
      mInvProjArtInertia(Eigen::Matrix<double, DOF, DOF>::Zero()),
      mInvProjArtInertiaImplicit(Eigen::Matrix<double, DOF, DOF>::Zero()),
      mTotalForce(Eigen::Matrix<double, DOF, 1>::Zero()),
      mTotalImpulse(Eigen::Matrix<double, DOF, 1>::Zero())
  {
    for (size_t i = 0; i < DOF; ++i)
      mGenCoords.push_back(&mCoordinate[i]);

    mSpringStiffness.resize(DOF, 0.0);
    mDampingCoefficient.resize(DOF, 0.0);
    mRestPosition.resize(DOF, 0.0);
  }

  /// Constructor
  ~MultiDofJoint() {}

protected:
  //----------------------------------------------------------------------------
  // Recursive dynamics algorithms
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void addVelocityTo(Eigen::Vector6d& _vel)
  {
    // Add joint velocity to _vel
    for (size_t i = 0; i < DOF; ++i)
      _vel.noalias() += mJacobian.col(i) * mCoordinate[i].getVel();

    // Verification
    assert(!math::isNan(_vel));
  }

  // Documentation inherited
  void setPartialAccelerationTo(Eigen::Vector6d& _partialAcceleration,
                                const Eigen::Vector6d& _childVelocity)
  {
    // ad(V, S * dq)
    _partialAcceleration
        = math::ad(_childVelocity, mJacobian * GenCoordSystem::getGenVels());
        //= math::ad(_childVelocity, mJacobian * mCoordinate.getVel());

    // Add joint acceleration
//    _partialAcceleration.noalias() += mJacobianDeriv * GenCoordSystem::getGenVels();
    //_partialAcceleration.noalias() += mJacobianDeriv * mCoordinate.getVel();
    for (size_t i = 0; i < DOF; ++i)
    {
      _partialAcceleration.noalias()
          += mJacobianDeriv.col(i) * mCoordinate[i].getVel();
    }

    // Verification
    assert(!math::isNan(_partialAcceleration));
  }

  // Documentation inherited
  void addAccelerationTo(Eigen::Vector6d& _acc)
  {
    // Add joint acceleration to _acc
    for (size_t i = 0; i < DOF; ++i)
      _acc.noalias() += mJacobian.col(i) * mCoordinate[i].getAcc();

    // Verification
    assert(!math::isNan(_acc));
  }

  // Documentation inherited
  void addVelocityChangeTo(Eigen::Vector6d& _velocityChange)
  {
    // Add joint velocity change to _velocityChange
    for (size_t i = 0; i < DOF; ++i)
    {
      _velocityChange.noalias()
          += mJacobian.col(i) * mCoordinate[i].getVelChange();
    }

    // Verification
    assert(!math::isNan(_velocityChange));
  }

  // Documentation inherited
  void addChildArtInertiaTo(Eigen::Matrix6d& _parentArtInertia,
                            const Eigen::Matrix6d& _childArtInertia)
  {
    // Child body's articulated inertia
    Eigen::Matrix<double, 6, DOF> AIS = _childArtInertia * mJacobian;
    Eigen::Matrix6d PI = _childArtInertia;
    PI.noalias() -= AIS * mInvProjArtInertia * AIS.transpose();
    assert(!math::isNan(PI));

    // Add child body's articulated inertia to parent body's articulated inertia.
    // Note that mT should be updated.
    _parentArtInertia += math::transformInertia(mT.inverse(), PI);
  }

  // Documentation inherited
  void addChildArtInertiaImplicitTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia)
  {
    // Child body's articulated inertia
    Eigen::Matrix<double, 6, DOF> AIS = _childArtInertia * mJacobian;
    Eigen::Matrix6d PI = _childArtInertia;
    PI.noalias() -= AIS * mInvProjArtInertiaImplicit * AIS.transpose();
    assert(!math::isNan(PI));

    // Add child body's articulated inertia to parent body's articulated inertia.
    // Note that mT should be updated.
    _parentArtInertia += math::transformInertia(mT.inverse(), PI);
  }

  // Documentation inherited
  void updateInvProjArtInertia(const Eigen::Matrix6d& _artInertia)
  {
    // Projected articulated inertia
    Eigen::Matrix<double, DOF, DOF> projAI
        = mJacobian.transpose() * _artInertia * mJacobian;

    // Inversion of projected articulated inertia
    //mInvProjArtInertia = projAI.inverse();
    mInvProjArtInertia
        = projAI.ldlt().solve(Eigen::Matrix<double, DOF, DOF>::Identity());

    // Verification
    assert(!math::isNan(mInvProjArtInertia));
  }

  // Documentation inherited
  void updateInvProjArtInertiaImplicit(
      const Eigen::Matrix6d& _artInertia, double _timeStep)
  {
    // Projected articulated inertia
    Eigen::Matrix<double, DOF, DOF> projAI
        = mJacobian.transpose() * _artInertia * mJacobian;

    // Add additional inertia for implicit damping and spring force
    for (size_t i = 0; i < DOF; ++i)
    {
      projAI(i, i) += _timeStep * mDampingCoefficient[i]
                      + _timeStep * _timeStep * mSpringStiffness[i];
    }

    // Inversion of projected articulated inertia
    mInvProjArtInertiaImplicit = projAI.inverse();
  //  mInvProjArtInertiaImplicit = projAI.ldlt().solve(Eigen::Matrix3d::Identity());

    // Verification
    assert(!math::isNan(mInvProjArtInertiaImplicit));
  }

  // Documentation inherited
  void addChildBiasForceTo(Eigen::Vector6d& _parentBiasForce,
                           const Eigen::Matrix6d& _childArtInertia,
                           const Eigen::Vector6d& _childBiasForce,
                           const Eigen::Vector6d& _childPartialAcc)
  {
    // Compute beta
    Eigen::Vector6d beta
        = _childBiasForce
          + _childArtInertia
            * (_childPartialAcc
               + mJacobian * mInvProjArtInertiaImplicit * mTotalForce);

//    Eigen::Vector6d beta
//        = _childBiasForce;
//    beta.noalias() += _childArtInertia * _childPartialAcc;
//    beta.noalias() += _childArtInertia *  mJacobian * mInvProjArtInertiaImplicit * mTotalForce;

    // Verification
    assert(!math::isNan(beta));

    // Add child body's bias force to parent body's bias force. Note that mT
    // should be updated.
    _parentBiasForce += math::dAdInvT(mT, beta);
  }

  // Documentation inherited
  void addChildBiasImpulseTo(Eigen::Vector6d& _parentBiasImpulse,
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

  // Documentation inherited
  void updateTotalForce(const Eigen::Vector6d& _bodyForce,
                        double _timeStep)
  {
    // Spring and damping forces
    Eigen::Matrix<double, DOF, 1> springForce;
    Eigen::Matrix<double, DOF, 1> dampingForce;

    for (size_t i = 0; i < DOF; ++i)
    {
      springForce[i]
          = -mSpringStiffness[i] * (mCoordinate[i].getPos() - mRestPosition[i]
                                    + mCoordinate[i].getVel() * _timeStep);

      dampingForce[i] = -mDampingCoefficient[i] * mCoordinate[i].getVel();
    }

    //
    mTotalForce = GenCoordSystem::getGenForces() + springForce + dampingForce;
    mTotalForce.noalias() -= mJacobian.transpose() * (_bodyForce);
  }

  // Documentation inherited
  void updateTotalImpulse(const Eigen::Vector6d& _bodyImpulse)
  {
    //
    mTotalImpulse = GenCoordSystem::getConstraintImpulses();
    mTotalImpulse.noalias() -= mJacobian.transpose() * _bodyImpulse;
  }

  // Documentation inherited
  void updateAcceleration(const Eigen::Matrix6d& _artInertia,
                          const Eigen::Vector6d& _spatialAcc)
  {
    //
    Eigen::Matrix<double, DOF, 1> acc
        = mInvProjArtInertiaImplicit
          * (mTotalForce - mJacobian.transpose()
                           * _artInertia * math::AdInvT(mT, _spatialAcc));

    // Verification
    assert(!math::isNan(acc));

    //
    for (size_t i = 0; i < DOF; ++i)
      mCoordinate[i].setAcc(acc[i]);
  }

  // Documentation inherited
  void updateVelocityChange(const Eigen::Matrix6d& _artInertia,
                            const Eigen::Vector6d& _velocityChange)
  {
    //
    Eigen::Matrix<double, DOF, 1> delVel
        = mInvProjArtInertia
          * (mTotalImpulse - mJacobian.transpose()
                             * _artInertia * math::AdInvT(mT, _velocityChange));

    // Verification
    assert(!math::isNan(delVel));

    //
    for (size_t i = 0; i < DOF; ++i)
      mCoordinate[i].setVelChange(delVel[i]);
  }

  // Documentation inherited
  void clearConstraintImpulse()
  {
    mTotalImpulse.setZero();
  }

  //----------------------------------------------------------------------------
  // Recursive algorithms for equations of motion
  //----------------------------------------------------------------------------

  // Documentation inherited
  void getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                               const size_t _col,
                               const Eigen::Matrix6d& _artInertia,
                               const Eigen::Vector6d& _spatialAcc)
  {
    //
    mInvMassMatrixSegment
        = mInvProjArtInertia
          * (mTotalForce - mJacobian.transpose()
                           * _artInertia * math::AdInvT(mT, _spatialAcc));

    // Verification
    assert(!math::isNan(mInvMassMatrixSegment));

    // Index
    size_t iStart = mCoordinate[0].getSkeletonIndex();

    // Assign
    _invMassMat.block<DOF, 1>(iStart, _col) = mInvMassMatrixSegment;
  }

  // Documentation inherited
  void addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
  {
    //
    _acc += mJacobian * mInvMassMatrixSegment;
  }

protected:
  /// Generalized coordinates
  GenCoord mCoordinate[DOF];

  /// Spatial Jacobian
  Eigen::Matrix<double, 6, DOF> mJacobian;

  /// Time derivative of spatial Jacobian
  Eigen::Matrix<double, 6, DOF> mJacobianDeriv;

  /// Inverse of projected articulated inertia
  Eigen::Matrix<double, DOF, DOF> mInvProjArtInertia;

  /// Inverse of projected articulated inertia for implicit joint damping and
  /// spring forces
  Eigen::Matrix<double, DOF, DOF> mInvProjArtInertiaImplicit;

  /// Total force projected on joint space
  Eigen::Matrix<double, DOF, 1> mTotalForce;

  /// Total impluse projected on joint space
  Eigen::Matrix<double, DOF, 1> mTotalImpulse;

  ///
  Eigen::Matrix<double, DOF, 1> mInvMassMatrixSegment;

private:
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_MULTIDOFJOINT_H_
