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

#ifndef DART_DYNAMICS_SINGLEDOFJOINT_H_
#define DART_DYNAMICS_SINGLEDOFJOINT_H_

#include <string>

#include "dart/dynamics/Joint.h"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

/// class SingleDofJoint
class SingleDofJoint : public Joint
{
public:
  /// Constructor
  SingleDofJoint(const std::string& _name);

  /// Destructor
  ~SingleDofJoint();

protected:
  //----------------------------------------------------------------------------
  // Recursive dynamics algorithms
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void addVelocityTo(Eigen::Vector6d& _vel);

  // Documentation inherited
  virtual void setPartialAccelerationTo(Eigen::Vector6d& _partialAcceleration,
                                        const Eigen::Vector6d& _childVelocity);

  // Documentation inherited
  virtual void addAccelerationTo(Eigen::Vector6d& _acc);

  // Documentation inherited
  virtual void addVelocityChangeTo(Eigen::Vector6d& _velocityChange);

  // Documentation inherited
  virtual void addChildArtInertiaTo(Eigen::Matrix6d& _parentArtInertia,
                                    const Eigen::Matrix6d& _childArtInertia);

  // Documentation inherited
  virtual void addChildArtInertiaImplicitTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  // Documentation inherited
  virtual void updateInvProjArtInertia(const Eigen::Matrix6d& _artInertia);

  // Documentation inherited
  virtual void updateInvProjArtInertiaImplicit(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep);

  // Documentation inherited
  virtual void addChildBiasForceTo(Eigen::Vector6d& _parentBiasForce,
                                   const Eigen::Matrix6d& _childArtInertia,
                                   const Eigen::Vector6d& _childBiasForce,
                                   const Eigen::Vector6d& _childPartialAcc);

  // Documentation inherited
  virtual void addChildBiasImpulseTo(Eigen::Vector6d& _parentBiasImpulse,
                                     const Eigen::Matrix6d& _childArtInertia,
                                     const Eigen::Vector6d& _childBiasImpulse);

  // Documentation inherited
  virtual void updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                double _timeStep);

  // Documentation inherited
  virtual void updateTotalImpulse(const Eigen::Vector6d& _bodyImpulse);

  // Documentation inherited
  virtual void updateAcceleration(const Eigen::Matrix6d& _artInertia,
                                  const Eigen::Vector6d& _spatialAcc);

  // Documentation inherited
  virtual void updateVelocityChange(const Eigen::Matrix6d& _artInertia,
                                    const Eigen::Vector6d& _velocityChange);

  // Documentation inherited
  virtual void clearConstraintImpulse();

  //----------------------------------------------------------------------------
  // Recursive algorithms for equations of motion
  //----------------------------------------------------------------------------

  /// Add child's bias force to parent's one
  virtual void addChildBiasForceForInvMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce);

  /// Add child's bias force to parent's one
  virtual void addChildBiasForceForInvAugMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce);

  ///
  virtual void updateTotalForceForInvMassMatrix(
      const Eigen::Vector6d& _bodyForce);

  // Documentation inherited
  virtual void getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                       const size_t _col,
                                       const Eigen::Matrix6d& _artInertia,
                                       const Eigen::Vector6d& _spatialAcc);

  // Documentation inherited
  virtual void getInvAugMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                          const size_t _col,
                                          const Eigen::Matrix6d& _artInertia,
                                          const Eigen::Vector6d& _spatialAcc);

  // Documentation inherited
  virtual void addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc);

protected:
  /// Generalized coordinate
  GenCoord mCoordinate;

  /// Rotational axis
  Eigen::Vector3d mAxis;

  //----------------------------------------------------------------------------
  // For recursive dynamics algorithms
  //----------------------------------------------------------------------------

  /// Spatial Jacobian
  Eigen::Vector6d mJacobian;

  /// Time derivative of spatial Jacobian
  Eigen::Vector6d mJacobianDeriv;

  /// Inverse of projected articulated inertia
  double mInvProjArtInertia;

  /// Inverse of projected articulated inertia for implicit joint damping and
  /// spring forces
  double mInvProjArtInertiaImplicit;

  /// Total force projected on joint space
  double mTotalForce;

  /// Total impluse projected on joint space
  double mTotalImpulse;

  //----------------------------------------------------------------------------
  // For equations of motion
  //----------------------------------------------------------------------------

  ///
  double mInvM_a;

  ///
  double mInvMassMatrixSegment;

private:
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SINGLEDOFJOINT_H_
