/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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
  virtual ~SingleDofJoint();

  // Documentation inherited
  DEPRECATED(4.1)
  virtual size_t getDof() const;

  // Documentation inherited
  virtual size_t getNumDofs() const;

  // Documentation inherited
  virtual void setIndexInSkeleton(size_t _index, size_t _indexInSkeleton);

  // Documentation inherited
  virtual size_t getIndexInSkeleton(size_t _index) const;

  //----------------------------------------------------------------------------
  // Command
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setCommand(size_t _index, double _command);

  // Documentation inherited
  virtual double getCommand(size_t _index) const;

  // Documentation inherited
  virtual void setCommands(const Eigen::VectorXd& _commands);

  // Documentation inherited
  virtual Eigen::VectorXd getCommands() const;

  // Documentation inherited
  virtual void resetCommands();

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  // TODO(JS): Not to use Eigen::VectorXd

  // Documentation inherited
  virtual void setPosition(size_t _index, double _position);

  // Documentation inherited
  virtual double getPosition(size_t _index) const;

  // Documentation inherited
  virtual void setPositions(const Eigen::VectorXd& _positions);

  // Documentation inherited
  virtual Eigen::VectorXd getPositions() const;

  // Documentation inherited
  virtual void resetPositions();

  // Documentation inherited
  virtual void setPositionLowerLimit(size_t _index, double _position);

  // Documentation inherited
  virtual double getPositionLowerLimit(size_t _index) const;

  // Documentation inherited
  virtual void setPositionUpperLimit(size_t _index, double _position);

  // Documentation inherited
  virtual double getPositionUpperLimit(size_t _index) const;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setVelocity(size_t _index, double _velocity);

  // Documentation inherited
  virtual double getVelocity(size_t _index) const;

  // Documentation inherited
  virtual void setVelocities(const Eigen::VectorXd& _velocities);

  // Documentation inherited
  virtual Eigen::VectorXd getVelocities() const;

  // Documentation inherited
  virtual void resetVelocities();

  // Documentation inherited
  virtual void setVelocityLowerLimit(size_t _index, double _velocity);

  // Documentation inherited
  virtual double getVelocityLowerLimit(size_t _index) const;

  // Documentation inherited
  virtual void setVelocityUpperLimit(size_t _index, double _velocity);

  // Documentation inherited
  virtual double getVelocityUpperLimit(size_t _index) const;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setAcceleration(size_t _index, double _acceleration);

  // Documentation inherited
  virtual double getAcceleration(size_t _index) const;

  // Documentation inherited
  virtual void setAccelerations(const Eigen::VectorXd& _accelerations);

  // Documentation inherited
  virtual Eigen::VectorXd getAccelerations() const;

  // Documentation inherited
  virtual void resetAccelerations();

  // Documentation inherited
  virtual void setAccelerationLowerLimit(size_t _index, double _acceleration);

  // Documentation inherited
  virtual double getAccelerationLowerLimit(size_t _index) const;

  // Documentation inherited
  virtual void setAccelerationUpperLimit(size_t _index, double _acceleration);

  // Documentation inherited
  virtual double getAccelerationUpperLimit(size_t _index) const;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setForce(size_t _index, double _force);

  // Documentation inherited
  virtual double getForce(size_t _index);

  // Documentation inherited
  virtual void setForces(const Eigen::VectorXd& _forces);

  // Documentation inherited
  virtual Eigen::VectorXd getForces() const;

  // Documentation inherited
  virtual void resetForces();

  // Documentation inherited
  virtual void setForceLowerLimit(size_t _index, double _force);

  // Documentation inherited
  virtual double getForceLowerLimit(size_t _index) const;

  // Documentation inherited
  virtual void setForceUpperLimit(size_t _index, double _force);

  // Documentation inherited
  virtual double getForceUpperLimit(size_t _index) const;

  //----------------------------------------------------------------------------
  // Velocity change
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setVelocityChange(size_t _index, double _velocityChange);

  // Documentation inherited
  virtual double getVelocityChange(size_t _index) const;

  // Documentation inherited
  virtual void resetVelocityChanges();

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setConstraintImpulse(size_t _index, double _impulse);

  // Documentation inherited
  virtual double getConstraintImpulse(size_t _index) const;

  // Documentation inherited
  virtual void resetConstraintImpulses();

  //----------------------------------------------------------------------------
  // Integration
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void integratePositions(double _dt);

  // Documentation inherited
  virtual void integrateVelocities(double _dt);

  //----------------------------------------------------------------------------
  // Spring and damper
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setSpringStiffness(size_t _index, double _k);

  // Documentation inherited
  virtual double getSpringStiffness(size_t _index) const;

  // Documentation inherited
  virtual void setRestPosition(size_t _index, double _q0);

  // Documentation inherited
  virtual double getRestPosition(size_t _index) const;

  // Documentation inherited
  virtual void setDampingCoefficient(size_t _index, double _d);

  // Documentation inherited
  virtual double getDampingCoefficient(size_t _index) const;

  //----------------------------------------------------------------------------

  /// Get potential energy
  virtual double getPotentialEnergy() const;

  // Documentation inherited
  virtual Eigen::Vector6d getBodyConstraintWrench() const override;

protected:
  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual const math::Jacobian getLocalJacobian() const override;

  // Documentation inherited
  virtual const math::Jacobian getLocalJacobianTimeDeriv() const override;

  // Documentation inherited
  virtual void addVelocityTo(Eigen::Vector6d& _vel) override;

  // Documentation inherited
  virtual void setPartialAccelerationTo(
      Eigen::Vector6d& _partialAcceleration,
      const Eigen::Vector6d& _childVelocity) override;

  // Documentation inherited
  virtual void addAccelerationTo(Eigen::Vector6d& _acc) override;

  // Documentation inherited
  virtual void addVelocityChangeTo(Eigen::Vector6d& _velocityChange) override;

  // Documentation inherited
  virtual void addChildArtInertiaTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia) override;

  // Documentation inherited
  virtual void addChildArtInertiaImplicitTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia) override;

  // Documentation inherited
  virtual void updateInvProjArtInertia(
      const Eigen::Matrix6d& _artInertia) override;

  // Documentation inherited
  virtual void updateInvProjArtInertiaImplicit(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep) override;

  // Documentation inherited
  virtual void addChildBiasForceTo(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc) override;

  // Documentation inherited
  virtual void addChildBiasImpulseTo(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse) override;

  // Documentation inherited
  virtual void updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                  double _timeStep) override;

  // Documentation inherited
  virtual void updateTotalImpulse(
      const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  virtual void resetTotalImpulses() override;

  // Documentation inherited
  virtual void updateAcceleration(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  virtual void updateVelocityChange(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _velocityChange) override;

  // Documentation inherited
  virtual void updateForceID(const Eigen::Vector6d& _bodyForce,
                             double _timeStep,
                             bool _withDampingForces,
                             bool _withSpringForces) override;

  // Documentation inherited
  virtual void updateForceFD(const Eigen::Vector6d& _bodyForce,
                             double _timeStep,
                             bool _withDampingForces,
                             bool _withSpringForces) override;

  // Documentation inherited
  virtual void updateImpulseID(const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  virtual void updateImpulseFD(const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  virtual void updateConstrainedTerms(double _timeStep) override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Recursive algorithm routines for equations of motion
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void addChildBiasForceForInvMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  // Documentation inherited
  virtual void addChildBiasForceForInvAugMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  // Documentation inherited
  virtual void updateTotalForceForInvMassMatrix(
      const Eigen::Vector6d& _bodyForce) override;

  // Documentation inherited
  virtual void getInvMassMatrixSegment(
      Eigen::MatrixXd& _invMassMat,
      const size_t _col,
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  virtual void getInvAugMassMatrixSegment(
      Eigen::MatrixXd& _invMassMat,
      const size_t _col,
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  virtual void addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc) override;

  // Documentation inherited
  virtual Eigen::VectorXd getSpatialToGeneralized(
      const Eigen::Vector6d& _spatial) override;

  /// \}

protected:
  // TODO(JS): Need?
  ///
  size_t mIndexInSkeleton;

  /// Command
  double mCommand;

  //----------------------------------------------------------------------------
  // Configuration
  //----------------------------------------------------------------------------

  /// Position
  double mPosition;

  /// Lower limit of position
  double mPositionLowerLimit;

  /// Upper limit of position
  double mPositionUpperLimit;

  /// Derivatives w.r.t. an arbitrary scalr variable
  double mPositionDeriv;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  /// Generalized velocity
  double mVelocity;

  /// Min value allowed.
  double mVelocityLowerLimit;

  /// Max value allowed.
  double mVelocityUpperLimit;

  /// Derivatives w.r.t. an arbitrary scalr variable
  double mVelocityDeriv;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  /// Generalized acceleration
  double mAcceleration;

  /// Min value allowed.
  double mAccelerationLowerLimit;

  /// upper limit of generalized acceleration
  double mAccelerationUpperLimit;

  /// Derivatives w.r.t. an arbitrary scalr variable
  double mAccelerationDeriv;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  /// Generalized force
  double mForce;

  /// Min value allowed.
  double mForceLowerLimit;

  /// Max value allowed.
  double mForceUpperLimit;

  /// Derivatives w.r.t. an arbitrary scalr variable
  double mForceDeriv;

  //----------------------------------------------------------------------------
  // Impulse
  //----------------------------------------------------------------------------

  /// Change of generalized velocity
  double mVelocityChange;

  /// Generalized impulse
  double mImpulse;

  /// Generalized constraint impulse
  double mConstraintImpulse;

  //----------------------------------------------------------------------------
  // Spring and damper
  //----------------------------------------------------------------------------

  /// Joint spring stiffness
  double mSpringStiffness;

  /// Rest joint position for joint spring
  double mRestPosition;

  /// Joint damping coefficient
  double mDampingCoefficient;

  //----------------------------------------------------------------------------
  // For recursive dynamics algorithms
  //----------------------------------------------------------------------------

  /// Spatial Jacobian expressed in the child body frame
  Eigen::Vector6d mJacobian;

  /// Time derivative of spatial Jacobian expressed in the child body frame
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
  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  void addChildArtInertiaToForceType(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaToAccelerationType(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaImplicitToForceType(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaImplicitToAccelerationType(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void updateInvProjArtInertiaForceType(
      const Eigen::Matrix6d& _artInertia);

  void updateInvProjArtInertiaAccelerationType(
      const Eigen::Matrix6d& _artInertia);

  void updateInvProjArtInertiaImplicitForceType(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep);

  void updateInvProjArtInertiaImplicitAccelerationType(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep);

  void addChildBiasForceToForceType(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc);

  void addChildBiasForceToAccelerationType(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc);

  void addChildBiasImpulseToForceType(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse);

  void addChildBiasImpulseToAccelerationType(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse);

  void updateTotalForceForceType(const Eigen::Vector6d& _bodyForce,
                                  double _timeStep);

  void updateTotalForceAccelerationType(const Eigen::Vector6d& _bodyForce,
                                        double _timeStep);

  void updateTotalImpulseForceType(
      const Eigen::Vector6d& _bodyImpulse);

  void updateTotalImpulseAccelerationType(
      const Eigen::Vector6d& _bodyImpulse);

  void updateAccelerationForceType(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc);

  void updateAccelerationAccelerationType(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc);

  void updateVelocityChangeForceType(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _velocityChange);

  void updateVelocityChangeAccelerationType(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _velocityChange);

  void updateConstrainedTermsForceType(double _timeStep);

  void updateConstrainedTermsAccelerationType(double _timeStep);

  /// \}
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SINGLEDOFJOINT_H_
