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

#ifndef DART_DYNAMICS_ZERODOFJOINT_H_
#define DART_DYNAMICS_ZERODOFJOINT_H_

#include <string>

#include "dart/dynamics/Joint.h"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

/// class ZeroDofJoint
class ZeroDofJoint : public Joint
{
public:

  struct Properties : Joint::Properties
  {
    Properties(const Joint::Properties& _properties = Joint::Properties());
    virtual ~Properties() = default;
  };

  ZeroDofJoint(const ZeroDofJoint&) = delete;

  /// Destructor
  virtual ~ZeroDofJoint();

  /// Get the Properties of this ZeroDofJoint
  Properties getZeroDofJointProperties() const;

  //----------------------------------------------------------------------------
  // Interface for generalized coordinates
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual DegreeOfFreedom* getDof(size_t) override;

  // Documentation inherited
  virtual const DegreeOfFreedom* getDof(size_t) const override;

  // Documentation inherited
  const std::string& setDofName(size_t, const std::string&, bool ) override;

  // Documentation inherited
  void preserveDofName(size_t, bool) override;

  // Documentation inherited
  bool isDofNamePreserved(size_t) const override;

  const std::string& getDofName(size_t) const override;

  // Documentation inherited
  virtual size_t getNumDofs() const override;

  // Documentation inherited
  virtual size_t getIndexInSkeleton(size_t _index) const override;

  // Documentation inherited
  virtual size_t getIndexInTree(size_t _index) const override;

  //----------------------------------------------------------------------------
  // Command
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setCommand(size_t _index, double _command) override;

  // Documentation inherited
  virtual double getCommand(size_t _index) const override;

  // Documentation inherited
  virtual void setCommands(const Eigen::VectorXd& _commands) override;

  // Documentation inherited
  virtual Eigen::VectorXd getCommands() const override;

  // Documentation inherited
  virtual void resetCommands() override;

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setPosition(size_t, double) override;

  // Documentation inherited
  virtual double getPosition(size_t _index) const override;

  // Documentation inherited
  virtual void setPositions(const Eigen::VectorXd& _positions) override;

  // Documentation inherited
  virtual Eigen::VectorXd getPositions() const override;

  // Documentation inherited
  virtual void setPositionLowerLimit(size_t _index, double _position) override;

  // Documentation inherited
  virtual double getPositionLowerLimit(size_t _index) const override;

  // Documentation inherited
  virtual void setPositionUpperLimit(size_t _index, double _position) override;

  // Documentation inherited
  virtual double getPositionUpperLimit(size_t _index) const override;

  // Documentation inherited
  virtual bool hasPositionLimit(size_t _index) const override;

  // Documentation inherited
  virtual void resetPosition(size_t _index) override;

  // Documentation inherited
  virtual void resetPositions() override;

  // Documentation inherited
  virtual void setInitialPosition(size_t _index, double _initial) override;

  // Documentation inherited
  virtual double getInitialPosition(size_t _index) const override;

  // Documentation inherited
  virtual void setInitialPositions(const Eigen::VectorXd& _initial) override;

  // Documentation inherited
  virtual Eigen::VectorXd getInitialPositions() const override;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setVelocity(size_t _index, double _velocity) override;

  // Documentation inherited
  virtual double getVelocity(size_t _index) const override;

  // Documentation inherited
  virtual void setVelocities(const Eigen::VectorXd& _velocities) override;

  // Documentation inherited
  virtual Eigen::VectorXd getVelocities() const override;

  // Documentation inherited
  virtual void setVelocityLowerLimit(size_t _index, double _velocity) override;

  // Documentation inherited
  virtual double getVelocityLowerLimit(size_t _index) const override;

  // Documentation inherited
  virtual void setVelocityUpperLimit(size_t _index, double _velocity) override;

  // Documentation inherited
  virtual double getVelocityUpperLimit(size_t _index) const override;

  // Documentation inherited
  virtual void resetVelocity(size_t _index) override;

  // Documentation inherited
  virtual void resetVelocities() override;

  // Documentation inherited
  virtual void setInitialVelocity(size_t _index, double _initial) override;

  // Documentation inherited
  virtual double getInitialVelocity(size_t _index) const override;

  // Documentation inherited
  virtual void setInitialVelocities(const Eigen::VectorXd& _initial) override;

  // Documentation inherited
  virtual Eigen::VectorXd getInitialVelocities() const override;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setAcceleration(size_t _index, double _acceleration) override;

  // Documentation inherited
  virtual double getAcceleration(size_t _index) const override;

  // Documentation inherited
  virtual void setAccelerations(const Eigen::VectorXd& _accelerations) override;

  // Documentation inherited
  virtual Eigen::VectorXd getAccelerations() const override;

  // Documentation inherited
  virtual void resetAccelerations() override;

  // Documentation inherited
  virtual void setAccelerationLowerLimit(size_t _index, double _acceleration) override;

  // Documentation inherited
  virtual double getAccelerationLowerLimit(size_t _index) const override;

  // Documentation inherited
  virtual void setAccelerationUpperLimit(size_t _index, double _acceleration) override;

  // Documentation inherited
  virtual double getAccelerationUpperLimit(size_t _index) const override;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setForce(size_t _index, double _force) override;

  // Documentation inherited
  virtual double getForce(size_t _index) override;

  // Documentation inherited
  virtual void setForces(const Eigen::VectorXd& _forces) override;

  // Documentation inherited
  virtual Eigen::VectorXd getForces() const override;

  // Documentation inherited
  virtual void resetForces() override;

  // Documentation inherited
  virtual void setForceLowerLimit(size_t _index, double _force) override;

  // Documentation inherited
  virtual double getForceLowerLimit(size_t _index) const override;

  // Documentation inherited
  virtual void setForceUpperLimit(size_t _index, double _force) override;

  // Documentation inherited
  virtual double getForceUpperLimit(size_t _index) const override;

  //----------------------------------------------------------------------------
  // Velocity change
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setVelocityChange(size_t _index, double _velocityChange) override;

  // Documentation inherited
  virtual double getVelocityChange(size_t _index) const override;

  // Documentation inherited
  virtual void resetVelocityChanges() override;

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setConstraintImpulse(size_t _index, double _impulse) override;

  // Documentation inherited
  virtual double getConstraintImpulse(size_t _index) const override;

  // Documentation inherited
  virtual void resetConstraintImpulses() override;

  //----------------------------------------------------------------------------
  // Integration and finite difference
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void integratePositions(double _dt) override;

  // Documentation inherited
  virtual void integrateVelocities(double _dt) override;

  // Documentation inherited
  Eigen::VectorXd getPositionDifferences(
      const Eigen::VectorXd& _q2, const Eigen::VectorXd& _q1) const override;

  //----------------------------------------------------------------------------
  /// \{ \name Passive forces - spring, viscous friction, Coulomb friction
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setSpringStiffness(size_t _index, double _k) override;

  // Documentation inherited
  virtual double getSpringStiffness(size_t _index) const override;

  // Documentation inherited
  virtual void setRestPosition(size_t _index, double _q0) override;

  // Documentation inherited
  virtual double getRestPosition(size_t _index) const override;

  // Documentation inherited
  virtual void setDampingCoefficient(size_t _index, double _d) override;

  // Documentation inherited
  virtual double getDampingCoefficient(size_t _index) const override;

  // Documentation inherited
  virtual void setCoulombFriction(size_t _index, double _friction) override;

  // Documentation inherited
  virtual double getCoulombFriction(size_t _index) const override;

  /// \}

  //----------------------------------------------------------------------------

  /// Get potential energy
  virtual double getPotentialEnergy() const override;

  // Documentation inherited
  virtual Eigen::Vector6d getBodyConstraintWrench() const override;

protected:

  /// Constructor called by inheriting classes
  ZeroDofJoint();

  // Documentation inherited
  void registerDofs() override;

  // Documentation inherited
  virtual void updateDegreeOfFreedomNames() override;

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual const math::Jacobian getLocalJacobian() const override;

  // Documentation inherited
  const math::Jacobian getLocalJacobian(
      const Eigen::VectorXd& _positions) const override;

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

  /// Add child's bias force to parent's one
  virtual void addChildBiasForceForInvMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  /// Add child's bias force to parent's one
  virtual void addChildBiasForceForInvAugMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  ///
  virtual void updateTotalForceForInvMassMatrix(
      const Eigen::Vector6d& _bodyForce) override;

  // Documentation inherited
  virtual void getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                       const size_t _col,
                                       const Eigen::Matrix6d& _artInertia,
                                       const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  virtual void getInvAugMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                          const size_t _col,
                                          const Eigen::Matrix6d& _artInertia,
                                          const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  virtual void addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc) override;

  // Documentation inherited
  virtual Eigen::VectorXd getSpatialToGeneralized(
      const Eigen::Vector6d& _spatial) override;

  /// \}

private:

  /// Used by getDofName()
  const std::string emptyString;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_ZERODOFJOINT_H_
