/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_DYNAMICS_ZERODOFJOINT_HPP_
#define DART_DYNAMICS_ZERODOFJOINT_HPP_

#include <string>

#include "dart/dynamics/Joint.hpp"

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
  DegreeOfFreedom* getDof(std::size_t) override;

  // Documentation inherited
  const DegreeOfFreedom* getDof(std::size_t) const override;

  // Documentation inherited
  const std::string& setDofName(std::size_t, const std::string&, bool ) override;

  // Documentation inherited
  void preserveDofName(std::size_t, bool) override;

  // Documentation inherited
  bool isDofNamePreserved(std::size_t) const override;

  const std::string& getDofName(std::size_t) const override;

  // Documentation inherited
  std::size_t getNumDofs() const override;

  // Documentation inherited
  std::size_t getIndexInSkeleton(std::size_t _index) const override;

  // Documentation inherited
  std::size_t getIndexInTree(std::size_t _index) const override;

  //----------------------------------------------------------------------------
  // Command
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setCommand(std::size_t _index, double _command) override;

  // Documentation inherited
  double getCommand(std::size_t _index) const override;

  // Documentation inherited
  void setCommands(const Eigen::VectorXd& _commands) override;

  // Documentation inherited
  Eigen::VectorXd getCommands() const override;

  // Documentation inherited
  void resetCommands() override;

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setPosition(std::size_t, double) override;

  // Documentation inherited
  double getPosition(std::size_t _index) const override;

  // Documentation inherited
  void setPositions(const Eigen::VectorXd& _positions) override;

  // Documentation inherited
  Eigen::VectorXd getPositions() const override;

  // Documentation inherited
  void setPositionLowerLimit(std::size_t _index, double _position) override;

  // Documentation inherited
  double getPositionLowerLimit(std::size_t _index) const override;

  // Documentation inherited
  void setPositionLowerLimits(const Eigen::VectorXd& lowerLimits) override;

  // Documentation inherited
  Eigen::VectorXd getPositionLowerLimits() const override;

  // Documentation inherited
  void setPositionUpperLimit(std::size_t index, double position) override;

  // Documentation inherited
  double getPositionUpperLimit(std::size_t index) const override;

  // Documentation inherited
  void setPositionUpperLimits(const Eigen::VectorXd& upperLimits) override;

  // Documentation inherited
  Eigen::VectorXd getPositionUpperLimits() const override;

  // Documentation inherited
  bool hasPositionLimit(std::size_t _index) const override;

  // Documentation inherited
  void resetPosition(std::size_t _index) override;

  // Documentation inherited
  void resetPositions() override;

  // Documentation inherited
  void setInitialPosition(std::size_t _index, double _initial) override;

  // Documentation inherited
  double getInitialPosition(std::size_t _index) const override;

  // Documentation inherited
  void setInitialPositions(const Eigen::VectorXd& _initial) override;

  // Documentation inherited
  Eigen::VectorXd getInitialPositions() const override;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setVelocity(std::size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocity(std::size_t _index) const override;

  // Documentation inherited
  void setVelocities(const Eigen::VectorXd& _velocities) override;

  // Documentation inherited
  Eigen::VectorXd getVelocities() const override;

  // Documentation inherited
  void setVelocityLowerLimit(std::size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocityLowerLimit(std::size_t _index) const override;

  // Documentation inherited
  void setVelocityLowerLimits(const Eigen::VectorXd& lowerLimits) override;

  // Documentation inherited
  Eigen::VectorXd getVelocityLowerLimits() const override;

  // Documentation inherited
  void setVelocityUpperLimit(std::size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocityUpperLimit(std::size_t _index) const override;

  // Documentation inherited
  void setVelocityUpperLimits(const Eigen::VectorXd& upperLimits) override;

  // Documentation inherited
  Eigen::VectorXd getVelocityUpperLimits() const override;

  // Documentation inherited
  void resetVelocity(std::size_t _index) override;

  // Documentation inherited
  void resetVelocities() override;

  // Documentation inherited
  void setInitialVelocity(std::size_t _index, double _initial) override;

  // Documentation inherited
  double getInitialVelocity(std::size_t _index) const override;

  // Documentation inherited
  void setInitialVelocities(const Eigen::VectorXd& _initial) override;

  // Documentation inherited
  Eigen::VectorXd getInitialVelocities() const override;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setAcceleration(std::size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAcceleration(std::size_t _index) const override;

  // Documentation inherited
  void setAccelerations(const Eigen::VectorXd& _accelerations) override;

  // Documentation inherited
  Eigen::VectorXd getAccelerations() const override;

  // Documentation inherited
  void resetAccelerations() override;

  // Documentation inherited
  void setAccelerationLowerLimit(std::size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAccelerationLowerLimit(std::size_t _index) const override;

  // Documentation inherited
  void setAccelerationLowerLimits(const Eigen::VectorXd& lowerLimits) override;

  // Documentation inherited
  Eigen::VectorXd getAccelerationLowerLimits() const override;

  // Documentation inherited
  void setAccelerationUpperLimit(std::size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAccelerationUpperLimit(std::size_t _index) const override;

  // Documentation inherited
  void setAccelerationUpperLimits(const Eigen::VectorXd& upperLimits) override;

  // Documentation inherited
  Eigen::VectorXd getAccelerationUpperLimits() const override;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setForce(std::size_t _index, double _force) override;

  // Documentation inherited
  double getForce(std::size_t _index) const override;

  // Documentation inherited
  void setForces(const Eigen::VectorXd& _forces) override;

  // Documentation inherited
  Eigen::VectorXd getForces() const override;

  // Documentation inherited
  void resetForces() override;

  // Documentation inherited
  void setForceLowerLimit(std::size_t _index, double _force) override;

  // Documentation inherited
  double getForceLowerLimit(std::size_t _index) const override;

  // Documentation inherited
  void setForceLowerLimits(const Eigen::VectorXd& lowerLimits) override;

  // Documentation inherited
  Eigen::VectorXd getForceLowerLimits() const override;

  // Documentation inherited
  void setForceUpperLimit(std::size_t _index, double _force) override;

  // Documentation inherited
  double getForceUpperLimit(std::size_t _index) const override;

  // Documentation inherited
  void setForceUpperLimits(const Eigen::VectorXd& upperLimits) override;

  // Documentation inherited
  Eigen::VectorXd getForceUpperLimits() const override;

  //----------------------------------------------------------------------------
  // Velocity change
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setVelocityChange(std::size_t _index, double _velocityChange) override;

  // Documentation inherited
  double getVelocityChange(std::size_t _index) const override;

  // Documentation inherited
  void resetVelocityChanges() override;

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setConstraintImpulse(std::size_t _index, double _impulse) override;

  // Documentation inherited
  double getConstraintImpulse(std::size_t _index) const override;

  // Documentation inherited
  void resetConstraintImpulses() override;

  //----------------------------------------------------------------------------
  // Integration and finite difference
  //----------------------------------------------------------------------------

  // Documentation inherited
  void integratePositions(double _dt) override;

  // Documentation inherited
  void integrateVelocities(double _dt) override;

  // Documentation inherited
  Eigen::VectorXd getPositionDifferences(
      const Eigen::VectorXd& _q2, const Eigen::VectorXd& _q1) const override;

  //----------------------------------------------------------------------------
  /// \{ \name Passive forces - spring, viscous friction, Coulomb friction
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setSpringStiffness(std::size_t _index, double _k) override;

  // Documentation inherited
  double getSpringStiffness(std::size_t _index) const override;

  // Documentation inherited
  void setRestPosition(std::size_t _index, double _q0) override;

  // Documentation inherited
  double getRestPosition(std::size_t _index) const override;

  // Documentation inherited
  void setDampingCoefficient(std::size_t _index, double _d) override;

  // Documentation inherited
  double getDampingCoefficient(std::size_t _index) const override;

  // Documentation inherited
  void setCoulombFriction(std::size_t _index, double _friction) override;

  // Documentation inherited
  double getCoulombFriction(std::size_t _index) const override;

  /// \}

  //----------------------------------------------------------------------------

  // Documentation inherited
  double computePotentialEnergy() const override;

  // Documentation inherited
  Eigen::Vector6d getBodyConstraintWrench() const override;

protected:

  /// Constructor called by inheriting classes
  ZeroDofJoint();

  // Documentation inherited
  void registerDofs() override;

  // Documentation inherited
  void updateDegreeOfFreedomNames() override;

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian getRelativeJacobian() const override;

  // Documentation inherited
  math::Jacobian getRelativeJacobian(
      const Eigen::VectorXd& _positions) const override;

  // Documentation inherited
  const math::Jacobian getRelativeJacobianTimeDeriv() const override;

  // Documentation inherited
  void addVelocityTo(Eigen::Vector6d& _vel) override;

  // Documentation inherited
  void setPartialAccelerationTo(
      Eigen::Vector6d& _partialAcceleration,
      const Eigen::Vector6d& _childVelocity) override;

  // Documentation inherited
  void addAccelerationTo(Eigen::Vector6d& _acc) override;

  // Documentation inherited
  void addVelocityChangeTo(Eigen::Vector6d& _velocityChange) override;

  // Documentation inherited
  void addChildArtInertiaTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia) override;

  // Documentation inherited
  void addChildArtInertiaImplicitTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia) override;

  // Documentation inherited
  void updateInvProjArtInertia(
      const Eigen::Matrix6d& _artInertia) override;

  // Documentation inherited
  void updateInvProjArtInertiaImplicit(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep) override;

  // Documentation inherited
  void addChildBiasForceTo(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc) override;

  // Documentation inherited
  void addChildBiasImpulseTo(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse) override;

  // Documentation inherited
  void updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                  double _timeStep) override;

  // Documentation inherited
  void updateTotalImpulse(
      const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  void resetTotalImpulses() override;

  // Documentation inherited
  void updateAcceleration(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  void updateVelocityChange(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _velocityChange) override;

  // Documentation inherited
  void updateForceID(const Eigen::Vector6d& _bodyForce,
                             double _timeStep,
                             bool _withDampingForces,
                             bool _withSpringForces) override;

  // Documentation inherited
  void updateForceFD(const Eigen::Vector6d& _bodyForce,
                             double _timeStep,
                             bool _withDampingForces,
                             bool _withSpringForces) override;

  // Documentation inherited
  void updateImpulseID(const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  void updateImpulseFD(const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  void updateConstrainedTerms(double _timeStep) override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Recursive algorithm routines for equations of motion
  //----------------------------------------------------------------------------

  /// Add child's bias force to parent's one
  void addChildBiasForceForInvMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  /// Add child's bias force to parent's one
  void addChildBiasForceForInvAugMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  ///
  void updateTotalForceForInvMassMatrix(
      const Eigen::Vector6d& _bodyForce) override;

  // Documentation inherited
  void getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                       const std::size_t _col,
                                       const Eigen::Matrix6d& _artInertia,
                                       const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  void getInvAugMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                          const std::size_t _col,
                                          const Eigen::Matrix6d& _artInertia,
                                          const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  void addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc) override;

  // Documentation inherited
  Eigen::VectorXd getSpatialToGeneralized(
      const Eigen::Vector6d& _spatial) override;

  /// \}

private:

  /// Used by getDofName()
  const std::string emptyString;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_ZERODOFJOINT_HPP_
