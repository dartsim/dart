/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_MULTIDOFJOINT_HPP_
#define DART_DYNAMICS_MULTIDOFJOINT_HPP_

#include <string>
#include <array>

#include "dart/config.hpp"
#include "dart/common/Console.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/common/RequiresAspect.hpp"
#include "dart/dynamics/detail/MultiDofJointAspect.hpp"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

/// class MultiDofJoint
template<std::size_t DOF>
class MultiDofJoint : public detail::MultiDofJointBase< MultiDofJoint<DOF>, DOF >
{
public:

  constexpr static std::size_t NumDofs = DOF;
  using Vector = Eigen::Matrix<double, DOF, 1>;
  using Base = detail::MultiDofJointBase<MultiDofJoint<DOF>, DOF>;
  using UniqueProperties = detail::MultiDofJointUniqueProperties<DOF>;
  using Properties = detail::MultiDofJointProperties<DOF>;
  using AspectState = typename Base::AspectState;
  using AspectProperties = typename Base::AspectProperties;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR( typename MultiDofJoint<DOF>::Aspect, MultiDofJointAspect )

  MultiDofJoint(const MultiDofJoint&) = delete;

  /// Destructor
  virtual ~MultiDofJoint();

  /// Set the Properties of this MultiDofJoint
  void setProperties(const Properties& _properties);

  /// Set the Properties of this MultiDofJoint
  void setProperties(const UniqueProperties& _properties);

  /// Set the AspectState of this MultiDofJoint
  void setAspectState(const AspectState& state);

  /// Set the AspectProperties of this MultiDofJoint
  void setAspectProperties(const AspectProperties& properties);

  /// Get the Properties of this MultiDofJoint
  Properties getMultiDofJointProperties() const;

  /// Copy the Properties of another MultiDofJoint
  void copy(const MultiDofJoint<DOF>& _otherJoint);

  /// Copy the Properties of another MultiDofJoint
  void copy(const MultiDofJoint<DOF>* _otherJoint);

  /// Same as copy(const MutliDofJoint&)
  MultiDofJoint<DOF>& operator=(const MultiDofJoint<DOF>& _otherJoint);

  //----------------------------------------------------------------------------
  // Interface for generalized coordinates
  //----------------------------------------------------------------------------

  // Documentation inherited
  DegreeOfFreedom* getDof(std::size_t index) override;

  // Documentation inherited
  const DegreeOfFreedom* getDof(std::size_t _index) const override;

  // Documentation inherited
  std::size_t getNumDofs() const override;

  // Documentation inherited
  const std::string& setDofName(std::size_t _index,
                                const std::string& _name,
                                bool _preserveName=true) override;

  // Docuemntation inherited
  void preserveDofName(std::size_t _index, bool _preserve) override;

  // Documentation inherited
  bool isDofNamePreserved(std::size_t _index) const override;

  // Documentation inherited
  const std::string& getDofName(std::size_t _index) const override;

  // Documentation inherited
  std::size_t getIndexInSkeleton(std::size_t _index) const override;

  // Documentation inherited
  std::size_t getIndexInTree(std::size_t _index) const override;

  //----------------------------------------------------------------------------
  // Command
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setCommand(std::size_t _index, double command) override;

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
  void setPosition(std::size_t _index, double _position) override;

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
  void setPositionUpperLimit(std::size_t _index, double _position) override;

  // Documentation inherited
  double getPositionUpperLimit(std::size_t _index) const override;

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
  void setVelocityUpperLimit(std::size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocityUpperLimit(std::size_t _index) const override;

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
  void setAccelerationUpperLimit(std::size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAccelerationUpperLimit(std::size_t _index) const override;

  //----------------------------------------------------------------------------
  // Fixed-size mutators and accessors
  //----------------------------------------------------------------------------

  // Note: The fixed-size versions of these functions exist to make it easier
  // to comply with the auto-updating design. Use these functions to avoid
  // accessing mPosition directly, that way it is easier to ensure that the
  // auto-updating design assumptions are being satisfied when reviewing the
  // code.

  /// Fixed-size version of setPositions()
  void setPositionsStatic(const Vector& _positions);

  /// Fixed-size version of getPositions()
  const Vector& getPositionsStatic() const;

  /// Fixed-size version of setVelocities()
  void setVelocitiesStatic(const Vector& _velocities);

  /// Fixed-size version of getVelocities()
  const Vector& getVelocitiesStatic() const;

  /// Fixed-size version of setAccelerations()
  void setAccelerationsStatic(const Vector& _accels);

  /// Fixed-size version of getAccelerations()
  const Vector& getAccelerationsStatic() const;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setForce(std::size_t _index, double _force) override;

  // Documentation inherited
  double getForce(std::size_t _index) override;

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
  void setForceUpperLimit(std::size_t _index, double _force) override;

  // Documentation inherited
  double getForceUpperLimit(std::size_t _index) const override;

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

  /// Fixed-size version of getPositionDifferences()
  virtual Eigen::Matrix<double, DOF, 1> getPositionDifferencesStatic(
      const Vector& _q2, const Vector& _q1) const;

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
  MultiDofJoint(const Properties& properties);

  // Docuemntation inherited
  void registerDofs() override;

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian getRelativeJacobian() const override;

  /// Fixed-size version of getRelativeJacobian()
  const Eigen::Matrix<double, 6, DOF>& getRelativeJacobianStatic() const;

  // Documentation inherited
  math::Jacobian getRelativeJacobian(
      const Eigen::VectorXd& _positions) const override;

  /// Fixed-size version of getRelativeJacobian()
  virtual Eigen::Matrix<double, 6, DOF> getRelativeJacobianStatic(
      const Eigen::Matrix<double, DOF, 1>& _positions) const = 0;

  // Documentation inherited
  const math::Jacobian getRelativeJacobianTimeDeriv() const override;

  /// Fixed-size version of getRelativeJacobianTimeDeriv()
  const Eigen::Matrix<double, 6, DOF>& getRelativeJacobianTimeDerivStatic() const;

  /// Get the inverse of the projected articulated inertia
  const Eigen::Matrix<double, DOF, DOF>& getInvProjArtInertia() const;

  /// Get the inverse of projected articulated inertia for implicit joint
  /// damping and spring forces
  const Eigen::Matrix<double, DOF, DOF>& getInvProjArtInertiaImplicit() const;

  // Documentation inherited
  void updateRelativeSpatialVelocity() const override;

  // Documentation inherited
  void updateRelativeSpatialAcceleration() const override;

  // Documentation inherited
  void updateRelativePrimaryAcceleration() const override;

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
      const Eigen::Matrix6d& _artInertia, double _timeStep) override;

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

  // Documentation inherited
  void addChildBiasForceForInvMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  // Documentation inherited
  void addChildBiasForceForInvAugMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  // Documentation inherited
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

protected:

  /// Array of DegreeOfFreedom objects
  std::array<DegreeOfFreedom*, DOF> mDofs;

  //----------------------------------------------------------------------------
  // Impulse
  //----------------------------------------------------------------------------

  /// Change of generalized velocity
  Vector mVelocityChanges;

  /// Generalized impulse
  Vector mImpulses;

  /// Generalized constraint impulse
  Vector mConstraintImpulses;

  //----------------------------------------------------------------------------
  // For recursive dynamics algorithms
  //----------------------------------------------------------------------------

  /// Spatial Jacobian expressed in the child body frame
  ///
  /// Do not use directly! Use getRelativeJacobianStatic() to access this quantity
  mutable Eigen::Matrix<double, 6, DOF> mJacobian;

  /// Time derivative of spatial Jacobian expressed in the child body frame
  ///
  /// Do not use directly! Use getRelativeJacobianTimeDerivStatic() to access this
  /// quantity
  mutable Eigen::Matrix<double, 6, DOF> mJacobianDeriv;

  /// Inverse of projected articulated inertia
  ///
  /// Do not use directly! Use getInvProjArtInertia() to get this quantity
  mutable Eigen::Matrix<double, DOF, DOF> mInvProjArtInertia;

  /// Inverse of projected articulated inertia for implicit joint damping and
  /// spring forces
  ///
  /// Do not use directly! Use getInvProjArtInertiaImplicit() to access this
  /// quantity
  mutable Eigen::Matrix<double, DOF, DOF> mInvProjArtInertiaImplicit;

  /// Total force projected on joint space
  Vector mTotalForce;

  /// Total impluse projected on joint space
  Vector mTotalImpulse;

  //----------------------------------------------------------------------------
  // For equations of motion
  //----------------------------------------------------------------------------

  ///
  Vector mInvM_a;

  ///
  Vector mInvMassMatrixSegment;

private:
  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  void addChildArtInertiaToDynamic(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaToKinematic(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaImplicitToDynamic(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaImplicitToKinematic(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void updateInvProjArtInertiaDynamic(
      const Eigen::Matrix6d& _artInertia);

  void updateInvProjArtInertiaKinematic(
      const Eigen::Matrix6d& _artInertia);

  void updateInvProjArtInertiaImplicitDynamic(
      const Eigen::Matrix6d& _artInertia, double _timeStep);

  void updateInvProjArtInertiaImplicitKinematic(
      const Eigen::Matrix6d& _artInertia, double _timeStep);

  void addChildBiasForceToDynamic(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc);

  void addChildBiasForceToKinematic(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc);

  void addChildBiasImpulseToDynamic(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse);

  void addChildBiasImpulseToKinematic(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse);

  void updateTotalForceDynamic(const Eigen::Vector6d& _bodyForce,
                                    double _timeStep);

  void updateTotalForceKinematic(const Eigen::Vector6d& _bodyForce,
                                          double _timeStep);

  void updateTotalImpulseDynamic(
        const Eigen::Vector6d& _bodyImpulse);

  void updateTotalImpulseKinematic(
        const Eigen::Vector6d& _bodyImpulse);

  void updateAccelerationDynamic(
        const Eigen::Matrix6d& _artInertia,
        const Eigen::Vector6d& _spatialAcc);

  void updateAccelerationKinematic(
        const Eigen::Matrix6d& _artInertia,
        const Eigen::Vector6d& _spatialAcc);

  void updateVelocityChangeDynamic(
        const Eigen::Matrix6d& _artInertia,
        const Eigen::Vector6d& _velocityChange);

  void updateVelocityChangeKinematic(
        const Eigen::Matrix6d& _artInertia,
        const Eigen::Vector6d& _velocityChange);

  void updateConstrainedTermsDynamic(double _timeStep);

  void updateConstrainedTermsKinematic(double _timeStep);

  /// \}
};

}  // namespace dynamics
}  // namespace dart

#include "dart/dynamics/detail/MultiDofJoint.hpp"

#endif  // DART_DYNAMICS_MULTIDOFJOINT_HPP_
