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

#ifndef DART_DYNAMICS_MULTIDOFJOINT_H_
#define DART_DYNAMICS_MULTIDOFJOINT_H_

#include <iostream>
#include <string>
#include <array>

#include "dart/config.h"
#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

/// class MultiDofJoint
template<size_t DOF>
class MultiDofJoint : public Joint
{
public:

  typedef Eigen::Matrix<double, DOF, 1> Vector;

  struct UniqueProperties
  {
    /// Lower limit of position
    Vector mPositionLowerLimits;

    /// Upper limit of position
    Vector mPositionUpperLimits;

    /// Min value allowed.
    Vector mVelocityLowerLimits;

    /// Max value allowed.
    Vector mVelocityUpperLimits;

    /// Min value allowed.
    Vector mAccelerationLowerLimits;

    /// upper limit of generalized acceleration
    Vector mAccelerationUpperLimits;

    /// Min value allowed.
    Vector mForceLowerLimits;

    /// Max value allowed.
    Vector mForceUpperLimits;

    /// Joint spring stiffness
    Vector mSpringStiffnesses;

    /// Rest joint position for joint spring
    Vector mRestPositions;

    /// Joint damping coefficient
    Vector mDampingCoefficients;

    /// Joint Coulomb friction
    Vector mFrictions;

    /// True if the name of the corresponding DOF is not allowed to be
    /// overwritten
    std::array<bool, DOF> mPreserveDofNames;

    /// The name of the DegreesOfFreedom for this Joint
    std::array<std::string, DOF> mDofNames;

    UniqueProperties(
      const Vector& _positionLowerLimits = Vector::Constant(-DART_DBL_INF),
      const Vector& _positionUpperLimits = Vector::Constant( DART_DBL_INF),
      const Vector& _velocityLowerLimits = Vector::Constant(-DART_DBL_INF),
      const Vector& _velocityUpperLimits = Vector::Constant( DART_DBL_INF),
      const Vector& _accelerationLowerLimits = Vector::Constant(-DART_DBL_INF),
      const Vector& _accelerationUpperLimits = Vector::Constant( DART_DBL_INF),
      const Vector& _forceLowerLimits = Vector::Constant(-DART_DBL_INF),
      const Vector& _forceUpperLimits = Vector::Constant( DART_DBL_INF),
      const Vector& _springStiffness = Vector::Constant(0.0),
      const Vector& _restPosition = Vector::Constant(0.0),
      const Vector& _dampingCoefficient = Vector::Constant(0.0),
      const Vector& _coulombFrictions = Vector::Constant(0.0));

    virtual ~UniqueProperties() = default;
  };

  struct Properties : Joint::Properties, UniqueProperties
  {
    Properties(
        const Joint::Properties& _jointProperties = Joint::Properties(),
        const UniqueProperties& _multiDofProperties = UniqueProperties());

    virtual ~Properties() = default;
  };

  /// Constructor
//  DEPRECATED(4.5) // Use MultiDofJoint(const Properties&)
  MultiDofJoint(const std::string& _name);

  /// Destructor
  virtual ~MultiDofJoint();

  /// Set the Properties of this MultiDofJoint
  void setProperties(const Properties& _properties);

  /// Set the Properties of this MultiDofJoint
  void setProperties(const UniqueProperties& _properties);

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
  DegreeOfFreedom* getDof(size_t index) override;

  // Documentation inherited
  const DegreeOfFreedom* getDof(size_t _index) const override;

  // Documentation inherited
  size_t getNumDofs() const override;

  // Documentation inherited
  const std::string& setDofName(size_t _index,
                                const std::string& _name,
                                bool _preserveName=true) override;

  // Docuemntation inherited
  void preserveDofName(size_t _index, bool _preserve) override;

  // Documentation inherited
  bool isDofNamePreserved(size_t _index) const override;

  // Documentation inherited
  const std::string& getDofName(size_t _index) const override;

  // Documentation inherited
  virtual size_t getIndexInSkeleton(size_t _index) const override;

  // Documentation inherited
  virtual size_t getIndexInTree(size_t _index) const override;

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

  // Documentation inherited
  void setPosition(size_t _index, double _position) override;

  // Documentation inherited
  double getPosition(size_t _index) const override;

  // Documentation inherited
  void setPositions(const Eigen::VectorXd& _positions) override;

  // Documentation inherited
  Eigen::VectorXd getPositions() const override;

  // Documentation inherited
  void resetPositions() override;

  // Documentation inherited
  void setPositionLowerLimit(size_t _index, double _position) override;

  // Documentation inherited
  double getPositionLowerLimit(size_t _index) const override;

  // Documentation inherited
  void setPositionUpperLimit(size_t _index, double _position) override;

  // Documentation inherited
  double getPositionUpperLimit(size_t _index) const override;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setVelocity(size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocity(size_t _index) const override;

  // Documentation inherited
  void setVelocities(const Eigen::VectorXd& _velocities) override;

  // Documentation inherited
  Eigen::VectorXd getVelocities() const override;

  // Documentation inherited
  void resetVelocities() override;

  // Documentation inherited
  void setVelocityLowerLimit(size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocityLowerLimit(size_t _index) const override;

  // Documentation inherited
  void setVelocityUpperLimit(size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocityUpperLimit(size_t _index) const override;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setAcceleration(size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAcceleration(size_t _index) const override;

  // Documentation inherited
  void setAccelerations(const Eigen::VectorXd& _accelerations) override;

  // Documentation inherited
  Eigen::VectorXd getAccelerations() const override;

  // Documentation inherited
  void resetAccelerations() override;

  // Documentation inherited
  void setAccelerationLowerLimit(size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAccelerationLowerLimit(size_t _index) const override;

  // Documentation inherited
  void setAccelerationUpperLimit(size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAccelerationUpperLimit(size_t _index) const override;

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
  void setForce(size_t _index, double _force) override;

  // Documentation inherited
  double getForce(size_t _index) override;

  // Documentation inherited
  void setForces(const Eigen::VectorXd& _forces) override;

  // Documentation inherited
  Eigen::VectorXd getForces() const override;

  // Documentation inherited
  void resetForces() override;

  // Documentation inherited
  void setForceLowerLimit(size_t _index, double _force) override;

  // Documentation inherited
  double getForceLowerLimit(size_t _index) const override;

  // Documentation inherited
  void setForceUpperLimit(size_t _index, double _force) override;

  // Documentation inherited
  double getForceUpperLimit(size_t _index) const override;

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

  // Documentation inherited
  virtual double getPotentialEnergy() const;

  // Documentation inherited
  virtual Eigen::Vector6d getBodyConstraintWrench() const override;

protected:

  /// Constructor called by inheriting classes
  MultiDofJoint(const Properties& _properties);

  // Docuemntation inherited
  void registerDofs() override;

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian getLocalJacobian() const override;

  /// Fixed-size version of getLocalJacobian()
  const Eigen::Matrix<double, 6, DOF>& getLocalJacobianStatic() const;

  // Documentation inherited
  const math::Jacobian getLocalJacobianTimeDeriv() const override;

  /// Fixed-size version of getLocalJacobianTimeDeriv()
  const Eigen::Matrix<double, 6, DOF>& getLocalJacobianTimeDerivStatic() const;

  /// Get the inverse of the projected articulated inertia
  const Eigen::Matrix<double, DOF, DOF>& getInvProjArtInertia() const;

  /// Get the inverse of projected articulated inertia for implicit joint
  /// damping and spring forces
  const Eigen::Matrix<double, DOF, DOF>& getInvProjArtInertiaImplicit() const;

  // Documentation inherited
  void updateLocalSpatialVelocity() const override;

  // Documentation inherited
  void updateLocalSpatialAcceleration() const override;

  // Documentation inherited
  void updateLocalPrimaryAcceleration() const override;

  // Documentation inherited
  void addVelocityTo(Eigen::Vector6d& _vel) override;

  // Documentation inherited
  void setPartialAccelerationTo(
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
                               const size_t _col,
                               const Eigen::Matrix6d& _artInertia,
                               const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  void getInvAugMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                  const size_t _col,
                                  const Eigen::Matrix6d& _artInertia,
                                  const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  void addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc) override;

  // Documentation inherited
  Eigen::VectorXd getSpatialToGeneralized(
      const Eigen::Vector6d& _spatial) override;

  /// \}

protected:

  /// Properties of this MultiDofJoint
  typename MultiDofJoint<DOF>::UniqueProperties mMultiDofP;

  /// Array of DegreeOfFreedom objects
  std::array<DegreeOfFreedom*, DOF> mDofs;

  /// Command
  Vector mCommands;

  //----------------------------------------------------------------------------
  // Configuration
  //----------------------------------------------------------------------------

  /// Position
  Vector mPositions;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Vector mPositionDeriv;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  /// Generalized velocity
  Vector mVelocities;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Vector mVelocitiesDeriv;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  /// Generalized acceleration
  Vector mAccelerations;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Vector mAccelerationsDeriv;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  /// Generalized force
  Vector mForces;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Vector mForcesDeriv;

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
  /// Do not use directly! Use getLocalJacobianStatic() to access this quantity
  mutable Eigen::Matrix<double, 6, DOF> mJacobian;

  /// Time derivative of spatial Jacobian expressed in the child body frame
  ///
  /// Do not use directly! Use getLocalJacobianTimeDerivStatic() to access this
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

//==============================================================================
template <size_t DOF>
MultiDofJoint<DOF>::UniqueProperties::UniqueProperties(
    const Vector& _positionLowerLimits,
    const Vector& _positionUpperLimits,
    const Vector& _velocityLowerLimits,
    const Vector& _velocityUpperLimits,
    const Vector& _accelerationLowerLimits,
    const Vector& _accelerationUpperLimits,
    const Vector& _forceLowerLimits,
    const Vector& _forceUpperLimits,
    const Vector& _springStiffness,
    const Vector& _restPosition,
    const Vector& _dampingCoefficient,
    const Vector& _coulombFrictions)
  : mPositionLowerLimits(_positionLowerLimits),
    mPositionUpperLimits(_positionUpperLimits),
    mVelocityLowerLimits(_velocityLowerLimits),
    mVelocityUpperLimits(_velocityUpperLimits),
    mAccelerationLowerLimits(_accelerationLowerLimits),
    mAccelerationUpperLimits(_accelerationUpperLimits),
    mForceLowerLimits(_forceLowerLimits),
    mForceUpperLimits(_forceUpperLimits),
    mSpringStiffnesses(_springStiffness),
    mRestPositions(_restPosition),
    mDampingCoefficients(_dampingCoefficient),
    mFrictions(_coulombFrictions)
{
  for (size_t i = 0; i < DOF; ++i)
    mPreserveDofNames[i] = false;
}

//==============================================================================
template <size_t DOF>
MultiDofJoint<DOF>::Properties::Properties(
    const Joint::Properties& _jointProperties,
    const UniqueProperties& _multiDofProperties)
  : Joint::Properties(_jointProperties),
    UniqueProperties(_multiDofProperties)
{
  // Do nothing
}

//==============================================================================
template <size_t DOF>
MultiDofJoint<DOF>::MultiDofJoint(const std::string& _name)
  : Joint(_name),
    mCommands(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mPositions(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mPositionDeriv(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mVelocities(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mVelocitiesDeriv(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mAccelerations(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mAccelerationsDeriv(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mForces(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mForcesDeriv(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mVelocityChanges(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mImpulses(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mConstraintImpulses(Eigen::Matrix<double, DOF, 1>::Constant(0.0)),
    mJacobian(Eigen::Matrix<double, 6, DOF>::Zero()),
    mJacobianDeriv(Eigen::Matrix<double, 6, DOF>::Zero()),
    mInvProjArtInertia(Eigen::Matrix<double, DOF, DOF>::Zero()),
    mInvProjArtInertiaImplicit(Eigen::Matrix<double, DOF, DOF>::Zero()),
    mTotalForce(Eigen::Matrix<double, DOF, 1>::Zero()),
    mTotalImpulse(Eigen::Matrix<double, DOF, 1>::Zero())
{
  for (size_t i = 0; i < DOF; ++i)
    mDofs[i] = createDofPointer(i);
}

//==============================================================================
template <size_t DOF>
MultiDofJoint<DOF>::~MultiDofJoint()
{
  for (size_t i = 0; i < DOF; ++i)
    delete mDofs[i];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setProperties(const Properties& _properties)
{
  Joint::setProperties(static_cast<const Joint::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setProperties(const UniqueProperties& _properties)
{
  for(size_t i=0; i<DOF; ++i)
  {
    setDofName(i, _properties.mDofNames[i], _properties.mPreserveDofNames[i]);
    setPositionLowerLimit(i, _properties.mPositionLowerLimits[i]);
    setPositionUpperLimit(i, _properties.mPositionUpperLimits[i]);
    setVelocityLowerLimit(i, _properties.mVelocityLowerLimits[i]);
    setVelocityUpperLimit(i, _properties.mVelocityUpperLimits[i]);
    setAccelerationLowerLimit(i, _properties.mAccelerationLowerLimits[i]);
    setAccelerationUpperLimit(i, _properties.mAccelerationUpperLimits[i]);
    setForceLowerLimit(i, _properties.mForceLowerLimits[i]);
    setForceUpperLimit(i, _properties.mForceUpperLimits[i]);
    setSpringStiffness(i, _properties.mSpringStiffnesses[i]);
    setRestPosition(i, _properties.mRestPositions[i]);
    setDampingCoefficient(i, _properties.mDampingCoefficients[i]);
    setCoulombFriction(i, _properties.mFrictions[i]);
  }
}

//==============================================================================
template <size_t DOF>
typename MultiDofJoint<DOF>::Properties
MultiDofJoint<DOF>::getMultiDofJointProperties() const
{
  return MultiDofJoint<DOF>::Properties(mJointP, mMultiDofP);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::copy(const MultiDofJoint<DOF>& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getMultiDofJointProperties());
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::copy(const MultiDofJoint<DOF>* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*_otherJoint);
}

//==============================================================================
template <size_t DOF>
MultiDofJoint<DOF>& MultiDofJoint<DOF>::operator=(
    const MultiDofJoint<DOF>& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
template <size_t DOF>
DegreeOfFreedom* MultiDofJoint<DOF>::getDof(size_t _index)
{
  if (_index < DOF)
    return mDofs[_index];

  dterr << "[MultiDofJoint::getDof] Attempting to access index (" << _index
        << "). The index must be less than (" << DOF << ")!\n";
  assert(false);
  return nullptr;
}

//==============================================================================
template <size_t DOF>
const DegreeOfFreedom* MultiDofJoint<DOF>::getDof(size_t _index) const
{
  if (_index < DOF)
    return mDofs[_index];

  dterr << "[MultiDofJoint::getDof] Attempting to access index (" << _index
        << "). The index must be less than (" << DOF << ")!\n";
  assert(false);
  return nullptr;
}

//==============================================================================
template <size_t DOF>
const std::string& MultiDofJoint<DOF>::setDofName(size_t _index,
                                                  const std::string& _name,
                                                  bool _preserveName)
{
  if(DOF <= _index)
  {
    dtwarn << "[MultiDofJoint::setDofName] Attempting to set the name of DOF "
           << "index " << _index << ", which is out of bounds. We will set "
           << "the name of DOF index 0 instead\n";
    _index = 0;
  }

  preserveDofName(_index, _preserveName);
  std::string& dofName = mMultiDofP.mDofNames[_index];
  if(_name == dofName)
    return dofName;

  SkeletonPtr skel = mChildBodyNode? mChildBodyNode->getSkeleton() : nullptr;
  if(skel)
  {
    dofName =
        skel->mNameMgrForDofs.changeObjectName(mDofs[_index], _name);
  }
  else
    dofName = _name;

  return dofName;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::preserveDofName(size_t _index, bool _preserve)
{
  if (DOF <= _index)
  {
    dtwarn << "[MultiDofJoint::preserveDofName] Attempting to preserve the "
           << "name of DOF index " << _index << ", which is out of bounds. We "
           << "will preserve the name of DOF index 0 instead\n";
    _index = 0;
  }

  mMultiDofP.mPreserveDofNames[_index] = _preserve;
}

//==============================================================================
template <size_t DOF>
bool MultiDofJoint<DOF>::isDofNamePreserved(size_t _index) const
{
  if(DOF <= _index)
  {
    dtwarn << "[MultiDofJoint::isDofNamePreserved] Requesting whether DOF "
           << "index " << _index << " is preserved, but this is out of bounds "
           << "(max " << DOF-1 << "). We will return the result of DOF index 0 "
           << "instead\n";
    _index = 0;
  }

  return mMultiDofP.mPreserveDofNames[_index];
}

//==============================================================================
template <size_t DOF>
const std::string& MultiDofJoint<DOF>::getDofName(size_t _index) const
{
  if(DOF <= _index)
  {
    dterr << "[MultiDofJoint::getDofName] Requested name of DOF index "
          << _index << ", but that is out of bounds (max " << DOF-1 << "). "
          << "Returning name of DOF 0\n";
    return mMultiDofP.mDofNames[0];
  }

  return mMultiDofP.mDofNames[_index];
}

//==============================================================================
template <size_t DOF>
size_t MultiDofJoint<DOF>::getNumDofs() const
{
  return DOF;
}

//==============================================================================
template <size_t DOF>
size_t MultiDofJoint<DOF>::getIndexInSkeleton(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::getIndexInSkeleton] index (" << _index
          << ") out of range. Must be less than " << getNumDofs() << "!\n";
    assert(false);
    return 0;
  }

  return mDofs[_index]->mIndexInSkeleton;
}

//==============================================================================
template <size_t DOF>
size_t MultiDofJoint<DOF>::getIndexInTree(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::getIndexInTree] index (" << _index
          << ") out of range. Must be less than " << getNumDofs() << "!\n";
    assert(false);
    return 0;
  }

  return mDofs[_index]->mIndexInTree;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setCommand(size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::setCommand]: index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mCommands[_index] = _position;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getCommand(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::getCommand]: index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mCommands[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setCommands(const Eigen::VectorXd& _commands)
{
  if (static_cast<size_t>(_commands.size()) != getNumDofs())
  {
    dterr << "[MultiDofJoint::setCommands]: commands's size["
          << _commands.size() << "] is different with the dof ["
          << getNumDofs() << "]" << std::endl;
    return;
  }

  mCommands = _commands;
}

//==============================================================================
template <size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getCommands() const
{
  return mCommands;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::resetCommands()
{
  mCommands.setZero();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setPosition(size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    dterr << "setPosition index[" << _index << "] out of range" << std::endl;
    return;
  }

  // Note: It would not make much sense to use setPositionsStatic() here
  mPositions[_index] = _position;
  notifyPositionUpdate();
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getPosition(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "setPosition index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return getPositionsStatic()[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setPositions(const Eigen::VectorXd& _positions)
{
  if (static_cast<size_t>(_positions.size()) != getNumDofs())
  {
    dterr << "setPositions positions's size[" << _positions.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  setPositionsStatic(_positions);
}

//==============================================================================
template <size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getPositions() const
{
  return getPositionsStatic();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::resetPositions()
{
  setPositionsStatic(Eigen::Matrix<double, DOF, 1>::Zero());
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setPositionLowerLimit(size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    dterr << "setPositionLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mMultiDofP.mPositionLowerLimits[_index] = _position;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getPositionLowerLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getPositionLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mMultiDofP.mPositionLowerLimits[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setPositionUpperLimit(size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    dterr << "setPositionUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mMultiDofP.mPositionUpperLimits[_index] = _position;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getPositionUpperLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getPositionUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mMultiDofP.mPositionUpperLimits[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setVelocity(size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    dterr << "setVelocity index[" << _index << "] out of range" << std::endl;
    return;
  }

  // Note: It would not make much sense to use setVelocitiesStatic() here
  mVelocities[_index] = _velocity;
  notifyVelocityUpdate();
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getVelocity(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getVelocity index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return getVelocitiesStatic()[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setVelocities(const Eigen::VectorXd& _velocities)
{
  if (static_cast<size_t>(_velocities.size()) != getNumDofs())
  {
    dterr << "setVelocities velocities's size[" << _velocities.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  setVelocitiesStatic(_velocities);
}

//==============================================================================
template <size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getVelocities() const
{
  return getVelocitiesStatic();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::resetVelocities()
{
  setVelocitiesStatic(Eigen::Matrix<double, DOF, 1>::Zero());
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setVelocityLowerLimit(size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    dterr << "setVelocityLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mMultiDofP.mVelocityLowerLimits[_index] = _velocity;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getVelocityLowerLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getVelocityLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mMultiDofP.mVelocityLowerLimits[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setVelocityUpperLimit(size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    dterr << "setVelocityUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mMultiDofP.mVelocityUpperLimits[_index] = _velocity;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getVelocityUpperLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getVelocityUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mMultiDofP.mVelocityUpperLimits[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setAcceleration(size_t _index, double _acceleration)
{
  if (_index >= getNumDofs())
  {
    dterr << "setAcceleration index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  // Note: It would not make much sense to use setAccelerationsStatic() here
  mAccelerations[_index] = _acceleration;
  notifyAccelerationUpdate();

#if DART_MAJOR_VERSION == 4
  if (mJointP.mActuatorType == ACCELERATION)
    mCommands[_index] = getAccelerationsStatic()[_index];
#endif
  // TODO: Remove at DART 5.0.
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getAcceleration(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getAcceleration index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return getAccelerationsStatic()[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setAccelerations(const Eigen::VectorXd& _accelerations)
{
  if (static_cast<size_t>(_accelerations.size()) != getNumDofs())
  {
    dterr << "setAccelerations accelerations's size[" << _accelerations.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  setAccelerationsStatic(_accelerations);

#if DART_MAJOR_VERSION == 4
  if (mJointP.mActuatorType == ACCELERATION)
    mCommands = getAccelerationsStatic();
#endif
  // TODO: Remove at DART 5.0.
}

//==============================================================================
template <size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getAccelerations() const
{
  return getAccelerationsStatic();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::resetAccelerations()
{
  setAccelerationsStatic(Eigen::Matrix<double, DOF, 1>::Zero());
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setAccelerationLowerLimit(size_t _index,
                                                   double _acceleration)
{
  if (_index >= getNumDofs())
  {
    dterr << "setAccelerationLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mMultiDofP.mAccelerationLowerLimits[_index] = _acceleration;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getAccelerationLowerLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getAccelerationLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mMultiDofP.mAccelerationLowerLimits[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setAccelerationUpperLimit(size_t _index,
                                                   double _acceleration)
{
  if (_index >= getNumDofs())
  {
    dterr << "setAccelerationUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mMultiDofP.mAccelerationUpperLimits[_index] = _acceleration;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getAccelerationUpperLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getAccelerationUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mMultiDofP.mAccelerationUpperLimits[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setPositionsStatic(const Vector& _positions)
{
  mPositions = _positions;
  notifyPositionUpdate();
}

//==============================================================================
template <size_t DOF>
const typename MultiDofJoint<DOF>::Vector&
MultiDofJoint<DOF>::getPositionsStatic() const
{
  return mPositions;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setVelocitiesStatic(const Vector& _velocities)
{
  mVelocities = _velocities;
  notifyVelocityUpdate();
}

//==============================================================================
template <size_t DOF>
const typename MultiDofJoint<DOF>::Vector&
MultiDofJoint<DOF>::getVelocitiesStatic() const
{
  return mVelocities;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setAccelerationsStatic(const Vector& _accels)
{
  mAccelerations = _accels;
  notifyAccelerationUpdate();
}

//==============================================================================
template <size_t DOF>
const typename MultiDofJoint<DOF>::Vector&
MultiDofJoint<DOF>::getAccelerationsStatic() const
{
  return mAccelerations;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setForce(size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    dterr << "setForce index[" << _index << "] out of range" << std::endl;
    return;
  }

  mForces[_index] = _force;

#if DART_MAJOR_VERSION == 4
  if (mJointP.mActuatorType == FORCE)
    mCommands[_index] = mForces[_index];
#endif
  // TODO: Remove at DART 5.0.
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getForce(size_t _index)
{
  if (_index >= getNumDofs())
  {
    dterr << "getForce index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mForces[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setForces(const Eigen::VectorXd& _forces)
{
  if (static_cast<size_t>(_forces.size()) != getNumDofs())
  {
    dterr << "setForces forces's size[" << _forces.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  mForces = _forces;

#if DART_MAJOR_VERSION == 4
  if (mJointP.mActuatorType == FORCE)
    mCommands = mForces;
#endif
  // TODO: Remove at DART 5.0.
}

//==============================================================================
template <size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getForces() const
{
  return mForces;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::resetForces()
{
  mForces.setZero();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setForceLowerLimit(size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    dterr << "setForceLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mMultiDofP.mForceLowerLimits[_index] = _force;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getForceLowerLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getForceMin index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mMultiDofP.mForceLowerLimits[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setForceUpperLimit(size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    dterr << "setForceUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mMultiDofP.mForceUpperLimits[_index] = _force;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getForceUpperLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getForceUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mMultiDofP.mForceUpperLimits[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setVelocityChange(size_t _index,
                                           double _velocityChange)
{
  if (_index >= getNumDofs())
  {
    dterr << "setVelocityChange index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mVelocityChanges[_index] = _velocityChange;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getVelocityChange(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getVelocityChange index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mVelocityChanges[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::resetVelocityChanges()
{
  mVelocityChanges.setZero();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setConstraintImpulse(size_t _index, double _impulse)
{
  if (_index >= getNumDofs())
  {
    dterr << "setConstraintImpulse index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mConstraintImpulses[_index] = _impulse;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getConstraintImpulse(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getConstraintImpulse index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mConstraintImpulses[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::resetConstraintImpulses()
{
  mConstraintImpulses.setZero();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::integratePositions(double _dt)
{
  setPositionsStatic(getPositionsStatic() + getVelocitiesStatic() * _dt);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::integrateVelocities(double _dt)
{
  setVelocitiesStatic(getVelocitiesStatic() + getAccelerationsStatic() * _dt);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setSpringStiffness(size_t _index, double _k)
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::setSpringStiffness()]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_k >= 0.0);

  mMultiDofP.mSpringStiffnesses[_index] = _k;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getSpringStiffness(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::getSpringStiffness()]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mMultiDofP.mSpringStiffnesses[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setRestPosition(size_t _index, double _q0)
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::setRestPosition()]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  if (mMultiDofP.mPositionLowerLimits[_index] > _q0
      || mMultiDofP.mPositionUpperLimits[_index] < _q0)
  {
    dterr << "Rest position of joint[" << getName() << "], " << _q0
          << ", is out of the limit range["
          << mMultiDofP.mPositionLowerLimits[_index] << ", "
          << mMultiDofP.mPositionUpperLimits[_index] << "] in index[" << _index
          << "].\n";
    return;
  }

  mMultiDofP.mRestPositions[_index] = _q0;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getRestPosition(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::getRestPosition()]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mMultiDofP.mRestPositions[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setDampingCoefficient(size_t _index, double _d)
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::setDampingCoefficient()]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_d >= 0.0);

  mMultiDofP.mDampingCoefficients[_index] = _d;

}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getDampingCoefficient(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::getDampingCoefficient()]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mMultiDofP.mDampingCoefficients[_index];
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setCoulombFriction(size_t _index, double _friction)
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::setFriction()]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_friction >= 0.0);

  mMultiDofP.mFrictions[_index] = _friction;
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getCoulombFriction(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[MultiDofJoint::getFriction()]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mMultiDofP.mFrictions[_index];
}

//==============================================================================
template <size_t DOF>
double MultiDofJoint<DOF>::getPotentialEnergy() const
{
  // Spring energy
  Eigen::VectorXd displacement = getPositionsStatic() - mMultiDofP.mRestPositions;
  double pe = 0.5 * displacement.dot(mMultiDofP.mSpringStiffnesses.asDiagonal()
                                     * displacement);

  return pe;
}

//==============================================================================
template <size_t DOF>
Eigen::Vector6d MultiDofJoint<DOF>::getBodyConstraintWrench() const
{
  assert(mChildBodyNode);
  return mChildBodyNode->getBodyForce() - getLocalJacobianStatic() * mForces;
}

//==============================================================================
template <size_t DOF>
MultiDofJoint<DOF>::MultiDofJoint(const Properties& _properties)
  : Joint(_properties),
    mMultiDofP(_properties),
    mCommands(Vector::Zero()),
    mPositions(Vector::Zero()),
    mPositionDeriv(Vector::Zero()),
    mVelocities(Vector::Zero()),
    mVelocitiesDeriv(Vector::Zero()),
    mAccelerations(Vector::Zero()),
    mAccelerationsDeriv(Vector::Zero()),
    mForces(Vector::Zero()),
    mForcesDeriv(Vector::Zero()),
    mVelocityChanges(Vector::Zero()),
    mImpulses(Vector::Zero()),
    mConstraintImpulses(Vector::Zero()),
    mJacobian(Eigen::Matrix<double, 6, DOF>::Zero()),
    mJacobianDeriv(Eigen::Matrix<double, 6, DOF>::Zero()),
    mInvProjArtInertia(Eigen::Matrix<double, DOF, DOF>::Zero()),
    mInvProjArtInertiaImplicit(Eigen::Matrix<double, DOF, DOF>::Zero()),
    mTotalForce(Vector::Zero()),
    mTotalImpulse(Vector::Zero())
{
  for (size_t i = 0; i < DOF; ++i)
    mDofs[i] = createDofPointer(i);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::registerDofs()
{
  SkeletonPtr skel = mChildBodyNode->getSkeleton();
  for (size_t i = 0; i < DOF; ++i)
  {
    mMultiDofP.mDofNames[i] =
        skel->mNameMgrForDofs.issueNewNameAndAdd(mDofs[i]->getName(), mDofs[i]);
  }
}

//==============================================================================
template <size_t DOF>
const math::Jacobian MultiDofJoint<DOF>::getLocalJacobian() const
{
  if(mIsLocalJacobianDirty)
  {
    updateLocalJacobian(false);
    mIsLocalJacobianDirty = false;
  }
  return mJacobian;
}

//==============================================================================
template <size_t DOF>
const Eigen::Matrix<double, 6, DOF>&
MultiDofJoint<DOF>::getLocalJacobianStatic() const
{
  if(mIsLocalJacobianDirty)
  {
    updateLocalJacobian(false);
    mIsLocalJacobianDirty = false;
  }
  return mJacobian;
}

//==============================================================================
template <size_t DOF>
const math::Jacobian MultiDofJoint<DOF>::getLocalJacobianTimeDeriv() const
{
  if(mIsLocalJacobianTimeDerivDirty)
  {
    updateLocalJacobianTimeDeriv();
    mIsLocalJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
template <size_t DOF>
const Eigen::Matrix<double, 6, DOF>&
MultiDofJoint<DOF>::getLocalJacobianTimeDerivStatic() const
{
  if(mIsLocalJacobianTimeDerivDirty)
  {
    updateLocalJacobianTimeDeriv();
    mIsLocalJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
template <size_t DOF>
const Eigen::Matrix<double, DOF, DOF>&
MultiDofJoint<DOF>::getInvProjArtInertia() const
{
  Joint::updateArticulatedInertia();
  return mInvProjArtInertia;
}

//==============================================================================
template <size_t DOF>
const Eigen::Matrix<double, DOF, DOF>&
MultiDofJoint<DOF>::getInvProjArtInertiaImplicit() const
{
  Joint::updateArticulatedInertia();
  return mInvProjArtInertiaImplicit;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateLocalSpatialVelocity() const
{
  mSpatialVelocity = getLocalJacobianStatic() * getVelocitiesStatic();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateLocalSpatialAcceleration() const
{
  mSpatialAcceleration = getLocalPrimaryAcceleration()
                    + getLocalJacobianTimeDerivStatic() * getVelocitiesStatic();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateLocalPrimaryAcceleration() const
{
  mPrimaryAcceleration = getLocalJacobianStatic() * getAccelerationsStatic();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addVelocityTo(Eigen::Vector6d& _vel)
{
  // Add joint velocity to _vel
  _vel.noalias() += getLocalJacobianStatic() * getVelocitiesStatic();

  // Verification
  assert(!math::isNan(_vel));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& _childVelocity)
{
  // ad(V, S * dq) + dS * dq
  _partialAcceleration = math::ad(_childVelocity,
                      getLocalJacobianStatic() * getVelocitiesStatic())
                    + getLocalJacobianTimeDerivStatic() * getVelocitiesStatic();
  // Verification
  assert(!math::isNan(_partialAcceleration));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addAccelerationTo(Eigen::Vector6d& _acc)
{
  // Add joint acceleration to _acc
  _acc.noalias() += getLocalJacobianStatic() * getAccelerationsStatic();

  // Verification
  assert(!math::isNan(_acc));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addVelocityChangeTo(Eigen::Vector6d& _velocityChange)
{
  // Add joint velocity change to _velocityChange
  _velocityChange.noalias() += getLocalJacobianStatic() * mVelocityChanges;

  // Verification
  assert(!math::isNan(_velocityChange));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaTo(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaToDynamic(_parentArtInertia,
                                       _childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildArtInertiaToKinematic(_parentArtInertia,
                                             _childArtInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaToDynamic(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Matrix<double, 6, DOF> AIS = _childArtInertia * getLocalJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertia * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(), PI);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaToKinematic(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaImplicitToDynamic(_parentArtInertia,
                                                _childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildArtInertiaImplicitToKinematic(_parentArtInertia,
                                                _childArtInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaImplicitToDynamic(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Matrix<double, 6, DOF> AIS = _childArtInertia * getLocalJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertiaImplicit * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(), PI);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaImplicitToKinematic(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertia(
    const Eigen::Matrix6d& _artInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaDynamic(_artInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateInvProjArtInertiaKinematic(_artInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaDynamic(
    const Eigen::Matrix6d& _artInertia)
{
  // Projected articulated inertia
  const Eigen::Matrix<double, 6, DOF>& Jacobian = getLocalJacobianStatic();
  const Eigen::Matrix<double, DOF, DOF> projAI
      = Jacobian.transpose() * _artInertia * Jacobian;

  // Inversion of projected articulated inertia
  //mInvProjArtInertia = projAI.inverse();
  mInvProjArtInertia
      = projAI.ldlt().solve(Eigen::Matrix<double, DOF, DOF>::Identity());

  // Verification
  assert(!math::isNan(mInvProjArtInertia));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaKinematic(
    const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaImplicitDynamic(_artInertia, _timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateInvProjArtInertiaImplicitKinematic(_artInertia, _timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaImplicitDynamic(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  // Projected articulated inertia
  const Eigen::Matrix<double, 6, DOF>& Jacobian = getLocalJacobianStatic();
  Eigen::Matrix<double, DOF, DOF> projAI
      = Jacobian.transpose() * _artInertia * Jacobian;

  // Add additional inertia for implicit damping and spring force
  for (size_t i = 0; i < DOF; ++i)
  {
    projAI(i, i) += _timeStep * mMultiDofP.mDampingCoefficients[i]
        + _timeStep * _timeStep * mMultiDofP.mSpringStiffnesses[i];
  }

  // Inversion of projected articulated inertia
  //    mInvProjArtInertiaImplicit = projAI.inverse();
  mInvProjArtInertiaImplicit
      = projAI.ldlt().solve(Eigen::Matrix<double, DOF, DOF>::Identity());

  // Verification
  assert(!math::isNan(mInvProjArtInertiaImplicit));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaImplicitKinematic(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  // Do nothing
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceTo(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasForceToDynamic(_parentBiasForce,
                                 _childArtInertia,
                                 _childBiasForce,
                                 _childPartialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildBiasForceToKinematic(_parentBiasForce,
                                   _childArtInertia,
                                   _childBiasForce,
                                   _childPartialAcc);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceToDynamic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasForce
        + _childArtInertia
          * (_childPartialAcc
             + getLocalJacobianStatic()*getInvProjArtInertiaImplicit()
               *mTotalForce);

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceToKinematic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasForce
        + _childArtInertia*(_childPartialAcc
                            + getLocalJacobianStatic()*getAccelerationsStatic());

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasImpulseToDynamic(_parentBiasImpulse,
                                   _childArtInertia,
                                   _childBiasImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildBiasImpulseToKinematic(_parentBiasImpulse,
                                     _childArtInertia,
                                     _childBiasImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildBiasImpulseToDynamic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasImpulse
        + _childArtInertia*getLocalJacobianStatic()
          *getInvProjArtInertia()*mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildBiasImpulseToKinematic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getLocalTransform(), _childBiasImpulse);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateTotalForce(
    const Eigen::Vector6d& _bodyForce,
    double _timeStep)
{
  assert(_timeStep > 0.0);

  switch (mJointP.mActuatorType)
  {
    case FORCE:
      mForces = mCommands;
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case PASSIVE:
    case SERVO:
      mForces.setZero();
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case ACCELERATION:
      setAccelerationsStatic(mCommands);
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case VELOCITY:
      setAccelerationsStatic( (mCommands - getVelocitiesStatic()) / _timeStep );
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case LOCKED:
      setVelocitiesStatic(Eigen::Matrix<double, DOF, 1>::Zero());
      setAccelerationsStatic(Eigen::Matrix<double, DOF, 1>::Zero());
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateTotalForceDynamic(
    const Eigen::Vector6d& _bodyForce,
    double _timeStep)
{
  // Spring force
  const Eigen::Matrix<double, DOF, 1> springForce
      = (-mMultiDofP.mSpringStiffnesses).asDiagonal()
        *(getPositionsStatic() - mMultiDofP.mRestPositions
          + getVelocitiesStatic()*_timeStep);

  // Damping force
  const Eigen::Matrix<double, DOF, 1> dampingForce
      = (-mMultiDofP.mDampingCoefficients).asDiagonal()*getVelocitiesStatic();

  //
  mTotalForce = mForces + springForce + dampingForce;
  mTotalForce.noalias() -= getLocalJacobianStatic().transpose()*_bodyForce;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateTotalForceKinematic(
    const Eigen::Vector6d& _bodyForce,
    double _timeStep)
{
  // Do nothing
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateTotalImpulse(
    const Eigen::Vector6d& _bodyImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateTotalImpulseDynamic(_bodyImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateTotalImpulseKinematic(_bodyImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateTotalImpulseDynamic(
    const Eigen::Vector6d& _bodyImpulse)
{
  //
  mTotalImpulse = mConstraintImpulses;
  mTotalImpulse.noalias() -= getLocalJacobianStatic().transpose()*_bodyImpulse;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateTotalImpulseKinematic(
    const Eigen::Vector6d& _bodyImpulse)
{
  // Do nothing
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::resetTotalImpulses()
{
  mTotalImpulse.setZero();
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateAcceleration(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateAccelerationDynamic(_artInertia, _spatialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateAccelerationKinematic(_artInertia, _spatialAcc);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateAccelerationDynamic(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  setAccelerationsStatic( getInvProjArtInertiaImplicit()
        * (mTotalForce - getLocalJacobianStatic().transpose()
           *_artInertia*math::AdInvT(getLocalTransform(), _spatialAcc)) );

  // Verification
  assert(!math::isNan(getAccelerationsStatic()));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateAccelerationKinematic(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  // Do nothing
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateVelocityChange(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _velocityChange)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateVelocityChangeDynamic(_artInertia, _velocityChange);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateVelocityChangeKinematic(_artInertia, _velocityChange);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateVelocityChangeDynamic(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _velocityChange)
{
  //
  mVelocityChanges
      = getInvProjArtInertia()
      * (mTotalImpulse - getLocalJacobianStatic().transpose()
         *_artInertia*math::AdInvT(getLocalTransform(), _velocityChange));

  // Verification
  assert(!math::isNan(mVelocityChanges));
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateVelocityChangeKinematic(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _velocityChange)
{
  // Do nothing
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateForceID(const Eigen::Vector6d& _bodyForce,
                                       double _timeStep,
                                       bool _withDampingForces,
                                       bool _withSpringForces)
{
  mForces = getLocalJacobianStatic().transpose()*_bodyForce;

  // Damping force
  if (_withDampingForces)
  {
    const Eigen::Matrix<double, DOF, 1> dampingForces
        = (-mMultiDofP.mDampingCoefficients).asDiagonal()*getVelocitiesStatic();
    mForces -= dampingForces;
  }

  // Spring force
  if (_withSpringForces)
  {
    const Eigen::Matrix<double, DOF, 1> springForces
        = (-mMultiDofP.mSpringStiffnesses).asDiagonal()
          *(getPositionsStatic() - mMultiDofP.mRestPositions
            + getVelocitiesStatic()*_timeStep);
    mForces -= springForces;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateForceFD(const Eigen::Vector6d& _bodyForce,
                                       double _timeStep,
                                       bool _withDampingForces,
                                       bool _withSpringForces)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateForceID(_bodyForce, _timeStep, _withDampingForces,
                    _withSpringForces);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateImpulseID(const Eigen::Vector6d& _bodyImpulse)
{
  mImpulses = getLocalJacobianStatic().transpose()*_bodyImpulse;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateImpulseFD(const Eigen::Vector6d& _bodyImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateImpulseID(_bodyImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateConstrainedTerms(double _timeStep)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateConstrainedTermsDynamic(_timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateConstrainedTermsKinematic(_timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateConstrainedTermsDynamic(double _timeStep)
{
  const double invTimeStep = 1.0 / _timeStep;

  setVelocitiesStatic(getVelocitiesStatic() + mVelocityChanges);
  setAccelerationsStatic(getAccelerationsStatic()
                         + mVelocityChanges*invTimeStep);
  mForces.noalias() += mImpulses*invTimeStep;
  // Note: As long as this is only called from BodyNode::updateConstrainedTerms
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateConstrainedTermsKinematic(
    double _timeStep)
{
  mForces.noalias() += mImpulses / _timeStep;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia * getLocalJacobianStatic()
                    * getInvProjArtInertia() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia * getLocalJacobianStatic()
                    * getInvProjArtInertiaImplicit() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& _bodyForce)
{
  // Compute alpha
  mInvM_a = mForces;
  mInvM_a.noalias() -= getLocalJacobianStatic().transpose() * _bodyForce;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::getInvMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertia()
      * (mInvM_a - getLocalJacobianStatic().transpose()
         * _artInertia * math::AdInvT(getLocalTransform(), _spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  _invMassMat.block<DOF, 1>(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertiaImplicit()
      * (mInvM_a - getLocalJacobianStatic().transpose()
         * _artInertia * math::AdInvT(getLocalTransform(), _spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  _invMassMat.block<DOF, 1>(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
template <size_t DOF>
void MultiDofJoint<DOF>::addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
{
  //
  _acc += getLocalJacobianStatic() * mInvMassMatrixSegment;
}

//==============================================================================
template <size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getSpatialToGeneralized(
    const Eigen::Vector6d& _spatial)
{
  return getLocalJacobianStatic().transpose() * _spatial;
}

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_MULTIDOFJOINT_H_
