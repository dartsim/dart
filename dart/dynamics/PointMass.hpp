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

#ifndef DART_DYNAMICS_POINTMASS_HPP_
#define DART_DYNAMICS_POINTMASS_HPP_

#include <vector>
#include <Eigen/Dense>
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/Entity.hpp"

namespace dart {
namespace dynamics {

class EllipsoidShape;
class SoftBodyNode;

class PointMassNotifier;

///
class PointMass : public common::Subject
{
public:
  friend class SoftBodyNode;

  /// State for each PointMass
  struct State
  {
    /// Position
    Eigen::Vector3d mPositions;

    /// Generalized velocity
    Eigen::Vector3d mVelocities;

    /// Generalized acceleration
    Eigen::Vector3d mAccelerations;

    /// Generalized force
    Eigen::Vector3d mForces;

    /// Default constructor
    State(const Eigen::Vector3d& positions = Eigen::Vector3d::Zero(),
          const Eigen::Vector3d& velocities = Eigen::Vector3d::Zero(),
          const Eigen::Vector3d& accelerations = Eigen::Vector3d::Zero(),
          const Eigen::Vector3d& forces = Eigen::Vector3d::Zero());

    bool operator==(const State& other) const;

    virtual ~State() = default;
  };

  /// Properties for each PointMass
  struct Properties
  {
    /// Resting position viewed in the parent SoftBodyNode frame
    Eigen::Vector3d mX0;

    /// Mass.
    double mMass;

    /// Indices of connected Point Masses
    std::vector<std::size_t> mConnectedPointMassIndices;

    /// Lower limit of position
    Eigen::Vector3d mPositionLowerLimits; // Currently unused

    /// Upper limit of position
    Eigen::Vector3d mPositionUpperLimits; // Currently unused

    /// Min value allowed.
    Eigen::Vector3d mVelocityLowerLimits; // Currently unused

    /// Max value allowed.
    Eigen::Vector3d mVelocityUpperLimits; // Currently unused

    /// Min value allowed.
    Eigen::Vector3d mAccelerationLowerLimits; // Currently unused

    /// upper limit of generalized acceleration
    Eigen::Vector3d mAccelerationUpperLimits; // Currently unused

    /// Min value allowed.
    Eigen::Vector3d mForceLowerLimits; // Currently unused

    /// Max value allowed.
    Eigen::Vector3d mForceUpperLimits; // Currently unused

    Properties(const Eigen::Vector3d& _X0 = Eigen::Vector3d::Zero(),
               double _mass = 0.0005,
               const std::vector<std::size_t>& _connections = std::vector<std::size_t>(),
               const Eigen::Vector3d& _positionLowerLimits =
                                      Eigen::Vector3d::Constant(-math::constantsd::inf()),
               const Eigen::Vector3d& _positionUpperLimits =
                                      Eigen::Vector3d::Constant( math::constantsd::inf()),
               const Eigen::Vector3d& _velocityLowerLimits =
                                      Eigen::Vector3d::Constant(-math::constantsd::inf()),
               const Eigen::Vector3d& _velocityUpperLimits =
                                      Eigen::Vector3d::Constant( math::constantsd::inf()),
               const Eigen::Vector3d& _accelerationLowerLimits =
                                      Eigen::Vector3d::Constant(-math::constantsd::inf()),
               const Eigen::Vector3d& _accelerationUpperLimits =
                                      Eigen::Vector3d::Constant( math::constantsd::inf()),
               const Eigen::Vector3d& _forceLowerLimits =
                                      Eigen::Vector3d::Constant(-math::constantsd::inf()),
               const Eigen::Vector3d& _forceUpperLimits =
                                      Eigen::Vector3d::Constant( math::constantsd::inf()));

    void setRestingPosition(const Eigen::Vector3d& _x);

    void setMass(double _mass);

    bool operator==(const Properties& other) const;

    bool operator!=(const Properties& other) const;

    virtual ~Properties() = default;
  };

  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------

  /// Default destructor
  virtual ~PointMass();

  /// State of this PointMass
  State& getState();

  /// State of this PointMass
  const State& getState() const;

  ///
  std::size_t getIndexInSoftBodyNode() const;

  ///
  void setMass(double _mass);

  ///
  double getMass() const;

  ///
  double getPsi() const;

  ///
  double getImplicitPsi() const;

  ///
  double getPi() const;

  ///
  double getImplicitPi() const;

  ///
  void addConnectedPointMass(PointMass* _pointMass);

  ///
  std::size_t getNumConnectedPointMasses() const;

  ///
  PointMass* getConnectedPointMass(std::size_t _idx);

  ///
  const PointMass* getConnectedPointMass(std::size_t _idx) const;


  /// Set whether this point mass is colliding with other objects. Note that
  /// this status is set by the constraint solver during dynamics simulation but
  /// not by collision detector.
  /// \param[in] True if this point mass is colliding.
  void setColliding(bool _isColliding);

  /// Return whether this point mass is set to be colliding with other objects.
  /// \return True if this point mass is colliding.
  bool isColliding();

  //----------------------------------------------------------------------------

  // Documentation inherited
  std::size_t getNumDofs() const;

//  // Documentation inherited
//  void setIndexInSkeleton(std::size_t _index, std::size_t _indexInSkeleton);

//  // Documentation inherited
//  std::size_t getIndexInSkeleton(std::size_t _index) const;

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setPosition(std::size_t _index, double _position);

  // Documentation inherited
  double getPosition(std::size_t _index) const;

  // Documentation inherited
  void setPositions(const Eigen::Vector3d& _positions);

  // Documentation inherited
  const Eigen::Vector3d& getPositions() const;

  // Documentation inherited
  void resetPositions();

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setVelocity(std::size_t _index, double _velocity);

  // Documentation inherited
  double getVelocity(std::size_t _index) const;

  // Documentation inherited
  void setVelocities(const Eigen::Vector3d& _velocities);

  // Documentation inherited
  const Eigen::Vector3d& getVelocities() const;

  // Documentation inherited
  void resetVelocities();

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setAcceleration(std::size_t _index, double _acceleration);

  // Documentation inherited
  double getAcceleration(std::size_t _index) const;

  // Documentation inherited
  void setAccelerations(const Eigen::Vector3d& _accelerations);

  // Documentation inherited
  const Eigen::Vector3d& getAccelerations() const;

  /// Get the Eta term of this PointMass
  const Eigen::Vector3d& getPartialAccelerations() const;

  // Documentation inherited
  void resetAccelerations();

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setForce(std::size_t _index, double _force);

  // Documentation inherited
  double getForce(std::size_t _index);

  // Documentation inherited
  void setForces(const Eigen::Vector3d& _forces);

  // Documentation inherited
  const Eigen::Vector3d& getForces() const;

  // Documentation inherited
  void resetForces();

  //----------------------------------------------------------------------------
  // Velocity change
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setVelocityChange(std::size_t _index, double _velocityChange);

  // Documentation inherited
  double getVelocityChange(std::size_t _index);

  // Documentation inherited
  void resetVelocityChanges();

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setConstraintImpulse(std::size_t _index, double _impulse);

  // Documentation inherited
  double getConstraintImpulse(std::size_t _index);

  // Documentation inherited
  void resetConstraintImpulses();

  //----------------------------------------------------------------------------
  // Integration
  //----------------------------------------------------------------------------

  // Documentation inherited
  void integratePositions(double _dt);

  // Documentation inherited
  void integrateVelocities(double _dt);

  //----------------------------------------------------------------------------

  /// Add linear Cartesian force to this node.
  /// \param[in] _force External force.
  /// \param[in] _isForceLocal True if _force's reference frame is of the parent
  ///                          soft body node. False if _force's reference frame
  ///                          is of the world.
  void addExtForce(const Eigen::Vector3d& _force, bool _isForceLocal = false);

  ///
  void clearExtForce();

  //----------------------------------------------------------------------------
  // Constraints
  //   - Following functions are managed by constraint solver.
  //----------------------------------------------------------------------------
  /// Set constraint impulse
  void setConstraintImpulse(const Eigen::Vector3d& _constImp,
                            bool _isLocal = false);

  /// Add constraint impulse
  void addConstraintImpulse(const Eigen::Vector3d& _constImp,
                            bool _isLocal = false);

  /// Clear constraint impulse
  void clearConstraintImpulse();

  /// Get constraint impulse
  Eigen::Vector3d getConstraintImpulses() const;

  //----------------------------------------------------------------------------
  ///
  void setRestingPosition(const Eigen::Vector3d& _p);

  ///
  const Eigen::Vector3d& getRestingPosition() const;

  ///
  const Eigen::Vector3d& getLocalPosition() const;

  ///
  const Eigen::Vector3d& getWorldPosition() const;

  /// \todo Temporary function.
  Eigen::Matrix<double, 3, Eigen::Dynamic> getBodyJacobian();
  Eigen::Matrix<double, 3, Eigen::Dynamic> getWorldJacobian();

  /// Return velocity change due to impulse
  const Eigen::Vector3d& getBodyVelocityChange() const;

  ///
  SoftBodyNode* getParentSoftBodyNode();

  ///
  const SoftBodyNode* getParentSoftBodyNode() const;

  /// The number of the generalized coordinates by which this node is
  ///        affected.
//  int getNumDependentGenCoords() const;

  /// Return a generalized coordinate index from the array index
  ///        (< getNumDependentDofs).
//  int getDependentGenCoord(int _arrayIndex) const;

  /// Get the generalized velocity at the position of this point mass
  ///        where the velocity is expressed in the parent soft body node frame.
  const Eigen::Vector3d& getBodyVelocity() const;

  /// Get the generalized velocity at the position of this point mass
  ///        where the velocity is expressed in the world frame.
  Eigen::Vector3d getWorldVelocity() const;

  /// Get the generalized acceleration at the position of this point mass
  ///        where the acceleration is expressed in the parent soft body node
  ///        frame.
  const Eigen::Vector3d& getBodyAcceleration() const;

  /// Get the generalized acceleration at the position of this point mass
  ///        where the acceleration is expressed in the world frame.
  Eigen::Vector3d getWorldAcceleration() const;

protected:
  /// Constructor used by SoftBodyNode
  explicit PointMass(SoftBodyNode* _softBodyNode);

  ///
  void init();

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  /// \brief Update transformation.
  void updateTransform() const;

  /// \brief Update body velocity.
  void updateVelocity() const;

  /// \brief Update partial body acceleration due to parent joint's velocity.
  void updatePartialAcceleration() const;

  /// \brief Update articulated body inertia. Forward dynamics routine.
  /// \param[in] _timeStep Rquired for implicit joint stiffness and damping.
  void updateArtInertiaFD(double _timeStep) const;

  /// \brief Update bias force associated with the articulated body inertia.
  /// Forward dynamics routine.
  /// \param[in] _gravity Vector of gravitational acceleration
  /// \param[in] _timeStep Rquired for implicit joint stiffness and damping.
  void updateBiasForceFD(double _dt, const Eigen::Vector3d& _gravity);

  /// \brief Update bias impulse associated with the articulated body inertia.
  /// Impulse-based forward dynamics routine.
  void updateBiasImpulseFD();

  /// \brief Update body acceleration with the partial body acceleration.
  void updateAccelerationID() const;

  /// \brief Update body acceleration. Forward dynamics routine.
  void updateAccelerationFD();

  /// \brief Update body velocity change. Impluse-based forward dynamics
  /// routine.
  void updateVelocityChangeFD();

  /// \brief Update body force. Inverse dynamics routine.
  void updateTransmittedForceID(const Eigen::Vector3d& _gravity,
                                bool _withExternalForces = false);

  /// \brief Update body force. Forward dynamics routine.
  void updateTransmittedForce();

  /// \brief Update body force. Impulse-based forward dynamics routine.
  void updateTransmittedImpulse();

  /// \brief Update the joint force. Inverse dynamics routine.
  void updateJointForceID(double _timeStep,
                          double _withDampingForces,
                          double _withSpringForces);

  /// \brief Update constrained terms due to the constraint impulses. Foward
  /// dynamics routine.
  void updateConstrainedTermsFD(double _timeStep);

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Equations of motion related routines
  //----------------------------------------------------------------------------

  ///
  void updateMassMatrix();

  ///
  void aggregateMassMatrix(Eigen::MatrixXd& _MCol, int _col);

  ///
  void aggregateAugMassMatrix(Eigen::MatrixXd& _MCol, int _col,
                              double _timeStep);

  ///
  void updateInvMassMatrix();

  ///
  void updateInvAugMassMatrix();

  ///
  void aggregateInvMassMatrix(Eigen::MatrixXd& _MInvCol, int _col);

  ///
  void aggregateInvAugMassMatrix(Eigen::MatrixXd& _MInvCol, int _col,
                                 double _timeStep);

  ///
  void aggregateGravityForceVector(Eigen::VectorXd& _g,
                                   const Eigen::Vector3d& _gravity);

  ///
  void updateCombinedVector();

  ///
  void aggregateCombinedVector(Eigen::VectorXd& _Cg,
                               const Eigen::Vector3d& _gravity);

  /// Aggregate the external forces mFext in the generalized
  ///        coordinates recursively.
  void aggregateExternalForces(Eigen::VectorXd& _Fext);

  /// \}

  //-------------------- Cache Data for Mass Matrix ----------------------------
  ///
  Eigen::Vector3d mM_dV;

  ///
  Eigen::Vector3d mM_F;

  //----------------- Cache Data for Mass Inverse Matrix -----------------------
  ///
  Eigen::Vector3d mBiasForceForInvMeta;

  //---------------- Cache Data for Gravity Force Vector -----------------------
  ///
  Eigen::Vector3d mG_F;

  //------------------- Cache Data for Combined Vector -------------------------
  ///
  Eigen::Vector3d mCg_dV;

  ///
  Eigen::Vector3d mCg_F;

protected:
  // TODO(JS): Need?
  ///
//  Eigen::Matrix<std::size_t, 3, 1> mIndexInSkeleton;

  /// SoftBodyNode that this PointMass belongs to
  SoftBodyNode* mParentSoftBodyNode;

  /// Index of this PointMass within the SoftBodyNode
  std::size_t mIndex;

  //----------------------------------------------------------------------------
  // Configuration
  //----------------------------------------------------------------------------

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mPositionDeriv;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mVelocitiesDeriv;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mAccelerationsDeriv;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------


  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mForcesDeriv;

  //----------------------------------------------------------------------------
  // Impulse
  //----------------------------------------------------------------------------

  /// Change of generalized velocity
  Eigen::Vector3d mVelocityChanges;

//  /// Generalized impulse
//  Eigen::Vector3d mImpulse;

  /// Generalized constraint impulse
  Eigen::Vector3d mConstraintImpulses;

  //----------------------------------------------------------------------------

  /// Current position viewed in world frame.
  mutable Eigen::Vector3d mW;

  /// Current position viewed in parent soft body node frame.
  mutable Eigen::Vector3d mX;

  /// Current velocity viewed in parent soft body node frame.
  mutable Eigen::Vector3d mV;

  /// Partial Acceleration of this PointMass
  mutable Eigen::Vector3d mEta;

  ///
  Eigen::Vector3d mAlpha;

  ///
  Eigen::Vector3d mBeta;

  /// Current acceleration viewed in parent body node frame.
  mutable Eigen::Vector3d mA;

  ///
  Eigen::Vector3d mF;

  ///
  mutable double mPsi;

  ///
  mutable double mImplicitPsi;

  ///
  mutable double mPi;

  ///
  mutable double mImplicitPi;

  /// Bias force
  Eigen::Vector3d mB;

  /// External force.
  Eigen::Vector3d mFext;

  /// A increasingly sorted list of dependent dof indices.
  std::vector<std::size_t> mDependentGenCoordIndices;

  /// Whether the node is currently in collision with another node.
  bool mIsColliding;

  //------------------------- Impulse-based Dyanmics ---------------------------
  /// Velocity change due to constraint impulse
  Eigen::Vector3d mDelV;

  /// Impulsive bias force due to external impulsive force exerted on
  ///        bodies of the parent skeleton.
  Eigen::Vector3d mImpB;

  /// Cache data for mImpB
  Eigen::Vector3d mImpAlpha;

  /// Cache data for mImpB
  Eigen::Vector3d mImpBeta;

  /// Generalized impulsive body force w.r.t. body frame.
  Eigen::Vector3d mImpF;

  PointMassNotifier* mNotifier;
};

//struct PointMassPair
//{
//  PointMass* pm1;
//  PointMass* pm2;
//};

class PointMassNotifier : public Entity
{
public:

  PointMassNotifier(SoftBodyNode* _parentSoftBody, const std::string& _name);

  bool needsPartialAccelerationUpdate() const;

  void clearTransformNotice();
  void clearVelocityNotice();
  void clearPartialAccelerationNotice();
  void clearAccelerationNotice();

  void dirtyTransform() override;
  void dirtyVelocity() override;
  void dirtyAcceleration() override;

  // Documentation inherited
  const std::string& setName(const std::string& _name) override;

  // Documentation inherited
  const std::string& getName() const override;

protected:

  std::string mName;

  bool mNeedPartialAccelerationUpdate;
  // TODO(JS): Rename this to mIsPartialAccelerationDirty in DART 7

  SoftBodyNode* mParentSoftBodyNode;

};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_POINTMASS_HPP_
