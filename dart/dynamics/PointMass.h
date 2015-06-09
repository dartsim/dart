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

#ifndef DART_DYNAMICS_POINTMASS_H_
#define DART_DYNAMICS_POINTMASS_H_

#include <vector>
#include <Eigen/Dense>
#include "dart/common/Deprecated.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class EllipsoidShape;
class SoftBodyNode;

///
class PointMass
{
public:
  friend class SoftBodyNode;

  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------
  /// Default constructor
  explicit PointMass(SoftBodyNode* _softBodyNode);

  /// Default destructor
  virtual ~PointMass();

  ///
  void setMass(double _mass);

  ///
  double getMass() const;

  ///
  void addConnectedPointMass(PointMass* _pointMass);

  ///
  int getNumConnectedPointMasses() const;

  ///
  PointMass* getConnectedPointMass(size_t _idx);

  ///
  const PointMass* getConnectedPointMass(size_t _idx) const;


  /// Set whether this point mass is colliding with others.
  /// \param[in] True if this point mass is colliding.
  void setColliding(bool _isColliding);

  /// Get whether this point mass is colliding with others.
  /// \return True if this point mass is colliding.
  bool isColliding();

  //----------------------------------------------------------------------------

  // Documentation inherited
  size_t getNumDofs() const;

//  // Documentation inherited
//  void setIndexInSkeleton(size_t _index, size_t _indexInSkeleton);

//  // Documentation inherited
//  size_t getIndexInSkeleton(size_t _index) const;

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setPosition(size_t _index, double _position);

  // Documentation inherited
  double getPosition(size_t _index) const;

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
  void setVelocity(size_t _index, double _velocity);

  // Documentation inherited
  double getVelocity(size_t _index) const;

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
  void setAcceleration(size_t _index, double _acceleration);

  // Documentation inherited
  double getAcceleration(size_t _index) const;

  // Documentation inherited
  void setAccelerations(const Eigen::Vector3d& _accelerations);

  // Documentation inherited
  const Eigen::Vector3d& getAccelerations() const;

  // Documentation inherited
  void resetAccelerations();

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setForce(size_t _index, double _force);

  // Documentation inherited
  double getForce(size_t _index);

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
  void setVelocityChange(size_t _index, double _velocityChange);

  // Documentation inherited
  double getVelocityChange(size_t _index);

  // Documentation inherited
  void resetVelocityChanges();

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setConstraintImpulse(size_t _index, double _impulse);

  // Documentation inherited
  double getConstraintImpulse(size_t _index);

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
  ///
  void init();

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  /// \brief Update transformation.
  void updateTransform();

  /// \brief Update body velocity.
  void updateVelocity();

  /// \brief Update partial body acceleration due to parent joint's velocity.
  void updatePartialAcceleration();

  /// \brief Update articulated body inertia. Forward dynamics routine.
  /// \param[in] _timeStep Rquired for implicit joint stiffness and damping.
  void updateArtInertiaFD(double _timeStep);

  /// \brief Update bias force associated with the articulated body inertia.
  /// Forward dynamics routine.
  /// \param[in] _gravity Vector of gravitational acceleration
  /// \param[in] _timeStep Rquired for implicit joint stiffness and damping.
  void updateBiasForceFD(double _dt, const Eigen::Vector3d& _gravity);

  /// \brief Update bias impulse associated with the articulated body inertia.
  /// Impulse-based forward dynamics routine.
  void updateBiasImpulseFD();

  /// \brief Update body acceleration with the partial body acceleration.
  void updateAccelerationID();

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

  //- DEPRECATED ---------------------------------------------------------------

  /// \warning Please use updateAccelerationID().
  DEPRECATED(4.3)
  void updateAcceleration();

  /// \warning Please use updateTransmittedForceID().
  DEPRECATED(4.3)
  void updateBodyForce(const Eigen::Vector3d& _gravity,
                       bool _withExternalForces = false);

  /// \warning Please use updateArtInertiaFD().
  DEPRECATED(4.3)
  void updateArticulatedInertia(double _dt);

  /// \warning Please use updateJointForceID().
  DEPRECATED(4.3)
  void updateGeneralizedForce(bool _withDampingForces = false);

  /// \warning Please use updateAccelerationFD().
  DEPRECATED(4.3)
  void updateJointAndBodyAcceleration();

  /// \brief Update impulsive bias force for impulse-based forward dynamics
  /// algorithm.
  /// \warning Please use updateBiasImpulseFD().
  DEPRECATED(4.3)
  void updateBiasImpulse();

  /// \brief Update joint velocity change for impulse-based forward dynamics
  /// algorithm.
  /// \warning Please use updateVelocityChangeFD().
  DEPRECATED(4.3)
  void updateJointVelocityChange();

  /// \brief Update body velocity change for impulse-based forward dynamics
  /// algorithm.
  DEPRECATED(4.3)
  void updateBodyVelocityChange();

  /// \warning Please use updateTransmittedImpulse().
  DEPRECATED(4.3)
  void updateBodyImpForceFwdDyn();

  DEPRECATED(4.3)
  void updateConstrainedJointAndBodyAcceleration(double _timeStep);

  DEPRECATED(4.3)
  void updateConstrainedTransmittedForce(double _timeStep);

  DEPRECATED(4.3)
  void updateVelocityWithVelocityChange();

  DEPRECATED(4.3)
  void updateAccelerationWithVelocityChange(double _timeStep);

  DEPRECATED(4.3)
  void updateForceWithImpulse(double _timeStep);

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Equations of motion related routines
  //----------------------------------------------------------------------------

  ///
  void updateMassMatrix();

  ///
  void aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col);

  ///
  void aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                              double _timeStep);

  ///
  void updateInvMassMatrix();

  ///
  void updateInvAugMassMatrix();

  ///
  void aggregateInvMassMatrix(Eigen::MatrixXd* _MInvCol, int _col);

  ///
  void aggregateInvAugMassMatrix(Eigen::MatrixXd* _MInvCol, int _col,
                                 double _timeStep);

  ///
  void aggregateGravityForceVector(Eigen::VectorXd* _g,
                                   const Eigen::Vector3d& _gravity);

  ///
  void updateCombinedVector();

  ///
  void aggregateCombinedVector(Eigen::VectorXd* _Cg,
                               const Eigen::Vector3d& _gravity);

  /// Aggregate the external forces mFext in the generalized
  ///        coordinates recursively.
  void aggregateExternalForces(Eigen::VectorXd* _Fext);

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

  //---------------------------- Rendering -------------------------------------
  ///
  virtual void draw(renderer::RenderInterface* _ri = NULL,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true) const;

protected:
  // TODO(JS): Need?
  ///
//  Eigen::Matrix<size_t, 3, 1> mIndexInSkeleton;

  //----------------------------------------------------------------------------
  // Configuration
  //----------------------------------------------------------------------------

  /// Position
  Eigen::Vector3d mPositions;

  /// Lower limit of position
  Eigen::Vector3d mPositionLowerLimits;

  /// Upper limit of position
  Eigen::Vector3d mPositionUpperLimits;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mPositionDeriv;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  /// Generalized velocity
  Eigen::Vector3d mVelocities;

  /// Min value allowed.
  Eigen::Vector3d mVelocityLowerLimits;

  /// Max value allowed.
  Eigen::Vector3d mVelocityUpperLimits;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mVelocitiesDeriv;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  /// Generalized acceleration
  Eigen::Vector3d mAccelerations;

  /// Min value allowed.
  Eigen::Vector3d mAccelerationLowerLimits;

  /// upper limit of generalized acceleration
  Eigen::Vector3d mAccelerationUpperLimits;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mAccelerationsDeriv;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  /// Generalized force
  Eigen::Vector3d mForces;

  /// Min value allowed.
  Eigen::Vector3d mForceLowerLimits;

  /// Max value allowed.
  Eigen::Vector3d mForceUpperLimits;

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

  /// Mass.
  double mMass;

  /// Current position viewed in world frame.
  Eigen::Vector3d mW;

  /// Current position viewed in parent soft body node frame.
  Eigen::Vector3d mX;

  /// Resting postion viewed in parent soft body node frame.
  Eigen::Vector3d mX0;

  /// Current velocity viewed in parent soft body node frame.
  Eigen::Vector3d mV;

  ///
  Eigen::Vector3d mEta;

  ///
  Eigen::Vector3d mAlpha;

  ///
  Eigen::Vector3d mBeta;

  /// Current acceleration viewed in parent body node frame.
  Eigen::Vector3d mA;

  ///
  Eigen::Vector3d mF;

  ///
  double mPsi;

  ///
  double mImplicitPsi;

  ///
  double mPi;

  ///
  double mImplicitPi;

  /// Bias force
  Eigen::Vector3d mB;

  ///
  SoftBodyNode* mParentSoftBodyNode;

  ///
  std::vector<PointMass*> mConnectedPointMasses;

  /// External force.
  Eigen::Vector3d mFext;

  /// A increasingly sorted list of dependent dof indices.
  std::vector<int> mDependentGenCoordIndices;

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

private:
  EllipsoidShape* mShape;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//struct PointMassPair
//{
//  PointMass* pm1;
//  PointMass* pm2;
//};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_POINTMASS_H_
