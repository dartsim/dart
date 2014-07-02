/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_DYNAMICS_SKELETON_H_
#define DART_DYNAMICS_SKELETON_H_

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "dart/common/Deprecated.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class BodyNode;
class SoftBodyNode;
class PointMass;
class Joint;
class Marker;

/// struct GenCoordInfo
struct GenCoordInfo
{
  Joint* joint;
  size_t localIndex;
};

/// class Skeleton
class Skeleton
{
public:
  //----------------------------------------------------------------------------
  // Constructor and Destructor
  //----------------------------------------------------------------------------

  /// Constructor
  explicit Skeleton(const std::string& _name = "Skeleton");

  /// Destructor
  virtual ~Skeleton();

  //----------------------------------------------------------------------------
  // Properties
  //----------------------------------------------------------------------------

  /// Set name.
  void setName(const std::string& _name);

  /// Get name.
  const std::string& getName() const;

  /// Enable self collision check
  void enableSelfCollision(bool _enableAdjecentBodies = false);

  /// Disable self collision check
  void disableSelfCollision();

  /// Return true if self collision check is enabled
  bool isEnabledSelfCollisionCheck() const;

  /// Return true if self collision check is enabled including adjacent
  /// bodies
  bool isEnabledAdjacentBodyCheck() const;

  /// Set whether this skeleton will be updated by forward dynamics.
  /// \param[in] _isMobile True if this skeleton is mobile.
  void setMobile(bool _isMobile);

  /// Get whether this skeleton will be updated by forward dynamics.
  /// \return True if this skeleton is mobile.
  bool isMobile() const;

  /// Set time step. This timestep is used for implicit joint damping
  /// force.
  void setTimeStep(double _timeStep);

  /// Get time step.
  double getTimeStep() const;

  /// Set 3-dim gravitational acceleration. The gravity is used for
  /// calculating gravity force vector of the skeleton.
  void setGravity(const Eigen::Vector3d& _gravity);

  /// Get 3-dim gravitational acceleration.
  const Eigen::Vector3d& getGravity() const;

  /// Get total mass of the skeleton. The total mass is calculated at
  /// init().
  double getMass() const;

  //----------------------------------------------------------------------------
  // Structueral Properties
  //----------------------------------------------------------------------------

  /// Add a body node
  void addBodyNode(BodyNode* _body);

  /// Get number of body nodes
  size_t getNumBodyNodes() const;

  /// Get number of rigid body nodes.
  size_t getNumRigidBodyNodes() const;

  /// Get number of soft body nodes.
  size_t getNumSoftBodyNodes() const;

  /// Get root body node
  BodyNode* getRootBodyNode() const;

  /// Get body node whose index is _idx
  BodyNode* getBodyNode(size_t _idx) const;

  /// Get soft body node.
  SoftBodyNode* getSoftBodyNode(size_t _idx) const;

  /// Get body node whose name is _name
  BodyNode* getBodyNode(const std::string& _name) const;

  /// Get soft body node.
  SoftBodyNode* getSoftBodyNode(const std::string& _name) const;

  /// Get joint whose index is _idx
  Joint* getJoint(size_t _idx) const;

  /// Get joint whose name is _name
  Joint* getJoint(const std::string& _name) const;

  /// Get marker whose name is _name
  Marker* getMarker(const std::string& _name) const;

  //----------------------------------------------------------------------------
  // Initialization
  //----------------------------------------------------------------------------
  /// Initialize this skeleton for kinematics and dynamics
  void init(double _timeStep = 0.001,
            const Eigen::Vector3d& _gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

  //----------------------------------------------------------------------------
  // Generalized coordinate system
  //----------------------------------------------------------------------------

  /// Return degrees of freedom of this skeleton
  DEPRECATED(4.1) size_t getDof() const;

  /// Return degrees of freedom of this skeleton
  size_t getNumDofs() const;

  ///
  GenCoordInfo getGenCoordInfo(size_t _index) const;

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  /// Set a single position
  void setPosition(size_t _index, double _position);

  /// Get a single position
  double getPosition(size_t _index) const;

  /// Set configurations defined in terms of generalized coordinates and update
  /// Cartesian terms of body nodes using following parameters.
  void setPositions(const Eigen::VectorXd& _positions);

  /// Get positions
  Eigen::VectorXd getPositions() const;

  /// Set the configuration of this skeleton described in generalized
  /// coordinates. The order of input configuration is determined by _id.
  void setPositionSegment(const std::vector<size_t>& _id,
                          const Eigen::VectorXd& _positions);

  /// Get the configuration of this skeleton described in generalized
  /// coordinates. The returned order of configuration is determined by _id.
  Eigen::VectorXd getPositionSegment(
      const std::vector<size_t>& _id) const;

  /// Set zero all the positions
  void resetPositions();

  /// Set lower limit of position
  void setPositionLowerLimit(size_t _index, double _position);

  /// Get lower limit for position
  double getPositionLowerLimit(size_t _index);

  /// Set upper limit for position
  void setPositionUpperLimit(size_t _index, double _position);

  /// Get upper limit for position
  double getPositionUpperLimit(size_t _index);

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  /// Set a single velocity
  void setVelocity(size_t _index, double _velocity);

  /// Get a single velocity
  double getVelocity(size_t _index) const;

  /// Set generalized velocities
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  void setVelocities(const Eigen::VectorXd& _genVels);

  /// Get velocities
  Eigen::VectorXd getVelocities() const;

  /// Set zero all the velocities
  void resetVelocities();

  /// Set lower limit of velocity
  void setVelocityLowerLimit(size_t _index, double _velocity);

  /// Get lower limit of velocity
  double getVelocityLowerLimit(size_t _index);

  /// Set upper limit of velocity
  void setVelocityUpperLimit(size_t _index, double _velocity);

  /// Get upper limit of velocity
  double getVelocityUpperLimit(size_t _index);

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  /// Set a single acceleration
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  void setAcceleration(size_t _index, double _acceleration);

  /// Get a single acceleration
  double getAcceleration(size_t _index) const;

  /// Set generalized accelerations
  void setAccelerations(const Eigen::VectorXd& _accelerations);

  /// Get accelerations
  Eigen::VectorXd getAccelerations() const;

  /// Set zero all the accelerations
  void resetAccelerations();

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  /// Set a single force
  void setForce(size_t _index, double _force);

  /// Get a single force
  double getForce(size_t _index);

  /// Set forces
  void setForces(const Eigen::VectorXd& _forces);

  /// Get forces
  Eigen::VectorXd getForces() const;

  /// Set zero all the forces
  void resetForces();

  /// Set lower limit of force
  void setForceLowerLimit(size_t _index, double _force);

  /// Get lower limit of force
  double getForceLowerLimit(size_t _index);

  /// Set upper limit of position
  void setForceUpperLimit(size_t _index, double _force);

  /// Get upper limit of position
  double getForceUpperLimit(size_t _index);

  //----------------------------------------------------------------------------
  // Velocity change
  //----------------------------------------------------------------------------

  ///
  Eigen::VectorXd getVelocityChanges() const;

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  ///
  void setConstraintImpulses(const Eigen::VectorXd& _impulses);

  ///
  Eigen::VectorXd getConstraintImpulses() const;

  //----------------------------------------------------------------------------
  // Integration
  //----------------------------------------------------------------------------

  // Documentation inherited
  void integratePositions(double _dt);

  // Documentation inherited
  void integrateVelocities(double _dt);

  //----------------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------------

  /// Set the state of this skeleton described in generalized coordinates
  void setState(const Eigen::VectorXd& _state);

  /// Get the state of this skeleton described in generalized coordinates
  Eigen::VectorXd getState() const;

  //----------------------------------------------------------------------------
  // Kinematics algorithms
  //----------------------------------------------------------------------------

  /// Compute forward kinematics
  void computeForwardKinematics(bool _updateTransforms = true,
                                bool _updateVels = true,
                                bool _updateAccs = true);

  //----------------------------------------------------------------------------
  // Dynamics algorithms
  //----------------------------------------------------------------------------

  /// Compute forward dynamics
  void computeForwardDynamics();

  /// Compute inverse dynamics
  void computeInverseDynamics(bool _withExternalForces = false,
                              bool _withDampingForces = false);

  /// Compute hybrid dynamics
//  void computeHybridDynamics();

  //----------------------------------------------------------------------------
  // Impulse-based dynamics algorithms
  //----------------------------------------------------------------------------

  /// Clear constraint impulses: (a) spatial constraints on BodyNode and
  /// (b) generalized constraints on Joint
  virtual void clearConstraintImpulses();

  // TODO(JS): To be deprecated
  /// Set constraint force vector.
  void setConstraintForceVector(const Eigen::VectorXd& _Fc);

  /// Update bias impulses
  void updateBiasImpulse(BodyNode* _bodyNode);

  /// Update bias impulses due to impulse[_imp] on body node [_bodyNode]
  void updateBiasImpulse(BodyNode* _bodyNode, const Eigen::Vector6d& _imp);

  /// Update bias impulses due to impulse[_imp] on body node [_bodyNode]
  void updateBiasImpulse(SoftBodyNode* _softBodyNode,
                         PointMass* _pointMass,
                         const Eigen::Vector3d& _imp);

  /// Update velocity changes in body nodes and joints due to applied
  /// impulse
  void updateVelocityChange();

  // TODO(JS): Better naming
  /// Set whether this skeleton is constrained. ConstraintSolver will
  ///  mark this.
  void setImpulseApplied(bool _val);

  /// Get whether this skeleton is constrained
  bool isImpulseApplied() const;

  /// Compute impulse-based forward dynamics
  void computeImpulseForwardDynamics();

  /// Compute impulse-based inverse dynamics
//  void computeImpulseInverseDynamics() {}

  /// Compute impulse-based hybrid dynamics
//  void computeImpulseHybridDynamics() {}

  //----------------------------------------------------------------------------
  // Equations of Motion
  //----------------------------------------------------------------------------

  /// Get mass matrix of the skeleton.
  const Eigen::MatrixXd& getMassMatrix();

  /// Get augmented mass matrix of the skeleton. This matrix is used
  /// in ConstraintDynamics to compute constraint forces. The matrix is
  /// M + h*D + h*h*K where D is diagonal joint damping coefficient matrix, K is
  /// diagonal joint stiffness matrix, and h is simulation time step.
  const Eigen::MatrixXd& getAugMassMatrix();

  /// Get inverse of mass matrix of the skeleton.
  const Eigen::MatrixXd& getInvMassMatrix();

  /// Get inverse of augmented mass matrix of the skeleton.
  const Eigen::MatrixXd& getInvAugMassMatrix();

  /// Get Coriolis force vector of the skeleton.
  const Eigen::VectorXd& getCoriolisForceVector();

  /// Get gravity force vector of the skeleton.
  const Eigen::VectorXd& getGravityForceVector();

  /// Get combined vector of Coriolis force and gravity force of the skeleton.
  const Eigen::VectorXd& getCombinedVector();

  /// Get external force vector of the skeleton.
  const Eigen::VectorXd& getExternalForceVector();

  /// Get damping force of the skeleton.
//  const Eigen::VectorXd& getDampingForceVector();

  /// Get constraint force vector.
  const Eigen::VectorXd& getConstraintForceVector();

  /// Set internal force vector.
//  void setInternalForceVector(const Eigen::VectorXd& _forces);

  /// Clear internal forces.
//  void clearInternalForces();

  /// Clear external forces, which are manually added to the body nodes
  /// by the user.
  void clearExternalForces();

  //----------------------------------------------------------------------------

  /// Get skeleton's COM w.r.t. world frame.
  Eigen::Vector3d getWorldCOM();

  /// Get skeleton's COM velocity w.r.t. world frame.
  Eigen::Vector3d getWorldCOMVelocity();

  /// Get skeleton's COM acceleration w.r.t. world frame.
  Eigen::Vector3d getWorldCOMAcceleration();

  /// Get skeleton's COM Jacobian w.r.t. world frame.
  Eigen::MatrixXd getWorldCOMJacobian();

  /// Get skeleton's COM Jacobian time derivative w.r.t. world frame.
  Eigen::MatrixXd getWorldCOMJacobianTimeDeriv();

  /// Get kinetic energy of this skeleton.
  virtual double getKineticEnergy() const;

  /// Get potential energy of this skeleton.
  virtual double getPotentialEnergy() const;

  //----------------------------------------------------------------------------
  // Rendering
  //----------------------------------------------------------------------------

  /// Draw this skeleton
  void draw(renderer::RenderInterface* _ri = NULL,
            const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
            bool _useDefaultColor = true) const;

  /// Draw markers in this skeleton
  void drawMarkers(renderer::RenderInterface* _ri = NULL,
                   const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                   bool _useDefaultColor = true) const;

public:
  /// Compute recursion part A of forward dynamics
  void computeForwardDynamicsRecursionPartA();

  /// Compute recursion part B of forward dynamics
  void computeForwardDynamicsRecursionPartB();

  /// Compute recursion part A of inverse dynamics
  void computeInverseDynamicsRecursionA();

  /// Compute recursion part B of inverse dynamics
  void computeInverseDynamicsRecursionB(bool _withExternalForces = false,
                                        bool _withDampingForces = false);

  /// Compute recursion part A of hybrid dynamics
//  void computeHybridDynamicsRecursionA();

  /// Compute recursion part B of hybrid dynamics
//  void computeHybridDynamicsRecursionB();

protected:
  /// Update mass matrix of the skeleton.
  virtual void updateMassMatrix();

  /// Update augmented mass matrix of the skeleton.
  virtual void updateAugMassMatrix();

  /// Update inverse of mass matrix of the skeleton.
  virtual void updateInvMassMatrix();

  /// Update inverse of augmented mass matrix of the skeleton.
  virtual void updateInvAugMassMatrix();

  /// Update Coriolis force vector of the skeleton.
  virtual void updateCoriolisForceVector();

  /// Update gravity force vector of the skeleton.
  virtual void updateGravityForceVector();

  /// Update combined vector of the skeletong.
  virtual void updateCombinedVector();

  /// update external force vector to generalized torques.
  virtual void updateExternalForceVector();

//  /// Update damping force vector.
//  virtual void updateDampingForceVector();

protected:
  /// Name
  std::string mName;

  /// Degrees of freedom
  size_t mDof;

  ///
  std::vector<GenCoordInfo> mGenCoordInfos;

  /// True if self collision check is enabled
  bool mEnabledSelfCollisionCheck;

  /// True if self collision check is enabled including adjacent bodies
  bool mEnabledAdjacentBodyCheck;

  /// List of body nodes in the skeleton.
  std::vector<BodyNode*> mBodyNodes;

  /// List of Soft body node list in the skeleton
  std::vector<SoftBodyNode*> mSoftBodyNodes;

  /// If the skeleton is not mobile, its dynamic effect is equivalent
  /// to having infinite mass. If the configuration of an immobile skeleton are
  /// manually changed, the collision results might not be correct.
  bool mIsMobile;

  /// Time step for implicit joint damping force.
  double mTimeStep;

  /// Gravity vector.
  Eigen::Vector3d mGravity;

  /// Total mass.
  double mTotalMass;

  /// Dirty flag for articulated body inertia
  bool mIsArticulatedInertiaDirty;

  /// Mass matrix for the skeleton.
  Eigen::MatrixXd mM;

  /// Dirty flag for the mass matrix.
  bool mIsMassMatrixDirty;

  /// Mass matrix for the skeleton.
  Eigen::MatrixXd mAugM;

  /// Dirty flag for the mass matrix.
  bool mIsAugMassMatrixDirty;

  /// Inverse of mass matrix for the skeleton.
  Eigen::MatrixXd mInvM;

  /// Dirty flag for the inverse of mass matrix.
  bool mIsInvMassMatrixDirty;

  /// Inverse of augmented mass matrix for the skeleton.
  Eigen::MatrixXd mInvAugM;

  /// Dirty flag for the inverse of augmented mass matrix.
  bool mIsInvAugMassMatrixDirty;

  /// Coriolis vector for the skeleton which is C(q,dq)*dq.
  Eigen::VectorXd mCvec;

  /// Dirty flag for the Coriolis force vector.
  bool mIsCoriolisVectorDirty;

  /// Gravity vector for the skeleton; computed in nonrecursive
  /// dynamics only.
  Eigen::VectorXd mG;

  /// Dirty flag for the gravity force vector.
  bool mIsGravityForceVectorDirty;

  /// Combined coriolis and gravity vector which is C(q, dq)*dq + g(q).
  Eigen::VectorXd mCg;

  /// Dirty flag for the combined vector of Coriolis and gravity.
  bool mIsCombinedVectorDirty;

  /// External force vector for the skeleton.
  Eigen::VectorXd mFext;

  /// Dirty flag for the external force vector.
  bool mIsExternalForceVectorDirty;

  /// Constraint force vector.
  Eigen::VectorXd mFc;

  /// Damping force vector.
  Eigen::VectorXd mFd;

  /// Dirty flag for the damping force vector.
  bool mIsDampingForceVectorDirty;

  // TODO(JS): Better naming
  /// Flag for status of impulse testing.
  bool mIsImpulseApplied;

  //----------------------------------------------------------------------------
  // Union finding
  //----------------------------------------------------------------------------
public:

  ///
  void resetUnion()
  {
    mUnionRootSkeleton = this;
    mUnionSize = 1;
  }

  ///
  Skeleton* mUnionRootSkeleton;

  ///
  size_t mUnionSize;

  ///
  size_t mUnionIndex;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SKELETON_H_
