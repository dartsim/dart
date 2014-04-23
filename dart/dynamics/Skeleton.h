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
#include "dart/dynamics/GenCoordSystem.h"

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

/// \brief class Skeleton
class Skeleton : public GenCoordSystem
{
public:
  //----------------------------------------------------------------------------
  // Constructor and Destructor
  //----------------------------------------------------------------------------
  /// \brief Constructor
  explicit Skeleton(const std::string& _name = "Skeleton");

  /// \brief Destructor
  virtual ~Skeleton();

  //----------------------------------------------------------------------------
  // Properties
  //----------------------------------------------------------------------------
  /// \brief Set name.
  void setName(const std::string& _name);

  /// \brief Get name.
  const std::string& getName() const;

  /// \brief Enable self collision check
  void enableSelfCollision(bool _enableAdjecentBodies = false);

  /// \brief Disable self collision check
  void disableSelfCollision();

  /// \brief Return true if self collision check is enabled
  bool isEnabledSelfCollisionCheck() const;

  /// \brief Return true if self collision check is enabled including adjacent
  /// bodies
  bool isEnabledAdjacentBodyCheck() const;

  /// \brief Set whether this skeleton will be updated by forward dynamics.
  /// \param[in] _isMobile True if this skeleton is mobile.
  void setMobile(bool _isMobile);

  /// \brief Get whether this skeleton will be updated by forward dynamics.
  /// \return True if this skeleton is mobile.
  bool isMobile() const;

  /// \brief Set time step. This timestep is used for implicit joint damping
  /// force.
  void setTimeStep(double _timeStep);

  /// \brief Get time step.
  double getTimeStep() const;

  /// \brief Set 3-dim gravitational acceleration. The gravity is used for
  /// calculating gravity force vector of the skeleton.
  void setGravity(const Eigen::Vector3d& _gravity);

  /// \brief Get 3-dim gravitational acceleration.
  const Eigen::Vector3d& getGravity() const;

  /// \brief Get total mass of the skeleton. The total mass is calculated at
  /// init().
  double getMass() const;

  //----------------------------------------------------------------------------
  // Structueral Properties
  //----------------------------------------------------------------------------
  /// \brief Add a body node
  void addBodyNode(BodyNode* _body);

  /// \brief Get number of body nodes
  int getNumBodyNodes() const;

  /// \brief Get number of rigid body nodes.
  int getNumRigidBodyNodes() const;

  /// \brief Get number of soft body nodes.
  int getNumSoftBodyNodes() const;

  /// \brief Get root body node
  BodyNode* getRootBodyNode() const;

  /// \brief Get body node whose index is _idx
  BodyNode* getBodyNode(int _idx) const;

  /// \brief Get soft body node.
  SoftBodyNode* getSoftBodyNode(int _idx) const;

  /// \brief Get body node whose name is _name
  BodyNode* getBodyNode(const std::string& _name) const;

  /// \brief Get soft body node.
  SoftBodyNode* getSoftBodyNode(const std::string& _name) const;

  /// \brief Get joint whose index is _idx
  Joint* getJoint(int _idx) const;

  /// \brief Get joint whose name is _name
  Joint* getJoint(const std::string& _name) const;

  /// \brief Get marker whose name is _name
  Marker* getMarker(const std::string& _name) const;

  //----------------------------------------------------------------------------
  // Initialization
  //----------------------------------------------------------------------------
  /// \brief Initialize this skeleton for kinematics and dynamics
  void init(double _timeStep = 0.001,
            const Eigen::Vector3d& _gravity = Eigen::Vector3d(0.0, 0.0, -9.81));


  //----------------------------------------------------------------------------
  // Interfaces for generalized coordinates
  //----------------------------------------------------------------------------
  /// \brief Set configurations defined in terms of generalized coordinates
  /// and update Cartesian terms of body nodes using following parameters.
  /// \param[in] _updateTransforms True to update transformations of body nodes
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setConfigs(const Eigen::VectorXd& _configs,
                          bool _updateTransforms = true,
                          bool _updateVels = false,
                          bool _updateAccs = false);

  /// \brief Set generalized velocities
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setGenVels(const Eigen::VectorXd& _genVels,
                          bool _updateVels = true,
                          bool _updateAccs = false);

  /// \brief Set generalized accelerations
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setGenAccs(const Eigen::VectorXd& _genAccs,
                          bool _updateAccs);

  /// \brief Set the configuration of this skeleton described in generalized
  /// coordinates. The order of input configuration is determined by _id.
  /// \param[in] _updateTransforms True to update transformations of body nodes
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  void setConfigSegs(const std::vector<int>& _id,
                     const Eigen::VectorXd& _configs,
                     bool _updateTransforms = true,
                     bool _updateVels = false,
                     bool _updateAccs = false);

  /// \brief Get the configuration of this skeleton described in generalized
  /// coordinates. The returned order of configuration is determined by _id.
  Eigen::VectorXd getConfigSegs(const std::vector<int>& _id) const;

  /// \brief Set the state of this skeleton described in generalized coordinates
  /// \param[in] _updateTransforms True to update transformations of body nodes
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  void setState(const Eigen::VectorXd& _state,
                bool _updateTransforms = true,
                bool _updateVels = true,
                bool _updateAccs = false);

  /// \brief Get the state of this skeleton described in generalized coordinates
  Eigen::VectorXd getState() const;

  //----------------------------------------------------------------------------
  // Integration
  //----------------------------------------------------------------------------
  // Documentation inherited
  virtual void integrateConfigs(double _dt);

  // Documentation inherited
  virtual void integrateGenVels(double _dt);

  //----------------------------------------------------------------------------
  // Kinematics algorithms
  //----------------------------------------------------------------------------
  /// \brief Compute forward kinematics
  void computeForwardKinematics(bool _updateTransforms = true,
                                bool _updateVels = true,
                                bool _updateAccs = true);

  //----------------------------------------------------------------------------
  // Dynamics algorithms
  //----------------------------------------------------------------------------
  /// \brief Compute forward dynamics
  void computeForwardDynamics();

  /// \brief Compute inverse dynamics
  void computeInverseDynamics(bool _withExternalForces = false,
                              bool _withDampingForces = false);

  /// \brief Compute hybrid dynamics
  void computeHybridDynamics();

  //----------------------------------------------------------------------------
  // Impulse-based dynamics
  //----------------------------------------------------------------------------
  /// \brief Clear constraint impulses: (a) spatial constraints on BodyNode and
  /// (b) generalized constraints on Joint
  virtual void clearConstraintImpulses();

  // TODO(JS): To be deprecated
  /// \brief Set constraint force vector.
  void setConstraintForceVector(const Eigen::VectorXd& _Fc);

  /// \brief Update bias impulses
  void updateBiasImpulse(BodyNode* _bodyNode);

  /// \brief Update bias impulses due to impulse[_imp] on body node [_bodyNode]
  void updateBiasImpulse(BodyNode* _bodyNode, const Eigen::Vector6d& _imp);

  /// \brief Update bias impulses due to impulse[_imp] on body node [_bodyNode]
  void updateBiasImpulse(SoftBodyNode* _softBodyNode,
                         PointMass* _pointMass,
                         const Eigen::Vector3d& _imp);

  /// \brief Update velocity changes in body nodes and joints due to applied
  /// impulse
  void updateVelocityChange();

  // TODO(JS): Better naming
  /// \brief Set whether this skeleton is constrained. ConstraintSolver will
  ///  mark this.
  void setImpulseApplied(bool _val);

  /// \brief Get whether this skeleton is constrained
  bool isImpulseApplied() const;

  /// \brief Compute impulse-based forward dynamics
  void computeImpulseForwardDynamics();

  /// \brief Compute impulse-based inverse dynamics
  void computeImpulseInverseDynamics() {}

  /// \brief Compute impulse-based hybrid dynamics
  void computeImpulseHybridDynamics() {}

  //----------------------------------------------------------------------------
  // Equations of Motion
  //----------------------------------------------------------------------------
  /// \brief Get mass matrix of the skeleton.
  const Eigen::MatrixXd& getMassMatrix();

  /// \brief Get augmented mass matrix of the skeleton. This matrix is used
  /// in ConstraintDynamics to compute constraint forces. The matrix is
  /// M + h*D + h*h*K where D is diagonal joint damping coefficient matrix, K is
  /// diagonal joint stiffness matrix, and h is simulation time step.
  const Eigen::MatrixXd& getAugMassMatrix();

  /// \brief Get inverse of mass matrix of the skeleton.
  const Eigen::MatrixXd& getInvMassMatrix();

  /// \brief Get inverse of augmented mass matrix of the skeleton.
  const Eigen::MatrixXd& getInvAugMassMatrix();

  /// \brief Get Coriolis force vector of the skeleton.
  const Eigen::VectorXd& getCoriolisForceVector();

  /// \brief Get gravity force vector of the skeleton.
  const Eigen::VectorXd& getGravityForceVector();

  /// \brief Get combined vector of Coriolis force and gravity force of the
  ///        skeleton.
  const Eigen::VectorXd& getCombinedVector();

  /// \brief Get external force vector of the skeleton.
  const Eigen::VectorXd& getExternalForceVector();

  /// \brief Get internal force vector of the skeleton.
  Eigen::VectorXd getInternalForceVector() const;

  /// \brief Get damping force of the skeleton.
  const Eigen::VectorXd& getDampingForceVector();

  /// \brief Get constraint force vector.
  const Eigen::VectorXd& getConstraintForceVector();

  /// \brief Set internal force vector.
  void setInternalForceVector(const Eigen::VectorXd& _forces);

  /// \brief Set upper limit of the internal force vector.
  void setMinInternalForceVector(const Eigen::VectorXd& _minForces);

  /// \brief Get lower limit of the internal force vector.
  Eigen::VectorXd getMinInternalForces() const;

  /// \brief Set upper limit of the internal force vector.
  void setMaxInternalForceVector(const Eigen::VectorXd& _maxForces);

  /// \brief Get upper limit of the internal force vector.
  Eigen::VectorXd getMaxInternalForceVector() const;

  /// \brief Clear internal forces.
  void clearInternalForces();

  /// \brief Clear external forces, which are manually added to the body nodes
  /// by the user.
  void clearExternalForces();

  //----------------------------------------------------------------------------
  /// \brief Get skeleton's COM w.r.t. world frame.
  Eigen::Vector3d getWorldCOM();

  /// \brief Get skeleton's COM velocity w.r.t. world frame.
  Eigen::Vector3d getWorldCOMVelocity();

  /// \brief Get skeleton's COM acceleration w.r.t. world frame.
  Eigen::Vector3d getWorldCOMAcceleration();

  /// \brief Get skeleton's COM Jacobian w.r.t. world frame.
  Eigen::MatrixXd getWorldCOMJacobian();

  /// \brief Get skeleton's COM Jacobian time derivative w.r.t. world frame.
  Eigen::MatrixXd getWorldCOMJacobianTimeDeriv();

  /// \brief Get kinetic energy of this skeleton.
  virtual double getKineticEnergy() const;

  /// \brief Get potential energy of this skeleton.
  virtual double getPotentialEnergy() const;

  //----------------------------------------------------------------------------
  // Rendering
  //----------------------------------------------------------------------------
  /// \brief Draw this skeleton
  void draw(renderer::RenderInterface* _ri = NULL,
            const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
            bool _useDefaultColor = true) const;

  /// \brief Draw markers in this skeleton
  void drawMarkers(renderer::RenderInterface* _ri = NULL,
                   const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                   bool _useDefaultColor = true) const;

protected:
  /// \brief Name
  std::string mName;

  /// \brief True if self collision check is enabled
  bool mEnabledSelfCollisionCheck;

  /// \brief True if self collision check is enabled including adjacent bodies
  bool mEnabledAdjacentBodyCheck;

  /// \brief List of body nodes in the skeleton.
  std::vector<BodyNode*> mBodyNodes;

  /// \brief List of Soft body node list in the skeleton
  std::vector<SoftBodyNode*> mSoftBodyNodes;

  /// \brief If the skeleton is not mobile, its dynamic effect is equivalent
  /// to having infinite mass. If the configuration of an immobile skeleton are
  /// manually changed, the collision results might not be correct.
  bool mIsMobile;

  /// \brief Time step for implicit joint damping force.
  double mTimeStep;

  /// \brief Gravity vector.
  Eigen::Vector3d mGravity;

  /// \brief Total mass.
  double mTotalMass;

  /// \brief Dirty flag for articulated body inertia
  bool mIsArticulatedInertiaDirty;

  /// \brief Mass matrix for the skeleton.
  Eigen::MatrixXd mM;

  /// \brief Dirty flag for the mass matrix.
  bool mIsMassMatrixDirty;

  /// \brief Mass matrix for the skeleton.
  Eigen::MatrixXd mAugM;

  /// \brief Dirty flag for the mass matrix.
  bool mIsAugMassMatrixDirty;

  /// \brief Inverse of mass matrix for the skeleton.
  Eigen::MatrixXd mInvM;

  /// \brief Dirty flag for the inverse of mass matrix.
  bool mIsInvMassMatrixDirty;

  /// \brief Inverse of augmented mass matrix for the skeleton.
  Eigen::MatrixXd mInvAugM;

  /// \brief Dirty flag for the inverse of augmented mass matrix.
  bool mIsInvAugMassMatrixDirty;

  /// \brief Coriolis vector for the skeleton which is C(q,dq)*dq.
  Eigen::VectorXd mCvec;

  /// \brief Dirty flag for the Coriolis force vector.
  bool mIsCoriolisVectorDirty;

  /// \brief Gravity vector for the skeleton; computed in nonrecursive
  /// dynamics only.
  Eigen::VectorXd mG;

  /// \brief Dirty flag for the gravity force vector.
  bool mIsGravityForceVectorDirty;

  /// \brief Combined coriolis and gravity vector which is C(q, dq)*dq + g(q).
  Eigen::VectorXd mCg;

  /// \brief Dirty flag for the combined vector of Coriolis and gravity.
  bool mIsCombinedVectorDirty;

  /// \brief External force vector for the skeleton.
  Eigen::VectorXd mFext;

  /// \brief Dirty flag for the external force vector.
  bool mIsExternalForceVectorDirty;

  /// \brief Constraint force vector.
  Eigen::VectorXd mFc;

  /// \brief Damping force vector.
  Eigen::VectorXd mFd;

  /// \brief Dirty flag for the damping force vector.
  bool mIsDampingForceVectorDirty;

  // TODO(JS): Better naming
  /// \brief Flag for status of impulse testing.
  bool mIsImpulseApplied;

  //----------------------------------------------------------------------------
  // Union finding
  //----------------------------------------------------------------------------
public:

  /// \brief
  void resetUnion()
  {
    mUnionRootSkeleton = this;
    mUnionSize = 1;
  }

  /// \brief
  Skeleton* mUnionRootSkeleton;

  /// \brief
  size_t mUnionSize;

  /// \brief
  size_t mUnionIndex;

protected:
  /// \brief Update mass matrix of the skeleton.
  virtual void updateMassMatrix();

  /// \brief Update augmented mass matrix of the skeleton.
  virtual void updateAugMassMatrix();

  /// \brief Update inverse of mass matrix of the skeleton.
  virtual void updateInvMassMatrix();

  /// \brief Update inverse of augmented mass matrix of the skeleton.
  virtual void updateInvAugMassMatrix();

  /// \brief Update Coriolis force vector of the skeleton.
  virtual void updateCoriolisForceVector();

  /// \brief Update gravity force vector of the skeleton.
  virtual void updateGravityForceVector();

  /// \brief Update combined vector of the skeletong.
  virtual void updateCombinedVector();

  /// \brief update external force vector to generalized torques.
  virtual void updateExternalForceVector();

  /// \brief Update damping force vector.
  virtual void updateDampingForceVector();

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SKELETON_H_
