/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
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

#include "dart/common/Deprecated.h"
#include "dart/common/NameManager.h"
#include "dart/dynamics/MetaSkeleton.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

/// class Skeleton
class Skeleton : public MetaSkeleton
{
public:

  struct Properties
  {
    /// Name
    std::string mName;

    /// If the skeleton is not mobile, its dynamic effect is equivalent
    /// to having infinite mass. If the configuration of an immobile skeleton
    /// are manually changed, the collision results might not be correct.
    bool mIsMobile;

    /// Gravity vector.
    Eigen::Vector3d mGravity;

    /// Time step for implicit joint damping force.
    double mTimeStep;

    /// True if self collision check is enabled. Use mEnabledAdjacentBodyCheck
    /// to disable collision checks between adjacent bodies.
    bool mEnabledSelfCollisionCheck;

    /// True if self collision check is enabled, including adjacent bodies.
    /// Note: If mEnabledSelfCollisionCheck is false, then this value will be
    /// ignored.
    bool mEnabledAdjacentBodyCheck;

    Properties(
        const std::string& _name = "Skeleton",
        bool _isMobile = true,
        const Eigen::Vector3d& _gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
        double _timeStep = 0.001,
        bool _enabledSelfCollisionCheck = false,
        bool _enableAdjacentBodyCheck = false);
  };

  //----------------------------------------------------------------------------
  /// \{ \name Constructor and Destructor
  //----------------------------------------------------------------------------

  /// Create a new Skeleton inside of a shared_ptr
  static SkeletonPtr create(const std::string& _name="Skeleton");

  /// Create a new Skeleton inside of a shared_ptr
  static SkeletonPtr create(const Properties& _properties);

  /// Get the shared_ptr that manages this Skeleton
  SkeletonPtr getPtr();

  /// Get the shared_ptr that manages this Skeleton
  ConstSkeletonPtr getPtr() const;

  /// Destructor
  virtual ~Skeleton();

  /// Remove copy operator
  Skeleton& operator=(const Skeleton& _other) = delete;

  /// Create an identical clone of this Skeleton.
  ///
  /// Note: the state of the Skeleton will NOT be cloned, only the structure and
  /// properties will be [TODO(MXG): copy the state as well]
  SkeletonPtr clone() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Properties
  //----------------------------------------------------------------------------

  /// Set the Properties of this Skeleton
  void setProperties(const Properties& _properties);

  /// Get the Properties of this Skeleton
  const Properties& getSkeletonProperties() const;

  /// Set name.
  const std::string& setName(const std::string& _name);

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

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  /// Create a Joint and child BodyNode pair of the given types. When creating
  /// a root (parentless) BodyNode, pass in nullptr for the _parent argument.
  template <class JointType, class NodeType = BodyNode>
  std::pair<JointType*, NodeType*> createJointAndBodyNodePair(
    BodyNode* _parent = nullptr,
    const typename JointType::Properties& _jointProperties =
                                              typename JointType::Properties(),
    const typename NodeType::Properties& _bodyProperties =
                                              typename NodeType::Properties());

  /// Add a body node
  DEPRECATED(4.5)
  void addBodyNode(BodyNode* _body);

  // Documentation inherited
  size_t getNumBodyNodes() const override;

  /// Get number of rigid body nodes.
  size_t getNumRigidBodyNodes() const;

  /// Get number of soft body nodes.
  size_t getNumSoftBodyNodes() const;

  /// Get the number of independent trees that this Skeleton contains
  size_t getNumTrees() const;

  /// Get the root BodyNode of the tree whose index in this Skeleton is _treeIdx
  BodyNode* getRootBodyNode(size_t _treeIdx = 0);

  /// Get the const root BodyNode of the tree whose index in this Skeleton is
  /// _treeIdx
  const BodyNode* getRootBodyNode(size_t _treeIdx = 0) const;

  // Documentation inherited
  BodyNode* getBodyNode(size_t _idx) override;

  // Documentation inherited
  const BodyNode* getBodyNode(size_t _idx) const override;

  /// Get SoftBodyNode whose index is _idx
  SoftBodyNode* getSoftBodyNode(size_t _idx);

  /// Get const SoftBodyNode whose index is _idx
  const SoftBodyNode* getSoftBodyNode(size_t _idx) const;

  /// Get body node whose name is _name
  BodyNode* getBodyNode(const std::string& _name);

  /// Get const body node whose name is _name
  const BodyNode* getBodyNode(const std::string& _name) const;

  /// Get soft body node whose name is _name
  SoftBodyNode* getSoftBodyNode(const std::string& _name);

  /// Get const soft body node whose name is _name
  const SoftBodyNode* getSoftBodyNode(const std::string& _name) const;

  // Documentation inherited
  const std::vector<BodyNode*>& getBodyNodes() override;

  // Documentation inherited
  const std::vector<const BodyNode*>& getBodyNodes() const override;

  // Documentation inherited
  size_t getIndexOf(const BodyNode* _bn, bool _warning=true) const override;

  /// Get the BodyNodes belonging to a tree in this Skeleton
  const std::vector<BodyNode*>& getTreeBodyNodes(size_t _treeIdx);

  /// Get the BodyNodes belonging to a tree in this Skeleton
  std::vector<const BodyNode*> getTreeBodyNodes(size_t _treeIdx) const;

  // Documentation inherited
  size_t getNumJoints() const override;

  // Documentation inherited
  Joint* getJoint(size_t _idx) override;

  // Documentation inherited
  const Joint* getJoint(size_t _idx) const override;

  /// Get Joint whose name is _name
  Joint* getJoint(const std::string& _name);

  /// Get const Joint whose name is _name
  const Joint* getJoint(const std::string& _name) const;

  // Documentation inherited
  size_t getIndexOf(const Joint* _joint, bool _warning=true) const override;

  // Documentation inherited
  size_t getNumDofs() const override;

  // Documentation inherited
  DegreeOfFreedom* getDof(size_t _idx) override;

  // Documentation inherited
  const DegreeOfFreedom* getDof(size_t _idx) const override;

  /// Get degree of freedom (aka generalized coordinate) whose name is _name
  DegreeOfFreedom* getDof(const std::string& _name);

  /// Get degree of freedom (aka generalized coordinate) whose name is _name
  const DegreeOfFreedom* getDof(const std::string& _name) const;

  // Documentation inherited
  const std::vector<DegreeOfFreedom*>& getDofs() override;

  // Documentation inherited
  std::vector<const DegreeOfFreedom*> getDofs() const override;

  // Documentation inherited
  size_t getIndexOf(const DegreeOfFreedom* _dof,
                    bool _warning=true) const override;

  /// Get the DegreesOfFreedom belonging to a tree in this Skeleton
  const std::vector<DegreeOfFreedom*>& getTreeDofs(size_t _treeIdx);

  /// Get the DegreesOfFreedom belonging to a tree in this Skeleton
  const std::vector<const DegreeOfFreedom*>& getTreeDofs(size_t _treeIdx) const;

  /// Get marker whose name is _name
  Marker* getMarker(const std::string& _name);

  /// Get const marker whose name is _name
  const Marker* getMarker(const std::string& _name) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Deprecated
  //----------------------------------------------------------------------------

  /// Initialize this skeleton for kinematics and dynamics
  DEPRECATED(4.5)
  void init(double _timeStep = 0.001,
            const Eigen::Vector3d& _gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

  /// Set the configuration of this skeleton described in generalized
  /// coordinates. The order of input configuration is determined by _id.
  ///
  /// DEPRECATED: Use setPositionSegment(const std::vector<size_t>&,
  /// const Eigen::VectorXd&) instead
  DEPRECATED(4.5)
  void setPositionSegment(const std::vector<size_t>& _indices,
                          const Eigen::VectorXd& _positions);

  /// Get the configuration of this skeleton described in generalized
  /// coordinates. The returned order of configuration is determined by _id.
  ///
  /// DEPRECATED: Use getPositions(const std::vector<size_t>&) instead
  DEPRECATED(4.5)
  Eigen::VectorXd getPositionSegment(const std::vector<size_t>& _indices) const;

  /// Set the generalized velocities of a segment of this Skeleton. The order of
  /// input is determined by _id
  ///
  /// DEPRECATED: Use setVelocities(const std::vector<size_t>&,
  /// const Eigen::VectorXd&) instead
  DEPRECATED(4.5)
  void setVelocitySegment(const std::vector<size_t>& _indices,
                          const Eigen::VectorXd& _velocities);

  /// Get the generalized velocities of a segment of this Skeleton. The returned
  /// order of the velocities is determined by _id.
  ///
  /// DEPRECATED: use getVelocities(const std::vector<size_t>&) instead
  DEPRECATED(4.5)
  Eigen::VectorXd getVelocitySegment(const std::vector<size_t>& _id) const;

  /// Set the generalized accelerations of a segment of this Skeleton. The order
  /// of input is determined by _id
  ///
  /// DEPRECATED: Use setAccelerations(const std::vector<size_t>&,
  /// const Eigen::VectorXd&) instead
  DEPRECATED(4.5)
  void setAccelerationSegment(const std::vector<size_t>& _id,
                              const Eigen::VectorXd& _accelerations);

  /// Get the generalized accelerations of a segment of this Skeleton. The
  /// returned order of the accelerations is determined by _id
  ///
  /// DEPRECATED: Use getAccelerations(const std::vector<size_t>&) instead
  DEPRECATED(4.5)
  Eigen::VectorXd getAccelerationSegment(const std::vector<size_t>& _indices) const;

  /// \}

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  ///
  DEPRECATED(4.2)
  void setConstraintImpulses(const Eigen::VectorXd& _impulses);

  ///
  DEPRECATED(4.2)
  Eigen::VectorXd getConstraintImpulses() const;

  //----------------------------------------------------------------------------
  // Integration and finite difference
  //----------------------------------------------------------------------------

  // Documentation inherited
  void integratePositions(double _dt);

  // Documentation inherited
  void integrateVelocities(double _dt);

  /// Return the difference of two generalized positions which are measured in
  /// the configuration space of this Skeleton. If the configuration space is
  /// Euclidean space, this function returns _q2 - _q1. Otherwise, it depends on
  /// the type of the configuration space.
  Eigen::VectorXd getPositionDifferences(
      const Eigen::VectorXd& _q2, const Eigen::VectorXd& _q1) const;

  /// Return the difference of two generalized velocities or accelerations which
  /// are measured in the tangent space at the identity. Since the tangent
  /// spaces are vector spaces, this function always returns _dq2 - _dq1.
  Eigen::VectorXd getVelocityDifferences(
      const Eigen::VectorXd& _dq2, const Eigen::VectorXd& _dq1) const;

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

  //----------------------------------------------------------------------------
  // Impulse-based dynamics algorithms
  //----------------------------------------------------------------------------

  /// Clear constraint impulses: (a) spatial constraints on BodyNode and
  /// (b) generalized constraints on Joint
  void clearConstraintImpulses();

  /// Set constraint force vector.
  DEPRECATED(4.2)
  void setConstraintForceVector(const Eigen::VectorXd& _Fc);

  /// Update bias impulses
  void updateBiasImpulse(BodyNode* _bodyNode);

  /// \brief Update bias impulses due to impulse [_imp] on body node [_bodyNode]
  /// \param _bodyNode Body node contraint impulse, _imp, is applied
  /// \param _imp Constraint impulse expressed in body frame of _bodyNode
  void updateBiasImpulse(BodyNode* _bodyNode, const Eigen::Vector6d& _imp);

  /// \brief Update bias impulses due to impulse [_imp] on body node [_bodyNode]
  /// \param _bodyNode Body node contraint impulse, _imp1, is applied
  /// \param _imp Constraint impulse expressed in body frame of _bodyNode1
  /// \param _bodyNode Body node contraint impulse, _imp2, is applied
  /// \param _imp Constraint impulse expressed in body frame of _bodyNode2
  void updateBiasImpulse(BodyNode* _bodyNode1, const Eigen::Vector6d& _imp1,
                         BodyNode* _bodyNode2, const Eigen::Vector6d& _imp2);

  /// \brief Update bias impulses due to impulse[_imp] on body node [_bodyNode]
  void updateBiasImpulse(SoftBodyNode* _softBodyNode,
                         PointMass* _pointMass,
                         const Eigen::Vector3d& _imp);

  /// \brief Update velocity changes in body nodes and joints due to applied
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

  //----------------------------------------------------------------------------
  /// \{ \name Jacobians
  //----------------------------------------------------------------------------

  // Documentation inherited
  math::Jacobian getJacobian(const BodyNode* _bodyNode) const override;

  // Documentation inherited
  math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const override;

  // Documentation inherited
  math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getWorldJacobian(const BodyNode* _bodyNode) const override;

  // Documentation inherited
  math::Jacobian getWorldJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const override;

  // Documentation inherited
  math::LinearJacobian getLinearJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::LinearJacobian getLinearJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::AngularJacobian getAngularJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::Jacobian getJacobianSpatialDeriv(const BodyNode* _bodyNode) const override;

  // Documentation inherited
  math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const override;

  // Documentation inherited
  math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getJacobianClassicDeriv(const BodyNode* _bodyNode) const override;

  // Documentation inherited
  math::Jacobian getJacobianClassicDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getJacobianClassicDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::LinearJacobian getLinearJacobianDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::LinearJacobian getLinearJacobianDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset = Eigen::Vector3d::Zero(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::AngularJacobian getAngularJacobianDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Equations of Motion
  //----------------------------------------------------------------------------

  /// Get total mass of the skeleton. The total mass is calculated as BodyNodes
  /// are added and is updated as BodyNode mass is changed, so this is a
  /// constant-time O(1) operation for the Skeleton class.
  double getMass() const override;

  /// Get the mass matrix of a specific tree in the Skeleton
  const Eigen::MatrixXd& getMassMatrix(size_t _treeIdx) const;

  // Documentation inherited
  const Eigen::MatrixXd& getMassMatrix() const override;

  /// Get the augmented mass matrix of a specific tree in the Skeleton
  const Eigen::MatrixXd& getAugMassMatrix(size_t _treeIdx) const;

  // Documentation inherited
  const Eigen::MatrixXd& getAugMassMatrix() const override;

  /// Get the inverse mass matrix of a specific tree in the Skeleton
  const Eigen::MatrixXd& getInvMassMatrix(size_t _treeIdx) const;

  // Documentation inherited
  const Eigen::MatrixXd& getInvMassMatrix() const override;

  /// Get the inverse augmented mass matrix of a tree
  const Eigen::MatrixXd& getInvAugMassMatrix(size_t _treeIdx) const;

  // Documentation inherited
  const Eigen::MatrixXd& getInvAugMassMatrix() const override;

  /// Get Coriolis force vector of the skeleton.
  /// \remarks Please use getCoriolisForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getCoriolisForceVector() const;

  /// Get the Coriolis force vector of a tree in this Skeleton
  const Eigen::VectorXd& getCoriolisForces(size_t _treeIdx) const;

  // Documentation inherited
  const Eigen::VectorXd& getCoriolisForces() const override;

  /// Get gravity force vector of the skeleton.
  /// \remarks Please use getGravityForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getGravityForceVector() const;

  /// Get the gravity forces for a tree in this Skeleton
  const Eigen::VectorXd& getGravityForces(size_t _treeIdx) const;

  // Documentation inherited
  const Eigen::VectorXd& getGravityForces() const override;

  /// Get combined vector of Coriolis force and gravity force of the skeleton.
  /// \remarks Please use getCoriolisAndGravityForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getCombinedVector() const;

  /// Get the combined vector of Coriolis force and gravity force of a tree
  const Eigen::VectorXd& getCoriolisAndGravityForces(size_t _treeIdx) const;

  // Documentation inherited
  const Eigen::VectorXd& getCoriolisAndGravityForces() const override;

  /// Get external force vector of the skeleton.
  /// \remarks Please use getExternalForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getExternalForceVector() const;

  /// Get the external force vector of a tree in the Skeleton
  const Eigen::VectorXd& getExternalForces(size_t _treeIdx) const;

  // Documentation inherited
  const Eigen::VectorXd& getExternalForces() const override;

  /// Get damping force of the skeleton.
//  const Eigen::VectorXd& getDampingForceVector();

  /// Get constraint force vector.
  /// \remarks Please use getConstraintForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getConstraintForceVector();

  /// Get constraint force vector for a tree
  const Eigen::VectorXd& getConstraintForces(size_t _treeIdx) const;

  /// Get constraint force vector
  const Eigen::VectorXd& getConstraintForces() const override;

  // Documentation inherited
  void clearExternalForces() override;

  // Documentation inherited
  void clearInternalForces() override;

  // Documentation inherited
  double getKineticEnergy() const override;

  // Documentation inherited
  double getPotentialEnergy() const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Center of Mass Jacobian
  //----------------------------------------------------------------------------

  /// Get the Skeleton's COM with respect to any Frame (default is World Frame)
  Eigen::Vector3d getCOM(
      const Frame* _withRespectTo = Frame::World()) const override;

  /// Get the Skeleton's COM spatial velocity in terms of any Frame (default is
  /// World Frame)
  Eigen::Vector6d getCOMSpatialVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// Get the Skeleton's COM linear velocity in terms of any Frame (default is
  /// World Frame)
  Eigen::Vector3d getCOMLinearVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// Get the Skeleton's COM spatial acceleration in terms of any Frame (default
  /// is World Frame)
  Eigen::Vector6d getCOMSpatialAcceleration(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// Get the Skeleton's COM linear acceleration in terms of any Frame (default
  /// is World Frame)
  Eigen::Vector3d getCOMLinearAcceleration(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// Get the Skeleton's COM Jacobian in terms of any Frame (default is World
  /// Frame)
  math::Jacobian getCOMJacobian(
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// Get the Skeleton's COM Linear Jacobian in terms of any Frame (default is
  /// World Frame)
  math::LinearJacobian getCOMLinearJacobian(
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// Get the Skeleton's COM Jacobian spatial time derivative in terms of any
  /// Frame (default is World Frame).
  ///
  /// NOTE: Since this is a spatial time derivative, it is only meant to be used
  /// with spatial acceleration vectors. If you are using classical linear
  /// vectors, then use getCOMLinearJacobianDeriv() instead.
  math::Jacobian getCOMJacobianSpatialDeriv(
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// Get the Skeleton's COM Linear Jacobian time derivative in terms of any
  /// Frame (default is World Frame).
  ///
  /// NOTE: Since this is a classical time derivative, it is only meant to be
  /// used with classical acceleration vectors. If you are using spatial
  /// vectors, then use getCOMJacobianSpatialDeriv() instead.
  math::LinearJacobian getCOMLinearJacobianDeriv(
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// Get skeleton's COM w.r.t. world frame.
  ///
  /// Deprecated in 4.4. Please use getCOM() instead
  DEPRECATED(4.4)
  Eigen::Vector3d getWorldCOM();

  /// Get skeleton's COM velocity w.r.t. world frame.
  ///
  /// Deprecated in 4.4. Please use getCOMLinearVelocity() instead
  DEPRECATED(4.4)
  Eigen::Vector3d getWorldCOMVelocity();

  /// Get skeleton's COM acceleration w.r.t. world frame.
  ///
  /// Deprecated in 4.4. Please use getCOMAcceleration() instead
  DEPRECATED(4.4)
  Eigen::Vector3d getWorldCOMAcceleration();

  /// Get skeleton's COM Jacobian w.r.t. world frame.
  DEPRECATED(4.4)
  Eigen::MatrixXd getWorldCOMJacobian();

  /// Get skeleton's COM Jacobian time derivative w.r.t. world frame.
  DEPRECATED(4.4)
  Eigen::MatrixXd getWorldCOMJacobianTimeDeriv();

  /// \}

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
  ///
  /// Deprecated as of 4.4. Auto-updating makes this function irrelevant
  DEPRECATED(4.4)
  void computeForwardDynamicsRecursionPartA();

  /// Compute recursion part B of forward dynamics
  void computeForwardDynamicsRecursionPartB();

  /// Compute recursion part A of inverse dynamics
  ///
  /// Deprecated as of 4.4. Auto-updating makes this function irrelevant
  DEPRECATED(4.4)
  void computeInverseDynamicsRecursionA();

  /// Compute recursion part B of inverse dynamics
  void computeInverseDynamicsRecursionB(bool _withExternalForces = false,
                                        bool _withDampingForces = false,
                                        bool _withSpringForces = false);

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------
  friend class BodyNode;
  friend class SoftBodyNode;
  friend class Joint;
  friend class SingleDofJoint;
  template<size_t> friend class MultiDofJoint;
  friend class DegreeOfFreedom;

protected:
  class DataCache;

  /// Constructor called by create()
  Skeleton(const Properties& _properties);

  /// Setup this Skeleton with its shared_ptr
  void setPtr(const SkeletonPtr& _ptr);

  /// Register a BodyNode with the Skeleton. Internal use only.
  void registerBodyNode(BodyNode* _newBodyNode);

  /// Register a Joint with the Skeleton. Internal use only.
  void registerJoint(Joint* _newJoint);

  /// Remove a BodyNode from the Skeleton. Internal use only.
  void unregisterBodyNode(BodyNode* _oldBodyNode);

  /// Remove a Joint from the Skeleton. Internal use only.
  void unregisterJoint(Joint* _oldJoint);

  /// Move a subtree of BodyNodes from this Skeleton to another Skeleton
  bool moveBodyNodeTree(Joint* _parentJoint, BodyNode* _bodyNode,
                        SkeletonPtr _newSkeleton,
                        BodyNode* _parentNode);

  /// Move a subtree of BodyNodes from this Skeleton to another Skeleton while
  /// changing the Joint type of the top parent Joint.
  ///
  /// Returns a nullptr if the move failed for any reason.
  template <class JointType>
  JointType* moveBodyNodeTree(
      BodyNode* _bodyNode,
      const SkeletonPtr& _newSkeleton,
      BodyNode* _parentNode,
      const typename JointType::Properties& _joint);

  /// Copy a subtree of BodyNodes onto another Skeleton while leaving the
  /// originals intact
  std::pair<Joint*, BodyNode*> cloneBodyNodeTree(
      Joint* _parentJoint,
      const BodyNode* _bodyNode,
      const SkeletonPtr& _newSkeleton,
      BodyNode* _parentNode,
      bool _recursive) const;

  /// Copy a subtree of BodyNodes onto another Skeleton while leaving the
  /// originals intact, but alter the top parent Joint to a new type
  template <class JointType>
  std::pair<JointType*, BodyNode*> cloneBodyNodeTree(
      const BodyNode* _bodyNode,
      const SkeletonPtr& _newSkeleton,
      BodyNode* _parentNode,
      const typename JointType::Properties& _joint,
      bool _recursive) const;

  /// Create a vector representation of a subtree of BodyNodes
  std::vector<const BodyNode*> constructBodyNodeTree(
      const BodyNode* _bodyNode) const;

  std::vector<BodyNode*> constructBodyNodeTree(BodyNode* _bodyNode);

  /// Create a vector representation of a subtree of BodyNodes and remove that
  /// subtree from this Skeleton without deleting them
  std::vector<BodyNode*> extractBodyNodeTree(BodyNode* _bodyNode);

  /// Take in and register a subtree of BodyNodes
  void receiveBodyNodeTree(const std::vector<BodyNode*>& _tree);

  /// Notify that the articulated inertia and everything that depends on it
  /// needs to be updated
  void notifyArticulatedInertiaUpdate(size_t _treeIdx);

  /// Update the computation for total mass
  void updateTotalMass();

  /// Update the dimensions for a specific cache
  void updateCacheDimensions(DataCache& _cache);

  /// Update the dimensions for a tree's cache
  void updateCacheDimensions(size_t _treeIdx);

  /// Update the articulated inertia of a tree
  void updateArticulatedInertia(size_t _tree) const;

  /// Update the articulated inertias of the skeleton
  void updateArticulatedInertia() const;

  /// Update the mass matrix of a tree
  void updateMassMatrix(size_t _treeIdx) const;

  /// Update mass matrix of the skeleton.
  void updateMassMatrix() const;

  void updateAugMassMatrix(size_t _treeIdx) const;

  /// Update augmented mass matrix of the skeleton.
  void updateAugMassMatrix() const;

  /// Update the inverse mass matrix of a tree
  void updateInvMassMatrix(size_t _treeIdx) const;

  /// Update inverse of mass matrix of the skeleton.
  void updateInvMassMatrix() const;

  /// Update the inverse augmented mass matrix of a tree
  void updateInvAugMassMatrix(size_t _treeIdx) const;

  /// Update inverse of augmented mass matrix of the skeleton.
  void updateInvAugMassMatrix() const;

  /// Update Coriolis force vector of the skeleton.
  /// \remarks Please use updateCoriolisForces() instead.
  DEPRECATED(4.2)
  virtual void updateCoriolisForceVector();

  /// Update Coriolis force vector for a tree in the Skeleton
  void updateCoriolisForces(size_t _treeIdx) const;

  /// Update Coriolis force vector of the skeleton.
  void updateCoriolisForces() const;

  /// Update gravity force vector of the skeleton.
  /// \remarks Please use updateGravityForces() instead.
  DEPRECATED(4.2)
  virtual void updateGravityForceVector();

  /// Update the gravity force vector of a tree
  void updateGravityForces(size_t _treeIdx) const;

  /// Update gravity force vector of the skeleton.
  void updateGravityForces() const;

  /// Update combined vector of the skeleton.
  /// \remarks Please use updateCoriolisAndGravityForces() instead.
  DEPRECATED(4.2)
  virtual void updateCombinedVector();

  /// Update the combined vector for a tree in this Skeleton
  void updateCoriolisAndGravityForces(size_t _treeIdx) const;

  /// Update combined vector of the skeleton.
  void updateCoriolisAndGravityForces() const;

  /// update external force vector to generalized forces.
  /// \remarks Please use updateExternalForces() instead.
  DEPRECATED(4.2)
  virtual void updateExternalForceVector();

  /// Update external force vector to generalized forces for a tree
  void updateExternalForces(size_t _treeIdx) const;

  // TODO(JS): Not implemented yet
  /// update external force vector to generalized forces.
  void updateExternalForces() const;

  /// Compute the constraint force vector for a tree
  const Eigen::VectorXd& computeConstraintForces(DataCache& cache) const;

//  /// Update damping force vector.
//  virtual void updateDampingForceVector();

  /// Add a BodyNode to the BodyNode NameManager
  const std::string& addEntryToBodyNodeNameMgr(BodyNode* _newNode);

  /// Add a Joint to to the Joint NameManager
  const std::string& addEntryToJointNameMgr(Joint* _newJoint);

  /// Add a SoftBodyNode to the SoftBodyNode NameManager
  void addEntryToSoftBodyNodeNameMgr(SoftBodyNode* _newNode);

  /// Add entries for all the Markers belonging to BodyNode _node
  void addMarkersOfBodyNode(BodyNode* _node);

  /// Remove entries for all the Markers belonging to BodyNode _node
  void removeMarkersOfBodyNode(BodyNode* _node);

  /// Add a Marker entry
  const std::string& addEntryToMarkerNameMgr(Marker* _newMarker);

protected:

  /// Properties of this Skeleton
  Properties mSkeletonP;

  /// The resource-managing pointer to this Skeleton
  std::weak_ptr<Skeleton> mPtr;

  /// List of Soft body node list in the skeleton
  std::vector<SoftBodyNode*> mSoftBodyNodes;

  /// NameManager for tracking BodyNodes
  dart::common::NameManager<BodyNode*> mNameMgrForBodyNodes;

  /// NameManager for tracking Joints
  dart::common::NameManager<Joint*> mNameMgrForJoints;

  /// NameManager for tracking DegreesOfFreedom
  dart::common::NameManager<DegreeOfFreedom*> mNameMgrForDofs;

  /// NameManager for tracking SoftBodyNodes
  dart::common::NameManager<SoftBodyNode*> mNameMgrForSoftBodyNodes;

  /// NameManager for tracking Markers
  dart::common::NameManager<Marker*> mNameMgrForMarkers;

  struct DirtyFlags
  {
    /// Default constructor
    DirtyFlags();

    /// Dirty flag for articulated body inertia
    bool mArticulatedInertia;

    /// Dirty flag for the mass matrix.
    bool mMassMatrix;

    /// Dirty flag for the mass matrix.
    bool mAugMassMatrix;

    /// Dirty flag for the inverse of mass matrix.
    bool mInvMassMatrix;

    /// Dirty flag for the inverse of augmented mass matrix.
    bool mInvAugMassMatrix;

    /// Dirty flag for the gravity force vector.
    bool mGravityForces;

    /// Dirty flag for the Coriolis force vector.
    bool mCoriolisForces;

    /// Dirty flag for the combined vector of Coriolis and gravity.
    bool mCoriolisAndGravityForces;

    /// Dirty flag for the external force vector.
    bool mExternalForces;

    /// Dirty flag for the damping force vector.
    bool mDampingForces;
  };

  struct DataCache
  {
    DirtyFlags mDirty;

    /// BodyNodes belonging to this tree
    std::vector<BodyNode*> mBodyNodes;

    /// Cache for const BodyNodes, for the sake of the API
    std::vector<const BodyNode*> mConstBodyNodes;

    /// Degrees of Freedom belonging to this tree
    std::vector<DegreeOfFreedom*> mDofs;

    /// Cache for const Degrees of Freedom, for the sake of the API
    std::vector<const DegreeOfFreedom*> mConstDofs;

    /// Mass matrix cache
    Eigen::MatrixXd mM;

    /// Mass matrix for the skeleton.
    Eigen::MatrixXd mAugM;

    /// Inverse of mass matrix for the skeleton.
    Eigen::MatrixXd mInvM;

    /// Inverse of augmented mass matrix for the skeleton.
    Eigen::MatrixXd mInvAugM;

    /// Coriolis vector for the skeleton which is C(q,dq)*dq.
    Eigen::VectorXd mCvec;

    /// Gravity vector for the skeleton; computed in nonrecursive
    /// dynamics only.
    Eigen::VectorXd mG;

    /// Combined coriolis and gravity vector which is C(q, dq)*dq + g(q).
    Eigen::VectorXd mCg;

    /// External force vector for the skeleton.
    Eigen::VectorXd mFext;

    /// Constraint force vector.
    Eigen::VectorXd mFc;
  };

  mutable std::vector<DataCache> mTreeCache;

  mutable DataCache mSkelCache;

  /// Total mass.
  double mTotalMass;

  // TODO(JS): Better naming
  /// Flag for status of impulse testing.
  bool mIsImpulseApplied;

public:
  //--------------------------------------------------------------------------
  // Union finding
  //--------------------------------------------------------------------------
  ///
  void resetUnion()
  {
    mUnionRootSkeleton = mPtr;
    mUnionSize = 1;
  }

  ///
  std::weak_ptr<Skeleton> mUnionRootSkeleton;

  ///
  size_t mUnionSize;

  ///
  size_t mUnionIndex;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "dart/dynamics/detail/Skeleton.h"

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SKELETON_H_
