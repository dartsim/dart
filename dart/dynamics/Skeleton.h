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

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "dart/common/Deprecated.h"
#include "dart/math/Geometry.h"
#include "dart/common/NameManager.h"
#include "dart/common/Subject.h"
#include "dart/dynamics/Frame.h"

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
class DegreeOfFreedom;

/// struct GenCoordInfo
struct GenCoordInfo
{
  Joint* joint;
  size_t localIndex;
} DEPRECATED(4.3);

/// class Skeleton
class Skeleton : public common::Subject
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

  /// Create a Joint and child BodyNode pair of the given types. When creating
  /// a root (parentless) BodyNode, pass in nullptr for the _parent argument.
  template <class JointType, class NodeType = BodyNode>
  std::pair<JointType*, NodeType*> createJointAndBodyNodePair(
    BodyNode* _parent = nullptr,
    const typename JointType::Properties& _jointProperties = JointType::Properties(),
    const typename NodeType::Properties& _bodyProperties = NodeType::Properties())
  {
    JointType* joint = new JointType(_jointProperties);
    NodeType* node = new NodeType(_parent, joint, _bodyProperties);
    registerBodyNode(node);

    return std::pair<JointType*, NodeType*>(joint, node);
  }

  /// Add a body node
  // TODO(MXG): Deprecate this
  void addBodyNode(BodyNode* _body);

  /// Get number of body nodes
  size_t getNumBodyNodes() const;

  /// Get number of rigid body nodes.
  size_t getNumRigidBodyNodes() const;

  /// Get number of soft body nodes.
  size_t getNumSoftBodyNodes() const;

  /// Get root BodyNode
  BodyNode* getRootBodyNode();

  /// Get const root BodyNode
  const BodyNode* getRootBodyNode() const;

  /// Get body node whose index is _idx
  BodyNode* getBodyNode(size_t _idx);

  /// Get const body node whose index is _idx
  const BodyNode* getBodyNode(size_t _idx) const;

  /// Get soft body node whose index is _idx
  SoftBodyNode* getSoftBodyNode(size_t _idx);

  /// Get const soft body node whose index is _idx
  const SoftBodyNode* getSoftBodyNode(size_t _idx) const;

  /// Get body node whose name is _name
  BodyNode* getBodyNode(const std::string& _name);

  /// Get const body node whose name is _name
  const BodyNode* getBodyNode(const std::string& _name) const;

  /// Get soft body node whose name is _name
  SoftBodyNode* getSoftBodyNode(const std::string& _name);

  /// Get const soft body node whose name is _name
  const SoftBodyNode* getSoftBodyNode(const std::string& _name) const;

  /// Get number of joints
  size_t getNumJoints() const;

  /// Get joint whose index is _idx
  Joint* getJoint(size_t _idx);

  /// Get const joint whose index is _idx
  const Joint* getJoint(size_t _idx) const;

  /// Get joint whose name is _name
  Joint* getJoint(const std::string& _name);

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  DegreeOfFreedom* getDof(size_t _idx);

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  const DegreeOfFreedom* getDof(size_t _idx) const;

  /// Get degree of freedom (aka generalized coordinate) whose name is _name
  DegreeOfFreedom* getDof(const std::string& _name);

  /// Get degree of freedom (aka generalized coordinate) whose name is _name
  const DegreeOfFreedom* getDof(const std::string& _name) const;

  /// Get const joint whose name is _name
  const Joint* getJoint(const std::string& _name) const;

  /// Get marker whose name is _name
  Marker* getMarker(const std::string& _name);

  /// Get const marker whose name is _name
  const Marker* getMarker(const std::string& _name) const;

  //----------------------------------------------------------------------------
  // Initialization
  //----------------------------------------------------------------------------
  /// Initialize this skeleton for kinematics and dynamics
  // TODO(MXG): Deprecate this
  void init(double _timeStep = 0.001,
            const Eigen::Vector3d& _gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

  //----------------------------------------------------------------------------
  // Generalized coordinate system
  //----------------------------------------------------------------------------

  /// Return degrees of freedom of this skeleton
  DEPRECATED(4.1)
  size_t getDof() const;

  /// Return degrees of freedom of this skeleton
  size_t getNumDofs() const;

  /// \brief Return _index-th GenCoordInfo
  /// \warning GenCoordInfo is deprecated so this function is not necessary
  /// anymore. Please use DegreeOfFreedom by calling getDof(). We will keep this
  /// function until the next major version up only for backward compatibility
  /// in minor version ups.
  DEPRECATED(4.3)
  GenCoordInfo getGenCoordInfo(size_t _index) const;

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  /// Set a single command
  virtual void setCommand(size_t _index, double _command);

  /// Set a sinlge command
  virtual double getCommand(size_t _index) const;

  /// Set commands
  virtual void setCommands(const Eigen::VectorXd& _commands);

  /// Get commands
  virtual Eigen::VectorXd getCommands() const;

  /// Set zero all the positions
  virtual void resetCommands();

  /// \}

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  /// Set a single position
  void setPosition(size_t _index, double _position);

  /// Get a single position
  double getPosition(size_t _index) const;

  /// Set generalized positions
  void setPositions(const Eigen::VectorXd& _positions);

  /// Get generalized positions
  Eigen::VectorXd getPositions() const;

  /// Set the configuration of this skeleton described in generalized
  /// coordinates. The order of input configuration is determined by _id.
  void setPositionSegment(const std::vector<size_t>& _id,
                          const Eigen::VectorXd& _positions);

  /// Get the configuration of this skeleton described in generalized
  /// coordinates. The returned order of configuration is determined by _id.
  Eigen::VectorXd getPositionSegment(const std::vector<size_t>& _id) const;

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
  void setVelocities(const Eigen::VectorXd& _velocities);

  /// Set the generalized velocities of a segment of this Skeleton. The order of
  /// input is determined by _id
  void setVelocitySegment(const std::vector<size_t>& _id,
                          const Eigen::VectorXd& _velocities);

  /// Get generalized velocities
  Eigen::VectorXd getVelocities() const;

  /// Get the generalized velocities of a segment of this Skeleton. The returned
  /// order of the velocities is determined by _id.
  Eigen::VectorXd getVelocitySegment(const std::vector<size_t>& _id) const;

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

  /// Set the generalized accelerations of a segment of this Skeleton. The order
  /// of input is determined by _id
  void setAccelerationSegment(const std::vector<size_t>& _id,
                              const Eigen::VectorXd& _accelerations);

  /// Get accelerations
  Eigen::VectorXd getAccelerations() const;

  /// Get the generalized accelerations of a segment of this Skeleton. The
  /// returned order of the accelerations is determined by _id
  Eigen::VectorXd getAccelerationSegment(const std::vector<size_t>& _id) const;

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
  DEPRECATED(4.2)
  void setConstraintImpulses(const Eigen::VectorXd& _impulses);

  /// Set constraint impulses applying to joint
  void setJointConstraintImpulses(const Eigen::VectorXd& _impulses);

  ///
  DEPRECATED(4.2)
  Eigen::VectorXd getConstraintImpulses() const;

  /// Return constraint impulses applied to joint
  Eigen::VectorXd getJointConstraintImpulses() const;

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

  /// Compute impulse-based inverse dynamics
//  void computeImpulseInverseDynamics() {}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobians
  //----------------------------------------------------------------------------

  /// Get the spatial Jacobian targeting the origin of a BodyNode. The Jacobian
  /// is expressed in the Frame of the BodyNode.
  math::Jacobian getJacobian(const BodyNode* _bodyNode) const;

  /// Get the spatial Jacobian targeting the origin of a BodyNode. You can
  /// specify a coordinate Frame to express the Jabocian in.
  math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. The Jacobian is expressed
  /// in the Frame of the BodyNode.
  math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jabocian in.
  math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian targeting the origin of a BodyNode. The Jacobian
  /// is expressed in the World Frame.
  math::Jacobian getWorldJacobian(const BodyNode* _bodyNode) const;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. The Jacobian is expressed
  /// in the World Frame.
  math::Jacobian getWorldJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const;

  /// Get the linear Jacobian targeting the origin of a BodyNode. You can
  /// specify a coordinate Frame to express the Jabocian in.
  math::LinearJacobian getLinearJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the linear Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jabocian in.
  math::LinearJacobian getLinearJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the angular Jacobian of a BodyNode. You can specify a coordinate Frame
  /// to express the Jabocian in.
  math::AngularJacobian getAngularJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. The Jacobian is expressed in the Frame of the BodyNode.
  math::Jacobian getJacobianSpatialDeriv(const BodyNode* _bodyNode) const;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. You can specify a coordinate Frame to express the Jabocian in.
  math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// The Jacobian is expressed in the Frame of the BodyNode.
  math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. The Jacobian is expressed in the World Frame.
  math::Jacobian getJacobianClassicDeriv(const BodyNode* _bodyNode) const;

  /// Get the spatial Jacobian time derivative targeting the origin a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  math::Jacobian getJacobianClassicDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  math::Jacobian getJacobianClassicDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the linear Jacobian (classical) time derivative targeting the origin
  /// of a BodyNode. The _offset is expected in coordinates of the BodyNode
  /// Frame. You can specify a coordinate Frame to express the Jabocian in.
  math::LinearJacobian getLinearJacobianDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the linear Jacobian (classical) time derivative targeting an offset in
  /// a BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  math::LinearJacobian getLinearJacobianDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset = Eigen::Vector3d::Zero(),
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the angular Jacobian time derivative of a BodyNode. You can specify a
  /// coordinate Frame to express the Jabocian in.
  math::AngularJacobian getAngularJacobianDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// \}

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
  /// \remarks Please use getCoriolisForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getCoriolisForceVector();

  /// Get Coriolis force vector of the skeleton.
  const Eigen::VectorXd& getCoriolisForces();

  /// Get gravity force vector of the skeleton.
  /// \remarks Please use getGravityForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getGravityForceVector();

  /// Get gravity force vector of the skeleton.
  const Eigen::VectorXd& getGravityForces();

  /// Get combined vector of Coriolis force and gravity force of the skeleton.
  /// \remarks Please use getCoriolisAndGravityForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getCombinedVector();

  /// Get combined vector of Coriolis force and gravity force of the skeleton.
  const Eigen::VectorXd& getCoriolisAndGravityForces();

  /// Get external force vector of the skeleton.
  /// \remarks Please use getExternalForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getExternalForceVector();

  /// Get external force vector of the skeleton.
  const Eigen::VectorXd& getExternalForces();

  /// Get damping force of the skeleton.
//  const Eigen::VectorXd& getDampingForceVector();

  /// Get constraint force vector.
  /// \remarks Please use getConstraintForces() instead.
  DEPRECATED(4.2)
  const Eigen::VectorXd& getConstraintForceVector();

  /// Get constraint force vector.
  const Eigen::VectorXd& getConstraintForces();

  /// Set internal force vector.
//  void setInternalForceVector(const Eigen::VectorXd& _forces);

  /// Clear internal forces.
//  void clearInternalForces();

  /// Clear external forces, which are manually added to the body nodes
  /// by the user.
  void clearExternalForces();

  //----------------------------------------------------------------------------

  /// Get the Skeleton's COM with respect to any Frame (default is World Frame)
  Eigen::Vector3d getCOM(const Frame* _withRespectTo = Frame::World()) const;

  /// Get the Skeleton's COM spatial velocity in terms of any Frame (default is
  /// World Frame)
  Eigen::Vector6d getCOMSpatialVelocity(
                          const Frame* _relativeTo = Frame::World(),
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the Skeleton's COM linear velocity in terms of any Frame (default is
  /// World Frame)
  Eigen::Vector3d getCOMLinearVelocity(
                          const Frame* _relativeTo = Frame::World(),
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the Skeleton's COM spatial acceleration in terms of any Frame (default
  /// is World Frame)
  Eigen::Vector6d getCOMSpatialAcceleration(
                          const Frame* _relativeTo = Frame::World(),
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the Skeleton's COM linear acceleration in terms of any Frame (default
  /// is World Frame)
  Eigen::Vector3d getCOMLinearAcceleration(
                          const Frame* _relativeTo = Frame::World(),
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the Skeleton's COM Jacobian in terms of any Frame (default is World
  /// Frame)
  math::Jacobian getCOMJacobian(
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the Skeleton's COM Linear Jacobian in terms of any Frame (default is
  /// World Frame)
  math::LinearJacobian getCOMLinearJacobian(
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the Skeleton's COM Jacobian spatial time derivative in terms of any
  /// Frame (default is World Frame).
  ///
  /// NOTE: Since this is a spatial time derivative, it is only meant to be used
  /// with spatial acceleration vectors. If you are using classical linear
  /// vectors, then use getCOMLinearJacobianDeriv() instead.
  math::Jacobian getCOMJacobianSpatialDeriv(
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the Skeleton's COM Linear Jacobian time derivative in terms of any
  /// Frame (default is World Frame).
  ///
  /// NOTE: Since this is a classical time derivative, it is only meant to be
  /// used with classical acceleration vectors. If you are using spatial
  /// vectors, then use getCOMJacobianSpatialDeriv() instead.
  math::LinearJacobian getCOMLinearJacobianDeriv(
                          const Frame* _inCoordinatesOf = Frame::World()) const;

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

  /// Get kinetic energy of this skeleton.
  double getKineticEnergy() const;

  /// Get potential energy of this skeleton.
  double getPotentialEnergy() const;

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
  friend class DegreeOfFreedom;

protected:
  /// Register a BodyNode with the Skeleton. Internal use only.
  void registerBodyNode(BodyNode* _newBodyNode);

  /// Register a Joint with the Skeleton. Internal use only.
  void registerJoint(Joint* _newJoint);

  /// Remove a Joint from the Skeleton. Internal use only.
  void unregisterJoint(Joint* _oldJoint);

  /// Notify that the articulated inertia and everything that depends on it
  /// needs to be updated
  void notifyArticulatedInertiaUpdate();

  /// Update the computation for total mass
  void updateTotalMass();

  /// Update the articulated inertias of the skeleton
  void updateArticulatedInertia() const;

  /// Update mass matrix of the skeleton.
  void updateMassMatrix();

  /// Update augmented mass matrix of the skeleton.
  void updateAugMassMatrix();

  /// Update inverse of mass matrix of the skeleton.
  void updateInvMassMatrix();

  /// Update inverse of augmented mass matrix of the skeleton.
  void updateInvAugMassMatrix();

  /// Update Coriolis force vector of the skeleton.
  /// \remarks Please use updateCoriolisForces() instead.
  DEPRECATED(4.2)
  virtual void updateCoriolisForceVector();

  /// Update Coriolis force vector of the skeleton.
  void updateCoriolisForces();

  /// Update gravity force vector of the skeleton.
  /// \remarks Please use updateGravityForces() instead.
  DEPRECATED(4.2)
  virtual void updateGravityForceVector();

  /// Update gravity force vector of the skeleton.
  void updateGravityForces();

  /// Update combined vector of the skeleton.
  /// \remarks Please use updateCoriolisAndGravityForces() instead.
  DEPRECATED(4.2)
  virtual void updateCombinedVector();

  /// Update combined vector of the skeleton.
  void updateCoriolisAndGravityForces();

  /// update external force vector to generalized forces.
  /// \remarks Please use updateExternalForces() instead.
  DEPRECATED(4.2)
  virtual void updateExternalForceVector();

  // TODO(JS): Not implemented yet
  /// update external force vector to generalized forces.
  void updateExternalForces();

//  /// Update damping force vector.
//  virtual void updateDampingForceVector();

  /// Add a BodyNode to the BodyNode NameManager
  const std::string& addEntryToBodyNodeNameMgr(BodyNode* _newNode);

  /// Add a Joint to to the Joint NameManager
  const std::string& addEntryToJointNameMgr(Joint* _newJoint);

  /// Add a DegreeOfFreedom to the Dof NameManager
  const std::string& addEntryToDofNameMgr(DegreeOfFreedom* _newDof);

  /// Add a SoftBodyNode to the SoftBodyNode NameManager
  void addEntryToSoftBodyNodeNameMgr(SoftBodyNode* _newNode);

  /// Add entries for all the Markers belonging to BodyNode _node
  void addMarkersOfBodyNode(BodyNode* _node);

  /// Remove entries for all the Markers belonging to BodyNode _node
  void removeMarkersOfBodyNode(BodyNode* _node);

  /// Add a Marker entry
  const std::string& addEntryToMarkerNameMgr(Marker* _newMarker);

protected:
  /// Name
  std::string mName;

  /// Number of degrees of freedom (aka generalized coordinates)
  size_t mNumDofs;

  /// \brief Array of GenCoordInfo objects
  /// \warning GenCoordInfo is deprecated because the functionality is replaced
  /// by DegreeOfFreedom.
  DEPRECATED(4.3)
  std::vector<GenCoordInfo> mGenCoordInfos;

  /// Array of DegreeOfFreedom objects within all the joints in this Skeleton
  std::vector<DegreeOfFreedom*> mDofs;

  /// True if self collision check is enabled
  bool mEnabledSelfCollisionCheck;

  /// True if self collision check is enabled including adjacent bodies
  bool mEnabledAdjacentBodyCheck;

  /// List of body nodes in the skeleton.
  std::vector<BodyNode*> mBodyNodes;

  /// List of Soft body node list in the skeleton
  std::vector<SoftBodyNode*> mSoftBodyNodes;

  /// NameManager for tracking BodyNodes
  dart::common::NameManager<BodyNode> mNameMgrForBodyNodes;

  /// NameManager for tracking Joints
  dart::common::NameManager<Joint> mNameMgrForJoints;

  /// NameManager for tracking DegreesOfFreedom
  dart::common::NameManager<DegreeOfFreedom> mNameMgrForDofs;

  /// NameManager for tracking SoftBodyNodes
  dart::common::NameManager<SoftBodyNode> mNameMgrForSoftBodyNodes;

  /// NameManager for tracking Markers
  dart::common::NameManager<Marker> mNameMgrForMarkers;

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
  mutable bool mIsArticulatedInertiaDirty;

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
  bool mIsCoriolisForcesDirty;

  /// Gravity vector for the skeleton; computed in nonrecursive
  /// dynamics only.
  Eigen::VectorXd mG;

  /// Dirty flag for the gravity force vector.
  bool mIsGravityForcesDirty;

  /// Combined coriolis and gravity vector which is C(q, dq)*dq + g(q).
  Eigen::VectorXd mCg;

  /// Dirty flag for the combined vector of Coriolis and gravity.
  bool mIsCoriolisAndGravityForcesDirty;

  /// External force vector for the skeleton.
  Eigen::VectorXd mFext;

  /// Dirty flag for the external force vector.
  bool mIsExternalForcesDirty;

  /// Constraint force vector.
  Eigen::VectorXd mFc;

  /// Damping force vector.
  Eigen::VectorXd mFd;

  /// Dirty flag for the damping force vector.
  bool mIsDampingForcesDirty;

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
