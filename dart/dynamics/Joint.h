/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#ifndef DART_DYNAMICS_JOINT_H_
#define DART_DYNAMICS_JOINT_H_

#include <string>
#include <vector>

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
class Skeleton;
class DegreeOfFreedom;

/// class Joint
class Joint
{
public:
  /// Actuator type
  ///
  /// The command is taken by setCommand() or setCommands(), and the meaning of
  /// command is different depending on the actuator type. The default actuator
  /// type is FORCE. (TODO: FreeJoint should be PASSIVE?)
  ///
  /// FORCE/PASSIVE/SERVO joints are dynamic joints while
  /// ACCELERATION/VELOCITY/LOCKED joints are kinematic joints.
  ///
  /// Note the presence of joint damping force and joint spring force for all
  /// the actuator types if the coefficients are non-zero. The default
  /// coefficients are zero.
  ///
  /// \sa setActuatorType(), getActuatorType(),
  /// setSpringStiffness(), setDampingCoefficient(),
  enum ActuatorType
  {
    /// Command input is joint force, and the output is joint acceleration.
    ///
    /// If the command is zero, then it's identical to passive joint. The valid
    /// joint constraints are position limit, velocity limit, and Coulomb
    /// friction, and the invalid joint constraint is force limit.
    FORCE,

    /// Passive joint doesn't take any command input, and the output is joint
    /// acceleration.
    ///
    /// The valid joint constraints are position limit, velocity limit, and
    /// Coulomb friction, and the invalid joint constraint is force limit.
    PASSIVE,

    /// Command input is desired velocity, and the output is joint acceleration.
    ///
    /// The constraint solver will try to track the desired velocity within the
    /// joint force limit. All the joint constarints are valid.
    SERVO,
    // TODO: Not implemented yet.

    /// Command input is joint acceleration, and the output is joint force.
    ///
    /// The joint acceleration is always satisfied but it doesn't take the joint
    /// force limit into account. All the joint constraints are invalid.
    ACCELERATION,

    /// Command input is joint velocity, and the output is joint force.
    ///
    /// The joint velocity is always satisfied but it doesn't take the joint
    /// force limit into account. If you want to consider the joint force limit,
    /// should use SERVO instead. All the joint constraints are invalid.
    VELOCITY,

    /// Locked joint always set the velocity and acceleration to zero so that
    /// the joint dosen't move at all (locked), and the output is joint force.
    /// force.
    ///
    /// All the joint constraints are invalid.
    LOCKED
  };

  /// Default actuator type
  static const ActuatorType DefaultActuatorType;

  /// Constructor
  explicit Joint(const std::string& _name = "Joint");

  /// Destructor
  virtual ~Joint();

  /// \brief Set joint name and return the name.
  /// \param[in] _renameDofs If true, the names of the joint's degrees of
  /// freedom will be updated by calling updateDegreeOfFreedomNames().
  ///
  /// If the name is already taken, this will return an altered version which
  /// will be used by the Skeleton. Otherwise, return _name.
  const std::string& setName(const std::string& _name,
                             bool _renameDofs = true);

  /// Get joint name
  const std::string& getName() const;

  /// Set actuator type
  void setActuatorType(ActuatorType _actuatorType);

  /// Get actuator type
  ActuatorType getActuatorType() const;

  /// Return true if this joint is kinematic joint.
  ///
  /// Kinematic joint means the motion is prescribed by position or velocity or
  /// acceleration, which is determined by the actuator type.
  /// ACCELERATION/VELOCITY/LOCKED are kinematic joints while
  /// FORCE/PASSIVE/SERVO are dynamic joints.
  bool isKinematic() const;

  /// Return true if this joint is dynamic joint.
  bool isDynamic() const;

  /// Get the child BodyNode of this Joint
  BodyNode* getChildBodyNode();

  /// Get the (const) child BodyNode of this Joint
  const BodyNode* getChildBodyNode() const;

  /// Get the parent BodyNode of this Joint
  BodyNode* getParentBodyNode();

  /// Get the (const) parent BodyNode of this Joint
  const BodyNode* getParentBodyNode() const;

  /// Get the Skeleton that this Joint belongs to. The skeleton set by init().
  Skeleton* getSkeleton();

  /// Get the (const) Skeleton that this Joint belongs to.
  const Skeleton* getSkeleton() const;

  /// Set transformation from parent body node to this joint
  virtual void setTransformFromParentBodyNode(const Eigen::Isometry3d& _T);

  /// Set transformation from child body node to this joint
  virtual void setTransformFromChildBodyNode(const Eigen::Isometry3d& _T);

  /// Get transformation from parent body node to this joint
  const Eigen::Isometry3d& getTransformFromParentBodyNode() const;

  /// Get transformation from child body node to this joint
  const Eigen::Isometry3d& getTransformFromChildBodyNode() const;

  /// Set to enforce joint position limit
  ///
  /// The joint position limit is valid when the actutor type is one of
  /// PASSIVE/FORCE.
  ///
  /// \sa ActuatorType
  void setPositionLimited(bool _isPositionLimited);

  /// Get whether enforcing joint position limit
  ///
  /// The joint position limit is valid when the actutor type is one of
  /// PASSIVE/FORCE.
  ///
  /// \sa ActuatorType
  bool isPositionLimited() const;

  /// Set an unique index in skeleton of a generalized coordinate in this joint
  virtual void setIndexInSkeleton(size_t _index, size_t _indexInSkeleton) = 0;

  /// Get an unique index in skeleton of a generalized coordinate in this joint
  virtual size_t getIndexInSkeleton(size_t _index) const = 0;

  /// Get number of generalized coordinates
  DEPRECATED(4.1)
  virtual size_t getDof() const = 0;

  /// Get an object to access the _index-th degree of freedom (generalized
  /// coordinate) of this Joint
  virtual DegreeOfFreedom* getDof(size_t _index) = 0;

  /// Get an object to access the _index-th degree of freedom (generalized
  /// coordinate) of this Joint
  virtual const DegreeOfFreedom* getDof(size_t _index) const = 0;

  /// Get number of generalized coordinates
  virtual size_t getNumDofs() const = 0;

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  /// Set a single command
  virtual void setCommand(size_t _index, double _command) = 0;

  /// Set a sinlge command
  virtual double getCommand(size_t _index) const = 0;

  /// Set commands
  virtual void setCommands(const Eigen::VectorXd& _commands) = 0;

  /// Get commands
  virtual Eigen::VectorXd getCommands() const = 0;

  /// Set zero all the positions
  virtual void resetCommands() = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Position
  //----------------------------------------------------------------------------

  /// Set a single position
  virtual void setPosition(size_t _index, double _position) = 0;

  /// Get a single position
  virtual double getPosition(size_t _index) const = 0;

  /// Set positions
  virtual void setPositions(const Eigen::VectorXd& _positions) = 0;

  /// Get positions
  virtual Eigen::VectorXd getPositions() const = 0;

  /// Set zero all the positions
  virtual void resetPositions() = 0;

  /// Set lower limit of position
  virtual void setPositionLowerLimit(size_t _index, double _position) = 0;

  /// Get lower limit for position
  virtual double getPositionLowerLimit(size_t _index) const = 0;

  /// Set upper limit for position
  virtual void setPositionUpperLimit(size_t _index, double _position) = 0;

  /// Get upper limit for position
  virtual double getPositionUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity
  //----------------------------------------------------------------------------

  /// Set a single velocity
  virtual void setVelocity(size_t _index, double _velocity) = 0;

  /// Get a single velocity
  virtual double getVelocity(size_t _index) const = 0;

  /// Set velocities
  virtual void setVelocities(const Eigen::VectorXd& _velocities) = 0;

  /// Get velocities
  virtual Eigen::VectorXd getVelocities() const = 0;

  /// Set zero all the velocities
  virtual void resetVelocities() = 0;

  /// Set lower limit of velocity
  virtual void setVelocityLowerLimit(size_t _index, double _velocity) = 0;

  /// Get lower limit of velocity
  virtual double getVelocityLowerLimit(size_t _index) const = 0;

  /// Set upper limit of velocity
  virtual void setVelocityUpperLimit(size_t _index, double _velocity) = 0;

  /// Get upper limit of velocity
  virtual double getVelocityUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Acceleration
  //----------------------------------------------------------------------------

  /// Set a single acceleration
  virtual void setAcceleration(size_t _index, double _acceleration) = 0;

  /// Get a single acceleration
  virtual double getAcceleration(size_t _index) const = 0;

  /// Set accelerations
  virtual void setAccelerations(const Eigen::VectorXd& _accelerations) = 0;

  /// Get accelerations
  virtual Eigen::VectorXd getAccelerations() const = 0;

  /// Set zero all the accelerations
  virtual void resetAccelerations() = 0;

  /// Set lower limit of acceleration
  virtual void setAccelerationLowerLimit(size_t _index, double _acceleration) = 0;

  /// Get lower limit of acceleration
  virtual double getAccelerationLowerLimit(size_t _index) const = 0;

  /// Set upper limit of acceleration
  virtual void setAccelerationUpperLimit(size_t _index, double _acceleration) = 0;

  /// Get upper limit of acceleration
  virtual double getAccelerationUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Force
  //----------------------------------------------------------------------------

  /// Set a single force
  virtual void setForce(size_t _index, double _force) = 0;

  /// Get a single force
  virtual double getForce(size_t _index) = 0;

  /// Set forces
  virtual void setForces(const Eigen::VectorXd& _forces) = 0;

  /// Get forces
  virtual Eigen::VectorXd getForces() const = 0;

  /// Set zero all the forces
  virtual void resetForces() = 0;

  /// Set lower limit of force
  virtual void setForceLowerLimit(size_t _index, double _force) = 0;

  /// Get lower limit of force
  virtual double getForceLowerLimit(size_t _index) const = 0;

  /// Set upper limit of position
  virtual void setForceUpperLimit(size_t _index, double _force) = 0;

  /// Get upper limit of position
  virtual double getForceUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity change
  //----------------------------------------------------------------------------

  /// Set a single velocity change
  virtual void setVelocityChange(size_t _index, double _velocityChange) = 0;

  /// Get a single velocity change
  virtual double getVelocityChange(size_t _index) const = 0;

  /// Set zero all the velocity change
  virtual void resetVelocityChanges() = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Constraint impulse
  //----------------------------------------------------------------------------

  /// Set a single constraint impulse
  virtual void setConstraintImpulse(size_t _index, double _impulse) = 0;

  /// Get a single constraint impulse
  virtual double getConstraintImpulse(size_t _index) const = 0;

  /// Set zero all the constraint impulses
  virtual void resetConstraintImpulses() = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Integration
  //----------------------------------------------------------------------------

  /// Integrate positions using Euler method
  virtual void integratePositions(double _dt) = 0;

  /// Integrate velocities using Euler method
  virtual void integrateVelocities(double _dt) = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Passive forces - spring, viscous friction, Coulomb friction
  //----------------------------------------------------------------------------

  /// Set stiffness of joint spring force.
  /// \param[in] _index Index of joint axis.
  /// \param[in] _k Spring stiffness.
  virtual void setSpringStiffness(size_t _index, double _k) = 0;

  /// Get stiffness of joint spring force.
  /// \param[in] _index Index of joint axis.
  virtual double getSpringStiffness(size_t _index) const = 0;

  /// Set rest position of spring force.
  /// \param[in] _index Index of joint axis.
  /// \param[in] _q0 Rest position.
  virtual void setRestPosition(size_t _index, double _q0) = 0;

  /// Get rest position of spring force.
  /// \param[in] _index Index of joint axis.
  /// \return Rest position.
  virtual double getRestPosition(size_t _index) const = 0;

  /// Set coefficient of joint damping (viscous friction) force.
  /// \param[in] _index Index of joint axis.
  /// \param[in] _coeff Damping coefficient.
  virtual void setDampingCoefficient(size_t _index, double _coeff) = 0;

  /// Get coefficient of joint damping (viscous friction) force.
  /// \param[in] _index Index of joint axis.
  virtual double getDampingCoefficient(size_t _index) const = 0;

  /// Set joint Coulomb friction froce.
  /// \param[in] _index Index of joint axis.
  /// \param[in] _friction Joint Coulomb friction froce given index.
  virtual void setCoulombFriction(size_t _index, double _friction) = 0;

  /// Get joint Coulomb friction froce.
  /// \param[in] _index Index of joint axis.
  virtual double getCoulombFriction(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------

  /// Get potential energy
  virtual double getPotentialEnergy() const = 0;

  //----------------------------------------------------------------------------

  /// Get transformation from parent BodyNode to child BodyNode
  const Eigen::Isometry3d& getLocalTransform() const;

  /// Get the velocity from the parent BodyNode to the child BodyNode
  const Eigen::Vector6d& getLocalSpatialVelocity() const;

  /// Get the acceleration from the parent BodyNode to the child BodyNode
  const Eigen::Vector6d& getLocalSpatialAcceleration() const;

  /// Get generalized Jacobian from parent body node to child body node
  /// w.r.t. local generalized coordinate
  virtual const math::Jacobian getLocalJacobian() const = 0;

  /// Get time derivative of generalized Jacobian from parent body node
  /// to child body node w.r.t. local generalized coordinate
  virtual const math::Jacobian getLocalJacobianTimeDeriv() const = 0;

  /// Get whether this joint contains _genCoord
  /// \param[in] Generalized coordinate to see
  /// \return True if this joint contains _genCoord
//  bool contains(const GenCoord* _genCoord) const;

  /// Get local index of the dof at this joint; if the dof is not presented at
  /// this joint, return -1
//  int getGenCoordLocalIndex(int _dofSkelIndex) const;

  /// Get constraint wrench expressed in body node frame
  virtual Eigen::Vector6d getBodyConstraintWrench() const = 0;
  // TODO: Need more informative name.

  /// Get spring force
  ///
  /// We apply spring force in implicit manner. The spring force is
  /// F = -(springStiffness * q(k+1)), where q(k+1) is approximated as
  /// q(k) + h * dq(k) * h^2 * ddq(k). Since, in the recursive forward dynamics
  /// algorithm, ddq(k) is unknown variable that we want to obtain as the
  /// result, the spring force here is just
  /// F = -springStiffness * (q(k) + h * dq(k)) and
  /// -springStiffness * h^2 * ddq(k) term is rearranged at the recursive
  /// forward dynamics algorithm, and it affects on the articulated inertia.
  /// \sa BodyNode::updateArticulatedInertia(double).
  ///
  /// \param[in] _timeStep Time step used for approximating q(k+1).
//  Eigen::VectorXd getSpringForces(double _timeStep) const;

  /// Get damping force
  ///
  /// We apply the damping force in implicit manner. The damping force is
  /// F = -(dampingCoefficient * dq(k+1)), where dq(k+1) is approximated as
  /// dq(k) + h * ddq(k). Since, in the recursive forward dynamics algorithm,
  /// ddq(k) is unknown variable that we want to obtain as the result, the
  /// damping force here is just F = -(dampingCoefficient * dq(k)) and
  /// -dampingCoefficient * h * ddq(k) term is rearranged at the recursive
  /// forward dynamics algorithm, and it affects on the articulated inertia.
  /// \sa BodyNode::updateArticulatedInertia(double).
//  Eigen::VectorXd getDampingForces() const;

  //----------------------------------------------------------------------------
  // Rendering
  //----------------------------------------------------------------------------

  ///
  void applyGLTransform(renderer::RenderInterface* _ri);

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class BodyNode;
  friend class SoftBodyNode;
  friend class Skeleton;

protected:
  /// Initialize this joint. This function is called by BodyNode::init()
  virtual void init(Skeleton* _skel);

  /// \brief Create a DegreeOfFreedom pointer.
  /// \param[in] _name DegreeOfFreedom's name.
  /// \param[in] _indexInJoint DegreeOfFreedom's index within the joint. Note
  /// that the index should be unique within the joint.
  ///
  /// DegreeOfFreedom should be created by the Joint because the DegreeOfFreedom
  /// class has a protected constructor, and the Joint is responsible for memory
  /// management of the pointer which is returned.
  DegreeOfFreedom* createDofPointer(const std::string& _name,
                                    size_t _indexInJoint);

  /// Update the names of the joint's degrees of freedom. Used when setName() is
  /// called with _rename_dofs set to true.
  virtual void updateDegreeOfFreedomNames() = 0;

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  /// Update transformation from parent BodyNode to child BodyNode
  virtual void updateLocalTransform() const = 0;

  /// Update velocity from the parent BodyNode to the child BodyNode
  virtual void updateLocalSpatialVelocity() const = 0;

  /// Update acceleration from the parent BodyNode to the child BodyNode
  virtual void updateLocalSpatialAcceleration() const = 0;

  /// Update generalized Jacobian from parent body node to child body
  /// node w.r.t. local generalized coordinate
  ///
  /// The _mandatory argument can be set to false if the Jacobian update request
  /// is due to a change in Joint positions, because not all Joint types have a
  /// Local Jacobian that depends on their Joint positions, so a Local Jacobian
  /// update would not actually be required.
  virtual void updateLocalJacobian(bool _mandatory=true) const = 0;

  /// Update time derivative of generalized Jacobian from parent body
  /// node to child body node w.r.t. local generalized coordinate
  ///
  /// If the Local Jacobian Time Derivative of this Joint is zero, then this
  /// function will be a no op.
  virtual void updateLocalJacobianTimeDeriv() const = 0;

  /// Add joint velocity to _vel
  virtual void addVelocityTo(Eigen::Vector6d& _vel) = 0;

  /// Set joint partial acceleration to _partialAcceleration
  virtual void setPartialAccelerationTo(
      Eigen::Vector6d& _partialAcceleration,
      const Eigen::Vector6d& _childVelocity) = 0;
  // TODO(JS): Rename with more informative name

  /// Add joint acceleration to _acc
  virtual void addAccelerationTo(Eigen::Vector6d& _acc) = 0;

  /// Add joint velocity change to _velocityChange
  virtual void addVelocityChangeTo(Eigen::Vector6d& _velocityChange) = 0;

  /// Add child's articulated inertia to parent's one
  virtual void addChildArtInertiaTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia) = 0;

  /// Add child's articulated inertia to parent's one. Forward dynamics routine.
  virtual void addChildArtInertiaImplicitTo(
      Eigen::Matrix6d& _parentArtInertiaImplicit,
      const Eigen::Matrix6d& _childArtInertiaImplicit) = 0;
  // TODO(JS): rename to updateAInertiaChildAInertia()

  /// Update inverse of projected articulated body inertia
  virtual void updateInvProjArtInertia(const Eigen::Matrix6d& _artInertia) = 0;

  /// Forward dynamics routine.
  virtual void updateInvProjArtInertiaImplicit(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep) = 0;
  // TODO(JS): rename to updateAInertiaPsi()

  /// Add child's bias force to parent's one
  virtual void addChildBiasForceTo(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc) = 0;

  /// Add child's bias impulse to parent's one
  virtual void addChildBiasImpulseTo(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse) = 0;

  /// Update joint total force
  virtual void updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                double _timeStep) = 0;
  // TODO: rename

  /// Update joint total impulse
  virtual void updateTotalImpulse(const Eigen::Vector6d& _bodyImpulse) = 0;
  // TODO: rename

  /// Set total impulses to zero
  virtual void resetTotalImpulses() = 0;

  /// Update joint acceleration
  virtual void updateAcceleration(const Eigen::Matrix6d& _artInertia,
                                  const Eigen::Vector6d& _spatialAcc) = 0;

  /// Update joint velocity change
  /// \param _artInertia
  /// \param _velocityChange
  virtual void updateVelocityChange(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _velocityChange) = 0;

  /// Update joint force for inverse dynamics.
  /// \param[in] _bodyForce Transmitting spatial body force from the parent
  /// BodyNode to the child BodyNode. The spatial force is expressed in the
  /// child BodyNode's frame.
  virtual void updateForceID(const Eigen::Vector6d& _bodyForce,
                             double _timeStep,
                             bool _withDampingForces,
                             bool _withSpringForces) = 0;

  /// Update joint force for forward dynamics.
  /// \param[in] _bodyForce Transmitting spatial body force from the parent
  /// BodyNode to the child BodyNode. The spatial force is expressed in the
  /// child BodyNode's frame.
  virtual void updateForceFD(const Eigen::Vector6d& _bodyForce,
                             double _timeStep,
                             bool _withDampingForcese,
                             bool _withSpringForces) = 0;

  /// Update joint impulses for inverse dynamics
  virtual void updateImpulseID(const Eigen::Vector6d& _bodyImpulse) = 0;

  /// Update joint impulses for forward dynamics
  virtual void updateImpulseFD(const Eigen::Vector6d& _bodyImpulse) = 0;

  /// Update constrained terms for forward dynamics
  virtual void updateConstrainedTerms(double _timeStep) = 0;

  //- DEPRECATED ---------------------------------------------------------------

  /// updateVelocityWithVelocityChange
  DEPRECATED(4.3)
  virtual void updateVelocityWithVelocityChange() {}

  /// updateAccelerationWithVelocityChange
  DEPRECATED(4.3)
  virtual void updateAccelerationWithVelocityChange(double _timeStep) {}

  /// updateForceWithImpulse
  DEPRECATED(4.3)
  virtual void updateForceWithImpulse(double _timeStep) {}

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Recursive algorithm routines for equations of motion
  //----------------------------------------------------------------------------

  /// Add child's bias force to parent's one
  virtual void addChildBiasForceForInvMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) = 0;

  /// Add child's bias force to parent's one
  virtual void addChildBiasForceForInvAugMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) = 0;

  ///
  virtual void updateTotalForceForInvMassMatrix(
      const Eigen::Vector6d& _bodyForce) = 0;

  ///
  virtual void getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                       const size_t _col,
                                       const Eigen::Matrix6d& _artInertia,
                                       const Eigen::Vector6d& _spatialAcc) = 0;

  ///
  virtual void getInvAugMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                       const size_t _col,
                                       const Eigen::Matrix6d& _artInertia,
                                       const Eigen::Vector6d& _spatialAcc) = 0;

  ///
  virtual void addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc) = 0;

  ///
  virtual Eigen::VectorXd getSpatialToGeneralized(
      const Eigen::Vector6d& _spatial) = 0;

  /// \}

  /// Notify that a position update is needed
  virtual void notifyPositionUpdate();

  /// Notify that a velocity update is needed
  virtual void notifyVelocityUpdate();

  /// Notify that an acceleration update is needed
  virtual void notifyAccelerationUpdate();

protected:
  /// Joint name
  std::string mName;

  /// Actuator type
  ActuatorType mActuatorType;

  /// Child BodyNode pointer that this Joint belongs to
  BodyNode* mChildBodyNode;

  /// Skeleton pointer that this joint belongs to
  Skeleton* mSkeleton;

  /// Transformation from parent body node to this joint
  Eigen::Isometry3d mT_ParentBodyToJoint;

  /// Transformation from child body node to this joint
  Eigen::Isometry3d mT_ChildBodyToJoint;

  /// Local transformation
  ///
  /// Do not use directly! Use getLocalTransform() to access this quantity
  mutable Eigen::Isometry3d mT;

  /// Relative spatial velocity from parent BodyNode to child BodyNode where the
  /// velocity is expressed in child body Frame
  mutable Eigen::Vector6d mSpatialVelocity;

  /// Relative spatial acceleration from parent BodyNode to child BodyNode where
  /// the acceleration is expressed in the child body Frame
  mutable Eigen::Vector6d mSpatialAcceleration;

  /// True iff this joint's position has changed since the last call to
  /// getLocalTransform()
  bool mNeedPositionUpdate;

  /// True iff this joint's position or velocity has changed since the last call
  /// to getLocalSpatialVelocity()
  bool mNeedVelocityUpdate;

  /// True iff this joint's position, velocity, or acceleration has changed
  /// since the last call to getLocalSpatialAcceleration()
  bool mNeedAccelerationUpdate;

  /// True iff this joint's local Jacobian has not been updated since the last
  /// position change
  bool mIsLocalJacobianDirty;

  /// True iff this joint's local Jacobian time derivative has not been updated
  /// since the last position or velocity change
  bool mIsLocalJacobianTimeDerivDirty;

  /// Transmitting wrench from parent body to child body expressed in child body
  DEPRECATED(4.3)
  Eigen::Vector6d mWrench;

protected:
  /// True if the joint limits are enforced in dynamic simulation
  bool mIsPositionLimited;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_JOINT_H_
