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
#include <memory>

#include "dart/common/Deprecated.h"
#include "dart/common/Subject.h"
#include "dart/common/VersionCounter.h"
#include "dart/common/EmbeddedAspect.h"
#include "dart/math/MathTypes.h"
#include "dart/dynamics/SmartPointer.h"
#include "dart/dynamics/detail/Joint.h"

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
class Joint : public virtual common::Subject,
              public virtual common::VersionCounter,
              public common::EmbedProperties<Joint, detail::JointProperties>
{
public:

  using CompositeProperties = common::Composite::Properties;
  using Properties = detail::JointProperties;

  typedef detail::ActuatorType ActuatorType;
  static constexpr ActuatorType FORCE        = detail::FORCE;
  static constexpr ActuatorType PASSIVE      = detail::PASSIVE;
  static constexpr ActuatorType SERVO        = detail::SERVO;
  static constexpr ActuatorType ACCELERATION = detail::ACCELERATION;
  static constexpr ActuatorType VELOCITY     = detail::VELOCITY;
  static constexpr ActuatorType LOCKED       = detail::LOCKED;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, JointAspect);

  struct ExtendedProperties : Properties
  {
    /// Composed constructor
    ExtendedProperties(
        const Properties& standardProperties = Properties(),
        const CompositeProperties& aspectProperties = CompositeProperties());

    /// Composed move constructor
    ExtendedProperties(
        Properties&& standardProperties,
        CompositeProperties&& aspectProperties);

    /// Properties of all the Aspects attached to this Joint
    CompositeProperties mAspectProperties;
  };

  /// Default actuator type
  static const ActuatorType DefaultActuatorType;

  Joint(const Joint&) = delete;

  /// Destructor
  virtual ~Joint();

  /// Set the Properties of this Joint
  void setProperties(const Properties& properties);

  /// Set the AspectProperties of this Joint
  void setAspectProperties(const AspectProperties& properties);

  /// Get the Properties of this Joint
  const Properties& getJointProperties() const;

  /// Copy the properties of another Joint
  void copy(const Joint& _otherJoint);

  /// Copy the properties of another Joint
  void copy(const Joint* _otherJoint);

  /// Same as copy(const Joint&)
  Joint& operator=(const Joint& _otherJoint);

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

  /// Increments Skeleton version number
  size_t incrementVersion() override;

  /// Gets the Skeleton version number
  size_t getVersion() const override;

  /// Gets a string representing the joint type
  virtual const std::string& getType() const = 0;

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

  /// Get the Skeleton that this Joint belongs to
  SkeletonPtr getSkeleton();

  /// Get the (const) Skeleton that this Joint belongs to.
  std::shared_ptr<const Skeleton> getSkeleton() const;

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
  void setPositionLimitEnforced(bool _isPositionLimited);

  /// Deprecated. Replaced by setPositionLimitEnforced.
  DEPRECATED(5.0) void setPositionLimited(bool _isPositionLimited);

  /// Get whether enforcing joint position limit
  ///
  /// The joint position limit is valid when the actutor type is one of
  /// PASSIVE/FORCE.
  ///
  /// \sa ActuatorType
  bool isPositionLimitEnforced() const;

  /// Deprecated. Replaced by isPositionLimitEnforced.
  DEPRECATED(5.0) bool isPositionLimited() const;

  /// Get a unique index in skeleton of a generalized coordinate in this Joint
  virtual size_t getIndexInSkeleton(size_t _index) const = 0;

  /// Get a unique index in the kinematic tree of a generalized coordinate in
  /// this Joint
  virtual size_t getIndexInTree(size_t _index) const = 0;

  /// Get the index of this Joint within its Skeleton
  size_t getJointIndexInSkeleton() const;

  /// Get the index of this Joint within its tree
  size_t getJointIndexInTree() const;

  /// Get the index of the tree that this Joint belongs to
  size_t getTreeIndex() const;

  /// Get an object to access the _index-th degree of freedom (generalized
  /// coordinate) of this Joint
  virtual DegreeOfFreedom* getDof(size_t _index) = 0;

  /// Get an object to access the _index-th degree of freedom (generalized
  /// coordinate) of this Joint
  virtual const DegreeOfFreedom* getDof(size_t _index) const = 0;

  /// Alternative to DegreeOfFreedom::setName()
  virtual const std::string& setDofName(size_t _index,
                                const std::string& _name,
                                bool _preserveName=true) = 0;

  /// Alternative to DegreeOfFreedom::preserveName()
  virtual void preserveDofName(size_t _index, bool _preserve) = 0;

  /// Alternative to DegreeOfFreedom::isNamePreserved()
  virtual bool isDofNamePreserved(size_t _index) const = 0;

  /// Alternative to DegreeOfFreedom::getName()
  virtual const std::string& getDofName(size_t _index) const = 0;

  /// Get number of generalized coordinates
  virtual size_t getNumDofs() const = 0;

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  /// Set a single command
  virtual void setCommand(size_t _index, double _command) = 0;

  /// Get a single command
  virtual double getCommand(size_t _index) const = 0;

  /// Set all commands for this Joint
  virtual void setCommands(const Eigen::VectorXd& _commands) = 0;

  /// Get all commands for this Joint
  virtual Eigen::VectorXd getCommands() const = 0;

  /// Set all the commands for this Joint to zero
  virtual void resetCommands() = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Position
  //----------------------------------------------------------------------------

  /// Set the position of a single generalized coordinate
  virtual void setPosition(size_t _index, double _position) = 0;

  /// Get the position of a single generalized coordinate
  virtual double getPosition(size_t _index) const = 0;

  /// Set the positions of all generalized coordinates in this Joint
  virtual void setPositions(const Eigen::VectorXd& _positions) = 0;

  /// Get the positions of all generalized coordinates in this Joint
  virtual Eigen::VectorXd getPositions() const = 0;

  /// Set lower limit for position
  virtual void setPositionLowerLimit(size_t _index, double _position) = 0;

  /// Get lower limit for position
  virtual double getPositionLowerLimit(size_t _index) const = 0;

  /// Set upper limit for position
  virtual void setPositionUpperLimit(size_t _index, double _position) = 0;

  /// Get upper limit for position
  virtual double getPositionUpperLimit(size_t _index) const = 0;

  /// Get whether a generalized coordinate is cyclic. Return true if and only
  /// if this generalized coordinate has an infinite number of positions that
  /// produce the same local transform. Note that, for a multi-DOF joint,
  /// producing a cycle may require altering the position of this Joint's other
  /// generalized coordinates.
  virtual bool isCyclic(size_t _index) const = 0;

  /// Get whether the position of a generalized coordinate has limits.
  virtual bool hasPositionLimit(size_t _index) const = 0;

  /// Set the position of this generalized coordinate to its initial position
  virtual void resetPosition(size_t _index) = 0;

  /// Set the positions of all generalized coordinates in this Joint to their
  /// initial positions
  virtual void resetPositions() = 0;

  /// Change the position that resetPositions() will give to this coordinate
  virtual void setInitialPosition(size_t _index, double _initial) = 0;

  /// Get the position that resetPosition() will give to this coordinate
  virtual double getInitialPosition(size_t _index) const = 0;

  /// Change the positions that resetPositions() will give to this Joint's
  /// coordinates
  virtual void setInitialPositions(const Eigen::VectorXd& _initial) = 0;

  /// Get the positions that resetPositions() will give to this Joint's
  /// coordinates
  virtual Eigen::VectorXd getInitialPositions() const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity
  //----------------------------------------------------------------------------

  /// Set the velocity of a single generalized coordinate
  virtual void setVelocity(size_t _index, double _velocity) = 0;

  /// Get the velocity of a single generalized coordinate
  virtual double getVelocity(size_t _index) const = 0;

  /// Set the velocities of all generalized coordinates in this Joint
  virtual void setVelocities(const Eigen::VectorXd& _velocities) = 0;

  /// Get the velocities of all generalized coordinates in this Joint
  virtual Eigen::VectorXd getVelocities() const = 0;

  /// Set lower limit for velocity
  virtual void setVelocityLowerLimit(size_t _index, double _velocity) = 0;

  /// Get lower limit for velocity
  virtual double getVelocityLowerLimit(size_t _index) const = 0;

  /// Set upper limit for velocity
  virtual void setVelocityUpperLimit(size_t _index, double _velocity) = 0;

  /// Get upper limit for velocity
  virtual double getVelocityUpperLimit(size_t _index) const = 0;

  /// Set the velocity of a generalized coordinate in this Joint to its initial
  /// velocity
  virtual void resetVelocity(size_t _index) = 0;

  /// Set the velocities of all generalized coordinates in this Joint to their
  /// initial velocities
  virtual void resetVelocities() = 0;

  /// Change the velocity that resetVelocity() will give to this coordinate
  virtual void setInitialVelocity(size_t _index, double _initial) = 0;

  /// Get the velocity that resetVelocity() will give to this coordinate
  virtual double getInitialVelocity(size_t _index) const = 0;

  /// Change the velocities that resetVelocities() will give to this Joint's
  /// coordinates
  virtual void setInitialVelocities(const Eigen::VectorXd& _initial) = 0;

  /// Get the velocities that resetVelocities() will give to this Joint's
  /// coordinates
  virtual Eigen::VectorXd getInitialVelocities() const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Acceleration
  //----------------------------------------------------------------------------

  /// Set the acceleration of a single generalized coordinate
  virtual void setAcceleration(size_t _index, double _acceleration) = 0;

  /// Get the acceleration of a single generalized coordinate
  virtual double getAcceleration(size_t _index) const = 0;

  /// Set the accelerations of all generalized coordinates in this Joint
  virtual void setAccelerations(const Eigen::VectorXd& _accelerations) = 0;

  /// Get the accelerations of all generalized coordinates in this Joint
  virtual Eigen::VectorXd getAccelerations() const = 0;

  /// Set the accelerations of all generalized coordinates in this Joint to zero
  virtual void resetAccelerations() = 0;

  /// Set lower limit for acceleration
  virtual void setAccelerationLowerLimit(size_t _index, double _acceleration) = 0;

  /// Get lower limit for acceleration
  virtual double getAccelerationLowerLimit(size_t _index) const = 0;

  /// Set upper limit for acceleration
  virtual void setAccelerationUpperLimit(size_t _index, double _acceleration) = 0;

  /// Get upper limit for acceleration
  virtual double getAccelerationUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Force
  //----------------------------------------------------------------------------

  /// Set the force of a single generalized coordinate
  virtual void setForce(size_t _index, double _force) = 0;

  /// Get the force of a single generalized coordinate
  virtual double getForce(size_t _index) = 0;

  /// Set the forces of all generalized coordinates in this Joint
  virtual void setForces(const Eigen::VectorXd& _forces) = 0;

  /// Get the forces of all generalized coordinates in this Joint
  virtual Eigen::VectorXd getForces() const = 0;

  /// Set the forces of all generalized coordinates in this Joint to zero
  virtual void resetForces() = 0;

  /// Set lower limit for force
  virtual void setForceLowerLimit(size_t _index, double _force) = 0;

  /// Get lower limit for force
  virtual double getForceLowerLimit(size_t _index) const = 0;

  /// Set upper limit for force
  virtual void setForceUpperLimit(size_t _index, double _force) = 0;

  /// Get upper limit for force
  virtual double getForceUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Sanity Check
  //----------------------------------------------------------------------------

  /// Returns false if the initial position or initial velocity are outside of
  /// limits
  // TODO: Consider extending this to a more comprehensive sanity check
  bool checkSanity(bool _printWarnings = true) const;

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
  /// \{ \name Integration and finite difference
  //----------------------------------------------------------------------------

  /// Integrate positions using Euler method
  virtual void integratePositions(double _dt) = 0;

  /// Integrate velocities using Euler method
  virtual void integrateVelocities(double _dt) = 0;

  /// Return the difference of two generalized coordinates which are measured in
  /// the configuration space of this Skeleton.
  virtual Eigen::VectorXd getPositionDifferences(
      const Eigen::VectorXd& _q2, const Eigen::VectorXd& _q1) const = 0;

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

  /// Get the J * q_dd of this joint
  const Eigen::Vector6d& getLocalPrimaryAcceleration() const;

  /// Get generalized Jacobian from parent body node to child body node
  /// w.r.t. local generalized coordinate
  virtual const math::Jacobian getLocalJacobian() const = 0;

  /// Get generalized Jacobian from parent body node to child body node
  /// w.r.t. local generalized coordinate
  virtual math::Jacobian getLocalJacobian(
      const Eigen::VectorXd& _positions) const = 0;

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
  /// \{ \name Update Notifiers
  //----------------------------------------------------------------------------

  /// Notify that a position update is needed
  void notifyPositionUpdate();

  /// Notify that a velocity update is needed
  void notifyVelocityUpdate();

  /// Notify that an acceleration update is needed
  void notifyAccelerationUpdate();

  /// \}

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

  /// Constructor called by inheriting class
  Joint();

  /// Create a clone of this Joint. This may only be called by the Skeleton
  /// class.
  virtual Joint* clone() const = 0;

  /// Called by the Skeleton class
  virtual void registerDofs() = 0;

  /// \brief Create a DegreeOfFreedom pointer.
  /// \param[in] _name DegreeOfFreedom's name.
  /// \param[in] _indexInJoint DegreeOfFreedom's index within the joint. Note
  /// that the index should be unique within the joint.
  ///
  /// DegreeOfFreedom should be created by the Joint because the DegreeOfFreedom
  /// class has a protected constructor, and the Joint is responsible for memory
  /// management of the pointer which is returned.
  DegreeOfFreedom* createDofPointer(size_t _indexInJoint);

  /// Update the names of the joint's degrees of freedom. Used when setName() is
  /// called with _renameDofs set to true.
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

  /// Update the J * q_dd of this joint
  virtual void updateLocalPrimaryAcceleration() const = 0;

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

  /// Tells the Skeleton to update the articulated inertia if it needs updating
  void updateArticulatedInertia() const;

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

protected:

  /// Child BodyNode pointer that this Joint belongs to
  BodyNode* mChildBodyNode;

  /// Local transformation
  ///
  /// Do not use directly! Use getLocalTransform() to access this
  mutable Eigen::Isometry3d mT;

  /// Relative spatial velocity from parent BodyNode to child BodyNode where the
  /// velocity is expressed in child body Frame
  ///
  /// Do not use directly! Use getLocalSpatialVelocity() to access this
  mutable Eigen::Vector6d mSpatialVelocity;

  /// Relative spatial acceleration from parent BodyNode to child BodyNode where
  /// the acceleration is expressed in the child body Frame
  ///
  /// Do not use directly! Use getLocalSpatialAcceleration() to access this
  mutable Eigen::Vector6d mSpatialAcceleration;

  /// J * q_dd
  ///
  /// Do not use directly! Use getLocalPrimaryAcceleration() to access this
  mutable Eigen::Vector6d mPrimaryAcceleration;

  /// True iff this joint's position has changed since the last call to
  /// getLocalTransform()
  mutable bool mNeedTransformUpdate;

  /// True iff this joint's position or velocity has changed since the last call
  /// to getLocalSpatialVelocity()
  mutable bool mNeedSpatialVelocityUpdate;

  /// True iff this joint's position, velocity, or acceleration has changed
  /// since the last call to getLocalSpatialAcceleration()
  mutable bool mNeedSpatialAccelerationUpdate;

  /// True iff this joint's position, velocity, or acceleration has changed
  /// since the last call to getLocalPrimaryAcceleration()
  mutable bool mNeedPrimaryAccelerationUpdate;

  /// True iff this joint's local Jacobian has not been updated since the last
  /// position change
  mutable bool mIsLocalJacobianDirty;

  /// True iff this joint's local Jacobian time derivative has not been updated
  /// since the last position or velocity change
  mutable bool mIsLocalJacobianTimeDerivDirty;

public:

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace detail {

template <class AspectType>
void JointPropertyUpdate(AspectType* aspect)
{
  aspect->getComposite()->notifyPositionUpdate();
  aspect->getComposite()->updateLocalJacobian();
}

} // namespace detail

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_JOINT_H_
