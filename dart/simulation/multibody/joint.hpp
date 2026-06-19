/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#pragma once

#include <dart/simulation/entity.hpp>
#include <dart/simulation/fwd.hpp>
#include <dart/simulation/multibody/joint_type.hpp>

#include <Eigen/Core>

#include <limits>
#include <string>
#include <string_view>

#include <cstddef>

namespace dart::simulation {

/// Public joint actuator type used by the experimental multibody facade.
///
/// Describes how a joint is driven during forward dynamics. `Force` (the
/// default) applies the commanded joint effort; `Passive` applies no commanded
/// effort (only passive spring/damping/friction forces act). The remaining
/// modes are reserved and not yet implemented in the forward dynamics.
enum class ActuatorType
{
  Force,
  Passive,
  Servo,
  Velocity,
  Acceleration,
  Locked,
  Mimic
};

/// Solver-neutral policy for point-joint constraint projection stiffness.
///
/// The policy applies to public world-owned point-joint facades. A finite
/// linear or angular stiffness makes the matching projected rows finite-
/// stiffness rows; infinity keeps the default hard-constraint behavior.
struct JointConstraintProjectionPolicy
{
  /// Starting projection stiffness. Must be finite and non-negative.
  double startStiffness = 1.0;

  /// Linear projection material stiffness. Infinity keeps hard rows.
  double linearStiffness = std::numeric_limits<double>::infinity();

  /// Angular projection material stiffness. Infinity keeps hard rows.
  double angularStiffness = std::numeric_limits<double>::infinity();
};

/// Generic Joint handle class
///
/// Represents a joint connecting two links in a Multibody. This is a
/// lightweight handle to World-owned articulation state.
///
/// Unlike traditional OOP physics engines, the experimental stack uses a single
/// generic Joint class. The handle provides a unified interface regardless of
/// joint type (revolute, prismatic, ball, etc.).
///
/// This design provides:
/// - No polymorphism overhead (no vtables)
/// - Better cache locality (data-oriented)
/// - Type checking via JointType enum at runtime
/// - Natural integration with ECS systems
///
/// Usage:
/// @code
///   auto link = robot.addLink("link1", {
///       .parentLink = root,
///       .jointName = "shoulder",
///       .jointType = JointType::Revolute
///   });
///
///   Joint joint = link.getParentJoint();
///   joint.setPosition(0.5);
///   joint.setLimits(-M_PI, M_PI);
/// @endcode
///
/// @note Joint handles are lightweight and safe to copy. Handles become invalid
///       if the referenced joint or World is destroyed.
class DART_SIMULATION_API Joint
{
public:
  /// Constructor (typically called by Link::getParentJoint)
  ///
  /// @param entity The opaque identity token.
  /// @param world Pointer to the World owning this entity
  Joint(Entity entity, World* world);

  /// Get the name of this joint
  ///
  /// @return The joint name
  [[nodiscard]] std::string_view getName() const;

  /// Get the joint type
  ///
  /// @return JointType enum (Revolute, Prismatic, etc.)
  [[nodiscard]] JointType getType() const;

  /// Get the joint actuator type (default ActuatorType::Force).
  [[nodiscard]] ActuatorType getActuatorType() const;

  /// Set the joint actuator type.
  ///
  /// `Force` applies the commanded joint effort; `Passive` ignores it (only
  /// passive spring/damping/friction forces act); `Velocity` drives the joint
  /// to its commanded velocity (see setCommandVelocity) via a velocity-level
  /// constraint. The remaining modes are not yet implemented and are rejected
  /// by the forward dynamics at step time.
  void setActuatorType(ActuatorType actuatorType);

  /// Get the commanded target velocity used by the `Velocity` actuator type.
  [[nodiscard]] Eigen::VectorXd getCommandVelocity() const;

  /// Set the commanded target velocity used by the `Velocity` actuator type.
  ///
  /// When the actuator type is `Velocity`, the forward dynamics drives the
  /// joint's generalized velocity to this value each step. Values must be
  /// finite, with size equal to getDOFCount().
  void setCommandVelocity(const Eigen::VectorXd& velocity);

  /// Get the primary joint axis
  ///
  /// Valid for: Revolute, Prismatic, Screw, Universal, Planar
  /// @return 3D unit vector representing the primary axis
  /// @throws InvalidArgumentException if joint type doesn't use axis
  /// (Spherical, Floating)
  [[nodiscard]] Eigen::Vector3d getAxis() const;

  /// Get the secondary joint axis
  ///
  /// Valid for: Universal (perpendicular rotation axis), Planar (in-plane
  /// direction)
  /// @return 3D unit vector representing the secondary axis
  /// @throws InvalidArgumentException if joint type doesn't use axis2
  [[nodiscard]] Eigen::Vector3d getAxis2() const;

  /// Get the screw pitch (translation per radian of rotation)
  ///
  /// Valid for: Screw joints only
  /// @return Pitch in meters per radian
  /// @throws InvalidArgumentException if not a Screw joint
  [[nodiscard]] double getPitch() const;

  /// Set the screw pitch (translation per radian of rotation).
  ///
  /// Valid for Screw joints; the value must be finite. The articulated-body
  /// forward dynamics couples rotation and translation by this pitch.
  void setPitch(double pitch);

  /// Get the number of generalized coordinates owned by this joint.
  ///
  /// Fixed joints return 0. Revolute, prismatic, and screw joints return 1;
  /// higher-dimensional joints return their public generalized-coordinate
  /// dimension.
  [[nodiscard]] std::size_t getDOFCount() const;

  /// Get a copy of the generalized joint position vector.
  [[nodiscard]] Eigen::VectorXd getPosition() const;

  /// Set the generalized joint position vector.
  ///
  /// @param position Vector with size equal to getDOFCount().
  /// @throws InvalidArgumentException if this handle is invalid, if the vector
  /// size does not match the joint DOF count, or if any value is non-finite.
  void setPosition(const Eigen::VectorXd& position);

  /// Get a copy of the generalized joint velocity vector.
  [[nodiscard]] Eigen::VectorXd getVelocity() const;

  /// Set the generalized joint velocity vector.
  ///
  /// @param velocity Vector with size equal to getDOFCount().
  /// @throws InvalidArgumentException if this handle is invalid, if the vector
  /// size does not match the joint DOF count, or if any value is non-finite.
  void setVelocity(const Eigen::VectorXd& velocity);

  /// Get a copy of the generalized joint force (effort) vector.
  [[nodiscard]] Eigen::VectorXd getForce() const;

  /// Set the generalized joint force (effort) vector.
  ///
  /// This is the actuation effort applied at the joint (torque for revolute,
  /// force for prismatic) and is consumed by forward dynamics. Unlike
  /// rigid-body external forces, the joint effort persists across steps until
  /// changed, so it models a held actuator command.
  ///
  /// @param force Vector with size equal to getDOFCount().
  /// @throws InvalidArgumentException if this handle is invalid, if the vector
  /// size does not match the joint DOF count, or if any value is non-finite.
  void setForce(const Eigen::VectorXd& force);

  /// Get a copy of the generalized joint acceleration vector.
  ///
  /// Reflects the most recent acceleration computed by forward dynamics. It is
  /// zero before the first step.
  [[nodiscard]] Eigen::VectorXd getAcceleration() const;

  /// Get the per-coordinate spring stiffness vector.
  [[nodiscard]] Eigen::VectorXd getSpringStiffness() const;

  /// Set the per-coordinate spring stiffness vector.
  ///
  /// A nonzero stiffness applies a passive restoring force
  /// `-stiffness * (position - restPosition)` during forward dynamics. Values
  /// must be non-negative and finite, with size equal to getDOFCount().
  void setSpringStiffness(const Eigen::VectorXd& stiffness);

  /// Get the per-coordinate spring rest position vector.
  [[nodiscard]] Eigen::VectorXd getRestPosition() const;

  /// Set the per-coordinate spring rest position vector.
  ///
  /// Values must be finite, with size equal to getDOFCount().
  void setRestPosition(const Eigen::VectorXd& restPosition);

  /// Get the per-coordinate damping coefficient vector.
  [[nodiscard]] Eigen::VectorXd getDampingCoefficient() const;

  /// Set the per-coordinate damping coefficient vector.
  ///
  /// A nonzero coefficient applies a passive force `-damping * velocity` during
  /// forward dynamics. Values must be non-negative and finite, with size equal
  /// to getDOFCount().
  void setDampingCoefficient(const Eigen::VectorXd& damping);

  /// Get the per-coordinate rotor/reflected inertia (armature).
  [[nodiscard]] Eigen::VectorXd getArmature() const;

  /// Set the per-coordinate rotor/reflected inertia (armature).
  ///
  /// Armature is added to the joint-space mass-matrix diagonal, modeling the
  /// reflected inertia of a geared actuator. It improves integration stability
  /// for stiff actuators. Values must be non-negative and finite, with size
  /// equal to getDOFCount().
  void setArmature(const Eigen::VectorXd& armature);

  /// Get the per-coordinate Coulomb (dry) friction magnitudes.
  [[nodiscard]] Eigen::VectorXd getCoulombFriction() const;

  /// Set the per-coordinate Coulomb (dry) friction magnitudes.
  ///
  /// A nonzero value applies a bounded dry-friction force/torque that opposes
  /// joint motion: it holds the coordinate at rest while the required holding
  /// effort stays within the friction bound (stiction) and otherwise resists
  /// motion at the friction magnitude (kinetic). Values must be non-negative
  /// and finite, with size equal to getDOFCount().
  void setCoulombFriction(const Eigen::VectorXd& friction);

  /// Set the per-coordinate position limits.
  ///
  /// Each lower bound must be less than or equal to the matching upper bound.
  /// Use +/- infinity for an unbounded coordinate (the default). The
  /// semi-implicit multibody integration clamps positions to these limits and
  /// arrests the velocity driving a coordinate past a limit.
  ///
  /// @param lower Lower bounds, size getDOFCount().
  /// @param upper Upper bounds, size getDOFCount().
  void setPositionLimits(
      const Eigen::VectorXd& lower, const Eigen::VectorXd& upper);

  /// Get the per-coordinate lower position limits (default -infinity).
  [[nodiscard]] Eigen::VectorXd getPositionLowerLimits() const;

  /// Get the per-coordinate upper position limits (default +infinity).
  [[nodiscard]] Eigen::VectorXd getPositionUpperLimits() const;

  /// Set the per-coordinate velocity limits.
  ///
  /// Each lower bound must be less than or equal to the matching upper bound.
  /// Use +/- infinity for an unbounded coordinate (the default). The
  /// semi-implicit multibody integration clamps generalized velocities to these
  /// limits each step.
  ///
  /// @param lower Lower bounds, size getDOFCount().
  /// @param upper Upper bounds, size getDOFCount().
  void setVelocityLimits(
      const Eigen::VectorXd& lower, const Eigen::VectorXd& upper);

  /// Get the per-coordinate lower velocity limits (default -infinity).
  [[nodiscard]] Eigen::VectorXd getVelocityLowerLimits() const;

  /// Get the per-coordinate upper velocity limits (default +infinity).
  [[nodiscard]] Eigen::VectorXd getVelocityUpperLimits() const;

  /// Set the per-coordinate actuation effort (force/torque) limits.
  ///
  /// Each lower bound must be less than or equal to the matching upper bound.
  /// Use +/- infinity for an unbounded coordinate (the default). The
  /// semi-implicit multibody integration clamps the commanded joint effort (set
  /// via setForce) to these limits before solving; passive spring and damping
  /// forces are not subject to the effort limits.
  ///
  /// @param lower Lower bounds, size getDOFCount().
  /// @param upper Upper bounds, size getDOFCount().
  void setEffortLimits(
      const Eigen::VectorXd& lower, const Eigen::VectorXd& upper);

  /// Get the per-coordinate lower effort limits (default -infinity).
  [[nodiscard]] Eigen::VectorXd getEffortLowerLimits() const;

  /// Get the per-coordinate upper effort limits (default +infinity).
  [[nodiscard]] Eigen::VectorXd getEffortUpperLimits() const;

  /// Set the point-joint break-force threshold.
  ///
  /// A finite non-negative value is required. The default value, 0, disables
  /// automatic breakage. When projected point-joint rows accumulate a
  /// constraint load at or above this threshold, the joint is marked broken and
  /// excluded from later point-joint extraction until resetBreakage().
  void setBreakForce(double breakForce);

  /// Get the point-joint break-force threshold.
  [[nodiscard]] double getBreakForce() const;

  /// Return whether this joint has been broken by break-force handling.
  [[nodiscard]] bool isBroken() const;

  /// Clear the broken flag so the joint can participate in extraction again.
  void resetBreakage();

  /// Set the point-joint constraint projection policy.
  ///
  /// This applies to public rigid-body/articulated point-joint facades that
  /// extract into solver projection rows.
  void setConstraintProjectionPolicy(
      const JointConstraintProjectionPolicy& policy);

  /// Get the effective point-joint constraint projection policy.
  [[nodiscard]] JointConstraintProjectionPolicy getConstraintProjectionPolicy()
      const;

  /// Get the parent link
  ///
  /// @return Parent Link handle
  [[nodiscard]] Link getParentLink() const;

  /// Get the child link
  ///
  /// @return Child Link handle
  [[nodiscard]] Link getChildLink() const;

  /// Get the parent rigid body for a public rigid-body fixed joint.
  ///
  /// @return Parent RigidBody handle
  /// @throws InvalidArgumentException if this joint does not connect rigid
  /// bodies.
  [[nodiscard]] RigidBody getParentRigidBody() const;

  /// Get the child rigid body for a public rigid-body fixed joint.
  ///
  /// @return Child RigidBody handle
  /// @throws InvalidArgumentException if this joint does not connect rigid
  /// bodies.
  [[nodiscard]] RigidBody getChildRigidBody() const;

  /// Get the backend-neutral identity token for this joint.
  [[nodiscard]] Entity getEntity() const;

  /// Check if this handle is valid (entity still exists)
  ///
  /// @return True if the entity is valid
  [[nodiscard]] bool isValid() const;

  // TODO: Add methods for:
  // - Getting/setting joint limits
  // - Getting/setting effort limits
  // - Computing joint transforms

private:
  Entity m_entity; ///< Opaque entity token
  World* m_world;  ///< Non-owning pointer to World
};

} // namespace dart::simulation
