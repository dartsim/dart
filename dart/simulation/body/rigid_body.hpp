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

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/entity.hpp>
#include <dart/simulation/frame/frame.hpp>
#include <dart/simulation/fwd.hpp>

#include <optional>
#include <string>
#include <vector>

namespace dart::simulation {

/// Family policy for a rigid body participating in deformable contact.
///
/// This policy describes how a rigid body is interpreted by deformable-body
/// contact and sweep limiting without exposing solver implementation toggles
/// as separate shared-handle booleans.
struct DeformableObstaclePolicy
{
  /// Treat the body as a one-sided z-up ground barrier.
  bool groundBarrier = false;

  /// Treat the body's surface as an obstacle for deformable sweep limiting.
  bool surfaceObstacle = false;

  /// Keep barrier contact while excluding this obstacle from sweep limiting.
  bool barrierOnly = false;
};

//==============================================================================
/// RigidBody handle class
///
/// Represents a single rigid object in the world. This is a lightweight handle
/// to World-owned simulation state.
///
/// RigidBody inherits from Frame, which means it can participate in all
/// frame-related operations such as transform queries, velocity/acceleration
/// computations (future), and can be used as a reference frame.
///
/// The initial public surface exposes name, transform, world-frame velocity,
/// inertial state, and force/torque accumulators. Future enhancements will add:
/// - Collision shapes
/// - Collision detection integration
///
/// @note RigidBody objects are owned by World and accessed via handles.
class DART_SIMULATION_API RigidBody : public Frame
{
public:
  /// Constructor (typically called by World::addRigidBody)
  RigidBody(Entity entity, World* world);

  /// Get the name of the rigid body
  [[nodiscard]] std::string getName() const;

  /// Set the rigid body's world transform.
  ///
  /// This updates the public frame transform and the internal rigid-body
  /// dynamics pose so kinematics-only reads and subsequent physics steps start
  /// from the same pose.
  void setTransform(const Eigen::Isometry3d& transform);

  /// Get the body's world-frame linear velocity.
  [[nodiscard]] Eigen::Vector3d getLinearVelocity() const;

  /// Set the body's world-frame linear velocity.
  ///
  /// The value must contain only finite coordinates. Subsequent physics steps
  /// use this velocity directly.
  void setLinearVelocity(const Eigen::Vector3d& velocity);

  /// Get the body's world-frame angular velocity.
  [[nodiscard]] Eigen::Vector3d getAngularVelocity() const;

  /// Set the body's world-frame angular velocity.
  ///
  /// The value must contain only finite coordinates. Subsequent physics steps
  /// use this velocity directly.
  void setAngularVelocity(const Eigen::Vector3d& velocity);

  /// Get the body's mass.
  [[nodiscard]] double getMass() const;

  /// Set the body's mass.
  ///
  /// The value must be positive and finite. Subsequent physics steps use this
  /// mass directly.
  void setMass(double mass);

  /// Get the body's body-frame inertia tensor.
  [[nodiscard]] Eigen::Matrix3d getInertia() const;

  /// Set the body's body-frame inertia tensor.
  ///
  /// The value must be finite, symmetric, and positive definite. Subsequent
  /// physics steps use this inertia directly.
  void setInertia(const Eigen::Matrix3d& inertia);

  /// Get the body's accumulated world-frame force.
  [[nodiscard]] Eigen::Vector3d getForce() const;

  /// Set the body's accumulated world-frame force.
  ///
  /// The value must contain only finite coordinates. The force is a persistent
  /// applied load: physics steps read it into a transient force buffer without
  /// clearing it, so callers clear or update it explicitly.
  void setForce(const Eigen::Vector3d& force);

  /// Add to the body's accumulated world-frame force.
  ///
  /// The value must contain only finite coordinates.
  void applyForce(const Eigen::Vector3d& force);

  /// Clear the body's accumulated force.
  void clearForce();

  /// Get the body's accumulated world-frame torque.
  [[nodiscard]] Eigen::Vector3d getTorque() const;

  /// Set the body's accumulated world-frame torque.
  ///
  /// The value must contain only finite coordinates. The torque is a persistent
  /// applied load: physics steps read it into a transient force buffer without
  /// clearing it, so callers clear or update it explicitly.
  void setTorque(const Eigen::Vector3d& torque);

  /// Add to the body's accumulated world-frame torque.
  ///
  /// The value must contain only finite coordinates.
  void applyTorque(const Eigen::Vector3d& torque);

  /// Clear the body's accumulated torque.
  void clearTorque();

  /// Apply an instantaneous world-frame linear impulse.
  ///
  /// Dynamic bodies receive `impulse / mass` as an immediate linear-velocity
  /// change. Static and kinematic bodies ignore impulses because they represent
  /// infinite-mass or prescribed-motion states in the public rigid-body model.
  void applyLinearImpulse(const Eigen::Vector3d& impulse);

  /// Apply an instantaneous world-frame angular impulse about the center of
  /// mass.
  ///
  /// Dynamic bodies receive `I_world^-1 * impulse` as an immediate
  /// angular-velocity change. Static and kinematic bodies ignore impulses.
  void applyAngularImpulse(const Eigen::Vector3d& impulse);

  //--------------------------------------------------------------------------
  // Derived dynamic quantities
  //
  // These assume the body's center of mass is at the body frame origin, which
  // matches the experimental rigid-body integration model.
  //--------------------------------------------------------------------------

  /// Get the body's world-frame linear momentum (mass * linear velocity).
  [[nodiscard]] Eigen::Vector3d getLinearMomentum() const;

  /// Get the body's world-frame angular momentum about its center of mass.
  [[nodiscard]] Eigen::Vector3d getAngularMomentum() const;

  /// Get the body's kinetic energy (translational + rotational).
  [[nodiscard]] double getKineticEnergy() const;

  /// Get the body's gravitational potential energy for the world's gravity.
  ///
  /// Defined as `-mass * gravity . position`, so the value increases as the
  /// body moves against gravity. With the default gravity `(0, 0, -9.81)` this
  /// equals `mass * 9.81 * z`.
  [[nodiscard]] double getPotentialEnergy() const;

  //--------------------------------------------------------------------------
  // Collision geometry
  //--------------------------------------------------------------------------

  /// Set whether the body is static (immovable).
  ///
  /// A static body ignores gravity and applied forces, is not integrated, and
  /// is treated as infinite mass by the contact solver.
  void setStatic(bool isStatic);

  /// Return whether the body is static.
  [[nodiscard]] bool isStatic() const;

  /// Set whether the body is kinematic (prescribed motion).
  ///
  /// A kinematic body is advanced by its prescribed linear/angular velocity
  /// each step and acts as a moving support/driver: it receives no contact or
  /// dynamics response and is unaffected by gravity, but its moving surface
  /// drags contacting dynamic bodies through friction (a conveyor or a
  /// turntable) and its swept motion is honored by the conservative CCD so
  /// resting dynamic bodies stay intersection-free. Honored by the rigid IPC
  /// contact solver (World::RigidBodySolver::Ipc); under the default
  /// sequential-impulse solver a kinematic body behaves like a static one.
  /// Setting a body kinematic clears its static flag.
  ///
  /// Supported motion is tangential/co-moving relative to the bodies it touches
  /// (drag). A kinematic body prescribed to move *normally into* a dynamic body
  /// faster than the barrier can push it aside is not guaranteed
  /// intersection-free, since prescribed motion cannot be slowed by contact;
  /// robust normal pushing is future work.
  void setKinematic(bool isKinematic);

  /// Return whether the body is kinematic.
  [[nodiscard]] bool isKinematic() const;

  /// Return whether this body is currently asleep under World deactivation.
  ///
  /// Returns false when deactivation is disabled, unsupported for the active
  /// world configuration, or the body has not entered sleep.
  [[nodiscard]] bool isSleeping() const;

  /// Return this body's current deactivation group index, or -1 when none.
  [[nodiscard]] int getDeactivationGroupIndex() const;

  /// Set the body's restitution (bounciness) coefficient in [0, 1].
  ///
  /// The contact solver combines two bodies' restitution by taking the maximum.
  /// Zero (the default) is fully inelastic.
  void setRestitution(double restitution);

  /// Get the body's restitution coefficient (default 0).
  [[nodiscard]] double getRestitution() const;

  /// Set the body's contact friction coefficient (non-negative).
  ///
  /// The contact solver combines two bodies' friction as the geometric mean.
  /// Defaults to 1.
  void setFriction(double friction);

  /// Get the body's contact friction coefficient (default 1).
  [[nodiscard]] double getFriction() const;

  /// Set (or replace) this body's collision shape.
  ///
  /// The shape is centered at the body frame origin and is used by
  /// `World::collide()` collision queries. Dimensions must be positive.
  void setCollisionShape(const CollisionShape& shape);

  /// Add a collision shape without replacing existing shapes.
  ///
  /// Multiple shapes attached to one body are treated as a compound collision
  /// geometry by `World::collide()`. Shapes on the same body do not collide
  /// with each other.
  void addCollisionShape(const CollisionShape& shape);

  /// Set this body's deformable-contact obstacle policy.
  ///
  /// This does not change rigid-body collision behavior. Ground barriers are
  /// one-sided z-up barriers; surface obstacles contribute a triangulated
  /// static surface for deformable sweep limiting; barrier-only obstacles keep
  /// barrier contact while skipping sweep limiting.
  void setDeformableObstaclePolicy(const DeformableObstaclePolicy& policy);

  /// Return this body's deformable-contact obstacle policy.
  [[nodiscard]] DeformableObstaclePolicy getDeformableObstaclePolicy() const;

  /// Get this body's collision shape, if one is attached.
  ///
  /// For compound collision geometry this returns the first attached shape.
  [[nodiscard]] std::optional<CollisionShape> getCollisionShape() const;

  /// Get all collision shapes attached to this body.
  [[nodiscard]] std::vector<CollisionShape> getCollisionShapes() const;

  /// Return whether this body has a collision shape attached.
  [[nodiscard]] bool hasCollisionShape() const;

  // Note: getEntity(), getWorld(), isValid() inherited from Frame

  // TODO: Add methods for:
  // - Accessing collision shapes
  // - Enabling/disabling physics
};

} // namespace dart::simulation
