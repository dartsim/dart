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
#include <dart/simulation/entity.hpp>
#include <dart/simulation/frame/frame.hpp>
#include <dart/simulation/multibody/joint.hpp>

#include <Eigen/Core>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace dart::simulation {

/// Link handle class
///
/// Represents a rigid link in a Multibody kinematic tree. This is a lightweight
/// handle to World-owned articulation state.
///
/// Links are rigid bodies connected by joints in an articulated system.
/// Each link (except the root) has exactly one parent joint connecting it
/// to its parent link.
///
/// ## Frame Interface
///
/// Link inherits from Frame, providing full access to frame-related queries:
/// @code
///   auto base = robot.addLink("base");
///   auto ee = robot.addLink("end_effector");
///
///   // Use Frame methods directly on Link
///   auto T_world_ee = ee.getWorldTransform();
///   auto T_base_ee = ee.getTransform(base);
/// @endcode
///
/// Usage:
/// @code
///   auto root = robot.addLink("base");
///   auto link1 = robot.addLink("link1", {
///       .parentLink = root,
///       .jointName = "shoulder",
///       .jointType = JointType::Revolute
///   });
///   std::cout << link1.getName() << std::endl;
/// @endcode
///
/// @note Link handles are lightweight and safe to copy. Handles become invalid
///       if the referenced link or World is destroyed.
///
/// @see Frame
class DART_SIMULATION_API Link : public Frame
{
public:
  /// Constructor (typically called by Multibody::addLink)
  ///
  /// @param entity The opaque identity token.
  /// @param world Pointer to the owning World.
  Link(Entity entity, World* world);

  /// Get the name of this link
  ///
  /// @return The link name
  [[nodiscard]] std::string_view getName() const;

  /// Get the parent joint handle (invalid handle if root link)
  ///
  /// @return Parent Joint handle (check with isValid() for root links)
  [[nodiscard]] Joint getParentJoint() const;

  //--------------------------------------------------------------------------
  // Inertial properties
  //
  // The inertia tensor is expressed about the center of mass, which may be
  // offset from the link frame origin (see getCenterOfMass/setCenterOfMass; it
  // defaults to the origin).
  //--------------------------------------------------------------------------

  /// Get the link's mass.
  [[nodiscard]] double getMass() const;

  /// Set the link's mass.
  ///
  /// Must be positive and finite. Used by multibody integration.
  void setMass(double mass);

  /// Get the link's body-frame inertia tensor about its center of mass.
  [[nodiscard]] Eigen::Matrix3d getInertia() const;

  /// Set the link's body-frame inertia tensor about its center of mass.
  ///
  /// Must be finite, symmetric, and positive definite.
  void setInertia(const Eigen::Matrix3d& inertia);

  /// Get the link's center of mass expressed in the link frame (default: the
  /// origin).
  [[nodiscard]] Eigen::Vector3d getCenterOfMass() const;

  /// Set the link's center of mass expressed in the link frame.
  ///
  /// A nonzero offset places the center of mass away from the link frame
  /// origin; the inertia tensor (see setInertia) remains about the center of
  /// mass. Must contain only finite values.
  void setCenterOfMass(const Eigen::Vector3d& centerOfMass);

  //--------------------------------------------------------------------------
  // External forces
  //--------------------------------------------------------------------------

  /// Apply an external force to this link at an application point.
  ///
  /// The force accumulates into the link's external-force buffer and is
  /// consumed by the next `World::step()` under either integration family
  /// (semi-implicit forward dynamics or the variational integrator), then
  /// cleared. This is a one-shot, per-step force in the style of legacy
  /// `BodyNode::addExtForce`: it must be re-applied every step to keep acting.
  ///
  /// @param force The applied force vector.
  /// @param point The application point. Defaults to the link frame origin.
  /// @param forceInWorldFrame Whether `force` is expressed in the world frame
  ///        (true, default) or the link frame (false).
  /// @param pointInWorldFrame Whether `point` is expressed in the world frame
  ///        (true) or the link frame (false, default).
  void applyForce(
      const Eigen::Vector3d& force,
      const Eigen::Vector3d& point = Eigen::Vector3d::Zero(),
      bool forceInWorldFrame = true,
      bool pointInWorldFrame = false);

  //--------------------------------------------------------------------------
  // Collision geometry
  //--------------------------------------------------------------------------

  /// Get the collision shape attached to this link, if any.
  ///
  /// For compound collision geometry this returns the first attached shape.
  [[nodiscard]] std::optional<CollisionShape> getCollisionShape() const;

  /// Get all collision shapes attached to this link.
  [[nodiscard]] std::vector<CollisionShape> getCollisionShapes() const;

  /// Set (or replace) the collision shape attached to this link.
  ///
  /// Once a link has a collision shape it participates in `World::collide()`,
  /// posed by the link's forward-kinematics world transform.
  void setCollisionShape(const CollisionShape& shape);

  /// Add a collision shape without replacing existing shapes.
  ///
  /// Multiple shapes attached to one link are treated as a compound collision
  /// geometry by `World::collide()`. Shapes on the same link do not collide
  /// with each other.
  void addCollisionShape(const CollisionShape& shape);

  /// Return whether this link has a collision shape.
  [[nodiscard]] bool hasCollisionShape() const;

  //--------------------------------------------------------------------------
  // Frame interface overrides
  //--------------------------------------------------------------------------

  /// Get the world transform of this Link
  ///
  /// Unlike FreeFrame/FixedFrame which use lazy evaluation, Link's
  /// worldTransform is directly updated by the forward kinematics system. This
  /// method simply returns the cached value without any computation.
  ///
  /// @return World-frame transformation (updated by forward kinematics)
  [[nodiscard]] const Eigen::Isometry3d& getWorldTransform() const;

  // TODO: Add methods for:
  // - Accessing mass properties
  // - Getting child joints
  // - Accessing collision shapes

  // Note: getEntity(), getWorld(), isValid() inherited from Frame
};

} // namespace dart::simulation
