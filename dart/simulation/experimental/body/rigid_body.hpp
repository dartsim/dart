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

#include <dart/simulation/experimental/fwd.hpp>

#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/frame/frame.hpp>
#include <dart/simulation/experimental/shape/shape_node.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <string>

namespace dart::simulation::experimental {

//==============================================================================
/// RigidBody handle class
///
/// Represents a single rigid object in the world. This is a lightweight handle
/// to the underlying entity in the ECS registry.
///
/// RigidBody inherits from Frame, which means it can participate in all
/// frame-related operations such as transform queries, velocity/acceleration
/// computations (future), and can be used as a reference frame.
///
/// PLACEHOLDER: This is a minimal implementation to demonstrate World's
/// ability to manage multiple object types (Multibody and RigidBody).
/// Future enhancements will add:
/// - Physics properties (mass, inertia, collision shapes)
/// - State access (pose, velocity)
/// - Force application
/// - Collision detection integration
///
/// @note Handles are lightweight (entity ID + pointer) and safe to copy.
///       Handles become invalid if the underlying entity is destroyed
///       or the World is destroyed.
class DART_EXPERIMENTAL_API RigidBody : public Frame
{
public:
  /// Constructor (typically called by World::addRigidBody)
  RigidBody(entt::entity entity, World* world);

  /// Get the name of the rigid body
  [[nodiscard]] std::string getName() const;

  //--------------------------------------------------------------------------
  // Frame interface overrides
  //--------------------------------------------------------------------------

  /// Get the local transform of this rigid body
  ///
  /// Returns the transform from the parent frame to this rigid body frame.
  [[nodiscard]] const Eigen::Isometry3d& getLocalTransform() const override;

  // Note: getEntity(), getWorld(), isValid() inherited from Frame

  /// Get the mass of the rigid body
  [[nodiscard]] double getMass() const;

  /// Set the mass of the rigid body
  void setMass(double mass);

  /// Get the inertia matrix (3x3)
  [[nodiscard]] Eigen::Matrix3d getInertia() const;

  /// Set the inertia matrix (3x3)
  void setInertia(const Eigen::Matrix3d& inertia);

  /// Get the position (world frame)
  [[nodiscard]] Eigen::Vector3d getPosition() const;

  /// Set the position (world frame)
  void setPosition(const Eigen::Vector3d& position);

  /// Get the orientation as quaternion
  [[nodiscard]] Eigen::Quaterniond getOrientation() const;

  /// Set the orientation from quaternion
  void setOrientation(const Eigen::Quaterniond& orientation);

  /// Get the linear velocity (world frame)
  [[nodiscard]] Eigen::Vector3d getLinearVelocity() const;

  /// Set the linear velocity (world frame)
  void setLinearVelocity(const Eigen::Vector3d& velocity);

  /// Get the angular velocity (world frame)
  [[nodiscard]] Eigen::Vector3d getAngularVelocity() const;

  /// Set the angular velocity (world frame)
  void setAngularVelocity(const Eigen::Vector3d& velocity);

  /// Get the accumulated force
  [[nodiscard]] Eigen::Vector3d getForce() const;

  /// Add force (accumulates until cleared)
  void addForce(const Eigen::Vector3d& force);

  /// Get the accumulated torque
  [[nodiscard]] Eigen::Vector3d getTorque() const;

  /// Add torque (accumulates until cleared)
  void addTorque(const Eigen::Vector3d& torque);

  /// Clear accumulated forces and torques
  void clearForces();

  /// Create a ShapeNode attached to this RigidBody
  ///
  /// @param shape Collision geometry to attach
  /// @param name Optional name for the shape
  /// @param options Shape configuration options
  ShapeNode createShapeNode(
      const dart::dynamics::ShapePtr& shape,
      std::string_view name = "",
      const ShapeNodeOptions& options = ShapeNodeOptions{});
};

} // namespace dart::simulation::experimental
