/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/simulation/experimental/comps/joint.hpp> // For JointType

#include <Eigen/Core>
#include <entt/entt.hpp>

#include <string>
#include <string_view>

namespace dart::simulation::experimental {

/// Generic Joint handle class
///
/// Represents a joint connecting two links in a MultiBody. This is a
/// lightweight handle to the underlying entity in the ECS registry.
///
/// Unlike traditional OOP physics engines, the experimental stack uses a single
/// generic Joint class with ECS architecture. All joint type-specific data is
/// stored in JointComponent in the centralized entt::registry. The handle
/// provides a unified interface regardless of joint type (revolute, prismatic,
/// ball, etc.).
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
///       .jointType = JointType::REVOLUTE
///   });
///
///   Joint joint = link.getParentJoint();
///   joint.setPosition(0.5);
///   joint.setLimits(-M_PI, M_PI);
/// @endcode
///
/// @note Joint handles are lightweight (entity ID + pointer) and safe to copy.
///       Handles become invalid if the underlying entity is destroyed
///       or the World is destroyed.
class DART_EXPERIMENTAL_API Joint
{
public:
  /// Constructor (typically called by Link::getParentJoint)
  ///
  /// @param entity The entity ID in the registry
  /// @param world Pointer to the World owning this entity
  Joint(entt::entity entity, World* world);

  /// Get the name of this joint
  ///
  /// @return The joint name
  [[nodiscard]] std::string_view getName() const;

  /// Get the joint type
  ///
  /// @return JointType enum (REVOLUTE, PRISMATIC, etc.)
  [[nodiscard]] comps::JointType getType() const;

  /// Get the primary joint axis
  ///
  /// Valid for: Revolute, Prismatic, Screw, Universal, Planar
  /// @return 3D unit vector representing the primary axis
  /// @throws InvalidArgumentException if joint type doesn't use axis (Ball,
  /// Free)
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

  /// Get the parent link
  ///
  /// @return Parent Link handle
  [[nodiscard]] Link getParentLink() const;

  /// Get the child link
  ///
  /// @return Child Link handle
  [[nodiscard]] Link getChildLink() const;

  /// Get the underlying ECS entity handle
  ///
  /// @return The entity ID
  [[nodiscard]] entt::entity getEntity() const;

  /// Check if this handle is valid (entity still exists)
  ///
  /// @return True if the entity is valid
  [[nodiscard]] bool isValid() const;

  // TODO: Add methods for:
  // - Getting/setting position, velocity, acceleration
  // - Getting/setting joint limits
  // - Getting/setting effort limits
  // - Computing joint transforms

private:
  entt::entity m_entity; ///< Entity ID in the registry
  World* m_world;        ///< Non-owning pointer to World
};

} // namespace dart::simulation::experimental
