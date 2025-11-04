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

#include <dart7/frame/frame.hpp>
#include <dart7/multi_body/joint.hpp>

#include <entt/entt.hpp>

#include <string>
#include <string_view>

namespace dart7 {

/// Link handle class
///
/// Represents a rigid link in a MultiBody kinematic tree. This is a lightweight
/// handle to the underlying entity in the ECS registry.
///
/// Links are rigid bodies connected by joints in an articulated system.
/// Each link (except the root) has exactly one parent joint connecting it
/// to its parent link.
///
/// ## Frame Interface
///
/// Link inherits from Frame, providing full access to frame-related queries:
/// \code
///   auto base = robot.addLink("base");
///   auto ee = robot.addLink("end_effector");
///
///   // Use Frame methods directly on Link
///   auto T_world_ee = ee.getWorldTransform();
///   auto T_base_ee = ee.getTransform(base);
/// \endcode
///
/// Usage:
/// \code
///   auto root = robot.addLink("base");
///   auto link1 = robot.addLink("link1", {
///       .parentLink = root.getEntity(),
///       .jointName = "shoulder",
///       .jointType = JointType::REVOLUTE
///   });
///   std::cout << link1.getName() << std::endl;
/// \endcode
///
/// @note Link handles are lightweight (entity ID + pointer) and safe to copy.
///       Handles become invalid if the underlying entity is destroyed
///       or the World is destroyed.
///
/// @see Frame
class Link : public Frame
{
public:
  /// Constructor (typically called by MultiBody::addLink)
  ///
  /// @param entity The entity ID in the registry
  /// @param world Pointer to the World owning this entity
  Link(entt::entity entity, World* world);

  /// Get the name of this link
  ///
  /// @return The link name
  [[nodiscard]] std::string_view getName() const;

  /// Get the parent joint handle (invalid handle if root link)
  ///
  /// @return Parent Joint handle (check with isValid() for root links)
  [[nodiscard]] Joint getParentJoint() const;

  //--------------------------------------------------------------------------
  // Frame interface overrides
  //--------------------------------------------------------------------------

  /// Get the local transform of this Link with respect to its parent
  ///
  /// Returns the transform from the parent joint frame to this link frame.
  /// This is stored in the Link component as transformFromParentJoint.
  ///
  /// @return Local transform relative to parent joint
  [[nodiscard]] const Eigen::Isometry3d& getLocalTransform() const override;

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

} // namespace dart7
