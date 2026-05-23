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

#include <dart/simulation/experimental/multi_body/link.hpp> // Need complete type for LinkOptions

#include <Eigen/Core>
#include <entt/entt.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental {

/// Joint construction value object.
///
/// JointSpec describes the public joint configuration used when connecting a
/// child link to a parent link. It intentionally contains joint concepts only;
/// the parent link remains part of the link-construction call so the same
/// joint spec can be reused for multiple construction sites.
///
/// Usage:
/// @code
///   auto base = robot.addLink("base");
///   auto forearm = robot.addLink("forearm", base, JointSpec{
///       .name = "elbow",
///       .type = JointType::Revolute,
///       .axis = Eigen::Vector3d::UnitZ(),
///   });
/// @endcode
struct JointSpec
{
  std::string name;                     ///< Name of connecting joint
  JointType type = JointType::Revolute; ///< Type of joint
  Eigen::Vector3d axis
      = Eigen::Vector3d::UnitZ(); ///< Joint axis (rotation or translation)

  // Future: Add more joint-specific parameters.
  // Eigen::Vector3d axis2;  // For Universal, Planar
  // double pitch;           // For Screw
};

/// Options for creating a link with parent joint
///
/// Specifies how a link connects to its parent via a joint.
/// Used for child links (not root links).
///
/// Usage:
/// @code
///   auto root = robot.addLink("base");  // Root link (no options)
///
///   // Revolute joint
///   auto link1 = robot.addLink("link1", {
///       .parentLink = root,
///       .jointName = "shoulder",
///       .jointType = JointType::Revolute,
///       .axis = {0, 0, 1}
///   });
///
///   // Prismatic joint
///   auto link2 = robot.addLink("link2", {
///       .parentLink = link1,
///       .jointName = "slider",
///       .jointType = JointType::Prismatic,
///       .axis = {1, 0, 0}
///   });
/// @endcode
struct LinkOptions
{
  Link parentLink;                           ///< Parent link handle
  std::string jointName;                     ///< Name of connecting joint
  JointType jointType = JointType::Revolute; ///< Type of joint
  Eigen::Vector3d axis
      = Eigen::Vector3d::UnitZ(); ///< Joint axis (rotation or translation)

  // Future: Add more joint-specific parameters
  // Eigen::Vector3d axis2;  // For Universal, Planar
  // double pitch;           // For Screw
};

/// MultiBody represents an articulated rigid body system
///
/// This is a lightweight handle class that references entity data stored
/// in the World's entt::registry. It provides convenient API to access
/// and modify MultiBody properties without owning the data.
///
/// A MultiBody consists of multiple rigid Links connected by Joints.
/// This class is suitable for representing:
/// - Robotic manipulators (e.g., UR5, Franka Emika Panda)
/// - Humanoid characters (e.g., bipeds, quadrupeds)
/// - Mechanical systems (e.g., furniture, mechanisms)
///
/// The term "MultiBody" is used for consistency with other classes in
/// this library (e.g., RigidBody, SoftBody, FixedFrame, FreeFrame).
///
/// @note Handles are lightweight (entity ID + pointer) and safe to copy.
///       Handles become invalid if the underlying entity is destroyed
///       or the World is destroyed.
class DART_EXPERIMENTAL_API MultiBody
{
public:
  /// Construct a MultiBody handle
  ///
  /// @param entity The entity ID in the registry
  /// @param world Pointer to the World owning this entity
  MultiBody(entt::entity entity, World* world);

  /// Get the name of this MultiBody
  ///
  /// @return The name
  std::string_view getName() const;

  /// Set the name of this MultiBody
  ///
  /// @param name New name
  void setName(std::string_view name);

  /// Get the number of links in this MultiBody
  ///
  /// @return Number of links
  std::size_t getLinkCount() const;

  /// Get the number of joints in this MultiBody
  ///
  /// @return Number of joints
  std::size_t getJointCount() const;

  /// Get the total number of degrees of freedom
  ///
  /// @return Total DOFs across all joints
  std::size_t getDOFCount() const;

  /// Get a link by name
  ///
  /// @param name Link name
  /// @return Link handle if found, std::nullopt otherwise
  std::optional<Link> getLink(std::string_view name) const;

  /// Get a joint by name
  ///
  /// @param name Joint name
  /// @return Joint handle if found, std::nullopt otherwise
  std::optional<Joint> getJoint(std::string_view name) const;

  /// Get all link handles in construction order.
  ///
  /// The returned vector is a snapshot of lightweight handles. The handles
  /// remain tied to this MultiBody's World and follow the same validity rules
  /// as handles returned by addLink() and getLink().
  [[nodiscard]] std::vector<Link> getLinks() const;

  /// Get all joint handles in construction order.
  ///
  /// The returned vector is a snapshot of lightweight handles. The handles
  /// remain tied to this MultiBody's World and follow the same validity rules
  /// as handles returned by getJoint().
  [[nodiscard]] std::vector<Joint> getJoints() const;

  /// Get link names in construction order.
  ///
  /// Duplicate names, if present, are preserved in the returned snapshot.
  [[nodiscard]] std::vector<std::string> getLinkNames() const;

  /// Get joint names in construction order.
  ///
  /// Duplicate names, if present, are preserved in the returned snapshot.
  [[nodiscard]] std::vector<std::string> getJointNames() const;

  /// Get the entity ID (for advanced users)
  ///
  /// @return The entity ID
  entt::entity getEntity() const;

  /// Get the World pointer (for advanced users)
  ///
  /// @return Pointer to the World
  World* getWorld() const;

  //--------------------------------------------------------------------------
  /// @name Kinematic Structure (Design-time only)
  //--------------------------------------------------------------------------

  /// Create a link (root link with no parent joint)
  ///
  /// Creates a root link without a parent joint. This is typically the
  /// base/root of your kinematic tree.
  ///
  /// @param name Link name (empty = auto-generate "link_NNN")
  /// @return Link handle
  /// @throws InvalidArgumentException if in simulation mode
  Link addLink(std::string_view name = "");

  /// Create a link with parent joint
  ///
  /// Creates a link and its parent joint atomically using LinkOptions.
  /// This maintains kinematic tree validity.
  ///
  /// @param name Link name (empty = auto-generate "link_NNN")
  /// @param options LinkOptions specifying parent and joint configuration
  /// @return Link handle (use getParentJoint() to access the created joint)
  /// @throws InvalidArgumentException if in simulation mode or parent invalid
  Link addLink(std::string_view name, const LinkOptions& options);

  /// Create a link with parent joint
  ///
  /// Creates a link and its parent joint atomically using a parent Link and
  /// JointSpec. This is the preferred DART 7 experimental construction shape
  /// because JointSpec can be shared by C++ and dartpy.
  ///
  /// @param name Link name (empty = auto-generate "link_NNN")
  /// @param parentLink Parent link handle
  /// @param joint JointSpec describing the parent joint
  /// @return Link handle (use getParentJoint() to access the created joint)
  /// @throws InvalidArgumentException if in simulation mode, parent invalid, or
  /// joint parameters invalid
  Link addLink(
      std::string_view name,
      const Link& parentLink,
      const JointSpec& joint = {});

private:
  entt::entity m_entity; ///< Entity ID in the registry
  World* m_world;        ///< Non-owning pointer to World
};

} // namespace dart::simulation::experimental
