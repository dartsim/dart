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

#include <dart7/comps/frame_types.hpp>
#include <dart7/ecs/entity_object.hpp>
#include <dart7/fwd.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

namespace dart7 {

/// Frame represents a spatial reference frame in the kinematic tree.
///
/// A Frame is anything with a spatial location and orientation that can
/// participate in kinematic queries such as transform, velocity, and
/// acceleration computations. Frames form a hierarchical tree structure.
///
/// This is a lightweight handle class that wraps an entt::entity with
/// convenient methods for frame-related operations. It's inspired by DART 6's
/// Frame class but adapted for the ECS architecture.
///
/// ## What is a Frame?
///
/// A Frame can be:
/// - **Link**: Most common - rigid bodies in articulated systems
/// - **Joint Frame**: Origin/axis of a joint (future)
/// - **SimpleFrame**: User-defined frame with fixed offset from parent
/// (future)
/// - **ShapeFrame**: Frame attached to collision/visual shapes (future)
/// - **World Frame**: Root of all frame hierarchies
///
/// ## Usage Examples
///
/// @code
/// // Get transforms
/// auto world_T_ee = endEffector.getWorldTransform();
/// auto base_T_ee = endEffector.getTransform(baseLink);
///
/// // Express in different coordinates
/// auto T = endEffector.getTransform(someLink, Frame::world());
///
/// // Velocities (future - Phase 3)
/// auto v = endEffector.getSpatialVelocity(baseLink, Frame::world());
/// @endcode
///
/// ## Implementation Notes
///
/// Phase 1 (Current): Basic transform queries
/// - getWorldTransform()
/// - getTransform(relativeTo)
/// - getTransform(relativeTo, expressedIn)
///
/// Phase 2: Velocity/Acceleration (Future - after dynamics)
/// Phase 3: Additional frame types (SimpleFrame, ShapeFrame)
///
/// @see LinkComponent, FrameTag
class Frame : public EntityObjectWith<
                  TagComps<comps::FrameTag>,
                  ReadOnlyComps<>,
                  WriteOnlyComps<>,
                  ReadWriteComps<comps::FrameState, comps::FrameCache>>
{
public:
  // Allow internal classes to access getEntity() for ECS operations
  friend class World;
  friend class FreeFrame;
  friend class FixedFrame;
  friend class MultiBody;

  /// Construct a Frame handle from an entity and world
  ///
  /// @param entity Entity ID in the ECS registry
  /// @param world Pointer to the owning World instance
  ///
  /// @note The entity must have FrameTag and LinkComponent (for Phase 1)
  Frame(entt::entity entity, World* world);

  /// Virtual destructor
  virtual ~Frame() = default;

  //--------------------------------------------------------------------------
  // Transform Queries
  //--------------------------------------------------------------------------

  /// Get the world transform of this Frame
  ///
  /// Returns the pose (position + orientation) of this Frame expressed in
  /// the world coordinate system. This is the absolute pose.
  ///
  /// @return Isometry transformation from world frame to this frame
  ///
  /// @pre Forward kinematics must have been computed
  ///      (World::updateKinematics() must have been called)
  ///
  /// Get the local transform of this Frame with respect to its parent Frame
  ///
  /// Returns the transform of this frame relative to its parent frame (local).
  /// Default implementation returns Identity (for world frame).
  /// Derived classes override this to provide frame-type-specific behavior.
  ///
  /// @return Local transform relative to parent frame
  [[nodiscard]] virtual const Eigen::Isometry3d& getLocalTransform() const;

  /// Get the parent frame of this Frame
  ///
  /// This method checks the entity's components to determine the frame type
  /// and returns the appropriate parent frame:
  /// - FreeFrame: returns parent from FreeFrameProperties component
  /// - FixedFrame: returns parent from FixedFrameProperties component
  /// - Link: returns parent from Link component
  /// - World frame: returns itself
  ///
  /// @return Parent frame handle
  ///
  /// @note This is NOT virtual - it dispatches based on ECS components.
  ///       This is more scalable than OOP inheritance for the ECS pattern.
  [[nodiscard]] Frame getParentFrame() const;

  /// Set the parent frame (for FreeFrame and FixedFrame)
  ///
  /// Changes the parent frame and invalidates the world transform cache.
  /// This dispatches to the correct component (FreeFrameProperties or
  /// FixedFrameProperties) based on the entity's type.
  ///
  /// @param parent New parent frame
  /// @throws InvalidOperationException if called on Link (Links cannot change
  /// parent)
  void setParentFrame(const Frame& parent);

  // TODO: Add syntactic sugar for creating child frames directly from Frame
  //       Once implementation is stable, add convenience methods like:
  //         frame.addFixedFrame("child", offset)
  //         frame.addFreeFrame("child")
  //       instead of always requiring:
  //         world.addFixedFrame("child", frame, offset)
  //       This would provide more intuitive parent-child API similar to DART 6.

  /// Get the transform of this frame (in world coordinates)
  ///
  /// This is the most commonly used transform query - it returns the pose
  /// of this frame expressed in world coordinates.
  ///
  /// Uses lazy evaluation: computes world transform only when dirty by
  /// calling: worldTransform = parent.getTransform() * getLocalTransform()
  ///
  /// Following DART 6's pattern exactly:
  /// - Checks if world frame (returns Identity)
  /// - Checks dirty flag (needTransformUpdate)
  /// - If dirty, recursively computes via parent chain
  /// - Returns cached worldTransform
  ///
  /// @return World-frame transformation
  [[nodiscard]] const Eigen::Isometry3d& getTransform() const;

  /// Convenience accessors for common decomposition queries
  [[nodiscard]] Eigen::Vector3d getTranslation() const;
  [[nodiscard]] Eigen::Matrix3d getRotation() const;
  [[nodiscard]] Eigen::Quaterniond getQuaternion() const;
  [[nodiscard]] Eigen::Matrix4d getTransformMatrix() const;

  /// Get transform of this Frame relative to another Frame
  ///
  /// Computes the relative transformation from @p relativeTo frame to this
  /// frame, expressed in world coordinates.
  ///
  /// Mathematical definition:
  /// @code
  /// T_relativeTo_this = T_world_relativeTo^{-1} * T_world_this
  /// @endcode
  ///
  /// @param relativeTo The reference frame for relative transform
  /// @return Transformation from 'relativeTo' frame to this frame
  ///
  /// @pre Both frames must be valid and from the same World instance
  ///
  /// Examples:
  /// @code
  /// // Transform from base to end effector (in world coords)
  /// auto T = endEffector.getTransform(base);
  /// @endcode
  [[nodiscard]] Eigen::Isometry3d getTransform(const Frame& relativeTo) const;

  /// Get transform relative to another Frame, expressed in a third Frame's
  /// coordinates
  ///
  /// This is the most general form of transform query. It computes the
  /// relative transform and then expresses it in a desired coordinate system.
  ///
  /// Mathematical definition:
  /// @code
  /// T_relativeTo_this = T_world_relativeTo^{-1} * T_world_this  // Relative
  /// transform R_coord = rotation of expressedIn frame result.rotation =
  /// R_coord^T * T_relativeTo_this.rotation result.translation = R_coord^T *
  /// T_relativeTo_this.translation
  /// @endcode
  ///
  /// @param relativeTo The reference frame
  /// @param expressedIn Frame whose coordinate system to express result in
  /// @return Transformation from 'to' to this, in expressedIn coords
  ///
  /// @pre All three frames must be valid and from the same World instance
  ///
  /// Examples:
  /// @code
  /// // Transform from link1 to link2, expressed in link1's coordinates
  /// auto T = link2.getTransform(link1, link1);
  ///
  /// // Same but expressed in world coordinates (default behavior)
  /// auto T = link2.getTransform(link1, Frame::world());
  /// @endcode
  [[nodiscard]] Eigen::Isometry3d getTransform(
      const Frame& to, const Frame& expressedIn) const;

  //--------------------------------------------------------------------------
  // Static World Frame
  //--------------------------------------------------------------------------

  /// Get the world frame (root of frame hierarchy)
  ///
  /// Returns a Frame representing the world coordinate system. This is the
  /// ultimate parent of all frames in the scene.
  ///
  /// @return Frame handle for the world frame
  ///
  /// @note Each World instance has its own world frame entity
  ///
  /// Example:
  /// @code
  /// auto T = endEffector.getTransform(Frame::world());
  /// @endcode
  [[nodiscard]] static Frame world();

  //--------------------------------------------------------------------------
  // Validity and Internal Access
  //--------------------------------------------------------------------------

  /// Check if this Frame handle is valid
  ///
  /// A Frame is valid if:
  /// - It has a non-null entity ID
  /// - It has a non-null World pointer
  /// - The entity exists in the World's registry
  /// - The entity has FrameTag
  ///
  /// @return true if valid, false otherwise
  [[nodiscard]] bool isValid() const;

  /// Check if this is the world frame
  ///
  /// @return true if this is the world frame, false otherwise
  [[nodiscard]] bool isWorld() const
  {
    return m_entity == entt::null;
  }

  /// Check if two frames refer to the same entity
  ///
  /// This checks if two Frame handles point to the exact same entity
  /// in the same World instance.
  ///
  /// @param other Another Frame to compare with
  /// @return true if both frames refer to the same entity
  [[nodiscard]] bool isSameInstanceAs(const Frame& other) const
  {
    return m_entity == other.m_entity && m_world == other.m_world;
  }

  /// Get the owning World instance
  ///
  /// @return Pointer to World instance
  [[nodiscard]] World* getWorld() const
  {
    return m_world;
  }

  //--------------------------------------------------------------------------
  // Comparison Operators
  //--------------------------------------------------------------------------

  /// Equality comparison
  ///
  /// Two Frames are equal if they refer to the same entity in the same World.
  ///
  /// @param other Frame to compare with
  /// @return true if equal, false otherwise
  bool operator==(const Frame& other) const
  {
    return m_entity == other.m_entity && m_world == other.m_world;
  }

  /// Inequality comparison
  bool operator!=(const Frame& other) const
  {
    return !(*this == other);
  }

  /// Get the underlying entity ID
  ///
  /// This is primarily for testing and internal use. External users should
  /// generally use Frame methods rather than accessing the entity directly.
  ///
  /// @return Entity ID in the ECS registry
  [[nodiscard]] entt::entity getEntity() const
  {
    return m_entity;
  }

  // Note: m_entity and m_world are inherited from EntityObject<Frame>
};

} // namespace dart7
