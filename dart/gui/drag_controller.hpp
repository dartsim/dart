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

#ifndef DART_GUI_DRAG_CONTROLLER_HPP_
#define DART_GUI_DRAG_CONTROLLER_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/input_event.hpp>
#include <dart/gui/scene.hpp>
#include <dart/gui/scene_extractor.hpp>

#include <dart/dynamics/fwd.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <vector>

namespace dart {
namespace gui {

/// Backend-neutral interface for objects that can be dragged
class DART_GUI_API Draggable
{
public:
  virtual ~Draggable() = default;

  /// Return true if this draggable handles the given hit
  virtual bool canDrag(const HitResult& hit) const = 0;

  /// Called when drag begins
  virtual void beginDrag(const HitResult& hit) = 0;

  /// Called each frame while dragging; delta is in world space
  virtual void updateDrag(
      const Eigen::Vector3d& delta, const ModifierKeys& mods)
      = 0;

  /// Called when drag ends
  virtual void endDrag() = 0;
};

/// Constraint types for drag operations
enum class ConstraintType
{
  Unconstrained,
  Line,
  Plane
};

/// Backend-neutral drag controller that operates on Scene + HitResult
class DART_GUI_API DragController
{
public:
  DragController() = default;

  /// Register a draggable
  void addDraggable(std::unique_ptr<Draggable> draggable);

  /// Remove all draggables
  void clearDraggables();

  /// Compute constrained cursor delta in world space.
  ///
  /// Projects mouse movement onto a world-space line or plane through
  /// \p fromPosition, or onto a camera-parallel plane if unconstrained.
  ///
  /// @param fromPosition  Reference point in world space
  /// @param constraint    Type of constraint to apply
  /// @param constraintVector  Line direction or plane normal
  /// @param camera        Current camera configuration
  /// @param screenDx      Screen-space delta X (pixels)
  /// @param screenDy      Screen-space delta Y (pixels)
  /// @param screenWidth   Viewport width in pixels
  /// @param screenHeight  Viewport height in pixels
  /// @return World-space delta vector
  static Eigen::Vector3d getDeltaCursor(
      const Eigen::Vector3d& fromPosition,
      ConstraintType constraint,
      const Eigen::Vector3d& constraintVector,
      const Camera& camera,
      double screenDx,
      double screenDy,
      double screenWidth,
      double screenHeight);

  /// Handle a mouse press event. Returns true if a drag started.
  bool handleMousePress(
      const HitResult& hit,
      const std::unordered_map<uint64_t, EntityInfo>& entityMap);

  /// Handle mouse movement while dragging
  void handleMouseDrag(
      double screenDx,
      double screenDy,
      const Camera& camera,
      double screenWidth,
      double screenHeight,
      const ModifierKeys& mods);

  /// Handle mouse release
  void handleMouseRelease();

  /// Query whether a drag is in progress
  bool isDragging() const;

  /// Get the currently active draggable (nullptr if not dragging)
  Draggable* activeDraggable() const;

private:
  std::vector<std::unique_ptr<Draggable>> draggables_;
  Draggable* active_ = nullptr;
  bool dragging_ = false;
  Eigen::Vector3d drag_origin_ = Eigen::Vector3d::Zero();
};

class DART_GUI_API SimpleFrameDraggable : public Draggable
{
public:
  SimpleFrameDraggable(
      dart::dynamics::SimpleFrame* frame, uint64_t markerNodeId);

  bool canDrag(const HitResult& hit) const override;
  void beginDrag(const HitResult& hit) override;
  void updateDrag(
      const Eigen::Vector3d& delta, const ModifierKeys& mods) override;
  void endDrag() override;

  dart::dynamics::SimpleFrame* frame() const
  {
    return frame_;
  }

private:
  dart::dynamics::SimpleFrame* frame_;
  uint64_t marker_node_id_;
  Eigen::Isometry3d saved_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d pick_offset_ = Eigen::Vector3d::Zero();
  Eigen::AngleAxisd saved_rotation_ = Eigen::AngleAxisd::Identity();
};

class DART_GUI_API BodyNodeDraggable : public Draggable
{
public:
  BodyNodeDraggable(
      dart::dynamics::BodyNode* bodyNode,
      const std::unordered_map<uint64_t, EntityInfo>& entityMap,
      bool useExternalIK = true,
      bool useWholeBody = false);

  bool canDrag(const HitResult& hit) const override;
  void beginDrag(const HitResult& hit) override;
  void updateDrag(
      const Eigen::Vector3d& delta, const ModifierKeys& mods) override;
  void endDrag() override;

  dart::dynamics::BodyNode* bodyNode() const
  {
    return body_node_;
  }

private:
  dart::dynamics::BodyNode* body_node_;
  const std::unordered_map<uint64_t, EntityInfo>& entity_map_;
  bool use_external_ik_;
  bool use_whole_body_;
  dart::dynamics::InverseKinematicsPtr ik_;
  Eigen::Isometry3d saved_target_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d saved_local_offset_ = Eigen::Vector3d::Zero();
  Eigen::AngleAxisd saved_rotation_ = Eigen::AngleAxisd::Identity();
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_DRAG_CONTROLLER_HPP_
