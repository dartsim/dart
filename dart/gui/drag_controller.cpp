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

#include "dart/gui/drag_controller.hpp"

#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/inverse_kinematics.hpp"
#include "dart/dynamics/simple_frame.hpp"

#include <cmath>

namespace dart {
namespace gui {

void DragController::addDraggable(std::unique_ptr<Draggable> draggable)
{
  if (draggable) {
    draggables_.push_back(std::move(draggable));
  }
}

void DragController::clearDraggables()
{
  if (dragging_ && active_) {
    active_->endDrag();
  }
  draggables_.clear();
  active_ = nullptr;
  dragging_ = false;
}

Eigen::Vector3d DragController::getDeltaCursor(
    const Eigen::Vector3d& fromPosition,
    ConstraintType constraint,
    const Eigen::Vector3d& constraintVector,
    const Camera& camera,
    double screenDx,
    double screenDy,
    double screenWidth,
    double screenHeight)
{
  // Build camera coordinate frame
  Eigen::Vector3d forward = (camera.target - camera.position);
  const double dist = forward.norm();
  if (dist < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  forward /= dist;
  Eigen::Vector3d right = forward.cross(camera.up);
  if (right.norm() < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  right.normalize();
  Eigen::Vector3d up = right.cross(forward);
  up.normalize();

  // Compute world-space displacement from screen-space delta.
  // Scale by distance from camera to reference point and FOV.
  const double distToPoint = (fromPosition - camera.position).norm();
  const double fovRad = camera.fovy * M_PI / 180.0;
  const double halfH = distToPoint * std::tan(fovRad * 0.5);
  const double halfW = halfH * (screenWidth / screenHeight);

  const double worldDx = (screenDx / screenWidth) * halfW * 2.0;
  const double worldDy = -(screenDy / screenHeight) * halfH * 2.0;

  Eigen::Vector3d delta = right * worldDx + up * worldDy;

  if (constraint == ConstraintType::Unconstrained) {
    return delta;
  }

  if (constraint == ConstraintType::Line) {
    // Project delta onto the constraint line direction
    Eigen::Vector3d dir = constraintVector;
    if (dir.norm() < 1e-12) {
      return Eigen::Vector3d::Zero();
    }
    dir.normalize();
    return dir * delta.dot(dir);
  }

  if (constraint == ConstraintType::Plane) {
    // Project delta onto the constraint plane
    Eigen::Vector3d normal = constraintVector;
    if (normal.norm() < 1e-12) {
      return delta;
    }
    normal.normalize();
    return delta - normal * delta.dot(normal);
  }

  return delta;
}

bool DragController::isDragging() const
{
  return dragging_;
}

Draggable* DragController::activeDraggable() const
{
  return active_;
}

bool DragController::handleMousePress(
    const HitResult& hit,
    const std::unordered_map<uint64_t, EntityInfo>& entityMap)
{
  (void)entityMap;
  if (dragging_) {
    return false;
  }

  for (const auto& draggable : draggables_) {
    if (!draggable) {
      continue;
    }
    if (!draggable->canDrag(hit)) {
      continue;
    }
    active_ = draggable.get();
    dragging_ = true;
    drag_origin_ = hit.point;
    active_->beginDrag(hit);
    return true;
  }

  return false;
}

void DragController::handleMouseDrag(
    double screenDx,
    double screenDy,
    const Camera& camera,
    double screenWidth,
    double screenHeight,
    const ModifierKeys& mods)
{
  if (!dragging_ || !active_) {
    return;
  }

  active_->updateDrag(
      screenDx, screenDy, camera, screenWidth, screenHeight, mods);
}

void DragController::handleMouseRelease()
{
  if (active_) {
    active_->endDrag();
  }
  active_ = nullptr;
  dragging_ = false;
}

//=============================================================================
SimpleFrameDraggable::SimpleFrameDraggable(
    dart::dynamics::SimpleFrame* frame, uint64_t markerNodeId)
  : frame_(frame), marker_node_id_(markerNodeId)
{
}

bool SimpleFrameDraggable::canDrag(const HitResult& hit) const
{
  return hit.node_id == marker_node_id_;
}

void SimpleFrameDraggable::beginDrag(const HitResult& hit)
{
  if (!frame_) {
    return;
  }

  saved_transform_ = frame_->getWorldTransform();
  pick_offset_ = hit.point - saved_transform_.translation();
  saved_rotation_ = Eigen::AngleAxisd(saved_transform_.rotation());
}

void SimpleFrameDraggable::updateDrag(
    double screenDx,
    double screenDy,
    const Camera& camera,
    double screenW,
    double screenH,
    const ModifierKeys& mods)
{
  if (!frame_) {
    return;
  }

  const Eigen::Vector3d fromPosition
      = saved_transform_.translation() + pick_offset_;
  const Eigen::Vector3d delta = DragController::getDeltaCursor(
      fromPosition,
      ConstraintType::Unconstrained,
      Eigen::Vector3d::UnitZ(),
      camera,
      screenDx,
      screenDy,
      screenW,
      screenH);

  if (mods.ctrl) {
    const double angle = delta.norm();
    if (angle < 1e-12) {
      return;
    }
    Eigen::Vector3d axis = delta.normalized();
    Eigen::AngleAxisd rot(angle, axis);
    saved_rotation_ = rot * saved_rotation_;
    saved_transform_.linear() = saved_rotation_.toRotationMatrix();
  } else {
    saved_transform_.translation() += delta;
  }

  frame_->setTransform(saved_transform_);
}

void SimpleFrameDraggable::updateDrag(
    const Eigen::Vector3d& delta, const ModifierKeys& mods)
{
  if (!frame_) {
    return;
  }

  if (mods.ctrl) {
    const double angle = delta.norm();
    if (angle < 1e-12) {
      return;
    }
    Eigen::Vector3d axis = delta.normalized();
    Eigen::AngleAxisd rot(angle, axis);
    saved_rotation_ = rot * saved_rotation_;
    saved_transform_.linear() = saved_rotation_.toRotationMatrix();
  } else {
    saved_transform_.translation() += delta;
  }

  frame_->setTransform(saved_transform_);
}

void SimpleFrameDraggable::endDrag() {}

//=============================================================================
BodyNodeDraggable::BodyNodeDraggable(
    dart::dynamics::BodyNode* bodyNode,
    const std::unordered_map<uint64_t, EntityInfo>& entityMap,
    bool useExternalIK,
    bool useWholeBody)
  : body_node_(bodyNode),
    entity_map_(entityMap),
    use_external_ik_(useExternalIK),
    use_whole_body_(useWholeBody)
{
}

bool BodyNodeDraggable::canDrag(const HitResult& hit) const
{
  auto it = entity_map_.find(hit.node_id);
  if (it == entity_map_.end()) {
    return false;
  }
  return it->second.body_node == body_node_;
}

void BodyNodeDraggable::beginDrag(const HitResult& hit)
{
  if (!body_node_) {
    return;
  }

  if (use_external_ik_) {
    ik_ = dart::dynamics::InverseKinematics::create(body_node_);
  } else {
    ik_ = body_node_->getIK(true);
  }

  if (!ik_) {
    return;
  }

  if (use_whole_body_) {
    ik_->useWholeBody();
  } else {
    ik_->useChain();
  }

  ik_->setGradientMethod<
      dart::dynamics::InverseKinematics::JacobianTranspose>();

  const Eigen::Isometry3d bodyTf = body_node_->getWorldTransform();
  saved_local_offset_ = bodyTf.inverse() * hit.point;
  ik_->setOffset(saved_local_offset_);

  saved_target_transform_ = bodyTf;
  saved_target_transform_.translation() = hit.point;
  saved_rotation_ = Eigen::AngleAxisd(bodyTf.rotation());

  auto target = ik_->getTarget();
  target->setTransform(saved_target_transform_);
}

void BodyNodeDraggable::updateDrag(
    double screenDx,
    double screenDy,
    const Camera& camera,
    double screenW,
    double screenH,
    const ModifierKeys& mods)
{
  if (!body_node_ || !ik_) {
    return;
  }

  const Eigen::Vector3d fromPosition = saved_target_transform_.translation();
  const Eigen::Vector3d delta = DragController::getDeltaCursor(
      fromPosition,
      ConstraintType::Unconstrained,
      Eigen::Vector3d::UnitZ(),
      camera,
      screenDx,
      screenDy,
      screenW,
      screenH);

  if (mods.ctrl) {
    const double angle = delta.norm();
    if (angle < 1e-12) {
      return;
    }
    Eigen::AngleAxisd rot(angle, delta.normalized());
    saved_rotation_ = rot * saved_rotation_;
    if (!mods.alt) {
      saved_target_transform_.linear() = saved_rotation_.toRotationMatrix();
    }
  } else {
    saved_target_transform_.translation() += delta;
  }

  auto target = ik_->getTarget();
  target->setTransform(saved_target_transform_);
  ik_->solveAndApply(true);
}

void BodyNodeDraggable::endDrag()
{
  if (!use_external_ik_ && body_node_) {
    body_node_->clearIK();
  }
  ik_ = nullptr;
}

//=============================================================================
InteractiveFrameDraggable::InteractiveFrameDraggable(
    InteractiveFrame* frame,
    std::unordered_map<uint64_t, std::size_t> toolNodeMap)
  : frame_(frame), tool_node_map_(std::move(toolNodeMap))
{
}

bool InteractiveFrameDraggable::canDrag(const HitResult& hit) const
{
  return tool_node_map_.count(hit.node_id) > 0;
}

void InteractiveFrameDraggable::beginDrag(const HitResult& hit)
{
  if (!frame_) {
    return;
  }

  auto it = tool_node_map_.find(hit.node_id);
  if (it == tool_node_map_.end()) {
    return;
  }

  const auto encoded = it->second;
  const auto type = static_cast<InteractiveTool::Type>(encoded / 3);
  const auto coord = encoded % 3;

  active_tool_ = frame_->getTool(type, coord);
  active_type_ = type;
  active_coord_ = coord;
  saved_transform_ = frame_->getTransform();
  pick_offset_ = hit.point - saved_transform_.translation();
}

void InteractiveFrameDraggable::updateDrag(
    double screenDx,
    double screenDy,
    const Camera& camera,
    double screenW,
    double screenH,
    const ModifierKeys& modifiers)
{
  (void)modifiers;
  if (!frame_ || !active_tool_) {
    return;
  }

  const Eigen::Vector3d axis
      = frame_->getTransform().rotation().col(active_coord_);
  const Eigen::Vector3d fromPosition
      = saved_transform_.translation() + pick_offset_;

  if (active_type_ == InteractiveTool::LINEAR) {
    const Eigen::Vector3d delta = DragController::getDeltaCursor(
        fromPosition,
        ConstraintType::Line,
        axis,
        camera,
        screenDx,
        screenDy,
        screenW,
        screenH);
    const Eigen::Vector3d translation = saved_transform_.translation() + delta;
    frame_->setTranslation(translation);
    saved_transform_.translation() = translation;
    return;
  }

  if (active_type_ == InteractiveTool::PLANAR) {
    const Eigen::Vector3d delta = DragController::getDeltaCursor(
        fromPosition,
        ConstraintType::Plane,
        axis,
        camera,
        screenDx,
        screenDy,
        screenW,
        screenH);
    const Eigen::Vector3d translation = saved_transform_.translation() + delta;
    frame_->setTranslation(translation);
    saved_transform_.translation() = translation;
    return;
  }

  if (active_type_ == InteractiveTool::ANGULAR) {
    const Eigen::Vector3d delta = DragController::getDeltaCursor(
        fromPosition,
        ConstraintType::Unconstrained,
        Eigen::Vector3d::UnitZ(),
        camera,
        screenDx,
        screenDy,
        screenW,
        screenH);
    const Eigen::Vector3d perp = delta - axis * delta.dot(axis);
    const double angle = perp.norm();
    if (angle < 1e-12) {
      return;
    }
    const Eigen::Vector3d axisNormalized = axis.normalized();
    const Eigen::Matrix3d rotation
        = Eigen::AngleAxisd(angle, axisNormalized).toRotationMatrix()
          * saved_transform_.rotation();
    frame_->setRotation(rotation);
    saved_transform_.linear() = rotation;
  }
}

void InteractiveFrameDraggable::endDrag()
{
  active_tool_ = nullptr;
}

} // namespace gui
} // namespace dart
