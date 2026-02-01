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

  const Eigen::Vector3d delta = getDeltaCursor(
      drag_origin_,
      ConstraintType::Unconstrained,
      Eigen::Vector3d::UnitZ(),
      camera,
      screenDx,
      screenDy,
      screenWidth,
      screenHeight);
  active_->updateDrag(delta, mods);
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
    const Eigen::Vector3d& delta, const ModifierKeys& mods)
{
  if (!body_node_ || !ik_) {
    return;
  }

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

} // namespace gui
} // namespace dart
