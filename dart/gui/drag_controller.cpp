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

} // namespace gui
} // namespace dart
