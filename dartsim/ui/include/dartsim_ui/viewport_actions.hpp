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

#include <dart/gui/gizmo.hpp>

#include <dart/dynamics/fwd.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dartsim_engine/scene_object.hpp>
#include <dartsim_engine/sim_engine.hpp>

namespace dartsim::ui {

/// Keyboard-style viewport movement request in world axes.
struct ViewportMoveInput
{
  bool left = false;
  bool right = false;
  bool forward = false;
  bool backward = false;
  bool up = false;
  bool down = false;
  bool fast = false;
  double step = 0.10;
  double fastMultiplier = 4.0;
};

/// Renderer-neutral gizmo proxy for the currently selected engine object.
struct ViewportTransformGizmo
{
  dart::dynamics::SimpleFramePtr target;
  dart::gui::Gizmo gizmo;
  ObjectId object = kNoObject;
};

/// Resolve a movement request into a world-space delta.
[[nodiscard]] Eigen::Vector3d viewportMoveDelta(const ViewportMoveInput& input);

/// Move the primary selected object through an undoable engine command.
bool moveSelectedFromViewport(
    SimEngine& engine, const ViewportMoveInput& input);

/// Create a transform gizmo proxy. The caller owns visibility and callbacks.
[[nodiscard]] ViewportTransformGizmo makeViewportTransformGizmo();

/// Synchronize the gizmo proxy to the current selected, visible, movable
/// object.
bool syncViewportTransformGizmo(
    ViewportTransformGizmo& state, const SimEngine& engine);

/// Commit a gizmo-authored transform back through the undoable command stack.
bool applyViewportTransformGizmo(
    SimEngine& engine,
    ViewportTransformGizmo& state,
    const Eigen::Isometry3d& transform);

} // namespace dartsim::ui
