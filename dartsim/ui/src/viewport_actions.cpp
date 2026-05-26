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

#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <dartsim_engine/commands.hpp>
#include <dartsim_ui/outliner_actions.hpp>
#include <dartsim_ui/viewport_actions.hpp>

#include <cmath>

namespace dartsim::ui {

namespace {

bool isTransformable(ObjectType type)
{
  return type == ObjectType::RigidBody || type == ObjectType::FreeFrame
         || type == ObjectType::FixedFrame;
}

const SceneObject* selectedTransformableObject(const SimEngine& engine)
{
  const ObjectId selected = engine.selection().primary();
  const SceneObject* object = engine.objects().model().find(selected);
  if (object == nullptr || !isTransformable(object->type)
      || selectedViewportRenderable(engine) == 0) {
    return nullptr;
  }
  return object;
}

bool isFinitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

} // namespace

Eigen::Vector3d viewportMoveDelta(const ViewportMoveInput& input)
{
  if (!isFinitePositive(input.step)
      || !isFinitePositive(input.fastMultiplier)) {
    return Eigen::Vector3d::Zero();
  }

  const double scale = input.step * (input.fast ? input.fastMultiplier : 1.0);
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  delta.x() += input.right ? scale : 0.0;
  delta.x() -= input.left ? scale : 0.0;
  delta.y() += input.forward ? scale : 0.0;
  delta.y() -= input.backward ? scale : 0.0;
  delta.z() += input.up ? scale : 0.0;
  delta.z() -= input.down ? scale : 0.0;
  return delta;
}

bool moveSelectedFromViewport(SimEngine& engine, const ViewportMoveInput& input)
{
  return moveSelectedBy(engine, viewportMoveDelta(input));
}

ViewportTransformGizmo makeViewportTransformGizmo()
{
  ViewportTransformGizmo state;
  state.target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "dartsim_selection_transform");
  state.gizmo.target = state.target;
  state.gizmo.label = "Selection Transform";
  state.gizmo.flags = dart::gui::GizmoFlags::All;
  state.gizmo.size = 0.22;
  return state;
}

bool syncViewportTransformGizmo(
    ViewportTransformGizmo& state, const SimEngine& engine)
{
  if (state.target == nullptr) {
    state = makeViewportTransformGizmo();
  }

  const SceneObject* object = selectedTransformableObject(engine);
  if (object == nullptr) {
    state.object = kNoObject;
    return false;
  }

  state.object = object->id;
  state.target->setTransform(object->transform, dart::dynamics::Frame::World());
  state.gizmo.label = object->name + " Transform";
  return true;
}

bool applyViewportTransformGizmo(
    SimEngine& engine,
    ViewportTransformGizmo& state,
    const Eigen::Isometry3d& transform)
{
  if (!engine.canEditScene() || !transform.matrix().allFinite()
      || state.object == kNoObject
      || engine.selection().primary() != state.object) {
    return false;
  }

  const SceneObject* object = selectedTransformableObject(engine);
  if (object == nullptr || object->id != state.object) {
    return false;
  }

  if (object->transform.matrix().isApprox(transform.matrix())) {
    return false;
  }

  engine.execute(commands::setTransform(state.object, transform));
  return syncViewportTransformGizmo(state, engine);
}

} // namespace dartsim::ui
