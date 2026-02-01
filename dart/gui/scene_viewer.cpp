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

#include "dart/gui/scene_viewer.hpp"

#include "dart/common/logging.hpp"
#include "dart/simulation/world.hpp"

#include <algorithm>

#include <cstdint>

namespace dart {
namespace gui {

namespace {

uint64_t makeSimpleFrameMarkerId(const dart::dynamics::SimpleFrame* frame)
{
  const auto raw = reinterpret_cast<uintptr_t>(frame);
  return static_cast<uint64_t>(raw) | (1ULL << 63);
}

void addSimpleFrameMarkers(
    const std::vector<dart::dynamics::SimpleFrame*>& frames, Scene& scene)
{
  for (auto* frame : frames) {
    if (!frame) {
      continue;
    }

    SceneNode node;
    node.id = makeSimpleFrameMarkerId(frame);
    node.transform = frame->getWorldTransform();
    node.shape = SphereData{0.03};
    node.material.color = Eigen::Vector4d(0.2, 0.8, 0.2, 0.8);
    node.visible = true;
    scene.nodes.push_back(std::move(node));
  }
}

} // namespace

SceneViewer::SceneViewer(
    std::unique_ptr<ViewerBackend> backend, const ViewerConfig& config)
  : backend_(std::move(backend)), config_(config)
{
  if (!backend_) {
    DART_WARN("SceneViewer constructed with a nullptr backend.");
  }
}

void SceneViewer::setWorld(std::shared_ptr<dart::simulation::World> world)
{
  world_ = std::move(world);
}

void SceneViewer::run()
{
  while (frame()) {
  }

  if (initialized_ && backend_) {
    backend_->shutdown();
    initialized_ = false;
  }
}

bool SceneViewer::frame()
{
  if (!backend_) {
    return false;
  }

  if (!initialized_) {
    if (!backend_->initialize(config_)) {
      DART_WARN("Failed to initialize viewer backend.");
      return false;
    }
    initialized_ = true;
  }

  if (backend_->shouldClose()) {
    backend_->shutdown();
    initialized_ = false;
    return false;
  }

  if (world_) {
    if (!paused_) {
      world_->step();
    }

    Scene extracted = extractor_.extract(*world_, simple_frames_);
    extracted.camera = scene_.camera;
    extracted.debug_lines = scene_.debug_lines;
    extracted.debug_points = scene_.debug_points;
    extracted.grid_config = scene_.grid_config;
    extracted.show_axes = scene_.show_axes;
    extracted.lights = scene_.lights;
    extracted.headlight = scene_.headlight;
    extracted.paused = paused_;
    extracted.sim_time = world_->getTime();
    addSimpleFrameMarkers(simple_frames_, extracted);
    scene_ = std::move(extracted);
  } else {
    Scene empty;
    empty.camera = scene_.camera;
    empty.debug_lines = scene_.debug_lines;
    empty.debug_points = scene_.debug_points;
    empty.grid_config = scene_.grid_config;
    empty.show_axes = scene_.show_axes;
    empty.lights = scene_.lights;
    empty.headlight = scene_.headlight;
    empty.paused = paused_;
    addSimpleFrameMarkers(simple_frames_, empty);
    scene_ = std::move(empty);
  }

  backend_->beginFrame();
  backend_->render(scene_);
  backend_->endFrame();

  auto events = backend_->pollEvents();

  camera_controller_.handleEvents(events, scene_.camera);

  for (const auto& event : events) {
    if (const auto* keyEvent = std::get_if<KeyEvent>(&event)) {
      if (!keyEvent->pressed) {
        continue;
      }

      switch (keyEvent->key) {
        case Key::Space:
          paused_ = !paused_;
          break;
        case Key::Enter:
          step();
          break;
        case Key::Escape:
          scene_.selected_node_id = std::nullopt;
          break;
        default:
          break;
      }
    }

    if (const auto* mouseEvent = std::get_if<MouseButtonEvent>(&event)) {
      if (mouseEvent->button == MouseButton::Left && mouseEvent->pressed) {
        auto hit = backend_->pickNode(scene_, mouseEvent->x, mouseEvent->y);
        if (hit
            && drag_controller_.handleMousePress(
                *hit, extractor_.entityMap())) {
          continue;
        }
        scene_.selected_node_id
            = hit ? std::make_optional(hit->node_id) : std::nullopt;
      }

      if (mouseEvent->button == MouseButton::Left && !mouseEvent->pressed
          && drag_controller_.isDragging()) {
        drag_controller_.handleMouseRelease();
      }
    }

    if (const auto* moveEvent = std::get_if<MouseMoveEvent>(&event)) {
      if (drag_controller_.isDragging()) {
        drag_controller_.handleMouseDrag(
            moveEvent->dx,
            moveEvent->dy,
            scene_.camera,
            static_cast<double>(config_.width),
            static_cast<double>(config_.height),
            moveEvent->modifiers);
      }
    }
  }

  return true;
}

void SceneViewer::pause()
{
  paused_ = true;
}

void SceneViewer::unpause()
{
  paused_ = false;
}

void SceneViewer::step()
{
  if (!world_) {
    return;
  }

  world_->step();

  Scene extracted = extractor_.extract(*world_, simple_frames_);
  extracted.camera = scene_.camera;
  extracted.debug_lines = scene_.debug_lines;
  extracted.debug_points = scene_.debug_points;
  extracted.grid_config = scene_.grid_config;
  extracted.show_axes = scene_.show_axes;
  extracted.lights = scene_.lights;
  extracted.headlight = scene_.headlight;
  addSimpleFrameMarkers(simple_frames_, extracted);
  scene_ = std::move(extracted);
}

bool SceneViewer::isPaused() const
{
  return paused_;
}

Camera& SceneViewer::camera()
{
  return scene_.camera;
}

const Camera& SceneViewer::camera() const
{
  return scene_.camera;
}

void SceneViewer::addDebugLine(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector4d& color)
{
  scene_.debug_lines.push_back(DebugLine{start, end, color});
}

void SceneViewer::addDebugPoint(
    const Eigen::Vector3d& pos, const Eigen::Vector4d& color, double size)
{
  DebugPoint point;
  point.position = pos;
  point.color = color;
  point.size = size;
  scene_.debug_points.push_back(point);
}

void SceneViewer::clearDebug()
{
  scene_.debug_lines.clear();
  scene_.debug_points.clear();
}

void SceneViewer::enableDragAndDrop(dart::dynamics::SimpleFrame* frame)
{
  if (!frame) {
    return;
  }

  auto existing
      = std::find(simple_frames_.begin(), simple_frames_.end(), frame);
  if (existing != simple_frames_.end()) {
    return;
  }

  simple_frames_.push_back(frame);
  drag_controller_.addDraggable(
      std::make_unique<SimpleFrameDraggable>(
          frame, makeSimpleFrameMarkerId(frame)));
}

void SceneViewer::enableDragAndDrop(
    dart::dynamics::BodyNode* bodyNode, bool useExternalIK, bool useWholeBody)
{
  if (!bodyNode) {
    return;
  }

  drag_controller_.addDraggable(
      std::make_unique<BodyNodeDraggable>(
          bodyNode, extractor_.entityMap(), useExternalIK, useWholeBody));
}

void SceneViewer::disableDragAndDrop(dart::dynamics::SimpleFrame* frame)
{
  if (!frame) {
    return;
  }

  auto it = std::remove(simple_frames_.begin(), simple_frames_.end(), frame);
  if (it == simple_frames_.end()) {
    return;
  }

  simple_frames_.erase(it, simple_frames_.end());
  drag_controller_.clearDraggables();
  for (auto* remaining : simple_frames_) {
    drag_controller_.addDraggable(
        std::make_unique<SimpleFrameDraggable>(
            remaining, makeSimpleFrameMarkerId(remaining)));
  }
}

DragController& SceneViewer::dragController()
{
  return drag_controller_;
}

std::optional<uint64_t> SceneViewer::selectedNodeId() const
{
  return scene_.selected_node_id;
}

void SceneViewer::captureScreenshot(const std::string& filename)
{
  if (backend_) {
    backend_->captureScreenshot(filename);
  }
}

} // namespace gui
} // namespace dart
