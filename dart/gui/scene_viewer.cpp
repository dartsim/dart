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

namespace dart {
namespace gui {

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

    Scene extracted = extractor_.extract(*world_);
    extracted.camera = scene_.camera;
    extracted.debug_lines = scene_.debug_lines;
    extracted.debug_points = scene_.debug_points;
    extracted.show_grid = scene_.show_grid;
    extracted.show_axes = scene_.show_axes;
    extracted.lights = scene_.lights;
    extracted.headlight = scene_.headlight;
    extracted.paused = paused_;
    extracted.sim_time = world_->getTime();
    scene_ = std::move(extracted);
  } else {
    Scene empty;
    empty.camera = scene_.camera;
    empty.debug_lines = scene_.debug_lines;
    empty.debug_points = scene_.debug_points;
    empty.show_grid = scene_.show_grid;
    empty.show_axes = scene_.show_axes;
    empty.lights = scene_.lights;
    empty.headlight = scene_.headlight;
    empty.paused = paused_;
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
        if (hit) {
          scene_.selected_node_id = hit->node_id;
        } else {
          scene_.selected_node_id = std::nullopt;
        }
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

  Scene extracted = extractor_.extract(*world_);
  extracted.camera = scene_.camera;
  extracted.debug_lines = scene_.debug_lines;
  extracted.debug_points = scene_.debug_points;
  extracted.show_grid = scene_.show_grid;
  extracted.show_axes = scene_.show_axes;
  extracted.lights = scene_.lights;
  extracted.headlight = scene_.headlight;
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
