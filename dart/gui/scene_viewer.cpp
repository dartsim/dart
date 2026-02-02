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
#include <iomanip>
#include <sstream>

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
  worlds_.clear();
  if (world) {
    worlds_.push_back(WorldEntry{std::move(world), true, true});
  }
}

void SceneViewer::addWorld(std::shared_ptr<dart::simulation::World> world)
{
  if (!world) {
    return;
  }
  auto it
      = std::find_if(worlds_.begin(), worlds_.end(), [&](const WorldEntry& e) {
          return e.world == world;
        });
  if (it != worlds_.end()) {
    return;
  }
  worlds_.push_back(WorldEntry{std::move(world), true, true});
}

void SceneViewer::removeWorld(
    const std::shared_ptr<dart::simulation::World>& world)
{
  auto it
      = std::find_if(worlds_.begin(), worlds_.end(), [&](const WorldEntry& e) {
          return e.world == world;
        });
  if (it != worlds_.end()) {
    worlds_.erase(it);
  }
}

void SceneViewer::setWorldActive(
    const std::shared_ptr<dart::simulation::World>& world, bool active)
{
  auto it
      = std::find_if(worlds_.begin(), worlds_.end(), [&](const WorldEntry& e) {
          return e.world == world;
        });
  if (it != worlds_.end()) {
    it->active = active;
  }
}

void SceneViewer::setWorldSimulating(
    const std::shared_ptr<dart::simulation::World>& world, bool simulating)
{
  auto it
      = std::find_if(worlds_.begin(), worlds_.end(), [&](const WorldEntry& e) {
          return e.world == world;
        });
  if (it != worlds_.end()) {
    it->simulating = simulating;
  }
}

void SceneViewer::setTargetFrequency(double hz)
{
  target_frequency_ = hz > 0 ? hz : 60.0;
}

void SceneViewer::setTargetRealTimeFactor(double rtf)
{
  target_rtf_ = rtf > 0 ? rtf : 1.0;
}

double SceneViewer::getLastRealTimeFactor() const
{
  return last_rtf_;
}

double SceneViewer::getLowestRealTimeFactor() const
{
  return lowest_rtf_;
}

double SceneViewer::getHighestRealTimeFactor() const
{
  return highest_rtf_;
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

  if (!worlds_.empty()) {
    if (pre_refresh_cb_) {
      pre_refresh_cb_();
    }

    if (!paused_) {
      for (auto& entry : worlds_) {
        if (entry.simulating && entry.world) {
          for (std::size_t i = 0; i < num_steps_per_cycle_; ++i) {
            if (pre_step_cb_) {
              pre_step_cb_();
            }
            entry.world->step();
            if (post_step_cb_) {
              post_step_cb_();
            }
          }
        }
      }
    }

    Scene merged;
    for (auto& entry : worlds_) {
      if (entry.active && entry.world) {
        Scene extracted = extractor_.extract(*entry.world, simple_frames_);
        merged.nodes.insert(
            merged.nodes.end(),
            std::make_move_iterator(extracted.nodes.begin()),
            std::make_move_iterator(extracted.nodes.end()));
      }
    }

    merged.camera = scene_.camera;
    merged.debug_lines = scene_.debug_lines;
    merged.debug_points = scene_.debug_points;
    merged.debug_polygons = scene_.debug_polygons;
    merged.debug_polyhedra = scene_.debug_polyhedra;
    merged.grid_config = scene_.grid_config;
    merged.show_axes = scene_.show_axes;
    merged.lights = scene_.lights;
    merged.headlight = scene_.headlight;
    merged.paused = paused_;

    if (!worlds_.empty() && worlds_[0].world) {
      merged.sim_time = worlds_[0].world->getTime();
    }

    addSimpleFrameMarkers(simple_frames_, merged);
    scene_ = std::move(merged);
  } else {
    Scene empty;
    empty.camera = scene_.camera;
    empty.debug_lines = scene_.debug_lines;
    empty.debug_points = scene_.debug_points;
    empty.debug_polygons = scene_.debug_polygons;
    empty.debug_polyhedra = scene_.debug_polyhedra;
    empty.grid_config = scene_.grid_config;
    empty.show_axes = scene_.show_axes;
    empty.lights = scene_.lights;
    empty.headlight = scene_.headlight;
    empty.paused = paused_;
    addSimpleFrameMarkers(simple_frames_, empty);
    scene_ = std::move(empty);
  }

  auto now = std::chrono::steady_clock::now();
  if (!first_frame_ && !worlds_.empty()) {
    double wall_seconds
        = std::chrono::duration<double>(now - last_frame_time_).count();
    if (wall_seconds > 1e-9) {
      double sim_dt = 0.001;
      if (worlds_[0].world) {
        sim_dt = worlds_[0].world->getTimeStep();
      }
      sim_dt *= static_cast<double>(num_steps_per_cycle_);
      if (!paused_) {
        last_rtf_ = sim_dt / wall_seconds;
        lowest_rtf_ = std::min(lowest_rtf_, last_rtf_);
        highest_rtf_ = std::max(highest_rtf_, last_rtf_);
      }
    }
  }
  last_frame_time_ = now;
  first_frame_ = false;

  if (post_refresh_cb_) {
    post_refresh_cb_();
  }

  if (recording_ && backend_) {
    std::ostringstream oss;
    oss << recording_directory_ << "/" << recording_prefix_ << std::setfill('0')
        << std::setw(static_cast<int>(recording_digits_))
        << recording_frame_count_ << ".png";
    backend_->captureScreenshot(oss.str());
    ++recording_frame_count_;
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
  if (worlds_.empty()) {
    return;
  }

  for (auto& entry : worlds_) {
    if (entry.simulating && entry.world) {
      if (pre_step_cb_) {
        pre_step_cb_();
      }
      entry.world->step();
      if (post_step_cb_) {
        post_step_cb_();
      }
    }
  }

  Scene merged;
  for (auto& entry : worlds_) {
    if (entry.active && entry.world) {
      Scene extracted = extractor_.extract(*entry.world, simple_frames_);
      merged.nodes.insert(
          merged.nodes.end(),
          std::make_move_iterator(extracted.nodes.begin()),
          std::make_move_iterator(extracted.nodes.end()));
    }
  }

  merged.camera = scene_.camera;
  merged.debug_lines = scene_.debug_lines;
  merged.debug_points = scene_.debug_points;
  merged.debug_polygons = scene_.debug_polygons;
  merged.debug_polyhedra = scene_.debug_polyhedra;
  merged.grid_config = scene_.grid_config;
  merged.show_axes = scene_.show_axes;
  merged.lights = scene_.lights;
  merged.headlight = scene_.headlight;
  addSimpleFrameMarkers(simple_frames_, merged);
  scene_ = std::move(merged);
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

void SceneViewer::addSupportPolygon(
    const std::vector<Eigen::Vector3d>& vertices,
    const Eigen::Vector3d& centroid,
    const Eigen::Vector3d& com)
{
  DebugPolygon poly;
  poly.vertices = vertices;
  poly.centroid = centroid;
  poly.com = com;

  // Color-code based on COM stability: check if COM is inside the polygon
  // Simple 2D point-in-polygon test on the XZ plane (assuming ground plane)
  bool inside = false;
  const std::size_t n = vertices.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    // Use x and z coordinates (assuming ZX ground plane)
    double xi = vertices[i].x(), zi = vertices[i].z();
    double xj = vertices[j].x(), zj = vertices[j].z();
    double cx = com.x(), cz = com.z();
    if (((zi > cz) != (zj > cz))
        && (cx < (xj - xi) * (cz - zi) / (zj - zi) + xi)) {
      inside = !inside;
    }
  }

  if (inside) {
    poly.fill_color = Eigen::Vector4d(0.0, 0.8, 0.0, 0.3); // green
    poly.com_color = Eigen::Vector4d(0.0, 1.0, 0.0, 1.0);
  } else {
    poly.fill_color = Eigen::Vector4d(0.8, 0.0, 0.0, 0.3); // red
    poly.com_color = Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
  }

  scene_.debug_polygons.push_back(std::move(poly));
}

void SceneViewer::addDebugPolyhedron(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::pair<std::size_t, std::size_t>>& edges,
    const Eigen::Vector4d& color)
{
  DebugPolyhedron poly;
  poly.vertices = vertices;
  poly.edges = edges;
  poly.color = color;
  scene_.debug_polyhedra.push_back(std::move(poly));
}

void SceneViewer::clearDebugOverlays()
{
  scene_.debug_polygons.clear();
  scene_.debug_polyhedra.clear();
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

void SceneViewer::startRecording(
    const std::string& directory, const std::string& prefix, std::size_t digits)
{
  recording_ = true;
  recording_directory_ = directory;
  recording_prefix_ = prefix;
  recording_digits_ = digits;
  recording_frame_count_ = 0;
}

void SceneViewer::stopRecording()
{
  recording_ = false;
}

bool SceneViewer::isRecording() const
{
  return recording_;
}

void SceneViewer::setPreStepCallback(StepCallback cb)
{
  pre_step_cb_ = std::move(cb);
}

void SceneViewer::setPostStepCallback(StepCallback cb)
{
  post_step_cb_ = std::move(cb);
}

void SceneViewer::setPreRefreshCallback(StepCallback cb)
{
  pre_refresh_cb_ = std::move(cb);
}

void SceneViewer::setPostRefreshCallback(StepCallback cb)
{
  post_refresh_cb_ = std::move(cb);
}

void SceneViewer::setNumStepsPerCycle(std::size_t n)
{
  num_steps_per_cycle_ = (n > 0) ? n : 1;
}

std::size_t SceneViewer::numStepsPerCycle() const
{
  return num_steps_per_cycle_;
}

} // namespace gui
} // namespace dart
