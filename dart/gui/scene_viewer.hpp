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

#ifndef DART_GUI_SCENEVIEWER_HPP_
#define DART_GUI_SCENEVIEWER_HPP_

#include <dart/gui/drag_controller.hpp>
#include <dart/gui/export.hpp>
#include <dart/gui/orbit_camera_controller.hpp>
#include <dart/gui/scene.hpp>
#include <dart/gui/scene_extractor.hpp>
#include <dart/gui/viewer_backend.hpp>

#include <dart/simulation/fwd.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace dart {
namespace gui {

struct WorldEntry
{
  std::shared_ptr<dart::simulation::World> world;
  bool active = true;
  bool simulating = true;
};

class DART_GUI_API ViewerAttachment
{
public:
  virtual ~ViewerAttachment() = default;
  virtual void refresh(Scene& scene) = 0;
};

class DART_GUI_API MouseEventHandler
{
public:
  virtual ~MouseEventHandler() = default;
  virtual void handleMouseEvent(const InputEvent& event, const Scene& scene)
      = 0;
};

class DART_GUI_API SceneViewer
{
public:
  using StepCallback = std::function<void()>;

  explicit SceneViewer(
      std::unique_ptr<ViewerBackend> backend, const ViewerConfig& config = {});

  void setWorld(std::shared_ptr<dart::simulation::World> world);

  void addWorld(std::shared_ptr<dart::simulation::World> world);
  void removeWorld(const std::shared_ptr<dart::simulation::World>& world);
  void setWorldActive(
      const std::shared_ptr<dart::simulation::World>& world, bool active);
  void setWorldSimulating(
      const std::shared_ptr<dart::simulation::World>& world, bool simulating);

  void setTargetFrequency(double hz);
  void setTargetRealTimeFactor(double rtf);
  double getLastRealTimeFactor() const;
  double getLowestRealTimeFactor() const;
  double getHighestRealTimeFactor() const;

  void run();

  bool frame();

  void pause();
  void unpause();
  void step();
  bool isPaused() const;

  Camera& camera();
  const Camera& camera() const;

  void addDebugLine(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& end,
      const Eigen::Vector4d& color = Eigen::Vector4d(1, 0, 0, 1));
  void addDebugPoint(
      const Eigen::Vector3d& pos,
      const Eigen::Vector4d& color = Eigen::Vector4d(1, 1, 0, 1),
      double size = 3.0);
  void clearDebug();

  void addSupportPolygon(
      const std::vector<Eigen::Vector3d>& vertices,
      const Eigen::Vector3d& centroid,
      const Eigen::Vector3d& com);
  void addDebugPolyhedron(
      const std::vector<Eigen::Vector3d>& vertices,
      const std::vector<std::pair<std::size_t, std::size_t>>& edges,
      const Eigen::Vector4d& color = Eigen::Vector4d(1, 1, 0, 0.8));
  void clearDebugOverlays();

  void enableDragAndDrop(dart::dynamics::SimpleFrame* frame);
  void disableDragAndDrop(dart::dynamics::SimpleFrame* frame);

  void enableDragAndDrop(
      dart::dynamics::BodyNode* bodyNode,
      bool useExternalIK = true,
      bool useWholeBody = false);

  DragController& dragController();

  std::optional<uint64_t> selectedNodeId() const;

  void captureScreenshot(const std::string& filename);

  void startRecording(
      const std::string& directory,
      const std::string& prefix = "frame",
      std::size_t digits = 6);
  void stopRecording();
  bool isRecording() const;

  void addAttachment(std::shared_ptr<ViewerAttachment> attachment);
  void removeAttachment(const std::shared_ptr<ViewerAttachment>& attachment);

  void addMouseEventHandler(MouseEventHandler* handler);
  void removeMouseEventHandler(MouseEventHandler* handler);

  void setPreStepCallback(StepCallback cb);
  void setPostStepCallback(StepCallback cb);
  void setPreRefreshCallback(StepCallback cb);
  void setPostRefreshCallback(StepCallback cb);
  void setNumStepsPerCycle(std::size_t n);
  std::size_t numStepsPerCycle() const;

private:
  std::unique_ptr<ViewerBackend> backend_;
  SceneExtractor extractor_;
  OrbitCameraController camera_controller_;
  DragController drag_controller_;
  std::vector<dart::dynamics::SimpleFrame*> simple_frames_;
  std::vector<WorldEntry> worlds_;
  Scene scene_;
  bool paused_ = false;
  bool initialized_ = false;
  ViewerConfig config_;
  StepCallback pre_step_cb_;
  StepCallback post_step_cb_;
  StepCallback pre_refresh_cb_;
  StepCallback post_refresh_cb_;
  std::size_t num_steps_per_cycle_ = 1;
  bool recording_ = false;
  std::string recording_directory_;
  std::string recording_prefix_;
  std::size_t recording_digits_ = 6;
  std::size_t recording_frame_count_ = 0;
  std::vector<std::shared_ptr<ViewerAttachment>> attachments_;
  std::vector<MouseEventHandler*> mouse_handlers_;

  // RTF tracking
  double target_frequency_ = 60.0;
  double target_rtf_ = 1.0;
  double last_rtf_ = 1.0;
  double lowest_rtf_ = std::numeric_limits<double>::max();
  double highest_rtf_ = 0.0;
  std::chrono::steady_clock::time_point last_frame_time_;
  bool first_frame_ = true;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_SCENEVIEWER_HPP_
