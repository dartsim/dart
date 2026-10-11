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

#ifndef DART_GUI_VIEWER_HPP_
#define DART_GUI_VIEWER_HPP_

#include <dart/gui/Export.hpp>

#include <dart/common/Subject.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include <cstddef>
#include <cstdint>

struct ImGuiContext;

namespace dart {
namespace simulation {
class World;
}
namespace dynamics {
class Entity;
class SimpleFrame;
class BodyNode;
class Shape;
} // namespace dynamics
namespace gui {
class Viewer;
namespace detail {
DART_GUI_API std::vector<std::uint8_t> readViewerPixelsRGBA(Viewer& viewer);
DART_GUI_API std::vector<std::uint8_t> captureViewerPixelsRGBA(Viewer& viewer);
} // namespace detail
class WorldNode;
class ViewerAttachment;
class ImGuiWidget;
class GUIEventHandler;
class GUIEventAdapter;
class DefaultEventHandler;
class DragAndDrop;
class SimpleFrameDnD;
class SimpleFrameShapeDnD;
class InteractiveFrameDnD;
class BodyNodeDnD;
class InteractiveFrame;
struct PickInfo;
enum class CameraMode
{
  RGBA,
  DEPTH
};
DART_GUI_API std::string toString(CameraMode mode);

/// Backend-neutral viewer. Graphics are initialized by setup or the first
/// frame.
class DART_GUI_API Viewer : public common::Subject
{
public:
  explicit Viewer(
      const Eigen::Vector4d& clearColor = Eigen::Vector4d(0.9, 0.9, 0.9, 1.0));
  virtual ~Viewer();
  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;
  void addWorldNode(std::shared_ptr<WorldNode> node, bool active = true);
  void removeWorldNode(const std::shared_ptr<WorldNode>& node);
  void removeWorldNode(const std::shared_ptr<simulation::World>& world);
  std::shared_ptr<WorldNode> getWorldNode(
      const std::shared_ptr<simulation::World>& world) const;
  std::size_t getNumWorldNodes() const;
  void setWorldNodeActive(
      const std::shared_ptr<WorldNode>& node, bool active = true);
  void setWorldNodeActive(
      const std::shared_ptr<simulation::World>& world, bool active = true);
  void addAttachment(std::shared_ptr<ViewerAttachment> attachment);
  void removeAttachment(const std::shared_ptr<ViewerAttachment>& attachment);
  const std::vector<std::shared_ptr<ViewerAttachment>>& getAttachments() const;
  void addWidget(std::shared_ptr<ImGuiWidget> widget);
  void removeWidget(const std::shared_ptr<ImGuiWidget>& widget);
  void addEventHandler(std::shared_ptr<GUIEventHandler> handler);
  void removeEventHandler(const std::shared_ptr<GUIEventHandler>& handler);
  /// Dispatch an input event; also usable without initializing graphics.
  bool handleEvent(const GUIEventAdapter& event);
  DefaultEventHandler* getDefaultEventHandler() const;
  void switchDefaultEventHandler(bool on);
  void addInstructionText(const std::string& text);
  const std::string& getInstructions() const;
  DragAndDrop* enableDragAndDrop(dynamics::Entity* entity);
  SimpleFrameDnD* enableDragAndDrop(dynamics::SimpleFrame* frame);
  SimpleFrameShapeDnD* enableDragAndDrop(
      dynamics::SimpleFrame* frame, dynamics::Shape* shape);
  InteractiveFrameDnD* enableDragAndDrop(InteractiveFrame* frame);
  BodyNodeDnD* enableDragAndDrop(
      dynamics::BodyNode* body,
      bool useExternalIK = true,
      bool useWholeBody = false);
  bool disableDragAndDrop(DragAndDrop* interaction);
  void updateDragAndDrops();
  void computeIntersections(
      const GUIEventAdapter& event, std::vector<PickInfo>& picks);
  void getRayUnderCursor(
      double x,
      double y,
      Eigen::Vector3d& nearPoint,
      Eigen::Vector3d& farPoint) const;
  Eigen::Vector3d getCameraDirection() const;
  void setWireframe(bool enabled);
  bool isWireframe() const;
  void setBodyWireframe(dynamics::BodyNode* body, bool enabled);
  bool getBodyWireframe(const dynamics::BodyNode* body) const;
  void simulate(bool on);
  bool isSimulating() const;
  void allowSimulation(bool allow);
  bool isAllowingSimulation() const;
  void setCameraHome(
      const Eigen::Vector3d& eye,
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& up);
  void setCameraHomePosition(
      const Eigen::Vector3d& eye,
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& up);
  void home();
  void setVerticalFieldOfView(double fov);
  double getVerticalFieldOfView() const;
  void setUpwardsDirection(const Eigen::Vector3d& up);
  void setCameraMode(CameraMode mode);
  CameraMode getCameraMode() const;
  void switchHeadlights(bool on);
  bool checkHeadlights() const;
  /// Explicit interactive dimensions are native window units: physical pixels
  /// on Linux and Windows, points on macOS.
  void setUpViewInWindow(int x, int y, int width, int height);
  /// Choose the initial window from the monitor work area and DPI.
  void setUpViewInWindow();
  /// Capture dimensions are physical pixels and require no Linux display.
  void setUpViewForCapture(int width, int height);
  void setGuiScale(double scale);
  double getGuiScale() const;
  /// Follow monitor DPI, or freeze the current scale when disabled.
  void setAutomaticGuiScale(bool automatic = true);
  /// Null until graphics setup; context lifetime belongs to this viewer.
  ImGuiContext* getImGuiContext() const;
  void setBackgroundColor(const Eigen::Vector4d& color);
  void setWindowTitle(const std::string& title);
  void captureScreen(const std::string& filename);
  void record(
      const std::string& directory,
      const std::string& prefix = "image",
      bool restart = false,
      std::size_t digits = 6);
  void pauseRecording();
  bool isRecording() const;
  virtual void updateViewer();
  void frame();
  void run();
  void setDone(bool done = true);
  bool done() const;
  std::string getRenderBackend() const;
  std::string getRenderDevice() const;

private:
  friend class DefaultEventHandler;
  double getWindowDepthRayParameter(double depth) const;
  friend class WorldNode;
  bool ownsWorldSteps(WorldNode* node);
  friend std::vector<std::uint8_t> detail::readViewerPixelsRGBA(Viewer& viewer);
  friend std::vector<std::uint8_t> detail::captureViewerPixelsRGBA(
      Viewer& viewer);
  bool frameImpl(std::vector<std::uint8_t>* captureRGBA);
  struct Impl;
  std::unique_ptr<Impl> mImpl;
};
} // namespace gui
} // namespace dart
#endif
