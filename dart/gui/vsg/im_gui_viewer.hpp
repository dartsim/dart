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

#include <dart/gui/vsg/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vsg/all.h>

#ifdef DART_HAS_VSGIMGUI
  #include <vsgImGui/RenderImGui.h>
  #include <vsgImGui/SendEventsToImGui.h>
  #include <vsgImGui/imgui.h>
#endif

#include <functional>
#include <memory>
#include <string>

namespace dart::gui::vsg {

#ifdef DART_HAS_VSGIMGUI

/// @brief Callback type for custom ImGui rendering
/// Called each frame to render custom ImGui widgets
using ImGuiCallback = std::function<void()>;

/// @brief VSG viewer with integrated ImGui support for interactive controls
///
/// ImGuiViewer extends VSG visualization with Dear ImGui widget rendering,
/// allowing interactive controls for collision visualization, debugging,
/// and parameter tuning.
///
/// @note Only available when built with vsgImGui support (DART_HAS_VSGIMGUI)
class DART_GUI_VSG_API ImGuiViewer
{
public:
  /// @brief Construct an ImGui-enabled viewer
  /// @param width Window width in pixels
  /// @param height Window height in pixels
  /// @param title Window title
  ImGuiViewer(
      int width = 800,
      int height = 600,
      const std::string& title = "DART VSG ImGui Viewer");

  ~ImGuiViewer();

  ImGuiViewer(const ImGuiViewer&) = delete;
  ImGuiViewer& operator=(const ImGuiViewer&) = delete;
  ImGuiViewer(ImGuiViewer&&) = default;
  ImGuiViewer& operator=(ImGuiViewer&&) = default;

  /// @brief Set the 3D scene to render
  void setScene(::vsg::ref_ptr<::vsg::Node> scene);

  /// @brief Get the root scene group
  [[nodiscard]] ::vsg::ref_ptr<::vsg::Group> getRoot() const;

  /// @brief Add a node to the scene
  void addNode(::vsg::ref_ptr<::vsg::Node> node);

  /// @brief Remove a node from the scene
  void removeNode(::vsg::ref_ptr<::vsg::Node> node);

  /// @brief Clear all nodes from the scene
  void clear();

  /// @brief Set the ImGui rendering callback
  /// @param callback Function called each frame to render ImGui widgets
  void setImGuiCallback(ImGuiCallback callback);

  /// @brief Set camera look-at parameters
  void lookAt(
      const Eigen::Vector3d& eye,
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ());

  /// @brief Reset camera to default position
  void resetCamera();

  /// @brief Set background clear color
  void setBackgroundColor(const Eigen::Vector4d& color);

  /// @brief Render one frame
  /// @return true if viewer is still active, false if should close
  bool frame();

  /// @brief Run the viewer loop until closed
  void run();

  /// @brief Check if the viewer should close
  [[nodiscard]] bool shouldClose() const;

  /// @brief Add a reference grid to the scene
  void addGrid(double size = 10.0, double spacing = 1.0);

  /// @brief Add coordinate axes to the scene
  void addAxes(double length = 1.0);

  /// @brief Get the native VSG window
  [[nodiscard]] ::vsg::ref_ptr<::vsg::Window> getWindow() const;

  /// @brief Get the native VSG viewer
  [[nodiscard]] ::vsg::ref_ptr<::vsg::Viewer> getViewer() const;

  /// @brief Get the camera
  [[nodiscard]] ::vsg::ref_ptr<::vsg::Camera> getCamera() const;

  /// @brief Compute picking ray from screen coordinates
  /// @param screenX Screen X coordinate (0 = left edge)
  /// @param screenY Screen Y coordinate (0 = top edge)
  /// @param rayOrigin [out] Origin of the ray in world space
  /// @param rayDirection [out] Direction of the ray in world space (normalized)
  /// @return true if ray was computed successfully
  bool computePickingRay(
      double screenX,
      double screenY,
      Eigen::Vector3d& rayOrigin,
      Eigen::Vector3d& rayDirection) const;

  /// @brief Get current window size
  /// @param width [out] Window width
  /// @param height [out] Window height
  void getWindowSize(int& width, int& height) const;

private:
  void setupViewer();
  void setupCamera();
  void compile();

  /// Custom Command class for ImGui rendering
  class ImGuiCommand;

  int m_width;
  int m_height;
  std::string m_title;
  Eigen::Vector4d m_backgroundColor{0.2, 0.2, 0.3, 1.0};

  ::vsg::ref_ptr<::vsg::Viewer> m_viewer;
  ::vsg::ref_ptr<::vsg::Window> m_window;
  ::vsg::ref_ptr<::vsg::Group> m_root;
  ::vsg::ref_ptr<::vsg::Group> m_sceneRoot;
  ::vsg::ref_ptr<::vsg::Camera> m_camera;
  ::vsg::ref_ptr<::vsg::LookAt> m_lookAt;

  ImGuiCallback m_imguiCallback;
  ::vsg::ref_ptr<ImGuiCommand> m_imguiCommand;
  ::vsg::ref_ptr<::vsg::Node> m_renderImGui;

  bool m_needsCompile{true};
};

#endif // DART_HAS_VSGIMGUI

} // namespace dart::gui::vsg
