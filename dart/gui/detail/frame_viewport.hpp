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

#ifndef DART_GUI_DETAIL_FRAME_VIEWPORT_HPP_
#define DART_GUI_DETAIL_FRAME_VIEWPORT_HPP_

#include <dart/gui/application.hpp>

#include <array>
#include <optional>

#include <cstddef>

struct GLFWwindow;
struct ImGuiIO;

namespace filament {
class Camera;
class View;
} // namespace filament

namespace dart::gui {

struct OrbitCameraController;
struct OrbitCameraControlOptions;

} // namespace dart::gui

namespace dart::gui::detail {

class SelectionController;

struct ViewportPaneFrame
{
  dart::gui::ViewportPaneKind kind = dart::gui::ViewportPaneKind::Perspective;
  dart::gui::OrbitCamera camera;
  int x = 0;
  int y = 0;
  int width = 1;
  int height = 1;
  bool active = false;
};

struct FrameViewport
{
  int width = 1;
  int height = 1;
  std::size_t paneCount = 1u;
  std::array<ViewportPaneFrame, dart::gui::kMaxViewportPanes> panes;
};

FrameViewport makeFrameViewport(
    const dart::gui::ViewportLayoutOptions& layoutOptions,
    int width,
    int height);

std::size_t activeViewportPaneIndex(const FrameViewport& viewport);

const ViewportPaneFrame& activeViewportPane(const FrameViewport& viewport);

std::optional<std::size_t> viewportPaneIndexAtCursor(
    const FrameViewport& viewport, double cursorX, double cursorY);

const ViewportPaneFrame* viewportPaneAtCursor(
    const FrameViewport& viewport, double cursorX, double cursorY);

FrameViewport updateFrameViewport(
    GLFWwindow* window,
    const std::array<::filament::View*, dart::gui::kMaxViewportPanes>& views,
    const std::array<::filament::Camera*, dart::gui::kMaxViewportPanes>&
        cameras,
    dart::gui::OrbitCameraController& cameraController,
    const SelectionController& selectionController,
    ImGuiIO& imguiIo,
    int defaultWidth,
    int defaultHeight,
    double worldTimeStep,
    bool showUi,
    double guiScale,
    const dart::gui::OrbitCameraControlOptions& cameraControls,
    const dart::gui::ViewportLayoutOptions& layoutOptions);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_FRAME_VIEWPORT_HPP_
