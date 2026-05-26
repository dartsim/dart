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

#include <dart/gui/detail/frame_viewport.hpp>
#include <dart/gui/detail/input.hpp>
#include <dart/gui/detail/render_environment.hpp>
#include <dart/gui/detail/selection.hpp>
#include <dart/gui/viewer.hpp>

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <string_view>

#include <cmath>
#include <cstddef>

namespace dart::gui::detail {
namespace {

ViewportPaneFrame makePaneFrame(
    const dart::gui::ViewportPaneView& pane,
    int x,
    int y,
    int width,
    int height)
{
  ViewportPaneFrame frame;
  frame.kind = pane.kind;
  frame.camera = pane.camera;
  frame.x = x;
  frame.y = y;
  frame.width = std::max(0, width);
  frame.height = std::max(0, height);
  frame.active = pane.active;
  return frame;
}

const dart::gui::ViewportPaneView& singlePaneView(
    const dart::gui::ViewportLayoutOptions& layoutOptions)
{
  if (layoutOptions.mode == dart::gui::ViewportLayoutMode::Quad) {
    for (std::size_t i = 0;
         i < layoutOptions.paneCount && i < dart::gui::kMaxViewportPanes;
         ++i) {
      if (layoutOptions.panes[i].active) {
        return layoutOptions.panes[i];
      }
    }
  }
  return layoutOptions.panes[0];
}

bool canSplitQuad(int width, int height)
{
  return width >= 2 && height >= 2;
}

void getFramebufferCursorPosition(
    GLFWwindow* window,
    int framebufferWidth,
    int framebufferHeight,
    double& x,
    double& y)
{
  glfwGetCursorPos(window, &x, &y);
  int windowWidth = framebufferWidth;
  int windowHeight = framebufferHeight;
  glfwGetWindowSize(window, &windowWidth, &windowHeight);
  if (windowWidth > 0) {
    x *= static_cast<double>(framebufferWidth)
         / static_cast<double>(windowWidth);
  }
  if (windowHeight > 0) {
    y *= static_cast<double>(framebufferHeight)
         / static_cast<double>(windowHeight);
  }
}

} // namespace

FrameViewport makeFrameViewport(
    const dart::gui::ViewportLayoutOptions& layoutOptions,
    int width,
    int height)
{
  FrameViewport viewport;
  viewport.width = std::max(1, width);
  viewport.height = std::max(1, height);

  if (layoutOptions.mode != dart::gui::ViewportLayoutMode::Quad
      || !canSplitQuad(viewport.width, viewport.height)) {
    viewport.paneCount = 1u;
    viewport.panes[0] = makePaneFrame(
        singlePaneView(layoutOptions), 0, 0, viewport.width, viewport.height);
    viewport.panes[0].active = true;
    return viewport;
  }

  viewport.paneCount = std::clamp<std::size_t>(
      layoutOptions.paneCount, 1u, dart::gui::kMaxViewportPanes);
  const int leftWidth = viewport.width / 2;
  const int topHeight = viewport.height / 2;
  const int rightWidth = std::max(1, viewport.width - leftWidth);
  const int bottomHeight = std::max(1, viewport.height - topHeight);

  for (std::size_t i = 0; i < viewport.paneCount; ++i) {
    const dart::gui::ViewportPaneView& pane = layoutOptions.panes[i];
    switch (pane.kind) {
      case dart::gui::ViewportPaneKind::Top:
        viewport.panes[i] = makePaneFrame(pane, 0, 0, leftWidth, topHeight);
        break;
      case dart::gui::ViewportPaneKind::Perspective:
        viewport.panes[i]
            = makePaneFrame(pane, leftWidth, 0, rightWidth, topHeight);
        break;
      case dart::gui::ViewportPaneKind::Front:
        viewport.panes[i]
            = makePaneFrame(pane, 0, topHeight, leftWidth, bottomHeight);
        break;
      case dart::gui::ViewportPaneKind::Right:
        viewport.panes[i] = makePaneFrame(
            pane, leftWidth, topHeight, rightWidth, bottomHeight);
        break;
    }
  }
  return viewport;
}

std::size_t activeViewportPaneIndex(const FrameViewport& viewport)
{
  for (std::size_t i = 0;
       i < viewport.paneCount && i < dart::gui::kMaxViewportPanes;
       ++i) {
    if (viewport.panes[i].active) {
      return i;
    }
  }
  return 0u;
}

const ViewportPaneFrame& activeViewportPane(const FrameViewport& viewport)
{
  return viewport.panes[activeViewportPaneIndex(viewport)];
}

std::optional<std::size_t> viewportPaneIndexAtCursor(
    const FrameViewport& viewport, double cursorX, double cursorY)
{
  if (!std::isfinite(cursorX) || !std::isfinite(cursorY)) {
    return std::nullopt;
  }
  for (std::size_t i = 0;
       i < viewport.paneCount && i < dart::gui::kMaxViewportPanes;
       ++i) {
    const ViewportPaneFrame& pane = viewport.panes[i];
    if (pane.width <= 0 || pane.height <= 0) {
      continue;
    }
    if (cursorX >= pane.x && cursorX < pane.x + pane.width && cursorY >= pane.y
        && cursorY < pane.y + pane.height) {
      return i;
    }
  }
  return std::nullopt;
}

const ViewportPaneFrame* viewportPaneAtCursor(
    const FrameViewport& viewport, double cursorX, double cursorY)
{
  const std::optional<std::size_t> index
      = viewportPaneIndexAtCursor(viewport, cursorX, cursorY);
  return index.has_value() ? &viewport.panes[*index] : nullptr;
}

std::string_view viewportPaneDisplayName(dart::gui::ViewportPaneKind kind)
{
  switch (kind) {
    case dart::gui::ViewportPaneKind::Perspective:
      return "Perspective";
    case dart::gui::ViewportPaneKind::Top:
      return "Top";
    case dart::gui::ViewportPaneKind::Front:
      return "Front";
    case dart::gui::ViewportPaneKind::Right:
      return "Right";
  }
  return "Viewport";
}

ViewportPaneLabelState viewportPaneLabelState(
    const FrameViewport& viewport, std::size_t paneIndex)
{
  ViewportPaneLabelState label;
  if (paneIndex >= viewport.paneCount
      || paneIndex >= dart::gui::kMaxViewportPanes) {
    label.text = "Viewport";
    return label;
  }

  label.text = viewportPaneDisplayName(viewport.panes[paneIndex].kind);
  label.active = paneIndex == activeViewportPaneIndex(viewport);
  return label;
}

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
    const dart::gui::ViewportLayoutOptions& layoutOptions)
{
  (void)guiScale;
  int width = defaultWidth;
  int height = defaultHeight;
  if (window != nullptr) {
    glfwGetFramebufferSize(window, &width, &height);
  }
  FrameViewport viewport = makeFrameViewport(layoutOptions, width, height);

  imguiIo.DisplaySize = ImVec2(
      static_cast<float>(viewport.width), static_cast<float>(viewport.height));
  imguiIo.DeltaTime = static_cast<float>(worldTimeStep);
  for (std::size_t i = 0; i < viewport.paneCount; ++i) {
    if (views[i] == nullptr || cameras[i] == nullptr) {
      continue;
    }
    const ViewportPaneFrame& pane = viewport.panes[i];
    configureViewportCamera(
        *views[i],
        *cameras[i],
        pane.camera,
        pane.x,
        viewport.height - pane.y - pane.height,
        pane.width,
        pane.height);
  }

  if (window != nullptr) {
    double cursorX = 0.0;
    double cursorY = 0.0;
    getFramebufferCursorPosition(
        window, viewport.width, viewport.height, cursorX, cursorY);
    const std::optional<std::size_t> cursorPane
        = viewportPaneIndexAtCursor(viewport, cursorX, cursorY);
    const bool cursorOverActivePane
        = cursorPane.has_value()
          && *cursorPane == activeViewportPaneIndex(viewport);
    const bool suppressCameraOrbit
        = selectionController.isDraggingSelection()
          || isSceneMouseInputCapturedByUi(showUi, imguiIo)
          || !cursorOverActivePane;
    updateCameraController(
        window, cameraController, suppressCameraOrbit, cameraControls);
  }

  return viewport;
}

} // namespace dart::gui::detail
