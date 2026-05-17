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

namespace dart::gui::detail {

FrameViewport updateFrameViewport(
    GLFWwindow* window,
    ::filament::View& view,
    ::filament::Camera& camera,
    dart::gui::OrbitCameraController& cameraController,
    const SelectionController& selectionController,
    ImGuiIO& imguiIo,
    int defaultWidth,
    int defaultHeight,
    double worldTimeStep,
    bool showUi,
    double guiScale)
{
  FrameViewport viewport{defaultWidth, defaultHeight};
  if (window != nullptr) {
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
  }
  viewport.width = std::max(1, viewport.width);
  viewport.height = std::max(1, viewport.height);

  imguiIo.DisplaySize = ImVec2(
      static_cast<float>(viewport.width), static_cast<float>(viewport.height));
  imguiIo.DeltaTime = static_cast<float>(worldTimeStep);
  configureViewportCamera(
      view, camera, cameraController.camera, viewport.width, viewport.height);

  if (window != nullptr) {
    double cursorX = 0.0;
    double cursorY = 0.0;
    glfwGetCursorPos(window, &cursorX, &cursorY);
    const bool suppressCameraOrbit
        = selectionController.isDraggingSelection()
          || (showUi && isInsideStatusPanel(cursorX, cursorY, guiScale));
    updateCameraController(window, cameraController, suppressCameraOrbit);
  }

  return viewport;
}

} // namespace dart::gui::detail
