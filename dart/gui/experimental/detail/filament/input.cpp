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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; OR
 *   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *   DAMAGE.
 */

#include "input.hpp"

#include "scenes.hpp"
#include "selection.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <limits>

namespace dart::gui::experimental::filament {

using dart::gui::experimental::DirectionalNudgeInput;
using dart::gui::experimental::OrbitCameraControllerInput;
using dart::gui::experimental::addOrbitCameraScroll;
using dart::gui::experimental::computeCameraRelativeNudge;
using dart::gui::experimental::requestSingleStep;
using dart::gui::experimental::togglePaused;
using dart::gui::experimental::updateOrbitCameraController;

namespace {

void handleScroll(GLFWwindow* window, double, double yOffset)
{
  auto* controller
      = static_cast<dart::gui::experimental::OrbitCameraController*>(
          glfwGetWindowUserPointer(window));
  if (controller != nullptr) {
    addOrbitCameraScroll(*controller, yOffset);
  }
}

} // namespace

void attachOrbitCameraController(
    GLFWwindow* window,
    dart::gui::experimental::OrbitCameraController& controller)
{
  if (window == nullptr) {
    return;
  }

  glfwSetWindowUserPointer(window, &controller);
  glfwSetScrollCallback(window, handleScroll);
}

bool isKeyDown(GLFWwindow* window, int key)
{
  return window != nullptr && glfwGetKey(window, key) == GLFW_PRESS;
}

void pollApplicationInput(
    GLFWwindow* window,
    DartScene& scene,
    SelectionController& selectionController,
    dart::gui::experimental::ViewerLifecycleState& lifecycle,
    ApplicationInputState& state)
{
  if (window == nullptr) {
    return;
  }

  glfwPollEvents();
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }

  const bool isSpacePressed
      = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
  if (isSpacePressed && !state.wasSpacePressed) {
    togglePaused(lifecycle);
  }
  state.wasSpacePressed = isSpacePressed;

  const bool isStepPressed = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
  if (isStepPressed && !state.wasStepPressed) {
    requestSingleStep(lifecycle, false);
  }
  state.wasStepPressed = isStepPressed;

  for (const auto& handle : scene.ikHandles) {
    if (glfwGetKey(window, handle.hotkey) == GLFW_PRESS) {
      selectionController.select(
          handle.targetRenderableId, handle.label + " IK target");
      lifecycle.paused = true;
    }
  }
}

void updateImGuiMouseInput(
    GLFWwindow* window,
    ImGuiIO& io,
    int framebufferWidth,
    int framebufferHeight)
{
  for (bool& mouseDown : io.MouseDown) {
    mouseDown = false;
  }

  if (window == nullptr) {
    const float offscreen = -std::numeric_limits<float>::max();
    io.MousePos = ImVec2(offscreen, offscreen);
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);
    return;
  }

  int windowWidth = framebufferWidth;
  int windowHeight = framebufferHeight;
  glfwGetWindowSize(window, &windowWidth, &windowHeight);
  const float xScale
      = windowWidth > 0
            ? static_cast<float>(framebufferWidth)
                  / static_cast<float>(windowWidth)
            : 1.0f;
  const float yScale
      = windowHeight > 0
            ? static_cast<float>(framebufferHeight)
                  / static_cast<float>(windowHeight)
            : 1.0f;
  io.DisplayFramebufferScale = ImVec2(xScale, yScale);

  double cursorX = 0.0;
  double cursorY = 0.0;
  glfwGetCursorPos(window, &cursorX, &cursorY);
  io.MousePos = ImVec2(
      static_cast<float>(cursorX) * xScale,
      static_cast<float>(cursorY) * yScale);
  io.MouseDown[0]
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  io.MouseDown[1]
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  io.MouseDown[2]
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
}

bool isInsideStatusPanel(double cursorX, double cursorY, double guiScale)
{
  return cursorX >= 20.0 * guiScale && cursorX <= 360.0 * guiScale
         && cursorY >= 20.0 * guiScale && cursorY <= 360.0 * guiScale;
}

bool isDragModifierDown(GLFWwindow* window)
{
  return isKeyDown(window, GLFW_KEY_LEFT_CONTROL)
         || isKeyDown(window, GLFW_KEY_RIGHT_CONTROL);
}

std::optional<Eigen::Vector3d> selectedDragAxisFromKeyboard(GLFWwindow* window)
{
  if (isKeyDown(window, GLFW_KEY_X)) {
    return Eigen::Vector3d::UnitX();
  }
  if (isKeyDown(window, GLFW_KEY_Y)) {
    return Eigen::Vector3d::UnitY();
  }
  if (isKeyDown(window, GLFW_KEY_Z)) {
    return Eigen::Vector3d::UnitZ();
  }

  return std::nullopt;
}

Eigen::Vector3d selectedNudgeFromKeyboard(
    GLFWwindow* window,
    const dart::gui::experimental::OrbitCamera& camera,
    double stepSize)
{
  if (window == nullptr) {
    return Eigen::Vector3d::Zero();
  }

  DirectionalNudgeInput input;
  input.left = isKeyDown(window, GLFW_KEY_LEFT);
  input.right = isKeyDown(window, GLFW_KEY_RIGHT);
  input.forward = isKeyDown(window, GLFW_KEY_UP);
  input.backward = isKeyDown(window, GLFW_KEY_DOWN);
  input.up = isKeyDown(window, GLFW_KEY_PAGE_UP);
  input.down = isKeyDown(window, GLFW_KEY_PAGE_DOWN);
  input.fast = isKeyDown(window, GLFW_KEY_LEFT_SHIFT)
               || isKeyDown(window, GLFW_KEY_RIGHT_SHIFT);
  input.stepSize = stepSize;
  return computeCameraRelativeNudge(camera, input);
}

void updateCameraController(
    GLFWwindow* window,
    dart::gui::experimental::OrbitCameraController& controller,
    bool suppressLeftMouseOrbit)
{
  if (window == nullptr) {
    return;
  }

  double x = 0.0;
  double y = 0.0;
  glfwGetCursorPos(window, &x, &y);

  OrbitCameraControllerInput input;
  input.cursorX = x;
  input.cursorY = y;
  input.orbit = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS
                && !suppressLeftMouseOrbit && !isDragModifierDown(window);
  input.pan
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS
        || glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  updateOrbitCameraController(controller, input);
}

} // namespace dart::gui::experimental::filament
