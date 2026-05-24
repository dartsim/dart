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

#include <cctype>

namespace dart::gui::detail {

using dart::gui::addOrbitCameraScroll;
using dart::gui::computeCameraRelativeNudge;
using dart::gui::DirectionalNudgeInput;
using dart::gui::KeyboardActionContext;
using dart::gui::KeyboardActionTrigger;
using dart::gui::KeyboardKey;
using dart::gui::KeyboardShortcut;
using dart::gui::OrbitCamera;
using dart::gui::OrbitCameraController;
using dart::gui::OrbitCameraControllerInput;
using dart::gui::requestSingleStep;
using dart::gui::resetOrbitCameraTracking;
using dart::gui::togglePaused;
using dart::gui::updateOrbitCameraController;

namespace {

void handleScroll(GLFWwindow* window, double, double yOffset)
{
  if (ImGui::GetCurrentContext() != nullptr
      && ImGui::GetIO().WantCaptureMouse) {
    return;
  }

  auto* controller = static_cast<dart::gui::OrbitCameraController*>(
      glfwGetWindowUserPointer(window));
  if (controller != nullptr) {
    addOrbitCameraScroll(*controller, yOffset);
  }
}

int glfwKeyForShortcut(const KeyboardShortcut& shortcut)
{
  if (shortcut.character != '\0') {
    const unsigned char value = static_cast<unsigned char>(shortcut.character);
    if (std::isalpha(value)) {
      return std::toupper(value);
    }

    switch (shortcut.character) {
      case '[':
      case '{':
        return GLFW_KEY_LEFT_BRACKET;
      case ']':
      case '}':
        return GLFW_KEY_RIGHT_BRACKET;
      case '/':
      case '?':
        return GLFW_KEY_SLASH;
      case '\\':
        return GLFW_KEY_BACKSLASH;
      default:
        break;
    }

    return value;
  }

  switch (shortcut.key) {
    case KeyboardKey::Tab:
      return GLFW_KEY_TAB;
    case KeyboardKey::Enter:
      return GLFW_KEY_ENTER;
    case KeyboardKey::Backspace:
      return GLFW_KEY_BACKSPACE;
    case KeyboardKey::Delete:
      return GLFW_KEY_DELETE;
    case KeyboardKey::Up:
      return GLFW_KEY_UP;
    case KeyboardKey::Down:
      return GLFW_KEY_DOWN;
    case KeyboardKey::Left:
      return GLFW_KEY_LEFT;
    case KeyboardKey::Right:
      return GLFW_KEY_RIGHT;
    case KeyboardKey::PageUp:
      return GLFW_KEY_PAGE_UP;
    case KeyboardKey::PageDown:
      return GLFW_KEY_PAGE_DOWN;
    case KeyboardKey::GraveAccent:
      return GLFW_KEY_GRAVE_ACCENT;
    case KeyboardKey::Unknown:
      return GLFW_KEY_UNKNOWN;
  }

  return GLFW_KEY_UNKNOWN;
}

bool isShiftDown(GLFWwindow* window)
{
  return glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
         || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}

bool isShortcutDown(GLFWwindow* window, const KeyboardShortcut& shortcut)
{
  const int glfwKey = glfwKeyForShortcut(shortcut);
  if (glfwKey == GLFW_KEY_UNKNOWN
      || glfwGetKey(window, glfwKey) != GLFW_PRESS) {
    return false;
  }

  switch (shortcut.character) {
    case '{':
    case '}':
    case '?':
      return isShiftDown(window);
    case '[':
    case ']':
    case '/':
      return !isShiftDown(window);
    default:
      break;
  }

  const unsigned char character
      = static_cast<unsigned char>(shortcut.character);
  if (std::isalpha(character)) {
    return (std::isupper(character) != 0) == isShiftDown(window);
  }

  return true;
}

void dispatchKeyboardActions(
    GLFWwindow* window,
    DartScene& scene,
    dart::gui::ViewerLifecycleState& lifecycle,
    OrbitCameraController& cameraController,
    const OrbitCamera& homeCamera,
    ApplicationInputState& state)
{
  if (state.customActionWasPressed.size() != scene.keyboardActions.size()) {
    state.customActionWasPressed.assign(scene.keyboardActions.size(), false);
  }

  KeyboardActionContext context;
  context.lifecycle = &lifecycle;
  context.renderSettings = &scene.renderSettings;
  context.resetCamera = [&cameraController, homeCamera]() {
    cameraController.camera = homeCamera;
    resetOrbitCameraTracking(cameraController);
  };

  for (std::size_t i = 0; i < scene.keyboardActions.size(); ++i) {
    const auto& action = scene.keyboardActions[i];
    const bool isPressed = isShortcutDown(window, action.shortcut);
    const bool wasPressed = state.customActionWasPressed[i];
    const bool shouldTrigger
        = action.trigger == KeyboardActionTrigger::Release
              ? (!isPressed && wasPressed)
              : (isPressed && (action.repeat || !wasPressed));
    if (shouldTrigger && action.callback) {
      action.callback(context);
    }
    state.customActionWasPressed[i] = isPressed;
  }
}

} // namespace

void attachOrbitCameraController(
    GLFWwindow* window, dart::gui::OrbitCameraController& controller)
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
    dart::gui::ViewerLifecycleState& lifecycle,
    OrbitCameraController& cameraController,
    const OrbitCamera& homeCamera,
    ApplicationInputState& state,
    bool& showPerfHud)
{
  if (window == nullptr) {
    return;
  }

  glfwPollEvents();
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }

  const bool isSpacePressed = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
  if (isSpacePressed && !state.wasSpacePressed) {
    togglePaused(lifecycle);
  }
  state.wasSpacePressed = isSpacePressed;

  // F2 toggles the performance HUD at runtime (no need to pass --perf-hud).
  const bool isPerfHudKeyPressed
      = glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS;
  if (isPerfHudKeyPressed && !state.wasPerfHudKeyPressed) {
    showPerfHud = !showPerfHud;
  }
  state.wasPerfHudKeyPressed = isPerfHudKeyPressed;

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

  dispatchKeyboardActions(
      window, scene, lifecycle, cameraController, homeCamera, state);
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
  const float xScale = windowWidth > 0 ? static_cast<float>(framebufferWidth)
                                             / static_cast<float>(windowWidth)
                                       : 1.0f;
  const float yScale = windowHeight > 0 ? static_cast<float>(framebufferHeight)
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

bool isSceneMouseInputCapturedByUi(bool showUi, const ImGuiIO& io)
{
  return showUi && io.WantCaptureMouse;
}

bool isDragModifierDown(GLFWwindow* window)
{
  return isKeyDown(window, GLFW_KEY_LEFT_CONTROL)
         || isKeyDown(window, GLFW_KEY_RIGHT_CONTROL);
}

bool isRotationDragModifierDown(GLFWwindow* window)
{
  return isDragModifierDown(window) && isShiftDown(window);
}

std::optional<Eigen::Vector3d> selectedDragAxisFromKeyboard(GLFWwindow* window)
{
  if (isKeyDown(window, GLFW_KEY_X) || isKeyDown(window, GLFW_KEY_1)) {
    return Eigen::Vector3d::UnitX();
  }
  if (isKeyDown(window, GLFW_KEY_Y) || isKeyDown(window, GLFW_KEY_2)) {
    return Eigen::Vector3d::UnitY();
  }
  if (isKeyDown(window, GLFW_KEY_Z) || isKeyDown(window, GLFW_KEY_3)) {
    return Eigen::Vector3d::UnitZ();
  }

  return std::nullopt;
}

Eigen::Vector3d selectedNudgeFromKeyboard(
    GLFWwindow* window, const dart::gui::OrbitCamera& camera, double stepSize)
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
    dart::gui::OrbitCameraController& controller,
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

} // namespace dart::gui::detail
