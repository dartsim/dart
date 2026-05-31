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

#ifndef DART_GUI_DETAIL_INPUT_HPP_
#define DART_GUI_DETAIL_INPUT_HPP_

#include <dart/gui/viewer.hpp>

#include <Eigen/Core>

#include <optional>
#include <vector>

struct GLFWwindow;
struct ImGuiIO;

namespace dart::gui::detail {

struct DartScene;
class SelectionController;

struct ApplicationInputState
{
  bool wasSpacePressed = false;
  bool wasStepPressed = false;
  bool wasPerfHudKeyPressed = false;
  std::vector<bool> customActionWasPressed;
};

void attachOrbitCameraController(
    GLFWwindow* window, dart::gui::OrbitCameraController& controller);

bool isKeyDown(GLFWwindow* window, int key);

void pollApplicationInput(
    GLFWwindow* window,
    DartScene& scene,
    SelectionController& selectionController,
    dart::gui::ViewerLifecycleState& lifecycle,
    dart::gui::OrbitCameraController& cameraController,
    const dart::gui::OrbitCamera& homeCamera,
    ApplicationInputState& state,
    bool& showPerfHud);

void updateImGuiMouseInput(
    GLFWwindow* window,
    ImGuiIO& io,
    int framebufferWidth,
    int framebufferHeight);

void updateImGuiMouseCursor(GLFWwindow* window, ImGuiIO& io);

void destroyImGuiMouseCursors(GLFWwindow* window);

bool isSceneMouseInputCapturedByUi(bool showUi, const ImGuiIO& io);

bool isDragModifierDown(GLFWwindow* window);

bool isRotationDragModifierDown(GLFWwindow* window);

std::optional<Eigen::Vector3d> selectedDragAxisFromKeyboard(GLFWwindow* window);

Eigen::Vector3d selectedNudgeFromKeyboard(
    GLFWwindow* window, const dart::gui::OrbitCamera& camera, double stepSize);

dart::gui::OrbitCameraControllerInput makeOrbitCameraControllerInput(
    double cursorX,
    double cursorY,
    bool leftMousePressed,
    bool rightMousePressed,
    bool middleMousePressed,
    bool suppressCameraMouse,
    bool dragModifierDown,
    const dart::gui::OrbitCameraControlOptions& controls);

void updateCameraController(
    GLFWwindow* window,
    dart::gui::OrbitCameraController& controller,
    bool suppressCameraMouse = false,
    const dart::gui::OrbitCameraControlOptions& controls
    = dart::gui::OrbitCameraControlOptions{});

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_INPUT_HPP_
