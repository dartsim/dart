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

#include <dart/gui/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <vector>

#include <cstdint>

namespace dart::gui {

struct PickRay
{
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
};

struct RunOptions
{
  std::string windowTitle = "dartsim";
  int width = 1280;
  int height = 720;
  int maxFrames = -1;
  double guiScale = 1.0;
  bool headless = false;
  std::string screenshotPath;
  std::string frameOutputDirectory;
};

struct RenderSettings
{
  bool shadowsEnabled = true;
};

struct ViewerLifecycleState
{
  int renderedFrames = 0;
  int skippedFrames = 0;
  bool paused = false;
  bool stepOnce = false;
  bool screenshotRequested = false;
  bool exitRequested = false;
};

struct OrbitCamera
{
  Eigen::Vector3d target = Eigen::Vector3d(0.0, 0.0, 0.45);
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  double yaw = -1.03;
  double pitch = 0.29;
  double distance = 6.1;
};

struct OrbitCameraBasis
{
  Eigen::Vector3d eye = Eigen::Vector3d::Zero();
  Eigen::Vector3d forward = Eigen::Vector3d::UnitX();
  Eigen::Vector3d right = Eigen::Vector3d::UnitY();
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
};

struct OrbitCameraUpdate
{
  double deltaX = 0.0;
  double deltaY = 0.0;
  double scrollDelta = 0.0;
  bool orbit = false;
  bool pan = false;
  double orbitScale = 0.006;
  double panScale = 0.0015;
  double scrollScale = 0.12;
  double minDistance = 0.35;
  double maxDistance = 80.0;
  double minPitch = -1.45;
  double maxPitch = 1.45;
};

struct OrbitCameraController
{
  OrbitCamera camera;
  double lastCursorX = 0.0;
  double lastCursorY = 0.0;
  double scrollDelta = 0.0;
  bool hasLastCursor = false;
};

struct OrbitCameraControllerInput
{
  double cursorX = 0.0;
  double cursorY = 0.0;
  bool hasCursor = true;
  bool orbit = false;
  bool pan = false;
};

struct DirectionalNudgeInput
{
  bool left = false;
  bool right = false;
  bool forward = false;
  bool backward = false;
  bool up = false;
  bool down = false;
  bool fast = false;
  double stepSize = 0.035;
  double fastMultiplier = 3.0;
};

struct ProjectionOptions
{
  double verticalFovDegrees = 45.0;
  std::optional<double> nearPlane;
  std::optional<double> farPlane;
  double nearScale = 0.004;
  double minNearPlane = 0.002;
  double maxNearPlane = 0.025;
  double minFarPlane = 30.0;
  double farPadding = 35.0;
};

struct PerspectiveProjection
{
  double verticalFovDegrees = 45.0;
  double aspectRatio = 1.0;
  double nearPlane = 0.002;
  double farPlane = 30.0;
};

DART_GUI_API void normalizeRunOptions(RunOptions& options);

DART_GUI_API bool shouldRequestScreenshot(
    const RunOptions& options, int renderedFrames, bool screenshotRequested);

DART_GUI_API bool shouldCaptureFrameOutput(const RunOptions& options);

DART_GUI_API std::string makeFrameOutputPath(
    const RunOptions& options, int frameNumber);

DART_GUI_API bool shouldStopAfterFrame(
    const RunOptions& options, int renderedFrames);

DART_GUI_API void togglePaused(ViewerLifecycleState& state);

DART_GUI_API void requestSingleStep(
    ViewerLifecycleState& state, bool pause = true);

DART_GUI_API void requestExit(ViewerLifecycleState& state);

DART_GUI_API bool shouldAdvanceSimulation(const ViewerLifecycleState& state);

DART_GUI_API void markSimulationAdvanced(ViewerLifecycleState& state);

DART_GUI_API bool shouldRequestScreenshot(
    const RunOptions& options, const ViewerLifecycleState& state);

DART_GUI_API void markScreenshotRequested(ViewerLifecycleState& state);

DART_GUI_API void markFrameRendered(ViewerLifecycleState& state);

DART_GUI_API void markFrameSkipped(ViewerLifecycleState& state);

DART_GUI_API bool shouldStopAfterFrame(
    const RunOptions& options, const ViewerLifecycleState& state);

DART_GUI_API bool writeRgbaPpm(
    const std::string& path,
    std::uint32_t width,
    std::uint32_t height,
    const std::vector<std::uint8_t>& rgbaPixels,
    bool originBottomLeft = false,
    std::string* errorMessage = nullptr);

DART_GUI_API OrbitCameraBasis makeOrbitCameraBasis(const OrbitCamera& camera);

DART_GUI_API Eigen::Vector3d cameraEye(const OrbitCamera& camera);

DART_GUI_API void updateOrbitCamera(
    OrbitCamera& camera, const OrbitCameraUpdate& update);

DART_GUI_API void addOrbitCameraScroll(
    OrbitCameraController& controller, double scrollDelta);

DART_GUI_API void resetOrbitCameraTracking(OrbitCameraController& controller);

DART_GUI_API void updateOrbitCameraController(
    OrbitCameraController& controller, const OrbitCameraControllerInput& input);

DART_GUI_API Eigen::Vector3d computeCameraRelativeNudge(
    const OrbitCamera& camera, const DirectionalNudgeInput& input);

DART_GUI_API PickRay makePerspectivePickRay(
    const OrbitCamera& camera,
    double cursorX,
    double cursorY,
    int width,
    int height,
    double verticalFovRadians = 0.7853981633974483);

DART_GUI_API PerspectiveProjection makePerspectiveProjection(
    const OrbitCamera& camera,
    int width,
    int height,
    const ProjectionOptions& options = {});

} // namespace dart::gui

#endif // DART_GUI_VIEWER_HPP_
