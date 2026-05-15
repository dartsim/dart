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

#include <dart/gui/experimental/viewer.hpp>

#include <algorithm>
#include <fstream>
#include <limits>

#include <cmath>

namespace dart::gui::experimental {

void normalizeRunOptions(RunOptions& options)
{
  options.width = std::max(1, options.width);
  options.height = std::max(1, options.height);
  if (!std::isfinite(options.guiScale) || options.guiScale <= 0.0) {
    options.guiScale = 1.0;
  }
  options.guiScale = std::clamp(options.guiScale, 0.5, 4.0);
  if (options.headless && options.maxFrames < 0) {
    options.maxFrames = 1;
  }
  if (!options.screenshotPath.empty() && options.maxFrames < 0) {
    options.maxFrames = 1;
  }
}

bool shouldRequestScreenshot(
    const RunOptions& options, int renderedFrames, bool screenshotRequested)
{
  if (options.screenshotPath.empty() || screenshotRequested) {
    return false;
  }
  if (options.maxFrames < 0) {
    return true;
  }
  return renderedFrames + 1 >= options.maxFrames;
}

bool shouldStopAfterFrame(const RunOptions& options, int renderedFrames)
{
  return options.maxFrames >= 0 && renderedFrames >= options.maxFrames;
}

void togglePaused(ViewerLifecycleState& state)
{
  state.paused = !state.paused;
}

void requestSingleStep(ViewerLifecycleState& state, bool pause)
{
  if (pause) {
    state.paused = true;
  }
  state.stepOnce = true;
}

bool shouldAdvanceSimulation(const ViewerLifecycleState& state)
{
  return !state.paused || state.stepOnce;
}

void markSimulationAdvanced(ViewerLifecycleState& state)
{
  state.stepOnce = false;
}

bool shouldRequestScreenshot(
    const RunOptions& options, const ViewerLifecycleState& state)
{
  return shouldRequestScreenshot(
      options, state.renderedFrames, state.screenshotRequested);
}

void markScreenshotRequested(ViewerLifecycleState& state)
{
  state.screenshotRequested = true;
}

void markFrameRendered(ViewerLifecycleState& state)
{
  if (state.renderedFrames < std::numeric_limits<int>::max()) {
    ++state.renderedFrames;
  }
  state.skippedFrames = 0;
}

void markFrameSkipped(ViewerLifecycleState& state)
{
  if (state.skippedFrames < std::numeric_limits<int>::max()) {
    ++state.skippedFrames;
  }
}

bool shouldStopAfterFrame(
    const RunOptions& options, const ViewerLifecycleState& state)
{
  return shouldStopAfterFrame(options, state.renderedFrames);
}

bool writeRgbaPpm(
    const std::string& path,
    std::uint32_t width,
    std::uint32_t height,
    const std::vector<std::uint8_t>& rgbaPixels,
    bool originBottomLeft,
    std::string* errorMessage)
{
  const auto fail = [errorMessage](const std::string& message) {
    if (errorMessage != nullptr) {
      *errorMessage = message;
    }
    return false;
  };

  if (width == 0 || height == 0) {
    return fail("PPM dimensions must be nonzero");
  }

  constexpr std::size_t channels = 4;
  const auto maxSize = std::numeric_limits<std::size_t>::max();
  const auto imageWidth = static_cast<std::size_t>(width);
  const auto imageHeight = static_cast<std::size_t>(height);
  if (imageWidth > maxSize / imageHeight
      || imageWidth * imageHeight > maxSize / channels) {
    return fail("PPM image dimensions overflow the addressable buffer size");
  }

  const std::size_t expectedSize = imageWidth * imageHeight * channels;
  if (rgbaPixels.size() != expectedSize) {
    return fail("PPM RGBA buffer size does not match width * height * 4 bytes");
  }

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return fail("Failed to open PPM output path: " + path);
  }

  out << "P6\n" << width << " " << height << "\n255\n";
  for (std::uint32_t y = 0; y < height; ++y) {
    const std::uint32_t sourceY = originBottomLeft ? height - 1 - y : y;
    const std::size_t row = static_cast<std::size_t>(sourceY) * imageWidth * 4;
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto* pixel = &rgbaPixels[row + static_cast<std::size_t>(x) * 4];
      out.write(reinterpret_cast<const char*>(pixel), 3);
    }
  }

  if (!out) {
    return fail("Failed while writing PPM output path: " + path);
  }

  return true;
}

OrbitCameraBasis makeOrbitCameraBasis(const OrbitCamera& camera)
{
  OrbitCameraBasis basis;
  basis.eye = cameraEye(camera);
  basis.forward = camera.target - basis.eye;
  if (basis.forward.squaredNorm() < 1e-12) {
    basis.forward = -Eigen::Vector3d::UnitX();
  } else {
    basis.forward.normalize();
  }

  const Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();
  basis.right = basis.forward.cross(worldUp);
  if (basis.right.squaredNorm() < 1e-12) {
    basis.right = Eigen::Vector3d::UnitX();
  } else {
    basis.right.normalize();
  }
  basis.up = basis.right.cross(basis.forward).normalized();
  return basis;
}

Eigen::Vector3d cameraEye(const OrbitCamera& camera)
{
  return camera.target
         + Eigen::Vector3d(
             camera.distance * std::cos(camera.pitch) * std::cos(camera.yaw),
             camera.distance * std::cos(camera.pitch) * std::sin(camera.yaw),
             camera.distance * std::sin(camera.pitch));
}

void updateOrbitCamera(OrbitCamera& camera, const OrbitCameraUpdate& update)
{
  if (update.orbit) {
    camera.yaw -= update.deltaX * update.orbitScale;
    camera.pitch += update.deltaY * update.orbitScale;
  }

  const double minPitch = std::min(update.minPitch, update.maxPitch);
  const double maxPitch = std::max(update.minPitch, update.maxPitch);
  camera.pitch = std::clamp(camera.pitch, minPitch, maxPitch);

  if (update.pan) {
    const OrbitCameraBasis basis = makeOrbitCameraBasis(camera);
    const double panScale = camera.distance * update.panScale;
    camera.target -= basis.right * update.deltaX * panScale;
    camera.target += basis.up * update.deltaY * panScale;
  }

  if (update.scrollDelta != 0.0) {
    camera.distance *= std::exp(-update.scrollDelta * update.scrollScale);
  }

  const double minDistance = std::max(0.0, update.minDistance);
  const double maxDistance = std::max(minDistance, update.maxDistance);
  camera.distance = std::clamp(camera.distance, minDistance, maxDistance);
}

void addOrbitCameraScroll(OrbitCameraController& controller, double scrollDelta)
{
  if (std::isfinite(scrollDelta)) {
    controller.scrollDelta += scrollDelta;
  }
}

void resetOrbitCameraTracking(OrbitCameraController& controller)
{
  controller.hasLastCursor = false;
}

void updateOrbitCameraController(
    OrbitCameraController& controller, const OrbitCameraControllerInput& input)
{
  const bool hasCursor = input.hasCursor && std::isfinite(input.cursorX)
                         && std::isfinite(input.cursorY);
  double dx = 0.0;
  double dy = 0.0;
  if (hasCursor) {
    if (!controller.hasLastCursor) {
      controller.lastCursorX = input.cursorX;
      controller.lastCursorY = input.cursorY;
      controller.hasLastCursor = true;
    }
    dx = input.cursorX - controller.lastCursorX;
    dy = input.cursorY - controller.lastCursorY;
    controller.lastCursorX = input.cursorX;
    controller.lastCursorY = input.cursorY;
  } else {
    resetOrbitCameraTracking(controller);
  }

  OrbitCameraUpdate update;
  update.deltaX = dx;
  update.deltaY = dy;
  update.scrollDelta = controller.scrollDelta;
  update.orbit = hasCursor && input.orbit;
  update.pan = hasCursor && input.pan;
  updateOrbitCamera(controller.camera, update);
  controller.scrollDelta = 0.0;
}

Eigen::Vector3d computeCameraRelativeNudge(
    const OrbitCamera& camera, const DirectionalNudgeInput& input)
{
  if (!std::isfinite(input.stepSize) || input.stepSize <= 0.0) {
    return Eigen::Vector3d::Zero();
  }

  const double fastMultiplier
      = std::isfinite(input.fastMultiplier) && input.fastMultiplier > 0.0
            ? input.fastMultiplier
            : 1.0;
  const double step = input.stepSize * (input.fast ? fastMultiplier : 1.0);
  if (!std::isfinite(step) || step <= 0.0) {
    return Eigen::Vector3d::Zero();
  }

  const auto basis = makeOrbitCameraBasis(camera);
  Eigen::Vector3d forward = basis.forward;
  forward.z() = 0.0;
  if (forward.squaredNorm() < 1e-12) {
    forward = -Eigen::Vector3d::UnitY();
  } else {
    forward.normalize();
  }

  Eigen::Vector3d right = basis.right;
  right.z() = 0.0;
  if (right.squaredNorm() < 1e-12) {
    right = Eigen::Vector3d::UnitX();
  } else {
    right.normalize();
  }

  Eigen::Vector3d nudge = Eigen::Vector3d::Zero();
  if (input.left) {
    nudge -= right * step;
  }
  if (input.right) {
    nudge += right * step;
  }
  if (input.forward) {
    nudge += forward * step;
  }
  if (input.backward) {
    nudge -= forward * step;
  }
  if (input.up) {
    nudge += Eigen::Vector3d::UnitZ() * step;
  }
  if (input.down) {
    nudge -= Eigen::Vector3d::UnitZ() * step;
  }
  return nudge;
}

PickRay makePerspectivePickRay(
    const OrbitCamera& camera,
    double cursorX,
    double cursorY,
    int width,
    int height,
    double verticalFovRadians)
{
  if (!std::isfinite(verticalFovRadians) || verticalFovRadians <= 0.0) {
    verticalFovRadians = 0.7853981633974483;
  }

  const int safeWidth = std::max(1, width);
  const int safeHeight = std::max(1, height);
  const OrbitCameraBasis basis = makeOrbitCameraBasis(camera);
  const double aspect
      = static_cast<double>(safeWidth) / static_cast<double>(safeHeight);
  const double ndcX = (2.0 * cursorX / static_cast<double>(safeWidth)) - 1.0;
  const double ndcY = 1.0 - (2.0 * cursorY / static_cast<double>(safeHeight));
  const double halfHeight = std::tan(verticalFovRadians * 0.5);

  PickRay ray;
  ray.origin = basis.eye;
  ray.direction = (basis.forward + basis.right * ndcX * aspect * halfHeight
                   + basis.up * ndcY * halfHeight)
                      .normalized();
  return ray;
}

PerspectiveProjection makePerspectiveProjection(
    const OrbitCamera& camera,
    int width,
    int height,
    const ProjectionOptions& options)
{
  const int safeWidth = std::max(1, width);
  const int safeHeight = std::max(1, height);
  const double safeDistance
      = std::isfinite(camera.distance) ? std::max(0.0, camera.distance) : 0.0;
  const double defaultNearScale = ProjectionOptions{}.nearScale;
  const double nearScale
      = std::isfinite(options.nearScale) && options.nearScale > 0.0
            ? options.nearScale
            : defaultNearScale;
  const double minNear = std::max(
      1e-6,
      std::isfinite(options.minNearPlane) && options.minNearPlane > 0.0
          ? options.minNearPlane
          : ProjectionOptions{}.minNearPlane);
  const double maxNearCandidate
      = std::isfinite(options.maxNearPlane) && options.maxNearPlane > 0.0
            ? options.maxNearPlane
            : ProjectionOptions{}.maxNearPlane;
  const double maxNear = std::max(minNear, maxNearCandidate);
  const double defaultNear
      = std::clamp(safeDistance * nearScale, minNear, maxNear);

  PerspectiveProjection projection;
  projection.aspectRatio
      = static_cast<double>(safeWidth) / static_cast<double>(safeHeight);
  projection.verticalFovDegrees = std::isfinite(options.verticalFovDegrees)
                                          && options.verticalFovDegrees > 0.0
                                          && options.verticalFovDegrees < 180.0
                                      ? options.verticalFovDegrees
                                      : ProjectionOptions{}.verticalFovDegrees;
  projection.nearPlane = options.nearPlane.has_value()
                                 && std::isfinite(*options.nearPlane)
                                 && *options.nearPlane > 0.0
                             ? *options.nearPlane
                             : defaultNear;

  const double minFar
      = std::isfinite(options.minFarPlane) && options.minFarPlane > 0.0
            ? options.minFarPlane
            : ProjectionOptions{}.minFarPlane;
  const double farPadding
      = std::isfinite(options.farPadding) && options.farPadding >= 0.0
            ? options.farPadding
            : ProjectionOptions{}.farPadding;
  const double defaultFar = std::max(
      std::max(minFar, safeDistance + farPadding), projection.nearPlane + 1.0);
  projection.farPlane = options.farPlane.has_value()
                                && std::isfinite(*options.farPlane)
                                && *options.farPlane > projection.nearPlane
                            ? *options.farPlane
                            : defaultFar;
  return projection;
}

} // namespace dart::gui::experimental
