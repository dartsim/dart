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

#include <dart/gui/detail/gui_scale.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>
#include <utility>

#include <cmath>

namespace dart::gui {

namespace {

std::string makeFrameOutputPath(
    std::string_view outputDirectory, int frameNumber)
{
  if (outputDirectory.empty()) {
    return {};
  }

  std::ostringstream filename;
  filename << "frame_" << std::setw(6) << std::setfill('0')
           << std::max(0, frameNumber) << ".ppm";
  return (std::filesystem::path(outputDirectory) / filename.str()).string();
}

std::string activeFrameOutputDirectory(
    const RunOptions& options, const ViewerLifecycleState& state)
{
  (void)options;
  if (state.frameOutputEnabled && !state.frameOutputDirectory.empty()) {
    return state.frameOutputDirectory;
  }
  return {};
}

} // namespace

void normalizeRunOptions(RunOptions& options)
{
  if (options.windowTitle.empty()) {
    options.windowTitle = "dartsim";
  }
  options.width = std::max(1, options.width);
  options.height = std::max(1, options.height);
  options.guiScale = detail::normalizeGuiUserScale(options.guiScale);
  if (options.headless && options.maxFrames < 0) {
    options.maxFrames = 1;
  }
  if ((!options.screenshotPath.empty() || !options.frameOutputDirectory.empty())
      && options.maxFrames < 0) {
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

bool shouldCaptureFrameOutput(const RunOptions& options)
{
  return !options.frameOutputDirectory.empty();
}

std::string makeFrameOutputPath(const RunOptions& options, int frameNumber)
{
  return makeFrameOutputPath(options.frameOutputDirectory, frameNumber);
}

void setFrameOutputCapture(
    ViewerLifecycleState& state, std::string outputDirectory, bool enabled)
{
  state.frameOutputDirectory = std::move(outputDirectory);
  state.frameOutputEnabled = enabled && !state.frameOutputDirectory.empty();
}

void toggleFrameOutputCapture(
    ViewerLifecycleState& state, std::string outputDirectory)
{
  const bool enable = !state.frameOutputEnabled
                      || state.frameOutputDirectory != outputDirectory;
  setFrameOutputCapture(state, std::move(outputDirectory), enable);
}

void setRecordedFramePlaybackIndex(
    ViewerLifecycleState& state, int frameCount, int frameIndex)
{
  if (frameCount <= 0) {
    state.recordedFramePlaybackIndex = 0;
    state.recordedFramePlaybackPlaying = false;
    return;
  }

  state.recordedFramePlaybackIndex = std::clamp(frameIndex, 0, frameCount - 1);
}

void stepRecordedFramePlayback(
    ViewerLifecycleState& state, int frameCount, int delta)
{
  setRecordedFramePlaybackIndex(
      state, frameCount, state.recordedFramePlaybackIndex + delta);
}

void toggleRecordedFramePlayback(ViewerLifecycleState& state, int frameCount)
{
  if (frameCount <= 0) {
    state.recordedFramePlaybackPlaying = false;
    state.recordedFramePlaybackIndex = 0;
    return;
  }

  setRecordedFramePlaybackIndex(
      state, frameCount, state.recordedFramePlaybackIndex);
  state.recordedFramePlaybackPlaying = !state.recordedFramePlaybackPlaying;
}

void advanceRecordedFramePlayback(ViewerLifecycleState& state, int frameCount)
{
  if (!state.recordedFramePlaybackPlaying) {
    setRecordedFramePlaybackIndex(
        state, frameCount, state.recordedFramePlaybackIndex);
    return;
  }

  if (frameCount <= 0) {
    state.recordedFramePlaybackPlaying = false;
    state.recordedFramePlaybackIndex = 0;
    return;
  }

  if (state.recordedFramePlaybackIndex + 1 >= frameCount) {
    state.recordedFramePlaybackPlaying = false;
    setRecordedFramePlaybackIndex(state, frameCount, frameCount - 1);
    return;
  }

  setRecordedFramePlaybackIndex(
      state, frameCount, state.recordedFramePlaybackIndex + 1);
}

bool shouldCaptureFrameOutput(
    const RunOptions& options, const ViewerLifecycleState& state)
{
  (void)options;
  return state.frameOutputEnabled && !state.frameOutputDirectory.empty();
}

std::string makeFrameOutputPath(
    const RunOptions& options,
    const ViewerLifecycleState& state,
    int frameNumber)
{
  return makeFrameOutputPath(
      activeFrameOutputDirectory(options, state), frameNumber);
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

void requestExit(ViewerLifecycleState& state)
{
  state.exitRequested = true;
}

void requestSceneSwitch(ViewerLifecycleState& state, std::string sceneId)
{
  state.requestedScene = std::move(sceneId);
  state.sceneSwitchRequested = true;
  state.sceneActivationPendingScene = state.requestedScene;
  state.sceneActivationStatus
      = "Starting demo '" + state.requestedScene + "'...";
}

void requestSceneReplay(ViewerLifecycleState& state, std::string sceneId)
{
  requestSceneSwitch(state, std::move(sceneId));
  state.paused = false;
  state.stepOnce = false;
}

void requestDockLayoutReset(ViewerLifecycleState& state)
{
  state.dockLayoutResetRequested = true;
}

bool consumeDockLayoutResetRequest(ViewerLifecycleState& state)
{
  const bool requested = state.dockLayoutResetRequested;
  state.dockLayoutResetRequested = false;
  return requested;
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

  Eigen::Vector3d upReference = camera.up;
  if (!upReference.allFinite() || upReference.squaredNorm() < 1e-12) {
    upReference = Eigen::Vector3d::UnitZ();
  } else {
    upReference.normalize();
  }

  basis.right = basis.forward.cross(upReference);
  if (basis.right.squaredNorm() < 1e-12) {
    const Eigen::Vector3d fallbackUp
        = std::abs(basis.forward.dot(Eigen::Vector3d::UnitZ())) < 0.95
              ? Eigen::Vector3d::UnitZ()
              : Eigen::Vector3d::UnitY();
    basis.right = basis.forward.cross(fallbackUp);
  }
  if (basis.right.squaredNorm() < 1e-12) {
    basis.right = Eigen::Vector3d::UnitY();
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

  if (update.zoom) {
    camera.distance *= std::exp(update.deltaY * update.zoomScale);
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
  if (input.locked) {
    controller.scrollDelta = 0.0;
    resetOrbitCameraTracking(controller);
    return;
  }

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
  update.zoom = hasCursor && input.zoom;
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

//==============================================================================
Eigen::Vector3d projectToPixels(
    const OrbitCamera& camera,
    int width,
    int height,
    const Eigen::Vector3d& point,
    const ProjectionOptions& options)
{
  const int safeWidth = std::max(1, width);
  const int safeHeight = std::max(1, height);
  const PerspectiveProjection projection
      = makePerspectiveProjection(camera, safeWidth, safeHeight, options);
  const OrbitCameraBasis basis = makeOrbitCameraBasis(camera);

  const Eigen::Vector3d offset = point - basis.eye;
  const double depth = offset.dot(basis.forward);
  const double safeDepth = std::abs(depth) < 1e-12 ? 1e-12 : depth;
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  const double focal
      = 1.0 / std::tan(projection.verticalFovDegrees * kDegreesToRadians * 0.5);
  const double aspect = std::max(projection.aspectRatio, 1e-6);
  const double ndcX = offset.dot(basis.right) / safeDepth * (focal / aspect);
  const double ndcY = offset.dot(basis.up) / safeDepth * focal;
  return {
      (ndcX * 0.5 + 0.5) * safeWidth,
      (1.0 - (ndcY * 0.5 + 0.5)) * safeHeight,
      depth};
}

namespace {

constexpr double kPi = 3.14159265358979323846;

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

// Conservative local half-extent for a descriptor's geometry, used to build a
// per-descriptor axis-aligned bounding box before transforming it to world
// space. Meshes/point clouds expose explicit local bounds; primitive shapes are
// sized from their descriptor fields.
Eigen::Vector3d localHalfExtent(const GeometryDescriptor& geometry)
{
  if (geometry.hasLocalBounds) {
    return 0.5 * (geometry.localBoundsMax - geometry.localBoundsMin).cwiseAbs();
  }

  switch (geometry.kind) {
    case ShapeKind::Box:
    case ShapeKind::Ellipsoid:
      return 0.5 * geometry.size.cwiseAbs();
    case ShapeKind::Sphere: {
      const double r
          = geometry.radius > 0.0 ? geometry.radius : 0.5 * geometry.size.x();
      return Eigen::Vector3d::Constant(std::abs(r));
    }
    case ShapeKind::Cylinder:
    case ShapeKind::Cone: {
      const double r = std::abs(geometry.radius);
      return Eigen::Vector3d(r, r, 0.5 * std::abs(geometry.height));
    }
    case ShapeKind::Capsule: {
      const double r = std::abs(geometry.radius);
      return Eigen::Vector3d(r, r, 0.5 * std::abs(geometry.height) + r);
    }
    default:
      // Fall back to the primitive size (may be zero for descriptor-only
      // geometry); such descriptors contribute only their world position.
      return 0.5 * geometry.size.cwiseAbs();
  }
}

Eigen::Vector3d localBoundsCenter(const GeometryDescriptor& geometry)
{
  if (geometry.hasLocalBounds) {
    return 0.5 * (geometry.localBoundsMax + geometry.localBoundsMin);
  }
  return Eigen::Vector3d::Zero();
}

} // namespace

bool orbitCameraViewPreset(
    std::string_view name, double& azimuthDegrees, double& elevationDegrees)
{
  // House presets resolved in the WP-ASV.4 design note. Azimuth maps to yaw and
  // elevation to pitch (see cameraEye), so these are plain spherical angles.
  if (name == "three-quarter") {
    azimuthDegrees = -45.0;
    elevationDegrees = 25.0;
    return true;
  }
  if (name == "front") {
    azimuthDegrees = -90.0;
    elevationDegrees = 0.0;
    return true;
  }
  if (name == "side") {
    azimuthDegrees = 0.0;
    elevationDegrees = 0.0;
    return true;
  }
  if (name == "top") {
    // Just under 90 degrees to avoid the forward-parallel-up degeneracy that
    // makeOrbitCameraBasis would otherwise have to patch.
    azimuthDegrees = -90.0;
    elevationDegrees = 89.0;
    return true;
  }
  return false;
}

OrbitCamera applyOrbitCameraView(
    OrbitCamera base, const OrbitCameraViewOptions& view)
{
  if (view.preset.has_value()) {
    double azimuthDegrees = 0.0;
    double elevationDegrees = 0.0;
    if (orbitCameraViewPreset(*view.preset, azimuthDegrees, elevationDegrees)) {
      base.yaw = degreesToRadians(azimuthDegrees);
      base.pitch = degreesToRadians(elevationDegrees);
    }
  }
  if (view.azimuthDegrees.has_value()) {
    base.yaw = degreesToRadians(*view.azimuthDegrees);
  }
  if (view.elevationDegrees.has_value()) {
    base.pitch = degreesToRadians(*view.elevationDegrees);
  }
  if (view.distance.has_value()) {
    base.distance = *view.distance;
  }
  if (view.target.has_value()) {
    base.target = *view.target;
  }
  return base;
}

BoundingSphere sceneBoundingSphere(
    const std::vector<RenderableDescriptor>& descriptors)
{
  Eigen::Vector3d minCorner
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maxCorner
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  bool sawAny = false;

  for (const RenderableDescriptor& descriptor : descriptors) {
    if (!descriptor.material.visible) {
      continue;
    }
    const Eigen::Vector3d halfExtent = localHalfExtent(descriptor.geometry);
    const Eigen::Vector3d localCenter = localBoundsCenter(descriptor.geometry);
    // Transform the 8 local AABB corners to world space and expand the union.
    for (int sx = -1; sx <= 1; sx += 2) {
      for (int sy = -1; sy <= 1; sy += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
          const Eigen::Vector3d localCorner = localCenter
                                              + Eigen::Vector3d(
                                                  sx * halfExtent.x(),
                                                  sy * halfExtent.y(),
                                                  sz * halfExtent.z());
          const Eigen::Vector3d worldCorner
              = descriptor.worldTransform * localCorner;
          if (!worldCorner.allFinite()) {
            continue;
          }
          minCorner = minCorner.cwiseMin(worldCorner);
          maxCorner = maxCorner.cwiseMax(worldCorner);
          sawAny = true;
        }
      }
    }
  }

  BoundingSphere sphere;
  if (!sawAny || !minCorner.allFinite() || !maxCorner.allFinite()) {
    sphere.center = Eigen::Vector3d::Zero();
    sphere.radius = 1.0;
    return sphere;
  }
  sphere.center = 0.5 * (minCorner + maxCorner);
  sphere.radius = 0.5 * (maxCorner - minCorner).norm();
  if (!(sphere.radius > 0.0) || !std::isfinite(sphere.radius)) {
    sphere.radius = 1.0;
  }
  return sphere;
}

OrbitCamera fitOrbitCamera(
    const BoundingSphere& sphere,
    double verticalFovDegrees,
    double azimuthDegrees,
    double elevationDegrees)
{
  OrbitCamera camera;
  camera.target = sphere.center;
  camera.up = Eigen::Vector3d::UnitZ();
  camera.yaw = degreesToRadians(azimuthDegrees);
  camera.pitch = degreesToRadians(elevationDegrees);

  const double radius = std::isfinite(sphere.radius) && sphere.radius > 0.0
                            ? sphere.radius
                            : 1.0;
  const double fovRadians = degreesToRadians(
      std::isfinite(verticalFovDegrees) && verticalFovDegrees > 0.0
              && verticalFovDegrees < 180.0
          ? verticalFovDegrees
          : ProjectionOptions{}.verticalFovDegrees);
  const double sinHalf = std::sin(0.5 * fovRadians);
  camera.distance = sinHalf > 1e-6 ? radius / sinHalf : radius;
  return camera;
}

} // namespace dart::gui
