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

#include <dart/gui/Export.hpp>
#include <dart/gui/detail/renderable.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <cstdint>

namespace dart::gui {

struct PickRay
{
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
};

enum class RenderOutputMode
{
  Color,
  Depth,
};

struct RenderSettings
{
  bool shadowsEnabled = true;
  RenderOutputMode outputMode = RenderOutputMode::Color;
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
  bool zoom = false;
  double orbitScale = 0.006;
  double panScale = 0.0015;
  double zoomScale = 0.006;
  double scrollScale = 0.12;
  double minDistance = 0.35;
  double maxDistance = 80.0;
  double minPitch = -1.45;
  double maxPitch = 1.45;
};

enum class OrbitCameraMouseMode
{
  Orbit,
  Pan,
  Zoom,
};

struct OrbitCameraControlOptions
{
  OrbitCameraMouseMode mouseMode = OrbitCameraMouseMode::Orbit;
  bool locked = false;
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
  bool locked = false;
  bool orbit = false;
  bool pan = false;
  bool zoom = false;
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
  double nearScale = 0.001;
  double minNearPlane = 0.0005;
  double maxNearPlane = 0.010;
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

/// A world-space bounding sphere over a set of renderables. Produced by
/// `sceneBoundingSphere` and consumed by the `fitOrbitCamera` helpers to
/// auto-frame a scene. `hasBounds` distinguishes a sphere computed from at
/// least one visible, bounded renderable from the finite fallback returned for
/// an empty or unbounded scene. Value type only so it can cross the public GUI
/// boundary.
struct BoundingSphere
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
  bool hasBounds = false;
};

/// Per-view camera overrides resolved onto a base `OrbitCamera`
/// (modify-not-replace). `preset` (if set) supplies azimuth/elevation from a
/// named canonical view; the explicit fields then override whichever of
/// azimuth/elevation/distance/target the caller provided. Angles are degrees
/// and map directly onto `OrbitCamera` yaw/pitch (see `cameraEye`): azimuth 0
/// places the eye on the +X side of the target and positive azimuth rotates the
/// eye toward +Y; elevation is the angle above the XY plane.
struct OrbitCameraViewOptions
{
  std::optional<std::string> preset;
  std::optional<double> azimuthDegrees;
  std::optional<double> elevationDegrees;
  std::optional<double> distance;
  std::optional<Eigen::Vector3d> target;
};

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

/// Projects a world-space point to pixel coordinates for an orbit camera.
/// Returns (x_pixels, y_pixels, view_depth) with the pixel origin at the
/// top-left corner; points behind the camera get a negative depth. This is
/// the shared projection primitive for debug-label compositing and
/// view-adequacy assessment, matching the offscreen render's framing.
DART_GUI_API Eigen::Vector3d projectToPixels(
    const OrbitCamera& camera,
    int width,
    int height,
    const Eigen::Vector3d& point,
    const ProjectionOptions& options = {});

/// Looks up a canonical camera view preset by name. On success writes the
/// preset azimuth/elevation (degrees) and returns true. Recognized names:
/// "three-quarter" (-45, 25), "front" (-90, 0), "side" (0, 0), "top" (-90, 89).
DART_GUI_API bool orbitCameraViewPreset(
    std::string_view name, double& azimuthDegrees, double& elevationDegrees);

/// Applies per-view overrides onto `base` (modify-not-replace): if a preset is
/// named it seeds azimuth/elevation, then any explicit azimuth/elevation map
/// onto yaw/pitch and any explicit distance/target replace those fields.
/// Distance auto-fit is intentionally not applied here (it needs scene bounds);
/// callers combine this with `sceneBoundingSphere`/`fitOrbitCamera` when
/// framing is requested. An unrecognized preset name is ignored.
DART_GUI_API OrbitCamera
applyOrbitCameraView(OrbitCamera base, const OrbitCameraViewOptions& view);

/// Computes a world-space bounding sphere over the union of the per-descriptor
/// axis-aligned bounding boxes. Invisible descriptors and descriptors without
/// usable finite bounds are ignored. Returns a unit-radius sphere at the origin
/// with `hasBounds == false` when no descriptor contributes bounds, so native
/// downstream framing stays finite while callers can fail closed.
DART_GUI_API BoundingSphere
sceneBoundingSphere(const std::vector<RenderableDescriptor>& descriptors);

/// Builds an `OrbitCamera` that frames `sphere` from the given azimuth and
/// elevation (degrees). The look-at target is the sphere center and the
/// distance is `radius / sin(verticalFov / 2)`, so the sphere fits the vertical
/// field of view.
///
DART_GUI_API OrbitCamera fitOrbitCamera(
    const BoundingSphere& sphere,
    double verticalFovDegrees,
    double azimuthDegrees,
    double elevationDegrees);

/// Builds an `OrbitCamera` that frames `sphere` inside a viewport. The limiting
/// field of view is derived from `width`/`height`, so portrait captures do not
/// crop a sphere that would fit the vertical field of view alone. `margin`
/// scales the fitted distance and defaults to no extra padding. Invalid
/// dimensions, field of view, and margin values receive finite safe defaults.
DART_GUI_API OrbitCamera fitOrbitCameraToViewport(
    const BoundingSphere& sphere,
    int width,
    int height,
    double verticalFovDegrees,
    double azimuthDegrees,
    double elevationDegrees,
    double margin = 1.0);

} // namespace dart::gui
