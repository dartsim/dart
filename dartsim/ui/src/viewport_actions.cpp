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

#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/object_manager.hpp>
#include <dartsim_ui/outliner_actions.hpp>
#include <dartsim_ui/viewport_actions.hpp>

#include <algorithm>
#include <limits>
#include <utility>

#include <cmath>

namespace dartsim::ui {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kMinCameraDistance = 0.8;
constexpr double kMaxCameraDistance = 80.0;
constexpr double kDefaultSceneRadius = 1.0;

bool isTransformable(ObjectType type)
{
  return type == ObjectType::RigidBody || type == ObjectType::FreeFrame
         || type == ObjectType::FixedFrame;
}

bool layerEnabled(
    const ViewportLayerFilterState& filters, ViewportLayerKind kind)
{
  switch (kind) {
    case ViewportLayerKind::RigidBodies:
      return filters.rigidBodies;
    case ViewportLayerKind::Links:
      return filters.links;
    case ViewportLayerKind::Frames:
      return filters.frames;
  }
  return true;
}

void setLayerEnabled(
    ViewportLayerFilterState& filters, ViewportLayerKind kind, bool visible)
{
  switch (kind) {
    case ViewportLayerKind::RigidBodies:
      filters.rigidBodies = visible;
      break;
    case ViewportLayerKind::Links:
      filters.links = visible;
      break;
    case ViewportLayerKind::Frames:
      filters.frames = visible;
      break;
  }
}

std::string layerLabel(ViewportLayerKind kind)
{
  switch (kind) {
    case ViewportLayerKind::RigidBodies:
      return "Rigid Bodies";
    case ViewportLayerKind::Links:
      return "Links";
    case ViewportLayerKind::Frames:
      return "Frames";
  }
  return "Unknown";
}

bool objectLayerEnabled(
    ObjectType type, const ViewportLayerFilterState& filters)
{
  switch (type) {
    case ObjectType::RigidBody:
      return filters.rigidBodies;
    case ObjectType::Link:
      return filters.links;
    case ObjectType::FreeFrame:
    case ObjectType::FixedFrame:
      return filters.frames;
    case ObjectType::MultiBody:
    case ObjectType::Joint:
      return false;
  }
  return false;
}

bool renderItemPassesLayer(
    const SimEngine& engine,
    const ViewportLayerFilterState& filters,
    const RenderItem& item)
{
  const SceneObject* object = engine.objects().model().find(item.id);
  return object != nullptr && objectLayerEnabled(object->type, filters);
}

const SceneObject* selectedTransformableObject(
    const SimEngine& engine, const ViewportLayerFilterState& filters)
{
  const ObjectId selected = engine.selection().primary();
  const SceneObject* object = engine.objects().model().find(selected);
  if (object == nullptr || !isTransformable(object->type)
      || !isViewportObjectVisible(engine, selected, filters)) {
    return nullptr;
  }
  return object;
}

bool isFinitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

struct ViewportBounds
{
  Eigen::Vector3d minimum
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  bool empty = true;
};

bool includePoint(ViewportBounds& bounds, const Eigen::Vector3d& point)
{
  if (!point.allFinite()) {
    return false;
  }

  bounds.minimum = bounds.empty ? point : bounds.minimum.cwiseMin(point);
  bounds.maximum = bounds.empty ? point : bounds.maximum.cwiseMax(point);
  bounds.empty = false;
  return true;
}

Eigen::Vector3d sanitizedDimensions(const Eigen::Vector3d& dimensions)
{
  if (!dimensions.allFinite()) {
    return Eigen::Vector3d::Ones();
  }
  return dimensions.cwiseAbs().cwiseMax(Eigen::Vector3d::Constant(0.01));
}

Eigen::Vector3d localHalfExtent(const RenderItem& item)
{
  const Eigen::Vector3d dimensions = sanitizedDimensions(item.dimensions);
  switch (item.shape) {
    case ShapeType::Box:
      return dimensions * 0.5;
    case ShapeType::Sphere:
      return Eigen::Vector3d::Constant(dimensions.x());
    case ShapeType::Cylinder:
      return Eigen::Vector3d(
          dimensions.x(), dimensions.x(), dimensions.y() * 0.5);
    case ShapeType::Capsule:
      return Eigen::Vector3d(
          dimensions.x(),
          dimensions.x(),
          dimensions.y() * 0.5 + dimensions.x());
    case ShapeType::Plane:
      return Eigen::Vector3d(3.0, 3.0, 0.01);
  }

  return Eigen::Vector3d::Constant(0.5);
}

void includeRenderItemBounds(ViewportBounds& bounds, const RenderItem& item)
{
  if (!item.worldTransform.matrix().allFinite()) {
    return;
  }

  const Eigen::Vector3d half = localHalfExtent(item);
  for (int sx : {-1, 1}) {
    for (int sy : {-1, 1}) {
      for (int sz : {-1, 1}) {
        includePoint(
            bounds,
            item.worldTransform
                * Eigen::Vector3d(
                    static_cast<double>(sx) * half.x(),
                    static_cast<double>(sy) * half.y(),
                    static_cast<double>(sz) * half.z()));
      }
    }
  }
}

ViewportBounds visibleSceneBounds(
    const SimEngine& engine, const ViewportLayerFilterState& filters)
{
  ViewportBounds bounds;
  for (const RenderItem& item : filteredViewportRenderItems(engine, filters)) {
    includeRenderItemBounds(bounds, item);
  }
  return bounds;
}

ViewportBounds selectedRenderableBounds(
    const SimEngine& engine, const ViewportLayerFilterState& filters)
{
  ViewportBounds bounds;
  const SelectionState selection = engine.selection().state();
  if (selection.ids.empty()) {
    return bounds;
  }

  for (const ObjectId selected : selection.ids) {
    for (const RenderItem& item :
         filteredViewportRenderItems(engine, filters)) {
      if (item.id == selected) {
        includeRenderItemBounds(bounds, item);
        break;
      }
    }
  }

  return bounds;
}

bool isEffectivelyVisible(const SimEngine& engine, ObjectId id)
{
  const SceneModel& model = engine.objects().model();
  ObjectId current = id;
  while (current != kNoObject) {
    const SceneObject* object = model.find(current);
    if (object == nullptr || !object->visible) {
      return false;
    }
    current = object->parent;
  }
  return true;
}

ViewportBounds selectedFocusBounds(
    const SimEngine& engine, const ViewportLayerFilterState& filters)
{
  ViewportBounds bounds = selectedRenderableBounds(engine, filters);
  for (const ObjectId selected : engine.selection().state().ids) {
    const SceneObject* object = engine.objects().model().find(selected);
    if (object == nullptr
        || (object->type != ObjectType::FreeFrame
            && object->type != ObjectType::FixedFrame)
        || !isViewportObjectVisible(engine, selected, filters)) {
      continue;
    }

    const std::optional<Eigen::Isometry3d> transform
        = engine.objects().worldTransformOf(selected);
    if (transform.has_value()) {
      includePoint(bounds, transform->translation());
    }
  }
  return bounds;
}

Eigen::Vector3d boundsCenter(const ViewportBounds& bounds)
{
  if (bounds.empty) {
    return Eigen::Vector3d::Zero();
  }
  return (bounds.minimum + bounds.maximum) * 0.5;
}

double boundsRadius(const ViewportBounds& bounds)
{
  if (bounds.empty) {
    return kDefaultSceneRadius;
  }
  const double radius = (bounds.maximum - bounds.minimum).norm() * 0.5;
  return isFinitePositive(radius) ? radius : kDefaultSceneRadius;
}

double cameraDistanceForRadius(double radius)
{
  const double safeRadius
      = isFinitePositive(radius) ? radius : kDefaultSceneRadius;
  return std::clamp(
      safeRadius * 2.8 + 0.35, kMinCameraDistance, kMaxCameraDistance);
}

dart::gui::OrbitCamera sanitizedCurrentCamera(
    const dart::gui::OrbitCamera& currentCamera)
{
  dart::gui::OrbitCamera camera = currentCamera;
  if (!camera.target.allFinite()) {
    camera.target = Eigen::Vector3d::Zero();
  }
  if (!camera.up.allFinite() || camera.up.squaredNorm() < 1e-12) {
    camera.up = Eigen::Vector3d::UnitZ();
  } else {
    camera.up.normalize();
  }
  if (!std::isfinite(camera.yaw)) {
    camera.yaw = -0.78;
  }
  if (!std::isfinite(camera.pitch)) {
    camera.pitch = 0.42;
  }
  camera.pitch = std::clamp(camera.pitch, -1.45, 1.45);
  if (!isFinitePositive(camera.distance)) {
    camera.distance = 6.1;
  }
  camera.distance
      = std::clamp(camera.distance, kMinCameraDistance, kMaxCameraDistance);
  return camera;
}

dart::gui::OrbitCamera cameraForView(
    const dart::gui::OrbitCamera& currentCamera,
    double yaw,
    double pitch,
    double distance)
{
  dart::gui::OrbitCamera camera = sanitizedCurrentCamera(currentCamera);
  camera.up = Eigen::Vector3d::UnitZ();
  camera.yaw = yaw;
  camera.pitch = std::clamp(pitch, -1.45, 1.45);
  camera.distance = std::clamp(
      isFinitePositive(distance) ? distance : camera.distance,
      kMinCameraDistance,
      kMaxCameraDistance);
  return camera;
}

dart::gui::OrbitCamera applyPreset(
    const dart::gui::OrbitCamera& currentCamera, ViewportCameraActionKind kind)
{
  switch (kind) {
    case ViewportCameraActionKind::Front:
      return cameraForView(
          currentCamera, -kPi * 0.5, 0.0, currentCamera.distance);
    case ViewportCameraActionKind::Back:
      return cameraForView(
          currentCamera, kPi * 0.5, 0.0, currentCamera.distance);
    case ViewportCameraActionKind::Left:
      return cameraForView(currentCamera, kPi, 0.0, currentCamera.distance);
    case ViewportCameraActionKind::Right:
      return cameraForView(currentCamera, 0.0, 0.0, currentCamera.distance);
    case ViewportCameraActionKind::Top:
      return cameraForView(
          currentCamera, -kPi * 0.5, 1.45, currentCamera.distance);
    case ViewportCameraActionKind::Bottom:
      return cameraForView(
          currentCamera, -kPi * 0.5, -1.45, currentCamera.distance);
    case ViewportCameraActionKind::Perspective:
      return cameraForView(currentCamera, -0.78, 0.42, currentCamera.distance);
    case ViewportCameraActionKind::FitScene:
    case ViewportCameraActionKind::FocusSelection:
      break;
  }

  return sanitizedCurrentCamera(currentCamera);
}

ViewportCameraAction makeCameraAction(
    ViewportCameraActionKind kind,
    std::string label,
    bool enabled = true,
    std::string disabledReason = {})
{
  ViewportCameraAction action;
  action.kind = kind;
  action.label = std::move(label);
  action.enabled = enabled;
  action.disabledReason = std::move(disabledReason);
  return action;
}

ViewportCameraControlAction makeCameraControlAction(
    ViewportCameraControlActionKind kind,
    std::string label,
    bool checked,
    bool enabled = true,
    std::string disabledReason = {})
{
  ViewportCameraControlAction action;
  action.kind = kind;
  action.label = std::move(label);
  action.checked = checked;
  action.enabled = enabled;
  action.disabledReason = std::move(disabledReason);
  return action;
}

std::string cameraMouseModeLabel(dart::gui::OrbitCameraMouseMode mode)
{
  switch (mode) {
    case dart::gui::OrbitCameraMouseMode::Orbit:
      return "Orbit Camera Mode";
    case dart::gui::OrbitCameraMouseMode::Pan:
      return "Pan Camera Mode";
    case dart::gui::OrbitCameraMouseMode::Zoom:
      return "Zoom Camera Mode";
  }
  return "Camera Mode";
}

} // namespace

Eigen::Vector3d viewportMoveDelta(const ViewportMoveInput& input)
{
  if (!isFinitePositive(input.step)
      || !isFinitePositive(input.fastMultiplier)) {
    return Eigen::Vector3d::Zero();
  }

  const double scale = input.step * (input.fast ? input.fastMultiplier : 1.0);
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  delta.x() += input.right ? scale : 0.0;
  delta.x() -= input.left ? scale : 0.0;
  delta.y() += input.forward ? scale : 0.0;
  delta.y() -= input.backward ? scale : 0.0;
  delta.z() += input.up ? scale : 0.0;
  delta.z() -= input.down ? scale : 0.0;
  return delta;
}

bool moveSelectedFromViewport(SimEngine& engine, const ViewportMoveInput& input)
{
  return moveSelectedFromViewport(engine, ViewportLayerFilterState{}, input);
}

bool moveSelectedFromViewport(
    SimEngine& engine,
    const ViewportLayerFilterState& filters,
    const ViewportMoveInput& input)
{
  if (selectedTransformableObject(engine, filters) == nullptr) {
    return false;
  }
  return moveSelectedBy(engine, viewportMoveDelta(input));
}

ViewportTransformGizmo makeViewportTransformGizmo()
{
  ViewportTransformGizmo state;
  state.target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "dartsim_selection_transform");
  state.gizmo.target = state.target;
  state.gizmo.label = "Selection Transform";
  state.gizmo.flags = dart::gui::GizmoFlags::All;
  state.gizmo.size = 0.22;
  return state;
}

bool syncViewportTransformGizmo(
    ViewportTransformGizmo& state, const SimEngine& engine)
{
  return syncViewportTransformGizmo(state, engine, ViewportLayerFilterState{});
}

bool syncViewportTransformGizmo(
    ViewportTransformGizmo& state,
    const SimEngine& engine,
    const ViewportLayerFilterState& filters)
{
  if (state.target == nullptr) {
    state = makeViewportTransformGizmo();
  }

  const SceneObject* object = selectedTransformableObject(engine, filters);
  if (object == nullptr) {
    state.object = kNoObject;
    return false;
  }

  state.object = object->id;
  state.target->setTransform(object->transform, dart::dynamics::Frame::World());
  state.gizmo.label = object->name + " Transform";
  return true;
}

bool applyViewportTransformGizmo(
    SimEngine& engine,
    ViewportTransformGizmo& state,
    const Eigen::Isometry3d& transform)
{
  return applyViewportTransformGizmo(
      engine, state, ViewportLayerFilterState{}, transform);
}

bool applyViewportTransformGizmo(
    SimEngine& engine,
    ViewportTransformGizmo& state,
    const ViewportLayerFilterState& filters,
    const Eigen::Isometry3d& transform)
{
  if (!engine.canEditScene() || !transform.matrix().allFinite()
      || state.object == kNoObject
      || engine.selection().primary() != state.object) {
    return false;
  }

  const SceneObject* object = selectedTransformableObject(engine, filters);
  if (object == nullptr || object->id != state.object) {
    return false;
  }

  if (object->transform.matrix().isApprox(transform.matrix())) {
    return false;
  }

  engine.execute(commands::setTransform(state.object, transform));
  return syncViewportTransformGizmo(state, engine, filters);
}

std::vector<RenderItem> filteredViewportRenderItems(
    const SimEngine& engine, const ViewportLayerFilterState& filters)
{
  std::vector<RenderItem> items;
  for (const RenderItem& item : engine.renderItems()) {
    if (renderItemPassesLayer(engine, filters, item)) {
      items.push_back(item);
    }
  }
  return items;
}

bool isViewportObjectVisible(
    const SimEngine& engine,
    ObjectId id,
    const ViewportLayerFilterState& filters)
{
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr || !objectLayerEnabled(object->type, filters)
      || !isEffectivelyVisible(engine, id)) {
    return false;
  }

  if (object->type == ObjectType::RigidBody
      || object->type == ObjectType::Link) {
    const std::vector<RenderItem> items
        = filteredViewportRenderItems(engine, filters);
    return std::find_if(
               items.begin(),
               items.end(),
               [id](const RenderItem& item) { return item.id == id; })
           != items.end();
  }

  return object->type == ObjectType::FreeFrame
         || object->type == ObjectType::FixedFrame;
}

bool selectViewportRenderable(
    SimEngine& engine,
    const ViewportLayerFilterState& filters,
    dart::gui::RenderableId renderableId)
{
  if (renderableId == 0) {
    return engine.select(kNoObject);
  }

  const ObjectId id = objectIdForRenderable(renderableId);
  return isViewportObjectVisible(engine, id, filters)
         && selectViewportRenderable(engine, renderableId);
}

dart::gui::RenderableId selectedViewportRenderable(
    const SimEngine& engine, const ViewportLayerFilterState& filters)
{
  const ObjectId selected = engine.selection().primary();
  if (!isViewportObjectVisible(engine, selected, filters)) {
    return 0;
  }

  const SceneObject* object = engine.objects().model().find(selected);
  if (object == nullptr
      || (object->type != ObjectType::RigidBody
          && object->type != ObjectType::Link)) {
    return 0;
  }
  return renderableIdForObject(selected);
}

std::string selectedViewportLabel(
    const SimEngine& engine, const ViewportLayerFilterState& filters)
{
  const ObjectId selected = engine.selection().primary();
  if (!isViewportObjectVisible(engine, selected, filters)) {
    return "none";
  }

  const SceneObject* object = engine.objects().model().find(selected);
  if (object == nullptr) {
    return "none";
  }

  return object->name + " [" + objectTypeLabel(object->type) + "]";
}

std::vector<ViewportLayerFilterAction> buildViewportLayerFilterActions(
    const ViewportLayerFilterState& filters)
{
  return {
      {ViewportLayerKind::RigidBodies,
       "Show Rigid Bodies",
       layerEnabled(filters, ViewportLayerKind::RigidBodies)},
      {ViewportLayerKind::Links,
       "Show Links",
       layerEnabled(filters, ViewportLayerKind::Links)},
      {ViewportLayerKind::Frames,
       "Show Frames",
       layerEnabled(filters, ViewportLayerKind::Frames)},
  };
}

ViewportLayerFilterActionResult setViewportLayerVisible(
    ViewportLayerFilterState& filters, ViewportLayerKind kind, bool visible)
{
  setLayerEnabled(filters, kind, visible);
  std::string message = layerLabel(kind);
  message += visible ? " shown" : " hidden";
  return {true, std::move(message)};
}

std::vector<ViewportCameraAction> buildViewportCameraActions(
    const SimEngine& engine)
{
  return buildViewportCameraActions(engine, ViewportLayerFilterState{});
}

std::vector<ViewportCameraAction> buildViewportCameraActions(
    const SimEngine& engine, const ViewportLayerFilterState& filters)
{
  const bool hasVisibleScene
      = !filteredViewportRenderItems(engine, filters).empty();
  const bool hasVisibleSelection = !selectedFocusBounds(engine, filters).empty;

  return {
      makeCameraAction(
          ViewportCameraActionKind::FitScene,
          "Fit Scene",
          hasVisibleScene,
          "No visible objects"),
      makeCameraAction(
          ViewportCameraActionKind::FocusSelection,
          "Focus Selection",
          hasVisibleSelection,
          "No visible selected object"),
      makeCameraAction(
          ViewportCameraActionKind::Perspective, "Perspective View"),
      makeCameraAction(ViewportCameraActionKind::Front, "Front View"),
      makeCameraAction(ViewportCameraActionKind::Back, "Back View"),
      makeCameraAction(ViewportCameraActionKind::Left, "Left View"),
      makeCameraAction(ViewportCameraActionKind::Right, "Right View"),
      makeCameraAction(ViewportCameraActionKind::Top, "Top View"),
      makeCameraAction(ViewportCameraActionKind::Bottom, "Bottom View"),
  };
}

ViewportCameraActionResult applyViewportCameraAction(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    ViewportCameraActionKind kind)
{
  return applyViewportCameraAction(
      engine, currentCamera, kind, ViewportLayerFilterState{});
}

ViewportCameraActionResult applyViewportCameraAction(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    ViewportCameraActionKind kind,
    const ViewportLayerFilterState& filters)
{
  dart::gui::OrbitCamera camera = sanitizedCurrentCamera(currentCamera);
  switch (kind) {
    case ViewportCameraActionKind::FitScene: {
      const ViewportBounds bounds = visibleSceneBounds(engine, filters);
      if (bounds.empty) {
        return {false, "No visible objects", std::nullopt};
      }
      camera.target = boundsCenter(bounds);
      camera.distance = cameraDistanceForRadius(boundsRadius(bounds));
      return {true, "Fit scene", camera};
    }
    case ViewportCameraActionKind::FocusSelection: {
      const ViewportBounds bounds = selectedFocusBounds(engine, filters);
      if (bounds.empty) {
        return {false, "No visible selected object", std::nullopt};
      }
      camera.target = boundsCenter(bounds);
      camera.distance = cameraDistanceForRadius(boundsRadius(bounds));
      return {true, "Focused selection", camera};
    }
    case ViewportCameraActionKind::Perspective:
      return {true, "Perspective view", applyPreset(camera, kind)};
    case ViewportCameraActionKind::Front:
      return {true, "Front view", applyPreset(camera, kind)};
    case ViewportCameraActionKind::Back:
      return {true, "Back view", applyPreset(camera, kind)};
    case ViewportCameraActionKind::Left:
      return {true, "Left view", applyPreset(camera, kind)};
    case ViewportCameraActionKind::Right:
      return {true, "Right view", applyPreset(camera, kind)};
    case ViewportCameraActionKind::Top:
      return {true, "Top view", applyPreset(camera, kind)};
    case ViewportCameraActionKind::Bottom:
      return {true, "Bottom view", applyPreset(camera, kind)};
  }

  return {false, "Unknown camera action", std::nullopt};
}

std::vector<ViewportCameraControlAction> buildViewportCameraControlActions(
    const SimEngine& engine,
    const ViewportLayerFilterState& filters,
    const ViewportCameraControlState& controls)
{
  const bool hasVisibleSelection = !selectedFocusBounds(engine, filters).empty;
  return {
      makeCameraControlAction(
          ViewportCameraControlActionKind::OrbitMode,
          "Orbit Camera Mode",
          controls.mouseMode == dart::gui::OrbitCameraMouseMode::Orbit),
      makeCameraControlAction(
          ViewportCameraControlActionKind::PanMode,
          "Pan Camera Mode",
          controls.mouseMode == dart::gui::OrbitCameraMouseMode::Pan),
      makeCameraControlAction(
          ViewportCameraControlActionKind::ZoomMode,
          "Zoom Camera Mode",
          controls.mouseMode == dart::gui::OrbitCameraMouseMode::Zoom),
      makeCameraControlAction(
          ViewportCameraControlActionKind::ToggleTrackSelection,
          "Track Selection",
          controls.trackSelection,
          controls.trackSelection || hasVisibleSelection,
          "No visible selected object"),
  };
}

ViewportCameraControlActionResult applyViewportCameraControlAction(
    const SimEngine& engine,
    const ViewportLayerFilterState& filters,
    ViewportCameraControlState& controls,
    ViewportCameraControlActionKind kind)
{
  switch (kind) {
    case ViewportCameraControlActionKind::OrbitMode:
      controls.mouseMode = dart::gui::OrbitCameraMouseMode::Orbit;
      return {true, cameraMouseModeLabel(controls.mouseMode)};
    case ViewportCameraControlActionKind::PanMode:
      controls.mouseMode = dart::gui::OrbitCameraMouseMode::Pan;
      return {true, cameraMouseModeLabel(controls.mouseMode)};
    case ViewportCameraControlActionKind::ZoomMode:
      controls.mouseMode = dart::gui::OrbitCameraMouseMode::Zoom;
      return {true, cameraMouseModeLabel(controls.mouseMode)};
    case ViewportCameraControlActionKind::ToggleTrackSelection:
      if (controls.trackSelection) {
        controls.trackSelection = false;
        return {true, "Selection tracking off"};
      }
      if (selectedFocusBounds(engine, filters).empty) {
        return {false, "No visible selected object"};
      }
      controls.trackSelection = true;
      return {true, "Selection tracking on"};
  }

  return {false, "Unknown camera control"};
}

dart::gui::OrbitCameraControlOptions viewportCameraControlOptions(
    const ViewportCameraControlState& controls)
{
  dart::gui::OrbitCameraControlOptions options;
  options.mouseMode = controls.mouseMode;
  return options;
}

ViewportCameraActionResult trackedSelectionCamera(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    const ViewportLayerFilterState& filters,
    const ViewportCameraControlState& controls)
{
  if (!controls.trackSelection) {
    return {false, "Selection tracking off", std::nullopt};
  }

  const ViewportBounds bounds = selectedFocusBounds(engine, filters);
  if (bounds.empty) {
    return {false, "No visible selected object", std::nullopt};
  }

  dart::gui::OrbitCamera camera = sanitizedCurrentCamera(currentCamera);
  camera.target = boundsCenter(bounds);
  return {true, "Tracking selection", camera};
}

} // namespace dartsim::ui
