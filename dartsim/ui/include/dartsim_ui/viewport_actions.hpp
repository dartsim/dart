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

#include <dart/gui/application.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/dynamics/fwd.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dartsim_engine/scene_object.hpp>
#include <dartsim_engine/sim_engine.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace dartsim::ui {

/// Keyboard-style viewport movement request in world axes.
struct ViewportMoveInput
{
  bool left = false;
  bool right = false;
  bool forward = false;
  bool backward = false;
  bool up = false;
  bool down = false;
  bool fast = false;
  double step = 0.10;
  double fastMultiplier = 4.0;
};

/// Renderer-neutral gizmo proxy for the currently selected engine object.
struct ViewportTransformGizmo
{
  dart::dynamics::SimpleFramePtr target;
  dart::gui::Gizmo gizmo;
  ObjectId object = kNoObject;
};

/// Viewport-only object layers. These filters do not change scene visibility.
enum class ViewportLayerKind
{
  RigidBodies,
  Links,
  Frames,
};

/// Editor view state for viewport layer filters.
struct ViewportLayerFilterState
{
  bool rigidBodies = true;
  bool links = true;
  bool frames = true;
};

/// View-menu action state for viewport layer filters.
struct ViewportLayerFilterAction
{
  ViewportLayerKind kind = ViewportLayerKind::RigidBodies;
  std::string label;
  bool checked = true;
};

/// Result of applying a viewport layer-filter action.
struct ViewportLayerFilterActionResult
{
  bool ok = false;
  std::string message;
};

/// Camera workflow command exposed by the View menu.
enum class ViewportCameraActionKind
{
  FitScene,
  FocusSelection,
  Perspective,
  Front,
  Back,
  Left,
  Right,
  Top,
  Bottom,
};

/// View-menu action state for camera workflow controls.
struct ViewportCameraAction
{
  ViewportCameraActionKind kind = ViewportCameraActionKind::FitScene;
  std::string label;
  bool enabled = true;
  std::string disabledReason;
};

/// Result of applying a camera workflow command.
struct ViewportCameraActionResult
{
  bool ok = false;
  std::string message;
  std::optional<dart::gui::OrbitCamera> camera;
};

/// Editor-owned camera control state. These settings are view-only.
struct ViewportCameraControlState
{
  dart::gui::OrbitCameraMouseMode mouseMode
      = dart::gui::OrbitCameraMouseMode::Orbit;
  bool trackSelection = false;
  bool lockCamera = false;
};

/// View-menu camera-control command.
enum class ViewportCameraControlActionKind
{
  OrbitMode,
  PanMode,
  ZoomMode,
  ToggleTrackSelection,
  ToggleCameraLock,
};

/// View-menu action state for camera mouse modes and selection tracking.
struct ViewportCameraControlAction
{
  ViewportCameraControlActionKind kind
      = ViewportCameraControlActionKind::OrbitMode;
  std::string label;
  bool checked = false;
  bool enabled = true;
  std::string disabledReason;
};

/// Result of applying a viewport camera-control command.
struct ViewportCameraControlActionResult
{
  bool ok = false;
  std::string message;
};

/// Viewport layout mode. Quad layout is editor state, not scene data.
enum class ViewportLayoutKind
{
  Single,
  Quad,
};

/// Logical pane selected inside the viewport layout.
enum class ViewportPaneKind
{
  Perspective,
  Front,
  Right,
  Top,
};

using OptionalOrbitCamera = std::optional<dart::gui::OrbitCamera>;
using ViewportPaneCameraSlots
    = std::array<OptionalOrbitCamera, dart::gui::kMaxViewportPanes>;

/// Editor-owned layout state for the rendered viewport area.
struct ViewportLayoutState
{
  ViewportLayoutKind layout = ViewportLayoutKind::Single;
  ViewportPaneKind activePane = ViewportPaneKind::Perspective;
  ViewportPaneCameraSlots paneCameras;
};

/// View-menu layout command.
enum class ViewportLayoutActionKind
{
  SingleView,
  QuadView,
  ActivatePerspectivePane,
  ActivateFrontPane,
  ActivateRightPane,
  ActivateTopPane,
};

/// View-menu action state for viewport layout and active pane selection.
struct ViewportLayoutAction
{
  ViewportLayoutActionKind kind = ViewportLayoutActionKind::SingleView;
  std::string label;
  bool checked = false;
  bool enabled = true;
  std::string disabledReason;
};

/// Result of applying a viewport layout action.
struct ViewportLayoutActionResult
{
  bool ok = false;
  std::string message;
};

/// Resolve a movement request into a world-space delta.
[[nodiscard]] Eigen::Vector3d viewportMoveDelta(const ViewportMoveInput& input);

/// Move the primary selected object through an undoable engine command.
bool moveSelectedFromViewport(
    SimEngine& engine, const ViewportMoveInput& input);
bool moveSelectedFromViewport(
    SimEngine& engine,
    const ViewportLayerFilterState& filters,
    const ViewportMoveInput& input);

/// Create a transform gizmo proxy. The caller owns visibility and callbacks.
[[nodiscard]] ViewportTransformGizmo makeViewportTransformGizmo();

/// Synchronize the gizmo proxy to the current selected, visible, movable
/// object.
bool syncViewportTransformGizmo(
    ViewportTransformGizmo& state, const SimEngine& engine);
bool syncViewportTransformGizmo(
    ViewportTransformGizmo& state,
    const SimEngine& engine,
    const ViewportLayerFilterState& filters);

/// Commit a gizmo-authored transform back through the undoable command stack.
bool applyViewportTransformGizmo(
    SimEngine& engine,
    ViewportTransformGizmo& state,
    const Eigen::Isometry3d& transform);
bool applyViewportTransformGizmo(
    SimEngine& engine,
    ViewportTransformGizmo& state,
    const ViewportLayerFilterState& filters,
    const Eigen::Isometry3d& transform);

/// Return render items that pass the current viewport layer filters.
[[nodiscard]] std::vector<RenderItem> filteredViewportRenderItems(
    const SimEngine& engine, const ViewportLayerFilterState& filters);

/// Viewport object visibility after scene visibility and layer filters.
[[nodiscard]] bool isViewportObjectVisible(
    const SimEngine& engine,
    ObjectId id,
    const ViewportLayerFilterState& filters);

/// Select a visible, layer-enabled viewport renderable.
bool selectViewportRenderable(
    SimEngine& engine,
    const ViewportLayerFilterState& filters,
    dart::gui::RenderableId renderableId);

/// Current selected viewport renderable and label after layer filtering.
[[nodiscard]] dart::gui::RenderableId selectedViewportRenderable(
    const SimEngine& engine, const ViewportLayerFilterState& filters);
[[nodiscard]] std::string selectedViewportLabel(
    const SimEngine& engine, const ViewportLayerFilterState& filters);

/// Build and apply viewport layer-filter toggles.
[[nodiscard]] std::vector<ViewportLayerFilterAction>
buildViewportLayerFilterActions(const ViewportLayerFilterState& filters);
[[nodiscard]] ViewportLayerFilterActionResult setViewportLayerVisible(
    ViewportLayerFilterState& filters, ViewportLayerKind kind, bool visible);

/// Build renderer-neutral camera workflow commands for the current scene.
[[nodiscard]] std::vector<ViewportCameraAction> buildViewportCameraActions(
    const SimEngine& engine);
[[nodiscard]] std::vector<ViewportCameraAction> buildViewportCameraActions(
    const SimEngine& engine, const ViewportLayerFilterState& filters);
[[nodiscard]] std::vector<ViewportCameraAction>
buildViewportAllPaneCameraActions(
    const SimEngine& engine,
    const ViewportLayerFilterState& filters,
    const ViewportLayoutState& state);

/// Compute the camera requested by a workflow command.
[[nodiscard]] ViewportCameraActionResult applyViewportCameraAction(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    ViewportCameraActionKind kind);
[[nodiscard]] ViewportCameraActionResult applyViewportCameraAction(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    ViewportCameraActionKind kind,
    const ViewportLayerFilterState& filters);
[[nodiscard]] ViewportCameraActionResult applyViewportCameraActionToAllPanes(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    ViewportCameraActionKind kind,
    const ViewportLayerFilterState& filters,
    ViewportLayoutState& state);

/// Build and apply camera mouse-mode and selection-tracking actions.
[[nodiscard]] std::vector<ViewportCameraControlAction>
buildViewportCameraControlActions(
    const SimEngine& engine,
    const ViewportLayerFilterState& filters,
    const ViewportCameraControlState& controls);
[[nodiscard]] ViewportCameraControlActionResult
applyViewportCameraControlAction(
    const SimEngine& engine,
    const ViewportLayerFilterState& filters,
    ViewportCameraControlState& controls,
    ViewportCameraControlActionKind kind);

/// Convert editor viewport camera controls to the generic GUI input seam.
[[nodiscard]] dart::gui::OrbitCameraControlOptions viewportCameraControlOptions(
    const ViewportCameraControlState& controls);

/// Compute a tracked camera target for the current selected, visible object.
[[nodiscard]] ViewportCameraActionResult trackedSelectionCamera(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    const ViewportLayerFilterState& filters,
    const ViewportCameraControlState& controls);

/// Build and apply single/quad viewport layout commands.
[[nodiscard]] std::vector<ViewportLayoutAction> buildViewportLayoutActions(
    const ViewportLayoutState& state);
[[nodiscard]] ViewportLayoutActionResult applyViewportLayoutAction(
    ViewportLayoutState& state, ViewportLayoutActionKind kind);
[[nodiscard]] std::optional<ViewportLayoutActionKind>
viewportPaneActivationAction(dart::gui::ViewportPaneKind pane);
[[nodiscard]] ViewportLayoutActionResult applyViewportPaneActivation(
    ViewportLayoutState& state, dart::gui::ViewportPaneKind pane);

/// Map a logical viewport pane to its canonical camera preset.
[[nodiscard]] ViewportCameraActionKind viewportPaneCameraAction(
    ViewportPaneKind pane);

/// Remember and restore editor-owned per-pane cameras.
void rememberViewportActivePaneCamera(
    ViewportLayoutState& state, const dart::gui::OrbitCamera& camera);
[[nodiscard]] ViewportCameraActionResult activeViewportPaneCamera(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    const ViewportLayerFilterState& filters,
    const ViewportLayoutState& state);

/// Convert editor viewport layout state to the generic GUI render layout seam.
[[nodiscard]] dart::gui::ViewportLayoutOptions viewportLayoutOptions(
    const SimEngine& engine,
    const dart::gui::OrbitCamera& currentCamera,
    const ViewportLayerFilterState& filters,
    const ViewportLayoutState& state);

} // namespace dartsim::ui
