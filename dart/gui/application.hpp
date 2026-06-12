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

#ifndef DART_GUI_APPLICATION_HPP_
#define DART_GUI_APPLICATION_HPP_

#include <dart/gui/debug.hpp>
#include <dart/gui/export.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <cstddef>

namespace dart::gui {

enum class InverseKinematicsSolveMode
{
  /// Solve the IK module attached to the target handle directly.
  Target,

  /// Solve the owning Skeleton's HierarchicalIK module.
  SkeletonHierarchy,
};

struct InverseKinematicsHandle
{
  std::string label;
  int hotkey = 0;
  dart::dynamics::SimpleFramePtr target;
  dart::dynamics::InverseKinematicsPtr ik;
  /// Selects local target IK or the owning skeleton hierarchy for GUI nudges.
  InverseKinematicsSolveMode solveMode = InverseKinematicsSolveMode::Target;
};

struct BodyNodeDragHandle
{
  std::string label;
  dart::dynamics::BodyNode* bodyNode = nullptr;
  bool useWholeBody = false;
};

struct RenderableSelection
{
  RenderableId id = 0;
  std::string label;
};

/// Payload for the viewer's mouse "force-drag" (a mouse spring).
///
/// Emitted by the selection controller when the user left-drags a renderable
/// that is not backed by a legacy `dart::dynamics::BodyNode` (e.g. the
/// SimpleFrame mirrors of a DART 7 `sx::World`). Applications resolve
/// `renderableName` to their own body and (re)apply `force` at
/// `applicationPoint` (both world-frame) every step while `active` is true,
/// then stop once a final event with `active == false` arrives. Renderables
/// backed by a BodyNode are handled in-engine via `addExtForce` and never
/// reach this callback.
struct ForceDragEvent
{
  RenderableId renderableId = 0;
  std::string renderableName;
  Eigen::Vector3d applicationPoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  bool active = false;
};

inline constexpr std::size_t kMaxViewportPanes = 4u;

enum class ViewportLayoutMode
{
  Single,
  Quad,
};

enum class ViewportPaneKind
{
  Perspective,
  Front,
  Right,
  Top,
};

struct ViewportPaneView
{
  ViewportPaneKind kind = ViewportPaneKind::Perspective;
  OrbitCamera camera;
  bool active = false;
};

struct ViewportLayoutOptions
{
  ViewportLayoutMode mode = ViewportLayoutMode::Single;
  std::array<ViewportPaneView, kMaxViewportPanes> panes;
  std::size_t paneCount = 1u;
};

enum class KeyboardKey
{
  Unknown,
  Tab,
  Enter,
  Backspace,
  Delete,
  Up,
  Down,
  Left,
  Right,
  PageUp,
  PageDown,
  GraveAccent,
};

struct KeyboardShortcut
{
  char character = '\0';
  KeyboardKey key = KeyboardKey::Unknown;

  static KeyboardShortcut characterKey(char value)
  {
    KeyboardShortcut shortcut;
    shortcut.character = value;
    return shortcut;
  }

  static KeyboardShortcut namedKey(KeyboardKey value)
  {
    KeyboardShortcut shortcut;
    shortcut.key = value;
    return shortcut;
  }
};

struct KeyboardActionContext
{
  ViewerLifecycleState* lifecycle = nullptr;
  RenderSettings* renderSettings = nullptr;
  std::function<void()> resetCamera;
};

enum class KeyboardActionTrigger
{
  Press,
  Release,
};

struct KeyboardAction
{
  std::string label;
  KeyboardShortcut shortcut;
  bool repeat = false;
  KeyboardActionTrigger trigger = KeyboardActionTrigger::Press;
  std::function<void(KeyboardActionContext&)> callback;
};

struct ApplicationOptions
{
  std::optional<RunOptions> runDefaults;
  std::function<void()> preStep;
  std::function<void()> postStep;
  std::function<void()> preRender;
  std::function<void()> postRender;
  double timeStep = 1.0 / 60.0;
  bool advanceSimulation = true;
  std::optional<OrbitCamera> camera;
  std::function<OrbitCameraControlOptions()> cameraControlsProvider;
  std::function<bool(OrbitCamera&)> cameraUpdater;
  std::function<ViewportLayoutOptions(const OrbitCamera&)>
      viewportLayoutProvider;

  /// Optional callback fired when a multi-view viewport pane is activated.
  ///
  /// The callback receives the logical renderer pane selected by a viewport
  /// click. Applications can use it to update editor-owned active-pane state
  /// without exposing backend renderer objects.
  std::function<void(ViewportPaneKind)> onViewportPaneActivated;

  RenderSettings renderSettings;
  std::string defaultScene;

  /// Permit launching with a scene that has no visible renderables.
  ///
  /// Built-in scene fixtures must extract visible content, so the startup path
  /// rejects an empty scene by default to catch broken fixtures. Applications
  /// that legitimately open empty (e.g. the dartsim editor's empty workspace)
  /// set this to skip that check; null debug overlays are then tolerated.
  bool allowEmptyScene = false;

  std::vector<Panel> panels;
  std::vector<Gizmo> gizmos;
  std::function<std::vector<DebugLabelDescriptor>()> debugLabels;

  /// Optional provider of per-frame debug geometry drawn into the built-in
  /// overlay.
  ///
  /// Lets any application contribute custom debug lines, triangles, and labels
  /// through one DART concept (`DebugScene`) each frame. The geometry renders
  /// with the same unlit, no-shadow, always-on-top treatment as the built-in
  /// debug overlays, so hosts no longer need to build their own overlay wiring.
  ///
  /// The provider runs on the frame loop's thread once per loop iteration
  /// (including frames the renderer skips for pacing) and its output is
  /// re-uploaded each call, so keep it cheap; derive geometry from state you
  /// already track rather than re-scanning large worlds. World stepping runs
  /// on the same thread, so reading live world state is safe today.
  std::function<DebugScene()> debugProvider;
  std::vector<InverseKinematicsHandle> ikHandles;
  std::vector<BodyNodeDragHandle> bodyNodeDragHandles;
  std::vector<KeyboardAction> keyboardActions;

  /// Optional provider of extra renderables appended to the scene each frame.
  ///
  /// Lets an application render geometry from any simulation source. The
  /// returned descriptors only need `id`, `geometry`, `material`, and
  /// `worldTransform`; dynamics pointers may stay null.
  std::function<std::vector<RenderableDescriptor>()> renderableProvider;

  /// Optional provider for the app-owned selected renderable.
  ///
  /// This keeps renderer selection synchronized with application state for
  /// renderables supplied through `renderableProvider`, without exposing
  /// renderer backend types to the application. Return id 0 for no selection.
  std::function<RenderableSelection()> selectedRenderableProvider;

  /// Optional callback fired after viewport picking changes selection.
  ///
  /// The callback receives id 0 when a click clears selection. Applications can
  /// ignore ids they do not own.
  std::function<void(RenderableId)> onRenderableSelected;

  /// Optional callback fired while the user mouse "force-drags" a renderable
  /// that is not backed by a legacy `dart::dynamics::BodyNode`.
  ///
  /// The selection controller computes a world-frame spring-damper force and
  /// invokes this each frame with `active == true` (the application must
  /// (re)apply the one-shot force to its own body every step), then once with
  /// `active == false` on release. Used by the DART 7 World demos to
  /// push/pull `sx::World` links rendered via SimpleFrame mirrors.
  std::function<void(const ForceDragEvent&)> onForceDrag;

  /// Enables a docking workspace over the main viewport so panels can be
  /// docked and rearranged. Only takes effect when the GUI build was compiled
  /// with docking support (see `isDockingAvailable()`); otherwise it is
  /// silently ignored.
  bool dockingEnabled = false;
};

/// Returns true if this GUI build was compiled with docking support. When
/// false, `ApplicationOptions::dockingEnabled` has no effect; see the build
/// documentation for enabling a docking-capable GUI build.
DART_GUI_API bool isDockingAvailable();

DART_GUI_API int runApplication(int argc, char* argv[]);

DART_GUI_API int runApplication(
    int argc, char* argv[], const char* defaultScene);

DART_GUI_API int runApplication(
    int argc, char* argv[], const ApplicationOptions& options);

/// One entry in a runtime-switchable demo catalog.
///
/// `factory` lazily builds the scene's `ApplicationOptions` (world, panels,
/// gizmos, handlers, camera) when the scene is first selected, so launching the
/// host stays fast and an asset/remote failure affects only that scene. `id` is
/// the stable identifier used by `--scene <id>` and by tests; `category` groups
/// scenes in the sidebar (categories are ordered by first appearance, scenes by
/// their order in the catalog vector).
struct DemoSceneEntry
{
  std::string id;
  std::string title;
  std::string category;
  std::string summary;
  std::function<ApplicationOptions()> factory;
};

/// Run a single window that hosts multiple demo scenes selectable at runtime
/// from a built-in "Demos" sidebar. Selecting a scene swaps the active
/// world/panels/handlers in place without recreating the window.
///
/// Initial scene: `--scene <id>` or the `DART_DEMOS_SCENE` environment
/// variable; otherwise the first catalog entry. `--cycle-scenes` advances
/// through every scene for a few frames each and then exits (used by headless
/// smoke tests). Remaining flags (`--headless`, `--frames`, `--screenshot`,
/// `--width`,
/// `--height`, `--backend`, ...) match `runApplication`.
DART_GUI_API int runDemos(
    int argc, char* argv[], std::vector<DemoSceneEntry> scenes);

} // namespace dart::gui

#endif // DART_GUI_APPLICATION_HPP_
