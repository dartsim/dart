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

#include <dart/simulation/fwd.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <functional>
#include <optional>
#include <string>
#include <vector>

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
  dart::simulation::WorldPtr world;
  std::optional<RunOptions> runDefaults;
  std::function<void()> preStep;
  std::function<void()> postStep;
  std::function<void()> preRender;
  std::function<void()> postRender;
  bool simulateWorld = true;
  std::optional<OrbitCamera> camera;
  std::function<OrbitCameraControlOptions()> cameraControlsProvider;
  std::function<bool(OrbitCamera&)> cameraUpdater;
  RenderSettings renderSettings;
  std::string defaultScene;
  std::vector<Panel> panels;
  std::vector<Gizmo> gizmos;
  std::function<std::vector<DebugLabelDescriptor>()> debugLabels;
  std::vector<InverseKinematicsHandle> ikHandles;
  std::vector<BodyNodeDragHandle> bodyNodeDragHandles;
  std::vector<KeyboardAction> keyboardActions;

  /// Optional provider of extra renderables appended to the scene each frame.
  ///
  /// Lets an application render geometry that is not backed by the legacy
  /// `dart::simulation::World` (e.g. the experimental-World editor in
  /// `dartsim/ui`). The returned descriptors only need `id`, `geometry`,
  /// `material`, and `worldTransform`; dynamics pointers may stay null.
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

} // namespace dart::gui

#endif // DART_GUI_APPLICATION_HPP_
