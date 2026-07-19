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

#ifndef DART_EXAMPLES_DEMOS_DEMOSCENE_HPP_
#define DART_EXAMPLES_DEMOS_DEMOSCENE_HPP_

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace dart_demos {

class DemoHost;

//==============================================================================
/// A single keyboard shortcut a scene wants to expose. The host wires this up
/// both as a key handler and as a clickable panel button, so every demo stays
/// fully usable without a keyboard.
struct KeyAction
{
  /// ASCII key code, or an osgGA::GUIEventAdapter::KeySymbol value.
  int key = 0;

  /// Short label shown next to the key in the panel, e.g. "Shoot sphere".
  std::string label;

  /// Invoked on key-down (or panel button click). The host wraps every call
  /// in a try/catch and disables the action after its first failure, so a
  /// broken callback can never crash the app.
  std::function<void()> callback;
};

//==============================================================================
/// Camera pose applied when a scene becomes active.
struct CameraHome
{
  ::osg::Vec3d eye;
  ::osg::Vec3d center;
  ::osg::Vec3d up;
};

//==============================================================================
/// Context handed to DemoSceneSetup::onActivate so a scene can register extra
/// viewer-level resources (attachments, custom event handlers, gizmo-typed
/// drag-and-drop, ...). Everything registered through this context is torn
/// down automatically the next time the host switches away from the scene.
class DemoHostContext
{
public:
  explicit DemoHostContext(DemoHost& host);

  /// The viewer the scene is being installed into.
  dart::gui::osg::ImGuiViewer* viewer() const;

  /// The world node that was just created for this scene.
  dart::gui::osg::WorldNode* worldNode() const;

  /// Registers an arbitrary teardown action to run when this scene is torn
  /// down (on switch, rebuild/reset, or app shutdown).
  void addTeardown(std::function<void()> teardown);

  /// Adds a ViewerAttachment and arranges for it to be removed automatically
  /// on teardown.
  void addAttachment(dart::gui::osg::ViewerAttachment* attachment);

  /// Adds an event handler and arranges for it to be removed automatically on
  /// teardown.
  void addEventHandler(::osgGA::GUIEventHandler* handler);

  /// Appends a line to the host's diagnostics log console.
  void log(const std::string& message) const;

private:
  DemoHost& mHost;
};

//==============================================================================
/// Everything a scene factory builds. The host owns and drives all of this;
/// the scene only supplies data and callbacks.
struct DemoSceneSetup
{
  /// The simulated world. Required; a null world is treated as a factory
  /// failure.
  dart::simulation::WorldPtr world;

  /// Called immediately before each simulation step.
  std::function<void()> preStep;

  /// Called immediately after each simulation step.
  std::function<void()> postStep;

  /// Called once per rendered frame, before the (possible) simulation
  /// step(s) -- unlike preStep/postStep, this runs even while the
  /// simulation is paused (it is wired to WorldNode::customPreRefresh(),
  /// which fires unconditionally at the top of every refresh()). Use this
  /// for visuals that should keep updating while paused (e.g. recoloring by
  /// live state, or an aim-line preview); prefer preStep/postStep for
  /// anything that must only run once per physics step. Optional.
  std::function<void()> preRefresh;

  /// Draws the scene-specific controls the host embeds in its right-hand
  /// panel. Optional.
  std::function<void()> renderPanel;

  /// Keyboard shortcuts, mirrored as panel buttons.
  std::vector<KeyAction> keyActions;

  /// Camera pose to apply when this scene becomes active. Optional.
  std::optional<CameraHome> cameraHome;

  /// Optional canonical JSON contract for the scene's physics and controls.
  /// The headless timeline evaluates this provider after host runtime options
  /// and legacy pre-run actions, immediately before timeline step 0, then
  /// writes the result as a structured object. This keeps the contract bound
  /// to the world that actually ran instead of the scene factory's initial
  /// snapshot.
  /// Renderer-only meshes, materials, and textures belong to separate visual
  /// provenance and must not be represented here.
  std::function<std::string()> physicsContractProvider;

  /// Whether the host should attach its default shadow technique.
  bool enableShadows = true;

  /// Frames to enable drag-and-drop on (SimpleFrameDnD: unconstrained
  /// free-drag). This is the right tool for a plain SimpleFrame that only needs
  /// a free-drag. For an InteractiveFrame whose per-axis/plane gizmo tools
  /// should be draggable, call Viewer::enableDragAndDrop(InteractiveFrame*)
  /// (InteractiveFrameDnD) from onActivate and release it via
  /// ctx.addTeardown(disableDragAndDrop) -- that is now safe to tear down at
  /// runtime (the gui-osg InteractiveFrameDnD destructor deletes the 9 per-tool
  /// sub-DnDs it allocates), whereas before its defaulted destructor leaked
  /// them and left them observing freed entities across a scene switch.
  std::vector<dart::dynamics::SimpleFramePtr> dragFrames;

  /// Optional hook for anything else a scene needs to register (extra
  /// attachments, custom event handlers, ...).
  std::function<void(DemoHostContext&)> onActivate;
};

//==============================================================================
/// Text that must make a scene self-contained in the Scene tab. Most examples
/// leave this empty; paper/research examples should fill all fields so the
/// catalog can verify their in-app explanation coverage.
struct ScenePanelDocumentation
{
  std::string overview;
  std::string expectedResult;
  std::string coverage;

  bool isComplete() const
  {
    return !overview.empty() && !expectedResult.empty() && !coverage.empty();
  }
};

//==============================================================================
/// One entry in the demo catalog. `factory` is only invoked lazily -- the
/// first time the demo is selected (and again on Rebuild/Reset) -- and may
/// throw to signal a startup failure; the host will surface the reason and
/// keep whatever was running before.
struct DemoScene
{
  /// Stable, snake_case identifier used by --scene and --cycle-scenes.
  std::string id;

  /// Human-readable title shown in the navigator and status messages.
  std::string title;

  /// Category used to group rows in the navigator (first-appearance order).
  std::string category;

  /// One-line summary shown as a row tooltip.
  std::string summary;

  /// Optional Scene-tab documentation metadata used by catalog checks.
  ScenePanelDocumentation scenePanelDocumentation;

  /// Builds (or rebuilds) the scene. May throw.
  std::function<DemoSceneSetup()> factory;
};

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_DEMOSCENE_HPP_
