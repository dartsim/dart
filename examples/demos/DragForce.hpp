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

#ifndef DART_EXAMPLES_DEMOS_DRAGFORCE_HPP_
#define DART_EXAMPLES_DEMOS_DRAGFORCE_HPP_

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <functional>
#include <memory>
#include <string>

namespace dart_demos {

//==============================================================================
/// Host-wide drag-force interaction (BRIEF-phase3.md #4) plus gizmo-on-
/// selection (#5), since both are "pick a body, spring it toward a target
/// point, render a line" and share the same tunables/status line.
///
///  - Momentary drag: Ctrl+left-click-drag on any BodyNode in any scene picks
///    it and applies a clamped spring force toward the live cursor position
///    (projected onto the plane through the original pick, parallel to the
///    camera -- the same technique DragAndDrop.cpp's unconstrained mode
///    uses) for as long as the button is held.
///  - Gizmo: attachGizmo(selectedBody) drops an InteractiveFrame at the
///    body's transform, draggable via InteractiveFrameDnD's own gizmo
///    handles. While paused, its transform teleports the body (only
///    meaningful for a FreeJoint body -- see the .cpp); while running, it
///    drives the same spring machinery continuously toward the gizmo's
///    current pose instead of the momentary cursor drag.
///
/// One instance is owned by DemoHost for the process lifetime. Its mouse
/// handler is installed once (ensureViewerConfigured()); onSceneInstalled()/
/// reset() rebind it to each scene's world.
///
/// Derives from dart::common::Observer so a drag/gizmo body deleted out from
/// under it mid-scene (e.g. AddDeleteSkelsScene's live delete while a gizmo
/// is attached or a Ctrl+drag is held) is noticed via
/// handleDestructionNotification() and released automatically, instead of
/// dereferencing a dangling BodyNode* every frame in applyPreStep/
/// applyPreRefresh.
class DragForce : public dart::common::Observer
{
public:
  DragForce();
  ~DragForce() override;

  /// Registers the persistent mouse handler that drives Ctrl+drag picking
  /// and plain-click selection. Call once, after the viewer's default event
  /// handler exists. `onPlainClickSelect` is invoked with the picked body (or
  /// nullptr for a miss) on a plain (non-Ctrl) left-click.
  void installMouseHandler(
      dart::gui::osg::ImGuiViewer* viewer,
      std::function<void(dart::dynamics::BodyNode*)> onPlainClickSelect);

  /// Binds the force-line visual to the newly-installed scene's world. Call
  /// from DemoHost::installScene after `world` is set as the current world.
  void onSceneInstalled(const dart::simulation::WorldPtr& world);

  /// Releases the gizmo and clears momentary-drag state. Call from
  /// DemoHost::teardownCurrentScene before the world is destroyed.
  void reset(dart::gui::osg::ImGuiViewer* viewer);

  /// Applies the momentary drag spring and/or the gizmo spring (while
  /// running). Call from the host's composed preStep.
  void applyPreStep();

  /// Updates the force-line visual and, while paused, teleports the gizmo
  /// target (FreeJoint bodies only). Call from the host's composed
  /// preRefresh every render frame.
  void applyPreRefresh(bool paused);

  /// True while the gizmo is attached to a body.
  bool isGizmoAttached() const
  {
    return mGizmoBody != nullptr;
  }

  /// Attaches the gizmo to `body` (detaching any previous gizmo first).
  void attachGizmo(
      dart::dynamics::BodyNode* body,
      const dart::simulation::WorldPtr& world,
      dart::gui::osg::ImGuiViewer* viewer);

  /// Detaches the gizmo, if attached.
  void detachGizmo(dart::gui::osg::ImGuiViewer* viewer);

  /// Renders the compact "Drag: <body> |F|=.. N" toolbar status line.
  void renderToolbarStatus(double guiScale) const;

  /// Renders the coeff/max-force tunable sliders (Diagnostics panel).
  void renderTunables(double guiScale);

  /// Releases the drag/gizmo the moment their body is destroyed, so no later
  /// frame dereferences the freed BodyNode. (dart::common::Observer)
  void handleDestructionNotification(
      const dart::common::Subject* subject) override;

private:
  class MouseHandler;
  friend class MouseHandler;

  void startDrag(
      dart::dynamics::BodyNode* body, const Eigen::Vector3d& worldPoint);
  void endDrag();
  void ensureLineVisual();
  void hideLineVisual();
  Eigen::Vector3d computeDragTarget() const;
  void applySpring(
      dart::dynamics::BodyNode* body,
      const Eigen::Vector3d& localPoint,
      const Eigen::Vector3d& target);
  void teleportToGizmo();

  dart::gui::osg::ImGuiViewer* mViewer = nullptr;
  dart::simulation::WorldPtr mWorld;

  // Momentary Ctrl+drag state.
  dart::dynamics::BodyNode* mDragBody = nullptr;
  Eigen::Vector3d mDragLocalPoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d mDragAnchorWorld = Eigen::Vector3d::Zero();
  bool mDragging = false;

  // Gizmo-on-selection state.
  dart::dynamics::BodyNode* mGizmoBody = nullptr;
  dart::gui::osg::InteractiveFramePtr mGizmoFrame;
  dart::gui::osg::InteractiveFrameDnD* mGizmoDnd = nullptr;

  // Shared force-line visual, lazily bound to the active world.
  dart::dynamics::SimpleFramePtr mLineFrame;
  dart::dynamics::LineSegmentShapePtr mLineShape;

  float mCoeff = 200.0f;
  float mMaxForce = 500.0f;

  // Toolbar status, refreshed by applyPreStep()/applyPreRefresh().
  std::string mStatusBodyName;
  double mStatusForceMag = 0.0;

  // Owned for the process lifetime once installed; see the .cpp for why it
  // is intentionally never deleted (mirrors DemoKeyHandler's persistent,
  // never-torn-down registration).
  MouseHandler* mMouseHandler = nullptr;
};

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_DRAGFORCE_HPP_
