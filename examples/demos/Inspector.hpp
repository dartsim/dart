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

#ifndef DART_EXAMPLES_DEMOS_INSPECTOR_HPP_
#define DART_EXAMPLES_DEMOS_INSPECTOR_HPP_

#include <dart/gui/osg/osg.hpp>

#include <dart/common/Observer.hpp>

#include <dart/dart.hpp>

#include <unordered_map>
#include <utility>
#include <vector>

namespace osg {
class Group;
} // namespace osg

namespace dart_demos {

//==============================================================================
/// Host-owned scene tree + body inspector (BRIEF-phase3.md #2): skeletons ->
/// bodies (tree) with a single host-owned selection, a detail view (world/
/// relative transform, velocities, mass, per-joint position/velocity/limits
/// with paused-only clamped sliders), and per-body wireframe/hide/highlight
/// visual toggles. One instance is owned by DemoHost and reset() on every
/// scene switch (the BodyNode/DegreeOfFreedom pointers it tracks belong to
/// the world that is about to be destroyed).
///
/// Derives from dart::common::Observer so a selection that gets deleted out
/// from under it (e.g. AddDeleteSkelsScene's live add/delete) is noticed via
/// handleDestructionNotification() and cleared automatically, instead of
/// leaving a dangling BodyNode*.
class Inspector : public dart::common::Observer
{
public:
  Inspector() = default;

  /// Clears selection (restoring any highlight tint first), pending joint
  /// edits, and per-body visual-toggle bookkeeping. Called whenever the
  /// active scene is torn down.
  void reset();

  /// The currently selected body, or nullptr if none. Read by DemoHost to
  /// drive the "Attach gizmo" affordance and the host-wide drag-force pick.
  dart::dynamics::BodyNode* getSelection() const
  {
    return mSelected;
  }

  /// Selects `body` (nullptr clears the selection). Safe to call with the
  /// already-selected body (a no-op) or with a body from a different world
  /// than the last selection (the caller is expected to reset() on scene
  /// switch, but this does not assume it).
  void setSelection(dart::dynamics::BodyNode* body);

  /// Renders the skeleton -> body tree in the caller's current ImGui window;
  /// clicking a row selects it.
  void renderTree(const dart::simulation::WorldPtr& world);

  /// Renders the detail view for the current selection: transforms,
  /// velocities, mass, per-joint sliders (editable only while `paused`), and
  /// the wireframe/hidden toggles. `worldNode` is the active scene's OSG
  /// group (DemoHost::getWorldNode()), used to locate the selected body's
  /// rendered nodes for the wireframe toggle; may be nullptr (the toggle is
  /// then a no-op checkbox).
  void renderDetail(bool paused, ::osg::Group* worldNode);

  /// Applies every pending joint-position edit queued by renderDetail's
  /// sliders (clamped to limits, isfinite-checked) and clears the queue.
  /// Safe to call every preStep and every preRefresh (idempotent once the
  /// queue is empty) -- see the .cpp for why both call sites exist.
  void applyQueuedEdits();

  /// Advances the selected body's highlight pulse. `wallTimeSeconds` should
  /// be a monotonically increasing wall-clock time (e.g. ImGui::GetTime())
  /// so the highlight keeps animating even while the simulation is paused.
  void updateHighlight(double wallTimeSeconds);

protected:
  // Documentation inherited (dart::common::Observer)
  void handleDestructionNotification(
      const dart::common::Subject* subject) override;

private:
  struct BodyVisualState
  {
    bool wireframe = false;
    bool hidden = false;
  };

  void renderBodyRow(dart::dynamics::BodyNode* body);
  void renderJointRow(dart::dynamics::Joint* joint);
  void setWireframe(
      dart::dynamics::BodyNode* body, bool enabled, ::osg::Group* worldNode);
  void setHidden(dart::dynamics::BodyNode* body, bool hidden);
  void snapshotHighlightColors();
  void restoreHighlight();

  dart::dynamics::BodyNode* mSelected = nullptr;

  /// (ShapeNode, original RGBA) pairs snapshotted when mSelected was set, so
  /// updateHighlight() can blend from the real color and restoreHighlight()
  /// can put it back exactly. Cleared (without restoring) if the selection
  /// itself was destroyed out from under us.
  std::vector<std::pair<dart::dynamics::ShapeNode*, Eigen::Vector4d>>
      mHighlightSnapshot;

  std::unordered_map<dart::dynamics::BodyNode*, BodyVisualState>
      mBodyVisualState;

  /// Queued joint-position edits from the paused-only sliders in
  /// renderDetail(), flushed by applyQueuedEdits().
  std::unordered_map<dart::dynamics::DegreeOfFreedom*, double> mPendingEdits;
};

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_INSPECTOR_HPP_
