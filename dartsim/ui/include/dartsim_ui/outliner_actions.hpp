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

#include <dart/gui/renderable.hpp>

#include <Eigen/Core>
#include <dartsim_engine/scene_object.hpp>
#include <dartsim_engine/sim_engine.hpp>

#include <set>
#include <string>
#include <vector>

namespace dartsim::ui {

/// One flattened scene-tree row consumed by the backend-hidden editor panels.
struct OutlinerRow
{
  ObjectId id = kNoObject;
  ObjectId parent = kNoObject;
  int depth = 0;
  std::string name;
  std::string type;
  std::string icon;
  bool selected = false;
  bool visible = true;
  bool hasChildren = false;
  bool expanded = true;
  bool renaming = false;
};

/// Editor-owned outliner UI state. The scene model remains authoritative; this
/// only tracks how the tree is presented and any pending inline rename draft.
struct OutlinerState
{
  std::set<ObjectId> collapsed;
  ObjectId renameTarget = kNoObject;
  std::string renameDraft;
};

enum class OutlinerContextActionKind
{
  Rename,
  Delete,
  Show,
  Hide,
};

struct OutlinerContextAction
{
  OutlinerContextActionKind kind = OutlinerContextActionKind::Rename;
  std::string label;
  bool enabled = false;
};

/// Result of a user-facing outliner action.
struct OutlinerActionResult
{
  bool ok = false;
  std::string message;
};

/// Human-readable object type label used by outliner and inspector panels.
[[nodiscard]] std::string objectTypeLabel(ObjectType type);
/// Compact object-type icon used by outliner rows.
[[nodiscard]] std::string objectTypeIcon(ObjectType type);

/// Build a deterministic, depth-first outliner snapshot from the scene model.
[[nodiscard]] std::vector<OutlinerRow> buildOutlinerRows(
    const SimEngine& engine);
[[nodiscard]] std::vector<OutlinerRow> buildOutlinerRows(
    const SimEngine& engine, const OutlinerState& state);

/// Label used for an ImGui-style tree row button.
[[nodiscard]] std::string outlinerButtonLabel(const OutlinerRow& row);
[[nodiscard]] std::string outlinerExpansionButtonLabel(const OutlinerRow& row);

/// Select a row target when it still exists in the scene.
bool selectOutlinerObject(
    SimEngine& engine, ObjectId id, bool additive = false);
/// Toggle a row target in the current multi-selection.
bool toggleOutlinerObjectSelection(SimEngine& engine, ObjectId id);
/// Clear selection from the outliner action seam.
bool clearOutlinerSelection(SimEngine& engine);

bool setOutlinerExpanded(
    OutlinerState& state, const SimEngine& engine, ObjectId id, bool expanded);
bool toggleOutlinerExpanded(
    OutlinerState& state, const SimEngine& engine, ObjectId id);

/// Set or toggle object visibility through the undoable command stack.
bool setOutlinerVisibility(SimEngine& engine, ObjectId id, bool visible);
bool toggleOutlinerVisibility(SimEngine& engine, ObjectId id);

[[nodiscard]] std::vector<OutlinerContextAction> buildOutlinerContextActions(
    const SimEngine& engine, ObjectId id);
OutlinerActionResult applyOutlinerContextAction(
    SimEngine& engine,
    OutlinerState& state,
    ObjectId id,
    OutlinerContextActionKind kind);

OutlinerActionResult beginOutlinerRename(
    OutlinerState& state, const SimEngine& engine, ObjectId id);
bool updateOutlinerRenameDraft(OutlinerState& state, std::string name);
OutlinerActionResult cancelOutlinerRename(OutlinerState& state);
OutlinerActionResult renameOutlinerObject(
    SimEngine& engine, ObjectId id, std::string name);
OutlinerActionResult commitOutlinerRename(
    SimEngine& engine, OutlinerState& state);

/// Convert between editor object ids and backend-hidden viewport renderable
/// ids.
[[nodiscard]] dart::gui::RenderableId renderableIdForObject(ObjectId id);
[[nodiscard]] ObjectId objectIdForRenderable(dart::gui::RenderableId id);

/// Select a visible viewport renderable through the same engine selection path
/// used by the scene outliner.
bool selectViewportRenderable(
    SimEngine& engine, dart::gui::RenderableId renderableId);

/// Current selected viewport renderable and label, or id 0 / "none".
[[nodiscard]] dart::gui::RenderableId selectedViewportRenderable(
    const SimEngine& engine);
[[nodiscard]] std::string selectedViewportLabel(const SimEngine& engine);

/// Move the primary selected movable object by `delta`.
bool moveSelectedBy(SimEngine& engine, const Eigen::Vector3d& delta);

} // namespace dartsim::ui
