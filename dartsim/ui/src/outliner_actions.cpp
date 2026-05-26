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

#include <Eigen/Geometry>
#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/selection_manager.hpp>
#include <dartsim_ui/outliner_actions.hpp>

#include <algorithm>
#include <limits>
#include <utility>

#include <cctype>
#include <cstddef>

namespace dartsim::ui {

namespace {

void appendOutlinerRows(
    const SimEngine& engine,
    const OutlinerState* state,
    ObjectId id,
    int depth,
    std::vector<OutlinerRow>& rows)
{
  const SceneModel& model = engine.objects().model();
  const SceneObject* object = model.find(id);
  if (object == nullptr) {
    return;
  }

  const auto& children = model.childrenOf(id);
  const bool hasChildren = !children.empty();
  const bool expanded = !hasChildren || state == nullptr
                        || state->collapsed.find(id) == state->collapsed.end();
  const bool renaming = state != nullptr && state->renameTarget == id;
  rows.push_back(
      OutlinerRow{
          object->id,
          object->parent,
          depth,
          object->name,
          objectTypeLabel(object->type),
          engine.selection().isSelected(id),
          object->visible,
          hasChildren,
          expanded,
          renaming});

  if (!expanded) {
    return;
  }
  for (const ObjectId child : children) {
    appendOutlinerRows(engine, state, child, depth + 1, rows);
  }
}

bool isMovable(ObjectType type)
{
  return type == ObjectType::RigidBody || type == ObjectType::FreeFrame
         || type == ObjectType::FixedFrame;
}

bool hasVisibleRenderable(const SimEngine& engine, ObjectId id)
{
  const std::vector<RenderItem> renderItems = engine.renderItems();
  return std::find_if(
             renderItems.begin(),
             renderItems.end(),
             [id](const RenderItem& item) { return item.id == id; })
         != renderItems.end();
}

std::string trimmed(std::string value)
{
  const auto isSpace = [](unsigned char c) {
    return std::isspace(c) != 0;
  };
  value.erase(
      value.begin(), std::find_if_not(value.begin(), value.end(), isSpace));
  value.erase(
      std::find_if_not(value.rbegin(), value.rend(), isSpace).base(),
      value.end());
  return value;
}

void pruneOutlinerState(OutlinerState& state, const SimEngine& engine)
{
  for (auto it = state.collapsed.begin(); it != state.collapsed.end();) {
    if (!engine.objects().model().contains(*it)) {
      it = state.collapsed.erase(it);
    } else {
      ++it;
    }
  }
  if (state.renameTarget != kNoObject
      && !engine.objects().model().contains(state.renameTarget)) {
    state.renameTarget = kNoObject;
    state.renameDraft.clear();
  }
}

OutlinerActionResult result(bool ok, std::string message)
{
  return {ok, std::move(message)};
}

} // namespace

std::string objectTypeLabel(ObjectType type)
{
  switch (type) {
    case ObjectType::RigidBody:
      return "RigidBody";
    case ObjectType::MultiBody:
      return "MultiBody";
    case ObjectType::Link:
      return "Link";
    case ObjectType::Joint:
      return "Joint";
    case ObjectType::FreeFrame:
      return "FreeFrame";
    case ObjectType::FixedFrame:
      return "FixedFrame";
  }
  return "Object";
}

std::vector<OutlinerRow> buildOutlinerRows(const SimEngine& engine)
{
  const SceneModel& model = engine.objects().model();
  std::vector<OutlinerRow> rows;
  rows.reserve(model.size());
  for (const ObjectId id : model.rootChildren()) {
    appendOutlinerRows(engine, nullptr, id, 0, rows);
  }
  return rows;
}

std::vector<OutlinerRow> buildOutlinerRows(
    const SimEngine& engine, const OutlinerState& state)
{
  const SceneModel& model = engine.objects().model();
  std::vector<OutlinerRow> rows;
  rows.reserve(model.size());
  for (const ObjectId id : model.rootChildren()) {
    appendOutlinerRows(engine, &state, id, 0, rows);
  }
  return rows;
}

std::string outlinerButtonLabel(const OutlinerRow& row)
{
  std::string label(static_cast<std::size_t>(row.depth) * 2u, ' ');
  label += row.selected ? "* " : "  ";
  label += row.name;
  label += " [";
  label += row.type;
  label += "]##outliner-";
  label += std::to_string(row.id);
  return label;
}

std::string outlinerExpansionButtonLabel(const OutlinerRow& row)
{
  if (!row.hasChildren) {
    return {};
  }
  return std::string(
             row.expanded ? "-##outliner-collapse-" : "+##outliner-expand-")
         + std::to_string(row.id);
}

bool selectOutlinerObject(SimEngine& engine, ObjectId id, bool additive)
{
  if (!engine.objects().model().contains(id)) {
    return false;
  }
  return engine.select(id, additive);
}

bool toggleOutlinerObjectSelection(SimEngine& engine, ObjectId id)
{
  if (!engine.objects().model().contains(id)) {
    return false;
  }
  return engine.toggleSelection(id);
}

bool clearOutlinerSelection(SimEngine& engine)
{
  return engine.clearSelection();
}

bool setOutlinerExpanded(
    OutlinerState& state, const SimEngine& engine, ObjectId id, bool expanded)
{
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr || engine.objects().model().childrenOf(id).empty()) {
    return false;
  }

  const bool wasExpanded = state.collapsed.find(id) == state.collapsed.end();
  if (wasExpanded == expanded) {
    return false;
  }

  if (expanded) {
    state.collapsed.erase(id);
  } else {
    state.collapsed.insert(id);
  }
  (void)object;
  return true;
}

bool toggleOutlinerExpanded(
    OutlinerState& state, const SimEngine& engine, ObjectId id)
{
  const bool expanded = state.collapsed.find(id) == state.collapsed.end();
  return setOutlinerExpanded(state, engine, id, !expanded);
}

bool setOutlinerVisibility(SimEngine& engine, ObjectId id, bool visible)
{
  if (!engine.canEditScene()) {
    return false;
  }
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr || object->visible == visible) {
    return false;
  }
  engine.execute(commands::setVisible(id, visible));
  const SceneObject* updated = engine.objects().model().find(id);
  return updated != nullptr && updated->visible == visible;
}

bool toggleOutlinerVisibility(SimEngine& engine, ObjectId id)
{
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr) {
    return false;
  }
  return setOutlinerVisibility(engine, id, !object->visible);
}

std::vector<OutlinerContextAction> buildOutlinerContextActions(
    const SimEngine& engine, ObjectId id)
{
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr) {
    return {};
  }

  const bool canEdit = engine.canEditScene();
  std::vector<OutlinerContextAction> actions;
  actions.push_back({OutlinerContextActionKind::Rename, "Rename", canEdit});
  actions.push_back({OutlinerContextActionKind::Delete, "Delete", canEdit});
  actions.push_back(
      {object->visible ? OutlinerContextActionKind::Hide
                       : OutlinerContextActionKind::Show,
       object->visible ? "Hide" : "Show",
       canEdit});
  return actions;
}

OutlinerActionResult beginOutlinerRename(
    OutlinerState& state, const SimEngine& engine, ObjectId id)
{
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr) {
    return result(false, "Missing object");
  }
  state.renameTarget = id;
  state.renameDraft = object->name;
  return result(true, "Renaming " + object->name);
}

bool updateOutlinerRenameDraft(OutlinerState& state, std::string name)
{
  if (state.renameTarget == kNoObject) {
    return false;
  }
  state.renameDraft = std::move(name);
  return true;
}

OutlinerActionResult cancelOutlinerRename(OutlinerState& state)
{
  if (state.renameTarget == kNoObject) {
    return result(false, "No rename in progress");
  }
  state.renameTarget = kNoObject;
  state.renameDraft.clear();
  return result(true, "Rename cancelled");
}

OutlinerActionResult renameOutlinerObject(
    SimEngine& engine, ObjectId id, std::string name)
{
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }

  name = trimmed(std::move(name));
  if (name.empty()) {
    return result(false, "Name required");
  }

  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr) {
    return result(false, "Missing object");
  }
  const std::string oldName = object->name;
  if (oldName == name) {
    return result(false, "Name unchanged");
  }

  engine.execute(commands::rename(id, name));
  const SceneObject* updated = engine.objects().model().find(id);
  if (updated == nullptr || updated->name != name) {
    return result(false, "Rename rejected");
  }
  return result(true, "Renamed " + oldName + " to " + name);
}

OutlinerActionResult commitOutlinerRename(
    SimEngine& engine, OutlinerState& state)
{
  if (state.renameTarget == kNoObject) {
    return result(false, "No rename in progress");
  }
  const OutlinerActionResult rename
      = renameOutlinerObject(engine, state.renameTarget, state.renameDraft);
  if (rename.ok) {
    state.renameTarget = kNoObject;
    state.renameDraft.clear();
  }
  return rename;
}

OutlinerActionResult applyOutlinerContextAction(
    SimEngine& engine,
    OutlinerState& state,
    ObjectId id,
    OutlinerContextActionKind kind)
{
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr) {
    pruneOutlinerState(state, engine);
    return result(false, "Missing object");
  }

  switch (kind) {
    case OutlinerContextActionKind::Rename:
      return beginOutlinerRename(state, engine, id);
    case OutlinerContextActionKind::Delete: {
      if (!engine.canEditScene()) {
        return result(false, "Scene locked");
      }
      const std::string name = object->name;
      engine.execute(commands::removeObject(id));
      pruneOutlinerState(state, engine);
      return engine.objects().model().contains(id)
                 ? result(false, "Delete rejected")
                 : result(true, "Deleted " + name);
    }
    case OutlinerContextActionKind::Show:
      return setOutlinerVisibility(engine, id, true)
                 ? result(true, "Shown " + object->name)
                 : result(false, "Show rejected");
    case OutlinerContextActionKind::Hide:
      return setOutlinerVisibility(engine, id, false)
                 ? result(true, "Hidden " + object->name)
                 : result(false, "Hide rejected");
  }
  return result(false, "Unknown action");
}

dart::gui::RenderableId renderableIdForObject(ObjectId id)
{
  if (id == kNoObject
      || id > static_cast<ObjectId>(
             std::numeric_limits<dart::gui::RenderableId>::max())) {
    return 0;
  }
  return static_cast<dart::gui::RenderableId>(id);
}

ObjectId objectIdForRenderable(dart::gui::RenderableId id)
{
  if (id == 0
      || id > static_cast<dart::gui::RenderableId>(
             std::numeric_limits<ObjectId>::max())) {
    return kNoObject;
  }
  return static_cast<ObjectId>(id);
}

bool selectViewportRenderable(
    SimEngine& engine, dart::gui::RenderableId renderableId)
{
  const ObjectId id = objectIdForRenderable(renderableId);
  if (id == kNoObject || !engine.objects().model().contains(id)
      || !hasVisibleRenderable(engine, id)) {
    return false;
  }
  return selectOutlinerObject(engine, id);
}

dart::gui::RenderableId selectedViewportRenderable(const SimEngine& engine)
{
  const ObjectId selected = engine.selection().primary();
  if (selected == kNoObject || !hasVisibleRenderable(engine, selected)) {
    return 0;
  }
  return renderableIdForObject(selected);
}

std::string selectedViewportLabel(const SimEngine& engine)
{
  const ObjectId selected = engine.selection().primary();
  if (selected == kNoObject || !hasVisibleRenderable(engine, selected)) {
    return "none";
  }

  const SceneObject* object = engine.objects().model().find(selected);
  if (object == nullptr) {
    return "none";
  }

  return object->name + " [" + objectTypeLabel(object->type) + "]";
}

bool moveSelectedBy(SimEngine& engine, const Eigen::Vector3d& delta)
{
  if (!engine.canEditScene() || delta.isZero()) {
    return false;
  }
  const ObjectId id = engine.selection().primary();
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr || !isMovable(object->type)) {
    return false;
  }

  Eigen::Isometry3d transform = object->transform;
  transform.translation() += delta;
  engine.execute(commands::setTransform(id, transform));

  const SceneObject* updated = engine.objects().model().find(id);
  return updated != nullptr
         && updated->transform.translation().isApprox(transform.translation());
}

} // namespace dartsim::ui
