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

#include <dartsim_engine/scene_model.hpp>
#include <dartsim_ui/console_actions.hpp>
#include <dartsim_ui/inspector_actions.hpp>
#include <dartsim_ui/outliner_actions.hpp>
#include <dartsim_ui/palette_actions.hpp>
#include <dartsim_ui/project_actions.hpp>
#include <dartsim_ui/relationship_actions.hpp>
#include <dartsim_ui/simulation_actions.hpp>
#include <dartsim_ui/watch_actions.hpp>

#include <algorithm>
#include <charconv>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <span>
#include <sstream>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cctype>
#include <cstddef>

namespace dartsim::ui {

namespace {

constexpr std::size_t kMaxConsoleStepCount = 1000;

ConsoleCommandResult result(bool ok, std::string message)
{
  return {ok, std::move(message)};
}

std::string lower(std::string_view value)
{
  std::string lowered(value);
  std::transform(
      lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
  return lowered;
}

std::string joinArguments(
    std::span<const std::string> tokens, std::size_t first)
{
  std::string joined;
  for (std::size_t i = first; i < tokens.size(); ++i) {
    if (!joined.empty()) {
      joined += " ";
    }
    joined += tokens[i];
  }
  return joined;
}

std::string ensureProjectExtension(std::string path)
{
  if (std::filesystem::path(path).extension().empty()) {
    path += ".dartsim";
  }
  return path;
}

bool hasFlag(std::span<const std::string> tokens, std::string_view flag)
{
  return std::find(tokens.begin(), tokens.end(), flag) != tokens.end();
}

bool hasWatchedObject(const WatchState& watch, ObjectId id)
{
  return std::find_if(
             watch.targets.begin(),
             watch.targets.end(),
             [id](const WatchTarget& target) { return target.id == id; })
         != watch.targets.end();
}

std::optional<WatchValueKind> parseWatchSignalKind(std::string_view token)
{
  const std::string signal = lower(token);
  if (signal == "time" || signal == "sim-time" || signal == "simulation-time") {
    return WatchValueKind::SimulationTime;
  }
  if (signal == "frame" || signal == "frame-count") {
    return WatchValueKind::FrameCount;
  }
  if (signal == "x" || signal == "translation-x") {
    return WatchValueKind::TranslationX;
  }
  if (signal == "y" || signal == "translation-y") {
    return WatchValueKind::TranslationY;
  }
  if (signal == "z" || signal == "height" || signal == "translation-z") {
    return WatchValueKind::TranslationZ;
  }
  if (signal == "mass") {
    return WatchValueKind::Mass;
  }
  if (signal == "joint" || signal == "joint-position") {
    return WatchValueKind::JointPosition;
  }
  if (signal == "sensor-range" || signal == "range") {
    return WatchValueKind::SensorRange;
  }
  if (signal == "sensor-fov" || signal == "field-of-view") {
    return WatchValueKind::SensorFieldOfView;
  }
  if (signal == "sensor-rate" || signal == "sensor-update-rate") {
    return WatchValueKind::SensorUpdateRate;
  }
  return std::nullopt;
}

std::optional<bool> parseOnOff(std::string_view token)
{
  const std::string value = lower(token);
  if (value == "on" || value == "enable" || value == "enabled") {
    return true;
  }
  if (value == "off" || value == "disable" || value == "disabled") {
    return false;
  }
  return std::nullopt;
}

std::optional<ObjectId> parseObjectId(std::string_view token)
{
  ObjectId id = 0;
  const char* first = token.data();
  const char* last = token.data() + token.size();
  const std::from_chars_result parsed = std::from_chars(first, last, id);
  if (parsed.ec != std::errc() || parsed.ptr != last || id == kNoObject) {
    return std::nullopt;
  }
  return id;
}

std::optional<std::size_t> parseSize(std::string_view token)
{
  std::size_t value = 0;
  const char* first = token.data();
  const char* last = token.data() + token.size();
  const std::from_chars_result parsed = std::from_chars(first, last, value);
  if (parsed.ec != std::errc() || parsed.ptr != last) {
    return std::nullopt;
  }
  return value;
}

struct TargetResolution
{
  ObjectId id = kNoObject;
  std::string error;
};

TargetResolution resolveObjectTarget(
    const SimEngine& engine, std::string_view target)
{
  if (target == "selected") {
    const ObjectId selected = engine.selection().primary();
    if (selected == kNoObject) {
      return {kNoObject, "No object selected"};
    }
    return {selected, {}};
  }

  const SceneModel& model = engine.objects().model();
  if (const std::optional<ObjectId> id = parseObjectId(target)) {
    if (model.contains(*id)) {
      return {*id, {}};
    }
  }

  std::vector<ObjectId> matches;
  for (const ObjectId id : model.allIds()) {
    const SceneObject* object = model.find(id);
    if (object != nullptr && object->name == target) {
      matches.push_back(id);
    }
  }

  if (matches.empty()) {
    return {kNoObject, "Object not found: " + std::string(target)};
  }
  if (matches.size() > 1) {
    return {kNoObject, "Ambiguous object name: " + std::string(target)};
  }
  return {matches.front(), {}};
}

enum class ListObjectFilter
{
  All,
  Selected,
  Visible,
  Hidden,
};

std::optional<ListObjectFilter> parseListObjectFilter(std::string_view token)
{
  const std::string filter = lower(token);
  if (filter == "all") {
    return ListObjectFilter::All;
  }
  if (filter == "selected" || filter == "selection") {
    return ListObjectFilter::Selected;
  }
  if (filter == "visible" || filter == "shown") {
    return ListObjectFilter::Visible;
  }
  if (filter == "hidden") {
    return ListObjectFilter::Hidden;
  }
  return std::nullopt;
}

bool effectiveRowVisibility(const SimEngine& engine, ObjectId id)
{
  const SceneModel& model = engine.objects().model();
  const SceneObject* object = model.find(id);
  std::size_t remaining = model.size();
  while (object != nullptr && remaining > 0) {
    if (!object->visible) {
      return false;
    }
    if (object->parent == kNoObject) {
      return true;
    }
    object = model.find(object->parent);
    --remaining;
  }
  return object != nullptr;
}

bool rowMatchesFilter(
    const OutlinerRow& row, ListObjectFilter filter, bool effectivelyVisible)
{
  switch (filter) {
    case ListObjectFilter::All:
      return true;
    case ListObjectFilter::Selected:
      return row.selected;
    case ListObjectFilter::Visible:
      return effectivelyVisible;
    case ListObjectFilter::Hidden:
      return !effectivelyVisible;
  }
  return false;
}

std::string emptyListMessage(ListObjectFilter filter)
{
  switch (filter) {
    case ListObjectFilter::All:
      return "No objects";
    case ListObjectFilter::Selected:
      return "No selected objects";
    case ListObjectFilter::Visible:
      return "No visible objects";
    case ListObjectFilter::Hidden:
      return "No hidden objects";
  }
  return "No objects";
}

std::string formatObjectListEntry(
    const OutlinerRow& row, bool effectivelyVisible)
{
  std::string entry = std::to_string(static_cast<unsigned long long>(row.id))
                      + ":" + row.name + " [" + row.type + "]";
  if (row.selected) {
    entry += " selected";
  }
  if (!effectivelyVisible) {
    entry += " hidden";
  }
  return entry;
}

ConsoleCommandResult applyListCommand(
    const SimEngine& engine, std::span<const std::string> tokens)
{
  if (tokens.size() > 2) {
    return result(false, "Usage: list [all|selected|visible|hidden]");
  }

  ListObjectFilter filter = ListObjectFilter::All;
  if (tokens.size() == 2) {
    const std::optional<ListObjectFilter> parsed
        = parseListObjectFilter(tokens[1]);
    if (!parsed.has_value()) {
      return result(false, "Usage: list [all|selected|visible|hidden]");
    }
    filter = *parsed;
  }

  std::vector<std::string> entries;
  for (const OutlinerRow& row : buildOutlinerRows(engine)) {
    const bool effectivelyVisible = effectiveRowVisibility(engine, row.id);
    if (rowMatchesFilter(row, filter, effectivelyVisible)) {
      entries.push_back(formatObjectListEntry(row, effectivelyVisible));
    }
  }

  if (entries.empty()) {
    return result(true, emptyListMessage(filter));
  }

  std::string message = "Objects (" + std::to_string(entries.size()) + "): ";
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (i != 0) {
      message += "; ";
    }
    message += entries[i];
  }
  return result(true, std::move(message));
}

std::string scalarText(double value)
{
  std::ostringstream out;
  out << std::setprecision(6) << value;
  return out.str();
}

std::string enumChoiceText(const InspectorEnumProperty& property)
{
  const auto it = std::find_if(
      property.choices.begin(),
      property.choices.end(),
      [&property](const InspectorEnumChoice& choice) {
        return choice.value == property.value;
      });
  return it == property.choices.end() ? std::to_string(property.value)
                                      : it->label;
}

void appendInspectorField(
    std::vector<std::string>& fields, std::string label, std::string value)
{
  fields.push_back(std::move(label) + "=" + std::move(value));
}

std::string formatInspectorStatus(const InspectorStatus& status)
{
  std::string message
      = status.name + " #"
        + std::to_string(static_cast<unsigned long long>(status.object)) + " ("
        + status.type + ")";
  if (status.locked) {
    message += " [read-only]";
  }

  std::vector<std::string> fields;
  for (const InspectorEnumProperty& property : status.enumProperties) {
    appendInspectorField(fields, property.label, enumChoiceText(property));
  }
  for (const InspectorNumericProperty& property : status.numericProperties) {
    appendInspectorField(fields, property.label, scalarText(property.value));
  }
  if (status.colorProperty.has_value()) {
    const Eigen::Vector4d& color = status.colorProperty->rgba;
    appendInspectorField(
        fields,
        status.colorProperty->label,
        "(" + scalarText(color.x()) + ", " + scalarText(color.y()) + ", "
            + scalarText(color.z()) + ", " + scalarText(color.w()) + ")");
  }

  if (fields.empty()) {
    return message;
  }
  message += ": ";
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i != 0) {
      message += ", ";
    }
    message += fields[i];
  }
  return message;
}

ConsoleCommandResult applyInspectCommand(
    const SimEngine& engine, std::span<const std::string> tokens)
{
  if (tokens.size() > 2) {
    return result(false, "Usage: inspect [id|name|selected]");
  }

  const std::string targetToken = tokens.size() == 2 ? tokens[1] : "selected";
  const TargetResolution target = resolveObjectTarget(engine, targetToken);
  if (target.id == kNoObject) {
    return result(false, target.error);
  }

  const InspectorStatus status = buildInspectorObjectStatus(engine, target.id);
  if (!status.hasSelection) {
    return result(false, "Object not found: " + targetToken);
  }
  return result(true, formatInspectorStatus(status));
}

std::optional<PaletteActionKind> createActionForToken(std::string_view token)
{
  static const std::unordered_map<std::string_view, PaletteActionKind> kActions{
      {"box", PaletteActionKind::AddBoxRigidBody},
      {"sphere", PaletteActionKind::AddSphereRigidBody},
      {"cylinder", PaletteActionKind::AddCylinderRigidBody},
      {"capsule", PaletteActionKind::AddCapsuleRigidBody},
      {"plane", PaletteActionKind::AddPlaneRigidBody},
      {"ground", PaletteActionKind::AddPlaneRigidBody},
      {"multibody", PaletteActionKind::AddMultiBody},
      {"root-link", PaletteActionKind::AddRootLink},
      {"fixed-link", PaletteActionKind::AddFixedLink},
      {"revolute-link", PaletteActionKind::AddRevoluteLink},
      {"prismatic-link", PaletteActionKind::AddPrismaticLink},
      {"free-frame", PaletteActionKind::AddFreeFrame},
      {"fixed-frame", PaletteActionKind::AddFixedFrame},
      {"camera-sensor", PaletteActionKind::AddCameraSensor},
      {"range-sensor", PaletteActionKind::AddRangeSensor},
      {"contact-sensor", PaletteActionKind::AddContactSensor},
      {"collision-box", PaletteActionKind::AddBoxCollision},
      {"box-collision", PaletteActionKind::AddBoxCollision},
      {"collision-sphere", PaletteActionKind::AddSphereCollision},
      {"sphere-collision", PaletteActionKind::AddSphereCollision},
      {"collision-cylinder", PaletteActionKind::AddCylinderCollision},
      {"cylinder-collision", PaletteActionKind::AddCylinderCollision},
      {"collision-capsule", PaletteActionKind::AddCapsuleCollision},
      {"capsule-collision", PaletteActionKind::AddCapsuleCollision},
      {"collision-plane", PaletteActionKind::AddPlaneCollision},
      {"plane-collision", PaletteActionKind::AddPlaneCollision},
      {"ground-box", PaletteActionKind::AddGroundAndBoxExample},
      {"two-link-arm", PaletteActionKind::AddTwoLinkArmExample},
  };

  const auto it = kActions.find(token);
  if (it == kActions.end()) {
    return std::nullopt;
  }
  return it->second;
}

ConsoleCommandResult applyCreateCommand(
    SimEngine& engine, std::span<const std::string> tokens)
{
  if (tokens.size() != 2) {
    return result(false, "Usage: create <kind>");
  }
  const std::string kind = lower(tokens[1]);
  const std::optional<PaletteActionKind> action = createActionForToken(kind);
  if (!action.has_value()) {
    return result(false, "Unknown create kind: " + tokens[1]);
  }
  const PaletteActionResult created = applyPaletteAction(engine, *action);
  return result(created.ok, created.message);
}

ConsoleCommandResult applyProjectCommand(
    SimEngine& engine, std::span<const std::string> tokens)
{
  const std::string command = lower(tokens[0]);
  if (command == "new") {
    if (tokens.size() > 2 || (tokens.size() == 2 && tokens[1] != "--discard")) {
      return result(false, "Usage: new [--discard]");
    }
    const DirtyProjectPolicy policy = hasFlag(tokens, "--discard")
                                          ? DirtyProjectPolicy::Discard
                                          : DirtyProjectPolicy::Block;
    const ProjectActionResult applied = newProject(engine, policy);
    return result(applied.ok, applied.message);
  }

  if (command == "close") {
    if (tokens.size() > 2 || (tokens.size() == 2 && tokens[1] != "--discard")) {
      return result(false, "Usage: close [--discard]");
    }
    const DirtyProjectPolicy policy = hasFlag(tokens, "--discard")
                                          ? DirtyProjectPolicy::Discard
                                          : DirtyProjectPolicy::Block;
    const ProjectActionResult applied = closeProject(engine, policy);
    return result(applied.ok, applied.message);
  }

  if (command == "open") {
    if (tokens.size() < 2 || tokens.size() > 3
        || (tokens.size() == 3 && tokens[2] != "--discard")) {
      return result(false, "Usage: open <path> [--discard]");
    }
    const DirtyProjectPolicy policy = hasFlag(tokens, "--discard")
                                          ? DirtyProjectPolicy::Discard
                                          : DirtyProjectPolicy::Block;
    const ProjectActionResult applied = openProject(engine, tokens[1], policy);
    return result(applied.ok, applied.message);
  }

  if (tokens.size() > 2) {
    return result(false, "Usage: save [path]");
  }
  const ProjectActionResult applied
      = tokens.size() == 2
            ? saveProjectAs(engine, ensureProjectExtension(tokens[1]))
            : saveProject(engine);
  return result(applied.ok, applied.message);
}

ConsoleCommandResult applySelectionCommand(
    SimEngine& engine, std::span<const std::string> tokens)
{
  const std::string command = lower(tokens[0]);
  if (command == "clear-selection") {
    if (tokens.size() != 1) {
      return result(false, "Usage: clear-selection");
    }
    const bool wasEmpty = engine.selection().empty();
    const bool changed = clearOutlinerSelection(engine);
    return result(changed || wasEmpty, "Selection cleared");
  }

  if (tokens.size() != 2) {
    return result(false, "Usage: " + command + " <id|name|selected>");
  }
  const TargetResolution target = resolveObjectTarget(engine, tokens[1]);
  if (target.id == kNoObject) {
    return result(false, target.error);
  }

  if (command == "select") {
    const SelectionState before = engine.selection().state();
    const bool alreadySelected
        = before.primary == target.id && before.ids.size() == 1u;
    return result(
        selectOutlinerObject(engine, target.id, false) || alreadySelected,
        "Selected " + tokens[1]);
  }
  if (command == "select-add") {
    const bool alreadySelected = engine.selection().isSelected(target.id);
    return result(
        selectOutlinerObject(engine, target.id, true) || alreadySelected,
        "Selected " + tokens[1]);
  }
  if (command == "deselect") {
    const bool wasSelected = engine.selection().isSelected(target.id);
    return result(
        engine.deselect(target.id) || !wasSelected, "Deselected " + tokens[1]);
  }

  return result(false, "Unknown selection command");
}

ConsoleCommandResult applyVisibilityCommand(
    SimEngine& engine, std::span<const std::string> tokens)
{
  const std::string command = lower(tokens[0]);
  if (tokens.size() > 2) {
    return result(false, "Usage: " + command + " [id|name|selected]");
  }

  const std::string targetToken = tokens.size() == 2 ? tokens[1] : "selected";
  const TargetResolution target = resolveObjectTarget(engine, targetToken);
  if (target.id == kNoObject) {
    return result(false, target.error);
  }

  const bool visible = command == "show";
  const SceneObject* object = engine.objects().model().find(target.id);
  if (object == nullptr) {
    return result(false, target.error);
  }
  if (object->visible == visible) {
    return result(
        true, visible ? "Object already shown" : "Object already hidden");
  }
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }
  const bool ok = setOutlinerVisibility(engine, target.id, visible);
  return result(ok, visible ? "Object shown" : "Object hidden");
}

ConsoleCommandResult applyEditCommand(
    SimEngine& engine, std::span<const std::string> tokens)
{
  const std::string command = lower(tokens[0]);
  if (command == "delete") {
    if (tokens.size() != 1) {
      return result(false, "Usage: delete");
    }
    if (engine.selection().empty()) {
      return result(false, "No selection");
    }
    const InspectorActionResult applied = deleteInspectorSelection(engine);
    return result(applied.ok, applied.message);
  }

  if (tokens.size() < 2) {
    return result(false, "Usage: rename <name>");
  }
  const ObjectId selected = engine.selection().primary();
  if (selected == kNoObject) {
    return result(false, "No object selected");
  }
  const OutlinerActionResult renamed
      = renameOutlinerObject(engine, selected, joinArguments(tokens, 1));
  return result(renamed.ok, renamed.message);
}

ConsoleCommandResult applySimulationCommand(
    SimEngine& engine, std::span<const std::string> tokens)
{
  const std::string command = lower(tokens[0]);
  if (command == "play") {
    if (tokens.size() != 1) {
      return result(false, "Usage: play");
    }
    const SimulationActionResult applied = playSimulation(engine);
    return result(applied.ok, applied.message);
  }
  if (command == "pause") {
    if (tokens.size() != 1) {
      return result(false, "Usage: pause");
    }
    const SimulationActionResult applied = pauseSimulation(engine);
    return result(applied.ok, applied.message);
  }
  if (command == "reset") {
    if (tokens.size() != 1) {
      return result(false, "Usage: reset");
    }
    const SimulationActionResult applied = resetSimulation(engine);
    return result(applied.ok, applied.message);
  }
  if (command == "restart") {
    if (tokens.size() != 1) {
      return result(false, "Usage: restart");
    }
    const SimulationActionResult applied = restartSimulation(engine);
    return result(applied.ok, applied.message);
  }
  if (command == "mode") {
    if (tokens.size() != 2) {
      return result(false, "Usage: mode <edit|simulation>");
    }
    const std::string mode = lower(tokens[1]);
    if (mode == "edit") {
      const SimulationActionResult applied = resetSimulation(engine);
      return result(applied.ok, applied.message);
    }
    if (mode == "simulation" || mode == "sim") {
      const SimulationActionResult applied = playSimulation(engine);
      return result(applied.ok, applied.message);
    }
    return result(false, "Unknown mode: " + tokens[1]);
  }
  if (command == "step") {
    if (tokens.size() > 2) {
      return result(false, "Usage: step [count]");
    }
    std::size_t count = 1;
    if (tokens.size() == 2) {
      const std::optional<std::size_t> parsed = parseSize(tokens[1]);
      if (!parsed.has_value()) {
        return result(false, "Invalid step count");
      }
      count = *parsed;
    }
    if (count > kMaxConsoleStepCount) {
      return result(false, "Step count must be 1000 or less");
    }
    if (engine.simulation().isRunning()) {
      return result(false, "Pause before stepping");
    }
    const SimulationActionResult applied = stepSimulation(engine, count);
    return result(applied.ok, applied.message);
  }
  if (command == "record") {
    if (tokens.size() != 2) {
      return result(false, "Usage: record <on|off>");
    }
    const std::string value = lower(tokens[1]);
    if (value != "on" && value != "off") {
      return result(false, "Usage: record <on|off>");
    }
    const SimulationActionResult applied
        = setSimulationRecording(engine, value == "on");
    return result(applied.ok, applied.message);
  }
  if (tokens.size() != 2) {
    return result(false, "Usage: replay <first|previous|next|last|frame>");
  }
  const std::string replayTarget = lower(tokens[1]);
  if (replayTarget == "first") {
    const SimulationActionResult applied = applySimulationReplayAction(
        engine, SimulationReplayActionKind::First);
    return result(applied.ok, applied.message);
  }
  if (replayTarget == "previous" || replayTarget == "prev") {
    const SimulationActionResult applied = applySimulationReplayAction(
        engine, SimulationReplayActionKind::Previous);
    return result(applied.ok, applied.message);
  }
  if (replayTarget == "next") {
    const SimulationActionResult applied
        = applySimulationReplayAction(engine, SimulationReplayActionKind::Next);
    return result(applied.ok, applied.message);
  }
  if (replayTarget == "last") {
    const SimulationActionResult applied
        = applySimulationReplayAction(engine, SimulationReplayActionKind::Last);
    return result(applied.ok, applied.message);
  }
  const std::optional<std::size_t> frame = parseSize(tokens[1]);
  if (!frame.has_value()) {
    return result(false, "Invalid replay frame");
  }
  const SimulationActionResult applied = seekSimulationReplay(engine, *frame);
  return result(applied.ok, applied.message);
}

ConsoleCommandResult applyRelationshipCommand(
    SimEngine& engine, std::span<const std::string> tokens)
{
  const std::string command = lower(tokens[0]);
  if (tokens.size() != 1) {
    return result(false, "Usage: " + command);
  }

  RelationshipActionKind kind = RelationshipActionKind::AttachSelectedToPrimary;
  if (command == "attach") {
    kind = RelationshipActionKind::AttachSelectedToPrimary;
  } else if (command == "detach") {
    kind = RelationshipActionKind::DetachPrimaryToWorld;
  } else if (command == "reparent-link") {
    kind = RelationshipActionKind::ReparentSelectedLinkToPrimary;
  } else if (command == "make-root") {
    kind = RelationshipActionKind::MakePrimaryLinkRoot;
  } else {
    return result(false, "Unknown relationship command");
  }

  const RelationshipActionResult applied
      = applyRelationshipAction(engine, kind);
  return result(applied.ok, applied.message);
}

ConsoleCommandResult applyWatchCommand(
    SimEngine& engine, WatchState* watch, std::span<const std::string> tokens)
{
  if (watch == nullptr) {
    return result(false, "Watch state unavailable");
  }

  const std::string command = lower(tokens[0]);
  if (command == "unwatch") {
    if (tokens.size() != 2) {
      return result(false, "Usage: unwatch <id|name|selected>");
    }

    ObjectId id = kNoObject;
    if (const std::optional<ObjectId> parsed = parseObjectId(tokens[1]);
        parsed.has_value() && hasWatchedObject(*watch, *parsed)) {
      id = *parsed;
    } else {
      const TargetResolution target = resolveObjectTarget(engine, tokens[1]);
      if (target.id == kNoObject) {
        return result(false, target.error);
      }
      id = target.id;
    }

    const WatchActionResult removed = unwatchObject(*watch, id);
    return result(removed.ok, removed.message);
  }

  if (tokens.size() == 4 && lower(tokens[1]) == "signal") {
    const std::optional<WatchValueKind> kind = parseWatchSignalKind(tokens[2]);
    if (!kind.has_value()) {
      return result(false, "Unknown watch signal: " + tokens[2]);
    }
    const std::optional<bool> enabled = parseOnOff(tokens[3]);
    if (!enabled.has_value()) {
      return result(false, "Usage: watch signal <signal> <on|off>");
    }
    const WatchActionResult applied
        = setWatchChartSignalEnabled(*watch, *kind, *enabled);
    return result(applied.ok, applied.message);
  }

  if (tokens.size() == 3 && lower(tokens[1]) == "save-preset") {
    const WatchActionResult saved = saveWatchPreset(*watch, engine, tokens[2]);
    return result(saved.ok, saved.message);
  }
  if (tokens.size() == 3 && lower(tokens[1]) == "preset") {
    const WatchActionResult applied
        = applyWatchPreset(*watch, engine, tokens[2]);
    return result(applied.ok, applied.message);
  }
  if (tokens.size() == 3 && lower(tokens[1]) == "delete-preset") {
    const WatchActionResult deleted = deleteWatchPreset(engine, tokens[2]);
    return result(deleted.ok, deleted.message);
  }
  if (tokens.size() == 2) {
    const std::string subcommand = lower(tokens[1]);
    if (subcommand == "save-preset" || subcommand == "preset"
        || subcommand == "delete-preset") {
      return result(
          false,
          "Usage: watch [target|selection|clear|sample] or watch signal "
          "<signal> <on|off> or watch <save-preset|preset|delete-preset> "
          "<name>");
    }
  }

  if (tokens.size() == 1) {
    const WatchStatus status = buildWatchStatus(*watch, engine);
    return result(true, status.summary);
  }
  if (tokens.size() != 2) {
    return result(
        false,
        "Usage: watch [target|selection|clear|sample] or watch signal "
        "<signal> <on|off> or watch <save-preset|preset|delete-preset> "
        "<name>");
  }

  const std::string targetToken = lower(tokens[1]);
  if (targetToken == "selection") {
    const WatchActionResult added = watchSelectedObjects(*watch, engine);
    return result(added.ok, added.message);
  }
  if (targetToken == "clear") {
    const WatchActionResult cleared = clearWatch(*watch);
    return result(cleared.ok, cleared.message);
  }
  if (targetToken == "sample") {
    recordWatchSample(*watch, engine);
    return result(true, "Recorded watch sample");
  }

  const TargetResolution target = resolveObjectTarget(engine, tokens[1]);
  if (target.id == kNoObject) {
    return result(false, target.error);
  }
  const WatchActionResult added = watchObject(*watch, engine, target.id);
  return result(added.ok, added.message);
}

ConsoleCommandResult buildStatus(const SimEngine& engine)
{
  const SimulationStatus sim = buildSimulationStatus(engine);
  std::string message = sim.modeLabel;
  message += ", selected ";
  message += std::to_string(engine.selection().state().ids.size());
  message += engine.isProjectDirty() ? ", dirty" : ", clean";
  if (engine.hasProjectPath()) {
    message += ", ";
    message += engine.projectPath();
  }
  return result(true, std::move(message));
}

ConsoleCommandResult applyConsoleCommandWithWatchState(
    SimEngine& engine, WatchState* watch, std::string_view input);

} // namespace

ConsoleCommandResult tokenizeConsoleCommand(
    std::string_view input, std::vector<std::string>& tokens)
{
  tokens.clear();

  std::string token;
  char quote = '\0';
  bool escaping = false;
  bool tokenActive = false;
  for (const char c : input) {
    if (escaping) {
      token.push_back(c);
      escaping = false;
      tokenActive = true;
      continue;
    }

    if (c == '\\') {
      escaping = true;
      tokenActive = true;
      continue;
    }

    if (quote != '\0') {
      if (c == quote) {
        quote = '\0';
      } else {
        token.push_back(c);
        tokenActive = true;
      }
      continue;
    }

    if (c == '\'' || c == '"') {
      quote = c;
      tokenActive = true;
      continue;
    }

    if (std::isspace(static_cast<unsigned char>(c)) != 0) {
      if (tokenActive) {
        tokens.push_back(std::move(token));
        token.clear();
        tokenActive = false;
      }
      continue;
    }

    token.push_back(c);
    tokenActive = true;
  }

  if (escaping) {
    token.push_back('\\');
  }
  if (quote != '\0') {
    return result(false, "Unclosed quote");
  }
  if (tokenActive) {
    tokens.push_back(std::move(token));
  }
  return result(true, {});
}

std::string consoleCommandHelpText()
{
  return consoleCommandHelpText(false);
}

std::string consoleCommandHelpText(bool watchCommandsAvailable)
{
  return "Commands: help, status, new [--discard], close [--discard], open "
         "<path> [--discard], save [path], create <kind>, select <id|name>, "
         "select-add "
         "<id|name>, deselect <id|name>, clear-selection, rename <name>, "
         "delete, inspect [id|name|selected], list "
         "[all|selected|visible|hidden], attach, detach, reparent-link, "
         "make-root, show [target], hide [target], mode <edit|simulation>, "
         "play (enter/resume Simulation Mode), pause, step [count], restart "
         "(stay in Simulation Mode), reset (return to Edit Mode), record "
         "<on|off>, replay "
         "<first|previous|next|last|frame> "
         "(paused Simulation Mode)"
         + std::string(
             watchCommandsAvailable
                 ? ", watch [target|selection|clear|sample], watch signal "
                   "<signal> <on|off>, watch save-preset <name>, watch "
                   "preset <name>, watch delete-preset <name>, unwatch "
                   "<target>"
                 : "");
}

ConsoleCommandResult applyConsoleCommand(
    SimEngine& engine, std::string_view input)
{
  return applyConsoleCommandWithWatchState(engine, nullptr, input);
}

ConsoleCommandResult applyConsoleCommand(
    SimEngine& engine, WatchState& watch, std::string_view input)
{
  return applyConsoleCommandWithWatchState(engine, &watch, input);
}

namespace {

ConsoleCommandResult applyConsoleCommandWithWatchState(
    SimEngine& engine, WatchState* watch, std::string_view input)
{
  std::vector<std::string> tokens;
  ConsoleCommandResult parsed = tokenizeConsoleCommand(input, tokens);
  if (!parsed.ok) {
    return parsed;
  }
  if (tokens.empty()) {
    return result(false, "No command");
  }

  const std::string command = lower(tokens.front());
  tokens.front() = command;
  const std::span<const std::string> view(tokens);

  if (command == "help") {
    if (tokens.size() != 1) {
      return result(false, "Usage: help");
    }
    return result(true, consoleCommandHelpText(watch != nullptr));
  }
  if (command == "status") {
    if (tokens.size() != 1) {
      return result(false, "Usage: status");
    }
    return buildStatus(engine);
  }
  if (command == "create") {
    return applyCreateCommand(engine, view);
  }
  if (command == "inspect") {
    return applyInspectCommand(engine, view);
  }
  if (command == "list") {
    return applyListCommand(engine, view);
  }
  if (command == "new" || command == "close" || command == "open"
      || command == "save") {
    ConsoleCommandResult applied = applyProjectCommand(engine, view);
    if (watch != nullptr && applied.ok
        && (command == "new" || command == "open")) {
      clearWatch(*watch);
    }
    return applied;
  }
  if (command == "select" || command == "select-add" || command == "deselect"
      || command == "clear-selection") {
    return applySelectionCommand(engine, view);
  }
  if (command == "show" || command == "hide") {
    return applyVisibilityCommand(engine, view);
  }
  if (command == "delete" || command == "rename") {
    return applyEditCommand(engine, view);
  }
  if (command == "attach" || command == "detach" || command == "reparent-link"
      || command == "make-root") {
    return applyRelationshipCommand(engine, view);
  }
  if (command == "mode" || command == "play" || command == "pause"
      || command == "step" || command == "restart" || command == "reset"
      || command == "record" || command == "replay") {
    return applySimulationCommand(engine, view);
  }
  if (command == "watch" || command == "unwatch") {
    return applyWatchCommand(engine, watch, view);
  }

  return result(false, "Unknown command: " + command);
}

} // namespace

} // namespace dartsim::ui
