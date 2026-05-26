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
#include <dartsim_ui/simulation_actions.hpp>

#include <algorithm>
#include <charconv>
#include <filesystem>
#include <optional>
#include <span>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>

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
    return result(false, "Usage: replay <frame>");
  }
  const std::optional<std::size_t> frame = parseSize(tokens[1]);
  if (!frame.has_value()) {
    return result(false, "Invalid replay frame");
  }
  const SimulationActionResult applied = seekSimulationReplay(engine, *frame);
  return result(applied.ok, applied.message);
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
  return "Commands: help, status, new [--discard], open <path> [--discard], "
         "save [path], create <kind>, select <id|name>, select-add "
         "<id|name>, deselect <id|name>, clear-selection, rename <name>, "
         "delete, show [target], hide [target], mode <edit|simulation>, play, "
         "pause, step [count], reset, record <on|off>, replay <frame>";
}

ConsoleCommandResult applyConsoleCommand(
    SimEngine& engine, std::string_view input)
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
    return result(true, consoleCommandHelpText());
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
  if (command == "new" || command == "open" || command == "save") {
    return applyProjectCommand(engine, view);
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
  if (command == "mode" || command == "play" || command == "pause"
      || command == "step" || command == "reset" || command == "record"
      || command == "replay") {
    return applySimulationCommand(engine, view);
  }

  return result(false, "Unknown command: " + command);
}

} // namespace dartsim::ui
