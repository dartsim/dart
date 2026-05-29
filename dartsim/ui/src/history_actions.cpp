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

#include <dartsim_ui/history_actions.hpp>

#include <optional>
#include <string>
#include <utility>

namespace dartsim::ui {

namespace {

std::string menuLabel(std::string verb, const std::string& commandLabel)
{
  if (commandLabel.empty()) {
    return verb;
  }
  return std::move(verb) + " " + commandLabel;
}

HistoryAction makeAction(
    HistoryActionKind kind,
    std::string label,
    bool enabled,
    std::string disabledReason)
{
  HistoryAction action;
  action.kind = kind;
  action.label = std::move(label);
  action.enabled = enabled;
  action.disabledReason = std::move(disabledReason);
  return action;
}

std::optional<HistoryAction> findAction(
    const std::vector<HistoryAction>& actions, HistoryActionKind kind)
{
  for (const HistoryAction& action : actions) {
    if (action.kind == kind) {
      return action;
    }
  }
  return std::nullopt;
}

HistoryActionResult result(
    bool ok, std::string message, const SimEngine& engine)
{
  return {ok, std::move(message), buildHistoryStatus(engine)};
}

} // namespace

HistoryStatus buildHistoryStatus(const SimEngine& engine)
{
  const CommandManager& commands = engine.commands();
  HistoryStatus status;
  status.canEditScene = engine.canEditScene();
  status.canUndo = commands.canUndo();
  status.canRedo = commands.canRedo();
  status.dirty = engine.isProjectDirty();
  status.atCleanHistoryState = engine.isHistoryAtCleanState();
  status.undoCount = commands.undoCount();
  status.redoCount = commands.redoCount();
  status.historyIndex = commands.historyIndex();
  status.cleanHistoryIndex = engine.cleanHistoryIndex();
  status.historyRevision = commands.currentRevision();
  status.cleanHistoryRevision = engine.cleanHistoryRevision();
  status.undoLabel = commands.undoLabel();
  status.redoLabel = commands.redoLabel();
  status.undoMenuLabel = menuLabel("Undo", status.undoLabel);
  status.redoMenuLabel = menuLabel("Redo", status.redoLabel);
  status.dirtyLabel = status.dirty ? "Unsaved changes" : "Saved";
  status.historyLabel = std::to_string(status.undoCount) + " undo / "
                        + std::to_string(status.redoCount) + " redo";
  return status;
}

std::vector<HistoryAction> buildHistoryActions(const SimEngine& engine)
{
  const HistoryStatus status = buildHistoryStatus(engine);
  const std::string lockedReason = "Scene edits locked in Simulation Mode";
  std::vector<HistoryAction> actions;
  actions.reserve(2);
  actions.push_back(makeAction(
      HistoryActionKind::Undo,
      status.undoMenuLabel,
      status.canEditScene && status.canUndo,
      status.canEditScene ? "Nothing to undo" : lockedReason));
  actions.push_back(makeAction(
      HistoryActionKind::Redo,
      status.redoMenuLabel,
      status.canEditScene && status.canRedo,
      status.canEditScene ? "Nothing to redo" : lockedReason));
  return actions;
}

HistoryActionResult applyHistoryAction(
    SimEngine& engine, HistoryActionKind kind)
{
  const std::vector<HistoryAction> actions = buildHistoryActions(engine);
  const std::optional<HistoryAction> action = findAction(actions, kind);
  if (!action.has_value()) {
    return result(false, "Unknown history action", engine);
  }
  if (!action->enabled) {
    return result(false, action->disabledReason, engine);
  }

  switch (kind) {
    case HistoryActionKind::Undo:
      return result(engine.undo(), action->label, engine);
    case HistoryActionKind::Redo:
      return result(engine.redo(), action->label, engine);
  }
  return result(false, "Unknown history action", engine);
}

} // namespace dartsim::ui
