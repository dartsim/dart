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

#include <dartsim_engine/object_manager.hpp>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/selection_manager.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace dartsim {

/// A named, forward-only editor operation.
///
/// Reversal is provided by the CommandManager via state snapshots (memento), so
/// commands only implement the forward mutation. This avoids hand-written and
/// bug-prone inverse logic while keeping operations named for the Edit menu.
class Command
{
public:
  using Action = std::function<void(ObjectManager&, SelectionManager&)>;

  Command(std::string label, Action action)
    : m_label(std::move(label)), m_action(std::move(action))
  {
  }

  [[nodiscard]] const std::string& label() const
  {
    return m_label;
  }

  void apply(ObjectManager& objects, SelectionManager& selection) const
  {
    m_action(objects, selection);
  }

private:
  std::string m_label;
  Action m_action;
};

/// Runs commands and maintains the undo/redo history.
///
/// Each executed command is bracketed by full editor-state snapshots
/// (SceneModel + selection). Undo restores the pre-command snapshot; redo
/// restores the post-command snapshot. Restoring a snapshot rebuilds the
/// derived experimental World.
class CommandManager
{
public:
  CommandManager(ObjectManager& objects, SelectionManager& selection)
    : m_objects(objects), m_selection(selection)
  {
  }

  /// Snapshot, run the command, snapshot again, and push to the undo history.
  ///
  /// While a macro is open (see beginMacro) the command is applied but not
  /// pushed individually; the whole macro becomes one undo entry.
  void execute(std::unique_ptr<Command> command);

  /// Convenience overload constructing a Command in place.
  void execute(std::string label, Command::Action action);

  /// Begin grouping subsequent commands into a single undoable transaction.
  /// Nested macros are flattened into the outermost one.
  void beginMacro(std::string label);

  /// Close the current macro, pushing one history entry if anything changed.
  void endMacro();

  [[nodiscard]] bool inMacro() const
  {
    return m_macroDepth > 0;
  }

  bool undo();
  bool redo();

  [[nodiscard]] bool canUndo() const
  {
    return !m_undo.empty();
  }
  [[nodiscard]] bool canRedo() const
  {
    return !m_redo.empty();
  }
  [[nodiscard]] std::string undoLabel() const;
  [[nodiscard]] std::string redoLabel() const;

  /// Drop all history (e.g. after loading a fresh project).
  void clearHistory();

private:
  struct EditorState
  {
    SceneModel model;
    SelectionState selection;

    bool operator==(const EditorState&) const = default;
  };
  struct HistoryEntry
  {
    std::string label;
    EditorState before;
    EditorState after;
  };

  [[nodiscard]] EditorState capture() const;
  void restore(const EditorState& state);

  ObjectManager& m_objects;
  SelectionManager& m_selection;
  std::vector<HistoryEntry> m_undo;
  std::vector<HistoryEntry> m_redo;

  int m_macroDepth = 0;
  bool m_macroDirty = false;
  std::string m_macroLabel;
  EditorState m_macroBefore;
};

} // namespace dartsim
