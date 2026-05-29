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

#include <dartsim_engine/command.hpp>

#include <utility>

namespace dartsim {

CommandManager::EditorState CommandManager::capture() const
{
  return EditorState{m_objects.model(), m_selection.state()};
}

void CommandManager::restore(const EditorState& state)
{
  m_objects.restoreModelSnapshot(state.model);
  m_selection.setState(state.selection);
}

bool CommandManager::execute(std::unique_ptr<Command> command)
{
  if (!command) {
    return false;
  }
  if (m_macroDepth > 0) {
    // Inside a macro: apply now; the macro pushes one combined history entry.
    command->apply(m_objects, m_selection);
    m_macroDirty = true;
    return true;
  }
  EditorState before = capture();
  const HistoryRevision beforeRevision = m_currentRevision;
  command->apply(m_objects, m_selection);
  EditorState after = capture();
  if (after == before) {
    // The command changed nothing (e.g. an early return on a name collision or
    // an invalid id). Recording it would add a no-op undo step and discard the
    // redo branch, which is unexpected data loss in editor workflows.
    return false;
  }
  const HistoryRevision afterRevision = m_nextRevision++;
  m_currentRevision = afterRevision;
  m_undo.push_back(
      HistoryEntry{
          command->label(),
          std::move(before),
          std::move(after),
          beforeRevision,
          afterRevision});
  m_redo.clear();
  return true;
}

void CommandManager::beginMacro(std::string label)
{
  if (m_macroDepth == 0) {
    m_macroBefore = capture();
    m_macroBeforeRevision = m_currentRevision;
    m_macroDirty = false;
    m_macroLabel = std::move(label);
  }
  ++m_macroDepth;
}

void CommandManager::endMacro()
{
  if (m_macroDepth == 0) {
    return;
  }
  --m_macroDepth;
  if (m_macroDepth > 0) {
    return; // close only the outermost macro
  }
  if (m_macroDirty) {
    EditorState after = capture();
    if (after != m_macroBefore) {
      const HistoryRevision afterRevision = m_nextRevision++;
      m_currentRevision = afterRevision;
      m_undo.push_back(
          HistoryEntry{
              std::move(m_macroLabel),
              std::move(m_macroBefore),
              std::move(after),
              m_macroBeforeRevision,
              afterRevision});
      m_redo.clear();
    }
  }
  m_macroDirty = false;
}

bool CommandManager::execute(std::string label, Command::Action action)
{
  return execute(
      std::make_unique<Command>(std::move(label), std::move(action)));
}

bool CommandManager::undo()
{
  if (m_undo.empty()) {
    return false;
  }
  HistoryEntry entry = std::move(m_undo.back());
  m_undo.pop_back();
  restore(entry.before);
  m_currentRevision = entry.beforeRevision;
  m_redo.push_back(std::move(entry));
  return true;
}

bool CommandManager::redo()
{
  if (m_redo.empty()) {
    return false;
  }
  HistoryEntry entry = std::move(m_redo.back());
  m_redo.pop_back();
  restore(entry.after);
  m_currentRevision = entry.afterRevision;
  m_undo.push_back(std::move(entry));
  return true;
}

std::string CommandManager::undoLabel() const
{
  return m_undo.empty() ? std::string() : m_undo.back().label;
}

std::string CommandManager::redoLabel() const
{
  return m_redo.empty() ? std::string() : m_redo.back().label;
}

void CommandManager::clearHistory()
{
  m_undo.clear();
  m_redo.clear();
  m_currentRevision = 0;
  m_nextRevision = 1;
  m_macroDepth = 0;
  m_macroDirty = false;
  m_macroLabel.clear();
  m_macroBefore = EditorState{};
  m_macroBeforeRevision = 0;
}

} // namespace dartsim
