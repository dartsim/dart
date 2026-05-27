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

#include <dart/simulation/experimental/world.hpp>

#include <dartsim_engine/scene_io.hpp>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/sim_engine.hpp>

#include <utility>

namespace dartsim {

SimEngine::SimEngine()
  : m_commands(m_objects, m_selection), m_simulation(m_objects)
{
  // Record each simulated step (no-op while not recording).
  m_simulation.onAfterStep = [this]() {
    m_recorder.capture(m_objects.world());
  };
  m_simulation.onModeChanged = [this]() {
    m_events.emit(EventType::ModeChanged);
  };
}

SimEngine::~SimEngine() = default;

void SimEngine::notifyChanged()
{
  if (m_onChanged) {
    m_onChanged();
  }
}

void SimEngine::execute(std::unique_ptr<Command> command)
{
  if (!canEditScene()) {
    return;
  }
  const bool wasDirty = isProjectDirty();
  const std::string oldPath = m_projectPath;
  std::string label = command ? command->label() : std::string();
  if (!m_commands.execute(std::move(command))) {
    // No-op command (e.g. a rejected edit): do not log it or signal a scene
    // change, so event-driven consumers don't run spurious refresh/save flows.
    return;
  }
  if (!label.empty()) {
    m_logger.info(std::move(label));
  }
  m_events.emit(EventType::SceneChanged);
  emitProjectStateChangedIfNeeded(wasDirty, oldPath);
  notifyChanged();
}

bool SimEngine::undo()
{
  if (!canEditScene()) {
    return false;
  }
  const bool wasDirty = isProjectDirty();
  const std::string oldPath = m_projectPath;
  const bool changed = m_commands.undo();
  if (changed) {
    m_logger.info("Undo");
    m_events.emit(EventType::SceneChanged);
    emitProjectStateChangedIfNeeded(wasDirty, oldPath);
    notifyChanged();
  }
  return changed;
}

bool SimEngine::redo()
{
  if (!canEditScene()) {
    return false;
  }
  const bool wasDirty = isProjectDirty();
  const std::string oldPath = m_projectPath;
  const bool changed = m_commands.redo();
  if (changed) {
    m_logger.info("Redo");
    m_events.emit(EventType::SceneChanged);
    emitProjectStateChangedIfNeeded(wasDirty, oldPath);
    notifyChanged();
  }
  return changed;
}

bool SimEngine::canEditScene() const
{
  return m_simulation.mode() == SimulationController::Mode::Edit;
}

bool SimEngine::select(ObjectId id, bool additive)
{
  const SelectionState before = m_selection.state();
  m_selection.select(id, additive);
  if (m_selection.state() == before) {
    return false;
  }
  m_events.emit(EventType::SelectionChanged, id);
  notifyChanged();
  return true;
}

bool SimEngine::deselect(ObjectId id)
{
  const SelectionState before = m_selection.state();
  m_selection.deselect(id);
  if (m_selection.state() == before) {
    return false;
  }
  m_events.emit(EventType::SelectionChanged, id);
  notifyChanged();
  return true;
}

bool SimEngine::toggleSelection(ObjectId id, bool additive)
{
  const SelectionState before = m_selection.state();
  m_selection.toggle(id, additive);
  if (m_selection.state() == before) {
    return false;
  }
  m_events.emit(EventType::SelectionChanged, id);
  notifyChanged();
  return true;
}

bool SimEngine::clearSelection()
{
  const SelectionState before = m_selection.state();
  m_selection.clear();
  if (m_selection.state() == before) {
    return false;
  }
  m_events.emit(EventType::SelectionChanged);
  notifyChanged();
  return true;
}

void SimEngine::startRecording()
{
  m_recorder.start(m_objects.model().timeStep);
  // Capture the starting state so replay includes frame 0.
  m_recorder.capture(m_objects.world());
  m_logger.info("Recording started");
  m_events.emit(EventType::RecordingChanged);
}

void SimEngine::stopRecording()
{
  m_recorder.stop();
  m_logger.info("Recording stopped");
  m_events.emit(EventType::RecordingChanged);
}

bool SimEngine::isRecording() const
{
  return m_recorder.isRecording();
}

void SimEngine::loadRecordingIntoPlayer()
{
  m_player.setRecording(m_recorder.recording());
}

bool SimEngine::replaySeek(std::size_t index)
{
  const bool ok = m_player.seek(m_objects.world(), index);
  if (ok) {
    notifyChanged();
  }
  return ok;
}

bool SimEngine::isProjectDirty() const
{
  return !(m_objects.model() == m_cleanProjectModel);
}

bool SimEngine::isHistoryAtCleanState() const
{
  return !isProjectDirty()
         && m_commands.currentRevision() == m_cleanHistoryRevision;
}

void SimEngine::resetRuntimeStateForProjectReplacement()
{
  const bool hadRecordingState = m_recorder.isRecording()
                                 || !m_recorder.recording().empty()
                                 || !m_player.empty();
  const bool hadSelection = !m_selection.empty();
  m_selection.clear();
  m_commands.clearHistory();
  ++m_projectGeneration;
  captureCleanProjectModel();
  if (hadSelection) {
    m_events.emit(EventType::SelectionChanged);
  }
  // Drop any pre-load Run-mode snapshot so Reset targets the replacement scene.
  m_simulation.clearForNewScene();
  // Drop any prior recording/replay; seeking it would restore old-world
  // snapshots into the freshly loaded scene and corrupt it.
  m_recorder.clear();
  m_player.clear();
  if (hadRecordingState) {
    m_events.emit(EventType::RecordingChanged);
  }
}

void SimEngine::newProject()
{
  if (m_commands.inMacro()) {
    m_logger.warning("Cannot create a new project during an edit transaction");
    return;
  }
  const bool wasDirty = isProjectDirty();
  const std::string oldPath = m_projectPath;
  m_objects.setModel(SceneModel{});
  m_projectPath.clear();
  resetRuntimeStateForProjectReplacement();
  m_logger.info("New project");
  emitProjectStateChangedIfNeeded(wasDirty, oldPath);
  m_events.emit(EventType::ProjectCreated);
  notifyChanged();
}

void SimEngine::captureCleanProjectModel()
{
  m_cleanProjectModel = m_objects.model();
  m_cleanHistoryIndex = m_commands.historyIndex();
  m_cleanHistoryRevision = m_commands.currentRevision();
}

void SimEngine::markProjectClean()
{
  const bool wasDirty = isProjectDirty();
  const std::string oldPath = m_projectPath;
  captureCleanProjectModel();
  if (wasDirty || oldPath != m_projectPath) {
    m_events.emit(EventType::ProjectStateChanged);
    notifyChanged();
  }
}

bool SimEngine::saveProject()
{
  if (m_projectPath.empty()) {
    return false;
  }
  return saveProject(m_projectPath);
}

bool SimEngine::saveProject(const std::string& path)
{
  const bool wasDirty = isProjectDirty();
  const std::string oldPath = m_projectPath;
  if (!scene_io::saveToFile(path, m_objects.model())) {
    return false;
  }
  m_projectPath = path;
  captureCleanProjectModel();
  m_logger.info("Saved project: " + path);
  emitProjectStateChangedIfNeeded(wasDirty, oldPath);
  m_events.emit(EventType::ProjectSaved);
  notifyChanged();
  return true;
}

bool SimEngine::loadProject(const std::string& path)
{
  if (m_commands.inMacro()) {
    m_logger.warning("Cannot load a project during an edit transaction");
    return false;
  }
  SceneModel model;
  if (!scene_io::loadFromFile(path, model)) {
    return false;
  }
  const bool wasDirty = isProjectDirty();
  const std::string oldPath = m_projectPath;
  m_objects.setModel(std::move(model));
  m_projectPath = path;
  resetRuntimeStateForProjectReplacement();
  m_logger.info("Loaded project: " + path);
  emitProjectStateChangedIfNeeded(wasDirty, oldPath);
  m_events.emit(EventType::ProjectLoaded);
  notifyChanged();
  return true;
}

void SimEngine::emitProjectStateChangedIfNeeded(
    bool wasDirty, const std::string& oldPath)
{
  if (wasDirty != isProjectDirty() || oldPath != m_projectPath) {
    m_events.emit(EventType::ProjectStateChanged);
  }
}

std::vector<RenderItem> SimEngine::renderItems() const
{
  return m_objects.computeRenderItems();
}

} // namespace dartsim
