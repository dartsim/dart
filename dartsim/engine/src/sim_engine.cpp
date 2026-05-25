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
  notifyChanged();
}

bool SimEngine::undo()
{
  if (!canEditScene()) {
    return false;
  }
  const bool changed = m_commands.undo();
  if (changed) {
    m_logger.info("Undo");
    m_events.emit(EventType::SceneChanged);
    notifyChanged();
  }
  return changed;
}

bool SimEngine::redo()
{
  if (!canEditScene()) {
    return false;
  }
  const bool changed = m_commands.redo();
  if (changed) {
    m_logger.info("Redo");
    m_events.emit(EventType::SceneChanged);
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

bool SimEngine::saveProject(const std::string& path) const
{
  return scene_io::saveToFile(path, m_objects.model());
}

bool SimEngine::loadProject(const std::string& path)
{
  SceneModel model;
  if (!scene_io::loadFromFile(path, model)) {
    return false;
  }
  m_objects.setModel(std::move(model));
  m_selection.clear();
  m_commands.clearHistory();
  // Drop any pre-load Run-mode snapshot so Reset targets the loaded scene.
  m_simulation.clearForNewScene();
  // Drop any prior recording/replay; seeking it would restore old-world
  // snapshots into the freshly loaded scene and corrupt it.
  m_recorder.clear();
  m_player.clear();
  m_logger.info("Loaded project: " + path);
  m_events.emit(EventType::ProjectLoaded);
  notifyChanged();
  return true;
}

std::vector<RenderItem> SimEngine::renderItems() const
{
  return m_objects.computeRenderItems();
}

} // namespace dartsim
