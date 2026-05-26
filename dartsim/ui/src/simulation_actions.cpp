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

#include <dartsim_ui/simulation_actions.hpp>

#include <optional>
#include <string>

#include <cmath>

namespace dartsim::ui {

namespace {

SimulationActionResult result(
    bool ok, std::string message, const SimEngine& engine)
{
  return {ok, std::move(message), buildSimulationStatus(engine)};
}

SimulationModeAction makeModeAction(
    SimulationModeActionKind kind,
    std::string label,
    bool enabled = true,
    std::string disabledReason = {})
{
  SimulationModeAction action;
  action.kind = kind;
  action.label = std::move(label);
  action.enabled = enabled;
  action.disabledReason = std::move(disabledReason);
  return action;
}

std::optional<SimulationModeAction> findModeAction(
    const std::vector<SimulationModeAction>& actions,
    SimulationModeActionKind kind)
{
  for (const SimulationModeAction& action : actions) {
    if (action.kind == kind) {
      return action;
    }
  }
  return std::nullopt;
}

} // namespace

SimulationStatus buildSimulationStatus(const SimEngine& engine)
{
  const SimulationController& sim = engine.simulation();
  SimulationStatus status;
  status.editMode = sim.mode() == SimulationController::Mode::Edit;
  status.simulationMode = sim.mode() == SimulationController::Mode::Simulation;
  status.playing = sim.isRunning();
  status.canEditScene = engine.canEditScene();
  status.hasCapturedEditState = sim.hasCapturedEditState();
  status.recording = engine.isRecording();
  status.hasReplay = !engine.player().empty();
  status.simTime = sim.simTime();
  status.realTimeFactor = sim.realTimeFactor();
  status.frameCount = sim.frameCount();
  status.replayFrameCount = engine.player().frameCount();
  status.replayFrame = engine.player().currentIndex();
  status.modeLabel = status.simulationMode ? "Simulation Mode" : "Edit Mode";
  status.modeDescription = status.simulationMode
                               ? "Running from the captured Edit Mode scene"
                               : "Author the scene before simulation";
  status.editStateLabel
      = status.canEditScene ? "Scene edits enabled" : "Scene edits locked";
  status.playbackLabel
      = status.editMode ? "Stopped" : (status.playing ? "Playing" : "Paused");
  status.resetTargetLabel = status.hasCapturedEditState
                                ? "Captured Edit Mode state"
                                : "Current Edit Mode state";
  return status;
}

std::vector<SimulationModeAction> buildSimulationModeActions(
    const SimEngine& engine)
{
  const SimulationStatus status = buildSimulationStatus(engine);
  std::vector<SimulationModeAction> actions;
  actions.reserve(4);

  actions.push_back(makeModeAction(
      SimulationModeActionKind::PlayOrResume,
      status.simulationMode ? "Resume Simulation" : "Enter Simulation Mode",
      !status.playing,
      "Simulation already playing"));
  actions.push_back(makeModeAction(
      SimulationModeActionKind::Pause,
      "Pause Simulation",
      status.simulationMode && status.playing,
      status.simulationMode ? "Simulation already paused"
                            : "Not in Simulation Mode"));
  actions.push_back(makeModeAction(
      SimulationModeActionKind::Step,
      "Step Simulation",
      !status.playing,
      "Pause before stepping"));
  actions.push_back(makeModeAction(
      SimulationModeActionKind::ReturnToEdit,
      "Return to Edit Mode",
      status.simulationMode || status.hasCapturedEditState
          || status.frameCount > 0,
      "Already in Edit Mode"));
  return actions;
}

SimulationActionResult applySimulationModeAction(
    SimEngine& engine, SimulationModeActionKind kind)
{
  const std::vector<SimulationModeAction> actions
      = buildSimulationModeActions(engine);
  const std::optional<SimulationModeAction> action
      = findModeAction(actions, kind);
  if (!action.has_value()) {
    return result(false, "Unknown simulation action", engine);
  }
  if (!action->enabled) {
    return result(false, action->disabledReason, engine);
  }

  switch (kind) {
    case SimulationModeActionKind::PlayOrResume:
      return playSimulation(engine);
    case SimulationModeActionKind::Pause:
      return pauseSimulation(engine);
    case SimulationModeActionKind::Step:
      return stepSimulation(engine);
    case SimulationModeActionKind::ReturnToEdit:
      return resetSimulation(engine);
  }
  return result(false, "Unknown simulation action", engine);
}

SimulationActionResult playSimulation(SimEngine& engine)
{
  engine.simulation().play();
  return result(true, "Simulation playing", engine);
}

SimulationActionResult pauseSimulation(SimEngine& engine)
{
  if (!engine.simulation().isRunning()) {
    return result(false, "Simulation already paused", engine);
  }
  engine.simulation().pause();
  return result(true, "Simulation paused", engine);
}

SimulationActionResult stepSimulation(SimEngine& engine, std::size_t count)
{
  if (count == 0) {
    return result(false, "Step count must be positive", engine);
  }
  engine.simulation().step(count);
  if (count == 1) {
    return result(true, "Simulation stepped", engine);
  }
  return result(
      true, "Simulation stepped " + std::to_string(count) + " frames", engine);
}

SimulationActionResult resetSimulation(SimEngine& engine)
{
  const SimulationStatus before = buildSimulationStatus(engine);
  if (before.editMode && !before.hasCapturedEditState
      && before.frameCount == 0) {
    return result(false, "Already in Edit Mode", engine);
  }
  engine.simulation().reset();
  return result(true, "Returned to Edit Mode", engine);
}

SimulationActionResult setSimulationRealTimeFactor(
    SimEngine& engine, double factor)
{
  if (!std::isfinite(factor) || factor < 0.0) {
    return result(false, "Invalid real-time factor", engine);
  }
  engine.simulation().setRealTimeFactor(factor);
  return result(true, "Updated real-time factor", engine);
}

SimulationActionResult setSimulationRecording(SimEngine& engine, bool recording)
{
  if (recording) {
    if (engine.isRecording()) {
      return result(false, "Recording already active", engine);
    }
    engine.startRecording();
    return result(true, "Recording started", engine);
  }

  if (!engine.isRecording()) {
    return result(false, "Recording already stopped", engine);
  }
  engine.stopRecording();
  engine.loadRecordingIntoPlayer();
  return result(true, "Recording stopped", engine);
}

SimulationActionResult seekSimulationReplay(
    SimEngine& engine, std::size_t frame)
{
  if (!engine.replaySeek(frame)) {
    return result(false, "Replay seek failed", engine);
  }
  return result(true, "Replay frame " + std::to_string(frame), engine);
}

} // namespace dartsim::ui
