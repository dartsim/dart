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

#include <dartsim_engine/sim_engine.hpp>

#include <string>
#include <vector>

#include <cstddef>

namespace dartsim::ui {

/// Snapshot consumed by the Simulation panel. Edit Mode is for authoring;
/// Simulation Mode is runtime playback/stepping over a captured edit-state
/// snapshot.
struct SimulationStatus
{
  bool editMode = true;
  bool simulationMode = false;
  bool playing = false;
  bool canEditScene = true;
  bool hasCapturedEditState = false;
  bool recording = false;
  bool hasReplay = false;
  double simTime = 0.0;
  double realTimeFactor = 1.0;
  std::size_t frameCount = 0;
  std::size_t replayFrameCount = 0;
  std::size_t replayFrame = 0;
  std::string modeLabel;
  std::string modeDescription;
  std::string editStateLabel;
  std::string playbackLabel;
  std::string resetTargetLabel;
};

/// Result of a user-facing Simulation panel action.
struct SimulationActionResult
{
  bool ok = false;
  std::string message;
  SimulationStatus status;
};

enum class SimulationModeActionKind
{
  PlayOrResume,
  Pause,
  Step,
  ReturnToEdit,
};

struct SimulationModeAction
{
  SimulationModeActionKind kind = SimulationModeActionKind::PlayOrResume;
  std::string label;
  bool enabled = true;
  std::string disabledReason;
};

/// Build panel-facing status from the engine's authoritative state.
[[nodiscard]] SimulationStatus buildSimulationStatus(const SimEngine& engine);

[[nodiscard]] std::vector<SimulationModeAction> buildSimulationModeActions(
    const SimEngine& engine);
SimulationActionResult applySimulationModeAction(
    SimEngine& engine, SimulationModeActionKind kind);

SimulationActionResult playSimulation(SimEngine& engine);
SimulationActionResult pauseSimulation(SimEngine& engine);
SimulationActionResult stepSimulation(SimEngine& engine, std::size_t count = 1);
SimulationActionResult resetSimulation(SimEngine& engine);
SimulationActionResult setSimulationRealTimeFactor(
    SimEngine& engine, double factor);

SimulationActionResult setSimulationRecording(
    SimEngine& engine, bool recording);
SimulationActionResult seekSimulationReplay(
    SimEngine& engine, std::size_t frame);

} // namespace dartsim::ui
