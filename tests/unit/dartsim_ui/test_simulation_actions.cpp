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
#include <dartsim_engine/sim_engine.hpp>
#include <dartsim_ui/simulation_actions.hpp>
#include <gtest/gtest.h>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

} // namespace

TEST(DartsimSimulationActions, StatusTracksModePlaybackAndResetTarget)
{
  SimEngine engine;
  ui::SimulationStatus status = ui::buildSimulationStatus(engine);
  EXPECT_TRUE(status.editMode);
  EXPECT_FALSE(status.simulationMode);
  EXPECT_TRUE(status.canEditScene);
  EXPECT_FALSE(status.hasCapturedEditState);
  EXPECT_EQ(status.modeLabel, "Edit Mode");
  EXPECT_EQ(status.modeDescription, "Author the scene before simulation");
  EXPECT_EQ(status.editStateLabel, "Scene edits enabled");
  EXPECT_EQ(status.playbackLabel, "Stopped");
  EXPECT_EQ(status.resetTargetLabel, "Current Edit Mode state");

  const auto play = ui::playSimulation(engine);
  EXPECT_TRUE(play.ok);
  EXPECT_EQ(play.message, "Entered Simulation Mode");
  EXPECT_TRUE(play.status.simulationMode);
  EXPECT_TRUE(play.status.playing);
  EXPECT_FALSE(play.status.canEditScene);
  EXPECT_TRUE(play.status.hasCapturedEditState);
  EXPECT_EQ(play.status.modeLabel, "Simulation Mode");
  EXPECT_EQ(
      play.status.modeDescription, "Running from the captured Edit Mode scene");
  EXPECT_EQ(play.status.editStateLabel, "Scene edits locked");
  EXPECT_EQ(play.status.playbackLabel, "Playing");
  EXPECT_EQ(play.status.resetTargetLabel, "Captured Edit Mode state");

  const auto pause = ui::pauseSimulation(engine);
  EXPECT_TRUE(pause.ok);
  EXPECT_TRUE(pause.status.simulationMode);
  EXPECT_FALSE(pause.status.playing);
  EXPECT_EQ(pause.status.playbackLabel, "Paused");

  const auto reset = ui::resetSimulation(engine);
  EXPECT_TRUE(reset.ok);
  EXPECT_EQ(reset.message, "Returned to Edit Mode");
  EXPECT_TRUE(reset.status.editMode);
  EXPECT_FALSE(reset.status.hasCapturedEditState);
  EXPECT_EQ(reset.status.resetTargetLabel, "Current Edit Mode state");
}

TEST(DartsimSimulationActions, ModeActionsExposeEditAndSimulationWorkflow)
{
  SimEngine engine;
  std::vector<ui::SimulationModeAction> actions
      = ui::buildSimulationModeActions(engine);
  ASSERT_EQ(actions.size(), 5u);
  EXPECT_EQ(actions[0].kind, ui::SimulationModeActionKind::PlayOrResume);
  EXPECT_EQ(actions[0].label, "Enter Simulation Mode");
  EXPECT_TRUE(actions[0].enabled);
  EXPECT_EQ(actions[1].label, "Pause Simulation");
  EXPECT_FALSE(actions[1].enabled);
  EXPECT_EQ(actions[1].disabledReason, "Not in Simulation Mode");
  EXPECT_EQ(actions[2].label, "Step Simulation");
  EXPECT_TRUE(actions[2].enabled);
  EXPECT_EQ(actions[3].label, "Restart Simulation");
  EXPECT_FALSE(actions[3].enabled);
  EXPECT_EQ(actions[3].disabledReason, "Enter Simulation Mode first");
  EXPECT_EQ(actions[4].label, "Return to Edit Mode");
  EXPECT_FALSE(actions[4].enabled);
  EXPECT_EQ(actions[4].disabledReason, "Already in Edit Mode");

  const auto noReturn = ui::applySimulationModeAction(
      engine, ui::SimulationModeActionKind::ReturnToEdit);
  EXPECT_FALSE(noReturn.ok);
  EXPECT_EQ(noReturn.message, "Already in Edit Mode");

  const auto noRestart = ui::applySimulationModeAction(
      engine, ui::SimulationModeActionKind::Restart);
  EXPECT_FALSE(noRestart.ok);
  EXPECT_EQ(noRestart.message, "Enter Simulation Mode first");

  const auto enter = ui::applySimulationModeAction(
      engine, ui::SimulationModeActionKind::PlayOrResume);
  EXPECT_TRUE(enter.ok);
  EXPECT_EQ(enter.message, "Entered Simulation Mode");

  actions = ui::buildSimulationModeActions(engine);
  EXPECT_EQ(actions[0].label, "Resume Simulation");
  EXPECT_FALSE(actions[0].enabled);
  EXPECT_EQ(actions[0].disabledReason, "Simulation already playing");
  EXPECT_TRUE(actions[1].enabled);
  EXPECT_FALSE(actions[2].enabled);
  EXPECT_EQ(actions[2].disabledReason, "Pause before stepping");
  EXPECT_TRUE(actions[3].enabled);
  EXPECT_TRUE(actions[4].enabled);

  const auto alreadyPlaying = ui::playSimulation(engine);
  EXPECT_FALSE(alreadyPlaying.ok);
  EXPECT_EQ(alreadyPlaying.message, "Simulation already playing");

  const auto pause = ui::applySimulationModeAction(
      engine, ui::SimulationModeActionKind::Pause);
  EXPECT_TRUE(pause.ok);
  EXPECT_EQ(pause.message, "Simulation paused");

  actions = ui::buildSimulationModeActions(engine);
  EXPECT_EQ(actions[0].label, "Resume Simulation");
  EXPECT_TRUE(actions[0].enabled);
  EXPECT_FALSE(actions[1].enabled);
  EXPECT_EQ(actions[1].disabledReason, "Simulation already paused");
  EXPECT_TRUE(actions[2].enabled);
  EXPECT_TRUE(actions[3].enabled);
  EXPECT_TRUE(actions[4].enabled);

  const auto resume = ui::applySimulationModeAction(
      engine, ui::SimulationModeActionKind::PlayOrResume);
  EXPECT_TRUE(resume.ok);
  EXPECT_EQ(resume.message, "Simulation resumed");
  EXPECT_TRUE(resume.status.playing);
  ASSERT_TRUE(ui::pauseSimulation(engine).ok);

  const auto step = ui::applySimulationModeAction(
      engine, ui::SimulationModeActionKind::Step);
  EXPECT_TRUE(step.ok);
  EXPECT_EQ(step.message, "Simulation stepped");
  EXPECT_GT(step.status.frameCount, 0u);

  const auto restart = ui::applySimulationModeAction(
      engine, ui::SimulationModeActionKind::Restart);
  EXPECT_TRUE(restart.ok);
  EXPECT_EQ(restart.message, "Simulation restarted");
  EXPECT_TRUE(restart.status.simulationMode);
  EXPECT_FALSE(restart.status.playing);
  EXPECT_TRUE(restart.status.hasCapturedEditState);
  EXPECT_EQ(restart.status.frameCount, 0u);

  const auto edit = ui::applySimulationModeAction(
      engine, ui::SimulationModeActionKind::ReturnToEdit);
  EXPECT_TRUE(edit.ok);
  EXPECT_EQ(edit.message, "Returned to Edit Mode");
  EXPECT_TRUE(edit.status.editMode);
}

TEST(DartsimSimulationActions, StepAndRealTimeFactorActionsValidateInput)
{
  SimEngine engine;

  const auto alreadyPaused = ui::pauseSimulation(engine);
  EXPECT_FALSE(alreadyPaused.ok);
  EXPECT_EQ(alreadyPaused.message, "Simulation already paused");

  const auto noStep = ui::stepSimulation(engine, 0);
  EXPECT_FALSE(noStep.ok);
  EXPECT_EQ(noStep.message, "Step count must be positive");
  EXPECT_TRUE(noStep.status.editMode);

  const auto stepped = ui::stepSimulation(engine, 2);
  EXPECT_TRUE(stepped.ok);
  EXPECT_EQ(stepped.message, "Simulation stepped 2 frames");
  EXPECT_TRUE(stepped.status.simulationMode);
  EXPECT_FALSE(stepped.status.playing);
  EXPECT_EQ(stepped.status.playbackLabel, "Paused");
  EXPECT_EQ(stepped.status.frameCount, 2u);

  const auto singleStep = ui::stepSimulation(engine, 1);
  EXPECT_TRUE(singleStep.ok);
  EXPECT_EQ(singleStep.message, "Simulation stepped");

  const auto invalidRtf = ui::setSimulationRealTimeFactor(engine, -1.0);
  EXPECT_FALSE(invalidRtf.ok);
  EXPECT_EQ(invalidRtf.message, "Invalid real-time factor");

  const auto validRtf = ui::setSimulationRealTimeFactor(engine, 2.5);
  EXPECT_TRUE(validRtf.ok);
  EXPECT_DOUBLE_EQ(validRtf.status.realTimeFactor, 2.5);
}

TEST(DartsimSimulationActions, ResetConsumesSnapshotBeforeFurtherEdits)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const ObjectId body = engine.selection().primary();
  ASSERT_NE(body, kNoObject);

  ASSERT_TRUE(ui::playSimulation(engine).status.hasCapturedEditState);
  ASSERT_TRUE(ui::resetSimulation(engine).ok);
  ASSERT_FALSE(ui::buildSimulationStatus(engine).hasCapturedEditState);

  engine.execute(commands::setMass(body, 7.0));
  EXPECT_TRUE(engine.isProjectDirty());
  const SceneObject* edited = engine.objects().model().find(body);
  ASSERT_NE(edited, nullptr);
  EXPECT_DOUBLE_EQ(edited->mass, 7.0);

  const auto extraReset = ui::resetSimulation(engine);
  EXPECT_FALSE(extraReset.ok);
  EXPECT_EQ(extraReset.message, "Already in Edit Mode");
  const SceneObject* afterReset = engine.objects().model().find(body);
  ASSERT_NE(afterReset, nullptr);
  EXPECT_DOUBLE_EQ(afterReset->mass, 7.0);
}

TEST(DartsimSimulationActions, RecordingAndReplayActionsExposeStatus)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 2.0)));

  std::vector<ui::SimulationReplayAction> replayActions
      = ui::buildSimulationReplayActions(engine);
  ASSERT_EQ(replayActions.size(), 4u);
  EXPECT_FALSE(replayActions[0].enabled);
  EXPECT_EQ(replayActions[0].disabledReason, "No replay loaded");
  EXPECT_FALSE(replayActions[1].enabled);
  EXPECT_FALSE(replayActions[2].enabled);
  EXPECT_FALSE(replayActions[3].enabled);

  const auto stopInactive = ui::setSimulationRecording(engine, false);
  EXPECT_FALSE(stopInactive.ok);
  EXPECT_EQ(stopInactive.message, "Recording already stopped");

  const auto start = ui::setSimulationRecording(engine, true);
  EXPECT_TRUE(start.ok);
  EXPECT_TRUE(start.status.recording);
  EXPECT_FALSE(start.status.hasReplay);

  const auto startAgain = ui::setSimulationRecording(engine, true);
  EXPECT_FALSE(startAgain.ok);
  EXPECT_EQ(startAgain.message, "Recording already active");

  ASSERT_TRUE(ui::playSimulation(engine).ok);
  const auto stopWhileRunning = ui::setSimulationRecording(engine, false);
  EXPECT_FALSE(stopWhileRunning.ok);
  EXPECT_EQ(stopWhileRunning.message, "Pause before stopping recording");
  EXPECT_TRUE(stopWhileRunning.status.recording);
  EXPECT_TRUE(stopWhileRunning.status.playing);
  EXPECT_FALSE(stopWhileRunning.status.hasReplay);

  ASSERT_TRUE(ui::pauseSimulation(engine).ok);
  ASSERT_TRUE(ui::stepSimulation(engine, 2).ok);
  const auto stop = ui::setSimulationRecording(engine, false);
  EXPECT_TRUE(stop.ok);
  EXPECT_FALSE(stop.status.recording);
  EXPECT_TRUE(stop.status.hasReplay);
  EXPECT_GT(stop.status.replayFrameCount, 0u);
  EXPECT_EQ(
      stop.status.replayLabel,
      "Replay frame 0 of " + std::to_string(stop.status.replayFrameCount - 1));

  replayActions = ui::buildSimulationReplayActions(engine);
  EXPECT_EQ(replayActions[0].label, "First Replay Frame");
  EXPECT_FALSE(replayActions[0].enabled);
  EXPECT_EQ(replayActions[0].disabledReason, "Already at first replay frame");
  EXPECT_EQ(replayActions[1].label, "Previous Replay Frame");
  EXPECT_FALSE(replayActions[1].enabled);
  EXPECT_EQ(replayActions[2].label, "Next Replay Frame");
  EXPECT_TRUE(replayActions[2].enabled);
  EXPECT_EQ(replayActions[3].label, "Last Replay Frame");
  EXPECT_TRUE(replayActions[3].enabled);

  const auto seek = ui::seekSimulationReplay(engine, 0);
  EXPECT_TRUE(seek.ok);
  EXPECT_EQ(seek.status.replayFrame, 0u);

  const auto next = ui::applySimulationReplayAction(
      engine, ui::SimulationReplayActionKind::Next);
  EXPECT_TRUE(next.ok);
  EXPECT_EQ(next.message, "Replay frame 1");
  EXPECT_EQ(next.status.replayFrame, 1u);

  const auto last = ui::applySimulationReplayAction(
      engine, ui::SimulationReplayActionKind::Last);
  EXPECT_TRUE(last.ok);
  EXPECT_EQ(last.status.replayFrame, stop.status.replayFrameCount - 1);

  replayActions = ui::buildSimulationReplayActions(engine);
  EXPECT_FALSE(replayActions[2].enabled);
  EXPECT_EQ(replayActions[2].disabledReason, "Already at last replay frame");
  EXPECT_FALSE(replayActions[3].enabled);
  EXPECT_EQ(replayActions[3].disabledReason, "Already at last replay frame");

  const auto previous = ui::applySimulationReplayAction(
      engine, ui::SimulationReplayActionKind::Previous);
  EXPECT_TRUE(previous.ok);
  EXPECT_EQ(previous.status.replayFrame, stop.status.replayFrameCount - 2);

  const auto first = ui::applySimulationReplayAction(
      engine, ui::SimulationReplayActionKind::First);
  EXPECT_TRUE(first.ok);
  EXPECT_EQ(first.status.replayFrame, 0u);

  ASSERT_TRUE(ui::resetSimulation(engine).ok);
  replayActions = ui::buildSimulationReplayActions(engine);
  EXPECT_FALSE(replayActions[0].enabled);
  EXPECT_EQ(
      replayActions[0].disabledReason,
      "Enter Simulation Mode before replaying");
  EXPECT_EQ(
      ui::seekSimulationReplay(engine, 0).message,
      "Enter Simulation Mode before replaying");
  ASSERT_TRUE(ui::stepSimulation(engine, 1).ok);

  const auto invalidSeek
      = ui::seekSimulationReplay(engine, stop.status.replayFrameCount + 10);
  EXPECT_FALSE(invalidSeek.ok);
  EXPECT_EQ(invalidSeek.message, "Replay seek failed");

  ASSERT_TRUE(ui::playSimulation(engine).ok);
  const auto playingSeek = ui::seekSimulationReplay(engine, 0);
  EXPECT_FALSE(playingSeek.ok);
  EXPECT_EQ(playingSeek.message, "Pause before replaying");
  ASSERT_TRUE(ui::pauseSimulation(engine).ok);

  ASSERT_TRUE(ui::setSimulationRecording(engine, true).ok);
  const auto recordingSeek = ui::seekSimulationReplay(engine, 0);
  EXPECT_FALSE(recordingSeek.ok);
  EXPECT_EQ(recordingSeek.message, "Stop recording before replaying");
}
