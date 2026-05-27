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
  EXPECT_EQ(play.message, "Simulation playing");
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
  EXPECT_EQ(enter.message, "Simulation playing");

  actions = ui::buildSimulationModeActions(engine);
  EXPECT_EQ(actions[0].label, "Resume Simulation");
  EXPECT_FALSE(actions[0].enabled);
  EXPECT_EQ(actions[0].disabledReason, "Simulation already playing");
  EXPECT_TRUE(actions[1].enabled);
  EXPECT_FALSE(actions[2].enabled);
  EXPECT_EQ(actions[2].disabledReason, "Pause before stepping");
  EXPECT_TRUE(actions[3].enabled);
  EXPECT_TRUE(actions[4].enabled);

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

  ASSERT_TRUE(ui::stepSimulation(engine, 2).ok);
  const auto stop = ui::setSimulationRecording(engine, false);
  EXPECT_TRUE(stop.ok);
  EXPECT_FALSE(stop.status.recording);
  EXPECT_TRUE(stop.status.hasReplay);
  EXPECT_GT(stop.status.replayFrameCount, 0u);

  const auto seek = ui::seekSimulationReplay(engine, 0);
  EXPECT_TRUE(seek.ok);
  EXPECT_EQ(seek.status.replayFrame, 0u);

  const auto invalidSeek
      = ui::seekSimulationReplay(engine, stop.status.replayFrameCount + 10);
  EXPECT_FALSE(invalidSeek.ok);
  EXPECT_EQ(invalidSeek.message, "Replay seek failed");
}
