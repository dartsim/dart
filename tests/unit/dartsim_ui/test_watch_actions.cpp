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
#include <dartsim_ui/watch_actions.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <cstdint>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

const ui::WatchValue* findValue(
    const ui::WatchRow& row, ui::WatchValueKind kind)
{
  const auto it = std::find_if(
      row.values.begin(),
      row.values.end(),
      [kind](const ui::WatchValue& value) { return value.kind == kind; });
  return it == row.values.end() ? nullptr : &*it;
}

const ui::WatchSeries* findSeries(
    const ui::WatchState& state,
    ObjectId object,
    ui::WatchValueKind kind,
    std::uint64_t projectGeneration = 0)
{
  const auto it = std::find_if(
      state.series.begin(),
      state.series.end(),
      [object, kind, projectGeneration](const ui::WatchSeries& series) {
        return series.object == object && series.kind == kind
               && (projectGeneration == 0
                   || series.projectGeneration == projectGeneration);
      });
  return it == state.series.end() ? nullptr : &*it;
}

const ui::WatchSignalOption* findSignal(
    const ui::WatchStatus& status, ui::WatchValueKind kind)
{
  const auto it = std::find_if(
      status.signalOptions.begin(),
      status.signalOptions.end(),
      [kind](const ui::WatchSignalOption& option) {
        return option.kind == kind;
      });
  return it == status.signalOptions.end() ? nullptr : &*it;
}

const ui::WatchPresetOption* findPreset(
    const ui::WatchStatus& status, const std::string& name)
{
  const auto it = std::find_if(
      status.presetOptions.begin(),
      status.presetOptions.end(),
      [&name](const ui::WatchPresetOption& option) {
        return option.name == name;
      });
  return it == status.presetOptions.end() ? nullptr : &*it;
}

} // namespace

TEST(DartsimWatchActions, EmptyWatchListReportsStatusAndSamplesGlobals)
{
  SimEngine engine;
  engine.markProjectClean();
  ui::WatchState state;

  const ui::WatchStatus empty = ui::buildWatchStatus(state, engine);
  EXPECT_EQ(empty.summary, "No watched objects");
  ASSERT_NE(findSignal(empty, ui::WatchValueKind::SimulationTime), nullptr);
  EXPECT_TRUE(findSignal(empty, ui::WatchValueKind::SimulationTime)->enabled);
  ASSERT_NE(findSignal(empty, ui::WatchValueKind::TranslationX), nullptr);
  EXPECT_FALSE(findSignal(empty, ui::WatchValueKind::TranslationX)->enabled);
  EXPECT_TRUE(empty.rows.empty());
  EXPECT_TRUE(empty.series.empty());

  const auto noSelection = ui::watchSelectedObjects(state, engine);
  EXPECT_FALSE(noSelection.ok);
  EXPECT_EQ(noSelection.message, "No selection to watch");

  const auto missing = ui::watchObject(state, engine, 999);
  EXPECT_FALSE(missing.ok);
  EXPECT_EQ(missing.message, "Missing object");

  ui::recordWatchSample(state, engine);
  const ui::WatchSeries* time
      = findSeries(state, kNoObject, ui::WatchValueKind::SimulationTime);
  const ui::WatchSeries* frame
      = findSeries(state, kNoObject, ui::WatchValueKind::FrameCount);
  ASSERT_NE(time, nullptr);
  ASSERT_NE(frame, nullptr);
  ASSERT_EQ(time->samples.size(), 1u);
  ASSERT_EQ(frame->samples.size(), 1u);
  EXPECT_DOUBLE_EQ(time->samples[0], 0.0);
  EXPECT_DOUBLE_EQ(frame->samples[0], 0.0);
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimWatchActions, ChartSignalTogglesControlSamples)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.markProjectClean();
  const std::size_t undoCount = engine.commands().undoCount();
  const auto revision = engine.commands().currentRevision();

  ui::WatchState state;
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);
  ui::recordWatchSample(state, engine);
  ASSERT_NE(findSeries(state, box, ui::WatchValueKind::TranslationZ), nullptr);
  EXPECT_EQ(findSeries(state, box, ui::WatchValueKind::TranslationX), nullptr);
  EXPECT_EQ(findSeries(state, box, ui::WatchValueKind::Mass), nullptr);

  const auto disabledZ = ui::setWatchChartSignalEnabled(
      state, ui::WatchValueKind::TranslationZ, false);
  EXPECT_TRUE(disabledZ.ok);
  EXPECT_EQ(disabledZ.message, "Disabled watch signal: z");
  EXPECT_EQ(findSeries(state, box, ui::WatchValueKind::TranslationZ), nullptr);

  const auto duplicateDisable = ui::setWatchChartSignalEnabled(
      state, ui::WatchValueKind::TranslationZ, false);
  EXPECT_FALSE(duplicateDisable.ok);
  EXPECT_EQ(duplicateDisable.message, "Watch signal already disabled: z");

  EXPECT_TRUE(
      ui::setWatchChartSignalEnabled(
          state, ui::WatchValueKind::TranslationX, true)
          .ok);
  EXPECT_TRUE(
      ui::setWatchChartSignalEnabled(state, ui::WatchValueKind::Mass, true).ok);
  ui::recordWatchSample(state, engine);

  const ui::WatchSeries* x
      = findSeries(state, box, ui::WatchValueKind::TranslationX);
  const ui::WatchSeries* mass
      = findSeries(state, box, ui::WatchValueKind::Mass);
  ASSERT_NE(x, nullptr);
  ASSERT_NE(mass, nullptr);
  ASSERT_EQ(x->samples.size(), 1u);
  ASSERT_EQ(mass->samples.size(), 1u);
  EXPECT_DOUBLE_EQ(x->samples[0], 1.0);
  EXPECT_DOUBLE_EQ(mass->samples[0], 1.0);
  EXPECT_EQ(findSeries(state, box, ui::WatchValueKind::TranslationZ), nullptr);

  const ui::WatchStatus status = ui::buildWatchStatus(state, engine);
  ASSERT_NE(findSignal(status, ui::WatchValueKind::TranslationX), nullptr);
  EXPECT_TRUE(findSignal(status, ui::WatchValueKind::TranslationX)->enabled);
  ASSERT_NE(findSignal(status, ui::WatchValueKind::TranslationZ), nullptr);
  EXPECT_FALSE(findSignal(status, ui::WatchValueKind::TranslationZ)->enabled);
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_EQ(engine.commands().undoCount(), undoCount);
  EXPECT_EQ(engine.commands().currentRevision(), revision);

  const auto disabledFrame = ui::setWatchChartSignalEnabled(
      state, ui::WatchValueKind::FrameCount, false);
  EXPECT_TRUE(disabledFrame.ok);
  EXPECT_EQ(disabledFrame.message, "Disabled watch signal: Frame");
  EXPECT_EQ(
      findSeries(state, kNoObject, ui::WatchValueKind::FrameCount), nullptr);
  ui::recordWatchSample(state, engine);
  EXPECT_EQ(
      findSeries(state, kNoObject, ui::WatchValueKind::FrameCount), nullptr);

  const auto enabledFrame = ui::setWatchChartSignalEnabled(
      state, ui::WatchValueKind::FrameCount, true);
  EXPECT_TRUE(enabledFrame.ok);
  EXPECT_EQ(enabledFrame.message, "Enabled watch signal: Frame");
  ui::recordWatchSample(state, engine);
  const ui::WatchSeries* frame
      = findSeries(state, kNoObject, ui::WatchValueKind::FrameCount);
  ASSERT_NE(frame, nullptr);
  ASSERT_EQ(frame->samples.size(), 1u);
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_EQ(engine.commands().undoCount(), undoCount);
  EXPECT_EQ(engine.commands().currentRevision(), revision);
}

TEST(DartsimWatchActions, SavedWatchPresetsPersistAndRestoreSessionState)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.markProjectClean();
  const std::size_t undoCount = engine.commands().undoCount();
  const auto revision = engine.commands().currentRevision();

  ui::WatchState state;
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);
  ASSERT_TRUE(
      ui::setWatchChartSignalEnabled(
          state, ui::WatchValueKind::TranslationX, true)
          .ok);
  ASSERT_TRUE(
      ui::setWatchChartSignalEnabled(
          state, ui::WatchValueKind::SensorRange, true)
          .ok);

  const auto saved = ui::saveWatchPreset(state, engine, " motion ");
  EXPECT_TRUE(saved.ok);
  EXPECT_EQ(saved.message, "Saved watch preset: motion");
  EXPECT_TRUE(engine.isProjectDirty());
  EXPECT_EQ(engine.commands().undoCount(), undoCount + 1);
  EXPECT_NE(engine.commands().currentRevision(), revision);

  const ui::WatchStatus withPreset = ui::buildWatchStatus(state, engine);
  const ui::WatchPresetOption* preset = findPreset(withPreset, "motion");
  ASSERT_NE(preset, nullptr);
  EXPECT_EQ(preset->targetCount, 1u);
  EXPECT_EQ(preset->missingTargetCount, 0u);
  EXPECT_EQ(preset->signalCount, 6u);
  EXPECT_EQ(preset->ignoredSignalCount, 0u);

  const auto duplicate = ui::saveWatchPreset(state, engine, "motion");
  EXPECT_FALSE(duplicate.ok);
  EXPECT_EQ(duplicate.message, "Watch preset already saved: motion");
  EXPECT_EQ(engine.commands().undoCount(), undoCount + 1);

  ASSERT_TRUE(ui::clearWatch(state).ok);
  ASSERT_TRUE(
      ui::setWatchChartSignalEnabled(
          state, ui::WatchValueKind::TranslationX, false)
          .ok);
  ASSERT_FALSE(
      findSignal(
          ui::buildWatchStatus(state, engine), ui::WatchValueKind::TranslationX)
          ->enabled);

  const bool dirtyBeforeApply = engine.isProjectDirty();
  const std::size_t undoCountBeforeApply = engine.commands().undoCount();
  const auto revisionBeforeApply = engine.commands().currentRevision();
  const auto* worldBeforeApply = &engine.objects().world();
  const auto applied = ui::applyWatchPreset(state, engine, "motion");
  EXPECT_TRUE(applied.ok);
  EXPECT_EQ(applied.message, "Applied watch preset: motion");
  EXPECT_EQ(engine.isProjectDirty(), dirtyBeforeApply);
  EXPECT_EQ(engine.commands().undoCount(), undoCountBeforeApply);
  EXPECT_EQ(engine.commands().currentRevision(), revisionBeforeApply);
  EXPECT_EQ(&engine.objects().world(), worldBeforeApply);
  ASSERT_EQ(state.targets.size(), 1u);
  EXPECT_EQ(state.targets[0].id, box);
  EXPECT_EQ(state.targets[0].projectGeneration, engine.projectGeneration());
  EXPECT_TRUE(state.series.empty());
  const ui::WatchStatus restored = ui::buildWatchStatus(state, engine);
  ASSERT_NE(findSignal(restored, ui::WatchValueKind::TranslationX), nullptr);
  EXPECT_TRUE(findSignal(restored, ui::WatchValueKind::TranslationX)->enabled);
  ASSERT_NE(findSignal(restored, ui::WatchValueKind::SensorRange), nullptr);
  EXPECT_TRUE(findSignal(restored, ui::WatchValueKind::SensorRange)->enabled);

  const std::size_t undoCountBeforeDelete = engine.commands().undoCount();
  const auto revisionBeforeDelete = engine.commands().currentRevision();
  const auto* worldBeforeDelete = &engine.objects().world();
  const auto deleted = ui::deleteWatchPreset(engine, "motion");
  EXPECT_TRUE(deleted.ok);
  EXPECT_EQ(deleted.message, "Deleted watch preset: motion");
  EXPECT_EQ(engine.commands().undoCount(), undoCountBeforeDelete + 1);
  EXPECT_NE(engine.commands().currentRevision(), revisionBeforeDelete);
  EXPECT_EQ(&engine.objects().world(), worldBeforeDelete);
  EXPECT_EQ(findPreset(ui::buildWatchStatus(state, engine), "motion"), nullptr);

  const auto* worldBeforeUndoDelete = &engine.objects().world();
  ASSERT_TRUE(engine.undo());
  EXPECT_EQ(&engine.objects().world(), worldBeforeUndoDelete);
  EXPECT_NE(findPreset(ui::buildWatchStatus(state, engine), "motion"), nullptr);
}

TEST(DartsimWatchActions, WatchPresetSkipsMissingTargetsAndLocksEditsInRunMode)
{
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  WorkspaceWatchPreset preset;
  preset.name = "stale";
  preset.targets = {box, 999};
  preset.chartSignals = {"translation_z", "unknown_signal", "translation_z"};
  engine.execute(commands::setWorkspaceWatchPresets({preset}));
  engine.markProjectClean();

  ui::WatchState state;
  const auto applied = ui::applyWatchPreset(state, engine, "stale");
  EXPECT_TRUE(applied.ok);
  EXPECT_EQ(
      applied.message,
      "Applied watch preset: stale (1 missing target skipped)");
  ASSERT_EQ(state.targets.size(), 1u);
  EXPECT_EQ(state.targets[0].id, box);
  const ui::WatchStatus status = ui::buildWatchStatus(state, engine);
  const ui::WatchPresetOption* option = findPreset(status, "stale");
  ASSERT_NE(option, nullptr);
  EXPECT_EQ(option->targetCount, 1u);
  EXPECT_EQ(option->missingTargetCount, 1u);
  EXPECT_EQ(option->signalCount, 1u);
  EXPECT_EQ(option->ignoredSignalCount, 1u);
  EXPECT_TRUE(findSignal(status, ui::WatchValueKind::TranslationZ)->enabled);
  EXPECT_FALSE(findSignal(status, ui::WatchValueKind::TranslationX)->enabled);

  ASSERT_TRUE(ui::playSimulation(engine).ok);
  EXPECT_FALSE(engine.canEditScene());
  const auto lockedSave = ui::saveWatchPreset(state, engine, "run-mode");
  EXPECT_FALSE(lockedSave.ok);
  EXPECT_EQ(lockedSave.message, "Watch presets locked during Simulation Mode");
  const auto lockedDelete = ui::deleteWatchPreset(engine, "stale");
  EXPECT_FALSE(lockedDelete.ok);
  EXPECT_EQ(
      lockedDelete.message, "Watch presets locked during Simulation Mode");
}

TEST(DartsimWatchActions, SelectedObjectsBuildRowsAndDeduplicate)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.markProjectClean();

  ui::WatchState state;
  const auto added = ui::watchSelectedObjects(state, engine);
  EXPECT_TRUE(added.ok);
  EXPECT_EQ(added.message, "Watching selection");
  ASSERT_EQ(state.targets.size(), 1u);
  EXPECT_EQ(state.targets[0].id, box);
  EXPECT_EQ(state.targets[0].projectGeneration, engine.projectGeneration());
  EXPECT_FALSE(engine.isProjectDirty());

  const auto duplicate = ui::watchSelectedObjects(state, engine);
  EXPECT_FALSE(duplicate.ok);
  EXPECT_EQ(duplicate.message, "Selection already watched");
  EXPECT_EQ(state.targets.size(), 1u);

  const ui::WatchStatus status = ui::buildWatchStatus(state, engine);
  EXPECT_EQ(status.summary, "1 watched object");
  ASSERT_EQ(status.rows.size(), 1u);
  EXPECT_EQ(status.rows[0].id, box);
  EXPECT_EQ(status.rows[0].name, "box");
  EXPECT_EQ(status.rows[0].type, "RigidBody");
  EXPECT_TRUE(status.rows[0].exists);
  EXPECT_TRUE(status.rows[0].selected);
  ASSERT_NE(
      findValue(status.rows[0], ui::WatchValueKind::TranslationX), nullptr);
  ASSERT_NE(
      findValue(status.rows[0], ui::WatchValueKind::TranslationY), nullptr);
  ASSERT_NE(
      findValue(status.rows[0], ui::WatchValueKind::TranslationZ), nullptr);
  ASSERT_NE(findValue(status.rows[0], ui::WatchValueKind::Mass), nullptr);
  EXPECT_DOUBLE_EQ(
      findValue(status.rows[0], ui::WatchValueKind::TranslationX)->value, 1.0);
  EXPECT_DOUBLE_EQ(
      findValue(status.rows[0], ui::WatchValueKind::TranslationY)->value, 2.0);
  EXPECT_DOUBLE_EQ(
      findValue(status.rows[0], ui::WatchValueKind::TranslationZ)->value, 3.0);
  EXPECT_DOUBLE_EQ(
      findValue(status.rows[0], ui::WatchValueKind::Mass)->value, 1.0);
}

TEST(DartsimWatchActions, SamplesBoundedObjectChartsAndSimulationSignals)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(0.0, 0.0, 1.0), "ball"));
  const ObjectId ball = engine.selection().primary();
  ASSERT_NE(ball, kNoObject);

  ui::WatchState state;
  state.historyCapacity = 2;
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);

  ui::recordWatchSample(state, engine);
  engine.execute(commands::setTransform(ball, translation(0.0, 0.0, 2.0)));
  ui::recordWatchSample(state, engine);
  engine.execute(commands::setTransform(ball, translation(0.0, 0.0, 4.0)));
  ASSERT_TRUE(ui::stepSimulation(engine, 3).ok);
  ui::recordWatchSample(state, engine);

  const ui::WatchSeries* height
      = findSeries(state, ball, ui::WatchValueKind::TranslationZ);
  const ui::WatchSeries* frame
      = findSeries(state, kNoObject, ui::WatchValueKind::FrameCount);
  const ui::WatchSeries* time
      = findSeries(state, kNoObject, ui::WatchValueKind::SimulationTime);
  ASSERT_NE(height, nullptr);
  ASSERT_NE(frame, nullptr);
  ASSERT_NE(time, nullptr);
  ASSERT_EQ(height->samples.size(), 2u);
  ASSERT_EQ(frame->samples.size(), 2u);
  ASSERT_EQ(time->samples.size(), 2u);
  EXPECT_DOUBLE_EQ(height->samples[0], 2.0);
  EXPECT_NEAR(height->samples[1], 4.0, 1e-3);
  EXPECT_DOUBLE_EQ(frame->samples[0], 0.0);
  EXPECT_DOUBLE_EQ(frame->samples[1], 3.0);
  EXPECT_GT(time->samples[1], time->samples[0]);

  const ui::WatchStatus status = ui::buildWatchStatus(state, engine);
  EXPECT_EQ(status.series.size(), state.series.size());
}

TEST(DartsimWatchActions, LinkWatchExposesJointPosition)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  ASSERT_NE(base, kNoObject);
  engine.execute(commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = engine.selection().primary();
  ASSERT_NE(forearm, kNoObject);
  engine.execute(commands::setJointPosition(forearm, 0.5));

  ui::WatchState state;
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);
  ui::recordWatchSample(state, engine);

  const ui::WatchStatus status = ui::buildWatchStatus(state, engine);
  ASSERT_EQ(status.rows.size(), 1u);
  const ui::WatchValue* jointPosition
      = findValue(status.rows[0], ui::WatchValueKind::JointPosition);
  ASSERT_NE(jointPosition, nullptr);
  EXPECT_DOUBLE_EQ(jointPosition->value, 0.5);

  const ui::WatchSeries* jointSeries
      = findSeries(state, forearm, ui::WatchValueKind::JointPosition);
  ASSERT_NE(jointSeries, nullptr);
  ASSERT_EQ(jointSeries->samples.size(), 1u);
  EXPECT_DOUBLE_EQ(jointSeries->samples[0], 0.5);
}

TEST(DartsimWatchActions, SensorWatchExposesDescriptorValues)
{
  SimEngine engine;
  engine.execute(commands::addSensor(SensorKind::Range, kNoObject));
  const ObjectId sensor = engine.selection().primary();
  ASSERT_NE(sensor, kNoObject);

  SensorDesc descriptor;
  descriptor.kind = SensorKind::Range;
  descriptor.range = 12.0;
  descriptor.fieldOfView = 45.0;
  descriptor.updateRate = 20.0;
  engine.execute(commands::setSensor(sensor, descriptor));

  ui::WatchState state;
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);
  ASSERT_TRUE(
      ui::setWatchChartSignalEnabled(
          state, ui::WatchValueKind::SensorRange, true)
          .ok);
  ui::recordWatchSample(state, engine);

  const ui::WatchStatus status = ui::buildWatchStatus(state, engine);
  ASSERT_EQ(status.rows.size(), 1u);
  EXPECT_EQ(status.rows[0].type, "Sensor");
  ASSERT_NE(
      findValue(status.rows[0], ui::WatchValueKind::TranslationX), nullptr);
  ASSERT_NE(
      findValue(status.rows[0], ui::WatchValueKind::SensorRange), nullptr);
  ASSERT_NE(
      findValue(status.rows[0], ui::WatchValueKind::SensorFieldOfView),
      nullptr);
  ASSERT_NE(
      findValue(status.rows[0], ui::WatchValueKind::SensorUpdateRate), nullptr);
  EXPECT_DOUBLE_EQ(
      findValue(status.rows[0], ui::WatchValueKind::SensorRange)->value, 12.0);
  EXPECT_DOUBLE_EQ(
      findValue(status.rows[0], ui::WatchValueKind::SensorFieldOfView)->value,
      45.0);
  EXPECT_DOUBLE_EQ(
      findValue(status.rows[0], ui::WatchValueKind::SensorUpdateRate)->value,
      20.0);

  const ui::WatchSeries* rangeSeries
      = findSeries(state, sensor, ui::WatchValueKind::SensorRange);
  ASSERT_NE(rangeSeries, nullptr);
  EXPECT_DOUBLE_EQ(rangeSeries->samples.back(), 12.0);
  ASSERT_NE(findSignal(status, ui::WatchValueKind::SensorRange), nullptr);
  EXPECT_TRUE(findSignal(status, ui::WatchValueKind::SensorRange)->enabled);
}

TEST(DartsimWatchActions, RemoveAndClearWatchState)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, Eigen::Isometry3d::Identity(), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  ui::WatchState state;
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);
  ui::recordWatchSample(state, engine);
  ASSERT_FALSE(state.series.empty());

  const auto removed = ui::unwatchObject(state, box);
  EXPECT_TRUE(removed.ok);
  EXPECT_EQ(removed.message, "Removed watch");
  EXPECT_TRUE(state.targets.empty());
  EXPECT_EQ(findSeries(state, box, ui::WatchValueKind::TranslationZ), nullptr);

  const auto missing = ui::unwatchObject(state, box);
  EXPECT_FALSE(missing.ok);
  EXPECT_EQ(missing.message, "Object was not watched");

  ui::recordWatchSample(state, engine);
  ASSERT_FALSE(state.series.empty());
  const auto cleared = ui::clearWatch(state);
  EXPECT_TRUE(cleared.ok);
  EXPECT_TRUE(state.targets.empty());
  EXPECT_TRUE(state.series.empty());

  const auto clearAgain = ui::clearWatch(state);
  EXPECT_FALSE(clearAgain.ok);
  EXPECT_EQ(clearAgain.message, "Watch list already empty");
}

TEST(DartsimWatchActions, MissingWatchedObjectStaysVisibleUntilRemoved)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, Eigen::Isometry3d::Identity(), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  ui::WatchState state;
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);
  ui::recordWatchSample(state, engine);
  const ui::WatchSeries* beforeRemove
      = findSeries(state, box, ui::WatchValueKind::TranslationZ);
  ASSERT_NE(beforeRemove, nullptr);
  ASSERT_EQ(beforeRemove->samples.size(), 1u);

  engine.execute(commands::removeObject(box));
  const ui::WatchStatus missingStatus = ui::buildWatchStatus(state, engine);
  ASSERT_EQ(missingStatus.rows.size(), 1u);
  EXPECT_FALSE(missingStatus.rows[0].exists);
  EXPECT_EQ(missingStatus.rows[0].name, "missing #" + std::to_string(box));
  EXPECT_EQ(missingStatus.rows[0].type, "Missing");
  EXPECT_TRUE(missingStatus.rows[0].values.empty());

  ui::recordWatchSample(state, engine);
  const ui::WatchSeries* afterRemove
      = findSeries(state, box, ui::WatchValueKind::TranslationZ);
  ASSERT_NE(afterRemove, nullptr);
  EXPECT_EQ(afterRemove->samples.size(), 1u);
}

TEST(DartsimWatchActions, ProjectGenerationPreventsIdReuseFromBinding)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, Eigen::Isometry3d::Identity(), "old"));
  const ObjectId oldBox = engine.selection().primary();
  ASSERT_NE(oldBox, kNoObject);
  const std::uint64_t oldGeneration = engine.projectGeneration();

  ui::WatchState state;
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);
  ui::recordWatchSample(state, engine);
  const ui::WatchSeries* oldSeries = findSeries(
      state, oldBox, ui::WatchValueKind::TranslationZ, oldGeneration);
  ASSERT_NE(oldSeries, nullptr);
  ASSERT_EQ(oldSeries->samples.size(), 1u);

  engine.newProject();
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(0.0, 0.0, 9.0), "new"));
  const ObjectId newSphere = engine.selection().primary();
  ASSERT_EQ(newSphere, oldBox);
  ASSERT_NE(engine.projectGeneration(), oldGeneration);

  const ui::WatchStatus staleStatus = ui::buildWatchStatus(state, engine);
  ASSERT_EQ(staleStatus.rows.size(), 1u);
  EXPECT_FALSE(staleStatus.rows[0].exists);
  EXPECT_EQ(staleStatus.rows[0].name, "missing #" + std::to_string(oldBox));

  ui::recordWatchSample(state, engine);
  oldSeries = findSeries(
      state, oldBox, ui::WatchValueKind::TranslationZ, oldGeneration);
  ASSERT_NE(oldSeries, nullptr);
  EXPECT_EQ(oldSeries->samples.size(), 1u);
  EXPECT_EQ(
      findSeries(
          state,
          newSphere,
          ui::WatchValueKind::TranslationZ,
          engine.projectGeneration()),
      nullptr);
}

TEST(DartsimWatchActions, ProjectEventsClearSessionWatchState)
{
  SimEngine engine;
  ui::WatchState state;
  engine.events().subscribe(
      [&state](const Event& event) { ui::handleWatchEvent(state, event); });

  engine.execute(commands::addRigidBody(ShapeType::Box));
  ASSERT_TRUE(ui::watchSelectedObjects(state, engine).ok);
  ui::recordWatchSample(state, engine);
  ASSERT_FALSE(state.targets.empty());
  ASSERT_FALSE(state.series.empty());

  engine.newProject();
  EXPECT_TRUE(state.targets.empty());
  EXPECT_TRUE(state.series.empty());
}
