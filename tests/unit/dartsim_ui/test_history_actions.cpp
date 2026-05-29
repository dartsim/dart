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
#include <dartsim_ui/history_actions.hpp>
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

TEST(DartsimHistoryActions, StatusExposesLabelsCountsAndCleanState)
{
  SimEngine engine;
  ui::HistoryStatus status = ui::buildHistoryStatus(engine);
  EXPECT_TRUE(status.canEditScene);
  EXPECT_FALSE(status.canUndo);
  EXPECT_FALSE(status.canRedo);
  EXPECT_FALSE(status.dirty);
  EXPECT_TRUE(status.atCleanHistoryState);
  EXPECT_EQ(status.undoCount, 0u);
  EXPECT_EQ(status.redoCount, 0u);
  EXPECT_EQ(status.historyIndex, 0u);
  EXPECT_EQ(status.cleanHistoryIndex, 0u);
  EXPECT_EQ(status.historyRevision, 0u);
  EXPECT_EQ(status.cleanHistoryRevision, 0u);
  EXPECT_EQ(status.undoMenuLabel, "Undo");
  EXPECT_EQ(status.redoMenuLabel, "Redo");
  EXPECT_EQ(status.dirtyLabel, "Saved");
  EXPECT_EQ(status.historyLabel, "0 undo / 0 redo");

  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  status = ui::buildHistoryStatus(engine);
  EXPECT_TRUE(status.canUndo);
  EXPECT_FALSE(status.canRedo);
  EXPECT_TRUE(status.dirty);
  EXPECT_FALSE(status.atCleanHistoryState);
  EXPECT_EQ(status.undoCount, 1u);
  EXPECT_EQ(status.redoCount, 0u);
  EXPECT_EQ(status.historyIndex, 1u);
  EXPECT_EQ(status.cleanHistoryIndex, 0u);
  EXPECT_EQ(status.undoLabel, "Add Rigid Body");
  EXPECT_EQ(status.undoMenuLabel, "Undo Add Rigid Body");
  EXPECT_EQ(status.dirtyLabel, "Unsaved changes");
  EXPECT_EQ(status.historyLabel, "1 undo / 0 redo");

  engine.markProjectClean();
  status = ui::buildHistoryStatus(engine);
  EXPECT_FALSE(status.dirty);
  EXPECT_TRUE(status.atCleanHistoryState);
  EXPECT_EQ(status.cleanHistoryIndex, 1u);
  EXPECT_EQ(status.cleanHistoryRevision, status.historyRevision);
}

TEST(DartsimHistoryActions, UndoRedoActionsPreserveRedoAndReportLabels)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  engine.markProjectClean();
  const CommandManager::HistoryRevision cleanRevision
      = engine.cleanHistoryRevision();

  std::vector<ui::HistoryAction> actions = ui::buildHistoryActions(engine);
  ASSERT_EQ(actions.size(), 2u);
  EXPECT_EQ(actions[0].kind, ui::HistoryActionKind::Undo);
  EXPECT_EQ(actions[0].label, "Undo Add Rigid Body");
  EXPECT_TRUE(actions[0].enabled);
  EXPECT_EQ(actions[1].kind, ui::HistoryActionKind::Redo);
  EXPECT_EQ(actions[1].label, "Redo");
  EXPECT_FALSE(actions[1].enabled);
  EXPECT_EQ(actions[1].disabledReason, "Nothing to redo");

  const auto undo = ui::applyHistoryAction(engine, ui::HistoryActionKind::Undo);
  EXPECT_TRUE(undo.ok);
  EXPECT_EQ(undo.message, "Undo Add Rigid Body");
  EXPECT_EQ(engine.objects().model().size(), 0u);
  EXPECT_TRUE(undo.status.canRedo);
  EXPECT_EQ(undo.status.redoLabel, "Add Rigid Body");
  EXPECT_EQ(undo.status.redoMenuLabel, "Redo Add Rigid Body");
  EXPECT_EQ(undo.status.historyIndex, 0u);
  EXPECT_NE(undo.status.historyRevision, cleanRevision);
  EXPECT_TRUE(engine.isProjectDirty());

  actions = ui::buildHistoryActions(engine);
  ASSERT_EQ(actions.size(), 2u);
  EXPECT_FALSE(actions[0].enabled);
  EXPECT_EQ(actions[0].disabledReason, "Nothing to undo");
  EXPECT_TRUE(actions[1].enabled);
  EXPECT_EQ(actions[1].label, "Redo Add Rigid Body");

  const auto redo = ui::applyHistoryAction(engine, ui::HistoryActionKind::Redo);
  EXPECT_TRUE(redo.ok);
  EXPECT_EQ(redo.message, "Redo Add Rigid Body");
  EXPECT_EQ(engine.objects().model().size(), 1u);
  EXPECT_FALSE(redo.status.canRedo);
  EXPECT_EQ(redo.status.historyIndex, 1u);
  EXPECT_EQ(redo.status.historyRevision, cleanRevision);
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimHistoryActions, DisabledActionsExplainSimulationModeLock)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  engine.simulation().play();

  const ui::HistoryStatus status = ui::buildHistoryStatus(engine);
  EXPECT_FALSE(status.canEditScene);
  EXPECT_TRUE(status.canUndo);

  const std::vector<ui::HistoryAction> actions
      = ui::buildHistoryActions(engine);
  ASSERT_EQ(actions.size(), 2u);
  EXPECT_FALSE(actions[0].enabled);
  EXPECT_EQ(actions[0].disabledReason, "Scene edits locked in Simulation Mode");
  EXPECT_FALSE(actions[1].enabled);
  EXPECT_EQ(actions[1].disabledReason, "Scene edits locked in Simulation Mode");

  const auto undo = ui::applyHistoryAction(engine, ui::HistoryActionKind::Undo);
  EXPECT_FALSE(undo.ok);
  EXPECT_EQ(undo.message, "Scene edits locked in Simulation Mode");
  EXPECT_EQ(engine.objects().model().size(), 1u);
}
