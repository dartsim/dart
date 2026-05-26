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
#include <dartsim_ui/outliner_actions.hpp>
#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

} // namespace

TEST(DartsimOutlinerActions, BuildsDepthFirstRowsFromSceneModel)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  engine.execute(commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = engine.selection().primary();
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 0.0, 0.5), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  ASSERT_TRUE(ui::selectOutlinerObject(engine, base));

  const std::vector<ui::OutlinerRow> rows = ui::buildOutlinerRows(engine);
  ASSERT_EQ(rows.size(), 4u);

  EXPECT_EQ(rows[0].id, arm);
  EXPECT_EQ(rows[0].parent, kNoObject);
  EXPECT_EQ(rows[0].depth, 0);
  EXPECT_EQ(rows[0].name, "arm");
  EXPECT_EQ(rows[0].type, "MultiBody");
  EXPECT_TRUE(rows[0].hasChildren);
  EXPECT_FALSE(rows[0].selected);

  EXPECT_EQ(rows[1].id, base);
  EXPECT_EQ(rows[1].parent, arm);
  EXPECT_EQ(rows[1].depth, 1);
  EXPECT_EQ(rows[1].name, "base");
  EXPECT_EQ(rows[1].type, "Link");
  EXPECT_TRUE(rows[1].hasChildren);
  EXPECT_TRUE(rows[1].selected);

  EXPECT_EQ(rows[2].id, forearm);
  EXPECT_EQ(rows[2].parent, base);
  EXPECT_EQ(rows[2].depth, 2);
  EXPECT_EQ(rows[2].name, "forearm");
  EXPECT_FALSE(rows[2].hasChildren);

  EXPECT_EQ(rows[3].id, box);
  EXPECT_EQ(rows[3].depth, 0);
  EXPECT_EQ(rows[3].name, "box");
  EXPECT_TRUE(rows[3].visible);
  EXPECT_EQ(
      ui::outlinerButtonLabel(rows[3]),
      "  box [RigidBody]##outliner-" + std::to_string(box));

  EXPECT_EQ(ui::objectTypeLabel(ObjectType::Joint), "Joint");
  EXPECT_EQ(ui::objectTypeLabel(ObjectType::FreeFrame), "FreeFrame");
  EXPECT_EQ(ui::objectTypeLabel(ObjectType::FixedFrame), "FixedFrame");
}

TEST(DartsimOutlinerActions, ExpandCollapseStateFiltersDescendants)
{
  SimEngine engine;
  ui::OutlinerState state;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  engine.execute(commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  ASSERT_NE(base, kNoObject);
  ASSERT_NE(forearm, kNoObject);

  std::vector<ui::OutlinerRow> rows = ui::buildOutlinerRows(engine, state);
  ASSERT_EQ(rows.size(), 3u);
  EXPECT_TRUE(rows[0].expanded);
  EXPECT_EQ(
      ui::outlinerExpansionButtonLabel(rows[0]),
      "-##outliner-collapse-" + std::to_string(arm));

  EXPECT_TRUE(ui::toggleOutlinerExpanded(state, engine, base));
  rows = ui::buildOutlinerRows(engine, state);
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[1].id, base);
  EXPECT_FALSE(rows[1].expanded);
  EXPECT_EQ(
      ui::outlinerExpansionButtonLabel(rows[1]),
      "+##outliner-expand-" + std::to_string(base));

  EXPECT_FALSE(ui::toggleOutlinerExpanded(state, engine, forearm));
  EXPECT_FALSE(ui::toggleOutlinerExpanded(state, engine, 99999));

  EXPECT_TRUE(ui::setOutlinerExpanded(state, engine, base, true));
  rows = ui::buildOutlinerRows(engine, state);
  ASSERT_EQ(rows.size(), 3u);
  EXPECT_EQ(rows[2].id, forearm);
  EXPECT_TRUE(ui::outlinerExpansionButtonLabel(rows[2]).empty());
}

TEST(DartsimOutlinerActions, SelectsExistingRowsAndRejectsMissingIds)
{
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box));
  const ObjectId first = engine.selection().primary();
  engine.execute(commands::addRigidBody(ShapeType::Sphere));
  const ObjectId second = engine.selection().primary();
  ASSERT_NE(first, kNoObject);
  ASSERT_NE(second, kNoObject);
  ASSERT_NE(first, second);

  EXPECT_TRUE(ui::selectOutlinerObject(engine, first));
  EXPECT_EQ(engine.selection().primary(), first);
  EXPECT_EQ(engine.selection().selected().size(), 1u);

  EXPECT_TRUE(ui::selectOutlinerObject(engine, second, /*additive=*/true));
  EXPECT_TRUE(engine.selection().isSelected(first));
  EXPECT_TRUE(engine.selection().isSelected(second));
  EXPECT_EQ(engine.selection().primary(), second);

  EXPECT_FALSE(ui::selectOutlinerObject(engine, 99999));
  EXPECT_EQ(engine.selection().primary(), second);
}

TEST(DartsimOutlinerActions, RenameStateCommitsThroughUndoableCommand)
{
  SimEngine engine;
  ui::OutlinerState state;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(1.0, 0.0, 1.0), "sphere"));
  ASSERT_NE(box, kNoObject);

  const auto begin = ui::beginOutlinerRename(state, engine, box);
  EXPECT_TRUE(begin.ok);
  EXPECT_EQ(state.renameTarget, box);
  EXPECT_EQ(state.renameDraft, "box");

  EXPECT_TRUE(ui::updateOutlinerRenameDraft(state, "  crate  "));
  const auto commit = ui::commitOutlinerRename(engine, state);
  EXPECT_TRUE(commit.ok);
  EXPECT_EQ(commit.message, "Renamed box to crate");
  EXPECT_EQ(state.renameTarget, kNoObject);
  ASSERT_NE(engine.objects().model().find(box), nullptr);
  EXPECT_EQ(engine.objects().model().find(box)->name, "crate");

  ASSERT_TRUE(engine.undo());
  ASSERT_NE(engine.objects().model().find(box), nullptr);
  EXPECT_EQ(engine.objects().model().find(box)->name, "box");

  ASSERT_TRUE(engine.redo());
  ASSERT_NE(engine.objects().model().find(box), nullptr);
  EXPECT_EQ(engine.objects().model().find(box)->name, "crate");

  EXPECT_TRUE(ui::beginOutlinerRename(state, engine, box).ok);
  EXPECT_TRUE(ui::updateOutlinerRenameDraft(state, "sphere"));
  const auto duplicate = ui::commitOutlinerRename(engine, state);
  EXPECT_FALSE(duplicate.ok);
  EXPECT_EQ(duplicate.message, "Rename rejected");
  EXPECT_EQ(state.renameTarget, box);
  EXPECT_EQ(engine.objects().model().find(box)->name, "crate");

  const auto cancel = ui::cancelOutlinerRename(state);
  EXPECT_TRUE(cancel.ok);
  EXPECT_EQ(state.renameTarget, kNoObject);

  engine.simulation().play();
  EXPECT_FALSE(ui::beginOutlinerRename(state, engine, box).ok);
  EXPECT_FALSE(ui::renameOutlinerObject(engine, box, "locked").ok);
}

TEST(DartsimOutlinerActions, RenameAndContextActionsRejectInvalidState)
{
  SimEngine engine;
  ui::OutlinerState state;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  EXPECT_FALSE(ui::updateOutlinerRenameDraft(state, "unused"));
  EXPECT_FALSE(ui::commitOutlinerRename(engine, state).ok);
  EXPECT_FALSE(ui::cancelOutlinerRename(state).ok);
  EXPECT_FALSE(ui::beginOutlinerRename(state, engine, 99999).ok);
  EXPECT_TRUE(ui::beginOutlinerRename(state, engine, box).ok);

  EXPECT_FALSE(ui::renameOutlinerObject(engine, box, "   ").ok);
  const auto unchanged = ui::renameOutlinerObject(engine, box, "box");
  EXPECT_FALSE(unchanged.ok);
  EXPECT_EQ(unchanged.message, "Name unchanged");
  EXPECT_FALSE(ui::renameOutlinerObject(engine, 99999, "missing").ok);

  state.collapsed.insert(box);
  state.renameTarget = box;
  state.renameDraft = "box";
  engine.execute(commands::removeObject(box));
  const auto missing = ui::applyOutlinerContextAction(
      engine, state, box, ui::OutlinerContextActionKind::Delete);
  EXPECT_FALSE(missing.ok);
  EXPECT_EQ(missing.message, "Missing object");
  EXPECT_TRUE(state.collapsed.empty());
  EXPECT_EQ(state.renameTarget, kNoObject);

  EXPECT_TRUE(ui::buildOutlinerContextActions(engine, box).empty());
}

TEST(DartsimOutlinerActions, VisibilityActionsUseUndoableEngineCommand)
{
  SimEngine engine;
  int changes = 0;
  engine.setOnChanged([&]() { ++changes; });

  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  ASSERT_EQ(engine.renderItems().size(), 1u);

  EXPECT_TRUE(ui::toggleOutlinerVisibility(engine, box));
  ASSERT_NE(engine.objects().model().find(box), nullptr);
  EXPECT_FALSE(engine.objects().model().find(box)->visible);
  EXPECT_TRUE(engine.renderItems().empty());

  const int changesBeforeNoOp = changes;
  EXPECT_FALSE(ui::setOutlinerVisibility(engine, box, false));
  EXPECT_EQ(changes, changesBeforeNoOp);

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().find(box)->visible);
  EXPECT_EQ(engine.renderItems().size(), 1u);

  ASSERT_TRUE(engine.redo());
  EXPECT_FALSE(engine.objects().model().find(box)->visible);
  EXPECT_TRUE(engine.renderItems().empty());

  EXPECT_FALSE(ui::toggleOutlinerVisibility(engine, 99999));
}

TEST(DartsimOutlinerActions, ContextVisibilityAndDeleteReportRejectedStates)
{
  SimEngine engine;
  ui::OutlinerState state;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  const auto showVisible = ui::applyOutlinerContextAction(
      engine, state, box, ui::OutlinerContextActionKind::Show);
  EXPECT_FALSE(showVisible.ok);
  EXPECT_EQ(showVisible.message, "Show rejected");

  ASSERT_TRUE(ui::setOutlinerVisibility(engine, box, false));
  const auto hideHidden = ui::applyOutlinerContextAction(
      engine, state, box, ui::OutlinerContextActionKind::Hide);
  EXPECT_FALSE(hideHidden.ok);
  EXPECT_EQ(hideHidden.message, "Hide rejected");

  ASSERT_TRUE(ui::setOutlinerVisibility(engine, box, true));
  engine.simulation().play();
  const auto lockedDelete = ui::applyOutlinerContextAction(
      engine, state, box, ui::OutlinerContextActionKind::Delete);
  EXPECT_FALSE(lockedDelete.ok);
  EXPECT_EQ(lockedDelete.message, "Scene locked");
  EXPECT_TRUE(engine.objects().model().contains(box));
}

TEST(DartsimOutlinerActions, ContextActionsDescribeAndApplySelectedRowCommands)
{
  SimEngine engine;
  ui::OutlinerState state;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  const std::vector<ui::OutlinerContextAction> actions
      = ui::buildOutlinerContextActions(engine, box);
  ASSERT_EQ(actions.size(), 3u);
  EXPECT_EQ(actions[0].kind, ui::OutlinerContextActionKind::Rename);
  EXPECT_EQ(actions[0].label, "Rename");
  EXPECT_TRUE(actions[0].enabled);
  EXPECT_EQ(actions[1].kind, ui::OutlinerContextActionKind::Delete);
  EXPECT_EQ(actions[1].label, "Delete");
  EXPECT_TRUE(actions[1].enabled);
  EXPECT_EQ(actions[2].kind, ui::OutlinerContextActionKind::Hide);
  EXPECT_EQ(actions[2].label, "Hide");
  EXPECT_TRUE(actions[2].enabled);

  const auto rename = ui::applyOutlinerContextAction(
      engine, state, box, ui::OutlinerContextActionKind::Rename);
  EXPECT_TRUE(rename.ok);
  EXPECT_EQ(state.renameTarget, box);
  EXPECT_EQ(state.renameDraft, "box");
  ASSERT_TRUE(ui::cancelOutlinerRename(state).ok);

  const auto hide = ui::applyOutlinerContextAction(
      engine, state, box, ui::OutlinerContextActionKind::Hide);
  EXPECT_TRUE(hide.ok);
  ASSERT_NE(engine.objects().model().find(box), nullptr);
  EXPECT_FALSE(engine.objects().model().find(box)->visible);

  const std::vector<ui::OutlinerContextAction> hiddenActions
      = ui::buildOutlinerContextActions(engine, box);
  ASSERT_EQ(hiddenActions.size(), 3u);
  EXPECT_EQ(hiddenActions[2].kind, ui::OutlinerContextActionKind::Show);
  EXPECT_EQ(hiddenActions[2].label, "Show");

  const auto show = ui::applyOutlinerContextAction(
      engine, state, box, ui::OutlinerContextActionKind::Show);
  EXPECT_TRUE(show.ok);
  EXPECT_TRUE(engine.objects().model().find(box)->visible);

  state.collapsed.insert(box);
  const auto deleted = ui::applyOutlinerContextAction(
      engine, state, box, ui::OutlinerContextActionKind::Delete);
  EXPECT_TRUE(deleted.ok);
  EXPECT_FALSE(engine.objects().model().contains(box));
  EXPECT_TRUE(state.collapsed.empty());
}

TEST(DartsimOutlinerActions, VisibilityActionsRejectSimulationMode)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  engine.simulation().play();
  ASSERT_FALSE(engine.canEditScene());
  const std::vector<ui::OutlinerContextAction> actions
      = ui::buildOutlinerContextActions(engine, box);
  ASSERT_EQ(actions.size(), 3u);
  EXPECT_FALSE(actions[0].enabled);
  EXPECT_FALSE(actions[1].enabled);
  EXPECT_FALSE(actions[2].enabled);

  EXPECT_FALSE(ui::setOutlinerVisibility(engine, box, false));
  ASSERT_NE(engine.objects().model().find(box), nullptr);
  EXPECT_TRUE(engine.objects().model().find(box)->visible);
  EXPECT_EQ(engine.renderItems().size(), 1u);
}

TEST(DartsimOutlinerActions, ViewportSelectionUsesVisibleRenderableIds)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(1.0, 0.0, 1.0), "sphere"));
  const ObjectId sphere = engine.selection().primary();
  ASSERT_NE(sphere, kNoObject);
  ASSERT_NE(box, sphere);

  EXPECT_EQ(ui::renderableIdForObject(kNoObject), 0u);
  EXPECT_EQ(ui::objectIdForRenderable(0), kNoObject);
  EXPECT_EQ(ui::objectIdForRenderable(ui::renderableIdForObject(box)), box);

  EXPECT_TRUE(
      ui::selectViewportRenderable(engine, ui::renderableIdForObject(box)));
  EXPECT_EQ(engine.selection().primary(), box);
  EXPECT_EQ(
      ui::selectedViewportRenderable(engine), ui::renderableIdForObject(box));
  EXPECT_EQ(ui::selectedViewportLabel(engine), "box [RigidBody]");

  EXPECT_FALSE(ui::selectViewportRenderable(engine, 99999u));
  EXPECT_EQ(engine.selection().primary(), box);

  ASSERT_TRUE(ui::setOutlinerVisibility(engine, box, false));
  EXPECT_EQ(ui::selectedViewportRenderable(engine), 0u);
  EXPECT_EQ(ui::selectedViewportLabel(engine), "none");
  EXPECT_FALSE(
      ui::selectViewportRenderable(engine, ui::renderableIdForObject(box)));
  EXPECT_EQ(engine.selection().primary(), box);

  EXPECT_TRUE(
      ui::selectViewportRenderable(engine, ui::renderableIdForObject(sphere)));
  EXPECT_EQ(engine.selection().primary(), sphere);
}

TEST(DartsimOutlinerActions, HiddenAncestorsSuppressViewportSelection)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  ASSERT_NE(base, kNoObject);
  ASSERT_TRUE(engine.objects().model().find(base) != nullptr);
  ASSERT_EQ(engine.selection().primary(), base);
  ASSERT_NE(ui::selectedViewportRenderable(engine), 0u);

  ASSERT_TRUE(ui::setOutlinerVisibility(engine, arm, false));
  EXPECT_EQ(ui::selectedViewportRenderable(engine), 0u);
  EXPECT_FALSE(
      ui::selectViewportRenderable(engine, ui::renderableIdForObject(base)));
  EXPECT_EQ(engine.selection().primary(), base);
}

TEST(DartsimOutlinerActions, MoveSelectedByMovesPrimaryMovableObject)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 2.0, 3.0)));
  const ObjectId body = engine.selection().primary();
  ASSERT_NE(body, kNoObject);

  EXPECT_FALSE(ui::moveSelectedBy(engine, Eigen::Vector3d::Zero()));
  EXPECT_TRUE(ui::moveSelectedBy(engine, Eigen::Vector3d(0.5, -1.0, 2.0)));
  ASSERT_NE(engine.objects().model().find(body), nullptr);
  EXPECT_TRUE(
      engine.objects().model().find(body)->transform.translation().isApprox(
          Eigen::Vector3d(1.5, 1.0, 5.0)));

  ASSERT_TRUE(engine.undo());
  ASSERT_NE(engine.objects().model().find(body), nullptr);
  EXPECT_TRUE(
      engine.objects().model().find(body)->transform.translation().isApprox(
          Eigen::Vector3d(1.0, 2.0, 3.0)));

  ASSERT_TRUE(engine.redo());
  ASSERT_NE(engine.objects().model().find(body), nullptr);
  EXPECT_TRUE(
      engine.objects().model().find(body)->transform.translation().isApprox(
          Eigen::Vector3d(1.5, 1.0, 5.0)));

  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  EXPECT_FALSE(ui::moveSelectedBy(engine, Eigen::Vector3d::UnitX()));
}

TEST(DartsimOutlinerActions, MoveSelectedBySupportsFramesAndRejectsRunMode)
{
  SimEngine engine;
  engine.execute(commands::addFreeFrame(translation(0.0, 1.0, 2.0), "marker"));
  const ObjectId frame = engine.selection().primary();
  ASSERT_NE(frame, kNoObject);

  EXPECT_TRUE(ui::moveSelectedBy(engine, Eigen::Vector3d(1.0, 0.0, 0.5)));
  ASSERT_NE(engine.objects().model().find(frame), nullptr);
  EXPECT_TRUE(
      engine.objects().model().find(frame)->transform.translation().isApprox(
          Eigen::Vector3d(1.0, 1.0, 2.5)));

  engine.simulation().play();
  EXPECT_FALSE(ui::moveSelectedBy(engine, Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      engine.objects().model().find(frame)->transform.translation().isApprox(
          Eigen::Vector3d(1.0, 1.0, 2.5)));
}
