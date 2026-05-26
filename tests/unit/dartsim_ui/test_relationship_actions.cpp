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
#include <dartsim_ui/relationship_actions.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

const ui::RelationshipAction* findAction(
    const std::vector<ui::RelationshipAction>& actions,
    ui::RelationshipActionKind kind)
{
  const auto it = std::find_if(
      actions.begin(),
      actions.end(),
      [kind](const ui::RelationshipAction& action) {
        return action.kind == kind;
      });
  return it == actions.end() ? nullptr : &*it;
}

} // namespace

TEST(DartsimRelationshipActions, DescribesAttachAndDetachAvailability)
{
  SimEngine engine;
  engine.execute(commands::addFreeFrame(translation(0.0, 0.0, 0.0), "child"));
  const ObjectId child = engine.selection().primary();
  engine.execute(commands::addFreeFrame(translation(2.0, 0.0, 0.0), "parent"));
  const ObjectId parent = engine.selection().primary();
  ASSERT_NE(child, kNoObject);
  ASSERT_NE(parent, kNoObject);

  std::vector<ui::RelationshipAction> actions
      = ui::buildRelationshipActions(engine);
  ASSERT_EQ(actions.size(), 4u);
  const ui::RelationshipAction* attach = findAction(
      actions, ui::RelationshipActionKind::AttachSelectedToPrimary);
  const ui::RelationshipAction* detach
      = findAction(actions, ui::RelationshipActionKind::DetachPrimaryToWorld);
  ASSERT_NE(attach, nullptr);
  ASSERT_NE(detach, nullptr);
  EXPECT_EQ(attach->label, "Attach Selected to Primary");
  EXPECT_FALSE(attach->enabled);
  EXPECT_EQ(attach->disabledReason, "Select one frame and one parent");
  EXPECT_EQ(detach->label, "Detach Primary to World");
  EXPECT_FALSE(detach->enabled);

  ASSERT_TRUE(engine.select(child));
  ASSERT_TRUE(engine.select(parent, /*additive=*/true));
  actions = ui::buildRelationshipActions(engine);
  ASSERT_EQ(actions.size(), 4u);
  EXPECT_TRUE(
      findAction(actions, ui::RelationshipActionKind::AttachSelectedToPrimary)
          ->enabled);
  EXPECT_EQ(
      findAction(actions, ui::RelationshipActionKind::AttachSelectedToPrimary)
          ->label,
      "Attach child to parent");
  EXPECT_FALSE(
      findAction(actions, ui::RelationshipActionKind::DetachPrimaryToWorld)
          ->enabled);

  engine.simulation().play();
  actions = ui::buildRelationshipActions(engine);
  ASSERT_EQ(actions.size(), 4u);
  for (const ui::RelationshipAction& action : actions) {
    EXPECT_FALSE(action.enabled);
    EXPECT_EQ(action.disabledReason, "Simulation Mode");
  }
}

TEST(DartsimRelationshipActions, AttachSelectedFrameToPrimaryParent)
{
  SimEngine engine;
  engine.execute(commands::addFreeFrame(translation(5.0, 0.0, 0.0), "child"));
  const ObjectId child = engine.selection().primary();
  engine.execute(commands::addFreeFrame(translation(2.0, 0.0, 0.0), "parent"));
  const ObjectId parent = engine.selection().primary();
  ASSERT_NE(child, kNoObject);
  ASSERT_NE(parent, kNoObject);

  ASSERT_TRUE(engine.select(child));
  ASSERT_TRUE(engine.select(parent, /*additive=*/true));
  const auto attached = ui::applyRelationshipAction(
      engine, ui::RelationshipActionKind::AttachSelectedToPrimary);
  EXPECT_TRUE(attached.ok);
  EXPECT_EQ(attached.object, child);
  EXPECT_EQ(attached.message, "Attached child to parent");
  ASSERT_NE(engine.objects().model().find(child), nullptr);
  EXPECT_EQ(engine.objects().model().find(child)->parent, parent);
  EXPECT_TRUE(
      engine.objects().model().find(child)->transform.translation().isApprox(
          Eigen::Vector3d(3.0, 0.0, 0.0)));

  std::vector<ui::RelationshipAction> actions
      = ui::buildRelationshipActions(engine);
  ASSERT_EQ(actions.size(), 4u);
  EXPECT_FALSE(
      findAction(actions, ui::RelationshipActionKind::AttachSelectedToPrimary)
          ->enabled);
  EXPECT_TRUE(
      findAction(actions, ui::RelationshipActionKind::DetachPrimaryToWorld)
          ->enabled);
  EXPECT_EQ(
      findAction(actions, ui::RelationshipActionKind::DetachPrimaryToWorld)
          ->label,
      "Detach child to World");

  ASSERT_TRUE(engine.undo());
  ASSERT_NE(engine.objects().model().find(child), nullptr);
  EXPECT_EQ(engine.objects().model().find(child)->parent, kNoObject);
}

TEST(DartsimRelationshipActions, DetachPrimaryFrameToWorld)
{
  SimEngine engine;
  engine.execute(commands::addFreeFrame(translation(1.0, 0.0, 0.0), "parent"));
  const ObjectId parent = engine.selection().primary();
  engine.execute(
      commands::addFixedFrame(parent, translation(0.0, 0.0, 2.0), "tool"));
  const ObjectId tool = engine.selection().primary();
  ASSERT_NE(parent, kNoObject);
  ASSERT_NE(tool, kNoObject);

  const auto detached = ui::applyRelationshipAction(
      engine, ui::RelationshipActionKind::DetachPrimaryToWorld);
  EXPECT_TRUE(detached.ok);
  EXPECT_EQ(detached.object, tool);
  EXPECT_EQ(detached.message, "Detached tool");
  ASSERT_NE(engine.objects().model().find(tool), nullptr);
  EXPECT_EQ(engine.objects().model().find(tool)->type, ObjectType::FreeFrame);
  EXPECT_EQ(engine.objects().model().find(tool)->parent, kNoObject);
  EXPECT_TRUE(
      engine.objects().model().find(tool)->transform.translation().isApprox(
          Eigen::Vector3d(1.0, 0.0, 2.0)));

  ASSERT_TRUE(engine.undo());
  ASSERT_NE(engine.objects().model().find(tool), nullptr);
  EXPECT_EQ(engine.objects().model().find(tool)->type, ObjectType::FixedFrame);
  EXPECT_EQ(engine.objects().model().find(tool)->parent, parent);
}

TEST(DartsimRelationshipActions, RejectsInvalidSelectionsAndSimulationMode)
{
  SimEngine engine;
  engine.execute(commands::addFreeFrame(translation(0.0, 0.0, 0.0), "frame"));
  const ObjectId frame = engine.selection().primary();
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 0.0, 0.0), "body"));
  const ObjectId body = engine.selection().primary();
  ASSERT_NE(frame, kNoObject);
  ASSERT_NE(body, kNoObject);

  engine.select(body);
  ASSERT_TRUE(engine.select(frame, /*additive=*/true));
  const auto rejectedAttach = ui::applyRelationshipAction(
      engine, ui::RelationshipActionKind::AttachSelectedToPrimary);
  EXPECT_FALSE(rejectedAttach.ok);
  EXPECT_EQ(rejectedAttach.message, "Select one frame and one parent");
  EXPECT_EQ(engine.objects().model().find(body)->parent, kNoObject);

  const auto rejectedDetach = ui::applyRelationshipAction(
      engine, ui::RelationshipActionKind::DetachPrimaryToWorld);
  EXPECT_FALSE(rejectedDetach.ok);
  EXPECT_EQ(rejectedDetach.message, "Select an attached frame");

  engine.simulation().play();
  const auto locked = ui::applyRelationshipAction(
      engine, ui::RelationshipActionKind::AttachSelectedToPrimary);
  EXPECT_FALSE(locked.ok);
  EXPECT_EQ(locked.message, "Scene locked");
}

TEST(DartsimRelationshipActions, ReparentsSelectedLinkToPrimaryParent)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  engine.execute(commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = engine.selection().primary();
  engine.execute(commands::addLink(arm, forearm, JointKind::Revolute, "tool"));
  const ObjectId tool = engine.selection().primary();
  ASSERT_NE(base, kNoObject);
  ASSERT_NE(forearm, kNoObject);
  ASSERT_NE(tool, kNoObject);

  ASSERT_EQ(engine.selection().primary(), tool);
  ASSERT_TRUE(engine.select(base, /*additive=*/true));
  std::vector<ui::RelationshipAction> actions
      = ui::buildRelationshipActions(engine);
  ASSERT_NE(
      findAction(
          actions, ui::RelationshipActionKind::ReparentSelectedLinkToPrimary),
      nullptr);
  EXPECT_TRUE(
      findAction(
          actions, ui::RelationshipActionKind::ReparentSelectedLinkToPrimary)
          ->enabled);
  EXPECT_EQ(
      findAction(
          actions, ui::RelationshipActionKind::ReparentSelectedLinkToPrimary)
          ->label,
      "Reparent tool to base");

  const auto reparented = ui::applyRelationshipAction(
      engine, ui::RelationshipActionKind::ReparentSelectedLinkToPrimary);
  EXPECT_TRUE(reparented.ok);
  EXPECT_EQ(reparented.object, tool);
  EXPECT_EQ(reparented.message, "Reparented tool to base");
  ASSERT_NE(engine.objects().model().find(tool), nullptr);
  EXPECT_EQ(engine.objects().model().find(tool)->parentLink, base);
  EXPECT_EQ(engine.objects().model().find(tool)->parent, base);

  ASSERT_TRUE(engine.undo());
  ASSERT_NE(engine.objects().model().find(tool), nullptr);
  EXPECT_EQ(engine.objects().model().find(tool)->parentLink, forearm);
}

TEST(DartsimRelationshipActions, MakesPrimaryLinkRoot)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  engine.execute(commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  ASSERT_NE(base, kNoObject);
  ASSERT_NE(forearm, kNoObject);

  std::vector<ui::RelationshipAction> actions
      = ui::buildRelationshipActions(engine);
  ASSERT_NE(
      findAction(actions, ui::RelationshipActionKind::MakePrimaryLinkRoot),
      nullptr);
  EXPECT_TRUE(
      findAction(actions, ui::RelationshipActionKind::MakePrimaryLinkRoot)
          ->enabled);
  EXPECT_EQ(
      findAction(actions, ui::RelationshipActionKind::MakePrimaryLinkRoot)
          ->label,
      "Make forearm Root Link");

  const auto rooted = ui::applyRelationshipAction(
      engine, ui::RelationshipActionKind::MakePrimaryLinkRoot);
  EXPECT_TRUE(rooted.ok);
  EXPECT_EQ(rooted.object, forearm);
  EXPECT_EQ(rooted.message, "Made forearm a root link");
  ASSERT_NE(engine.objects().model().find(forearm), nullptr);
  EXPECT_EQ(engine.objects().model().find(forearm)->parentLink, kNoObject);
  EXPECT_EQ(engine.objects().model().find(forearm)->parent, arm);

  ASSERT_TRUE(engine.undo());
  ASSERT_NE(engine.objects().model().find(forearm), nullptr);
  EXPECT_EQ(engine.objects().model().find(forearm)->parentLink, base);
}

TEST(DartsimRelationshipActions, ReportsCommandRejectionsAndUnknownActions)
{
  SimEngine engine;
  SceneObject rootFrame;
  rootFrame.type = ObjectType::FreeFrame;
  rootFrame.name = "tool";
  engine.objects().model().add(rootFrame);

  SceneObject rootFixed;
  rootFixed.type = ObjectType::FixedFrame;
  rootFixed.name = "tool";
  const ObjectId fixed = engine.objects().model().add(rootFixed);
  engine.objects().rebuild();
  ASSERT_TRUE(engine.select(fixed));

  const auto detached = ui::applyRelationshipAction(
      engine, ui::RelationshipActionKind::DetachPrimaryToWorld);
  EXPECT_FALSE(detached.ok);
  EXPECT_EQ(detached.message, "Detach rejected");
  ASSERT_NE(engine.objects().model().find(fixed), nullptr);
  EXPECT_EQ(engine.objects().model().find(fixed)->type, ObjectType::FixedFrame);

  const auto unknown = ui::applyRelationshipAction(
      engine, static_cast<ui::RelationshipActionKind>(999));
  EXPECT_FALSE(unknown.ok);
  EXPECT_EQ(unknown.message, "Unknown relationship action");
}
