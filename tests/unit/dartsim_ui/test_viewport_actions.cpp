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

#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>
#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/sim_engine.hpp>
#include <dartsim_ui/outliner_actions.hpp>
#include <dartsim_ui/viewport_actions.hpp>
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

TEST(DartsimViewportActions, ViewportMoveDeltaUsesWorldAxesAndFastScale)
{
  ui::ViewportMoveInput input;
  input.right = true;
  input.forward = true;
  input.down = true;
  input.step = 0.25;
  EXPECT_TRUE(
      ui::viewportMoveDelta(input).isApprox(
          Eigen::Vector3d(0.25, 0.25, -0.25)));

  input.fast = true;
  input.fastMultiplier = 3.0;
  EXPECT_TRUE(
      ui::viewportMoveDelta(input).isApprox(
          Eigen::Vector3d(0.75, 0.75, -0.75)));

  input.left = true;
  input.backward = true;
  input.up = true;
  EXPECT_TRUE(ui::viewportMoveDelta(input).isApprox(Eigen::Vector3d::Zero()));

  input.step = -1.0;
  EXPECT_TRUE(ui::viewportMoveDelta(input).isApprox(Eigen::Vector3d::Zero()));

  input.step = 0.25;
  input.fastMultiplier = 0.0;
  EXPECT_TRUE(ui::viewportMoveDelta(input).isApprox(Eigen::Vector3d::Zero()));
}

TEST(DartsimViewportActions, MoveSelectedFromViewportIsUndoable)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  ui::ViewportMoveInput input;
  input.left = true;
  input.up = true;
  input.step = 0.5;
  EXPECT_TRUE(ui::moveSelectedFromViewport(engine, input));
  ASSERT_NE(engine.objects().model().find(box), nullptr);
  EXPECT_TRUE(
      engine.objects().model().find(box)->transform.translation().isApprox(
          Eigen::Vector3d(0.5, 2.0, 3.5)));

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(
      engine.objects().model().find(box)->transform.translation().isApprox(
          Eigen::Vector3d(1.0, 2.0, 3.0)));

  ASSERT_TRUE(engine.redo());
  EXPECT_TRUE(
      engine.objects().model().find(box)->transform.translation().isApprox(
          Eigen::Vector3d(0.5, 2.0, 3.5)));
}

TEST(DartsimViewportActions, TransformGizmoTracksAndCommitsSelection)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(0.0, 0.0, 1.0), "sphere"));
  const ObjectId sphere = engine.selection().primary();
  ASSERT_NE(sphere, kNoObject);

  ui::ViewportTransformGizmo gizmo = ui::makeViewportTransformGizmo();
  ASSERT_NE(gizmo.target, nullptr);
  gizmo.target.reset();
  EXPECT_TRUE(ui::syncViewportTransformGizmo(gizmo, engine));
  ASSERT_NE(gizmo.target, nullptr);
  EXPECT_EQ(gizmo.object, sphere);
  EXPECT_EQ(gizmo.gizmo.label, "sphere Transform");
  EXPECT_TRUE(gizmo.target->getWorldTransform().translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 1.0)));

  Eigen::Isometry3d moved = translation(2.0, 0.5, 1.5);
  moved.linear()
      = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(ui::applyViewportTransformGizmo(engine, gizmo, moved));
  ASSERT_NE(engine.objects().model().find(sphere), nullptr);
  EXPECT_TRUE(
      engine.objects().model().find(sphere)->transform.matrix().isApprox(
          moved.matrix()));
  EXPECT_TRUE(
      gizmo.target->getWorldTransform().matrix().isApprox(moved.matrix()));

  EXPECT_FALSE(ui::applyViewportTransformGizmo(engine, gizmo, moved));

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(ui::syncViewportTransformGizmo(gizmo, engine));
  EXPECT_TRUE(
      engine.objects().model().find(sphere)->transform.translation().isApprox(
          Eigen::Vector3d(0.0, 0.0, 1.0)));
}

TEST(DartsimViewportActions, TransformGizmoRejectsUnsupportedHiddenAndRunMode)
{
  SimEngine engine;
  ui::ViewportTransformGizmo gizmo = ui::makeViewportTransformGizmo();

  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  EXPECT_FALSE(ui::syncViewportTransformGizmo(gizmo, engine));
  EXPECT_EQ(gizmo.object, kNoObject);

  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_TRUE(ui::syncViewportTransformGizmo(gizmo, engine));
  ASSERT_TRUE(ui::setOutlinerVisibility(engine, box, false));
  EXPECT_FALSE(ui::syncViewportTransformGizmo(gizmo, engine));
  EXPECT_EQ(gizmo.object, kNoObject);

  ASSERT_TRUE(ui::setOutlinerVisibility(engine, box, true));
  ASSERT_TRUE(ui::syncViewportTransformGizmo(gizmo, engine));
  engine.simulation().play();
  EXPECT_FALSE(
      ui::applyViewportTransformGizmo(
          engine, gizmo, translation(1.0, 0.0, 1.0)));
  EXPECT_TRUE(
      engine.objects().model().find(box)->transform.translation().isApprox(
          Eigen::Vector3d(0.0, 0.0, 1.0)));
}
