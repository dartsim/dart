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
#include <dartsim_ui/palette_actions.hpp>
#include <dartsim_ui/viewport_actions.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <cstddef>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

dart::gui::OrbitCamera testCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(10.0, 20.0, 30.0);
  camera.yaw = -0.5;
  camera.pitch = 0.25;
  camera.distance = 12.0;
  return camera;
}

std::size_t countRenderItemsOfType(
    const SimEngine& engine,
    const std::vector<RenderItem>& items,
    ObjectType type)
{
  return static_cast<std::size_t>(std::count_if(
      items.begin(), items.end(), [&engine, type](const RenderItem& item) {
        const SceneObject* object = engine.objects().model().find(item.id);
        return object != nullptr && object->type == type;
      }));
}

void expectCameraNear(
    const dart::gui::OrbitCamera& actual,
    const dart::gui::OrbitCamera& expected)
{
  EXPECT_TRUE(actual.target.isApprox(expected.target));
  EXPECT_TRUE(actual.up.isApprox(expected.up));
  EXPECT_EQ(actual.yaw, expected.yaw);
  EXPECT_EQ(actual.pitch, expected.pitch);
  EXPECT_EQ(actual.distance, expected.distance);
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

TEST(DartsimViewportActions, LayerFiltersSeparateRigidBodiesAndLinks)
{
  SimEngine engine;
  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddBoxRigidBody)
          .ok);
  const ObjectId box = engine.selection().primary();
  ASSERT_TRUE(
      ui::applyPaletteAction(
          engine, ui::PaletteActionKind::AddTwoLinkArmExample)
          .ok);

  ui::ViewportLayerFilterState filters;
  std::vector<RenderItem> items
      = ui::filteredViewportRenderItems(engine, filters);
  EXPECT_EQ(countRenderItemsOfType(engine, items, ObjectType::RigidBody), 1u);
  EXPECT_EQ(countRenderItemsOfType(engine, items, ObjectType::Link), 3u);

  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, false)
          .ok);
  items = ui::filteredViewportRenderItems(engine, filters);
  EXPECT_EQ(countRenderItemsOfType(engine, items, ObjectType::RigidBody), 0u);
  EXPECT_EQ(countRenderItemsOfType(engine, items, ObjectType::Link), 3u);
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, box, filters));
  EXPECT_TRUE(engine.objects().model().find(box)->visible);

  ASSERT_TRUE(
      ui::setViewportLayerVisible(filters, ui::ViewportLayerKind::Links, false)
          .ok);
  EXPECT_TRUE(ui::filteredViewportRenderItems(engine, filters).empty());

  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, true)
          .ok);
  items = ui::filteredViewportRenderItems(engine, filters);
  EXPECT_EQ(items.size(), 1u);
  EXPECT_EQ(items.front().id, box);
}

TEST(DartsimViewportActions, LayerFiltersClassifyAllViewportObjectTypes)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const ObjectId body = engine.selection().primary();
  ASSERT_NE(body, kNoObject);

  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId link = engine.selection().primary();
  ASSERT_NE(link, kNoObject);

  engine.execute(
      commands::addFreeFrame(translation(1.0, 0.0, 0.0), "free_frame"));
  const ObjectId freeFrame = engine.selection().primary();
  ASSERT_NE(freeFrame, kNoObject);
  engine.execute(
      commands::addFixedFrame(
          freeFrame, translation(0.0, 1.0, 0.0), "fixed_frame"));
  const ObjectId fixedFrame = engine.selection().primary();
  ASSERT_NE(fixedFrame, kNoObject);

  SceneObject joint;
  joint.type = ObjectType::Joint;
  joint.parent = arm;
  joint.name = "raw_joint";
  const ObjectId jointId = engine.objects().model().add(std::move(joint));
  engine.objects().rebuild();

  ui::ViewportLayerFilterState filters;
  EXPECT_TRUE(ui::isViewportObjectVisible(engine, body, filters));
  EXPECT_TRUE(ui::isViewportObjectVisible(engine, link, filters));
  EXPECT_TRUE(ui::isViewportObjectVisible(engine, freeFrame, filters));
  EXPECT_TRUE(ui::isViewportObjectVisible(engine, fixedFrame, filters));
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, arm, filters));
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, jointId, filters));
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, 999999, filters));

  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, false)
          .ok);
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, body, filters));
  EXPECT_TRUE(ui::isViewportObjectVisible(engine, link, filters));

  ASSERT_TRUE(
      ui::setViewportLayerVisible(filters, ui::ViewportLayerKind::Links, false)
          .ok);
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, link, filters));

  ASSERT_TRUE(
      ui::setViewportLayerVisible(filters, ui::ViewportLayerKind::Frames, false)
          .ok);
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, freeFrame, filters));
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, fixedFrame, filters));

  ASSERT_TRUE(
      ui::setViewportLayerVisible(filters, ui::ViewportLayerKind::Frames, true)
          .ok);
  ASSERT_TRUE(ui::setOutlinerVisibility(engine, freeFrame, false));
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, freeFrame, filters));
  EXPECT_FALSE(ui::isViewportObjectVisible(engine, fixedFrame, filters));
}

TEST(DartsimViewportActions, LayerFilterTogglesAreViewOnly)
{
  SimEngine engine;
  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddBoxRigidBody)
          .ok);
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.commands().clearHistory();
  engine.markProjectClean();

  ui::ViewportLayerFilterState filters;
  const auto hidden = ui::setViewportLayerVisible(
      filters, ui::ViewportLayerKind::RigidBodies, false);
  EXPECT_TRUE(hidden.ok);
  EXPECT_EQ(hidden.message, "Rigid Bodies hidden");
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_TRUE(engine.objects().model().find(box)->visible);

  const auto actions = ui::buildViewportLayerFilterActions(filters);
  ASSERT_EQ(actions.size(), 3u);
  EXPECT_FALSE(actions[0].checked);
  EXPECT_TRUE(actions[1].checked);
  EXPECT_TRUE(actions[2].checked);
}

TEST(DartsimViewportActions, LayerFiltersAffectViewportSelectionAndMovement)
{
  SimEngine engine;
  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddBoxRigidBody)
          .ok);
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  const Eigen::Vector3d original
      = engine.objects().model().find(box)->transform.translation();

  ui::ViewportLayerFilterState filters;
  EXPECT_NE(ui::selectedViewportRenderable(engine, filters), 0u);
  EXPECT_NE(ui::selectedViewportLabel(engine, filters), "none");

  engine.execute(commands::addFreeFrame(translation(3.0, 0.0, 0.0), "marker"));
  const ObjectId marker = engine.selection().primary();
  ASSERT_NE(marker, kNoObject);
  engine.commands().clearHistory();
  engine.markProjectClean();
  engine.select(box);

  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, false)
          .ok);
  EXPECT_EQ(ui::selectedViewportRenderable(engine, filters), 0u);
  EXPECT_EQ(ui::selectedViewportLabel(engine, filters), "none");

  ASSERT_TRUE(engine.select(marker));
  EXPECT_FALSE(
      ui::selectViewportRenderable(
          engine, filters, ui::renderableIdForObject(box)));
  EXPECT_EQ(engine.selection().primary(), marker);
  EXPECT_TRUE(ui::selectViewportRenderable(engine, filters, 0));
  EXPECT_EQ(engine.selection().primary(), kNoObject);
  ASSERT_TRUE(engine.select(box));

  ui::ViewportMoveInput input;
  input.right = true;
  EXPECT_FALSE(ui::moveSelectedFromViewport(engine, filters, input));
  EXPECT_TRUE(
      engine.objects().model().find(box)->transform.translation().isApprox(
          original));
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());

  ui::ViewportTransformGizmo gizmo = ui::makeViewportTransformGizmo();
  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, true)
          .ok);
  ASSERT_TRUE(ui::syncViewportTransformGizmo(gizmo, engine, filters));
  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, false)
          .ok);
  EXPECT_FALSE(
      ui::applyViewportTransformGizmo(
          engine, gizmo, filters, translation(8.0, 0.0, 0.0)));
  EXPECT_FALSE(ui::syncViewportTransformGizmo(gizmo, engine, filters));
  EXPECT_EQ(gizmo.object, kNoObject);
  EXPECT_TRUE(
      engine.objects().model().find(box)->transform.translation().isApprox(
          original));

  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, true)
          .ok);
  EXPECT_TRUE(ui::moveSelectedFromViewport(engine, filters, input));
}

TEST(DartsimViewportActions, LayerFiltersAffectFrameFocusAndGizmos)
{
  SimEngine engine;
  engine.execute(commands::addFreeFrame(translation(1.0, 2.0, 3.0), "frame"));
  const ObjectId frame = engine.selection().primary();
  ASSERT_NE(frame, kNoObject);
  engine.commands().clearHistory();
  engine.markProjectClean();

  ui::ViewportLayerFilterState filters;
  EXPECT_EQ(ui::selectedViewportRenderable(engine, filters), 0u);
  EXPECT_NE(ui::selectedViewportLabel(engine, filters), "none");
  const auto focused = ui::applyViewportCameraAction(
      engine,
      testCamera(),
      ui::ViewportCameraActionKind::FocusSelection,
      filters);
  ASSERT_TRUE(focused.ok);
  ASSERT_TRUE(focused.camera.has_value());
  EXPECT_TRUE(focused.camera->target.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  ui::ViewportTransformGizmo gizmo = ui::makeViewportTransformGizmo();
  EXPECT_TRUE(ui::syncViewportTransformGizmo(gizmo, engine, filters));
  EXPECT_EQ(gizmo.object, frame);

  ASSERT_TRUE(
      ui::setViewportLayerVisible(filters, ui::ViewportLayerKind::Frames, false)
          .ok);
  EXPECT_EQ(ui::selectedViewportLabel(engine, filters), "none");
  const auto hiddenFocus = ui::applyViewportCameraAction(
      engine,
      testCamera(),
      ui::ViewportCameraActionKind::FocusSelection,
      filters);
  EXPECT_FALSE(hiddenFocus.ok);
  EXPECT_FALSE(
      ui::applyViewportTransformGizmo(
          engine, gizmo, filters, translation(4.0, 5.0, 6.0)));
  ui::ViewportMoveInput input;
  input.up = true;
  EXPECT_FALSE(ui::moveSelectedFromViewport(engine, filters, input));
  EXPECT_TRUE(
      engine.objects().model().find(frame)->transform.translation().isApprox(
          Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_FALSE(ui::syncViewportTransformGizmo(gizmo, engine, filters));
  EXPECT_EQ(gizmo.object, kNoObject);
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimViewportActions, BuildsCameraActionsFromVisibleSceneState)
{
  SimEngine engine;
  const auto emptyActions = ui::buildViewportCameraActions(engine);
  ASSERT_GE(emptyActions.size(), 2u);
  EXPECT_EQ(emptyActions[0].kind, ui::ViewportCameraActionKind::FitScene);
  EXPECT_FALSE(emptyActions[0].enabled);
  EXPECT_EQ(emptyActions[0].disabledReason, "No visible objects");
  EXPECT_EQ(emptyActions[1].kind, ui::ViewportCameraActionKind::FocusSelection);
  EXPECT_FALSE(emptyActions[1].enabled);

  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  const auto visibleActions = ui::buildViewportCameraActions(engine);
  ASSERT_GE(visibleActions.size(), 9u);
  EXPECT_TRUE(visibleActions[0].enabled);
  EXPECT_TRUE(visibleActions[1].enabled);

  ASSERT_TRUE(ui::setOutlinerVisibility(engine, box, false));
  const auto hiddenActions = ui::buildViewportCameraActions(engine);
  EXPECT_FALSE(hiddenActions[0].enabled);
  EXPECT_FALSE(hiddenActions[1].enabled);
}

TEST(DartsimViewportActions, LayerFiltersAffectCameraActionsWithoutEditingScene)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(-10.0, 0.0, 0.5), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId link = engine.selection().primary();
  ASSERT_NE(link, kNoObject);
  engine.commands().clearHistory();
  engine.markProjectClean();

  ui::ViewportLayerFilterState filters;
  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, false)
          .ok);
  const auto fitWithoutRigidBodies = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FitScene, filters);
  ASSERT_TRUE(fitWithoutRigidBodies.ok);
  ASSERT_TRUE(fitWithoutRigidBodies.camera.has_value());
  EXPECT_NEAR(fitWithoutRigidBodies.camera->target.x(), 0.0, 2.0);

  ASSERT_TRUE(
      ui::setViewportLayerVisible(filters, ui::ViewportLayerKind::Links, false)
          .ok);
  const auto hiddenScene = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FitScene, filters);
  EXPECT_FALSE(hiddenScene.ok);
  EXPECT_FALSE(hiddenScene.camera.has_value());

  ASSERT_TRUE(engine.select(box));
  const auto hiddenSelection = ui::applyViewportCameraAction(
      engine,
      testCamera(),
      ui::ViewportCameraActionKind::FocusSelection,
      filters);
  EXPECT_FALSE(hiddenSelection.ok);

  const auto preset = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::Perspective, filters);
  EXPECT_TRUE(preset.ok);
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimViewportActions, FitSceneFramesAllVisibleRenderables)
{
  SimEngine engine;
  const auto empty = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FitScene);
  EXPECT_FALSE(empty.ok);
  EXPECT_FALSE(empty.camera.has_value());
  EXPECT_EQ(empty.message, "No visible objects");

  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(-2.0, 0.0, 0.5), "left_box"));
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(4.0, 2.0, 1.0), "right_sphere"));

  const dart::gui::OrbitCamera current = testCamera();
  const auto result = ui::applyViewportCameraAction(
      engine, current, ui::ViewportCameraActionKind::FitScene);

  ASSERT_TRUE(result.ok);
  ASSERT_TRUE(result.camera.has_value());
  EXPECT_EQ(result.message, "Fit scene");
  EXPECT_NEAR(result.camera->target.x(), 1.0, 0.75);
  EXPECT_NEAR(result.camera->target.y(), 1.0, 0.75);
  EXPECT_NEAR(result.camera->target.z(), 0.75, 0.75);
  EXPECT_EQ(result.camera->yaw, current.yaw);
  EXPECT_EQ(result.camera->pitch, current.pitch);
  EXPECT_GT(result.camera->distance, current.distance * 0.5);
}

TEST(DartsimViewportActions, FitSceneUsesFiniteBoundsForAllPrimitiveShapes)
{
  SimEngine engine;
  ASSERT_TRUE(
      ui::applyPaletteAction(
          engine, ui::PaletteActionKind::AddGroundAndBoxExample)
          .ok);
  ASSERT_TRUE(
      ui::applyPaletteAction(
          engine, ui::PaletteActionKind::AddCylinderRigidBody)
          .ok);
  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddCapsuleRigidBody)
          .ok);

  const auto result = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FitScene);

  ASSERT_TRUE(result.ok);
  ASSERT_TRUE(result.camera.has_value());
  EXPECT_TRUE(result.camera->target.allFinite());
  EXPECT_TRUE(std::isfinite(result.camera->distance));
  EXPECT_GE(result.camera->distance, 0.8);
  EXPECT_LE(result.camera->distance, 80.0);
}

TEST(DartsimViewportActions, FocusSelectionFramesOnlyVisibleSelectedObject)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(-8.0, 0.0, 0.5), "left_box"));
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(3.0, 4.0, 2.0), "sphere"));
  const ObjectId sphere = engine.selection().primary();
  ASSERT_NE(sphere, kNoObject);
  engine.commands().clearHistory();
  engine.markProjectClean();

  const auto focused = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FocusSelection);
  ASSERT_TRUE(focused.ok);
  ASSERT_TRUE(focused.camera.has_value());
  EXPECT_TRUE(
      focused.camera->target.isApprox(Eigen::Vector3d(3.0, 4.0, 2.0), 1e-9));
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());

  ASSERT_TRUE(ui::setOutlinerVisibility(engine, sphere, false));
  const auto hidden = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FocusSelection);
  EXPECT_FALSE(hidden.ok);
  EXPECT_FALSE(hidden.camera.has_value());
  EXPECT_EQ(hidden.message, "No visible selected object");
}

TEST(DartsimViewportActions, FocusSelectionFramesAllVisibleSelectedObjects)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(-5.0, 0.0, 0.5), "left_box"));
  const ObjectId left = engine.selection().primary();
  ASSERT_NE(left, kNoObject);
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(5.0, 0.0, 0.5), "right_sphere"));
  const ObjectId right = engine.selection().primary();
  ASSERT_NE(right, kNoObject);

  ASSERT_TRUE(engine.select(left, false));
  ASSERT_TRUE(engine.select(right, true));
  const auto focused = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FocusSelection);

  ASSERT_TRUE(focused.ok);
  ASSERT_TRUE(focused.camera.has_value());
  EXPECT_NEAR(focused.camera->target.x(), 0.0, 0.75);
  EXPECT_GT(focused.camera->distance, 10.0);
}

TEST(DartsimViewportActions, FocusSelectionCanFrameSelectedFrames)
{
  SimEngine engine;
  engine.execute(
      commands::addFreeFrame(translation(2.0, -3.0, 4.0), "target_frame"));
  const ObjectId frame = engine.selection().primary();
  ASSERT_NE(frame, kNoObject);
  engine.commands().clearHistory();
  engine.markProjectClean();

  const auto focused = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FocusSelection);
  ASSERT_TRUE(focused.ok);
  ASSERT_TRUE(focused.camera.has_value());
  EXPECT_TRUE(focused.camera->target.isApprox(Eigen::Vector3d(2.0, -3.0, 4.0)));
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());

  ASSERT_TRUE(ui::setOutlinerVisibility(engine, frame, false));
  const auto hidden = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FocusSelection);
  EXPECT_FALSE(hidden.ok);
}

TEST(DartsimViewportActions, FocusSelectionRejectsNonRenderableSelection)
{
  SimEngine engine;
  const auto empty = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FocusSelection);
  EXPECT_FALSE(empty.ok);
  EXPECT_EQ(empty.message, "No visible selected object");

  engine.execute(commands::addMultiBody("arm"));
  const auto multibody = ui::applyViewportCameraAction(
      engine, testCamera(), ui::ViewportCameraActionKind::FocusSelection);
  EXPECT_FALSE(multibody.ok);
  EXPECT_FALSE(multibody.camera.has_value());
}

TEST(DartsimViewportActions, CameraPresetsPreserveTargetAndDistance)
{
  const dart::gui::OrbitCamera current = testCamera();
  SimEngine engine;

  const auto front = ui::applyViewportCameraAction(
      engine, current, ui::ViewportCameraActionKind::Front);
  ASSERT_TRUE(front.ok);
  ASSERT_TRUE(front.camera.has_value());
  EXPECT_TRUE(front.camera->target.isApprox(current.target));
  EXPECT_EQ(front.camera->distance, current.distance);
  EXPECT_NEAR(front.camera->yaw, -3.14159265358979323846 * 0.5, 1e-12);
  EXPECT_NEAR(front.camera->pitch, 0.0, 1e-12);
  const dart::gui::OrbitCameraBasis frontBasis
      = dart::gui::makeOrbitCameraBasis(*front.camera);
  EXPECT_TRUE(frontBasis.forward.allFinite());
  EXPECT_TRUE(frontBasis.right.allFinite());

  const auto back = ui::applyViewportCameraAction(
      engine, current, ui::ViewportCameraActionKind::Back);
  ASSERT_TRUE(back.ok);
  ASSERT_TRUE(back.camera.has_value());
  EXPECT_NEAR(back.camera->yaw, 3.14159265358979323846 * 0.5, 1e-12);
  EXPECT_NEAR(back.camera->pitch, 0.0, 1e-12);

  const auto left = ui::applyViewportCameraAction(
      engine, current, ui::ViewportCameraActionKind::Left);
  ASSERT_TRUE(left.ok);
  ASSERT_TRUE(left.camera.has_value());
  EXPECT_NEAR(left.camera->yaw, 3.14159265358979323846, 1e-12);
  EXPECT_NEAR(left.camera->pitch, 0.0, 1e-12);

  const auto right = ui::applyViewportCameraAction(
      engine, current, ui::ViewportCameraActionKind::Right);
  ASSERT_TRUE(right.ok);
  ASSERT_TRUE(right.camera.has_value());
  EXPECT_NEAR(right.camera->yaw, 0.0, 1e-12);
  EXPECT_NEAR(right.camera->pitch, 0.0, 1e-12);

  const auto top = ui::applyViewportCameraAction(
      engine, current, ui::ViewportCameraActionKind::Top);
  ASSERT_TRUE(top.ok);
  ASSERT_TRUE(top.camera.has_value());
  EXPECT_TRUE(top.camera->target.isApprox(current.target));
  EXPECT_EQ(top.camera->distance, current.distance);
  EXPECT_NEAR(top.camera->pitch, 1.45, 1e-12);
  const dart::gui::OrbitCameraBasis topBasis
      = dart::gui::makeOrbitCameraBasis(*top.camera);
  EXPECT_TRUE(topBasis.forward.allFinite());
  EXPECT_TRUE(topBasis.right.allFinite());
  EXPECT_GT(topBasis.right.norm(), 0.0);

  const auto bottom = ui::applyViewportCameraAction(
      engine, current, ui::ViewportCameraActionKind::Bottom);
  ASSERT_TRUE(bottom.ok);
  ASSERT_TRUE(bottom.camera.has_value());
  EXPECT_NEAR(bottom.camera->pitch, -1.45, 1e-12);
  const dart::gui::OrbitCameraBasis bottomBasis
      = dart::gui::makeOrbitCameraBasis(*bottom.camera);
  EXPECT_TRUE(bottomBasis.forward.allFinite());
  EXPECT_TRUE(bottomBasis.right.allFinite());

  dart::gui::OrbitCamera invalid = current;
  invalid.target
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  invalid.distance = -1.0;
  const auto perspective = ui::applyViewportCameraAction(
      engine, invalid, ui::ViewportCameraActionKind::Perspective);
  ASSERT_TRUE(perspective.ok);
  ASSERT_TRUE(perspective.camera.has_value());
  EXPECT_TRUE(perspective.camera->target.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GE(perspective.camera->distance, 0.8);
}

TEST(DartsimViewportActions, ViewportLayoutActionsAreViewOnly)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 0.5), "box"));
  engine.commands().clearHistory();
  engine.markProjectClean();

  ui::ViewportLayoutState layout;
  auto actions = ui::buildViewportLayoutActions(layout);
  ASSERT_EQ(actions.size(), 6u);
  EXPECT_EQ(actions[0].kind, ui::ViewportLayoutActionKind::SingleView);
  EXPECT_TRUE(actions[0].checked);
  EXPECT_EQ(actions[1].kind, ui::ViewportLayoutActionKind::QuadView);
  EXPECT_FALSE(actions[1].checked);
  EXPECT_EQ(
      actions[2].kind, ui::ViewportLayoutActionKind::ActivatePerspectivePane);
  EXPECT_TRUE(actions[2].checked);
  EXPECT_FALSE(actions[2].enabled);
  EXPECT_EQ(actions[2].disabledReason, "Enable Four View Layout");

  auto disabledPane = ui::applyViewportLayoutAction(
      layout, ui::ViewportLayoutActionKind::ActivateFrontPane);
  EXPECT_FALSE(disabledPane.ok);
  EXPECT_EQ(disabledPane.message, "Enable Four View Layout");
  EXPECT_EQ(layout.layout, ui::ViewportLayoutKind::Single);
  EXPECT_EQ(layout.activePane, ui::ViewportPaneKind::Perspective);
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());

  auto result = ui::applyViewportLayoutAction(
      layout, ui::ViewportLayoutActionKind::QuadView);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(layout.layout, ui::ViewportLayoutKind::Quad);

  actions = ui::buildViewportLayoutActions(layout);
  ASSERT_EQ(actions.size(), 6u);
  EXPECT_FALSE(actions[0].checked);
  EXPECT_TRUE(actions[1].checked);
  EXPECT_TRUE(actions[2].enabled);
  EXPECT_TRUE(actions[2].checked);

  result = ui::applyViewportLayoutAction(
      layout, ui::ViewportLayoutActionKind::ActivateFrontPane);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.message, "Front Pane active");
  EXPECT_EQ(layout.activePane, ui::ViewportPaneKind::Front);

  const auto frontCamera = ui::applyViewportCameraAction(
      engine, testCamera(), ui::viewportPaneCameraAction(layout.activePane));
  ASSERT_TRUE(frontCamera.ok);
  ASSERT_TRUE(frontCamera.camera.has_value());
  EXPECT_NEAR(frontCamera.camera->yaw, -3.14159265358979323846 * 0.5, 1e-12);
  EXPECT_NEAR(frontCamera.camera->pitch, 0.0, 1e-12);

  result = ui::applyViewportLayoutAction(
      layout, ui::ViewportLayoutActionKind::SingleView);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(layout.layout, ui::ViewportLayoutKind::Single);
  EXPECT_EQ(layout.activePane, ui::ViewportPaneKind::Front);
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimViewportActions, ViewportPaneCameraMappingsAreCanonical)
{
  EXPECT_EQ(
      ui::viewportPaneCameraAction(ui::ViewportPaneKind::Perspective),
      ui::ViewportCameraActionKind::Perspective);
  EXPECT_EQ(
      ui::viewportPaneCameraAction(ui::ViewportPaneKind::Front),
      ui::ViewportCameraActionKind::Front);
  EXPECT_EQ(
      ui::viewportPaneCameraAction(ui::ViewportPaneKind::Right),
      ui::ViewportCameraActionKind::Right);
  EXPECT_EQ(
      ui::viewportPaneCameraAction(ui::ViewportPaneKind::Top),
      ui::ViewportCameraActionKind::Top);
}

TEST(DartsimViewportActions, ViewportLayoutRemembersIndependentPaneCameras)
{
  SimEngine engine;
  engine.commands().clearHistory();
  engine.markProjectClean();

  ui::ViewportLayerFilterState filters;
  ui::ViewportLayoutState layout;
  ASSERT_TRUE(
      ui::applyViewportLayoutAction(
          layout, ui::ViewportLayoutActionKind::QuadView)
          .ok);

  struct PaneCase
  {
    ui::ViewportPaneKind uiPane = ui::ViewportPaneKind::Perspective;
    ui::ViewportLayoutActionKind action
        = ui::ViewportLayoutActionKind::ActivatePerspectivePane;
    dart::gui::ViewportPaneKind guiPane
        = dart::gui::ViewportPaneKind::Perspective;
    dart::gui::OrbitCamera camera;
  };

  std::array<PaneCase, 4> cases{
      PaneCase{
          ui::ViewportPaneKind::Perspective,
          ui::ViewportLayoutActionKind::ActivatePerspectivePane,
          dart::gui::ViewportPaneKind::Perspective,
          testCamera()},
      PaneCase{
          ui::ViewportPaneKind::Top,
          ui::ViewportLayoutActionKind::ActivateTopPane,
          dart::gui::ViewportPaneKind::Top,
          testCamera()},
      PaneCase{
          ui::ViewportPaneKind::Front,
          ui::ViewportLayoutActionKind::ActivateFrontPane,
          dart::gui::ViewportPaneKind::Front,
          testCamera()},
      PaneCase{
          ui::ViewportPaneKind::Right,
          ui::ViewportLayoutActionKind::ActivateRightPane,
          dart::gui::ViewportPaneKind::Right,
          testCamera()}};
  for (std::size_t i = 0; i < cases.size(); ++i) {
    auto& paneCase = cases[i];
    paneCase.camera.target = Eigen::Vector3d(
        1.0 + static_cast<double>(i),
        2.0 + static_cast<double>(i) * 3.0,
        3.0 + static_cast<double>(i) * 5.0);
    paneCase.camera.yaw = -0.4 + static_cast<double>(i) * 0.2;
    paneCase.camera.pitch = -0.2 + static_cast<double>(i) * 0.3;
    paneCase.camera.distance = 6.0 + static_cast<double>(i);

    ASSERT_TRUE(ui::applyViewportLayoutAction(layout, paneCase.action).ok);
    ui::rememberViewportActivePaneCamera(layout, paneCase.camera);
  }

  for (const PaneCase& paneCase : cases) {
    ASSERT_TRUE(ui::applyViewportLayoutAction(layout, paneCase.action).ok);
    const auto activeCamera
        = ui::activeViewportPaneCamera(engine, testCamera(), filters, layout);
    ASSERT_TRUE(activeCamera.ok);
    ASSERT_TRUE(activeCamera.camera.has_value());
    expectCameraNear(*activeCamera.camera, paneCase.camera);

    const auto layoutOptions
        = ui::viewportLayoutOptions(engine, paneCase.camera, filters, layout);
    ASSERT_EQ(layoutOptions.paneCount, dart::gui::kMaxViewportPanes);
    for (const PaneCase& expectedPane : cases) {
      const auto pane = std::find_if(
          layoutOptions.panes.begin(),
          layoutOptions.panes.begin()
              + static_cast<std::ptrdiff_t>(layoutOptions.paneCount),
          [&expectedPane](const dart::gui::ViewportPaneView& view) {
            return view.kind == expectedPane.guiPane;
          });
      ASSERT_NE(
          pane,
          layoutOptions.panes.begin()
              + static_cast<std::ptrdiff_t>(layoutOptions.paneCount));
      expectCameraNear(pane->camera, expectedPane.camera);
      EXPECT_EQ(pane->active, expectedPane.uiPane == paneCase.uiPane);
    }
  }

  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimViewportActions, RendererPaneActivationUsesLayoutActions)
{
  SimEngine engine;
  engine.commands().clearHistory();
  engine.markProjectClean();

  ui::ViewportLayoutState layout;
  const std::array<
      std::pair<dart::gui::ViewportPaneKind, ui::ViewportLayoutActionKind>,
      4>
      expectedActions{
          std::pair{
              dart::gui::ViewportPaneKind::Perspective,
              ui::ViewportLayoutActionKind::ActivatePerspectivePane},
          std::pair{
              dart::gui::ViewportPaneKind::Front,
              ui::ViewportLayoutActionKind::ActivateFrontPane},
          std::pair{
              dart::gui::ViewportPaneKind::Right,
              ui::ViewportLayoutActionKind::ActivateRightPane},
          std::pair{
              dart::gui::ViewportPaneKind::Top,
              ui::ViewportLayoutActionKind::ActivateTopPane}};
  for (const auto& [pane, expectedAction] : expectedActions) {
    const std::optional<ui::ViewportLayoutActionKind> action
        = ui::viewportPaneActivationAction(pane);
    ASSERT_TRUE(action.has_value());
    EXPECT_EQ(*action, expectedAction);
  }

  auto disabled = ui::applyViewportPaneActivation(
      layout, dart::gui::ViewportPaneKind::Top);
  EXPECT_FALSE(disabled.ok);
  EXPECT_EQ(disabled.message, "Enable Four View Layout");
  EXPECT_EQ(layout.layout, ui::ViewportLayoutKind::Single);
  EXPECT_EQ(layout.activePane, ui::ViewportPaneKind::Perspective);
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());

  ASSERT_TRUE(
      ui::applyViewportLayoutAction(
          layout, ui::ViewportLayoutActionKind::QuadView)
          .ok);
  auto result = ui::applyViewportPaneActivation(
      layout, dart::gui::ViewportPaneKind::Right);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.message, "Right Pane active");
  EXPECT_EQ(layout.activePane, ui::ViewportPaneKind::Right);
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimViewportActions, ViewportLayoutOptionsExposeRendererPaneViews)
{
  SimEngine engine;
  engine.commands().clearHistory();
  engine.markProjectClean();

  const dart::gui::OrbitCamera current = testCamera();
  ui::ViewportLayerFilterState filters;
  ui::ViewportLayoutState layout;

  const auto single
      = ui::viewportLayoutOptions(engine, current, filters, layout);
  EXPECT_EQ(single.mode, dart::gui::ViewportLayoutMode::Single);
  ASSERT_EQ(single.paneCount, 1u);
  EXPECT_EQ(single.panes[0].kind, dart::gui::ViewportPaneKind::Perspective);
  EXPECT_TRUE(single.panes[0].active);
  EXPECT_TRUE(single.panes[0].camera.target.isApprox(current.target));
  EXPECT_EQ(single.panes[0].camera.distance, current.distance);
  EXPECT_EQ(single.panes[0].camera.yaw, current.yaw);
  EXPECT_EQ(single.panes[0].camera.pitch, current.pitch);

  ASSERT_TRUE(
      ui::applyViewportLayoutAction(
          layout, ui::ViewportLayoutActionKind::QuadView)
          .ok);
  const auto quad = ui::viewportLayoutOptions(engine, current, filters, layout);
  EXPECT_EQ(quad.mode, dart::gui::ViewportLayoutMode::Quad);
  ASSERT_EQ(quad.paneCount, dart::gui::kMaxViewportPanes);

  const std::array<dart::gui::ViewportPaneKind, dart::gui::kMaxViewportPanes>
      expectedKinds{
          dart::gui::ViewportPaneKind::Perspective,
          dart::gui::ViewportPaneKind::Top,
          dart::gui::ViewportPaneKind::Front,
          dart::gui::ViewportPaneKind::Right};
  for (std::size_t i = 0; i < expectedKinds.size(); ++i) {
    EXPECT_EQ(quad.panes[i].kind, expectedKinds[i]);
  }
  EXPECT_TRUE(quad.panes[0].active);
  EXPECT_FALSE(quad.panes[1].active);
  EXPECT_FALSE(quad.panes[2].active);
  EXPECT_FALSE(quad.panes[3].active);
  EXPECT_EQ(quad.panes[0].camera.yaw, current.yaw);
  EXPECT_EQ(quad.panes[0].camera.pitch, current.pitch);
  EXPECT_NEAR(quad.panes[1].camera.pitch, 1.45, 1e-12);
  EXPECT_NEAR(quad.panes[2].camera.yaw, -3.14159265358979323846 * 0.5, 1e-12);
  EXPECT_NEAR(quad.panes[2].camera.pitch, 0.0, 1e-12);
  EXPECT_NEAR(quad.panes[3].camera.yaw, 0.0, 1e-12);

  ASSERT_TRUE(
      ui::applyViewportLayoutAction(
          layout, ui::ViewportLayoutActionKind::ActivateFrontPane)
          .ok);
  const auto activeFront
      = ui::viewportLayoutOptions(engine, current, filters, layout);
  ASSERT_EQ(activeFront.paneCount, dart::gui::kMaxViewportPanes);
  EXPECT_FALSE(activeFront.panes[0].active);
  EXPECT_TRUE(activeFront.panes[2].active);
  EXPECT_EQ(activeFront.panes[2].camera.yaw, current.yaw);
  EXPECT_EQ(activeFront.panes[2].camera.pitch, current.pitch);

  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimViewportActions, CameraControlActionsSetMouseModeWithoutEditing)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 0.5), "box"));
  engine.commands().clearHistory();
  engine.markProjectClean();

  ui::ViewportLayerFilterState filters;
  ui::ViewportCameraControlState controls;
  auto actions
      = ui::buildViewportCameraControlActions(engine, filters, controls);
  ASSERT_EQ(actions.size(), 4u);
  EXPECT_EQ(actions[0].kind, ui::ViewportCameraControlActionKind::OrbitMode);
  EXPECT_TRUE(actions[0].checked);
  EXPECT_FALSE(actions[1].checked);
  EXPECT_FALSE(actions[2].checked);
  EXPECT_FALSE(actions[3].checked);
  EXPECT_TRUE(actions[3].enabled);

  auto result = ui::applyViewportCameraControlAction(
      engine, filters, controls, ui::ViewportCameraControlActionKind::PanMode);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.message, "Pan Camera Mode");
  EXPECT_EQ(controls.mouseMode, dart::gui::OrbitCameraMouseMode::Pan);
  EXPECT_EQ(
      ui::viewportCameraControlOptions(controls).mouseMode,
      dart::gui::OrbitCameraMouseMode::Pan);

  result = ui::applyViewportCameraControlAction(
      engine, filters, controls, ui::ViewportCameraControlActionKind::ZoomMode);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(controls.mouseMode, dart::gui::OrbitCameraMouseMode::Zoom);

  result = ui::applyViewportCameraControlAction(
      engine,
      filters,
      controls,
      ui::ViewportCameraControlActionKind::OrbitMode);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(controls.mouseMode, dart::gui::OrbitCameraMouseMode::Orbit);
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimViewportActions, SelectionTrackingFollowsVisibleSelection)
{
  SimEngine engine;
  ui::ViewportLayerFilterState filters;
  ui::ViewportCameraControlState controls;

  auto result = ui::applyViewportCameraControlAction(
      engine,
      filters,
      controls,
      ui::ViewportCameraControlActionKind::ToggleTrackSelection);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message, "No visible selected object");
  EXPECT_FALSE(controls.trackSelection);

  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(4.0, -2.0, 0.5), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.commands().clearHistory();
  engine.markProjectClean();

  result = ui::applyViewportCameraControlAction(
      engine,
      filters,
      controls,
      ui::ViewportCameraControlActionKind::ToggleTrackSelection);
  EXPECT_TRUE(result.ok);
  EXPECT_TRUE(controls.trackSelection);

  const dart::gui::OrbitCamera current = testCamera();
  auto tracked = ui::trackedSelectionCamera(engine, current, filters, controls);
  ASSERT_TRUE(tracked.ok);
  ASSERT_TRUE(tracked.camera.has_value());
  EXPECT_TRUE(tracked.camera->target.isApprox(Eigen::Vector3d(4.0, -2.0, 0.5)));
  EXPECT_EQ(tracked.camera->distance, current.distance);
  EXPECT_EQ(tracked.camera->yaw, current.yaw);
  EXPECT_EQ(tracked.camera->pitch, current.pitch);
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_FALSE(engine.isProjectDirty());

  ASSERT_TRUE(
      ui::setViewportLayerVisible(
          filters, ui::ViewportLayerKind::RigidBodies, false)
          .ok);
  tracked = ui::trackedSelectionCamera(engine, current, filters, controls);
  EXPECT_FALSE(tracked.ok);
  EXPECT_FALSE(tracked.camera.has_value());
  auto actions
      = ui::buildViewportCameraControlActions(engine, filters, controls);
  ASSERT_EQ(actions.size(), 4u);
  EXPECT_TRUE(actions[3].checked);
  EXPECT_TRUE(actions[3].enabled);

  result = ui::applyViewportCameraControlAction(
      engine,
      filters,
      controls,
      ui::ViewportCameraControlActionKind::ToggleTrackSelection);
  EXPECT_TRUE(result.ok);
  EXPECT_FALSE(controls.trackSelection);
}
