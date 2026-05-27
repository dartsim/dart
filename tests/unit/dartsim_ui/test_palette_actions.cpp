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

#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dartsim_engine/sim_engine.hpp>
#include <dartsim_ui/palette_actions.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

using namespace dartsim;

namespace {

const ui::PaletteAction& findAction(
    const std::vector<ui::PaletteAction>& actions, ui::PaletteActionKind kind)
{
  const auto it = std::find_if(
      actions.begin(), actions.end(), [kind](const ui::PaletteAction& action) {
        return action.kind == kind;
      });
  EXPECT_NE(it, actions.end());
  return *it;
}

} // namespace

TEST(DartsimPaletteActions, BuildsContextSensitiveActionList)
{
  SimEngine engine;
  std::vector<ui::PaletteAction> actions = ui::buildPaletteActions(engine);
  ASSERT_EQ(actions.size(), 17u);

  const auto& box = findAction(actions, ui::PaletteActionKind::AddBoxRigidBody);
  EXPECT_EQ(box.label, "Rigid Body / Box");
  EXPECT_TRUE(box.enabled);

  const auto& rootLink
      = findAction(actions, ui::PaletteActionKind::AddRootLink);
  EXPECT_EQ(rootLink.label, "Link / Root Link");
  EXPECT_FALSE(rootLink.enabled);
  EXPECT_EQ(rootLink.disabledReason, "Select a MultiBody");

  const auto& childLink
      = findAction(actions, ui::PaletteActionKind::AddRevoluteLink);
  EXPECT_FALSE(childLink.enabled);
  EXPECT_EQ(childLink.disabledReason, "Select a Link");

  const auto& groundAndBox
      = findAction(actions, ui::PaletteActionKind::AddGroundAndBoxExample);
  EXPECT_EQ(groundAndBox.label, "Example / Ground + Box");
  EXPECT_TRUE(groundAndBox.enabled);

  const auto& twoLinkArm
      = findAction(actions, ui::PaletteActionKind::AddTwoLinkArmExample);
  EXPECT_EQ(twoLinkArm.label, "Example / Two-Link Arm");
  EXPECT_TRUE(twoLinkArm.enabled);

  const auto& cameraSensor
      = findAction(actions, ui::PaletteActionKind::AddCameraSensor);
  EXPECT_EQ(cameraSensor.label, "Sensor / Camera");
  EXPECT_TRUE(cameraSensor.enabled);

  const auto rootRejected
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddRootLink);
  EXPECT_FALSE(rootRejected.ok);
  EXPECT_EQ(rootRejected.message, "Select a MultiBody");

  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddMultiBody).ok);
  const ObjectId multiBody = engine.selection().primary();
  ASSERT_NE(multiBody, kNoObject);
  actions = ui::buildPaletteActions(engine);
  EXPECT_TRUE(findAction(actions, ui::PaletteActionKind::AddRootLink).enabled);
  EXPECT_FALSE(
      findAction(actions, ui::PaletteActionKind::AddFixedLink).enabled);

  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddRootLink).ok);
  const ObjectId root = engine.selection().primary();
  ASSERT_NE(root, kNoObject);
  actions = ui::buildPaletteActions(engine);
  EXPECT_FALSE(findAction(actions, ui::PaletteActionKind::AddRootLink).enabled);
  EXPECT_TRUE(findAction(actions, ui::PaletteActionKind::AddFixedLink).enabled);
  EXPECT_TRUE(
      findAction(actions, ui::PaletteActionKind::AddRevoluteLink).enabled);
  EXPECT_TRUE(
      findAction(actions, ui::PaletteActionKind::AddPrismaticLink).enabled);
}

TEST(DartsimPaletteActions, PaletteIsLockedDuringSimulationMode)
{
  SimEngine engine;
  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddBoxRigidBody)
          .ok);
  const std::size_t beforeSize = engine.objects().model().size();

  engine.simulation().play();
  ASSERT_FALSE(engine.canEditScene());
  const std::vector<ui::PaletteAction> actions
      = ui::buildPaletteActions(engine);
  for (const ui::PaletteAction& action : actions) {
    EXPECT_FALSE(action.enabled) << action.label;
    EXPECT_EQ(action.disabledReason, "Simulation Mode") << action.label;
  }

  const auto locked = ui::applyPaletteAction(
      engine, ui::PaletteActionKind::AddSphereRigidBody);
  EXPECT_FALSE(locked.ok);
  EXPECT_EQ(locked.message, "Scene locked");
  EXPECT_EQ(engine.objects().model().size(), beforeSize);
}

TEST(DartsimPaletteActions, CreatesPrimitiveRigidBodiesWithUsefulDefaults)
{
  SimEngine engine;

  const auto box
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddBoxRigidBody);
  ASSERT_TRUE(box.ok);
  ASSERT_NE(box.object, kNoObject);
  const SceneObject* boxObject = engine.objects().model().find(box.object);
  ASSERT_NE(boxObject, nullptr);
  EXPECT_EQ(boxObject->shape.type, ShapeType::Box);
  EXPECT_TRUE(
      boxObject->shape.dimensions.isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ASSERT_TRUE(engine.undo());

  const auto sphere = ui::applyPaletteAction(
      engine, ui::PaletteActionKind::AddSphereRigidBody);
  ASSERT_TRUE(sphere.ok);
  ASSERT_NE(sphere.object, kNoObject);
  const SceneObject* sphereObject
      = engine.objects().model().find(sphere.object);
  ASSERT_NE(sphereObject, nullptr);
  EXPECT_EQ(sphereObject->type, ObjectType::RigidBody);
  EXPECT_EQ(sphereObject->shape.type, ShapeType::Sphere);
  EXPECT_TRUE(
      sphereObject->shape.dimensions.isApprox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  EXPECT_TRUE(sphereObject->transform.translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 1.0)));
  ASSERT_EQ(engine.renderItems().size(), 1u);
  EXPECT_EQ(engine.renderItems().front().id, sphere.object);

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().empty());

  const auto plane = ui::applyPaletteAction(
      engine, ui::PaletteActionKind::AddPlaneRigidBody);
  ASSERT_TRUE(plane.ok);
  const SceneObject* planeObject = engine.objects().model().find(plane.object);
  ASSERT_NE(planeObject, nullptr);
  EXPECT_EQ(planeObject->shape.type, ShapeType::Plane);
  EXPECT_TRUE(planeObject->shape.dimensions.isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(
      planeObject->transform.translation().isApprox(Eigen::Vector3d::Zero()));

  ASSERT_TRUE(engine.undo());
  const auto cylinder = ui::applyPaletteAction(
      engine, ui::PaletteActionKind::AddCylinderRigidBody);
  ASSERT_TRUE(cylinder.ok);
  const SceneObject* cylinderObject
      = engine.objects().model().find(cylinder.object);
  ASSERT_NE(cylinderObject, nullptr);
  EXPECT_EQ(cylinderObject->shape.type, ShapeType::Cylinder);
  EXPECT_TRUE(cylinderObject->shape.dimensions.isApprox(
      Eigen::Vector3d(0.35, 1.0, 0.35)));

  ASSERT_TRUE(engine.undo());
  const auto capsule = ui::applyPaletteAction(
      engine, ui::PaletteActionKind::AddCapsuleRigidBody);
  ASSERT_TRUE(capsule.ok);
  const SceneObject* capsuleObject
      = engine.objects().model().find(capsule.object);
  ASSERT_NE(capsuleObject, nullptr);
  EXPECT_EQ(capsuleObject->shape.type, ShapeType::Capsule);
  EXPECT_TRUE(
      capsuleObject->shape.dimensions.isApprox(Eigen::Vector3d(0.3, 1.0, 0.3)));
}

TEST(DartsimPaletteActions, CreatesSensorsAtWorldOrSelectedFrameParent)
{
  SimEngine engine;

  const auto camera
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddCameraSensor);
  ASSERT_TRUE(camera.ok);
  EXPECT_EQ(camera.message, "Added camera sensor");
  ASSERT_NE(camera.object, kNoObject);
  const SceneObject* cameraObject
      = engine.objects().model().find(camera.object);
  ASSERT_NE(cameraObject, nullptr);
  EXPECT_EQ(cameraObject->type, ObjectType::Sensor);
  EXPECT_EQ(cameraObject->parent, kNoObject);
  EXPECT_EQ(cameraObject->sensor.kind, SensorKind::Camera);
  ASSERT_EQ(engine.renderItems().size(), 1u);
  EXPECT_EQ(engine.renderItems().front().id, camera.object);

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().empty());

  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddBoxRigidBody)
          .ok);
  const ObjectId body = engine.selection().primary();
  ASSERT_NE(body, kNoObject);

  const auto range
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddRangeSensor);
  ASSERT_TRUE(range.ok);
  EXPECT_EQ(range.message, "Added range sensor");
  const SceneObject* rangeObject = engine.objects().model().find(range.object);
  ASSERT_NE(rangeObject, nullptr);
  EXPECT_EQ(rangeObject->type, ObjectType::Sensor);
  EXPECT_EQ(rangeObject->parent, body);
  EXPECT_EQ(rangeObject->sensor.kind, SensorKind::Range);

  ASSERT_TRUE(engine.select(body));
  const auto contact
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddContactSensor);
  ASSERT_TRUE(contact.ok);
  const SceneObject* contactObject
      = engine.objects().model().find(contact.object);
  ASSERT_NE(contactObject, nullptr);
  EXPECT_EQ(contactObject->sensor.kind, SensorKind::Contact);
  EXPECT_EQ(contactObject->parent, body);
}

TEST(DartsimPaletteActions, CreatesLinksWithContextAndJointType)
{
  SimEngine engine;
  const auto multiBodyResult
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddMultiBody);
  ASSERT_TRUE(multiBodyResult.ok);
  const ObjectId multiBody = multiBodyResult.object;

  const auto rootResult
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddRootLink);
  ASSERT_TRUE(rootResult.ok);
  const ObjectId root = rootResult.object;
  const SceneObject* rootObject = engine.objects().model().find(root);
  ASSERT_NE(rootObject, nullptr);
  EXPECT_EQ(rootObject->type, ObjectType::Link);
  EXPECT_EQ(rootObject->parent, multiBody);
  EXPECT_EQ(rootObject->multiBody, multiBody);
  EXPECT_EQ(rootObject->parentLink, kNoObject);
  EXPECT_EQ(rootObject->jointType, JointKind::Fixed);

  const auto childResult
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddRevoluteLink);
  ASSERT_TRUE(childResult.ok);
  const SceneObject* child = engine.objects().model().find(childResult.object);
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(child->type, ObjectType::Link);
  EXPECT_EQ(child->parent, root);
  EXPECT_EQ(child->multiBody, multiBody);
  EXPECT_EQ(child->parentLink, root);
  EXPECT_EQ(child->jointType, JointKind::Revolute);

  const auto fixedChildResult
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddFixedLink);
  ASSERT_TRUE(fixedChildResult.ok);
  const SceneObject* fixedChild
      = engine.objects().model().find(fixedChildResult.object);
  ASSERT_NE(fixedChild, nullptr);
  EXPECT_EQ(fixedChild->parentLink, childResult.object);
  EXPECT_EQ(fixedChild->jointType, JointKind::Fixed);

  const auto prismaticChildResult
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddPrismaticLink);
  ASSERT_TRUE(prismaticChildResult.ok);
  const SceneObject* prismaticChild
      = engine.objects().model().find(prismaticChildResult.object);
  ASSERT_NE(prismaticChild, nullptr);
  EXPECT_EQ(prismaticChild->parentLink, fixedChildResult.object);
  EXPECT_EQ(prismaticChild->jointType, JointKind::Prismatic);

  auto arm = engine.objects().world().getMultibody(
      engine.objects().model().find(multiBody)->name);
  ASSERT_TRUE(arm.has_value());
  EXPECT_EQ(arm->getLinkCount(), 4u);
  EXPECT_EQ(arm->getJointCount(), 3u);
}

TEST(DartsimPaletteActions, CreatesFramesThroughUndoableCommands)
{
  SimEngine engine;
  const auto freeFrame
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddFreeFrame);
  ASSERT_TRUE(freeFrame.ok);
  const SceneObject* freeObject
      = engine.objects().model().find(freeFrame.object);
  ASSERT_NE(freeObject, nullptr);
  EXPECT_EQ(freeObject->type, ObjectType::FreeFrame);

  const auto fixedFrame
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddFixedFrame);
  ASSERT_TRUE(fixedFrame.ok);
  const SceneObject* fixedObject
      = engine.objects().model().find(fixedFrame.object);
  ASSERT_NE(fixedObject, nullptr);
  EXPECT_EQ(fixedObject->type, ObjectType::FixedFrame);
  EXPECT_EQ(engine.objects().model().size(), 2u);

  ASSERT_TRUE(engine.undo());
  EXPECT_EQ(engine.objects().model().size(), 1u);
  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().empty());
}

TEST(DartsimPaletteActions, FixedFrameRequiresFrameLikeSelection)
{
  SimEngine engine;
  auto rejected
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddFixedFrame);
  EXPECT_FALSE(rejected.ok);
  EXPECT_EQ(rejected.message, "Select a parent frame");

  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddBoxRigidBody)
          .ok);
  const ObjectId body = engine.selection().primary();
  ASSERT_NE(body, kNoObject);
  std::vector<ui::PaletteAction> actions = ui::buildPaletteActions(engine);
  EXPECT_TRUE(
      findAction(actions, ui::PaletteActionKind::AddFixedFrame).enabled);

  const auto bodyFixed
      = ui::applyPaletteAction(engine, ui::PaletteActionKind::AddFixedFrame);
  ASSERT_TRUE(bodyFixed.ok);
  const SceneObject* fixed = engine.objects().model().find(bodyFixed.object);
  ASSERT_NE(fixed, nullptr);
  EXPECT_EQ(fixed->type, ObjectType::FixedFrame);
  EXPECT_EQ(fixed->parent, body);

  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddMultiBody).ok);
  actions = ui::buildPaletteActions(engine);
  EXPECT_FALSE(
      findAction(actions, ui::PaletteActionKind::AddFixedFrame).enabled);
}

TEST(DartsimPaletteActions, CreatesGroundAndBoxExampleAsSingleUndoStep)
{
  SimEngine engine;

  const auto preset = ui::applyPaletteAction(
      engine, ui::PaletteActionKind::AddGroundAndBoxExample);
  ASSERT_TRUE(preset.ok);
  EXPECT_EQ(preset.message, "Added ground and box example");
  EXPECT_EQ(engine.objects().model().size(), 2u);
  EXPECT_EQ(engine.commands().undoLabel(), "Add Ground And Box Example");

  const std::optional<ObjectId> groundId
      = engine.objects().model().findChildByName(kNoObject, "ground");
  ASSERT_TRUE(groundId.has_value());
  const SceneObject* ground = engine.objects().model().find(*groundId);
  ASSERT_NE(ground, nullptr);
  EXPECT_EQ(ground->type, ObjectType::RigidBody);
  EXPECT_EQ(ground->shape.type, ShapeType::Plane);
  EXPECT_TRUE(
      ground->transform.translation().isApprox(Eigen::Vector3d::Zero()));

  const std::optional<ObjectId> boxId
      = engine.objects().model().findChildByName(kNoObject, "box");
  ASSERT_TRUE(boxId.has_value());
  const SceneObject* box = engine.objects().model().find(*boxId);
  ASSERT_NE(box, nullptr);
  EXPECT_EQ(box->type, ObjectType::RigidBody);
  EXPECT_EQ(box->shape.type, ShapeType::Box);
  EXPECT_TRUE(box->shape.dimensions.isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));
  EXPECT_TRUE(
      box->transform.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.5)));
  EXPECT_EQ(engine.selection().primary(), *boxId);
  EXPECT_EQ(preset.object, *boxId);

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().empty());
  ASSERT_TRUE(engine.redo());
  EXPECT_EQ(engine.objects().model().size(), 2u);
}

TEST(DartsimPaletteActions, CreatesTwoLinkArmExampleAsSingleUndoStep)
{
  SimEngine engine;

  const auto preset = ui::applyPaletteAction(
      engine, ui::PaletteActionKind::AddTwoLinkArmExample);
  ASSERT_TRUE(preset.ok);
  EXPECT_EQ(preset.message, "Added two-link arm example");
  EXPECT_EQ(engine.objects().model().size(), 4u);
  EXPECT_EQ(engine.commands().undoLabel(), "Add Two-Link Arm Example");

  const std::optional<ObjectId> armId
      = engine.objects().model().findChildByName(kNoObject, "two_link_arm");
  ASSERT_TRUE(armId.has_value());
  const SceneObject* arm = engine.objects().model().find(*armId);
  ASSERT_NE(arm, nullptr);
  EXPECT_EQ(arm->type, ObjectType::MultiBody);

  const std::optional<ObjectId> baseId
      = engine.objects().model().findChildByName(*armId, "base");
  ASSERT_TRUE(baseId.has_value());
  const SceneObject* base = engine.objects().model().find(*baseId);
  ASSERT_NE(base, nullptr);
  EXPECT_EQ(base->type, ObjectType::Link);
  EXPECT_EQ(base->multiBody, *armId);
  EXPECT_EQ(base->parentLink, kNoObject);
  EXPECT_EQ(base->jointType, JointKind::Fixed);
  EXPECT_EQ(base->shape.type, ShapeType::Cylinder);

  const std::optional<ObjectId> shoulderId
      = engine.objects().model().findChildByName(*baseId, "shoulder");
  ASSERT_TRUE(shoulderId.has_value());
  const SceneObject* shoulder = engine.objects().model().find(*shoulderId);
  ASSERT_NE(shoulder, nullptr);
  EXPECT_EQ(shoulder->type, ObjectType::Link);
  EXPECT_EQ(shoulder->multiBody, *armId);
  EXPECT_EQ(shoulder->parentLink, *baseId);
  EXPECT_EQ(shoulder->jointType, JointKind::Revolute);
  EXPECT_TRUE(shoulder->jointAxis.isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_EQ(shoulder->shape.type, ShapeType::Capsule);

  const std::optional<ObjectId> elbowId
      = engine.objects().model().findChildByName(*shoulderId, "elbow");
  ASSERT_TRUE(elbowId.has_value());
  const SceneObject* elbow = engine.objects().model().find(*elbowId);
  ASSERT_NE(elbow, nullptr);
  EXPECT_EQ(elbow->type, ObjectType::Link);
  EXPECT_EQ(elbow->multiBody, *armId);
  EXPECT_EQ(elbow->parentLink, *shoulderId);
  EXPECT_EQ(elbow->jointType, JointKind::Revolute);
  EXPECT_TRUE(elbow->jointAxis.isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_EQ(elbow->shape.type, ShapeType::Capsule);
  EXPECT_EQ(engine.selection().primary(), *elbowId);
  EXPECT_EQ(preset.object, *elbowId);

  auto armHandle = engine.objects().world().getMultibody(arm->name);
  ASSERT_TRUE(armHandle.has_value());
  EXPECT_EQ(armHandle->getLinkCount(), 3u);
  EXPECT_EQ(armHandle->getJointCount(), 2u);

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().empty());
}
