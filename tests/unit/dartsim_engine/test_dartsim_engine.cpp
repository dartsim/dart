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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Geometry>
#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/name_manager.hpp>
#include <dartsim_engine/scene_io.hpp>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/sim_engine.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

} // namespace

//==============================================================================
// SceneModel
//==============================================================================

TEST(SceneModel, AddFindRemoveRecursive)
{
  SceneModel model;
  SceneObject parent;
  parent.type = ObjectType::MultiBody;
  parent.name = "robot";
  const ObjectId parentId = model.add(parent);

  SceneObject child;
  child.type = ObjectType::Link;
  child.name = "base";
  child.parent = parentId;
  const ObjectId childId = model.add(child);

  EXPECT_TRUE(model.contains(parentId));
  EXPECT_TRUE(model.contains(childId));
  EXPECT_EQ(model.childrenOf(parentId).size(), 1u);
  EXPECT_EQ(model.rootChildren().size(), 1u);

  model.remove(parentId);
  EXPECT_FALSE(model.contains(parentId));
  EXPECT_FALSE(model.contains(childId)); // descendants removed too
  EXPECT_TRUE(model.empty());
}

TEST(SceneModel, ReparentRejectsCycle)
{
  SceneModel model;
  SceneObject a;
  a.name = "a";
  const ObjectId aId = model.add(a);
  SceneObject b;
  b.name = "b";
  b.parent = aId;
  const ObjectId bId = model.add(b);

  EXPECT_FALSE(model.reparent(aId, bId));      // a under its own descendant
  EXPECT_TRUE(model.reparent(bId, kNoObject)); // b to root
  EXPECT_EQ(model.find(bId)->parent, kNoObject);
  EXPECT_EQ(model.rootChildren().size(), 2u);
}

TEST(SceneModel, NameUniquenessHelper)
{
  SceneModel model;
  SceneObject a;
  a.name = "Body";
  model.add(a);

  EXPECT_FALSE(model.isNameAvailable(kNoObject, "Body"));
  EXPECT_TRUE(model.isNameAvailable(kNoObject, "Other"));
  EXPECT_EQ(NameManager::makeUnique(model, kNoObject, "Body"), "Body 1");
  EXPECT_EQ(NameManager::makeUnique(model, kNoObject, ""), "Object");

  SceneObject fallback;
  fallback.name = "Object";
  model.add(fallback);
  EXPECT_EQ(NameManager::makeUnique(model, kNoObject, ""), "Object 1");
}

TEST(SceneModel, MalformedParentsAndMissingIdsAreHandled)
{
  SceneModel model;
  SceneObject orphan;
  orphan.name = "orphan";
  orphan.parent = 9999;
  const ObjectId orphanId = model.add(orphan);
  ASSERT_NE(model.find(orphanId), nullptr);
  EXPECT_EQ(model.find(orphanId)->parent, kNoObject);
  ASSERT_EQ(model.rootChildren().size(), 1u);
  EXPECT_EQ(model.rootChildren().front(), orphanId);

  EXPECT_FALSE(model.reparent(9999, kNoObject));
  EXPECT_FALSE(model.reparent(orphanId, 9999));
  EXPECT_TRUE(model.childrenOf(9999).empty());

  EXPECT_EQ(model.findChildByName(kNoObject, "orphan"), orphanId);
  EXPECT_FALSE(model.findChildByName(kNoObject, "missing").has_value());

  model.remove(9999);
  EXPECT_TRUE(model.contains(orphanId));

  model.clear();
  EXPECT_TRUE(model.empty());
  EXPECT_TRUE(model.allIds().empty());
}

//==============================================================================
// ObjectManager: rebuild + render snapshot
//==============================================================================

TEST(ObjectManager, RebuildCreatesBodiesAndResolvesTransforms)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "box";
  body.transform = translation(1.0, 2.0, 3.0);
  const ObjectId id = objects.model().add(body);
  objects.rebuild();

  ASSERT_TRUE(objects.world().hasRigidBody("box"));
  const auto transform = objects.worldTransformOf(id);
  ASSERT_TRUE(transform.has_value());
  EXPECT_TRUE(transform->translation().isApprox(Eigen::Vector3d(1, 2, 3)));

  const auto items = objects.computeRenderItems();
  EXPECT_EQ(items.size(), 1u);
  EXPECT_EQ(items.front().id, id);
}

TEST(ObjectManager, RebuildSanitizesNonFiniteRigidBodyMass)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "box";
  body.mass = std::numeric_limits<double>::quiet_NaN();
  const ObjectId id = objects.model().add(body);

  EXPECT_NO_THROW(objects.rebuild());
  const SceneObject* sanitized = objects.model().find(id);
  ASSERT_NE(sanitized, nullptr);
  EXPECT_TRUE(std::isfinite(sanitized->mass));
  EXPECT_DOUBLE_EQ(sanitized->mass, 1e-9);

  auto rigidBody = objects.world().getRigidBody("box");
  ASSERT_TRUE(rigidBody.has_value());
  EXPECT_DOUBLE_EQ(rigidBody->getMass(), 1e-9);
}

TEST(ObjectManager, MultiBodyWithLinks)
{
  ObjectManager objects;
  SceneObject mb;
  mb.type = ObjectType::MultiBody;
  mb.name = "arm";
  const ObjectId mbId = objects.model().add(mb);

  SceneObject base;
  base.type = ObjectType::Link;
  base.name = "base";
  base.parent = mbId;
  base.multiBody = mbId;
  const ObjectId baseId = objects.model().add(base);

  SceneObject fore;
  fore.type = ObjectType::Link;
  fore.name = "forearm";
  fore.parent = baseId;
  fore.multiBody = mbId;
  fore.parentLink = baseId;
  fore.jointType = JointKind::Revolute;
  objects.model().add(fore);

  objects.rebuild();

  ASSERT_TRUE(objects.world().hasMultibody("arm"));
  auto arm = objects.world().getMultibody("arm");
  ASSERT_TRUE(arm.has_value());
  EXPECT_EQ(arm->getLinkCount(), 2u);
  EXPECT_EQ(arm->getJointCount(), 1u);
}

TEST(ObjectManager, RebuildToleratesMalformedFramesAndJointKinds)
{
  ObjectManager objects;
  SceneObject rootFixed;
  rootFixed.type = ObjectType::FixedFrame;
  rootFixed.name = "ignored_root_fixed";
  const ObjectId rootFixedId = objects.model().add(rootFixed);

  SceneObject baseFrame;
  baseFrame.type = ObjectType::FreeFrame;
  baseFrame.name = "base_frame";
  const ObjectId baseFrameId = objects.model().add(baseFrame);

  SceneObject unnamedFixed;
  unnamedFixed.type = ObjectType::FixedFrame;
  unnamedFixed.parent = baseFrameId;
  objects.model().add(unnamedFixed);

  SceneObject mb;
  mb.type = ObjectType::MultiBody;
  mb.name = "joint_kinds";
  const ObjectId mbId = objects.model().add(mb);

  SceneObject base;
  base.type = ObjectType::Link;
  base.name = "base";
  base.parent = mbId;
  base.multiBody = mbId;
  const ObjectId baseId = objects.model().add(base);

  std::vector<ObjectId> parents;
  parents.push_back(baseId);
  const std::vector<JointKind> kinds{
      JointKind::Prismatic,
      JointKind::Screw,
      JointKind::Universal,
      JointKind::Ball,
      JointKind::Planar,
      JointKind::Free};
  for (std::size_t i = 0; i < kinds.size(); ++i) {
    SceneObject link;
    link.type = ObjectType::Link;
    link.name = "link_" + std::to_string(i);
    link.parent = parents.back();
    link.multiBody = mbId;
    link.parentLink = parents.back();
    link.jointType = kinds[i];
    parents.push_back(objects.model().add(link));
  }

  EXPECT_NO_THROW(objects.rebuild());
  EXPECT_TRUE(objects.worldTransformOf(rootFixedId).has_value());
  ASSERT_TRUE(objects.worldTransformOf(baseFrameId).has_value());

  auto multibody = objects.world().getMultibody("joint_kinds");
  ASSERT_TRUE(multibody.has_value());
  EXPECT_EQ(multibody->getLinkCount(), kinds.size() + 1);
  EXPECT_EQ(multibody->getJointCount(), kinds.size());
}

//==============================================================================
// SelectionManager
//==============================================================================

TEST(SelectionManager, SelectToggleDeselect)
{
  SelectionManager selection;
  selection.select(7);
  EXPECT_TRUE(selection.isSelected(7));
  EXPECT_EQ(selection.primary(), 7u);

  selection.select(8, /*additive=*/true);
  EXPECT_EQ(selection.selected().size(), 2u);
  EXPECT_EQ(selection.primary(), 8u);

  selection.toggle(8);
  EXPECT_FALSE(selection.isSelected(8));
  EXPECT_EQ(selection.primary(), 7u);

  selection.clear();
  EXPECT_TRUE(selection.empty());

  SelectionState state;
  state.ids = {3, 4};
  state.primary = 4;
  selection.setState(state);
  EXPECT_EQ(selection.state(), state);
  EXPECT_EQ(selection.primary(), 4u);
  EXPECT_EQ(selection.selected().size(), 2u);
}

//==============================================================================
// Command stack: undo / redo
//==============================================================================

TEST(CommandManager, AddUndoRedoRestoresObjectAndSelection)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(
      commands::addRigidBody(ShapeType::Box, translation(0, 0, 1)));
  EXPECT_EQ(objects.model().size(), 1u);
  const ObjectId id = selection.primary();
  EXPECT_NE(id, kNoObject);
  EXPECT_TRUE(commands.canUndo());

  EXPECT_TRUE(commands.undo());
  EXPECT_EQ(objects.model().size(), 0u);
  EXPECT_TRUE(selection.empty());
  EXPECT_TRUE(commands.canRedo());

  EXPECT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().size(), 1u);
  EXPECT_EQ(selection.primary(), id);
  EXPECT_TRUE(objects.world().hasRigidBody(objects.model().find(id)->name));
}

TEST(CommandManager, InvalidCommandsAndLabelsAreStable)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commandManager(objects, selection);

  EXPECT_FALSE(commandManager.execute(nullptr));
  EXPECT_FALSE(commandManager.undo());
  EXPECT_FALSE(commandManager.redo());
  EXPECT_EQ(commandManager.undoCount(), 0u);
  EXPECT_EQ(commandManager.redoCount(), 0u);
  EXPECT_EQ(commandManager.historyIndex(), 0u);
  EXPECT_EQ(commandManager.currentRevision(), 0u);
  commandManager.endMacro();
  EXPECT_FALSE(commandManager.inMacro());

  EXPECT_TRUE(commandManager.execute(
      "Add Body Directly",
      [](ObjectManager& objectManager, SelectionManager& selectionManager) {
        SceneObject object;
        object.type = ObjectType::RigidBody;
        object.name = "direct";
        const ObjectId id = objectManager.model().add(std::move(object));
        objectManager.rebuild();
        selectionManager.select(id);
      }));
  const CommandManager::HistoryRevision addRevision
      = commandManager.currentRevision();
  EXPECT_NE(addRevision, 0u);
  EXPECT_EQ(commandManager.undoCount(), 1u);
  EXPECT_EQ(commandManager.redoCount(), 0u);
  EXPECT_EQ(commandManager.historyIndex(), 1u);
  EXPECT_EQ(commandManager.undoLabel(), "Add Body Directly");
  EXPECT_TRUE(commandManager.redoLabel().empty());

  ASSERT_TRUE(commandManager.undo());
  EXPECT_EQ(commandManager.currentRevision(), 0u);
  EXPECT_EQ(commandManager.undoCount(), 0u);
  EXPECT_EQ(commandManager.redoCount(), 1u);
  EXPECT_EQ(commandManager.historyIndex(), 0u);
  EXPECT_TRUE(commandManager.undoLabel().empty());
  EXPECT_EQ(commandManager.redoLabel(), "Add Body Directly");

  ASSERT_TRUE(commandManager.redo());
  EXPECT_EQ(commandManager.currentRevision(), addRevision);
  EXPECT_EQ(commandManager.undoCount(), 1u);
  EXPECT_EQ(commandManager.redoCount(), 0u);

  commandManager.clearHistory();
  EXPECT_EQ(commandManager.currentRevision(), 0u);
  EXPECT_EQ(commandManager.undoCount(), 0u);
  EXPECT_FALSE(commandManager.canRedo());
  EXPECT_TRUE(commandManager.redoLabel().empty());
}

TEST(CommandManager, EditCommandsAndAutoNaming)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addRigidBody());
  commands.execute(commands::addRigidBody());
  // Two auto-named bodies must have distinct, non-empty names.
  const auto ids = objects.model().allIds();
  ASSERT_EQ(ids.size(), 2u);
  EXPECT_NE(
      objects.model().find(ids[0])->name, objects.model().find(ids[1])->name);

  const ObjectId id = ids[1];
  commands.execute(commands::setMass(id, 5.0));
  EXPECT_DOUBLE_EQ(objects.model().find(id)->mass, 5.0);

  commands.execute(commands::rename(id, "renamed"));
  EXPECT_EQ(objects.model().find(id)->name, "renamed");

  commands.execute(commands::removeObject(id));
  EXPECT_FALSE(objects.model().contains(id));
  EXPECT_EQ(objects.model().size(), 1u);
}

TEST(CommandManager, SetTimeStepSanitizesInvalidValues)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::setTimeStep(0.02));
  EXPECT_DOUBLE_EQ(objects.model().timeStep, 0.02);
  EXPECT_DOUBLE_EQ(objects.world().getTimeStep(), 0.02);

  commands.execute(commands::setTimeStep(-0.1));
  EXPECT_DOUBLE_EQ(objects.model().timeStep, 1e-9);
  EXPECT_DOUBLE_EQ(objects.world().getTimeStep(), 1e-9);

  SimulationController controller(objects);
  controller.play();
  controller.advance(1e-6);
  EXPECT_GT(controller.frameCount(), 0u);
}

TEST(CommandManager, SetMassSanitizesInvalidValues)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addRigidBody(ShapeType::Box));
  const ObjectId id = selection.primary();
  ASSERT_NE(id, kNoObject);

  commands.execute(
      commands::setMass(id, std::numeric_limits<double>::quiet_NaN()));
  const SceneObject* object = objects.model().find(id);
  ASSERT_NE(object, nullptr);
  EXPECT_DOUBLE_EQ(object->mass, 1e-9);
  const std::string name = object->name;

  auto rigidBody = objects.world().getRigidBody(name);
  ASSERT_TRUE(rigidBody.has_value());
  EXPECT_DOUBLE_EQ(rigidBody->getMass(), 1e-9);

  commands.execute(commands::setMass(id, -5.0));
  EXPECT_DOUBLE_EQ(objects.model().find(id)->mass, 1e-9);
  rigidBody = objects.world().getRigidBody(name);
  ASSERT_TRUE(rigidBody.has_value());
  EXPECT_DOUBLE_EQ(rigidBody->getMass(), 1e-9);
}

TEST(CommandManager, AddAndEditSensorDescriptors)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  ASSERT_TRUE(commands.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "parent")));
  const ObjectId parent = selection.primary();
  ASSERT_NE(parent, kNoObject);

  ASSERT_TRUE(commands.execute(
      commands::addSensor(
          SensorKind::Range,
          parent,
          translation(0.0, 0.0, 0.5),
          "range_sensor")));
  const ObjectId sensor = selection.primary();
  ASSERT_NE(sensor, kNoObject);
  const SceneObject* sensorObject = objects.model().find(sensor);
  ASSERT_NE(sensorObject, nullptr);
  EXPECT_EQ(sensorObject->type, ObjectType::Sensor);
  EXPECT_EQ(sensorObject->parent, parent);
  EXPECT_EQ(sensorObject->name, "range_sensor");
  EXPECT_EQ(sensorObject->sensor.kind, SensorKind::Range);
  EXPECT_DOUBLE_EQ(sensorObject->sensor.range, 10.0);
  EXPECT_DOUBLE_EQ(sensorObject->sensor.fieldOfView, 60.0);
  EXPECT_DOUBLE_EQ(sensorObject->sensor.updateRate, 30.0);

  const std::optional<Eigen::Isometry3d> sensorWorld
      = objects.worldTransformOf(sensor);
  ASSERT_TRUE(sensorWorld.has_value());
  EXPECT_TRUE(
      sensorWorld->translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.5)));

  const std::vector<RenderItem> items = objects.computeRenderItems();
  const auto item = std::find_if(
      items.begin(), items.end(), [sensor](const RenderItem& renderItem) {
        return renderItem.id == sensor;
      });
  ASSERT_NE(item, items.end());
  EXPECT_EQ(item->shape, ShapeType::Box);
  EXPECT_TRUE(item->dimensions.isApprox(Eigen::Vector3d(0.28, 0.18, 0.16)));
  EXPECT_TRUE(item->color.isApprox(Eigen::Vector4d(0.2, 0.75, 0.55, 1.0)));

  const std::size_t beforeNoOp = commands.undoCount();
  EXPECT_FALSE(
      commands.execute(commands::setSensor(sensor, sensorObject->sensor)));
  EXPECT_FALSE(
      commands.execute(commands::setSensor(parent, sensorObject->sensor)));
  EXPECT_FALSE(
      commands.execute(commands::setSensor(999, sensorObject->sensor)));
  EXPECT_EQ(commands.undoCount(), beforeNoOp);

  SensorDesc edited;
  edited.kind = SensorKind::Contact;
  edited.range = -1.0;
  edited.fieldOfView = 500.0;
  edited.updateRate = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(commands.execute(commands::setSensor(sensor, edited)));
  sensorObject = objects.model().find(sensor);
  ASSERT_NE(sensorObject, nullptr);
  EXPECT_EQ(sensorObject->sensor.kind, SensorKind::Contact);
  EXPECT_DOUBLE_EQ(sensorObject->sensor.range, 10.0);
  EXPECT_DOUBLE_EQ(sensorObject->sensor.fieldOfView, 179.0);
  EXPECT_DOUBLE_EQ(sensorObject->sensor.updateRate, 30.0);
  EXPECT_TRUE(
      sensorObject->shape.color.isApprox(Eigen::Vector4d(0.9, 0.55, 0.2, 1.0)));

  ASSERT_TRUE(commands.undo());
  sensorObject = objects.model().find(sensor);
  ASSERT_NE(sensorObject, nullptr);
  EXPECT_EQ(sensorObject->sensor.kind, SensorKind::Range);
  EXPECT_TRUE(sensorObject->shape.color.isApprox(
      Eigen::Vector4d(0.2, 0.75, 0.55, 1.0)));

  ASSERT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().find(sensor)->sensor.kind, SensorKind::Contact);

  const std::size_t undoCount = commands.undoCount();
  ASSERT_FALSE(commands.execute(commands::addSensor(SensorKind::Camera, 999)));
  EXPECT_EQ(commands.undoCount(), undoCount);
  EXPECT_EQ(selection.primary(), sensor);
}

TEST(CommandManager, AddAndEditCollisionDescriptors)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  ASSERT_TRUE(commands.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "parent")));
  const ObjectId parent = selection.primary();
  ASSERT_NE(parent, kNoObject);

  ASSERT_TRUE(commands.execute(
      commands::addCollision(
          ShapeType::Sphere,
          parent,
          translation(0.0, 0.0, 0.5),
          "contact_proxy")));
  const ObjectId collision = selection.primary();
  ASSERT_NE(collision, kNoObject);
  const SceneObject* collisionObject = objects.model().find(collision);
  ASSERT_NE(collisionObject, nullptr);
  EXPECT_EQ(collisionObject->type, ObjectType::Collision);
  EXPECT_EQ(collisionObject->parent, parent);
  EXPECT_EQ(collisionObject->name, "contact_proxy");
  EXPECT_EQ(collisionObject->shape.type, ShapeType::Sphere);
  EXPECT_TRUE(collisionObject->shape.color.isApprox(
      Eigen::Vector4d(0.95, 0.35, 0.25, 0.45)));
  EXPECT_DOUBLE_EQ(collisionObject->collision.friction, 0.8);
  EXPECT_DOUBLE_EQ(collisionObject->collision.restitution, 0.0);

  const std::optional<Eigen::Isometry3d> collisionWorld
      = objects.worldTransformOf(collision);
  ASSERT_TRUE(collisionWorld.has_value());
  EXPECT_TRUE(
      collisionWorld->translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.5)));

  std::vector<RenderItem> items = objects.computeRenderItems();
  auto item = std::find_if(
      items.begin(), items.end(), [collision](const RenderItem& renderItem) {
        return renderItem.id == collision;
      });
  ASSERT_NE(item, items.end());
  EXPECT_EQ(item->shape, ShapeType::Sphere);

  const std::size_t beforeNoOp = commands.undoCount();
  EXPECT_FALSE(commands.execute(
      commands::setCollision(collision, collisionObject->collision)));
  EXPECT_FALSE(commands.execute(
      commands::setCollision(parent, collisionObject->collision)));
  EXPECT_FALSE(commands.execute(commands::setCollision(999, {})));
  EXPECT_EQ(commands.undoCount(), beforeNoOp);

  CollisionDesc edited;
  edited.friction = -1.0;
  edited.restitution = 2.0;
  ASSERT_TRUE(commands.execute(commands::setCollision(collision, edited)));
  collisionObject = objects.model().find(collision);
  ASSERT_NE(collisionObject, nullptr);
  EXPECT_DOUBLE_EQ(collisionObject->collision.friction, 0.8);
  EXPECT_DOUBLE_EQ(collisionObject->collision.restitution, 1.0);

  ShapeDesc shape = collisionObject->shape;
  shape.type = ShapeType::Capsule;
  shape.dimensions = Eigen::Vector3d(0.25, 0.75, 0.25);
  ASSERT_TRUE(commands.execute(commands::setShape(collision, shape)));
  collisionObject = objects.model().find(collision);
  ASSERT_NE(collisionObject, nullptr);
  EXPECT_EQ(collisionObject->shape.type, ShapeType::Capsule);
  items = objects.computeRenderItems();
  item = std::find_if(
      items.begin(), items.end(), [collision](const RenderItem& renderItem) {
        return renderItem.id == collision;
      });
  ASSERT_NE(item, items.end());
  EXPECT_EQ(item->shape, ShapeType::Capsule);

  ASSERT_TRUE(commands.undo());
  EXPECT_EQ(objects.model().find(collision)->shape.type, ShapeType::Sphere);
  ASSERT_TRUE(commands.undo());
  EXPECT_DOUBLE_EQ(objects.model().find(collision)->collision.restitution, 0.0);
  ASSERT_TRUE(commands.redo());
  EXPECT_DOUBLE_EQ(objects.model().find(collision)->collision.restitution, 1.0);

  const std::size_t undoCount = commands.undoCount();
  ASSERT_FALSE(commands.execute(commands::addCollision(ShapeType::Box, 999)));
  EXPECT_EQ(commands.undoCount(), undoCount);
  EXPECT_EQ(selection.primary(), collision);
}

//==============================================================================
// SimulationController: Edit/Simulation mode, step, reset
//==============================================================================

TEST(SimulationController, StepAdvancesAndResetRestores)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "ball";
  body.transform = translation(0, 0, 5);
  objects.model().add(body);
  objects.model().timeStep = 0.01;
  objects.rebuild();

  SimulationController controller(objects);
  EXPECT_EQ(controller.mode(), SimulationController::Mode::Edit);

  controller.step(10);
  EXPECT_EQ(controller.mode(), SimulationController::Mode::Simulation);
  EXPECT_EQ(controller.frameCount(), 10u);
  EXPECT_GT(controller.simTime(), 0.0);

  controller.reset();
  EXPECT_EQ(controller.mode(), SimulationController::Mode::Edit);
  EXPECT_DOUBLE_EQ(controller.simTime(), 0.0);
  EXPECT_EQ(controller.frameCount(), 0u);
  // Design pose restored.
  auto rb = objects.world().getRigidBody("ball");
  ASSERT_TRUE(rb.has_value());
  EXPECT_TRUE(
      rb->getTransform().translation().isApprox(Eigen::Vector3d(0, 0, 5)));
}

TEST(SimulationController, TogglePauseAndAdvanceValidateInputs)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "ball";
  objects.model().add(body);
  objects.model().timeStep = 0.01;
  objects.rebuild();

  SimulationController controller(objects);
  controller.advance(1.0);
  EXPECT_EQ(controller.frameCount(), 0u);

  controller.togglePause();
  EXPECT_TRUE(controller.isRunning());
  controller.advance(-1.0);
  EXPECT_EQ(controller.frameCount(), 0u);

  controller.advance(0.05);
  EXPECT_GT(controller.frameCount(), 0u);
  controller.togglePause();
  EXPECT_FALSE(controller.isRunning());

  const std::size_t frameCount = controller.frameCount();
  objects.model().timeStep = 0.0;
  controller.togglePause();
  controller.advance(1.0);
  EXPECT_EQ(controller.frameCount(), frameCount);

  controller.setRealTimeFactor(-2.0);
  EXPECT_DOUBLE_EQ(controller.realTimeFactor(), 0.0);
}

//==============================================================================
// Recorder / Player: record + scrub replay
//==============================================================================

TEST(RecorderPlayer, RecordThenSeekRestoresState)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "ball";
  body.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  objects.model().add(body);
  objects.model().timeStep = 0.01;
  objects.rebuild();

  SimulationController controller(objects);
  Recorder recorder;
  recorder.start(objects.model().timeStep);
  recorder.capture(objects.world()); // frame 0
  controller.onAfterStep = [&]() {
    recorder.capture(objects.world());
  };
  controller.step(5);

  const double recordedEndTime = recorder.recording().frames().back().time;
  EXPECT_EQ(recorder.recording().frameCount(), 6u);
  EXPECT_DOUBLE_EQ(recorder.recording().frames().front().time, 0.0);

  // Stop recording, advance the live simulation further, then scrub back.
  recorder.stop();
  controller.onAfterStep = nullptr;
  controller.step(5);
  EXPECT_GT(controller.simTime(), recordedEndTime);

  Player player;
  player.setRecording(recorder.recording());
  ASSERT_TRUE(player.seek(objects.world(), 0));
  EXPECT_DOUBLE_EQ(objects.world().getTime(), 0.0);
}

TEST(RecorderPlayer, RecordingRoundTripsThroughStream)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "ball";
  objects.model().add(body);
  objects.rebuild();

  Recorder recorder;
  recorder.start(0.01);
  recorder.capture(objects.world());
  recorder.capture(objects.world());

  std::ostringstream out(std::ios::binary);
  recorder.recording().save(out);

  Recording loaded;
  std::istringstream in(out.str(), std::ios::binary);
  ASSERT_TRUE(loaded.load(in));
  EXPECT_EQ(loaded.frameCount(), recorder.recording().frameCount());
}

//==============================================================================
// SceneIO: human-readable project round-trip
//==============================================================================

TEST(SceneIO, TextRoundTripIsStable)
{
  SceneModel model;
  model.timeStep = 0.002;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "Body With Spaces";
  body.transform = translation(1.5, -2.0, 0.25);
  body.mass = 3.0;
  const ObjectId bodyId = model.add(body);

  WorkspaceWatchPreset preset;
  preset.name = "motion";
  preset.targets = {bodyId};
  preset.chartSignals = {"simulation_time", "translation_z"};
  model.workspace.watchPresets.push_back(preset);

  SceneObject mb;
  mb.type = ObjectType::MultiBody;
  mb.name = "arm";
  const ObjectId mbId = model.add(mb);
  SceneObject link;
  link.type = ObjectType::Link;
  link.name = "base";
  link.parent = mbId;
  link.multiBody = mbId;
  model.add(link);

  SceneObject sensor;
  sensor.type = ObjectType::Sensor;
  sensor.name = "range";
  sensor.parent = bodyId;
  sensor.sensor.kind = SensorKind::Range;
  sensor.sensor.range = 12.5;
  sensor.sensor.fieldOfView = 45.0;
  sensor.sensor.updateRate = 20.0;
  sensor.shape.dimensions = Eigen::Vector3d(0.28, 0.18, 0.16);
  const ObjectId sensorId = model.add(sensor);

  SceneObject collision;
  collision.type = ObjectType::Collision;
  collision.name = "contact_proxy";
  collision.parent = bodyId;
  collision.shape.type = ShapeType::Capsule;
  collision.shape.dimensions = Eigen::Vector3d(0.25, 0.75, 0.25);
  collision.collision.friction = 0.5;
  collision.collision.restitution = 0.2;
  const ObjectId collisionId = model.add(collision);

  const std::string text = scene_io::save(model);

  SceneModel loaded;
  ASSERT_TRUE(scene_io::load(text, loaded));
  EXPECT_EQ(loaded.size(), model.size());
  EXPECT_DOUBLE_EQ(loaded.timeStep, 0.002);
  EXPECT_EQ(loaded.workspace.watchPresets, model.workspace.watchPresets);
  const SceneObject* loadedSensor = loaded.find(sensorId);
  ASSERT_NE(loadedSensor, nullptr);
  EXPECT_EQ(loadedSensor->type, ObjectType::Sensor);
  EXPECT_EQ(loadedSensor->parent, bodyId);
  EXPECT_EQ(loadedSensor->sensor.kind, SensorKind::Range);
  EXPECT_DOUBLE_EQ(loadedSensor->sensor.range, 12.5);
  EXPECT_DOUBLE_EQ(loadedSensor->sensor.fieldOfView, 45.0);
  EXPECT_DOUBLE_EQ(loadedSensor->sensor.updateRate, 20.0);
  const SceneObject* loadedCollision = loaded.find(collisionId);
  ASSERT_NE(loadedCollision, nullptr);
  EXPECT_EQ(loadedCollision->type, ObjectType::Collision);
  EXPECT_EQ(loadedCollision->parent, bodyId);
  EXPECT_EQ(loadedCollision->shape.type, ShapeType::Capsule);
  EXPECT_DOUBLE_EQ(loadedCollision->collision.friction, 0.5);
  EXPECT_DOUBLE_EQ(loadedCollision->collision.restitution, 0.2);

  // Re-saving the loaded model yields identical text (stable round-trip).
  EXPECT_EQ(scene_io::save(loaded), text);
}

TEST(SceneIO, RejectsBadHeader)
{
  SceneModel out;
  EXPECT_FALSE(scene_io::load("not a scene file", out));
}

TEST(SceneIO, RejectsUnsupportedVersion)
{
  SceneModel out;
  // A reader must reject newer format versions instead of parsing them with
  // older rules (which would silently produce corrupt state).
  EXPECT_FALSE(scene_io::load("dartsim-scene 4\ntimestep 0.001\n", out));
}

TEST(SceneIO, RejectsDuplicateObjectIds)
{
  const std::string text
      = "dartsim-scene 1\n"
        "timestep 0.001\n"
        "object\n"
        "id 7\n"
        "type 0\n"
        "parent 0\n"
        "name first\n"
        "end\n"
        "object\n"
        "id 7\n"
        "type 0\n"
        "parent 0\n"
        "name second\n"
        "end\n";

  SceneModel out;
  EXPECT_FALSE(scene_io::load(text, out));
  EXPECT_TRUE(out.empty());
}

TEST(SceneIO, RejectsMalformedNumericFields)
{
  auto expectRejects = [](const char* text) {
    SceneModel out;
    EXPECT_FALSE(scene_io::load(text, out)) << text;
    EXPECT_TRUE(out.empty());
  };

  expectRejects("dartsim-scene 1\ntimestep nope\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id abc\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "transform 1 0 0 0 0 1 0 0 0 0 not-a-number 0 0 0 0 1\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "type bad\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "type 99\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "parent bad\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "visible bad\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "multibody bad\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "parentlink bad\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "jointkind bad\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "shapetype bad\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "sensorkind bad\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "sensorkind 99\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "sensorrange nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "sensorfov nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "sensorrate nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 3\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "collisionfriction nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 3\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "collisionrestitution nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "type 6\n"
      "sensorrange -1\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "type 6\n"
      "sensorfov 0.5\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "type 6\n"
      "sensorfov 180\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "type 6\n"
      "sensorrate 0\n"
      "end\n");
  expectRejects(
      "dartsim-scene 3\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "type 7\n"
      "collisionfriction -1\n"
      "end\n");
  expectRejects(
      "dartsim-scene 3\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "type 7\n"
      "collisionrestitution 1.5\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "linvel 1 2 nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "angvel 1 2 nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "mass nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "inertia 1 0 0 0 1 0 0 0 nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "dim 1 2 nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "color 1 0 0 nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "jointaxis 1 0 nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "jointpos nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "watch-preset\n"
      "name bad preset\n"
      "target nope\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "watch-preset\n"
      "name bad preset\n"
      "signal\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "watch-preset\n"
      "name    \n"
      "signal translation_z\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "watch-preset\n"
      "name duplicate\n"
      "signal translation_z\n"
      "end\n"
      "watch-preset\n"
      "name duplicate\n"
      "signal simulation_time\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "watch-preset\n"
      "name bad signal\n"
      "signal translation z\n"
      "end\n");
}

TEST(SceneIO, RejectsUnterminatedObjectBlocks)
{
  auto expectRejects = [](const char* text) {
    SceneModel out;
    EXPECT_FALSE(scene_io::load(text, out)) << text;
    EXPECT_TRUE(out.empty());
  };

  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "object\n"
      "id 7\n"
      "object\n"
      "id 8\n"
      "end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "watch-preset\n"
      "name missing end\n");
  expectRejects(
      "dartsim-scene 1\n"
      "timestep 0.001\n"
      "watch-preset\n"
      "name nested\n"
      "object\n"
      "id 1\n"
      "end\n");
}

TEST(SceneIO, EmptyAndMalformedParentsAreHandled)
{
  SceneModel out;
  EXPECT_FALSE(scene_io::load("", out));

  const std::string text
      = "dartsim-scene 1\n"
        "timestep 0.001\n"
        "object\n"
        "id 1\n"
        "type 0\n"
        "parent 2\n"
        "name cycle-a\n"
        "end\n"
        "object\n"
        "id 2\n"
        "type 0\n"
        "parent 1\n"
        "name cycle-b\n"
        "end\n"
        "object\n"
        "id 3\n"
        "type 0\n"
        "parent 999\n"
        "name missing-parent\n"
        "end\n";

  ASSERT_TRUE(scene_io::load(text, out));
  ASSERT_EQ(out.size(), 3u);
  for (const ObjectId id : out.allIds()) {
    ASSERT_NE(out.find(id), nullptr);
    EXPECT_EQ(out.find(id)->parent, kNoObject);
  }
  EXPECT_EQ(out.rootChildren().size(), 3u);
}

TEST(SceneIO, RerootsDescriptorsWithNonFrameParents)
{
  const std::string text
      = "dartsim-scene 3\n"
        "timestep 0.001\n"
        "object\n"
        "id 1\n"
        "type 1\n"
        "name arm\n"
        "end\n"
        "object\n"
        "id 2\n"
        "type 6\n"
        "parent 1\n"
        "name sensor\n"
        "end\n"
        "object\n"
        "id 3\n"
        "type 7\n"
        "parent 1\n"
        "name collision\n"
        "end\n";

  SceneModel out;
  ASSERT_TRUE(scene_io::load(text, out));
  ASSERT_EQ(out.size(), 3u);
  const SceneObject* sensor = out.find(2);
  ASSERT_NE(sensor, nullptr);
  EXPECT_EQ(sensor->type, ObjectType::Sensor);
  EXPECT_EQ(sensor->parent, kNoObject);
  const SceneObject* collision = out.find(3);
  ASSERT_NE(collision, nullptr);
  EXPECT_EQ(collision->type, ObjectType::Collision);
  EXPECT_EQ(collision->parent, kNoObject);
}

TEST(CommandManager, DuplicateExplicitNamesAreDeduplicated)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  // Two bodies with the same caller-supplied name must be de-duplicated;
  // otherwise rebuild() calls World::addRigidBody with a duplicate and throws.
  commands.execute(
      commands::addRigidBody(ShapeType::Box, translation(0, 0, 0), "box"));
  commands.execute(
      commands::addRigidBody(ShapeType::Box, translation(1, 0, 0), "box"));

  EXPECT_EQ(objects.model().size(), 2u);
  EXPECT_TRUE(objects.world().hasRigidBody("box"));
  EXPECT_TRUE(objects.world().hasRigidBody("box 1"));
}

TEST(CommandManager, NoOpCommandPreservesRedoBranch)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addRigidBody());
  const ObjectId id = selection.primary();
  ASSERT_TRUE(commands.undo()); // creates a pending redo branch
  ASSERT_TRUE(commands.canRedo());

  // A command that changes nothing (here the target id no longer exists) must
  // not record a history entry or discard the pending redo branch.
  commands.execute(commands::rename(id, "noop"));
  EXPECT_FALSE(commands.canUndo());
  EXPECT_TRUE(commands.canRedo());

  ASSERT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().size(), 1u);
}

TEST(CommandManager, RemoveDeselectsEntireSubtree)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  commands.execute(commands::addLink(arm));
  const ObjectId link = selection.primary();
  ASSERT_NE(arm, kNoObject);
  ASSERT_NE(link, arm);

  // Select both the parent and its descendant (additive multi-selection).
  selection.select(arm);
  selection.select(link, /*additive=*/true);
  ASSERT_TRUE(selection.isSelected(arm));
  ASSERT_TRUE(selection.isSelected(link));

  commands.execute(commands::removeObject(arm));
  EXPECT_FALSE(objects.model().contains(arm));
  EXPECT_FALSE(objects.model().contains(link)); // descendant removed too
  // Neither the root nor the descendant may linger in the selection.
  EXPECT_FALSE(selection.isSelected(link));
  EXPECT_TRUE(selection.empty());
}

TEST(CommandManager, AddLinkRejectsInvalidParentLink)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  ASSERT_EQ(objects.model().size(), 1u);

  // A parent link referencing a missing id is rejected (nothing added),
  // otherwise the orphaned link is silently dropped from the rebuilt world.
  commands.execute(commands::addLink(arm, /*parentLink=*/9999));
  EXPECT_EQ(objects.model().size(), 1u);

  // A non-Link object is not a valid parent link.
  commands.execute(commands::addRigidBody());
  const ObjectId body = selection.primary();
  commands.execute(commands::addLink(arm, body));
  EXPECT_EQ(objects.model().size(), 2u); // arm + body, no link

  // A real link in the same multibody is accepted as a parent.
  commands.execute(commands::addLink(arm));
  const ObjectId rootLink = selection.primary();
  commands.execute(commands::addLink(arm, rootLink));
  EXPECT_EQ(objects.model().size(), 4u); // arm, body, rootLink, child link
}

TEST(CommandManager, AddLinkPreservesParentChain)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  commands.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = selection.primary();
  commands.execute(
      commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = selection.primary();
  commands.execute(
      commands::addLink(arm, forearm, JointKind::Revolute, "tool"));
  const ObjectId tool = selection.primary();

  ASSERT_NE(objects.model().find(base), nullptr);
  ASSERT_NE(objects.model().find(forearm), nullptr);
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(base)->parent, arm);
  EXPECT_EQ(objects.model().find(forearm)->parent, base);
  EXPECT_EQ(objects.model().find(tool)->parent, forearm);
  ASSERT_EQ(objects.model().childrenOf(base).size(), 1u);
  EXPECT_EQ(objects.model().childrenOf(base).front(), forearm);

  auto armWorld = objects.world().getMultibody("arm");
  ASSERT_TRUE(armWorld.has_value());
  EXPECT_EQ(armWorld->getLinkCount(), 3u);

  commands.execute(commands::removeObject(base));
  EXPECT_TRUE(objects.model().contains(arm));
  EXPECT_FALSE(objects.model().contains(base));
  EXPECT_FALSE(objects.model().contains(forearm));
  EXPECT_FALSE(objects.model().contains(tool));
  EXPECT_TRUE(selection.empty());

  armWorld = objects.world().getMultibody("arm");
  ASSERT_TRUE(armWorld.has_value());
  EXPECT_EQ(armWorld->getLinkCount(), 0u);
}

TEST(CommandManager, SetLinkParentReparentsWithinMultiBody)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  commands.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = selection.primary();
  commands.execute(
      commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = selection.primary();
  commands.execute(
      commands::addLink(arm, forearm, JointKind::Revolute, "tool"));
  const ObjectId tool = selection.primary();
  ASSERT_NE(objects.model().find(base), nullptr);
  ASSERT_NE(objects.model().find(forearm), nullptr);
  ASSERT_NE(objects.model().find(tool), nullptr);

  commands.execute(commands::setLinkParent(tool, base));
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->parentLink, base);
  EXPECT_EQ(objects.model().find(tool)->parent, base);
  EXPECT_EQ(selection.primary(), tool);
  EXPECT_TRUE(objects.model().childrenOf(forearm).empty());
  const auto& baseChildren = objects.model().childrenOf(base);
  EXPECT_NE(
      std::find(baseChildren.begin(), baseChildren.end(), tool),
      baseChildren.end());

  auto armWorld = objects.world().getMultibody("arm");
  ASSERT_TRUE(armWorld.has_value());
  EXPECT_EQ(armWorld->getLinkCount(), 3u);

  ASSERT_TRUE(commands.undo());
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->parentLink, forearm);
  EXPECT_EQ(objects.model().find(tool)->parent, forearm);

  ASSERT_TRUE(commands.redo());
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->parentLink, base);

  commands.execute(commands::setLinkParent(tool, kNoObject));
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->parentLink, kNoObject);
  EXPECT_EQ(objects.model().find(tool)->parent, arm);
}

TEST(CommandManager, SetLinkParentRejectsInvalidParentsAndCycles)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  commands.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = selection.primary();
  commands.execute(
      commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = selection.primary();
  commands.execute(commands::addMultiBody("other"));
  const ObjectId other = selection.primary();
  commands.execute(
      commands::addLink(other, kNoObject, JointKind::Fixed, "other_base"));
  const ObjectId otherBase = selection.primary();
  const SceneModel before = objects.model();

  commands.execute(commands::setLinkParent(base, forearm));
  EXPECT_EQ(objects.model(), before);

  commands.execute(commands::setLinkParent(forearm, otherBase));
  EXPECT_EQ(objects.model(), before);

  commands.execute(commands::setLinkParent(forearm, 99999));
  EXPECT_EQ(objects.model(), before);

  commands.execute(commands::setLinkParent(arm, base));
  EXPECT_EQ(objects.model(), before);
}

TEST(CommandManager, SetJointKindAndAxisAreUndoableAndValidated)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  commands.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = selection.primary();
  commands.execute(
      commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = selection.primary();

  commands.execute(commands::setJointAxis(forearm, Eigen::Vector3d(2, 0, 0)));
  ASSERT_NE(objects.model().find(forearm), nullptr);
  EXPECT_TRUE(objects.model().find(forearm)->jointAxis.isApprox(
      Eigen::Vector3d::UnitX()));
  EXPECT_EQ(selection.primary(), forearm);

  commands.execute(
      commands::setJointAxis(
          forearm,
          Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)));
  EXPECT_TRUE(objects.model().find(forearm)->jointAxis.isApprox(
      Eigen::Vector3d::UnitZ()));

  commands.execute(commands::setJointKind(forearm, JointKind::Ball));
  EXPECT_EQ(objects.model().find(forearm)->jointType, JointKind::Ball);
  const SceneModel ballJointModel = objects.model();

  commands.execute(commands::setJointPosition(forearm, 0.75));
  EXPECT_EQ(objects.model(), ballJointModel);

  commands.execute(commands::setJointAxis(forearm, Eigen::Vector3d::UnitY()));
  EXPECT_EQ(objects.model(), ballJointModel);

  ASSERT_TRUE(commands.undo());
  EXPECT_EQ(objects.model().find(forearm)->jointType, JointKind::Revolute);
  ASSERT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().find(forearm)->jointType, JointKind::Ball);
}

TEST(CommandManager, ReparentRejectsUnsupportedMoves)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  commands.execute(commands::addLink(arm));
  const ObjectId link = selection.primary();
  commands.execute(commands::addFreeFrame());
  const ObjectId frame = selection.primary();
  commands.execute(commands::addRigidBody());
  const ObjectId body = selection.primary();

  // A link must stay under its owning multibody; moving it elsewhere would drop
  // it from the rebuilt world, so the reparent is rejected.
  commands.execute(commands::reparent(link, frame));
  ASSERT_NE(objects.model().find(link), nullptr);
  EXPECT_EQ(objects.model().find(link)->parent, arm);

  // rebuild() instantiates only root children, so nesting a non-link object
  // under another object is rejected: the body stays at the root and keeps
  // rendering instead of silently vanishing from the world.
  commands.execute(commands::reparent(body, frame));
  EXPECT_EQ(objects.model().find(body)->parent, kNoObject);
  EXPECT_TRUE(objects.world().hasRigidBody(objects.model().find(body)->name));
}

TEST(CommandManager, FixedFrameRequiresExistingFrameParent)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addFixedFrame());
  EXPECT_TRUE(objects.model().empty());
  EXPECT_TRUE(selection.empty());

  commands.execute(commands::addFreeFrame(translation(0.0, 0.0, 1.0), "base"));
  const ObjectId base = selection.primary();
  ASSERT_NE(base, kNoObject);

  commands.execute(commands::addFixedFrame(base, translation(1.0, 0.0, 0.0)));
  const ObjectId child = selection.primary();
  ASSERT_NE(child, kNoObject);
  ASSERT_NE(objects.model().find(child), nullptr);
  EXPECT_EQ(objects.model().find(child)->type, ObjectType::FixedFrame);
  EXPECT_EQ(objects.model().find(child)->parent, base);
  EXPECT_EQ(objects.model().childrenOf(base).size(), 1u);

  ASSERT_TRUE(commands.undo());
  EXPECT_TRUE(objects.model().contains(base));
  EXPECT_FALSE(objects.model().contains(child));
}

TEST(CommandManager, AttachAndDetachFramesPreserveWorldTransform)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(2.0, 0.0, 0.0), "body"));
  const ObjectId body = selection.primary();
  commands.execute(commands::addFreeFrame(translation(5.0, 1.0, 0.0), "tool"));
  const ObjectId tool = selection.primary();
  ASSERT_NE(body, kNoObject);
  ASSERT_NE(tool, kNoObject);

  const Eigen::Isometry3d before = *objects.worldTransformOf(tool);
  commands.execute(commands::attachFrame(tool, body));
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->parent, body);
  EXPECT_TRUE(objects.model().find(tool)->transform.translation().isApprox(
      Eigen::Vector3d(3.0, 1.0, 0.0)));
  ASSERT_TRUE(objects.worldTransformOf(tool).has_value());
  EXPECT_TRUE(
      objects.worldTransformOf(tool)->matrix().isApprox(before.matrix()));
  EXPECT_EQ(selection.primary(), tool);

  ASSERT_TRUE(commands.undo());
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->parent, kNoObject);
  EXPECT_TRUE(
      objects.worldTransformOf(tool)->matrix().isApprox(before.matrix()));

  ASSERT_TRUE(commands.redo());
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->parent, body);
  EXPECT_TRUE(
      objects.worldTransformOf(tool)->matrix().isApprox(before.matrix()));

  commands.execute(commands::detachFrame(tool));
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->parent, kNoObject);
  EXPECT_EQ(objects.model().find(tool)->type, ObjectType::FreeFrame);
  EXPECT_TRUE(
      objects.worldTransformOf(tool)->matrix().isApprox(before.matrix()));
}

TEST(CommandManager, DetachFixedFrameConvertsToRootFreeFrame)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addFreeFrame(translation(0.0, 0.0, 2.0), "base"));
  const ObjectId base = selection.primary();
  commands.execute(
      commands::addFixedFrame(base, translation(1.0, 0.0, 0.5), "tool"));
  const ObjectId tool = selection.primary();
  ASSERT_NE(base, kNoObject);
  ASSERT_NE(tool, kNoObject);

  const Eigen::Isometry3d before = *objects.worldTransformOf(tool);
  commands.execute(commands::detachFrame(tool));
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->type, ObjectType::FreeFrame);
  EXPECT_EQ(objects.model().find(tool)->parent, kNoObject);
  EXPECT_TRUE(
      objects.model().find(tool)->transform.matrix().isApprox(before.matrix()));
  EXPECT_TRUE(
      objects.worldTransformOf(tool)->matrix().isApprox(before.matrix()));

  ASSERT_TRUE(commands.undo());
  ASSERT_NE(objects.model().find(tool), nullptr);
  EXPECT_EQ(objects.model().find(tool)->type, ObjectType::FixedFrame);
  EXPECT_EQ(objects.model().find(tool)->parent, base);
}

TEST(CommandManager, AttachDetachFramesRejectInvalidRelationships)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(
      commands::addFreeFrame(translation(0.0, 0.0, 0.0), "parent"));
  const ObjectId parent = selection.primary();
  commands.execute(commands::addFreeFrame(translation(1.0, 0.0, 0.0), "child"));
  const ObjectId child = selection.primary();
  commands.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(2.0, 0.0, 0.0), "body"));
  const ObjectId body = selection.primary();
  ASSERT_NE(parent, kNoObject);
  ASSERT_NE(child, kNoObject);
  ASSERT_NE(body, kNoObject);

  commands.execute(commands::attachFrame(parent, child));
  ASSERT_EQ(objects.model().find(parent)->parent, child);

  // Cycles are rejected.
  commands.execute(commands::attachFrame(child, parent));
  EXPECT_EQ(objects.model().find(child)->parent, kNoObject);

  // Rigid bodies can be parents but not attached children in this scene model.
  commands.execute(commands::attachFrame(body, parent));
  EXPECT_EQ(objects.model().find(body)->parent, kNoObject);

  commands.execute(commands::detachFrame(parent));
  EXPECT_EQ(objects.model().find(parent)->parent, kNoObject);

  const SceneModel beforeNoOp = objects.model();
  commands.execute(commands::detachFrame(parent));
  EXPECT_EQ(objects.model(), beforeNoOp);
  commands.execute(commands::attachFrame(child, 99999));
  EXPECT_EQ(objects.model(), beforeNoOp);
}

//==============================================================================
// SimEngine: end-to-end design -> run -> record -> replay + project file
//==============================================================================

TEST(SimEngine, EndToEndEditorLoop)
{
  SimEngine engine;
  int changes = 0;
  engine.setOnChanged([&]() { ++changes; });

  engine.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(0, 0, 2)));
  engine.execute(commands::addMultiBody("arm"));
  EXPECT_EQ(engine.objects().model().size(), 2u);
  EXPECT_GE(changes, 2);

  // Undo/redo through the facade.
  EXPECT_TRUE(engine.undo());
  EXPECT_EQ(engine.objects().model().size(), 1u);
  EXPECT_TRUE(engine.redo());
  EXPECT_EQ(engine.objects().model().size(), 2u);

  // Render snapshot has the rigid body (multibody has no links yet).
  EXPECT_EQ(engine.renderItems().size(), 1u);

  // Run + record, then scrub replay.
  engine.objects().model().timeStep = 0.01;
  engine.objects().rebuild();
  engine.startRecording();
  engine.simulation().step(5);
  engine.stopRecording();
  EXPECT_GE(engine.recorder().recording().frameCount(), 6u);

  engine.loadRecordingIntoPlayer();
  ASSERT_TRUE(engine.replaySeek(0));
  EXPECT_DOUBLE_EQ(engine.objects().world().getTime(), 0.0);
}

TEST(SimEngine, NoOpCommandDoesNotSignalChange)
{
  SimEngine engine;
  int changes = 0;
  engine.setOnChanged([&]() { ++changes; });

  engine.execute(commands::addRigidBody());
  ASSERT_EQ(changes, 1); // a real edit signals exactly once

  // A rejected/no-op command (removing a non-existent id) must not look like a
  // successful mutation to event-driven consumers.
  engine.execute(commands::removeObject(99999));
  EXPECT_EQ(changes, 1);
}

TEST(SimEngine, SetVisibleControlsRenderItemsAndUndoRedo)
{
  SimEngine engine;
  int changes = 0;
  EventType lastEvent = EventType::ProjectStateChanged;
  engine.setOnChanged([&]() { ++changes; });
  engine.events().subscribe(
      [&](const Event& event) { lastEvent = event.type; });

  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 1)));
  const ObjectId id = engine.selection().primary();
  ASSERT_NE(id, kNoObject);
  ASSERT_EQ(engine.renderItems().size(), 1u);
  ASSERT_EQ(changes, 1);

  engine.execute(commands::setVisible(id, false));
  ASSERT_NE(engine.objects().model().find(id), nullptr);
  EXPECT_FALSE(engine.objects().model().find(id)->visible);
  EXPECT_TRUE(engine.renderItems().empty());
  EXPECT_EQ(changes, 2);
  EXPECT_EQ(lastEvent, EventType::SceneChanged);

  ASSERT_TRUE(engine.undo());
  ASSERT_NE(engine.objects().model().find(id), nullptr);
  EXPECT_TRUE(engine.objects().model().find(id)->visible);
  EXPECT_EQ(engine.renderItems().size(), 1u);
  EXPECT_TRUE(engine.commands().canRedo());

  const int changesBeforeNoOp = changes;
  engine.execute(commands::setVisible(id, true));
  EXPECT_EQ(changes, changesBeforeNoOp);
  EXPECT_TRUE(engine.commands().canRedo());

  ASSERT_TRUE(engine.redo());
  EXPECT_FALSE(engine.objects().model().find(id)->visible);
  EXPECT_TRUE(engine.renderItems().empty());
}

TEST(SimEngine, SetVisibleParentHidesRenderableDescendants)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  ASSERT_NE(base, kNoObject);
  ASSERT_EQ(engine.renderItems().size(), 1u);
  EXPECT_EQ(engine.renderItems().front().id, base);

  engine.execute(commands::setVisible(arm, false));
  ASSERT_NE(engine.objects().model().find(arm), nullptr);
  EXPECT_FALSE(engine.objects().model().find(arm)->visible);
  EXPECT_TRUE(engine.renderItems().empty());

  ASSERT_TRUE(engine.undo());
  ASSERT_NE(engine.objects().model().find(arm), nullptr);
  EXPECT_TRUE(engine.objects().model().find(arm)->visible);
  ASSERT_EQ(engine.renderItems().size(), 1u);
  EXPECT_EQ(engine.renderItems().front().id, base);
}

TEST(SimEngine, SelectSignalsSelectionChange)
{
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box));
  const ObjectId first = engine.selection().primary();
  engine.execute(commands::addRigidBody(ShapeType::Sphere));
  const ObjectId second = engine.selection().primary();
  ASSERT_NE(first, kNoObject);
  ASSERT_NE(second, kNoObject);
  ASSERT_NE(first, second);

  int changes = 0;
  Event lastEvent;
  engine.setOnChanged([&]() { ++changes; });
  engine.events().subscribe([&](const Event& event) { lastEvent = event; });

  EXPECT_TRUE(engine.select(first));
  EXPECT_EQ(changes, 1);
  EXPECT_EQ(lastEvent.type, EventType::SelectionChanged);
  EXPECT_EQ(lastEvent.object, first);
  EXPECT_EQ(engine.selection().primary(), first);

  EXPECT_FALSE(engine.select(first));
  EXPECT_EQ(changes, 1);

  EXPECT_TRUE(engine.select(second, /*additive=*/true));
  EXPECT_EQ(changes, 2);
  EXPECT_EQ(lastEvent.type, EventType::SelectionChanged);
  EXPECT_EQ(lastEvent.object, second);
  EXPECT_EQ(engine.selection().selected().size(), 2u);
  EXPECT_EQ(engine.selection().primary(), second);

  EXPECT_TRUE(engine.toggleSelection(first));
  EXPECT_EQ(changes, 3);
  EXPECT_EQ(lastEvent.type, EventType::SelectionChanged);
  EXPECT_EQ(lastEvent.object, first);
  EXPECT_FALSE(engine.selection().isSelected(first));
  EXPECT_EQ(engine.selection().primary(), second);

  EXPECT_TRUE(engine.toggleSelection(first));
  EXPECT_EQ(changes, 4);
  EXPECT_TRUE(engine.selection().isSelected(first));
  EXPECT_EQ(engine.selection().primary(), first);

  EXPECT_TRUE(engine.deselect(first));
  EXPECT_EQ(changes, 5);
  EXPECT_FALSE(engine.selection().isSelected(first));
  EXPECT_EQ(engine.selection().primary(), second);
  EXPECT_FALSE(engine.deselect(first));
  EXPECT_EQ(changes, 5);

  EXPECT_TRUE(engine.clearSelection());
  EXPECT_EQ(changes, 6);
  EXPECT_EQ(lastEvent.type, EventType::SelectionChanged);
  EXPECT_EQ(lastEvent.object, kNoObject);
  EXPECT_TRUE(engine.selection().empty());
  EXPECT_FALSE(engine.clearSelection());
  EXPECT_EQ(changes, 6);
}

TEST(SimEngine, NoOpRemoveDoesNotResetRunState)
{
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 5)));
  engine.objects().model().timeStep = 0.01;
  engine.objects().rebuild();
  engine.simulation().step(4);

  const double simTime = engine.simulation().simTime();
  const std::size_t frameCount = engine.simulation().frameCount();
  ASSERT_GT(simTime, 0.0);
  ASSERT_GT(frameCount, 0u);

  engine.execute(commands::removeObject(99999));
  EXPECT_EQ(engine.simulation().mode(), SimulationController::Mode::Simulation);
  EXPECT_DOUBLE_EQ(engine.simulation().simTime(), simTime);
  EXPECT_EQ(engine.simulation().frameCount(), frameCount);
}

TEST(SimEngine, EditCommandsAreIgnoredInSimulationMode)
{
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 5)));
  const ObjectId id = engine.selection().primary();
  ASSERT_NE(id, kNoObject);
  ASSERT_TRUE(engine.canEditScene());

  engine.simulation().play();
  ASSERT_EQ(engine.simulation().mode(), SimulationController::Mode::Simulation);
  EXPECT_FALSE(engine.canEditScene());

  int changes = 0;
  engine.setOnChanged([&]() { ++changes; });

  engine.execute(commands::setMass(id, 5.0));
  EXPECT_DOUBLE_EQ(engine.objects().model().find(id)->mass, 1.0);
  EXPECT_EQ(changes, 0);

  EXPECT_FALSE(engine.undo());
  EXPECT_TRUE(engine.objects().model().contains(id));
  EXPECT_EQ(changes, 0);

  engine.simulation().reset();
  EXPECT_TRUE(engine.canEditScene());
  EXPECT_TRUE(engine.undo());
  EXPECT_FALSE(engine.objects().model().contains(id));
}

TEST(SimEngine, ResetConsumesSimulationSnapshotBeforeNextEdit)
{
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 5)));
  const ObjectId id = engine.selection().primary();
  ASSERT_NE(id, kNoObject);

  engine.simulation().play();
  ASSERT_EQ(engine.simulation().mode(), SimulationController::Mode::Simulation);
  ASSERT_TRUE(engine.simulation().hasCapturedEditState());

  engine.simulation().reset();
  EXPECT_EQ(engine.simulation().mode(), SimulationController::Mode::Edit);
  EXPECT_FALSE(engine.simulation().hasCapturedEditState());

  engine.execute(commands::setMass(id, 8.0));
  const SceneObject* edited = engine.objects().model().find(id);
  ASSERT_NE(edited, nullptr);
  EXPECT_DOUBLE_EQ(edited->mass, 8.0);

  engine.simulation().reset();
  const SceneObject* afterReset = engine.objects().model().find(id);
  ASSERT_NE(afterReset, nullptr);
  EXPECT_DOUBLE_EQ(afterReset->mass, 8.0);
}

TEST(SimEngine, SimulationModeChangesEmitModeChanged)
{
  SimEngine engine;
  int modeChanges = 0;
  engine.events().subscribe([&](const Event& event) {
    if (event.type == EventType::ModeChanged) {
      ++modeChanges;
    }
  });

  engine.simulation().pause();
  EXPECT_EQ(modeChanges, 0);

  engine.simulation().play();
  EXPECT_EQ(modeChanges, 1);

  engine.simulation().play();
  EXPECT_EQ(modeChanges, 1);

  engine.simulation().pause();
  EXPECT_EQ(modeChanges, 2);

  engine.simulation().reset();
  EXPECT_EQ(modeChanges, 3);

  engine.simulation().step(1);
  EXPECT_EQ(modeChanges, 4);

  engine.simulation().clearForNewScene();
  EXPECT_EQ(modeChanges, 5);
}

TEST(SimEngine, ProjectFileRoundTrip)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path() / "dartsim_engine_test.dartsim";

  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box, translation(1, 1, 1)));
  const ObjectId box = engine.selection().primary();
  engine.execute(
      commands::addSensor(SensorKind::Camera, box, translation(0, 0, 0.5)));
  const ObjectId sensor = engine.selection().primary();
  engine.execute(
      commands::addCollision(ShapeType::Sphere, box, translation(0, 0, -0.5)));
  const ObjectId collision = engine.selection().primary();
  engine.execute(commands::addMultiBody("arm"));
  WorkspaceWatchPreset preset;
  preset.name = "motion";
  preset.targets = {box};
  preset.chartSignals = {"translation_z"};
  engine.execute(commands::setWorkspaceWatchPresets({preset}));
  const std::size_t expected = engine.objects().model().size();
  ASSERT_TRUE(engine.saveProject(path.string()));

  SimEngine reloaded;
  ASSERT_TRUE(reloaded.loadProject(path.string()));
  EXPECT_EQ(reloaded.objects().model().size(), expected);
  EXPECT_EQ(reloaded.objects().model().workspace.watchPresets.size(), 1u);
  EXPECT_EQ(
      reloaded.objects().model().workspace.watchPresets[0].name, "motion");
  EXPECT_EQ(
      reloaded.objects().model().workspace.watchPresets[0].targets[0], box);
  const SceneObject* reloadedSensor = reloaded.objects().model().find(sensor);
  ASSERT_NE(reloadedSensor, nullptr);
  EXPECT_EQ(reloadedSensor->type, ObjectType::Sensor);
  EXPECT_EQ(reloadedSensor->parent, box);
  EXPECT_EQ(reloadedSensor->sensor.kind, SensorKind::Camera);
  const SceneObject* reloadedCollision
      = reloaded.objects().model().find(collision);
  ASSERT_NE(reloadedCollision, nullptr);
  EXPECT_EQ(reloadedCollision->type, ObjectType::Collision);
  EXPECT_EQ(reloadedCollision->parent, box);
  EXPECT_EQ(reloadedCollision->shape.type, ShapeType::Sphere);
  EXPECT_FALSE(reloaded.commands().canUndo()); // history cleared on load

  std::filesystem::remove(path);
}

TEST(SimEngine, WorkspaceWatchPresetCommandNormalizesInputs)
{
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  WorkspaceWatchPreset preset;
  preset.name = " motion ";
  preset.targets = {box, box, kNoObject};
  preset.chartSignals = {"translation_z", "translation_z", "custom-signal"};
  engine.execute(commands::setWorkspaceWatchPresets({preset}));

  const auto& presets = engine.objects().model().workspace.watchPresets;
  ASSERT_EQ(presets.size(), 1u);
  EXPECT_EQ(presets[0].name, "motion");
  ASSERT_EQ(presets[0].targets.size(), 1u);
  EXPECT_EQ(presets[0].targets[0], box);
  ASSERT_EQ(presets[0].chartSignals.size(), 2u);
  EXPECT_EQ(presets[0].chartSignals[0], "translation_z");
  EXPECT_EQ(presets[0].chartSignals[1], "custom-signal");

  const WorkspaceSettings before = engine.objects().model().workspace;
  const std::size_t undoCount = engine.commands().undoCount();
  WorkspaceWatchPreset blankName;
  blankName.name = "  ";
  blankName.chartSignals = {"translation_z"};
  engine.execute(commands::setWorkspaceWatchPresets({blankName}));
  EXPECT_EQ(engine.objects().model().workspace, before);
  EXPECT_EQ(engine.commands().undoCount(), undoCount);

  WorkspaceWatchPreset duplicateA;
  duplicateA.name = "duplicate";
  duplicateA.chartSignals = {"translation_z"};
  WorkspaceWatchPreset duplicateB;
  duplicateB.name = " duplicate ";
  duplicateB.chartSignals = {"simulation_time"};
  engine.execute(commands::setWorkspaceWatchPresets({duplicateA, duplicateB}));
  EXPECT_EQ(engine.objects().model().workspace, before);
  EXPECT_EQ(engine.commands().undoCount(), undoCount);

  WorkspaceWatchPreset badSignal;
  badSignal.name = "bad";
  badSignal.chartSignals = {"translation z"};
  engine.execute(commands::setWorkspaceWatchPresets({badSignal}));
  EXPECT_EQ(engine.objects().model().workspace, before);
  EXPECT_EQ(engine.commands().undoCount(), undoCount);
}

TEST(SimEngine, ProjectDirtyStateTracksSavedSceneSnapshot)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_engine_dirty_state_test.dartsim";

  SimEngine engine;
  int changes = 0;
  EventType lastEvent = EventType::SceneChanged;
  engine.setOnChanged([&]() { ++changes; });
  engine.events().subscribe(
      [&](const Event& event) { lastEvent = event.type; });

  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_FALSE(engine.hasProjectPath());
  EXPECT_FALSE(engine.saveProject());
  EXPECT_EQ(changes, 0);

  engine.execute(commands::addRigidBody(ShapeType::Box, translation(1, 1, 1)));
  EXPECT_TRUE(engine.isProjectDirty());
  EXPECT_EQ(changes, 1);

  ASSERT_TRUE(engine.saveProject(path.string()));
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_TRUE(engine.isHistoryAtCleanState());
  EXPECT_EQ(engine.cleanHistoryIndex(), engine.commands().historyIndex());
  EXPECT_EQ(engine.cleanHistoryRevision(), engine.commands().currentRevision());
  EXPECT_TRUE(engine.hasProjectPath());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_EQ(changes, 2);
  EXPECT_EQ(lastEvent, EventType::ProjectSaved);

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.isProjectDirty());

  ASSERT_TRUE(engine.redo());
  EXPECT_FALSE(engine.isProjectDirty());

  const ObjectId id = engine.selection().primary();
  ASSERT_NE(id, kNoObject);
  engine.execute(commands::setMass(id, 2.0));
  EXPECT_TRUE(engine.isProjectDirty());

  ASSERT_TRUE(engine.saveProject());
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_TRUE(engine.isHistoryAtCleanState());
  EXPECT_EQ(engine.cleanHistoryIndex(), engine.commands().historyIndex());
  EXPECT_EQ(engine.cleanHistoryRevision(), engine.commands().currentRevision());
  EXPECT_EQ(lastEvent, EventType::ProjectSaved);

  engine.execute(commands::setMass(id, 4.0));
  ASSERT_TRUE(engine.isProjectDirty());
  engine.markProjectClean();
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_EQ(lastEvent, EventType::ProjectStateChanged);

  std::filesystem::remove(path);
}

TEST(SimEngine, CleanHistoryRevisionTracksSavePointAndRedoBranches)
{
  SimEngine engine;
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_TRUE(engine.isHistoryAtCleanState());
  EXPECT_EQ(engine.cleanHistoryIndex(), 0u);
  EXPECT_EQ(engine.cleanHistoryRevision(), 0u);

  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(engine.isProjectDirty());
  ASSERT_FALSE(engine.isHistoryAtCleanState());
  ASSERT_EQ(engine.commands().historyIndex(), 1u);

  engine.markProjectClean();
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_TRUE(engine.isHistoryAtCleanState());
  EXPECT_EQ(engine.cleanHistoryIndex(), 1u);
  const CommandManager::HistoryRevision cleanRevision
      = engine.cleanHistoryRevision();
  EXPECT_EQ(cleanRevision, engine.commands().currentRevision());

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.isProjectDirty());
  EXPECT_FALSE(engine.isHistoryAtCleanState());
  EXPECT_EQ(engine.commands().historyIndex(), 0u);
  EXPECT_TRUE(engine.commands().canRedo());

  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(2.0, 0.0, 1.0), "replacement"));
  EXPECT_EQ(engine.commands().historyIndex(), engine.cleanHistoryIndex());
  EXPECT_NE(engine.commands().currentRevision(), cleanRevision);
  EXPECT_FALSE(engine.isHistoryAtCleanState());
  EXPECT_TRUE(engine.isProjectDirty());
  EXPECT_FALSE(engine.commands().canRedo());
}

TEST(SimEngine, LoadAndNewProjectResetProjectState)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_engine_project_state_reset_test.dartsim";

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(2, 0, 1)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  EventType lastEvent = EventType::SceneChanged;
  std::vector<EventType> projectReplacementEvents;
  bool dirtyDuringModeChange = false;
  engine.events().subscribe([&](const Event& event) {
    lastEvent = event.type;
    projectReplacementEvents.push_back(event.type);
    if (event.type == EventType::ModeChanged) {
      dirtyDuringModeChange = engine.isProjectDirty();
    }
  });

  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 1)));
  engine.startRecording();
  engine.simulation().step(2);
  engine.stopRecording();
  engine.loadRecordingIntoPlayer();
  ASSERT_GT(engine.player().frameCount(), 0u);
  EXPECT_TRUE(engine.isProjectDirty());

  projectReplacementEvents.clear();
  ASSERT_TRUE(engine.loadProject(path.string()));
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_TRUE(engine.player().empty());
  EXPECT_FALSE(dirtyDuringModeChange);
  EXPECT_EQ(
      std::count(
          projectReplacementEvents.begin(),
          projectReplacementEvents.end(),
          EventType::SelectionChanged),
      1);
  EXPECT_EQ(
      std::count(
          projectReplacementEvents.begin(),
          projectReplacementEvents.end(),
          EventType::ModeChanged),
      1);
  EXPECT_EQ(
      std::count(
          projectReplacementEvents.begin(),
          projectReplacementEvents.end(),
          EventType::RecordingChanged),
      1);
  EXPECT_EQ(
      std::count(
          projectReplacementEvents.begin(),
          projectReplacementEvents.end(),
          EventType::ProjectStateChanged),
      1);
  EXPECT_EQ(lastEvent, EventType::ProjectLoaded);

  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 2)));
  EXPECT_TRUE(engine.isProjectDirty());
  ASSERT_NE(engine.selection().primary(), kNoObject);
  engine.startRecording();
  engine.simulation().step(2);
  engine.stopRecording();
  engine.loadRecordingIntoPlayer();
  ASSERT_GT(engine.player().frameCount(), 0u);
  ASSERT_EQ(engine.simulation().mode(), SimulationController::Mode::Simulation);

  projectReplacementEvents.clear();
  dirtyDuringModeChange = false;
  engine.newProject();
  EXPECT_TRUE(engine.objects().model().empty());
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_FALSE(engine.hasProjectPath());
  EXPECT_FALSE(engine.commands().canUndo());
  EXPECT_TRUE(engine.selection().empty());
  EXPECT_FALSE(engine.isRecording());
  EXPECT_TRUE(engine.recorder().recording().empty());
  EXPECT_TRUE(engine.player().empty());
  EXPECT_EQ(engine.simulation().mode(), SimulationController::Mode::Edit);
  EXPECT_FALSE(dirtyDuringModeChange);
  EXPECT_EQ(
      std::count(
          projectReplacementEvents.begin(),
          projectReplacementEvents.end(),
          EventType::SelectionChanged),
      1);
  EXPECT_EQ(
      std::count(
          projectReplacementEvents.begin(),
          projectReplacementEvents.end(),
          EventType::ModeChanged),
      1);
  EXPECT_EQ(
      std::count(
          projectReplacementEvents.begin(),
          projectReplacementEvents.end(),
          EventType::RecordingChanged),
      1);
  EXPECT_EQ(
      std::count(
          projectReplacementEvents.begin(),
          projectReplacementEvents.end(),
          EventType::ProjectStateChanged),
      1);
  EXPECT_EQ(lastEvent, EventType::ProjectCreated);

  std::filesystem::remove(path);
}

TEST(SimEngine, FailedProjectSaveAndLoadPreserveCurrentState)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_engine_preserve_project_state_test.dartsim";
  const std::filesystem::path missing
      = std::filesystem::temp_directory_path()
        / "dartsim_engine_missing_project_state_test.dartsim";
  const std::filesystem::path invalidSaveDir
      = std::filesystem::temp_directory_path() / "missing-dartsim-dir";
  const std::filesystem::path loadDirectory
      = std::filesystem::temp_directory_path()
        / "dartsim_engine_load_project_directory";
  std::filesystem::remove(missing);
  std::filesystem::remove_all(invalidSaveDir);
  std::filesystem::remove_all(loadDirectory);
  ASSERT_TRUE(std::filesystem::create_directories(loadDirectory));

  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 1)));
  ASSERT_TRUE(engine.saveProject(path.string()));
  const SceneModel savedModel = engine.objects().model();

  engine.execute(commands::setMass(engine.selection().primary(), 3.0));
  ASSERT_TRUE(engine.isProjectDirty());
  const SceneModel dirtyModel = engine.objects().model();

  EXPECT_FALSE(engine.loadProject(missing.string()));
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_TRUE(engine.isProjectDirty());
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  EXPECT_FALSE(engine.loadProject(loadDirectory.string()));
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_TRUE(engine.isProjectDirty());
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  const std::filesystem::path invalidSavePath
      = invalidSaveDir / "project.dartsim";
  EXPECT_FALSE(engine.saveProject(invalidSavePath.string()));
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_TRUE(engine.isProjectDirty());
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  engine.newProject();
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_NE(engine.objects().model(), savedModel);

  std::filesystem::remove(path);
  std::filesystem::remove_all(loadDirectory);
}

TEST(SimEngine, LoadProjectClearsRunSnapshot)
{
  const std::filesystem::path path = std::filesystem::temp_directory_path()
                                     / "dartsim_engine_load_reset_test.dartsim";

  SimEngine saved;
  saved.execute(commands::addRigidBody(ShapeType::Box, translation(1, 1, 1)));
  ASSERT_TRUE(saved.saveProject(path.string()));
  const std::size_t savedSize = saved.objects().model().size();

  // Enter Simulation Mode on a different (empty) scene so an edit snapshot is
  // captured, then load the saved project over it.
  SimEngine engine;
  engine.simulation().play();
  ASSERT_TRUE(engine.loadProject(path.string()));
  EXPECT_EQ(engine.objects().model().size(), savedSize);
  EXPECT_EQ(engine.simulation().mode(), SimulationController::Mode::Edit);

  // Reset must keep the loaded scene rather than restore the pre-load snapshot.
  engine.simulation().reset();
  EXPECT_EQ(engine.objects().model().size(), savedSize);

  std::filesystem::remove(path);
}

TEST(SimEngine, LoadProjectClearsRecordingAndPlayer)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_engine_load_clears_replay.dartsim";

  // Record several frames on one scene and load them into the player.
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 5)));
  engine.objects().model().timeStep = 0.01;
  engine.objects().rebuild();
  engine.startRecording();
  engine.simulation().step(3);
  engine.stopRecording();
  engine.loadRecordingIntoPlayer();
  ASSERT_GT(engine.player().frameCount(), 0u);

  // Load a different project over it.
  SimEngine other;
  other.execute(commands::addMultiBody("arm"));
  ASSERT_TRUE(other.saveProject(path.string()));
  ASSERT_TRUE(engine.loadProject(path.string()));

  // The stale recording/replay must not survive the load; otherwise seeking it
  // would restore old-world snapshots into the freshly loaded scene.
  EXPECT_FALSE(engine.isRecording());
  EXPECT_EQ(engine.recorder().recording().frameCount(), 0u);
  EXPECT_EQ(engine.player().frameCount(), 0u);
  EXPECT_FALSE(engine.replaySeek(0));

  std::filesystem::remove(path);
}

TEST(SimEngine, ProjectReplacementIsRejectedDuringOpenMacro)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_engine_macro_project_guard.dartsim";

  SimEngine saved;
  saved.execute(commands::addMultiBody("saved"));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  engine.commands().beginMacro("Open Transaction");
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel transactionModel = engine.objects().model();
  ASSERT_EQ(transactionModel.size(), 1u);

  engine.newProject();
  EXPECT_TRUE(engine.commands().inMacro());
  EXPECT_EQ(engine.objects().model(), transactionModel);
  ASSERT_FALSE(engine.logger().entries().empty());
  EXPECT_EQ(engine.logger().entries().back().level, LogLevel::Warning);
  EXPECT_EQ(
      engine.logger().entries().back().message,
      "Cannot create a new project during an edit transaction");

  EXPECT_FALSE(engine.loadProject(path.string()));
  EXPECT_TRUE(engine.commands().inMacro());
  EXPECT_EQ(engine.objects().model(), transactionModel);
  ASSERT_FALSE(engine.logger().entries().empty());
  EXPECT_EQ(engine.logger().entries().back().level, LogLevel::Warning);
  EXPECT_EQ(
      engine.logger().entries().back().message,
      "Cannot load a project during an edit transaction");

  engine.commands().endMacro();
  EXPECT_FALSE(engine.commands().inMacro());
  EXPECT_TRUE(engine.commands().canUndo());

  std::filesystem::remove(path);
}

//==============================================================================
// Command macros (grouped, single-undo transactions)
//==============================================================================

TEST(CommandManager, MacroGroupsIntoSingleUndo)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.beginMacro("Add Two Bodies");
  EXPECT_TRUE(commands.inMacro());
  commands.execute(commands::addRigidBody());
  commands.execute(commands::addRigidBody());
  commands.endMacro();
  EXPECT_FALSE(commands.inMacro());

  EXPECT_EQ(objects.model().size(), 2u);
  EXPECT_EQ(commands.undoLabel(), "Add Two Bodies");

  // One undo reverts the whole macro.
  ASSERT_TRUE(commands.undo());
  EXPECT_EQ(objects.model().size(), 0u);
  // One redo reapplies the whole macro.
  ASSERT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().size(), 2u);
}

TEST(CommandManager, MacroRevisionsRestoreSavedUndoRedoCursor)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.beginMacro("Add Two Bodies");
  commands.execute(commands::addRigidBody());
  commands.execute(commands::addRigidBody());
  commands.endMacro();

  const CommandManager::HistoryRevision macroRevision
      = commands.currentRevision();
  ASSERT_NE(macroRevision, 0u);
  EXPECT_EQ(commands.historyIndex(), 1u);

  ASSERT_TRUE(commands.undo());
  EXPECT_EQ(commands.currentRevision(), 0u);
  EXPECT_EQ(commands.historyIndex(), 0u);
  EXPECT_EQ(commands.redoCount(), 1u);

  ASSERT_TRUE(commands.redo());
  EXPECT_EQ(commands.currentRevision(), macroRevision);
  EXPECT_EQ(commands.historyIndex(), 1u);
  EXPECT_EQ(commands.undoCount(), 1u);
}

TEST(CommandManager, EmptyMacroAddsNoHistory)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.beginMacro("Nothing");
  commands.endMacro();
  EXPECT_FALSE(commands.canUndo());
}

TEST(CommandManager, NoOpMacroPreservesRedoBranch)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addRigidBody());
  ASSERT_TRUE(commands.undo());
  ASSERT_TRUE(commands.canRedo());
  EXPECT_EQ(commands.redoCount(), 1u);

  commands.beginMacro("No-op Transaction");
  EXPECT_TRUE(
      commands.execute("Do Nothing", [](ObjectManager&, SelectionManager&) {}));
  commands.endMacro();

  EXPECT_FALSE(commands.canUndo());
  EXPECT_TRUE(commands.canRedo());
  EXPECT_EQ(commands.redoCount(), 1u);

  ASSERT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().size(), 1u);
}

TEST(CommandManager, NestedMacrosFlattenToOneEntry)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.beginMacro("Outer");
  commands.execute(commands::addRigidBody());
  commands.beginMacro("Inner");
  commands.execute(commands::addRigidBody());
  commands.endMacro();
  EXPECT_TRUE(commands.inMacro()); // outer still open
  commands.execute(commands::addRigidBody());
  commands.endMacro();

  EXPECT_EQ(objects.model().size(), 3u);
  ASSERT_TRUE(commands.undo());
  EXPECT_EQ(objects.model().size(), 0u); // all three reverted together
}

//==============================================================================
// EventBus
//==============================================================================

TEST(EventBus, SubscribeEmitUnsubscribe)
{
  EventBus bus;
  int count = 0;
  EventType last = EventType::SceneChanged;
  const int token = bus.subscribe([&](const Event& event) {
    ++count;
    last = event.type;
  });
  EXPECT_EQ(bus.listenerCount(), 1u);

  bus.emit(EventType::SelectionChanged);
  EXPECT_EQ(count, 1);
  EXPECT_EQ(last, EventType::SelectionChanged);

  bus.unsubscribe(token);
  bus.emit(EventType::SceneChanged);
  EXPECT_EQ(count, 1); // no longer notified
  EXPECT_EQ(bus.listenerCount(), 0u);
}

TEST(EventBus, UnsubscribeDuringEmitIsSafe)
{
  EventBus bus;
  int selfCount = 0;
  int otherCount = 0;
  int selfToken = 0;
  // A listener that unsubscribes itself mid-dispatch must not invalidate the
  // iteration or skip the remaining listeners.
  selfToken = bus.subscribe([&](const Event&) {
    ++selfCount;
    bus.unsubscribe(selfToken);
  });
  bus.subscribe([&](const Event&) { ++otherCount; });

  bus.emit(EventType::SceneChanged);
  EXPECT_EQ(selfCount, 1);
  EXPECT_EQ(otherCount, 1);
  EXPECT_EQ(bus.listenerCount(), 1u); // self removed during dispatch

  bus.emit(EventType::SceneChanged);
  EXPECT_EQ(selfCount, 1); // self no longer notified
  EXPECT_EQ(otherCount, 2);
}

TEST(EventBus, EmptyListenersAndMissingUnsubscribeAreIgnored)
{
  EventBus bus;
  bus.unsubscribe(42);
  EXPECT_EQ(bus.listenerCount(), 0u);

  bus.subscribe({});
  bool called = false;
  bus.subscribe([&](const Event& event) {
    called = true;
    EXPECT_EQ(event.object, 7u);
  });
  EXPECT_EQ(bus.listenerCount(), 2u);

  bus.emit(Event{EventType::SceneChanged, 7});
  EXPECT_TRUE(called);
}

//==============================================================================
// Logger
//==============================================================================

TEST(Logger, RecordsLevelsAndCapsCapacity)
{
  Logger log(3);
  log.info("a");
  log.warning("b");
  log.error("c");
  log.info("d");

  EXPECT_EQ(log.size(), 3u);                     // capped at capacity
  EXPECT_EQ(log.entries().front().message, "b"); // oldest ("a") dropped
  EXPECT_EQ(log.entries().back().message, "d");
  EXPECT_EQ(log.entries().back().level, LogLevel::Info);

  log.clear();
  EXPECT_EQ(log.size(), 0u);
}

//==============================================================================
// SimEngine wiring: events + logging
//==============================================================================

TEST(SimEngine, ExecuteEmitsEventAndLogs)
{
  SimEngine engine;
  int events = 0;
  engine.events().subscribe([&](const Event&) { ++events; });

  engine.execute(commands::addRigidBody());
  EXPECT_GE(events, 1);
  ASSERT_GE(engine.logger().size(), 1u);
  EXPECT_EQ(engine.logger().entries().back().message, "Add Rigid Body");
}

TEST(SimEngine, MacroThroughFacadeGroupsUndo)
{
  SimEngine engine;
  engine.commands().beginMacro("Seed");
  engine.execute(commands::addRigidBody());
  engine.execute(commands::addMultiBody("arm"));
  engine.commands().endMacro();

  EXPECT_EQ(engine.objects().model().size(), 2u);
  ASSERT_TRUE(engine.undo());
  EXPECT_EQ(engine.objects().model().size(), 0u); // grouped undo
}
