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
#include <dartsim_ui/inspector_actions.hpp>
#include <dartsim_ui/palette_actions.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

const ui::InspectorNumericProperty* findNumeric(
    const ui::InspectorStatus& status, ui::InspectorNumericPropertyKind kind)
{
  const auto it = std::find_if(
      status.numericProperties.begin(),
      status.numericProperties.end(),
      [kind](const ui::InspectorNumericProperty& property) {
        return property.kind == kind;
      });
  return it == status.numericProperties.end() ? nullptr : &*it;
}

const ui::InspectorEnumProperty* findEnum(
    const ui::InspectorStatus& status, ui::InspectorEnumPropertyKind kind)
{
  const auto it = std::find_if(
      status.enumProperties.begin(),
      status.enumProperties.end(),
      [kind](const ui::InspectorEnumProperty& property) {
        return property.kind == kind;
      });
  return it == status.enumProperties.end() ? nullptr : &*it;
}

} // namespace

TEST(DartsimInspectorActions, EmptySelectionBuildsEmptyStatus)
{
  SimEngine engine;
  const ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  EXPECT_FALSE(status.hasSelection);
  EXPECT_EQ(status.selectionCount, 0u);
  EXPECT_TRUE(status.selectionSummary.empty());
  EXPECT_FALSE(status.locked);
  EXPECT_EQ(status.object, kNoObject);
  EXPECT_TRUE(status.enumProperties.empty());
  EXPECT_TRUE(status.numericProperties.empty());
  EXPECT_FALSE(status.colorProperty.has_value());
  EXPECT_FALSE(status.canDelete);

  const auto missing = ui::setInspectorNumericProperty(
      engine, ui::InspectorNumericPropertyKind::Mass, 2.0);
  EXPECT_FALSE(missing.ok);
  EXPECT_EQ(missing.message, "Missing object");
  EXPECT_FALSE(ui::deleteInspectorSelection(engine).ok);
}

TEST(DartsimInspectorActions, ObjectStatusDoesNotDependOnSelection)
{
  SimEngine engine;
  EXPECT_FALSE(ui::buildInspectorObjectStatus(engine, 42).hasSelection);

  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, translation(4.0, 5.0, 6.0), "sphere"));
  const ObjectId sphere = engine.selection().primary();
  ASSERT_NE(sphere, kNoObject);
  ASSERT_TRUE(engine.select(box));
  engine.markProjectClean();
  const std::size_t undoCount = engine.commands().undoCount();
  const auto revision = engine.commands().currentRevision();

  const ui::InspectorStatus status
      = ui::buildInspectorObjectStatus(engine, sphere);
  EXPECT_TRUE(status.hasSelection);
  EXPECT_EQ(status.selectionCount, 1u);
  EXPECT_EQ(status.selectionSummary, "1 object");
  EXPECT_FALSE(status.locked);
  EXPECT_FALSE(status.canDelete);
  EXPECT_EQ(status.object, sphere);
  EXPECT_EQ(status.name, "sphere");
  EXPECT_EQ(status.type, "RigidBody");
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::TranslationX),
      nullptr);
  EXPECT_DOUBLE_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::TranslationX)
          ->value,
      4.0);
  ASSERT_NE(
      findEnum(status, ui::InspectorEnumPropertyKind::ShapeType), nullptr);
  EXPECT_EQ(engine.selection().primary(), box);
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_EQ(engine.commands().undoCount(), undoCount);
  EXPECT_EQ(engine.commands().currentRevision(), revision);

  engine.simulation().play();
  const ui::InspectorStatus locked
      = ui::buildInspectorObjectStatus(engine, sphere);
  EXPECT_TRUE(locked.locked);
  EXPECT_FALSE(locked.canDelete);
  ASSERT_FALSE(locked.numericProperties.empty());
  EXPECT_FALSE(locked.numericProperties.front().editable);
}

TEST(DartsimInspectorActions, RigidBodyPropertiesEditThroughUndoableCommands)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(1.0, 2.0, 3.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);

  ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  EXPECT_TRUE(status.hasSelection);
  EXPECT_EQ(status.selectionCount, 1u);
  EXPECT_EQ(status.selectionSummary, "1 selected");
  EXPECT_FALSE(status.locked);
  EXPECT_EQ(status.object, box);
  EXPECT_EQ(status.name, "box");
  EXPECT_EQ(status.type, "RigidBody");
  EXPECT_TRUE(status.canDelete);
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::TranslationX),
      nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::TranslationX)
          ->section,
      "Transform");
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::Mass), nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::Mass)->section,
      "Physical");
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionX),
      nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionX)
          ->section,
      "Shape");
  ASSERT_NE(
      findEnum(status, ui::InspectorEnumPropertyKind::ShapeType), nullptr);
  EXPECT_EQ(
      findEnum(status, ui::InspectorEnumPropertyKind::ShapeType)->section,
      "Shape");
  ASSERT_TRUE(status.colorProperty.has_value());
  EXPECT_EQ(status.colorProperty->section, "Material");

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::TranslationX, 4.0)
          .ok);
  ASSERT_NE(engine.objects().model().find(box), nullptr);
  EXPECT_TRUE(
      engine.objects().model().find(box)->transform.translation().isApprox(
          Eigen::Vector3d(4.0, 2.0, 3.0)));
  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::TranslationY, 5.0)
          .ok);
  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::TranslationZ, 6.0)
          .ok);
  EXPECT_TRUE(
      engine.objects().model().find(box)->transform.translation().isApprox(
          Eigen::Vector3d(4.0, 5.0, 6.0)));

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::Mass, -5.0)
          .ok);
  EXPECT_DOUBLE_EQ(engine.objects().model().find(box)->mass, 1e-9);

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::ShapeDimensionY, 2.5)
          .ok);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(box)->shape.dimensions.y(), 2.5);
  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::ShapeDimensionZ, 3.5)
          .ok);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(box)->shape.dimensions.z(), 3.5);

  const Eigen::Vector4d invalidColor(
      -1.0, 0.25, 2.0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(ui::setInspectorShapeColor(engine, invalidColor).ok);
  EXPECT_TRUE(engine.objects().model().find(box)->shape.color.isApprox(
      Eigen::Vector4d(0.0, 0.25, 1.0, 1.0)));

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().find(box)->shape.color.isApprox(
      Eigen::Vector4d(0.8, 0.8, 0.85, 1.0)));
  ASSERT_TRUE(engine.redo());
  EXPECT_TRUE(engine.objects().model().find(box)->shape.color.isApprox(
      Eigen::Vector4d(0.0, 0.25, 1.0, 1.0)));
}

TEST(DartsimInspectorActions, LinkInspectorCoversJointAndVisualShape)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  ASSERT_NE(base, kNoObject);
  ui::InspectorStatus rootStatus = ui::buildInspectorStatus(engine);
  EXPECT_EQ(rootStatus.type, "Link");
  EXPECT_EQ(
      findNumeric(rootStatus, ui::InspectorNumericPropertyKind::JointPosition),
      nullptr);
  ASSERT_TRUE(rootStatus.colorProperty.has_value());
  EXPECT_NE(
      findEnum(rootStatus, ui::InspectorEnumPropertyKind::ShapeType), nullptr);
  EXPECT_EQ(
      findEnum(rootStatus, ui::InspectorEnumPropertyKind::JointKind), nullptr);

  engine.execute(commands::addLink(arm, base, JointKind::Revolute, "forearm"));
  const ObjectId forearm = engine.selection().primary();
  ASSERT_NE(forearm, kNoObject);
  ui::InspectorStatus childStatus = ui::buildInspectorStatus(engine);
  ASSERT_NE(
      findNumeric(childStatus, ui::InspectorNumericPropertyKind::JointPosition),
      nullptr);
  ASSERT_NE(
      findNumeric(
          childStatus, ui::InspectorNumericPropertyKind::ShapeDimensionX),
      nullptr);
  ASSERT_NE(
      findNumeric(childStatus, ui::InspectorNumericPropertyKind::JointAxisX),
      nullptr);
  ASSERT_NE(
      findEnum(childStatus, ui::InspectorEnumPropertyKind::JointKind), nullptr);
  EXPECT_EQ(
      findNumeric(childStatus, ui::InspectorNumericPropertyKind::Mass),
      nullptr);

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::JointPosition, 0.75)
          .ok);
  ASSERT_NE(engine.objects().model().find(forearm), nullptr);
  EXPECT_DOUBLE_EQ(engine.objects().model().find(forearm)->jointPosition, 0.75);

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::ShapeDimensionX, 0.45)
          .ok);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(forearm)->shape.dimensions.x(), 0.45);

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::JointAxisX, 1.0)
          .ok);
  Eigen::Vector3d expectedAxis = Eigen::Vector3d(1.0, 0.0, 1.0).normalized();
  EXPECT_TRUE(
      engine.objects().model().find(forearm)->jointAxis.isApprox(expectedAxis));
  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::JointAxisY, 1.0)
          .ok);
  expectedAxis.y() = 1.0;
  expectedAxis.normalize();
  EXPECT_TRUE(
      engine.objects().model().find(forearm)->jointAxis.isApprox(expectedAxis));
  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::JointAxisZ, 0.0)
          .ok);
  expectedAxis.z() = 0.0;
  expectedAxis.normalize();
  EXPECT_TRUE(
      engine.objects().model().find(forearm)->jointAxis.isApprox(expectedAxis));

  EXPECT_TRUE(
      ui::setInspectorEnumProperty(
          engine,
          ui::InspectorEnumPropertyKind::JointKind,
          static_cast<int>(JointKind::Ball))
          .ok);
  EXPECT_EQ(engine.objects().model().find(forearm)->jointType, JointKind::Ball);
  childStatus = ui::buildInspectorStatus(engine);
  EXPECT_EQ(
      findNumeric(childStatus, ui::InspectorNumericPropertyKind::JointPosition),
      nullptr);
  EXPECT_EQ(
      findNumeric(childStatus, ui::InspectorNumericPropertyKind::JointAxisX),
      nullptr);

  const auto unsupported = ui::setInspectorNumericProperty(
      engine, ui::InspectorNumericPropertyKind::Mass, 5.0);
  EXPECT_FALSE(unsupported.ok);
  EXPECT_EQ(unsupported.message, "Unsupported property");
}

TEST(DartsimInspectorActions, ShapeTypeEnumEditsShapeDefaults)
{
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box));
  const ObjectId body = engine.selection().primary();
  ASSERT_NE(body, kNoObject);
  ASSERT_NE(engine.objects().model().find(body), nullptr);

  ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  const ui::InspectorEnumProperty* shapeType
      = findEnum(status, ui::InspectorEnumPropertyKind::ShapeType);
  ASSERT_NE(shapeType, nullptr);
  EXPECT_EQ(shapeType->value, static_cast<int>(ShapeType::Box));
  EXPECT_EQ(shapeType->choices.size(), 5u);

  EXPECT_TRUE(
      ui::setInspectorEnumProperty(
          engine,
          ui::InspectorEnumPropertyKind::ShapeType,
          static_cast<int>(ShapeType::Sphere))
          .ok);
  ASSERT_NE(engine.objects().model().find(body), nullptr);
  EXPECT_EQ(engine.objects().model().find(body)->shape.type, ShapeType::Sphere);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(body)->shape.dimensions.x(), 1.0);

  status = ui::buildInspectorStatus(engine);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionY),
      nullptr);

  const auto invalid = ui::setInspectorEnumProperty(
      engine, ui::InspectorEnumPropertyKind::ShapeType, 999);
  EXPECT_FALSE(invalid.ok);
  EXPECT_EQ(invalid.message, "Unsupported choice");

  ASSERT_TRUE(engine.undo());
  EXPECT_EQ(engine.objects().model().find(body)->shape.type, ShapeType::Box);
}

TEST(DartsimInspectorActions, ShapePropertiesMatchPrimitiveKinds)
{
  SimEngine engine;

  engine.execute(commands::addRigidBody(ShapeType::Sphere));
  ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionX),
      nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionY),
      nullptr);

  engine.execute(commands::addRigidBody(ShapeType::Cylinder));
  status = ui::buildInspectorStatus(engine);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionX),
      nullptr);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionY),
      nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionZ),
      nullptr);

  engine.execute(commands::addRigidBody(ShapeType::Capsule));
  status = ui::buildInspectorStatus(engine);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionX),
      nullptr);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionY),
      nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionZ),
      nullptr);

  engine.execute(commands::addRigidBody(ShapeType::Plane));
  status = ui::buildInspectorStatus(engine);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionX),
      nullptr);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionY),
      nullptr);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionZ),
      nullptr);
}

TEST(
    DartsimInspectorActions,
    SensorInspectorEditsDescriptorThroughUndoableCommands)
{
  SimEngine engine;
  engine.execute(commands::addSensor(SensorKind::Camera, kNoObject));
  const ObjectId sensor = engine.selection().primary();
  ASSERT_NE(sensor, kNoObject);

  ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  EXPECT_TRUE(status.hasSelection);
  EXPECT_EQ(status.type, "Sensor");
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::TranslationX),
      nullptr);
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::SensorRange),
      nullptr);
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::SensorFieldOfView),
      nullptr);
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::SensorUpdateRate),
      nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::Mass), nullptr);
  EXPECT_FALSE(status.colorProperty.has_value());

  const ui::InspectorEnumProperty* sensorKind
      = findEnum(status, ui::InspectorEnumPropertyKind::SensorKind);
  ASSERT_NE(sensorKind, nullptr);
  EXPECT_EQ(sensorKind->label, "sensor kind");
  EXPECT_EQ(sensorKind->section, "Sensor");
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::SensorRange)
          ->section,
      "Sensor");
  ASSERT_EQ(sensorKind->choices.size(), 3u);
  EXPECT_EQ(sensorKind->choices[0].label, "Camera");
  EXPECT_EQ(sensorKind->choices[1].label, "Range");
  EXPECT_EQ(sensorKind->choices[2].label, "Contact");

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::SensorRange, -5.0)
          .ok);
  EXPECT_DOUBLE_EQ(engine.objects().model().find(sensor)->sensor.range, 10.0);

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::SensorFieldOfView, 500.0)
          .ok);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(sensor)->sensor.fieldOfView, 179.0);

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine,
          ui::InspectorNumericPropertyKind::SensorUpdateRate,
          std::numeric_limits<double>::quiet_NaN())
          .ok);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(sensor)->sensor.updateRate, 30.0);

  const auto changedKind = ui::setInspectorEnumProperty(
      engine,
      ui::InspectorEnumPropertyKind::SensorKind,
      static_cast<int>(SensorKind::Range));
  EXPECT_TRUE(changedKind.ok);
  EXPECT_EQ(changedKind.message, "Updated sensor kind");
  const SceneObject* edited = engine.objects().model().find(sensor);
  ASSERT_NE(edited, nullptr);
  EXPECT_EQ(edited->sensor.kind, SensorKind::Range);
  EXPECT_TRUE(
      edited->shape.color.isApprox(Eigen::Vector4d(0.2, 0.75, 0.55, 1.0)));

  ASSERT_TRUE(engine.undo());
  EXPECT_EQ(
      engine.objects().model().find(sensor)->sensor.kind, SensorKind::Camera);
}

TEST(
    DartsimInspectorActions,
    CollisionInspectorEditsShapeAndMaterialThroughUndoableCommands)
{
  SimEngine engine;
  engine.execute(
      commands::addCollision(
          ShapeType::Box, kNoObject, translation(1.0, 2.0, 3.0), "contact"));
  const ObjectId collision = engine.selection().primary();
  ASSERT_NE(collision, kNoObject);

  ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  EXPECT_TRUE(status.hasSelection);
  EXPECT_EQ(status.type, "Collision");
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::TranslationX),
      nullptr);
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::ShapeDimensionX),
      nullptr);
  ASSERT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::CollisionFriction),
      nullptr);
  ASSERT_NE(
      findNumeric(
          status, ui::InspectorNumericPropertyKind::CollisionRestitution),
      nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::Mass), nullptr);
  ASSERT_NE(
      findEnum(status, ui::InspectorEnumPropertyKind::ShapeType), nullptr);
  EXPECT_EQ(
      findNumeric(status, ui::InspectorNumericPropertyKind::CollisionFriction)
          ->section,
      "Collision");
  EXPECT_EQ(
      findEnum(status, ui::InspectorEnumPropertyKind::ShapeType)->section,
      "Shape");
  ASSERT_TRUE(status.colorProperty.has_value());
  EXPECT_EQ(status.colorProperty->section, "Material");

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::CollisionFriction, -5.0)
          .ok);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(collision)->collision.friction, 0.8);

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::CollisionRestitution, 2.0)
          .ok);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(collision)->collision.restitution, 1.0);

  EXPECT_TRUE(
      ui::setInspectorEnumProperty(
          engine,
          ui::InspectorEnumPropertyKind::ShapeType,
          static_cast<int>(ShapeType::Capsule))
          .ok);
  EXPECT_EQ(
      engine.objects().model().find(collision)->shape.type, ShapeType::Capsule);

  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::ShapeDimensionY, 0.75)
          .ok);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(collision)->shape.dimensions.y(), 0.75);

  const Eigen::Vector4d color(0.1, 0.2, 0.3, 0.4);
  EXPECT_TRUE(ui::setInspectorShapeColor(engine, color).ok);
  EXPECT_TRUE(
      engine.objects().model().find(collision)->shape.color.isApprox(color));

  ASSERT_TRUE(engine.undo());
  EXPECT_EQ(
      engine.objects().model().find(collision)->shape.type, ShapeType::Capsule);
  ASSERT_TRUE(engine.undo());
  EXPECT_NE(
      engine.objects().model().find(collision)->shape.dimensions.y(), 0.75);
}

TEST(DartsimInspectorActions, UnsupportedPropertiesAreRejected)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);

  auto unsupported = ui::setInspectorNumericProperty(
      engine, ui::InspectorNumericPropertyKind::Mass, 3.0);
  EXPECT_FALSE(unsupported.ok);
  EXPECT_EQ(unsupported.message, "Unsupported property");

  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  unsupported = ui::setInspectorNumericProperty(
      engine, ui::InspectorNumericPropertyKind::TranslationX, 1.0);
  EXPECT_FALSE(unsupported.ok);
  EXPECT_EQ(unsupported.message, "Unsupported property");

  engine.execute(
      commands::addLink(
          arm, engine.selection().primary(), JointKind::Screw, "screw_tool"));
  ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  EXPECT_NE(
      findNumeric(status, ui::InspectorNumericPropertyKind::JointPosition),
      nullptr);

  engine.execute(commands::addFreeFrame(translation(0.0, 0.0, 0.0), "marker"));
  unsupported = ui::setInspectorShapeColor(engine, Eigen::Vector4d::Ones());
  EXPECT_FALSE(unsupported.ok);
  EXPECT_EQ(unsupported.message, "Unsupported property");

  engine.execute(commands::addRigidBody(ShapeType::Box));
  unsupported = ui::setInspectorNumericProperty(
      engine, ui::InspectorNumericPropertyKind::JointPosition, 1.0);
  EXPECT_FALSE(unsupported.ok);
  EXPECT_EQ(unsupported.message, "Unsupported property");
}

TEST(DartsimInspectorActions, FrameInspectorEditsLocalTransformAndDeletes)
{
  SimEngine engine;
  engine.execute(commands::addFreeFrame(translation(0.0, 1.0, 2.0), "marker"));
  const ObjectId marker = engine.selection().primary();
  ASSERT_NE(marker, kNoObject);

  ui::InspectorStatus markerStatus = ui::buildInspectorStatus(engine);
  EXPECT_EQ(markerStatus.type, "FreeFrame");
  EXPECT_NE(
      findNumeric(markerStatus, ui::InspectorNumericPropertyKind::TranslationZ),
      nullptr);
  EXPECT_FALSE(markerStatus.colorProperty.has_value());

  engine.execute(
      commands::addFixedFrame(marker, translation(1.0, 0.0, 0.0), "tip"));
  const ObjectId tip = engine.selection().primary();
  ASSERT_NE(tip, kNoObject);

  ui::InspectorStatus tipStatus = ui::buildInspectorStatus(engine);
  EXPECT_EQ(tipStatus.type, "FixedFrame");
  EXPECT_TRUE(
      ui::setInspectorNumericProperty(
          engine, ui::InspectorNumericPropertyKind::TranslationX, 2.0)
          .ok);
  ASSERT_NE(engine.objects().model().find(tip), nullptr);
  EXPECT_DOUBLE_EQ(
      engine.objects().model().find(tip)->transform.translation().x(), 2.0);

  const auto deleted = ui::deleteInspectorSelection(engine);
  EXPECT_TRUE(deleted.ok);
  EXPECT_EQ(deleted.message, "Deleted tip");
  EXPECT_FALSE(engine.objects().model().contains(tip));
  EXPECT_TRUE(engine.objects().model().contains(marker));

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().contains(tip));
}

TEST(DartsimInspectorActions, MultiSelectionStatusAndDeleteUseRootSet)
{
  SimEngine engine;
  engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = engine.selection().primary();
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  engine.execute(
      commands::addRigidBody(
          ShapeType::Box, translation(0.0, 0.0, 1.0), "box"));
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(arm, kNoObject);
  ASSERT_NE(base, kNoObject);
  ASSERT_NE(box, kNoObject);

  ASSERT_TRUE(engine.select(arm));
  ASSERT_TRUE(engine.select(base, /*additive=*/true));
  ASSERT_TRUE(engine.select(box, /*additive=*/true));

  const ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  EXPECT_TRUE(status.hasSelection);
  EXPECT_EQ(status.selectionCount, 3u);
  EXPECT_EQ(status.selectionSummary, "3 selected; primary: box");
  EXPECT_EQ(status.object, box);
  EXPECT_EQ(status.name, "box");
  EXPECT_TRUE(status.canDelete);

  const auto deleted = ui::deleteInspectorSelection(engine);
  EXPECT_TRUE(deleted.ok);
  EXPECT_EQ(deleted.message, "Deleted 2 objects");
  EXPECT_FALSE(engine.objects().model().contains(arm));
  EXPECT_FALSE(engine.objects().model().contains(base));
  EXPECT_FALSE(engine.objects().model().contains(box));
  EXPECT_TRUE(engine.selection().empty());

  ASSERT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().contains(arm));
  EXPECT_TRUE(engine.objects().model().contains(base));
  EXPECT_TRUE(engine.objects().model().contains(box));
  EXPECT_TRUE(engine.selection().isSelected(arm));
  EXPECT_TRUE(engine.selection().isSelected(base));
  EXPECT_TRUE(engine.selection().isSelected(box));
  EXPECT_EQ(engine.selection().primary(), box);
}

TEST(DartsimInspectorActions, InspectorLocksEditsDuringSimulationMode)
{
  SimEngine engine;
  ASSERT_TRUE(
      ui::applyPaletteAction(engine, ui::PaletteActionKind::AddSphereRigidBody)
          .ok);
  const ObjectId sphere = engine.selection().primary();
  ASSERT_NE(sphere, kNoObject);

  engine.simulation().play();
  ASSERT_FALSE(engine.canEditScene());

  const ui::InspectorStatus status = ui::buildInspectorStatus(engine);
  EXPECT_TRUE(status.hasSelection);
  EXPECT_TRUE(status.locked);
  EXPECT_FALSE(status.canDelete);
  ASSERT_FALSE(status.numericProperties.empty());
  for (const ui::InspectorNumericProperty& property :
       status.numericProperties) {
    EXPECT_FALSE(property.editable);
  }
  ASSERT_TRUE(status.colorProperty.has_value());
  EXPECT_FALSE(status.colorProperty->editable);

  const auto numeric = ui::setInspectorNumericProperty(
      engine, ui::InspectorNumericPropertyKind::Mass, 5.0);
  EXPECT_FALSE(numeric.ok);
  EXPECT_EQ(numeric.message, "Scene locked");
  EXPECT_FALSE(ui::setInspectorShapeColor(engine, Eigen::Vector4d::Ones()).ok);
  EXPECT_FALSE(ui::deleteInspectorSelection(engine).ok);
  EXPECT_TRUE(engine.objects().model().contains(sphere));
}
