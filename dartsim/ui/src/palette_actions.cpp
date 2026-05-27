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
#include <dartsim_engine/command.hpp>
#include <dartsim_engine/commands.hpp>
#include <dartsim_ui/palette_actions.hpp>

#include <utility>

#include <cstddef>

namespace dartsim::ui {

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(x, y, z);
  return transform;
}

Eigen::Isometry3d spawnTransform(ShapeType shape)
{
  return shape == ShapeType::Plane ? translation(0.0, 0.0, 0.0)
                                   : translation(0.0, 0.0, 1.0);
}

ShapeDesc makeShape(
    ShapeType type, Eigen::Vector3d dimensions, Eigen::Vector4d color)
{
  ShapeDesc shape = defaultPaletteShape(type);
  shape.dimensions = std::move(dimensions);
  shape.color = std::move(color);
  return shape;
}

PaletteAction makeAction(
    PaletteActionKind kind,
    std::string label,
    bool enabled,
    std::string disabledReason = {})
{
  return {kind, std::move(label), enabled, std::move(disabledReason)};
}

PaletteActionResult result(
    bool ok, std::string message, ObjectId object = kNoObject)
{
  return {ok, std::move(message), object};
}

ObjectId selectedMultiBody(const SimEngine& engine)
{
  const SceneObject* selected
      = engine.objects().model().find(engine.selection().primary());
  if (selected == nullptr) {
    return kNoObject;
  }
  if (selected->type == ObjectType::MultiBody) {
    return selected->id;
  }
  return kNoObject;
}

ObjectId selectedLink(const SimEngine& engine)
{
  const SceneObject* selected
      = engine.objects().model().find(engine.selection().primary());
  return selected != nullptr && selected->type == ObjectType::Link
             ? selected->id
             : kNoObject;
}

ObjectId selectedFrameParent(const SimEngine& engine)
{
  const SceneObject* selected
      = engine.objects().model().find(engine.selection().primary());
  if (selected == nullptr) {
    return kNoObject;
  }

  const bool isFrameParent = selected->type == ObjectType::RigidBody
                             || selected->type == ObjectType::Link
                             || selected->type == ObjectType::FreeFrame
                             || selected->type == ObjectType::FixedFrame;
  return isFrameParent ? selected->id : kNoObject;
}

PaletteActionResult finishCreate(
    SimEngine& engine, std::size_t beforeSize, std::string message)
{
  const ObjectId object = engine.selection().primary();
  if (engine.objects().model().size() == beforeSize || object == kNoObject) {
    return result(false, "Create rejected");
  }
  return result(true, std::move(message), object);
}

PaletteActionResult addPrimitive(
    SimEngine& engine, ShapeType shape, std::string name)
{
  const std::size_t beforeSize = engine.objects().model().size();
  engine.execute(
      commands::addRigidBody(
          defaultPaletteShape(shape), spawnTransform(shape)));
  return finishCreate(engine, beforeSize, "Added " + name);
}

PaletteActionResult addChildLink(
    SimEngine& engine, JointKind joint, std::string message)
{
  const ObjectId parentLink = selectedLink(engine);
  if (parentLink == kNoObject) {
    return result(false, "Select a Link");
  }

  const SceneObject* parent = engine.objects().model().find(parentLink);
  if (parent == nullptr || parent->multiBody == kNoObject) {
    return result(false, "Select a Link");
  }

  const std::size_t beforeSize = engine.objects().model().size();
  engine.execute(commands::addLink(parent->multiBody, parentLink, joint));
  return finishCreate(engine, beforeSize, std::move(message));
}

PaletteActionResult addSensor(
    SimEngine& engine, SensorKind kind, std::string message)
{
  const ObjectId parent = selectedFrameParent(engine);
  const std::size_t beforeSize = engine.objects().model().size();
  engine.execute(commands::addSensor(kind, parent));
  return finishCreate(engine, beforeSize, std::move(message));
}

class MacroScope
{
public:
  MacroScope(CommandManager& commands, std::string label) : m_commands(commands)
  {
    m_commands.beginMacro(std::move(label));
  }

  MacroScope(const MacroScope&) = delete;
  MacroScope& operator=(const MacroScope&) = delete;

  ~MacroScope()
  {
    close();
  }

  void close()
  {
    if (m_open) {
      m_commands.endMacro();
      m_open = false;
    }
  }

private:
  CommandManager& m_commands;
  bool m_open = true;
};

PaletteActionResult addGroundAndBoxExample(SimEngine& engine)
{
  const std::size_t beforeSize = engine.objects().model().size();
  MacroScope macro(engine.commands(), "Add Ground And Box Example");

  engine.execute(
      commands::addRigidBody(
          makeShape(
              ShapeType::Plane,
              Eigen::Vector3d::UnitZ(),
              Eigen::Vector4d(0.45, 0.5, 0.45, 1.0)),
          translation(0.0, 0.0, 0.0),
          "ground"));
  engine.execute(
      commands::addRigidBody(
          makeShape(
              ShapeType::Box,
              Eigen::Vector3d(1.0, 1.0, 1.0),
              Eigen::Vector4d(0.2, 0.45, 0.85, 1.0)),
          translation(0.0, 0.0, 0.5),
          "box"));

  macro.close();
  return finishCreate(engine, beforeSize, "Added ground and box example");
}

PaletteActionResult addTwoLinkArmExample(SimEngine& engine)
{
  const std::size_t beforeSize = engine.objects().model().size();
  MacroScope macro(engine.commands(), "Add Two-Link Arm Example");

  engine.execute(commands::addMultiBody("two_link_arm"));
  const ObjectId arm = engine.selection().primary();
  engine.execute(commands::addLink(arm, kNoObject, JointKind::Fixed, "base"));
  const ObjectId base = engine.selection().primary();
  engine.execute(
      commands::setShape(
          base,
          makeShape(
              ShapeType::Cylinder,
              Eigen::Vector3d(0.25, 0.25, 0.25),
              Eigen::Vector4d(0.25, 0.25, 0.28, 1.0))));

  engine.execute(commands::addLink(arm, base, JointKind::Revolute, "shoulder"));
  const ObjectId shoulder = engine.selection().primary();
  engine.execute(commands::setJointAxis(shoulder, Eigen::Vector3d::UnitZ()));
  engine.execute(
      commands::setShape(
          shoulder,
          makeShape(
              ShapeType::Capsule,
              Eigen::Vector3d(0.12, 0.9, 0.12),
              Eigen::Vector4d(0.85, 0.35, 0.18, 1.0))));

  engine.execute(
      commands::addLink(arm, shoulder, JointKind::Revolute, "elbow"));
  const ObjectId elbow = engine.selection().primary();
  engine.execute(commands::setJointAxis(elbow, Eigen::Vector3d::UnitY()));
  engine.execute(
      commands::setShape(
          elbow,
          makeShape(
              ShapeType::Capsule,
              Eigen::Vector3d(0.1, 0.75, 0.1),
              Eigen::Vector4d(0.95, 0.7, 0.25, 1.0))));

  macro.close();
  return finishCreate(engine, beforeSize, "Added two-link arm example");
}

} // namespace

ShapeDesc defaultPaletteShape(ShapeType type)
{
  ShapeDesc shape;
  shape.type = type;
  switch (type) {
    case ShapeType::Box:
      shape.dimensions = Eigen::Vector3d(1.0, 1.0, 1.0);
      break;
    case ShapeType::Sphere:
      shape.dimensions = Eigen::Vector3d(0.5, 0.5, 0.5);
      break;
    case ShapeType::Cylinder:
      shape.dimensions = Eigen::Vector3d(0.35, 1.0, 0.35);
      break;
    case ShapeType::Capsule:
      shape.dimensions = Eigen::Vector3d(0.3, 1.0, 0.3);
      break;
    case ShapeType::Plane:
      shape.dimensions = Eigen::Vector3d::UnitZ();
      break;
  }
  return shape;
}

std::vector<PaletteAction> buildPaletteActions(const SimEngine& engine)
{
  const bool canEdit = engine.canEditScene();
  const bool hasMultiBody = selectedMultiBody(engine) != kNoObject;
  const bool hasLink = selectedLink(engine) != kNoObject;
  const bool hasFrameParent = selectedFrameParent(engine) != kNoObject;
  const std::string locked = canEdit ? std::string() : "Simulation Mode";

  std::vector<PaletteAction> actions;
  actions.push_back(makeAction(
      PaletteActionKind::AddBoxRigidBody, "Rigid Body / Box", canEdit, locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddSphereRigidBody,
      "Rigid Body / Sphere",
      canEdit,
      locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddCylinderRigidBody,
      "Rigid Body / Cylinder",
      canEdit,
      locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddCapsuleRigidBody,
      "Rigid Body / Capsule",
      canEdit,
      locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddPlaneRigidBody,
      "Rigid Body / Ground Plane",
      canEdit,
      locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddMultiBody, "MultiBody", canEdit, locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddRootLink,
      "Link / Root Link",
      canEdit && hasMultiBody,
      canEdit ? "Select a MultiBody" : locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddFixedLink,
      "Link / Fixed Child Link",
      canEdit && hasLink,
      canEdit ? "Select a Link" : locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddRevoluteLink,
      "Link / Revolute Child Link",
      canEdit && hasLink,
      canEdit ? "Select a Link" : locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddPrismaticLink,
      "Link / Prismatic Child Link",
      canEdit && hasLink,
      canEdit ? "Select a Link" : locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddFreeFrame, "Frame / Free Frame", canEdit, locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddFixedFrame,
      "Frame / Fixed Frame",
      canEdit && hasFrameParent,
      canEdit ? "Select a parent frame" : locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddCameraSensor, "Sensor / Camera", canEdit, locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddRangeSensor, "Sensor / Range", canEdit, locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddContactSensor,
      "Sensor / Contact",
      canEdit,
      locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddGroundAndBoxExample,
      "Example / Ground + Box",
      canEdit,
      locked));
  actions.push_back(makeAction(
      PaletteActionKind::AddTwoLinkArmExample,
      "Example / Two-Link Arm",
      canEdit,
      locked));
  return actions;
}

PaletteActionResult applyPaletteAction(
    SimEngine& engine, PaletteActionKind kind)
{
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }

  switch (kind) {
    case PaletteActionKind::AddBoxRigidBody:
      return addPrimitive(engine, ShapeType::Box, "box");
    case PaletteActionKind::AddSphereRigidBody:
      return addPrimitive(engine, ShapeType::Sphere, "sphere");
    case PaletteActionKind::AddCylinderRigidBody:
      return addPrimitive(engine, ShapeType::Cylinder, "cylinder");
    case PaletteActionKind::AddCapsuleRigidBody:
      return addPrimitive(engine, ShapeType::Capsule, "capsule");
    case PaletteActionKind::AddPlaneRigidBody:
      return addPrimitive(engine, ShapeType::Plane, "ground plane");
    case PaletteActionKind::AddMultiBody: {
      const std::size_t beforeSize = engine.objects().model().size();
      engine.execute(commands::addMultiBody());
      return finishCreate(engine, beforeSize, "Added multibody");
    }
    case PaletteActionKind::AddRootLink: {
      const ObjectId multiBody = selectedMultiBody(engine);
      if (multiBody == kNoObject) {
        return result(false, "Select a MultiBody");
      }
      const std::size_t beforeSize = engine.objects().model().size();
      engine.execute(commands::addLink(multiBody, kNoObject, JointKind::Fixed));
      return finishCreate(engine, beforeSize, "Added root link");
    }
    case PaletteActionKind::AddFixedLink:
      return addChildLink(engine, JointKind::Fixed, "Added fixed child link");
    case PaletteActionKind::AddRevoluteLink:
      return addChildLink(
          engine, JointKind::Revolute, "Added revolute child link");
    case PaletteActionKind::AddPrismaticLink:
      return addChildLink(
          engine, JointKind::Prismatic, "Added prismatic child link");
    case PaletteActionKind::AddFreeFrame: {
      const std::size_t beforeSize = engine.objects().model().size();
      engine.execute(commands::addFreeFrame());
      return finishCreate(engine, beforeSize, "Added free frame");
    }
    case PaletteActionKind::AddFixedFrame: {
      const ObjectId parentFrame = selectedFrameParent(engine);
      if (parentFrame == kNoObject) {
        return result(false, "Select a parent frame");
      }
      const std::size_t beforeSize = engine.objects().model().size();
      engine.execute(commands::addFixedFrame(parentFrame));
      return finishCreate(engine, beforeSize, "Added fixed frame");
    }
    case PaletteActionKind::AddCameraSensor:
      return addSensor(engine, SensorKind::Camera, "Added camera sensor");
    case PaletteActionKind::AddRangeSensor:
      return addSensor(engine, SensorKind::Range, "Added range sensor");
    case PaletteActionKind::AddContactSensor:
      return addSensor(engine, SensorKind::Contact, "Added contact sensor");
    case PaletteActionKind::AddGroundAndBoxExample:
      return addGroundAndBoxExample(engine);
    case PaletteActionKind::AddTwoLinkArmExample:
      return addTwoLinkArmExample(engine);
  }

  return result(false, "Unknown create action");
}

} // namespace dartsim::ui
