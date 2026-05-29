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
#include <dartsim_ui/inspector_actions.hpp>
#include <dartsim_ui/outliner_actions.hpp>

#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dartsim::ui {

namespace {

InspectorActionResult result(bool ok, std::string message)
{
  return {ok, std::move(message)};
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

bool isTransformEditable(ObjectType type)
{
  return type == ObjectType::RigidBody || type == ObjectType::FreeFrame
         || type == ObjectType::FixedFrame || type == ObjectType::Sensor
         || type == ObjectType::Collision;
}

bool isShapeEditable(ObjectType type)
{
  return type == ObjectType::RigidBody || type == ObjectType::Link
         || type == ObjectType::Collision;
}

bool hasSingleDofJoint(const SceneObject& object)
{
  if (object.type != ObjectType::Link || object.parentLink == kNoObject) {
    return false;
  }

  return object.jointType == JointKind::Revolute
         || object.jointType == JointKind::Prismatic
         || object.jointType == JointKind::Screw;
}

bool hasSingleAxisJoint(const SceneObject& object)
{
  return hasSingleDofJoint(object);
}

InspectorNumericProperty numeric(
    InspectorNumericPropertyKind kind,
    std::string section,
    std::string label,
    double value,
    double minimum,
    double maximum,
    bool editable)
{
  return {
      kind,
      std::move(label),
      std::move(section),
      value,
      minimum,
      maximum,
      editable};
}

InspectorEnumProperty enumProperty(
    InspectorEnumPropertyKind kind,
    std::string section,
    std::string label,
    int value,
    std::vector<InspectorEnumChoice> choices,
    bool editable)
{
  return {
      kind,
      std::move(label),
      std::move(section),
      value,
      std::move(choices),
      editable};
}

std::vector<InspectorEnumChoice> shapeTypeChoices()
{
  return {
      {"Box", static_cast<int>(ShapeType::Box)},
      {"Sphere", static_cast<int>(ShapeType::Sphere)},
      {"Cylinder", static_cast<int>(ShapeType::Cylinder)},
      {"Capsule", static_cast<int>(ShapeType::Capsule)},
      {"Plane", static_cast<int>(ShapeType::Plane)},
  };
}

std::vector<InspectorEnumChoice> jointKindChoices()
{
  return {
      {"Fixed", static_cast<int>(JointKind::Fixed)},
      {"Revolute", static_cast<int>(JointKind::Revolute)},
      {"Prismatic", static_cast<int>(JointKind::Prismatic)},
      {"Screw", static_cast<int>(JointKind::Screw)},
      {"Universal", static_cast<int>(JointKind::Universal)},
      {"Ball", static_cast<int>(JointKind::Ball)},
      {"Planar", static_cast<int>(JointKind::Planar)},
      {"Free", static_cast<int>(JointKind::Free)},
  };
}

std::vector<InspectorEnumChoice> sensorKindChoices()
{
  return {
      {"Camera", static_cast<int>(SensorKind::Camera)},
      {"Range", static_cast<int>(SensorKind::Range)},
      {"Contact", static_cast<int>(SensorKind::Contact)},
  };
}

std::optional<ShapeType> shapeTypeFromValue(int value)
{
  switch (static_cast<ShapeType>(value)) {
    case ShapeType::Box:
    case ShapeType::Sphere:
    case ShapeType::Cylinder:
    case ShapeType::Capsule:
    case ShapeType::Plane:
      return static_cast<ShapeType>(value);
  }
  return std::nullopt;
}

std::optional<JointKind> jointKindFromValue(int value)
{
  switch (static_cast<JointKind>(value)) {
    case JointKind::Fixed:
    case JointKind::Revolute:
    case JointKind::Prismatic:
    case JointKind::Screw:
    case JointKind::Universal:
    case JointKind::Ball:
    case JointKind::Planar:
    case JointKind::Free:
      return static_cast<JointKind>(value);
  }
  return std::nullopt;
}

std::optional<SensorKind> sensorKindFromValue(int value)
{
  switch (static_cast<SensorKind>(value)) {
    case SensorKind::Camera:
    case SensorKind::Range:
    case SensorKind::Contact:
      return static_cast<SensorKind>(value);
  }
  return std::nullopt;
}

ShapeDesc defaultShapeFor(ShapeType type, const Eigen::Vector4d& color)
{
  ShapeDesc shape;
  shape.type = type;
  shape.color = color;
  if (type == ShapeType::Plane) {
    shape.dimensions = Eigen::Vector3d::UnitZ();
  }
  return shape;
}

void appendTransformProperties(
    std::vector<InspectorNumericProperty>& properties,
    const SceneObject& object,
    bool editable)
{
  const Eigen::Vector3d position = object.transform.translation();
  properties.push_back(numeric(
      InspectorNumericPropertyKind::TranslationX,
      "Transform",
      "x",
      position.x(),
      -10.0,
      10.0,
      editable));
  properties.push_back(numeric(
      InspectorNumericPropertyKind::TranslationY,
      "Transform",
      "y",
      position.y(),
      -10.0,
      10.0,
      editable));
  properties.push_back(numeric(
      InspectorNumericPropertyKind::TranslationZ,
      "Transform",
      "z",
      position.z(),
      -10.0,
      10.0,
      editable));
}

void appendShapeProperties(
    std::vector<InspectorNumericProperty>& properties,
    const SceneObject& object,
    bool editable)
{
  const ShapeDesc& shape = object.shape;
  const double minPositive = 0.001;
  switch (shape.type) {
    case ShapeType::Box:
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionX,
          "Shape",
          "shape x",
          shape.dimensions.x(),
          minPositive,
          10.0,
          editable));
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionY,
          "Shape",
          "shape y",
          shape.dimensions.y(),
          minPositive,
          10.0,
          editable));
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionZ,
          "Shape",
          "shape z",
          shape.dimensions.z(),
          minPositive,
          10.0,
          editable));
      break;
    case ShapeType::Sphere:
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionX,
          "Shape",
          "radius",
          shape.dimensions.x(),
          minPositive,
          10.0,
          editable));
      break;
    case ShapeType::Cylinder:
    case ShapeType::Capsule:
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionX,
          "Shape",
          "radius",
          shape.dimensions.x(),
          minPositive,
          10.0,
          editable));
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionY,
          "Shape",
          "height",
          shape.dimensions.y(),
          minPositive,
          10.0,
          editable));
      break;
    case ShapeType::Plane:
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionX,
          "Shape",
          "normal x",
          shape.dimensions.x(),
          -1.0,
          1.0,
          editable));
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionY,
          "Shape",
          "normal y",
          shape.dimensions.y(),
          -1.0,
          1.0,
          editable));
      properties.push_back(numeric(
          InspectorNumericPropertyKind::ShapeDimensionZ,
          "Shape",
          "normal z",
          shape.dimensions.z(),
          -1.0,
          1.0,
          editable));
      break;
  }
}

void appendJointAxisProperties(
    std::vector<InspectorNumericProperty>& properties,
    const SceneObject& object,
    bool editable)
{
  properties.push_back(numeric(
      InspectorNumericPropertyKind::JointAxisX,
      "Joint",
      "axis x",
      object.jointAxis.x(),
      -1.0,
      1.0,
      editable));
  properties.push_back(numeric(
      InspectorNumericPropertyKind::JointAxisY,
      "Joint",
      "axis y",
      object.jointAxis.y(),
      -1.0,
      1.0,
      editable));
  properties.push_back(numeric(
      InspectorNumericPropertyKind::JointAxisZ,
      "Joint",
      "axis z",
      object.jointAxis.z(),
      -1.0,
      1.0,
      editable));
}

void appendSensorProperties(
    std::vector<InspectorNumericProperty>& properties,
    const SceneObject& object,
    bool editable)
{
  properties.push_back(numeric(
      InspectorNumericPropertyKind::SensorRange,
      "Sensor",
      "range",
      object.sensor.range,
      0.001,
      1000.0,
      editable));
  properties.push_back(numeric(
      InspectorNumericPropertyKind::SensorFieldOfView,
      "Sensor",
      "field of view",
      object.sensor.fieldOfView,
      1.0,
      179.0,
      editable));
  properties.push_back(numeric(
      InspectorNumericPropertyKind::SensorUpdateRate,
      "Sensor",
      "update rate",
      object.sensor.updateRate,
      0.001,
      1000.0,
      editable));
}

void appendCollisionProperties(
    std::vector<InspectorNumericProperty>& properties,
    const SceneObject& object,
    bool editable)
{
  properties.push_back(numeric(
      InspectorNumericPropertyKind::CollisionFriction,
      "Collision",
      "collision friction",
      object.collision.friction,
      0.0,
      10.0,
      editable));
  properties.push_back(numeric(
      InspectorNumericPropertyKind::CollisionRestitution,
      "Collision",
      "collision restitution",
      object.collision.restitution,
      0.0,
      1.0,
      editable));
}

const SceneObject* selectedObject(const SimEngine& engine)
{
  return engine.objects().model().find(engine.selection().primary());
}

InspectorStatus buildStatusForObject(
    const SimEngine& engine,
    const SceneObject& object,
    std::size_t selectionCount,
    std::string selectionSummary,
    bool canDelete)
{
  const bool editable = engine.canEditScene();
  InspectorStatus status;
  status.hasSelection = true;
  status.selectionCount = selectionCount;
  status.locked = !editable;
  status.object = object.id;
  status.name = object.name;
  status.type = objectTypeLabel(object.type);
  status.selectionSummary = std::move(selectionSummary);
  status.canDelete = canDelete && editable;

  if (isTransformEditable(object.type)) {
    appendTransformProperties(status.numericProperties, object, editable);
  }
  if (object.type == ObjectType::RigidBody) {
    status.numericProperties.push_back(numeric(
        InspectorNumericPropertyKind::Mass,
        "Physical",
        "mass",
        object.mass,
        0.01,
        100.0,
        editable));
  }
  if (hasSingleDofJoint(object)) {
    status.numericProperties.push_back(numeric(
        InspectorNumericPropertyKind::JointPosition,
        "Joint",
        "joint",
        object.jointPosition,
        -3.14159,
        3.14159,
        editable));
  }
  if (object.type == ObjectType::Link && object.parentLink != kNoObject) {
    status.enumProperties.push_back(enumProperty(
        InspectorEnumPropertyKind::JointKind,
        "Joint",
        "joint kind",
        static_cast<int>(object.jointType),
        jointKindChoices(),
        editable));
  }
  if (hasSingleAxisJoint(object)) {
    appendJointAxisProperties(status.numericProperties, object, editable);
  }
  if (object.type == ObjectType::Sensor) {
    status.enumProperties.push_back(enumProperty(
        InspectorEnumPropertyKind::SensorKind,
        "Sensor",
        "sensor kind",
        static_cast<int>(object.sensor.kind),
        sensorKindChoices(),
        editable));
    appendSensorProperties(status.numericProperties, object, editable);
  }
  if (object.type == ObjectType::Collision) {
    appendCollisionProperties(status.numericProperties, object, editable);
  }
  if (isShapeEditable(object.type)) {
    status.enumProperties.push_back(enumProperty(
        InspectorEnumPropertyKind::ShapeType,
        "Shape",
        "shape",
        static_cast<int>(object.shape.type),
        shapeTypeChoices(),
        editable));
    appendShapeProperties(status.numericProperties, object, editable);
    status.colorProperty = InspectorColorProperty{
        "color", "Material", object.shape.color, editable};
  }

  return status;
}

bool hasSelectedAncestor(
    const SceneModel& model,
    const std::unordered_set<ObjectId>& selected,
    ObjectId id)
{
  const SceneObject* object = model.find(id);
  ObjectId parent = object != nullptr ? object->parent : kNoObject;
  while (parent != kNoObject) {
    if (selected.find(parent) != selected.end()) {
      return true;
    }
    const SceneObject* parentObject = model.find(parent);
    if (parentObject == nullptr) {
      return false;
    }
    parent = parentObject->parent;
  }
  return false;
}

std::vector<ObjectId> selectedRootObjects(const SimEngine& engine)
{
  const SceneModel& model = engine.objects().model();
  const std::vector<ObjectId>& selected = engine.selection().selected();
  std::unordered_set<ObjectId> selectedIds(selected.begin(), selected.end());
  std::vector<ObjectId> roots;
  roots.reserve(selected.size());
  for (const ObjectId id : selected) {
    if (!model.contains(id) || hasSelectedAncestor(model, selectedIds, id)) {
      continue;
    }
    roots.push_back(id);
  }
  return roots;
}

void setTranslationComponent(
    Eigen::Isometry3d& transform,
    InspectorNumericPropertyKind kind,
    double value)
{
  switch (kind) {
    case InspectorNumericPropertyKind::TranslationX:
      transform.translation().x() = value;
      break;
    case InspectorNumericPropertyKind::TranslationY:
      transform.translation().y() = value;
      break;
    case InspectorNumericPropertyKind::TranslationZ:
      transform.translation().z() = value;
      break;
    default:
      break;
  }
}

void setShapeDimension(
    ShapeDesc& shape, InspectorNumericPropertyKind kind, double value)
{
  switch (kind) {
    case InspectorNumericPropertyKind::ShapeDimensionX:
      shape.dimensions.x() = value;
      break;
    case InspectorNumericPropertyKind::ShapeDimensionY:
      shape.dimensions.y() = value;
      break;
    case InspectorNumericPropertyKind::ShapeDimensionZ:
      shape.dimensions.z() = value;
      break;
    default:
      break;
  }
}

void setAxisComponent(
    Eigen::Vector3d& axis, InspectorNumericPropertyKind kind, double value)
{
  switch (kind) {
    case InspectorNumericPropertyKind::JointAxisX:
      axis.x() = value;
      break;
    case InspectorNumericPropertyKind::JointAxisY:
      axis.y() = value;
      break;
    case InspectorNumericPropertyKind::JointAxisZ:
      axis.z() = value;
      break;
    default:
      break;
  }
}

void setSensorComponent(
    SensorDesc& sensor, InspectorNumericPropertyKind kind, double value)
{
  switch (kind) {
    case InspectorNumericPropertyKind::SensorRange:
      sensor.range = value;
      break;
    case InspectorNumericPropertyKind::SensorFieldOfView:
      sensor.fieldOfView = value;
      break;
    case InspectorNumericPropertyKind::SensorUpdateRate:
      sensor.updateRate = value;
      break;
    default:
      break;
  }
}

void setCollisionComponent(
    CollisionDesc& collision, InspectorNumericPropertyKind kind, double value)
{
  switch (kind) {
    case InspectorNumericPropertyKind::CollisionFriction:
      collision.friction = value;
      break;
    case InspectorNumericPropertyKind::CollisionRestitution:
      collision.restitution = value;
      break;
    default:
      break;
  }
}

} // namespace

InspectorStatus buildInspectorStatus(const SimEngine& engine)
{
  const SceneObject* object = selectedObject(engine);
  if (object == nullptr) {
    return {};
  }

  const std::size_t selectionCount = engine.selection().selected().size();
  return buildStatusForObject(
      engine,
      *object,
      selectionCount,
      selectionCount == 1 ? "1 selected"
                          : std::to_string(selectionCount)
                                + " selected; primary: " + object->name,
      true);
}

InspectorStatus buildInspectorObjectStatus(const SimEngine& engine, ObjectId id)
{
  const SceneObject* object = engine.objects().model().find(id);
  if (object == nullptr) {
    return {};
  }

  return buildStatusForObject(engine, *object, 1, "1 object", false);
}

InspectorActionResult setInspectorNumericProperty(
    SimEngine& engine, InspectorNumericPropertyKind kind, double value)
{
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }

  const SceneObject* object = selectedObject(engine);
  if (object == nullptr) {
    return result(false, "Missing object");
  }

  switch (kind) {
    case InspectorNumericPropertyKind::TranslationX:
    case InspectorNumericPropertyKind::TranslationY:
    case InspectorNumericPropertyKind::TranslationZ: {
      if (!isTransformEditable(object->type)) {
        return result(false, "Unsupported property");
      }
      Eigen::Isometry3d transform = object->transform;
      setTranslationComponent(transform, kind, value);
      engine.execute(commands::setTransform(object->id, transform));
      return result(true, "Updated transform");
    }
    case InspectorNumericPropertyKind::Mass:
      if (object->type != ObjectType::RigidBody) {
        return result(false, "Unsupported property");
      }
      engine.execute(commands::setMass(object->id, value));
      return result(true, "Updated mass");
    case InspectorNumericPropertyKind::ShapeDimensionX:
    case InspectorNumericPropertyKind::ShapeDimensionY:
    case InspectorNumericPropertyKind::ShapeDimensionZ: {
      if (!isShapeEditable(object->type)) {
        return result(false, "Unsupported property");
      }
      ShapeDesc shape = object->shape;
      setShapeDimension(shape, kind, value);
      engine.execute(commands::setShape(object->id, shape));
      return result(true, "Updated shape");
    }
    case InspectorNumericPropertyKind::JointPosition:
      if (!hasSingleDofJoint(*object)) {
        return result(false, "Unsupported property");
      }
      engine.execute(commands::setJointPosition(object->id, value));
      return result(true, "Updated joint");
    case InspectorNumericPropertyKind::JointAxisX:
    case InspectorNumericPropertyKind::JointAxisY:
    case InspectorNumericPropertyKind::JointAxisZ: {
      if (!hasSingleAxisJoint(*object)) {
        return result(false, "Unsupported property");
      }
      Eigen::Vector3d axis = object->jointAxis;
      setAxisComponent(axis, kind, value);
      engine.execute(commands::setJointAxis(object->id, axis));
      return result(true, "Updated joint axis");
    }
    case InspectorNumericPropertyKind::SensorRange:
    case InspectorNumericPropertyKind::SensorFieldOfView:
    case InspectorNumericPropertyKind::SensorUpdateRate: {
      if (object->type != ObjectType::Sensor) {
        return result(false, "Unsupported property");
      }
      SensorDesc sensor = object->sensor;
      setSensorComponent(sensor, kind, value);
      engine.execute(commands::setSensor(object->id, sensor));
      return result(true, "Updated sensor");
    }
    case InspectorNumericPropertyKind::CollisionFriction:
    case InspectorNumericPropertyKind::CollisionRestitution: {
      if (object->type != ObjectType::Collision) {
        return result(false, "Unsupported property");
      }
      CollisionDesc collision = object->collision;
      setCollisionComponent(collision, kind, value);
      engine.execute(commands::setCollision(object->id, collision));
      return result(true, "Updated collision");
    }
  }

  return result(false, "Unsupported property");
}

InspectorActionResult setInspectorEnumProperty(
    SimEngine& engine, InspectorEnumPropertyKind kind, int value)
{
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }

  const SceneObject* object = selectedObject(engine);
  if (object == nullptr) {
    return result(false, "Missing object");
  }

  switch (kind) {
    case InspectorEnumPropertyKind::ShapeType: {
      if (!isShapeEditable(object->type)) {
        return result(false, "Unsupported property");
      }
      const std::optional<ShapeType> shapeType = shapeTypeFromValue(value);
      if (!shapeType.has_value()) {
        return result(false, "Unsupported choice");
      }
      engine.execute(
          commands::setShape(
              object->id, defaultShapeFor(*shapeType, object->shape.color)));
      return result(true, "Updated shape type");
    }
    case InspectorEnumPropertyKind::JointKind: {
      if (object->type != ObjectType::Link || object->parentLink == kNoObject) {
        return result(false, "Unsupported property");
      }
      const std::optional<JointKind> jointKind = jointKindFromValue(value);
      if (!jointKind.has_value()) {
        return result(false, "Unsupported choice");
      }
      engine.execute(commands::setJointKind(object->id, *jointKind));
      return result(true, "Updated joint kind");
    }
    case InspectorEnumPropertyKind::SensorKind: {
      if (object->type != ObjectType::Sensor) {
        return result(false, "Unsupported property");
      }
      const std::optional<SensorKind> sensorKind = sensorKindFromValue(value);
      if (!sensorKind.has_value()) {
        return result(false, "Unsupported choice");
      }
      SensorDesc sensor = object->sensor;
      sensor.kind = *sensorKind;
      engine.execute(commands::setSensor(object->id, sensor));
      return result(true, "Updated sensor kind");
    }
  }

  return result(false, "Unsupported property");
}

InspectorActionResult setInspectorShapeColor(
    SimEngine& engine, const Eigen::Vector4d& rgba)
{
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }

  const SceneObject* object = selectedObject(engine);
  if (object == nullptr) {
    return result(false, "Missing object");
  }
  if (!isShapeEditable(object->type)) {
    return result(false, "Unsupported property");
  }

  ShapeDesc shape = object->shape;
  shape.color = rgba;
  engine.execute(commands::setShape(object->id, shape));
  return result(true, "Updated color");
}

InspectorActionResult deleteInspectorSelection(SimEngine& engine)
{
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }

  const std::vector<ObjectId> roots = selectedRootObjects(engine);
  if (roots.empty()) {
    return result(false, "Missing object");
  }

  std::vector<std::string> names;
  names.reserve(roots.size());
  for (const ObjectId id : roots) {
    const SceneObject* object = engine.objects().model().find(id);
    if (object == nullptr) {
      return result(false, "Missing object");
    }
    names.push_back(object->name);
  }

  if (roots.size() == 1u) {
    engine.execute(commands::removeObject(roots.front()));
    return engine.objects().model().contains(roots.front())
               ? result(false, "Delete rejected")
               : result(true, "Deleted " + names.front());
  }

  {
    MacroScope macro(engine.commands(), "Delete Selection");
    for (const ObjectId id : roots) {
      engine.execute(commands::removeObject(id));
    }
  }

  for (const ObjectId id : roots) {
    if (engine.objects().model().contains(id)) {
      return result(false, "Delete rejected");
    }
  }
  return result(true, "Deleted " + std::to_string(roots.size()) + " objects");
}

} // namespace dartsim::ui
