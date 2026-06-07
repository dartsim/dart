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

#include "dart/simulation/io/serializer.hpp"

#include "dart/simulation/comps/all.hpp"
#include "dart/simulation/compute/variational_integration.hpp"
#include "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp"
#include "dart/simulation/io/binary_io.hpp"
#include "dart/simulation/io/category_serializer.hpp"

#include <algorithm>
#include <format>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cstddef>

namespace dart::simulation::io {

//==============================================================================
// SerializerRegistry
//==============================================================================

SerializerRegistry::SerializerRegistry() = default;

namespace {

template <typename ComponentT>
void registerComponentIfNeeded(SerializerRegistry& registry)
{
  if constexpr (comps::HasComponentCategory<ComponentT>) {
    if constexpr (ComponentT::serializable) {
      if (registry.getSerializer(ComponentT::getTypeName()) != nullptr) {
        return;
      }
      registry.registerSerializer(
          std::make_unique<CategoryComponentSerializer<ComponentT>>());
    }
  }
}

void clearMeshCollisionShape(CollisionShape& shape)
{
  shape.vertices.clear();
  shape.triangles.clear();
}

void writeMeshCollisionGeometry(
    std::ostream& output, const CollisionShape& shape)
{
  std::size_t vertexCount = shape.vertices.size();
  writePOD(output, vertexCount);
  for (const auto& vertex : shape.vertices) {
    writeVector3d(output, vertex);
  }

  std::size_t triangleCount = shape.triangles.size();
  writePOD(output, triangleCount);
  for (const auto& triangle : shape.triangles) {
    writePOD(output, triangle[0]);
    writePOD(output, triangle[1]);
    writePOD(output, triangle[2]);
  }
}

void readMeshCollisionGeometry(std::istream& input, CollisionShape& shape)
{
  std::size_t vertexCount = 0;
  readPOD(input, vertexCount);
  shape.vertices.resize(vertexCount);
  for (auto& vertex : shape.vertices) {
    readVector3d(input, vertex);
  }

  std::size_t triangleCount = 0;
  readPOD(input, triangleCount);
  shape.triangles.resize(triangleCount);
  for (auto& triangle : shape.triangles) {
    readPOD(input, triangle[0]);
    readPOD(input, triangle[1]);
    readPOD(input, triangle[2]);
  }
}

void setSingleCollisionShape(
    comps::CollisionGeometry& geometry, CollisionShape shape)
{
  geometry.shapes.clear();
  geometry.shapes.push_back(std::move(shape));
}

void writeMatrix3d(std::ostream& output, const Eigen::Matrix3d& matrix)
{
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index col = 0; col < 3; ++col) {
      writePOD(output, matrix(row, col));
    }
  }
}

void readMatrix3d(std::istream& input, Eigen::Matrix3d& matrix)
{
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index col = 0; col < 3; ++col) {
      readPOD(input, matrix(row, col));
    }
  }
}

void writeQuaterniond(std::ostream& output, const Eigen::Quaterniond& q)
{
  writePOD(output, q.w());
  writePOD(output, q.x());
  writePOD(output, q.y());
  writePOD(output, q.z());
}

void readQuaterniond(std::istream& input, Eigen::Quaterniond& q)
{
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  readPOD(input, w);
  readPOD(input, x);
  readPOD(input, y);
  readPOD(input, z);
  q = Eigen::Quaterniond(w, x, y, z);
}

void initializeCollisionShapeDefaults(CollisionShape& shape)
{
  shape.localTransform = Eigen::Isometry3d::Identity();
  shape.normal = Eigen::Vector3d::UnitZ();
  shape.offset = 0.0;
  clearMeshCollisionShape(shape);
}

void deserializeCollisionGeometryPreV7(
    std::istream& input, comps::CollisionGeometry& geometry)
{
  CollisionShape shape;
  readPOD(input, shape.type);
  readPOD(input, shape.radius);
  readVector3d(input, shape.halfExtents);
  initializeCollisionShapeDefaults(shape);
  setSingleCollisionShape(geometry, std::move(shape));
}

void deserializeCollisionGeometryV7ToV10(
    std::istream& input, comps::CollisionGeometry& geometry)
{
  CollisionShape shape;
  readPOD(input, shape.type);
  readPOD(input, shape.radius);
  readVector3d(input, shape.halfExtents);
  shape.localTransform = Eigen::Isometry3d::Identity();
  shape.normal = Eigen::Vector3d::UnitZ();
  shape.offset = 0.0;
  readMeshCollisionGeometry(input, shape);
  setSingleCollisionShape(geometry, std::move(shape));
}

void serializeCollisionShape(std::ostream& output, const CollisionShape& shape)
{
  writePOD(output, shape.type);
  writePOD(output, shape.radius);
  writeVector3d(output, shape.halfExtents);
  writeIsometry3d(output, shape.localTransform);
  writeVector3d(output, shape.normal);
  writePOD(output, shape.offset);
  writeMeshCollisionGeometry(output, shape);
}

void deserializeCollisionShape(std::istream& input, CollisionShape& shape)
{
  readPOD(input, shape.type);
  readPOD(input, shape.radius);
  readVector3d(input, shape.halfExtents);
  readIsometry3d(input, shape.localTransform);
  readVector3d(input, shape.normal);
  readPOD(input, shape.offset);
  readMeshCollisionGeometry(input, shape);
}

CollisionShapeType deserializeBranchV11CollisionShapeType(std::istream& input)
{
  std::uint32_t rawType = 0;
  readPOD(input, rawType);
  switch (rawType) {
    case 0u:
      return CollisionShapeType::Sphere;
    case 1u:
      return CollisionShapeType::Box;
    case 2u:
      return CollisionShapeType::Capsule;
    case 3u:
      return CollisionShapeType::Cylinder;
    case 4u:
      return CollisionShapeType::Plane;
    case 5u:
      return CollisionShapeType::Mesh;
    default:
      throw std::runtime_error(
          std::format("Unsupported legacy CollisionShapeType {}", rawType));
  }
}

void deserializeBranchV11CollisionShape(
    std::istream& input, CollisionShape& shape)
{
  shape.type = deserializeBranchV11CollisionShapeType(input);
  readPOD(input, shape.radius);
  double legacyHeight = 1.0;
  readPOD(input, legacyHeight);
  readVector3d(input, shape.halfExtents);
  readIsometry3d(input, shape.localTransform);
  readVector3d(input, shape.normal);
  readPOD(input, shape.offset);
  readMeshCollisionGeometry(input, shape);

  if (shape.type == CollisionShapeType::Capsule
      || shape.type == CollisionShapeType::Cylinder) {
    shape.halfExtents
        = Eigen::Vector3d(shape.radius, shape.radius, 0.5 * legacyHeight);
  }
}

void serializeCollisionGeometry(
    std::ostream& output, const comps::CollisionGeometry& geometry)
{
  std::size_t shapeCount = geometry.shapes.size();
  writePOD(output, shapeCount);
  for (const auto& shape : geometry.shapes) {
    serializeCollisionShape(output, shape);
  }
}

void deserializeCollisionGeometry(
    std::istream& input, comps::CollisionGeometry& geometry)
{
  std::size_t shapeCount = 0;
  readPOD(input, shapeCount);
  geometry.shapes.resize(shapeCount);
  for (auto& shape : geometry.shapes) {
    deserializeCollisionShape(input, shape);
  }
  geometry.revision = 0;
}

void deserializeBranchV11CollisionGeometry(
    std::istream& input, comps::CollisionGeometry& geometry)
{
  std::size_t shapeCount = 0;
  readPOD(input, shapeCount);
  geometry.shapes.resize(shapeCount);
  for (auto& shape : geometry.shapes) {
    deserializeBranchV11CollisionShape(input, shape);
  }
  geometry.revision = 0;
}

void deserializeLegacyMassProperties(
    std::istream& input, comps::MassProperties& mass)
{
  readPOD(input, mass.mass);
  for (int i = 0; i < 3; ++i) {
    for (int j = i; j < 3; ++j) {
      readPOD(input, mass.inertia(i, j));
      if (i != j) {
        mass.inertia(j, i) = mass.inertia(i, j);
      }
    }
  }
  readVector3d(input, mass.localCenterOfMass);
}

void deserializeLinkV8(
    std::istream& input, comps::Link& link, std::uint32_t formatVersion)
{
  readString(input, link.name);
  deserializeLegacyMassProperties(input, link.mass);

  link.transformFromParentToJoint = Eigen::Isometry3d::Identity();
  readIsometry3d(input, link.transformFromParentJoint);

  std::uint32_t parentJoint = static_cast<std::uint32_t>(entt::null);
  readPOD(input, parentJoint);
  link.parentJoint = static_cast<entt::entity>(parentJoint);

  std::size_t childJointCount = 0;
  readPOD(input, childJointCount);
  link.childJoints.resize(childJointCount);
  for (entt::entity& childJoint : link.childJoints) {
    std::uint32_t entityId = static_cast<std::uint32_t>(entt::null);
    readPOD(input, entityId);
    childJoint = static_cast<entt::entity>(entityId);
  }

  readIsometry3d(input, link.worldTransform);

  link.externalForce.setZero();
  if (formatVersion >= 5u) {
    for (Eigen::Index i = 0; i < link.externalForce.size(); ++i) {
      readPOD(input, link.externalForce[i]);
    }
  }
}

class AvbdRigidWorldPointJointConfigComponentSerializer final
  : public TypedComponentSerializer<::dart::simulation::detail::deformable_vbd::
                                        AvbdRigidWorldPointJointConfig>
{
public:
  using Component = ::dart::simulation::detail::deformable_vbd::
      AvbdRigidWorldPointJointConfig;

  [[nodiscard]] std::string_view getTypeName() const override
  {
    return "dart::simulation::detail::deformable_vbd::"
           "AvbdRigidWorldPointJointConfig";
  }

protected:
  void saveComponent(
      std::ostream& output,
      const Component& config,
      const EntityMap& entityMap) const override
  {
    (void)entityMap;
    writePOD(output, config.enabled);
    writeVector3d(output, config.localAnchorA);
    writeVector3d(output, config.localAnchorB);
    writeQuaterniond(output, config.targetRelativeOrientation);
    writeMatrix3d(output, config.linearAxes);
    writeMatrix3d(output, config.angularAxes);
    writePOD(output, config.linearAxisMask);
    writePOD(output, config.angularAxisMask);
    writePOD(output, config.startStiffness);
    writePOD(output, config.linearMaterialStiffness);
    writePOD(output, config.angularMaterialStiffness);
    writePOD(output, config.maxStiffness);
  }

  void loadComponent(std::istream& input, Component& config) const override
  {
    readPOD(input, config.enabled);
    readVector3d(input, config.localAnchorA);
    readVector3d(input, config.localAnchorB);
    readQuaterniond(input, config.targetRelativeOrientation);
    readMatrix3d(input, config.linearAxes);
    readMatrix3d(input, config.angularAxes);
    readPOD(input, config.linearAxisMask);
    readPOD(input, config.angularAxisMask);
    readPOD(input, config.startStiffness);
    readPOD(input, config.linearMaterialStiffness);
    readPOD(input, config.angularMaterialStiffness);
    readPOD(input, config.maxStiffness);
  }
};

void registerAvbdRigidWorldPointJointConfigSerializer(
    SerializerRegistry& registry)
{
  AvbdRigidWorldPointJointConfigComponentSerializer serializerProbe;
  if (registry.getSerializer(serializerProbe.getTypeName()) != nullptr) {
    return;
  }

  registry.registerSerializer(
      std::make_unique<AvbdRigidWorldPointJointConfigComponentSerializer>());
}

class AvbdRigidWorldDistanceSpringConfigComponentSerializer final
  : public TypedComponentSerializer<::dart::simulation::detail::deformable_vbd::
                                        AvbdRigidWorldDistanceSpringConfig>
{
public:
  using Component = ::dart::simulation::detail::deformable_vbd::
      AvbdRigidWorldDistanceSpringConfig;

  [[nodiscard]] std::string_view getTypeName() const override
  {
    return "dart::simulation::detail::deformable_vbd::"
           "AvbdRigidWorldDistanceSpringConfig";
  }

protected:
  void saveComponent(
      std::ostream& output,
      const Component& config,
      const EntityMap& entityMap) const override
  {
    const auto bodyA
        = config.bodyA != entt::null
              ? static_cast<std::uint32_t>(entityMap.at(config.bodyA))
              : static_cast<std::uint32_t>(entt::null);
    const auto bodyB
        = config.bodyB != entt::null
              ? static_cast<std::uint32_t>(entityMap.at(config.bodyB))
              : static_cast<std::uint32_t>(entt::null);

    writePOD(output, config.enabled);
    writePOD(output, bodyA);
    writePOD(output, bodyB);
    writeVector3d(output, config.localAnchorA);
    writeVector3d(output, config.localAnchorB);
    writePOD(output, config.restLength);
    writePOD(output, config.startStiffness);
    writePOD(output, config.materialStiffness);
    writePOD(output, config.maxStiffness);
  }

  void loadComponent(std::istream& input, Component& config) const override
  {
    std::uint32_t bodyA = static_cast<std::uint32_t>(entt::null);
    std::uint32_t bodyB = static_cast<std::uint32_t>(entt::null);

    readPOD(input, config.enabled);
    readPOD(input, bodyA);
    readPOD(input, bodyB);
    config.bodyA = static_cast<entt::entity>(bodyA);
    config.bodyB = static_cast<entt::entity>(bodyB);
    readVector3d(input, config.localAnchorA);
    readVector3d(input, config.localAnchorB);
    readPOD(input, config.restLength);
    readPOD(input, config.startStiffness);
    readPOD(input, config.materialStiffness);
    readPOD(input, config.maxStiffness);
  }
};

void registerAvbdRigidWorldDistanceSpringConfigSerializer(
    SerializerRegistry& registry)
{
  AvbdRigidWorldDistanceSpringConfigComponentSerializer serializerProbe;
  if (registry.getSerializer(serializerProbe.getTypeName()) != nullptr) {
    return;
  }

  registry.registerSerializer(
      std::make_unique<
          AvbdRigidWorldDistanceSpringConfigComponentSerializer>());
}

class CollisionGeometryComponentSerializer final
  : public TypedComponentSerializer<comps::CollisionGeometry>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return comps::CollisionGeometry::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const comps::CollisionGeometry& component,
      const EntityMap& /*entityMap*/) const override
  {
    serializeCollisionGeometry(output, component);
  }

  void loadComponent(
      std::istream& input, comps::CollisionGeometry& component) const override
  {
    deserializeCollisionGeometry(input, component);
  }

  void loadComponent(
      std::istream& input,
      comps::CollisionGeometry& component,
      std::uint32_t formatVersion) const override
  {
    if (formatVersion < 7u) {
      deserializeCollisionGeometryPreV7(input, component);
      return;
    }
    if (formatVersion <= 10u) {
      deserializeCollisionGeometryV7ToV10(input, component);
      return;
    }
    if (formatVersion == 11u) {
      deserializeBranchV11CollisionGeometry(input, component);
      return;
    }

    loadComponent(input, component);
  }
};

void registerCollisionGeometrySerializer(SerializerRegistry& registry)
{
  if (registry.getSerializer(comps::CollisionGeometry::getTypeName())
      != nullptr) {
    return;
  }

  registry.registerSerializer(
      std::make_unique<CollisionGeometryComponentSerializer>());
}

class LinkComponentSerializer final
  : public TypedComponentSerializer<comps::Link>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return comps::Link::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const comps::Link& component,
      const EntityMap& entityMap) const override
  {
    autoSerialize(output, component, entityMap);
  }

  void loadComponent(std::istream& input, comps::Link& component) const override
  {
    autoDeserialize(input, component);
  }

  void loadComponent(
      std::istream& input,
      comps::Link& component,
      std::uint32_t formatVersion) const override
  {
    if (formatVersion < 9u) {
      deserializeLinkV8(input, component, formatVersion);
      return;
    }

    loadComponent(input, component);
  }
};

void registerLinkSerializer(SerializerRegistry& registry)
{
  if (registry.getSerializer(comps::Link::getTypeName()) != nullptr) {
    return;
  }

  registry.registerSerializer(std::make_unique<LinkComponentSerializer>());
}

void registerBuiltInSerializers(SerializerRegistry& registry)
{
  registerComponentIfNeeded<comps::Name>(registry);

  registerComponentIfNeeded<comps::FrameTag>(registry);
  registerComponentIfNeeded<comps::FixedFrameTag>(registry);
  registerComponentIfNeeded<comps::FreeFrameTag>(registry);
  registerComponentIfNeeded<comps::FrameState>(registry);
  registerComponentIfNeeded<comps::FrameCache>(registry);
  registerComponentIfNeeded<comps::FixedFrameProperties>(registry);
  registerComponentIfNeeded<comps::FreeFrameProperties>(registry);

  registerComponentIfNeeded<comps::RigidBodyTag>(registry);
  registerComponentIfNeeded<comps::StaticBodyTag>(registry);
  registerCollisionGeometrySerializer(registry);
  registerComponentIfNeeded<comps::DeformableGroundBarrierTag>(registry);
  registerComponentIfNeeded<comps::DeformableSurfaceCcdObstacleTag>(registry);
  registerComponentIfNeeded<comps::ContactMaterial>(registry);
  registerComponentIfNeeded<comps::DeformableBodyTag>(registry);
  comps::registerDeformableBodySerializers(registry);

  registerComponentIfNeeded<comps::MultibodyTag>(registry);
  registerComponentIfNeeded<comps::MultibodyStructure>(registry);

  // The variational integrator's persistent two-step history (no entity
  // references, so no remap pass entry needed).
  registerComponentIfNeeded<compute::MultibodyVariationalState>(registry);

  // The variational integrator's ground-contact config (link indices, not
  // entity references, so likewise no remap pass entry needed) and its
  // augmented-Lagrangian dual state (per-point duals + cadence counter).
  registerComponentIfNeeded<comps::VariationalContact>(registry);
  registerComponentIfNeeded<comps::VariationalContactDualState>(registry);

  registerComponentIfNeeded<comps::LoopClosure>(registry);

  registerLinkSerializer(registry);
  registerComponentIfNeeded<comps::JointModel>(registry);
  registerComponentIfNeeded<comps::JointState>(registry);
  registerComponentIfNeeded<comps::JointActuation>(registry);
  registerComponentIfNeeded<comps::AvbdJointStiffness>(registry);
  registerAvbdRigidWorldPointJointConfigSerializer(registry);
  registerAvbdRigidWorldDistanceSpringConfigSerializer(registry);

  registerComponentIfNeeded<comps::Transform>(registry);
  registerComponentIfNeeded<comps::Velocity>(registry);
  registerComponentIfNeeded<comps::MassProperties>(registry);
  registerComponentIfNeeded<comps::Force>(registry);
  registerComponentIfNeeded<comps::DeactivationState>(registry);
}

} // namespace

//==============================================================================
SerializerRegistry& SerializerRegistry::instance()
{
  static SerializerRegistry s_instance;
  registerBuiltInSerializers(s_instance);
  return s_instance;
}

//==============================================================================
void SerializerRegistry::registerSerializer(
    std::unique_ptr<ComponentSerializer> serializer)
{
  if (!serializer) {
    throw std::invalid_argument("Cannot register null serializer");
  }

  std::string typeName(serializer->getTypeName());
  if (typeName.empty()) {
    throw std::invalid_argument("Serializer type name cannot be empty");
  }

  if (m_serializers.contains(typeName)) {
    throw std::runtime_error(
        "Serializer for type '" + typeName + "' is already registered");
  }

  m_serializers[typeName] = std::move(serializer);
}

//==============================================================================
const ComponentSerializer* SerializerRegistry::getSerializer(
    std::string_view typeName) const
{
  auto it = m_serializers.find(std::string(typeName));
  if (it != m_serializers.end()) {
    return it->second.get();
  }
  return nullptr;
}

//==============================================================================
const std::unordered_map<std::string, std::unique_ptr<ComponentSerializer>>&
SerializerRegistry::getSerializers() const
{
  return m_serializers;
}

//==============================================================================
void SerializerRegistry::saveAllEntities(
    std::ostream& output,
    const ::dart::simulation::detail::WorldRegistry& registry,
    EntityMap& entityMap) const
{
  // Collect entities in a deterministic order
  std::vector<entt::entity> entities;
  auto viewAll = registry.view<comps::Name>();
  for (auto entity : viewAll) {
    entities.push_back(entity);
  }

  std::ranges::sort(entities, [](entt::entity lhs, entt::entity rhs) {
    return static_cast<std::uint32_t>(lhs) < static_cast<std::uint32_t>(rhs);
  });

  // Write entity count
  writePOD(output, entities.size());

  // Build entity mapping (old ID -> saved ID)
  entityMap.clear();
  std::uint32_t sequential = 0;
  for (auto entity : entities) {
    entityMap[entity] = static_cast<entt::entity>(sequential++);
  }

  // Write each entity and its components
  for (auto entity : entities) {
    // Write entity ID (use sequential ID for stability)
    writePOD(output, static_cast<std::uint32_t>(entityMap[entity]));

    std::vector<std::string> componentTypes;
    for (const auto& [typeName, serializer] : m_serializers) {
      if (serializer->hasComponent(entity, registry)) {
        componentTypes.push_back(typeName);
      }
    }

    // Write component count
    std::size_t componentCount = componentTypes.size();
    writePOD(output, componentCount);

    // Write each component type name and data
    for (const auto& typeName : componentTypes) {
      // Write component type name
      writeString(output, typeName);

      // Write component data using serializer
      auto* serializer = getSerializer(typeName);
      serializer->save(output, entity, registry, entityMap);
    }
  }
}

//==============================================================================
void SerializerRegistry::loadAllEntities(
    std::istream& input,
    ::dart::simulation::detail::WorldRegistry& registry,
    EntityMap& entityMap,
    std::uint32_t formatVersion) const
{
  // Read entity count
  std::size_t entityCount;
  readPOD(input, entityCount);

  // Clear output map
  entityMap.clear();

  // Read each entity and its components
  for (std::size_t i = 0; i < entityCount; ++i) {
    // Read entity ID
    std::uint32_t serializedId;
    readPOD(input, serializedId);

    // Create new entity
    auto entity = registry.create();
    entityMap[static_cast<entt::entity>(serializedId)] = entity;

    // Read component count
    std::size_t componentCount;
    readPOD(input, componentCount);

    // Read each component
    for (std::size_t j = 0; j < componentCount; ++j) {
      // Read component type name
      std::string typeName;
      readString(input, typeName);

      // Get serializer for this component type
      auto* serializer = getSerializer(typeName);
      if (serializer) {
        // Use serializer to load component data
        serializer->load(input, entity, registry, formatVersion);
      } else {
        // Unknown component type - cannot skip (no size info)
        throw std::runtime_error(
            std::format("Unknown component type: {}", typeName));
      }
    }
  }

  // Second pass: Fix entity references using entityMap
  // For components that contain entity IDs, we need to remap them
  auto mbView = registry.view<comps::MultibodyStructure>();
  for (auto entity : mbView) {
    auto& comp = mbView.get<comps::MultibodyStructure>(entity);
    for (auto& e : comp.links) {
      if (e != entt::null) {
        e = entityMap.at(e);
      }
    }
    for (auto& e : comp.joints) {
      if (e != entt::null) {
        e = entityMap.at(e);
      }
    }
  }

  auto jointView = registry.view<comps::JointModel>();
  for (auto entity : jointView) {
    auto& comp = jointView.get<comps::JointModel>(entity);
    if (comp.parentLink != entt::null) {
      comp.parentLink = entityMap.at(comp.parentLink);
    }
    if (comp.childLink != entt::null) {
      comp.childLink = entityMap.at(comp.childLink);
    }
  }

  auto linkView = registry.view<comps::Link>();
  for (auto entity : linkView) {
    auto& comp = linkView.get<comps::Link>(entity);
    if (comp.parentJoint != entt::null) {
      comp.parentJoint = entityMap.at(comp.parentJoint);
    }
    for (auto& e : comp.childJoints) {
      if (e != entt::null) {
        e = entityMap.at(e);
      }
    }
  }

  auto loopClosureView = registry.view<comps::LoopClosure>();
  for (auto entity : loopClosureView) {
    auto& comp = loopClosureView.get<comps::LoopClosure>(entity);
    if (comp.frameA != entt::null) {
      comp.frameA = entityMap.at(comp.frameA);
    }
    if (comp.frameB != entt::null) {
      comp.frameB = entityMap.at(comp.frameB);
    }
  }

  // Remap FrameState parent references
  auto frameStateView = registry.view<comps::FrameState>();
  for (auto entity : frameStateView) {
    auto& comp = frameStateView.get<comps::FrameState>(entity);
    if (comp.parentFrame != entt::null) {
      comp.parentFrame = entityMap.at(comp.parentFrame);
    }
  }

  auto distanceSpringView
      = registry.view<::dart::simulation::detail::deformable_vbd::
                          AvbdRigidWorldDistanceSpringConfig>();
  for (auto entity : distanceSpringView) {
    auto& comp
        = distanceSpringView.get<::dart::simulation::detail::deformable_vbd::
                                     AvbdRigidWorldDistanceSpringConfig>(
            entity);
    if (comp.bodyA != entt::null) {
      comp.bodyA = entityMap.at(comp.bodyA);
    }
    if (comp.bodyB != entt::null) {
      comp.bodyB = entityMap.at(comp.bodyB);
    }
  }
}

//==============================================================================
void SerializerRegistry::clear()
{
  m_serializers.clear();
}

} // namespace dart::simulation::io
