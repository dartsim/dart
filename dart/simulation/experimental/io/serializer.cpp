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

#include "dart/simulation/experimental/io/serializer.hpp"

#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/io/binary_io.hpp"
#include "dart/simulation/experimental/io/category_serializer.hpp"

#include <algorithm>
#include <format>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace dart::simulation::experimental::io {

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

Eigen::VectorXd symmetricLowerBound(const Eigen::VectorXd& upperBound)
{
  if (upperBound.size() == 0) {
    return Eigen::VectorXd{};
  }
  return -upperBound.cwiseAbs();
}

Eigen::VectorXd symmetricUpperBound(const Eigen::VectorXd& upperBound)
{
  if (upperBound.size() == 0) {
    return Eigen::VectorXd{};
  }
  return upperBound.cwiseAbs();
}

void deserializeJointV1(std::istream& input, comps::Joint& joint)
{
  readPOD(input, joint.type);
  readString(input, joint.name);
  readVectorXd(input, joint.position);
  readVectorXd(input, joint.velocity);
  readVectorXd(input, joint.acceleration);
  readVectorXd(input, joint.torque);

  readVectorXd(input, joint.limits.lower);
  readVectorXd(input, joint.limits.upper);

  Eigen::VectorXd legacyVelocityLimit;
  Eigen::VectorXd legacyEffortLimit;
  readVectorXd(input, legacyVelocityLimit);
  readVectorXd(input, legacyEffortLimit);

  readVector3d(input, joint.axis);
  readVector3d(input, joint.axis2);
  readPOD(input, joint.pitch);

  std::uint32_t parentLink = static_cast<std::uint32_t>(entt::null);
  std::uint32_t childLink = static_cast<std::uint32_t>(entt::null);
  readPOD(input, parentLink);
  readPOD(input, childLink);
  joint.parentLink = static_cast<entt::entity>(parentLink);
  joint.childLink = static_cast<entt::entity>(childLink);

  const auto dof = static_cast<Eigen::Index>(joint.getDOF());
  joint.actuatorType = comps::ActuatorType::Force;
  joint.springStiffness = Eigen::VectorXd::Zero(dof);
  joint.dampingCoefficient = Eigen::VectorXd::Zero(dof);
  joint.restPosition = Eigen::VectorXd::Zero(dof);
  joint.armature = Eigen::VectorXd::Zero(dof);
  joint.coulombFriction = Eigen::VectorXd::Zero(dof);
  joint.commandVelocity = Eigen::VectorXd::Zero(dof);

  const double infinity = std::numeric_limits<double>::infinity();
  if (joint.limits.lower.size() != dof) {
    joint.limits.lower = Eigen::VectorXd::Constant(dof, -infinity);
  }
  if (joint.limits.upper.size() != dof) {
    joint.limits.upper = Eigen::VectorXd::Constant(dof, infinity);
  }
  joint.limits.velocityLower = legacyVelocityLimit.size() == dof
                                   ? symmetricLowerBound(legacyVelocityLimit)
                                   : Eigen::VectorXd::Constant(dof, -infinity);
  joint.limits.velocityUpper = legacyVelocityLimit.size() == dof
                                   ? symmetricUpperBound(legacyVelocityLimit)
                                   : Eigen::VectorXd::Constant(dof, infinity);
  joint.limits.effortLower = legacyEffortLimit.size() == dof
                                 ? symmetricLowerBound(legacyEffortLimit)
                                 : Eigen::VectorXd::Constant(dof, -infinity);
  joint.limits.effortUpper = legacyEffortLimit.size() == dof
                                 ? symmetricUpperBound(legacyEffortLimit)
                                 : Eigen::VectorXd::Constant(dof, infinity);
}

class JointComponentSerializer final
  : public TypedComponentSerializer<comps::Joint>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return comps::Joint::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const comps::Joint& component,
      const EntityMap& entityMap) const override
  {
    autoSerialize(output, component, entityMap);
  }

  void loadComponent(
      std::istream& input, comps::Joint& component) const override
  {
    autoDeserialize(input, component);
  }

  void loadComponent(
      std::istream& input,
      comps::Joint& component,
      std::uint32_t formatVersion) const override
  {
    if (formatVersion == 1u) {
      deserializeJointV1(input, component);
      return;
    }

    loadComponent(input, component);
  }
};

void registerJointSerializer(SerializerRegistry& registry)
{
  if (registry.getSerializer(comps::Joint::getTypeName()) != nullptr) {
    return;
  }

  registry.registerSerializer(std::make_unique<JointComponentSerializer>());
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
  registerComponentIfNeeded<comps::CollisionGeometry>(registry);
  registerComponentIfNeeded<comps::ContactMaterial>(registry);

  registerComponentIfNeeded<comps::MultibodyTag>(registry);
  registerComponentIfNeeded<comps::MultibodyStructure>(registry);

  registerComponentIfNeeded<comps::LoopClosure>(registry);

  registerComponentIfNeeded<comps::Link>(registry);
  registerJointSerializer(registry);

  registerComponentIfNeeded<comps::Transform>(registry);
  registerComponentIfNeeded<comps::Velocity>(registry);
  registerComponentIfNeeded<comps::MassProperties>(registry);
  registerComponentIfNeeded<comps::Force>(registry);
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
    const entt::registry& registry,
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
    entt::registry& registry,
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

  auto jointView = registry.view<comps::Joint>();
  for (auto entity : jointView) {
    auto& comp = jointView.get<comps::Joint>(entity);
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
}

//==============================================================================
void SerializerRegistry::clear()
{
  m_serializers.clear();
}

} // namespace dart::simulation::experimental::io
