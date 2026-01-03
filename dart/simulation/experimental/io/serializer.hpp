/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#pragma once

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/export.hpp>
#include <dart/simulation/experimental/io/binary_io.hpp>

#include <entt/entity/registry.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

namespace dart::simulation::experimental::io {

//==============================================================================
// Component Serializer Interface
//==============================================================================

// Abstract interface for component serialization
// Users can extend this to support custom component types
class DART_EXPERIMENTAL_API ComponentSerializer
{
public:
  virtual ~ComponentSerializer() = default;

  // Get the type name identifier for this component
  // This name is used in the binary format to identify component types
  // Example: "dart::simulation::experimental::JointComponent" or
  // "mylib::CustomJoint"
  virtual std::string_view getTypeName() const = 0;

  // Write component data to binary stream
  // Note: The type name is written automatically by the registry
  // @param entityMap Maps old entity IDs to sequential save IDs
  virtual void save(
      std::ostream& out,
      entt::entity entity,
      const entt::registry& registry,
      const EntityMap& entityMap) const = 0;

  // Read component data from binary stream and attach to entity
  // Note: The type name has already been read by the registry
  virtual void load(
      std::istream& in,
      entt::entity entity,
      entt::registry& registry) const = 0;

  // Check if an entity has this component type
  virtual bool hasComponent(
      entt::entity entity, const entt::registry& registry) const = 0;
};

//==============================================================================
// Serializer Registry
//==============================================================================

// Registry for component serializers
// This allows users to register custom serializers for their own components
//
// Usage example:
//   auto& registry = SerializerRegistry::instance();
//   registry.registerSerializer(std::make_unique<MyCustomSerializer>());
//
class DART_EXPERIMENTAL_API SerializerRegistry
{
public:
  // Get singleton instance
  static SerializerRegistry& instance();

  // Register a component serializer
  // The serializer's type name must be unique
  void registerSerializer(std::unique_ptr<ComponentSerializer> serializer);

  // Get serializer by type name
  // Returns nullptr if not found
  const ComponentSerializer* getSerializer(std::string_view typeName) const;

  // Get all registered serializers (for iteration during save)
  const std::unordered_map<std::string, std::unique_ptr<ComponentSerializer>>&
  getSerializers() const;

  // Save all entities and their components from registry to output stream
  // This is the main entry point for serialization - fully automatic!
  // @param entityMap Output mapping from old entity IDs to sequential save IDs
  void saveAllEntities(
      std::ostream& output,
      const entt::registry& registry,
      EntityMap& entityMap) const;

  // Load all entities and their components from input stream to registry
  // This is the main entry point for deserialization - fully automatic!
  // @param entityMap Output mapping from saved IDs to new entity IDs
  void loadAllEntities(
      std::istream& input,
      entt::registry& registry,
      EntityMap& entityMap) const;

  // Clear all registered serializers (primarily for testing)
  void clear();

private:
  SerializerRegistry();
  ~SerializerRegistry() = default;

  SerializerRegistry(const SerializerRegistry&) = delete;
  SerializerRegistry& operator=(const SerializerRegistry&) = delete;

  std::unordered_map<std::string, std::unique_ptr<ComponentSerializer>>
      m_serializers;
};

//==============================================================================
// Helper: Check if component is serializable
//==============================================================================

// Template to detect if a component has a 'serializable' static member
// Components can opt-out of serialization by defining:
//   static constexpr bool serializable = false;
//
// Default behavior (no serializable member): component IS serialized
template <typename ComponentT, typename = void>
struct IsSerializable : std::true_type
{
};

// Specialization for components with 'serializable' member
template <typename ComponentT>
struct IsSerializable<
    ComponentT,
    std::void_t<decltype(ComponentT::serializable)>>
  : std::bool_constant<ComponentT::serializable>
{
};

// Helper variable template
template <typename ComponentT>
inline constexpr bool IsSerializable_v = IsSerializable<ComponentT>::value;

//==============================================================================
// Helper: Typed Component Serializer
//==============================================================================

// Template helper for implementing component serializers
// This provides a convenient base class for serializers of specific component
// types
//
// Components can opt-out of serialization by defining:
//   static constexpr bool serializable = false;
//
// Usage example:
//   class MyComponentSerializer : public TypedComponentSerializer<MyComponent>
//   {
//     std::string_view getTypeName() const override
//     {
//       return "mylib::MyComponent";
//     }
//
//     void saveComponent(std::ostream& out, const MyComponent& comp) const
//     override { ... }
//
//     void loadComponent(std::istream& in, MyComponent& comp) const override {
//     ... }
//   };
//
template <typename ComponentT>
class TypedComponentSerializer : public ComponentSerializer
{
public:
  void save(
      std::ostream& out,
      entt::entity entity,
      const entt::registry& registry,
      const EntityMap& entityMap) const final
  {
    // Skip serialization if component has serializable = false
    if constexpr (!IsSerializable_v<ComponentT>) {
      // Component is marked as non-serializable (e.g., cache components)
      // Skip silently - this is intentional
      return;
    }

    if constexpr (std::is_empty_v<ComponentT>) {
      // Empty tag components - nothing to serialize
      ComponentT component;
      saveComponent(out, component, entityMap);
    } else {
      const auto& component = registry.get<ComponentT>(entity);
      saveComponent(out, component, entityMap);
    }
  }

  void load(std::istream& in, entt::entity entity, entt::registry& registry)
      const final
  {
    ComponentT component;
    loadComponent(in, component);

    if constexpr (std::is_empty_v<ComponentT>) {
      registry.emplace<ComponentT>(entity);
    } else {
      registry.emplace<ComponentT>(entity, std::move(component));
    }
  }

  bool hasComponent(
      entt::entity entity, const entt::registry& registry) const final
  {
    return registry.all_of<ComponentT>(entity);
  }

protected:
  // Override these in derived classes to implement serialization logic
  virtual void saveComponent(
      std::ostream& out,
      const ComponentT& component,
      const EntityMap& entityMap) const = 0;
  virtual void loadComponent(std::istream& in, ComponentT& component) const = 0;
};

//==============================================================================
// Registration Helper
//==============================================================================

// RAII helper for automatic serializer registration
// Place this in a .cpp file to automatically register a serializer at startup
//
// Usage example:
//   static SerializerRegistration<MyComponentSerializer> s_registration;
//
template <typename SerializerT>
class SerializerRegistration
{
public:
  SerializerRegistration()
  {
    // Create the derived serializer and cast to base pointer type
    SerializerT* derived = new SerializerT();
    ComponentSerializer* base = static_cast<ComponentSerializer*>(derived);
    std::unique_ptr<ComponentSerializer> serializer(base);
    SerializerRegistry::instance().registerSerializer(std::move(serializer));
  }
};

} // namespace dart::simulation::experimental::io
