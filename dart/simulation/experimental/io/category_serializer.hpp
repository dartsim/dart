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

#include <dart/simulation/experimental/common/macros.hpp>
#include <dart/simulation/experimental/comps/component_category.hpp>
#include <dart/simulation/experimental/io/auto_serialization.hpp>
#include <dart/simulation/experimental/io/serializer.hpp>

#include <entt/entt.hpp>

namespace dart::simulation::experimental::io {

//==============================================================================
// Generic ComponentSerializer for category-based components
//
// Automatically handles serialization/deserialization for components that
// inherit from PropertyComponent, StateComponent, TagComponent, or
// CacheComponent using Boost.PFR reflection.
//==============================================================================

template <typename T>
requires comps::HasComponentCategory<T>
class CategoryComponentSerializer : public TypedComponentSerializer<T>
{
public:
  //============================================================================
  // Constructor
  //============================================================================

  CategoryComponentSerializer() = default;

  //============================================================================
  // TypedComponentSerializer interface implementation
  //============================================================================

  [[nodiscard]] std::string_view getTypeName() const override
  {
    // Use component's type name
    return T::getTypeName();
  }

  //============================================================================
  // TypedComponentSerializer protected interface
  //============================================================================

protected:
  void saveComponent(
      std::ostream& output,
      const T& component,
      const EntityMap& entityMap) const override
  {
    // Automatic serialization via Boost.PFR
    autoSerialize(output, component, entityMap);
  }

  void loadComponent(std::istream& input, T& component) const override
  {
    // Automatic deserialization via Boost.PFR
    autoDeserialize(input, component);
    // Note: Entity remapping happens in SerializerRegistry::loadAllEntities()
    // after all components are loaded
  }
};

//==============================================================================
// Helper: Create and register a CategoryComponentSerializer
//==============================================================================

template <typename T>
requires comps::HasComponentCategory<T>
void registerCategoryComponent()
{
  if constexpr (T::serializable) {
    auto& registry = SerializerRegistry::instance();
    if (registry.getSerializer(T::getTypeName()) != nullptr) {
      return;
    }
    registry.registerSerializer(
        std::make_unique<CategoryComponentSerializer<T>>());
  }
}

//==============================================================================
// Helper: Auto-registration trigger via static initialization
//==============================================================================

template <typename T>
struct AutoComponentRegistration
{
  AutoComponentRegistration()
  {
    registerCategoryComponent<T>();
  }
};

//==============================================================================
// Macro for easy component registration in .cpp files
//==============================================================================

// Use this macro in a .cpp file to register a single component
#define DART8_REGISTER_COMPONENT(ComponentType)                                \
  namespace {                                                                  \
  struct ComponentType##_Registration                                          \
  {                                                                            \
    ComponentType##_Registration()                                             \
    {                                                                          \
      ::dart::simulation::experimental::io::registerCategoryComponent<ComponentType>();                 \
    }                                                                          \
  };                                                                           \
  [[maybe_unused]] static ComponentType##_Registration                         \
      s_##ComponentType##_registration;                                        \
  }

// Use this macro in a .cpp file to register multiple components at once
#define DART8_REGISTER_COMPONENTS_BEGIN()                                      \
  namespace {                                                                  \
  struct ComponentRegistration                                                 \
  {                                                                            \
    ComponentRegistration()                                                    \
    {
#define DART8_REGISTER_COMPONENTS_ADD(ComponentType)                           \
  ::dart::simulation::experimental::io::registerCategoryComponent<ComponentType>();

#define DART8_REGISTER_COMPONENTS_END()                                        \
  }                                                                            \
  }                                                                            \
  ;                                                                            \
  [[maybe_unused]] static ComponentRegistration s_registration;                \
  }

} // namespace dart::simulation::experimental::io
