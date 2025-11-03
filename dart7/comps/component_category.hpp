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

#include <entt/entt.hpp>

#include <string_view>
#include <tuple>
#include <type_traits>
#include <typeinfo>

namespace dart7::comps {

//==============================================================================
// Component Category System (Macro-based)
//
// Uses macros to inject category metadata without base classes.
// This keeps components as simple aggregates compatible with Boost.PFR.
//==============================================================================

/// Component category enum for compile-time dispatch
enum class ComponentCategory
{
  Property, // Simple data components (auto-serialized)
  State,    // Components with entity references (auto-remapped)
  Cache,    // Derived/computed data (never serialized)
  Tag       // Empty marker components
};

//==============================================================================
// Category Declaration Macros
//==============================================================================

/// Declare a Property Component
///
/// Automatically serialized via Boost.PFR reflection.
/// All member fields are serialized.
///
/// Example:
///   struct Name {
///     DART7_PROPERTY_COMPONENT(Name);
///     std::string name;
///   };
#define DART7_PROPERTY_COMPONENT(TypeName)                                     \
  static constexpr ::dart7::comps::ComponentCategory category                  \
      = ::dart7::comps::ComponentCategory::Property;                           \
  static constexpr bool serializable = true;                                   \
  [[nodiscard]] static std::string_view getTypeName()                          \
  {                                                                            \
    return typeid(TypeName).name();                                            \
  }

/// Declare a State Component
///
/// Automatically serialized with entity ID remapping.
/// Requires `entityFields()` method to declare entity reference fields.
///
/// Example:
///   struct FrameState {
///     DART7_STATE_COMPONENT(FrameState);
///     entt::entity parentFrame = entt::null;
///
///     static constexpr auto entityFields() {
///       return std::tuple{&FrameState::parentFrame};
///     }
///   };
#define DART7_STATE_COMPONENT(TypeName)                                        \
  static constexpr ::dart7::comps::ComponentCategory category                  \
      = ::dart7::comps::ComponentCategory::State;                              \
  static constexpr bool serializable = true;                                   \
  [[nodiscard]] static std::string_view getTypeName()                          \
  {                                                                            \
    return typeid(TypeName).name();                                            \
  }

/// Declare a Cache Component
///
/// Not serialized - reconstructed after deserialization.
///
/// Example:
///   struct FrameCache {
///     DART7_CACHE_COMPONENT(FrameCache);
///     Eigen::Isometry3d worldTransform;
///     bool needUpdate = true;
///   };
#define DART7_CACHE_COMPONENT(TypeName)                                        \
  static constexpr ::dart7::comps::ComponentCategory category                  \
      = ::dart7::comps::ComponentCategory::Cache;                              \
  static constexpr bool serializable = false;                                  \
  [[nodiscard]] static std::string_view getTypeName()                          \
  {                                                                            \
    return typeid(TypeName).name();                                            \
  }

/// Declare a Tag Component
///
/// Empty marker components with no data.
///
/// Example:
///   struct FreeFrameTag {
///     DART7_TAG_COMPONENT(FreeFrameTag);
///   };
#define DART7_TAG_COMPONENT(TypeName)                                          \
  static constexpr ::dart7::comps::ComponentCategory category                  \
      = ::dart7::comps::ComponentCategory::Tag;                                \
  static constexpr bool serializable = true;                                   \
  [[nodiscard]] static std::string_view getTypeName()                          \
  {                                                                            \
    return typeid(TypeName).name();                                            \
  }

//==============================================================================
// Type Traits & Concepts
//==============================================================================

/// Check if a type has component category metadata
template <typename T>
concept HasComponentCategory = requires
{
  typename std::integral_constant<ComponentCategory, T::category>;
};

/// Check if a type is a PropertyComponent
template <typename T>
concept IsPropertyComponent
    = HasComponentCategory<T> && T::category == ComponentCategory::Property;

/// Check if a type is a StateComponent
template <typename T>
concept IsStateComponent
    = HasComponentCategory<T> && T::category == ComponentCategory::State;

/// Check if a type is a CacheComponent
template <typename T>
concept IsCacheComponent
    = HasComponentCategory<T> && T::category == ComponentCategory::Cache;

/// Check if a type is a TagComponent
template <typename T>
concept IsTagComponent
    = HasComponentCategory<T> && T::category == ComponentCategory::Tag;

/// Check if a type has entityFields() method
template <typename T>
concept HasEntityFields = requires
{
  T::entityFields();
};

} // namespace dart7::comps
