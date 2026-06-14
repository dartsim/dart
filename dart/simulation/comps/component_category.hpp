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

#pragma once

#include <entt/entt.hpp>

#include <string_view>
#include <tuple>
#include <type_traits>
#include <typeinfo>

namespace dart::simulation::comps {

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
//
// Serialization identity (stable, compiler-independent)
// ----------------------------------------------------------------------------
// Each category macro takes an explicit `StableId` string literal that becomes
// the component's on-disk identity AND its SerializerRegistry key (via
// getTypeName()). This identity is INTENTIONALLY decoupled from the C++ type
// name: renaming a component's C++ type must NOT change its on-disk identity.
//
// Earlier revisions returned `typeid(TypeName).name()`, whose value is the
// compiler-mangled name. That made the on-disk format implementation-defined
// (different across compilers/ABIs) and fragile under C++ renames. The stable
// string ID removes both hazards.
//
// Format-break note (DART 7 has no compatibility debt): old binary packets that
// stored mangled-name identities will no longer load. This is intentional; the
// kBinaryFormatVersion bump in binary_io.hpp records the break. See WP-091.23.
//
// ID scheme: "comps.<TypeName>" for components in namespace
// dart::simulation::comps, "compute.<TypeName>" for compute-owned components.
// IDs MUST be globally unique across all components; the StableComponentIds
// unit test in test_serialization.cpp guards against duplicates and against any
// ID accidentally equal to a mangled typeid string.

/// Declare a Property Component
///
/// Automatically serialized via Boost.PFR reflection.
/// All member fields are serialized.
///
/// @param TypeName The C++ type being declared.
/// @param StableId String-literal on-disk identity (e.g. "comps.Name"). Must be
///        globally unique and stable across C++ renames.
///
/// Example:
///   struct Name {
///     DART_SIMULATION_PROPERTY_COMPONENT(Name, "comps.Name");
///     std::string name;
///   };
#define DART_SIMULATION_PROPERTY_COMPONENT(TypeName, StableId)                 \
  static constexpr ::dart::simulation::comps::ComponentCategory category       \
      = ::dart::simulation::comps::ComponentCategory::Property;                \
  static constexpr bool serializable = true;                                   \
  [[nodiscard]] static std::string_view getTypeName()                          \
  {                                                                            \
    return StableId;                                                           \
  }

/// Declare a State Component
///
/// Automatically serialized with entity ID remapping.
/// Requires `entityFields()` method to declare entity reference fields.
///
/// @param TypeName The C++ type being declared.
/// @param StableId String-literal on-disk identity. Must be globally unique and
///        stable across C++ renames.
///
/// Example:
///   struct FrameState {
///     DART_SIMULATION_STATE_COMPONENT(FrameState, "comps.FrameState");
///     entt::entity parentFrame = entt::null;
///
///     static constexpr auto entityFields() {
///       return std::tuple{&FrameState::parentFrame};
///     }
///   };
#define DART_SIMULATION_STATE_COMPONENT(TypeName, StableId)                    \
  static constexpr ::dart::simulation::comps::ComponentCategory category       \
      = ::dart::simulation::comps::ComponentCategory::State;                   \
  static constexpr bool serializable = true;                                   \
  [[nodiscard]] static std::string_view getTypeName()                          \
  {                                                                            \
    return StableId;                                                           \
  }

/// Declare a Cache Component
///
/// Not serialized - reconstructed after deserialization.
///
/// @param TypeName The C++ type being declared.
/// @param StableId String-literal identity. Cache components are not
///        serialized, but the ID is still required for a uniform interface and
///        must be globally unique.
///
/// Example:
///   struct FrameCache {
///     DART_SIMULATION_CACHE_COMPONENT(FrameCache, "comps.FrameCache");
///     Eigen::Isometry3d worldTransform;
///     bool needUpdate = true;
///   };
#define DART_SIMULATION_CACHE_COMPONENT(TypeName, StableId)                    \
  static constexpr ::dart::simulation::comps::ComponentCategory category       \
      = ::dart::simulation::comps::ComponentCategory::Cache;                   \
  static constexpr bool serializable = false;                                  \
  [[nodiscard]] static std::string_view getTypeName()                          \
  {                                                                            \
    return StableId;                                                           \
  }

/// Declare a Tag Component
///
/// Empty marker components with no data.
///
/// @param TypeName The C++ type being declared.
/// @param StableId String-literal on-disk identity. Must be globally unique and
///        stable across C++ renames.
///
/// Example:
///   struct FreeFrameTag {
///     DART_SIMULATION_TAG_COMPONENT(FreeFrameTag, "comps.FreeFrameTag");
///   };
#define DART_SIMULATION_TAG_COMPONENT(TypeName, StableId)                      \
  static constexpr ::dart::simulation::comps::ComponentCategory category       \
      = ::dart::simulation::comps::ComponentCategory::Tag;                     \
  static constexpr bool serializable = true;                                   \
  [[nodiscard]] static std::string_view getTypeName()                          \
  {                                                                            \
    return StableId;                                                           \
  }

//==============================================================================
// Type Traits & Concepts
//==============================================================================

/// Check if a type has component category metadata
template <typename T>
concept HasComponentCategory = requires {
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
concept HasEntityFields = requires { T::entityFields(); };

} // namespace dart::simulation::comps
