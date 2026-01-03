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

#include <entt/entt.hpp>

#include <format>
#include <source_location>
#include <string_view>

namespace dart::simulation::experimental {

/// Safe component access for ECS entities - prevents segfaults
///
/// This wrapper around the ECS registry's get() operation checks component
/// existence before access, throwing a clear exception instead of segfaulting.
///
/// **Why this exists:**
/// Direct registry access (e.g., EnTT's `registry.get<T>()`) dereferences
/// without checking, causing silent segfaults when components are missing.
/// This provides terrible developer experience - cryptic crashes with no
/// actionable information.
///
/// **Performance:**
/// - Debug builds: Negligible overhead (~1 pointer check)
/// - Release builds: Often optimized away by compiler
/// - NOT performance-critical: This is used in API layers, not hot loops
///
/// **Usage:**
/// ```cpp
/// // BAD - can segfault silently
/// const auto& comp = registry.get<MyComponent>(entity);
///
/// // GOOD - throws clear exception with entity ID and component type
/// const auto& comp = safeGet<MyComponent>(registry, entity);
/// ```
///
/// @tparam T Component type
/// @param registry ECS registry
/// @param entity Entity to get component from
/// @param loc Source location for error reporting (auto-filled)
/// @return Reference to the component
/// @throws InvalidArgumentException if component doesn't exist
template <typename T>
const T& safeGet(
    const entt::registry& registry,
    entt::entity entity,
    const std::source_location loc = std::source_location::current())
{
  const T* component = registry.try_get<T>(entity);

  DART8_THROW_T_IF(
      !component,
      InvalidArgumentException,
      "Entity {} is missing component '{}'. This indicates the entity was not "
      "properly initialized or has been corrupted. Location: {}:{}",
      static_cast<std::uint32_t>(entity),
      typeid(T).name(),
      loc.file_name(),
      loc.line());

  return *component;
}

/// Mutable version of safeGet
template <typename T>
T& safeGet(
    entt::registry& registry,
    entt::entity entity,
    const std::source_location loc = std::source_location::current())
{
  T* component = registry.try_get<T>(entity);

  DART8_THROW_T_IF(
      !component,
      InvalidArgumentException,
      "Entity {} is missing component '{}'. This indicates the entity was not "
      "properly initialized or has been corrupted. Location: {}:{}",
      static_cast<std::uint32_t>(entity),
      typeid(T).name(),
      loc.file_name(),
      loc.line());

  return *component;
}

} // namespace dart::simulation::experimental
