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

#include <cstdint>

namespace dart::simulation {

namespace detail {
struct EntityAccess;
} // namespace detail

/// Opaque, backend-neutral identifier for an object owned by a `World`.
///
/// The promoted DART 7 public simulation API uses `Entity` instead of the
/// underlying ECS entity type so that public headers do not depend on the
/// storage backend. Treat `Entity` as an opaque token obtained from, and passed
/// back to, the `World` and handle APIs.
struct Entity
{
  constexpr Entity() noexcept = default;

  friend bool operator==(const Entity&, const Entity&) = default;

private:
  friend struct detail::EntityAccess;

  explicit constexpr Entity(std::uint32_t value) noexcept : m_value(value) {}

  std::uint32_t m_value = 0xFFFFFFFFu;
};

namespace detail {

/// Internal raw-token access for `detail/entity_conversion.hpp`.
///
/// Keeping this in `detail` lets public handles keep an opaque value type while
/// internal translation code still round-trips through the ECS storage handle.
struct EntityAccess
{
  [[nodiscard]] static constexpr std::uint32_t rawValue(Entity entity) noexcept
  {
    return entity.m_value;
  }

  [[nodiscard]] static constexpr Entity fromRawValue(
      std::uint32_t value) noexcept
  {
    return Entity(value);
  }
};

} // namespace detail

} // namespace dart::simulation
