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

#ifndef DART_SIMULATION_ECSENTITY_HPP_
#define DART_SIMULATION_ECSENTITY_HPP_

#include <functional>

#include <cstdint>

namespace dart::simulation {

/// Opaque handle that identifies an entity in the internal ECS registry.
///
/// This is intentionally decoupled from the underlying ECS library so DART can
/// change internal implementations without affecting the public API.
class EcsEntity final
{
public:
  using ValueType = std::uint64_t;

  constexpr EcsEntity() = default;
  explicit constexpr EcsEntity(ValueType value) : mValue(value) {}

  [[nodiscard]] constexpr ValueType value() const
  {
    return mValue;
  }

  [[nodiscard]] constexpr bool isNull() const
  {
    return mValue == 0u;
  }

  friend constexpr bool operator==(EcsEntity, EcsEntity) = default;
  friend constexpr bool operator!=(EcsEntity, EcsEntity) = default;

private:
  ValueType mValue{0u};
};

} // namespace dart::simulation

namespace std {

template <>
struct hash<dart::simulation::EcsEntity>
{
  std::size_t operator()(
      const dart::simulation::EcsEntity& entity) const noexcept
  {
    return std::hash<dart::simulation::EcsEntity::ValueType>{}(entity.value());
  }
};

} // namespace std

#endif // DART_SIMULATION_ECSENTITY_HPP_
