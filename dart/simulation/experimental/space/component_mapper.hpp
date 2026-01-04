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

#include <Eigen/Core>
#include <entt/entt.hpp>

#include <functional>
#include <vector>

namespace dart::simulation::experimental {

/// Abstract interface for extracting component data to/from flat vectors
///
/// ComponentMapper knows how to convert between ECS component data and
/// flat vector representations. Each concrete mapper handles a specific
/// component type (or field within a component).
///
/// Design principles:
/// - Polymorphic interface for runtime flexibility
/// - In-place operations to avoid allocations
/// - Offset-based indexing for efficient concatenation
///
/// Example usage:
/// ```cpp
/// class PositionMapper : public ComponentMapper {
///   void toVector(const entt::registry& reg, ...) override {
///     auto view = reg.view<Position>();
///     for (auto entity : view) {
///       const auto& pos = view.get<Position>(entity);
///       vec[offset++] = pos.x;
///       vec[offset++] = pos.y;
///       vec[offset++] = pos.z;
///     }
///   }
/// };
/// ```
class ComponentMapper
{
public:
  virtual ~ComponentMapper() = default;

  /// Extract component data into vector
  /// @param registry ECS registry containing component data
  /// @param vec Output vector (must be pre-allocated)
  /// @param offset Starting offset in output vector
  /// @return Number of elements written
  virtual size_t toVector(
      const entt::registry& registry,
      std::vector<double>& vec,
      size_t offset) const
      = 0;

  /// Write vector data back to components
  /// @param registry ECS registry to modify
  /// @param vec Input vector containing data
  /// @param offset Starting offset in input vector
  /// @return Number of elements read
  virtual size_t fromVector(
      entt::registry& registry, const std::vector<double>& vec, size_t offset)
      = 0;

  /// Get dimension (number of scalars) this mapper handles
  [[nodiscard]] virtual size_t getDimension() const = 0;
};

/// Mapper for a single scalar variable
/// Useful for simple components with single values
class ScalarMapper : public ComponentMapper
{
public:
  /// Constructor
  /// @param getValue Function to extract value from registry
  /// @param setValue Function to set value in registry
  ScalarMapper(
      std::function<double(const entt::registry&)> getValue,
      std::function<void(entt::registry&, double)> setValue)
    : m_getValue(std::move(getValue)), m_setValue(std::move(setValue))
  {
  }

  size_t toVector(
      const entt::registry& registry,
      std::vector<double>& vec,
      size_t offset) const override
  {
    vec[offset] = m_getValue(registry);
    return 1;
  }

  size_t fromVector(
      entt::registry& registry,
      const std::vector<double>& vec,
      size_t offset) override
  {
    m_setValue(registry, vec[offset]);
    return 1;
  }

  [[nodiscard]] size_t getDimension() const override
  {
    return 1;
  }

private:
  std::function<double(const entt::registry&)> m_getValue;
  std::function<void(entt::registry&, double)> m_setValue;
};

/// Mapper for a vector of scalars from multiple entities
/// Extracts data from all entities that have a specific component
template <typename Component, typename Field>
class FieldMapper : public ComponentMapper
{
public:
  /// Constructor
  /// @param fieldPtr Pointer to member variable in component
  FieldMapper(Field Component::* fieldPtr) : m_fieldPtr(fieldPtr) {}

  size_t toVector(
      const entt::registry& registry,
      std::vector<double>& vec,
      size_t offset) const override
  {
    auto view = registry.view<Component>();
    size_t count = 0;

    for (auto entity : view) {
      const auto& component = view.template get<Component>(entity);
      const Field& field = component.*m_fieldPtr;

      // Handle scalar field
      if constexpr (std::is_arithmetic_v<Field>) {
        vec[offset + count] = static_cast<double>(field);
        ++count;
      }
      // Handle Eigen vectors
      else if constexpr (requires {
                           field.size();
                           field.data();
                         }) {
        for (Eigen::Index i = 0; i < field.size(); ++i) {
          vec[offset + count] = field[i];
          ++count;
        }
      }
    }

    return count;
  }

  size_t fromVector(
      entt::registry& registry,
      const std::vector<double>& vec,
      size_t offset) override
  {
    auto view = registry.view<Component>();
    size_t count = 0;

    for (auto entity : view) {
      auto& component = view.template get<Component>(entity);
      Field& field = component.*m_fieldPtr;

      // Handle scalar field
      if constexpr (std::is_arithmetic_v<Field>) {
        field = static_cast<Field>(vec[offset + count]);
        ++count;
      }
      // Handle Eigen vectors
      else if constexpr (requires {
                           field.size();
                           field.data();
                         }) {
        for (Eigen::Index i = 0; i < field.size(); ++i) {
          field[i] = vec[offset + count];
          ++count;
        }
      }
    }

    return count;
  }

  [[nodiscard]] size_t getDimension() const override
  {
    // Dimension must be computed at runtime based on number of entities
    // For now, return 0 to indicate dynamic sizing
    return 0;
  }

private:
  Field Component::* m_fieldPtr;
};

} // namespace dart::simulation::experimental
