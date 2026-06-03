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

#include <dart/simulation/experimental/detail/world_registry_types.hpp>

#include <Eigen/Core>
#include <entt/entt.hpp>

#include <concepts>
#include <functional>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace dart::simulation::experimental {

namespace detail {

template <typename Field>
concept ArithmeticField = std::integral<std::remove_cvref_t<Field>>
                          || std::floating_point<std::remove_cvref_t<Field>>;

template <typename Field>
concept EigenVectorField = requires(Field field, Eigen::Index i) {
  { field.size() } -> std::convertible_to<Eigen::Index>;
  field.data();
  field[i];
};

} // namespace detail

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
      size_t offset) const = 0;

  /// Write vector data back to components
  /// @param registry ECS registry to modify
  /// @param vec Input vector containing data
  /// @param offset Starting offset in input vector
  /// @return Number of elements read
  virtual size_t fromVector(
      entt::registry& registry, std::span<const double> vec, size_t offset) = 0;

  /// Get dimension (number of scalars) this mapper handles
  [[nodiscard]] virtual size_t getDimension() const = 0;
};

/// Optional interface for mappers that support World-owned registries.
///
/// Custom ComponentMapper implementations that should operate on
/// detail::WorldRegistry must also implement this interface. Keeping it
/// separate preserves the existing ComponentMapper contract for plain
/// entt::registry users while making allocator-aware World registry support
/// explicit.
class WorldRegistryComponentMapper
{
public:
  virtual ~WorldRegistryComponentMapper() = default;

  /// Extract component data from a World-owned allocator-aware registry.
  virtual size_t toVector(
      const detail::WorldRegistry& registry,
      std::vector<double>& vec,
      size_t offset) const = 0;

  /// Write vector data back to a World-owned allocator-aware registry.
  virtual size_t fromVector(
      detail::WorldRegistry& registry,
      std::span<const double> vec,
      size_t offset) = 0;
};

/// Mapper for a single scalar variable
/// Useful for simple components with single values
class ScalarMapper : public ComponentMapper, public WorldRegistryComponentMapper
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

  /// Constructor for World-owned allocator-aware registries.
  ScalarMapper(
      std::function<double(const detail::WorldRegistry&)> getValue,
      std::function<void(detail::WorldRegistry&, double)> setValue)
    : m_getWorldValue(std::move(getValue)), m_setWorldValue(std::move(setValue))
  {
  }

  size_t toVector(
      const entt::registry& registry,
      std::vector<double>& vec,
      size_t offset) const override
  {
    if (!m_getValue) {
      throw std::invalid_argument(
          "ScalarMapper does not support entt::registry extraction");
    }

    vec[offset] = m_getValue(registry);
    return 1;
  }

  size_t fromVector(
      entt::registry& registry,
      std::span<const double> vec,
      size_t offset) override
  {
    if (!m_setValue) {
      throw std::invalid_argument(
          "ScalarMapper does not support entt::registry injection");
    }

    m_setValue(registry, vec[offset]);
    return 1;
  }

  size_t toVector(
      const detail::WorldRegistry& registry,
      std::vector<double>& vec,
      size_t offset) const override
  {
    if (!m_getWorldValue) {
      throw std::invalid_argument(
          "ScalarMapper does not support WorldRegistry extraction");
    }

    vec[offset] = m_getWorldValue(registry);
    return 1;
  }

  size_t fromVector(
      detail::WorldRegistry& registry,
      std::span<const double> vec,
      size_t offset) override
  {
    if (!m_setWorldValue) {
      throw std::invalid_argument(
          "ScalarMapper does not support WorldRegistry injection");
    }

    m_setWorldValue(registry, vec[offset]);
    return 1;
  }

  [[nodiscard]] size_t getDimension() const override
  {
    return 1;
  }

private:
  std::function<double(const entt::registry&)> m_getValue;
  std::function<void(entt::registry&, double)> m_setValue;
  std::function<double(const detail::WorldRegistry&)> m_getWorldValue;
  std::function<void(detail::WorldRegistry&, double)> m_setWorldValue;
};

/// Mapper for a vector of scalars from multiple entities
/// Extracts data from all entities that have a specific component
template <typename Component, typename Field>
class FieldMapper : public ComponentMapper, public WorldRegistryComponentMapper
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
    return toVectorImpl(registry, vec, offset);
  }

  size_t toVector(
      const detail::WorldRegistry& registry,
      std::vector<double>& vec,
      size_t offset) const override
  {
    return toVectorImpl(registry, vec, offset);
  }

  size_t fromVector(
      entt::registry& registry,
      std::span<const double> vec,
      size_t offset) override
  {
    return fromVectorImpl(registry, vec, offset);
  }

  size_t fromVector(
      detail::WorldRegistry& registry,
      std::span<const double> vec,
      size_t offset) override
  {
    return fromVectorImpl(registry, vec, offset);
  }

  [[nodiscard]] size_t getDimension() const override
  {
    // Dimension must be computed at runtime based on number of entities
    // For now, return 0 to indicate dynamic sizing
    return 0;
  }

private:
  template <typename Registry>
  size_t toVectorImpl(
      const Registry& registry, std::vector<double>& vec, size_t offset) const
  {
    auto view = registry.template view<Component>();
    size_t count = 0;

    for (auto entity : view) {
      const auto& component = view.template get<Component>(entity);
      const Field& field = component.*m_fieldPtr;

      // Handle scalar field
      if constexpr (detail::ArithmeticField<Field>) {
        vec[offset + count] = static_cast<double>(field);
        ++count;
      }
      // Handle Eigen vectors
      else if constexpr (detail::EigenVectorField<Field>) {
        for (Eigen::Index i = 0; i < field.size(); ++i) {
          vec[offset + count] = field[i];
          ++count;
        }
      }
    }

    return count;
  }

  template <typename Registry>
  size_t fromVectorImpl(
      Registry& registry, std::span<const double> vec, size_t offset)
  {
    auto view = registry.template view<Component>();
    size_t count = 0;

    for (auto entity : view) {
      auto& component = view.template get<Component>(entity);
      Field& field = component.*m_fieldPtr;

      // Handle scalar field
      if constexpr (detail::ArithmeticField<Field>) {
        field = static_cast<Field>(vec[offset + count]);
        ++count;
      }
      // Handle Eigen vectors
      else if constexpr (detail::EigenVectorField<Field>) {
        for (Eigen::Index i = 0; i < field.size(); ++i) {
          field[i] = vec[offset + count];
          ++count;
        }
      }
    }

    return count;
  }

  Field Component::* m_fieldPtr;
};

} // namespace dart::simulation::experimental
