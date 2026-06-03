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

#include <dart/simulation/experimental/comps/component_category.hpp>
#include <dart/simulation/experimental/space/component_mapper.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/pfr.hpp>
#include <entt/entt.hpp>

#include <concepts>
#include <span>
#include <type_traits>

#include <cstddef>

namespace dart::simulation::experimental::space {

namespace detail {

template <typename Field>
concept ArithmeticField = std::integral<std::remove_cvref_t<Field>>
                          || std::floating_point<std::remove_cvref_t<Field>>;

template <typename Field>
concept EnumField = std::is_enum_v<std::remove_cvref_t<Field>>;

template <typename Field>
concept ScalarField = ArithmeticField<Field> || EnumField<Field>;

template <typename Field>
concept AggregateField = std::is_aggregate_v<std::remove_cvref_t<Field>>;

template <typename T>
concept IsometryField = std::same_as<std::remove_cvref_t<T>, Eigen::Isometry3d>
                        || std::derived_from<
                            std::remove_cvref_t<T>,
                            Eigen::Transform<double, 3, Eigen::Isometry>>;

} // namespace detail

/// Automatically extract scalars from a field for vector mapping
/// Handles POD types, Eigen vectors, and nested structs
template <typename Field>
size_t extractFieldToVector(
    const Field& field, std::vector<double>& vec, size_t offset)
{
  using FieldType = std::remove_cvref_t<Field>;
  size_t count = 0;

  if constexpr (detail::ArithmeticField<FieldType>) {
    // Scalar types (double, float, int, etc.)
    vec[offset] = static_cast<double>(field);
    count = 1;
  } else if constexpr (detail::IsometryField<FieldType>) {
    // Extract translation (3) + rotation as quaternion (4)
    // Handle early to avoid confusion with other Eigen types
    auto translation = field.translation();
    Eigen::Quaterniond rotation(field.rotation());
    vec[offset + 0] = translation.x();
    vec[offset + 1] = translation.y();
    vec[offset + 2] = translation.z();
    vec[offset + 3] = rotation.w();
    vec[offset + 4] = rotation.x();
    vec[offset + 5] = rotation.y();
    vec[offset + 6] = rotation.z();
    count = 7;
  } else if constexpr (std::same_as<FieldType, Eigen::Vector2d>) {
    vec[offset + 0] = field.x();
    vec[offset + 1] = field.y();
    count = 2;
  } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
    vec[offset + 0] = field.x();
    vec[offset + 1] = field.y();
    vec[offset + 2] = field.z();
    count = 3;
  } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
    for (Eigen::Index i = 0; i < field.size(); ++i) {
      vec[offset + i] = field[i];
    }
    count = field.size();
  } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
    vec[offset + 0] = field.w();
    vec[offset + 1] = field.x();
    vec[offset + 2] = field.y();
    vec[offset + 3] = field.z();
    count = 4;
  } else if constexpr (detail::EnumField<FieldType>) {
    // Enums as integers
    vec[offset] = static_cast<double>(static_cast<int>(field));
    count = 1;
  } else {
    // Nested struct - recursively extract all fields
    boost::pfr::for_each_field(field, [&](const auto& nestedField) {
      count += extractFieldToVector(nestedField, vec, offset + count);
    });
  }

  return count;
}

/// Automatically inject scalars from vector into a field
template <typename Field>
size_t injectVectorToField(
    Field& field, std::span<const double> vec, size_t offset)
{
  using FieldType = std::remove_cvref_t<Field>;
  size_t count = 0;

  if constexpr (detail::ArithmeticField<FieldType>) {
    field = static_cast<FieldType>(vec[offset]);
    count = 1;
  } else if constexpr (detail::IsometryField<FieldType>) {
    // Handle Isometry3d (before other Eigen types)
    Eigen::Vector3d translation(
        vec[offset + 0], vec[offset + 1], vec[offset + 2]);
    Eigen::Quaterniond rotation(
        vec[offset + 3], vec[offset + 4], vec[offset + 5], vec[offset + 6]);
    field.translation() = translation;
    field.linear() = rotation.toRotationMatrix();
    count = 7;
  } else if constexpr (std::same_as<FieldType, Eigen::Vector2d>) {
    field.x() = vec[offset + 0];
    field.y() = vec[offset + 1];
    count = 2;
  } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
    field.x() = vec[offset + 0];
    field.y() = vec[offset + 1];
    field.z() = vec[offset + 2];
    count = 3;
  } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
    for (Eigen::Index i = 0; i < field.size(); ++i) {
      field[i] = vec[offset + i];
    }
    count = field.size();
  } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
    field.w() = vec[offset + 0];
    field.x() = vec[offset + 1];
    field.y() = vec[offset + 2];
    field.z() = vec[offset + 3];
    count = 4;
  } else if constexpr (detail::EnumField<FieldType>) {
    field = static_cast<FieldType>(static_cast<int>(vec[offset]));
    count = 1;
  } else {
    // Nested struct - recursively inject to all fields
    boost::pfr::for_each_field(field, [&](auto& nestedField) {
      count += injectVectorToField(nestedField, vec, offset + count);
    });
  }

  return count;
}

/// Compute dimension of a field (how many scalars it contains)
template <typename Field>
constexpr size_t getFieldDimension()
{
  using FieldType = std::remove_cvref_t<Field>;

  if constexpr (detail::ScalarField<FieldType>) {
    return 1;
  } else if constexpr (std::same_as<FieldType, Eigen::Vector2d>) {
    return 2;
  } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
    return 3;
  } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
    return 4;
  } else if constexpr (detail::IsometryField<FieldType>) {
    return 7; // Translation (3) + Quaternion (4)
  } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
    return 0; // Dynamic size - must be computed at runtime
  } else if constexpr (detail::AggregateField<FieldType>) {
    // Nested aggregate struct - sum dimensions of all fields
    size_t total = 0;
    boost::pfr::for_each_field(FieldType{}, [&](const auto& field) {
      total += getFieldDimension<decltype(field)>();
    });
    return total;
  } else {
    // Non-aggregate type we don't know how to handle
    return 0;
  }
}

//==============================================================================
// Automatic Component Mapper for PropertyComponents
//==============================================================================

/// Automatically generated mapper for PropertyComponents
/// Uses Boost.PFR to extract all fields to/from vectors
template <comps::IsPropertyComponent Component>
class AutoPropertyMapper : public ComponentMapper
{
public:
  size_t toVector(
      const entt::registry& registry,
      std::vector<double>& vec,
      size_t offset) const override
  {
    return toVectorImpl(registry, vec, offset);
  }

  size_t toVector(
      const ::dart::simulation::experimental::detail::WorldRegistry& registry,
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
      ::dart::simulation::experimental::detail::WorldRegistry& registry,
      std::span<const double> vec,
      size_t offset) override
  {
    return fromVectorImpl(registry, vec, offset);
  }

  [[nodiscard]] size_t getDimension() const override
  {
    // Compute dimension from component structure
    size_t total = 0;
    boost::pfr::for_each_field(Component{}, [&](const auto& field) {
      total += getFieldDimension<decltype(field)>();
    });
    return total;
  }

private:
  template <typename Registry>
  size_t toVectorImpl(
      const Registry& registry, std::vector<double>& vec, size_t offset) const
  {
    auto view = registry.template view<Component>();
    size_t totalCount = 0;

    for (auto entity : view) {
      const auto& component = view.template get<Component>(entity);

      // Extract all fields using Boost.PFR
      boost::pfr::for_each_field(component, [&](const auto& field) {
        totalCount += extractFieldToVector(field, vec, offset + totalCount);
      });
    }

    return totalCount;
  }

  template <typename Registry>
  size_t fromVectorImpl(
      Registry& registry, std::span<const double> vec, size_t offset)
  {
    auto view = registry.template view<Component>();
    size_t totalCount = 0;

    for (auto entity : view) {
      auto& component = view.template get<Component>(entity);

      // Inject to all fields using Boost.PFR
      boost::pfr::for_each_field(component, [&](auto& field) {
        totalCount += injectVectorToField(field, vec, offset + totalCount);
      });
    }

    return totalCount;
  }
};

//==============================================================================
// Factory function to create mappers automatically
//==============================================================================

/// Create an automatic mapper for a PropertyComponent
template <comps::IsPropertyComponent Component>
std::unique_ptr<ComponentMapper> makeAutoMapper()
{
  return std::make_unique<AutoPropertyMapper<Component>>();
}

} // namespace dart::simulation::experimental::space
