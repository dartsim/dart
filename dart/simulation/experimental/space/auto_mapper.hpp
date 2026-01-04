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

#include <dart/simulation/experimental/comps/component_category.hpp>
#include <dart/simulation/experimental/space/component_mapper.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/pfr.hpp>
#include <entt/entt.hpp>

#include <type_traits>

#include <cstddef>

namespace dart::simulation::experimental::space {

/// Automatically extract scalars from a field for vector mapping
/// Handles POD types, Eigen vectors, and nested structs
template <typename Field>
size_t extractFieldToVector(
    const Field& field, std::vector<double>& vec, size_t offset)
{
  using FieldType = std::remove_cvref_t<Field>;
  size_t count = 0;

  if constexpr (std::is_arithmetic_v<FieldType>) {
    // Scalar types (double, float, int, etc.)
    vec[offset] = static_cast<double>(field);
    count = 1;
  } else if constexpr (
      std::is_same_v<
          FieldType,
          Eigen::
              Isometry3d> || std::is_base_of_v<Eigen::Transform<double, 3, Eigen::Isometry>, FieldType>) {
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
  } else if constexpr (std::is_same_v<FieldType, Eigen::Vector2d>) {
    vec[offset + 0] = field.x();
    vec[offset + 1] = field.y();
    count = 2;
  } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
    vec[offset + 0] = field.x();
    vec[offset + 1] = field.y();
    vec[offset + 2] = field.z();
    count = 3;
  } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
    for (Eigen::Index i = 0; i < field.size(); ++i) {
      vec[offset + i] = field[i];
    }
    count = field.size();
  } else if constexpr (std::is_same_v<FieldType, Eigen::Quaterniond>) {
    vec[offset + 0] = field.w();
    vec[offset + 1] = field.x();
    vec[offset + 2] = field.y();
    vec[offset + 3] = field.z();
    count = 4;
  } else if constexpr (std::is_enum_v<FieldType>) {
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
    Field& field, const std::vector<double>& vec, size_t offset)
{
  using FieldType = std::remove_cvref_t<Field>;
  size_t count = 0;

  if constexpr (std::is_arithmetic_v<FieldType>) {
    field = static_cast<FieldType>(vec[offset]);
    count = 1;
  } else if constexpr (
      std::is_same_v<
          FieldType,
          Eigen::
              Isometry3d> || std::is_base_of_v<Eigen::Transform<double, 3, Eigen::Isometry>, FieldType>) {
    // Handle Isometry3d (before other Eigen types)
    Eigen::Vector3d translation(
        vec[offset + 0], vec[offset + 1], vec[offset + 2]);
    Eigen::Quaterniond rotation(
        vec[offset + 3], vec[offset + 4], vec[offset + 5], vec[offset + 6]);
    field.translation() = translation;
    field.linear() = rotation.toRotationMatrix();
    count = 7;
  } else if constexpr (std::is_same_v<FieldType, Eigen::Vector2d>) {
    field.x() = vec[offset + 0];
    field.y() = vec[offset + 1];
    count = 2;
  } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
    field.x() = vec[offset + 0];
    field.y() = vec[offset + 1];
    field.z() = vec[offset + 2];
    count = 3;
  } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
    for (Eigen::Index i = 0; i < field.size(); ++i) {
      field[i] = vec[offset + i];
    }
    count = field.size();
  } else if constexpr (std::is_same_v<FieldType, Eigen::Quaterniond>) {
    field.w() = vec[offset + 0];
    field.x() = vec[offset + 1];
    field.y() = vec[offset + 2];
    field.z() = vec[offset + 3];
    count = 4;
  } else if constexpr (std::is_enum_v<FieldType>) {
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

  if constexpr (std::is_arithmetic_v<FieldType> || std::is_enum_v<FieldType>) {
    return 1;
  } else if constexpr (std::is_same_v<FieldType, Eigen::Vector2d>) {
    return 2;
  } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
    return 3;
  } else if constexpr (std::is_same_v<FieldType, Eigen::Quaterniond>) {
    return 4;
  } else if constexpr (std::is_same_v<FieldType, Eigen::Isometry3d>) {
    return 7; // Translation (3) + Quaternion (4)
  } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
    return 0; // Dynamic size - must be computed at runtime
  } else if constexpr (std::is_aggregate_v<FieldType>) {
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
    auto view = registry.view<Component>();
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

  size_t fromVector(
      entt::registry& registry,
      const std::vector<double>& vec,
      size_t offset) override
  {
    auto view = registry.view<Component>();
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

  [[nodiscard]] size_t getDimension() const override
  {
    // Compute dimension from component structure
    size_t total = 0;
    boost::pfr::for_each_field(Component{}, [&](const auto& field) {
      total += getFieldDimension<decltype(field)>();
    });
    return total;
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
