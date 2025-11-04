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

#include <dart7/comps/component_category.hpp>
#include <dart7/io/binary_io.hpp>

#include <boost/pfr.hpp>

#include <iostream>
#include <type_traits>

namespace dart7::io {

//==============================================================================
// Forward declarations for nested non-component types
//==============================================================================

template <typename T>
requires(!comps::HasComponentCategory<T>) void autoSerialize(
    std::ostream& out, const T& value, const EntityMap& entityMap);

template <typename T>
requires(!comps::HasComponentCategory<T>) void autoDeserialize(
    std::istream& in, T& value);

//==============================================================================
// Automatic Serialization for TagComponent (empty components)
//==============================================================================

template <comps::IsTagComponent T>
void autoSerialize(
    std::ostream& /*out*/,
    const T& /*component*/,
    const EntityMap& /*entityMap*/)
{
  // Tag components are empty - nothing to serialize
}

template <comps::IsTagComponent T>
void autoDeserialize(std::istream& /*in*/, T& /*component*/)
{
  // Tag components are empty - nothing to deserialize
}

//==============================================================================
// Automatic Serialization for PropertyComponent (simple data)
//
// Uses Boost.PFR to automatically serialize all fields
//==============================================================================

template <comps::IsPropertyComponent T>
void autoSerialize(
    std::ostream& out, const T& component, const EntityMap& /*entityMap*/)
{
  // Use Boost.PFR to iterate over all fields
  boost::pfr::for_each_field(component, [&out](const auto& field) {
    using FieldType = std::remove_cvref_t<decltype(field)>;

    // Handle different field types
    if constexpr (std::is_same_v<FieldType, std::string>) {
      writeString(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
      writeVector3d(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Isometry3d>) {
      writeIsometry3d(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Quaterniond>) {
      writePOD(out, field.w());
      writePOD(out, field.x());
      writePOD(out, field.y());
      writePOD(out, field.z());
    } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
      writeVectorXd(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Matrix3d>) {
      // Write 3x3 matrix (symmetric, so store 6 unique values)
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          writePOD(out, field(i, j));
        }
      }
    } else if constexpr (std::is_trivially_copyable_v<FieldType>) {
      // POD types (int, double, bool, enums, etc.)
      writePOD(out, field);
    } else {
      // Complex types - recursively serialize using PFR
      autoSerialize(out, field, EntityMap{});
    }
  });
}

template <comps::IsPropertyComponent T>
void autoDeserialize(std::istream& in, T& component)
{
  // Use Boost.PFR to iterate over all fields
  boost::pfr::for_each_field(component, [&in](auto& field) {
    using FieldType = std::remove_cvref_t<decltype(field)>;

    // Handle different field types
    if constexpr (std::is_same_v<FieldType, std::string>) {
      readString(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
      readVector3d(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Isometry3d>) {
      readIsometry3d(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Quaterniond>) {
      double w, x, y, z;
      readPOD(in, w);
      readPOD(in, x);
      readPOD(in, y);
      readPOD(in, z);
      field = Eigen::Quaterniond(w, x, y, z);
    } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
      readVectorXd(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Matrix3d>) {
      // Read 3x3 symmetric matrix
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          readPOD(in, field(i, j));
          if (i != j) {
            field(j, i) = field(i, j); // Symmetric
          }
        }
      }
    } else if constexpr (std::is_trivially_copyable_v<FieldType>) {
      // POD types
      readPOD(in, field);
    } else {
      // Complex types - recursively deserialize
      autoDeserialize(in, field);
    }
  });
}

//==============================================================================
// Automatic Serialization for StateComponent (with entity remapping)
//
// Uses Boost.PFR to automatically serialize all fields, with special handling
// for entity references marked in entityFields()
//==============================================================================

template <comps::IsStateComponent T>
void autoSerialize(
    std::ostream& out, const T& component, const EntityMap& entityMap)
{
  // Serialize all fields - entity fields get mapped automatically
  boost::pfr::for_each_field(component, [&](const auto& field) {
    using FieldType = std::remove_cvref_t<decltype(field)>;

    if constexpr (std::is_same_v<FieldType, entt::entity>) {
      // Single entity field - always remap
      std::uint32_t mappedId
          = (field != entt::null)
                ? static_cast<std::uint32_t>(entityMap.at(field))
                : static_cast<std::uint32_t>(entt::null);
      writePOD(out, mappedId);
    } else if constexpr (std::is_same_v<FieldType, std::vector<entt::entity>>) {
      // Vector of entities - always remap each
      std::size_t count = field.size();
      writePOD(out, count);
      for (const auto& entity : field) {
        std::uint32_t mappedId
            = (entity != entt::null)
                  ? static_cast<std::uint32_t>(entityMap.at(entity))
                  : static_cast<std::uint32_t>(entt::null);
        writePOD(out, mappedId);
      }
    } else if constexpr (std::is_same_v<FieldType, std::string>) {
      writeString(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
      writeVector3d(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Isometry3d>) {
      writeIsometry3d(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
      writeVectorXd(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Matrix3d>) {
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          writePOD(out, field(i, j));
        }
      }
    } else if constexpr (std::is_same_v<FieldType, Eigen::Quaterniond>) {
      writePOD(out, field.w());
      writePOD(out, field.x());
      writePOD(out, field.y());
      writePOD(out, field.z());
    } else if constexpr (std::is_trivially_copyable_v<FieldType>) {
      writePOD(out, field);
    } else {
      // Complex nested type
      autoSerialize(out, field, entityMap);
    }
  });
}

template <comps::IsStateComponent T>
void autoDeserialize(std::istream& in, T& component)
{
  // Deserialize all fields (entity remapping happens in second pass)
  boost::pfr::for_each_field(component, [&](auto& field) {
    using FieldType = std::remove_cvref_t<decltype(field)>;

    if constexpr (std::is_same_v<FieldType, entt::entity>) {
      // Read entity ID (will be remapped in second pass)
      std::uint32_t entityId;
      readPOD(in, entityId);
      field = static_cast<entt::entity>(entityId);
    } else if constexpr (std::is_same_v<FieldType, std::vector<entt::entity>>) {
      // Read vector of entities
      std::size_t count;
      readPOD(in, count);
      field.resize(count);
      for (auto& entity : field) {
        std::uint32_t entityId;
        readPOD(in, entityId);
        entity = static_cast<entt::entity>(entityId);
      }
    } else if constexpr (std::is_same_v<FieldType, std::string>) {
      readString(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
      readVector3d(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Isometry3d>) {
      readIsometry3d(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
      readVectorXd(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Matrix3d>) {
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          readPOD(in, field(i, j));
          if (i != j) {
            field(j, i) = field(i, j);
          }
        }
      }
    } else if constexpr (std::is_trivially_copyable_v<FieldType>) {
      readPOD(in, field);
    } else {
      // Complex nested type
      autoDeserialize(in, field);
    }
  });
}

//==============================================================================
// Unconstrained overloads for nested non-component types
// (e.g., JointLimits nested in Joint)
//==============================================================================

template <typename T>
requires(!comps::HasComponentCategory<T>) void autoSerialize(
    std::ostream& out, const T& value, const EntityMap& /*entityMap*/)
{
  // Use Boost.PFR to serialize nested non-component struct
  boost::pfr::for_each_field(value, [&](const auto& field) {
    using FieldType = std::remove_cvref_t<decltype(field)>;

    if constexpr (std::is_same_v<FieldType, std::string>) {
      writeString(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
      writeVectorXd(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
      writeVector3d(out, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Matrix3d>) {
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          writePOD(out, field(i, j));
        }
      }
    } else if constexpr (std::is_trivially_copyable_v<FieldType>) {
      writePOD(out, field);
    } else {
      // Recursively serialize nested structs
      autoSerialize(out, field, EntityMap{});
    }
  });
}

template <typename T>
requires(!comps::HasComponentCategory<T>) void autoDeserialize(
    std::istream& in, T& value)
{
  // Use Boost.PFR to deserialize nested non-component struct
  boost::pfr::for_each_field(value, [&](auto& field) {
    using FieldType = std::remove_cvref_t<decltype(field)>;

    if constexpr (std::is_same_v<FieldType, std::string>) {
      readString(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::VectorXd>) {
      readVectorXd(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Vector3d>) {
      readVector3d(in, field);
    } else if constexpr (std::is_same_v<FieldType, Eigen::Matrix3d>) {
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          readPOD(in, field(i, j));
          if (i != j) {
            field(j, i) = field(i, j);
          }
        }
      }
    } else if constexpr (std::is_trivially_copyable_v<FieldType>) {
      readPOD(in, field);
    } else {
      // Recursively deserialize nested structs
      autoDeserialize(in, field);
    }
  });
}

} // namespace dart7::io
