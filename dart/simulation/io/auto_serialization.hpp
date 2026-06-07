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

#include <dart/simulation/comps/component_category.hpp>
#include <dart/simulation/io/binary_io.hpp>

#include <boost/pfr.hpp>

#include <concepts>
#include <iostream>
#include <type_traits>

namespace dart::simulation::io {

//==============================================================================
// Forward declarations for nested non-component types
//==============================================================================

template <typename T>
  requires(!comps::HasComponentCategory<T>)
void autoSerialize(
    std::ostream& out, const T& value, const EntityMap& entityMap);

template <typename T>
  requires(!comps::HasComponentCategory<T>)
void autoDeserialize(std::istream& in, T& value);

namespace detail {

template <typename T>
inline constexpr bool IsVector3dList
    = std::same_as<T, std::vector<Eigen::Vector3d>>;

template <typename T>
inline constexpr bool IsVector3iList
    = std::same_as<T, std::vector<Eigen::Vector3i>>;

inline void writeVector3i(std::ostream& out, const Eigen::Vector3i& vec)
{
  writePOD(out, vec.x());
  writePOD(out, vec.y());
  writePOD(out, vec.z());
}

inline void readVector3i(std::istream& in, Eigen::Vector3i& vec)
{
  int x = 0;
  int y = 0;
  int z = 0;
  readPOD(in, x);
  readPOD(in, y);
  readPOD(in, z);
  vec = Eigen::Vector3i(x, y, z);
}

inline void writeVector3dList(
    std::ostream& out, const std::vector<Eigen::Vector3d>& values)
{
  const std::size_t count = values.size();
  writePOD(out, count);
  for (const Eigen::Vector3d& value : values) {
    writeVector3d(out, value);
  }
}

inline void readVector3dList(
    std::istream& in, std::vector<Eigen::Vector3d>& values)
{
  std::size_t count = 0;
  readPOD(in, count);
  values.resize(count);
  for (Eigen::Vector3d& value : values) {
    readVector3d(in, value);
  }
}

inline void writeVector3iList(
    std::ostream& out, const std::vector<Eigen::Vector3i>& values)
{
  const std::size_t count = values.size();
  writePOD(out, count);
  for (const Eigen::Vector3i& value : values) {
    writeVector3i(out, value);
  }
}

inline void readVector3iList(
    std::istream& in, std::vector<Eigen::Vector3i>& values)
{
  std::size_t count = 0;
  readPOD(in, count);
  values.resize(count);
  for (Eigen::Vector3i& value : values) {
    readVector3i(in, value);
  }
}

// A list of trivially-copyable elements (e.g. std::vector<std::size_t>),
// serialized as a count followed by the raw POD elements. Excludes the Eigen
// vector lists above, which keep their own dedicated packed layout.
template <typename T>
struct IsTrivialVectorHelper : std::false_type
{
};
template <typename U>
struct IsTrivialVectorHelper<std::vector<U>>
  : std::bool_constant<std::is_trivially_copyable_v<U>>
{
};

template <typename T>
inline constexpr bool IsTrivialList
    = IsTrivialVectorHelper<T>::value
      && !IsVector3dList<T> && !IsVector3iList<T>;

template <typename U>
inline void writeTrivialList(std::ostream& out, const std::vector<U>& values)
{
  const std::size_t count = values.size();
  writePOD(out, count);
  for (const U& value : values) {
    writePOD(out, value);
  }
}

template <typename U>
inline void readTrivialList(std::istream& in, std::vector<U>& values)
{
  std::size_t count = 0;
  readPOD(in, count);
  values.resize(count);
  for (U& value : values) {
    readPOD(in, value);
  }
}

} // namespace detail

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
    if constexpr (std::same_as<FieldType, std::string>) {
      writeString(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
      writeVector3d(out, field);
    } else if constexpr (detail::IsVector3dList<FieldType>) {
      detail::writeVector3dList(out, field);
    } else if constexpr (detail::IsVector3iList<FieldType>) {
      detail::writeVector3iList(out, field);
    } else if constexpr (detail::IsTrivialList<FieldType>) {
      detail::writeTrivialList(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Isometry3d>) {
      writeIsometry3d(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
      writePOD(out, field.w());
      writePOD(out, field.x());
      writePOD(out, field.y());
      writePOD(out, field.z());
    } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
      writeVectorXd(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Matrix3d>) {
      // Write 3x3 matrix (symmetric, so store 6 unique values)
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          writePOD(out, field(i, j));
        }
      }
    } else if constexpr (detail::TriviallyCopyable<FieldType>) {
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
    if constexpr (std::same_as<FieldType, std::string>) {
      readString(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
      readVector3d(in, field);
    } else if constexpr (detail::IsVector3dList<FieldType>) {
      detail::readVector3dList(in, field);
    } else if constexpr (detail::IsVector3iList<FieldType>) {
      detail::readVector3iList(in, field);
    } else if constexpr (detail::IsTrivialList<FieldType>) {
      detail::readTrivialList(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Isometry3d>) {
      readIsometry3d(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
      double w, x, y, z;
      readPOD(in, w);
      readPOD(in, x);
      readPOD(in, y);
      readPOD(in, z);
      field = Eigen::Quaterniond(w, x, y, z);
    } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
      readVectorXd(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Matrix3d>) {
      // Read 3x3 symmetric matrix
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          readPOD(in, field(i, j));
          if (i != j) {
            field(j, i) = field(i, j); // Symmetric
          }
        }
      }
    } else if constexpr (detail::TriviallyCopyable<FieldType>) {
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

    if constexpr (std::same_as<FieldType, entt::entity>) {
      // Single entity field - always remap
      std::uint32_t mappedId
          = (field != entt::null)
                ? static_cast<std::uint32_t>(entityMap.at(field))
                : static_cast<std::uint32_t>(entt::null);
      writePOD(out, mappedId);
    } else if constexpr (std::same_as<FieldType, std::vector<entt::entity>>) {
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
    } else if constexpr (std::same_as<FieldType, std::string>) {
      writeString(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
      writeVector3d(out, field);
    } else if constexpr (detail::IsVector3dList<FieldType>) {
      detail::writeVector3dList(out, field);
    } else if constexpr (detail::IsVector3iList<FieldType>) {
      detail::writeVector3iList(out, field);
    } else if constexpr (detail::IsTrivialList<FieldType>) {
      detail::writeTrivialList(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Isometry3d>) {
      writeIsometry3d(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
      writeVectorXd(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Matrix3d>) {
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          writePOD(out, field(i, j));
        }
      }
    } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
      writePOD(out, field.w());
      writePOD(out, field.x());
      writePOD(out, field.y());
      writePOD(out, field.z());
    } else if constexpr (std::same_as<FieldType, Eigen::Matrix<double, 6, 1>>) {
      for (int i = 0; i < 6; ++i) {
        writePOD(out, field[i]);
      }
    } else if constexpr (
        std::same_as<FieldType, std::vector<Eigen::Isometry3d>>) {
      writePOD(out, field.size());
      for (const auto& transform : field) {
        writeIsometry3d(out, transform);
      }
    } else if constexpr (
        std::same_as<FieldType, std::vector<Eigen::Matrix<double, 6, 1>>>) {
      writePOD(out, field.size());
      for (const auto& vector6 : field) {
        for (int i = 0; i < 6; ++i) {
          writePOD(out, vector6[i]);
        }
      }
    } else if constexpr (detail::TriviallyCopyable<FieldType>) {
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

    if constexpr (std::same_as<FieldType, entt::entity>) {
      // Read entity ID (will be remapped in second pass)
      std::uint32_t entityId;
      readPOD(in, entityId);
      field = static_cast<entt::entity>(entityId);
    } else if constexpr (std::same_as<FieldType, std::vector<entt::entity>>) {
      // Read vector of entities
      std::size_t count;
      readPOD(in, count);
      field.resize(count);
      for (auto& entity : field) {
        std::uint32_t entityId;
        readPOD(in, entityId);
        entity = static_cast<entt::entity>(entityId);
      }
    } else if constexpr (std::same_as<FieldType, std::string>) {
      readString(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
      readVector3d(in, field);
    } else if constexpr (detail::IsVector3dList<FieldType>) {
      detail::readVector3dList(in, field);
    } else if constexpr (detail::IsVector3iList<FieldType>) {
      detail::readVector3iList(in, field);
    } else if constexpr (detail::IsTrivialList<FieldType>) {
      detail::readTrivialList(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Isometry3d>) {
      readIsometry3d(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
      double w = 1.0;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      readPOD(in, w);
      readPOD(in, x);
      readPOD(in, y);
      readPOD(in, z);
      field = Eigen::Quaterniond(w, x, y, z);
    } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
      readVectorXd(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Matrix3d>) {
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          readPOD(in, field(i, j));
          if (i != j) {
            field(j, i) = field(i, j);
          }
        }
      }
    } else if constexpr (std::same_as<FieldType, Eigen::Matrix<double, 6, 1>>) {
      for (int i = 0; i < 6; ++i) {
        readPOD(in, field[i]);
      }
    } else if constexpr (
        std::same_as<FieldType, std::vector<Eigen::Isometry3d>>) {
      std::size_t count = 0;
      readPOD(in, count);
      field.resize(count);
      for (auto& transform : field) {
        readIsometry3d(in, transform);
      }
    } else if constexpr (
        std::same_as<FieldType, std::vector<Eigen::Matrix<double, 6, 1>>>) {
      std::size_t count = 0;
      readPOD(in, count);
      field.resize(count);
      for (auto& vector6 : field) {
        for (int i = 0; i < 6; ++i) {
          readPOD(in, vector6[i]);
        }
      }
    } else if constexpr (detail::TriviallyCopyable<FieldType>) {
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
  requires(!comps::HasComponentCategory<T>)
void autoSerialize(
    std::ostream& out, const T& value, const EntityMap& /*entityMap*/)
{
  // Use Boost.PFR to serialize nested non-component struct
  boost::pfr::for_each_field(value, [&](const auto& field) {
    using FieldType = std::remove_cvref_t<decltype(field)>;

    if constexpr (std::same_as<FieldType, std::string>) {
      writeString(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
      writeVectorXd(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
      writeVector3d(out, field);
    } else if constexpr (detail::IsVector3dList<FieldType>) {
      detail::writeVector3dList(out, field);
    } else if constexpr (detail::IsVector3iList<FieldType>) {
      detail::writeVector3iList(out, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Matrix3d>) {
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          writePOD(out, field(i, j));
        }
      }
    } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
      writePOD(out, field.w());
      writePOD(out, field.x());
      writePOD(out, field.y());
      writePOD(out, field.z());
    } else if constexpr (detail::TriviallyCopyable<FieldType>) {
      writePOD(out, field);
    } else {
      // Recursively serialize nested structs
      autoSerialize(out, field, EntityMap{});
    }
  });
}

template <typename T>
  requires(!comps::HasComponentCategory<T>)
void autoDeserialize(std::istream& in, T& value)
{
  // Use Boost.PFR to deserialize nested non-component struct
  boost::pfr::for_each_field(value, [&](auto& field) {
    using FieldType = std::remove_cvref_t<decltype(field)>;

    if constexpr (std::same_as<FieldType, std::string>) {
      readString(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::VectorXd>) {
      readVectorXd(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Vector3d>) {
      readVector3d(in, field);
    } else if constexpr (detail::IsVector3dList<FieldType>) {
      detail::readVector3dList(in, field);
    } else if constexpr (detail::IsVector3iList<FieldType>) {
      detail::readVector3iList(in, field);
    } else if constexpr (std::same_as<FieldType, Eigen::Matrix3d>) {
      for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
          readPOD(in, field(i, j));
          if (i != j) {
            field(j, i) = field(i, j);
          }
        }
      }
    } else if constexpr (std::same_as<FieldType, Eigen::Quaterniond>) {
      double w = 1.0;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      readPOD(in, w);
      readPOD(in, x);
      readPOD(in, y);
      readPOD(in, z);
      field = Eigen::Quaterniond(w, x, y, z);
    } else if constexpr (detail::TriviallyCopyable<FieldType>) {
      readPOD(in, field);
    } else {
      // Recursively deserialize nested structs
      autoDeserialize(in, field);
    }
  });
}

} // namespace dart::simulation::io
