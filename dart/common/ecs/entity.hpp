/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/common/Fwd.hpp>

#include <cstdint>

namespace dart::common {

/// Class representing an entity in entity-component-system
template <typename DataT>
class EntityT
{
public:
  /// The type used to store the entity data.
  using Data = DataT;

  /// The type used to store the entity ID.
  using IdType = Data;
  // We just use the low n bytes where n is sizeof(IdType) - sizeof(VersionType)
  // for storing the entity ID.

  /// The type used to store the entity version number.
  using VersionType = std::uint8_t;

  /// Constructs an invalid Entity object
  EntityT() = default;

  /// Constructs an Entity object with the given index and version.
  ///
  /// @param id The ID of the entity.
  /// @param version The version number of the entity.
  explicit EntityT(Data id, Data version);

  /// Returns the ID of this entity.
  ///
  /// @return The ID of the entity.
  [[nodiscard]] Data getId() const;

  /// Returns the version number of this entity.
  ///
  /// @return The version number of the entity.
  [[nodiscard]] Data getVersion() const;
  //
  // When an entity is destroyed, its version number is incremented. When a
  // system wants to check if an entity is valid, it compares the version number
  // of the entity to the version number stored in the entity manager. If the
  // two version numbers match, the entity is considered valid. If they do not
  // match, it means the entity has been destroyed and a new entity may have
  // been allocated with the same ID, so the entity is considered invalid.
  //
  // This mechanism allows the ECS to safely reuse entity IDs that have been
  // freed up by destroyed entities, without worrying about invalidating
  // references to those entities. By keeping track of the version number of
  // each entity, the ECS can ensure that an entity reference is only considered
  // valid if it corresponds to the most recent version of the entity with that
  // ID.
  //

  /// Returns true if this entity is equal to the given entity.
  ///
  /// @param entity The entity to compare to.
  /// @return True if the entities are equal, false otherwise.
  [[nodiscard]] bool operator==(const EntityT& entity) const;

  /// Returns true if this entity is not equal to the given entity.
  ///
  /// @param entity The entity to compare to.
  /// @return True if the entities are not equal, false otherwise.
  [[nodiscard]] bool operator!=(const EntityT& entity) const;

  /// Returns the maximum value that can be represented by the entity ID
  ///
  /// @return The maximum ID value.
  [[nodiscard]] static constexpr Data getMaxId();

  /// Returns the maximum value that can be represented by the entity version
  ///
  /// @return The maximum version value.
  [[nodiscard]] static constexpr Data getMaxVersion();

protected:
  /// The number of bits of the whole data type
  static constexpr Data DATA_BITS = sizeof(Data) * Data(8);

  /// The number of bits used to represent the entity version number in the data
  static constexpr Data VERSION_BITS = sizeof(VersionType) * Data(8); // [0, 255]

  /// The number of bits used to represent the entity ID in the data
  static constexpr Data ID_BITS
      = DATA_BITS - VERSION_BITS; // [0, 16,777,215] if Data == uint32_t

  /// A bit mask used to extract the entity version number from the data
  static constexpr Data VERSION_MASK = (Data(1) << VERSION_BITS) - Data(1);

  /// A bit mask used to extract the entity ID from the data
  static constexpr Data ID_MASK = (Data(1) << ID_BITS) - Data(1);

private:
  /// The data that stores the entity ID and version number.
  Data m_data;
};

using Entity = EntityT<std::uint32_t>;

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

#include <functional>

// Define a hash function for Entity
namespace std {

template <typename DataT>
struct hash<::dart::common::EntityT<DataT>>
{
  [[nodiscard]] std::size_t operator()(
      const ::dart::common::EntityT<DataT>& entity) const
  {
    return std::hash<typename ::dart::common::EntityT<DataT>::Data>()(
        entity.getId());
  }
};

} // namespace std

namespace dart::common {

//==============================================================================
template <typename DataT>
EntityT<DataT>::EntityT(Data id, Data version)
  : m_data((id & ID_MASK) | ((version & VERSION_MASK) << ID_BITS))
{
  static_assert(sizeof(DataT) * DataT(8) > VERSION_BITS);

  DART_ASSERT(getId() == id);
  DART_ASSERT(getVersion() == version);
}

//==============================================================================
template <typename DataT>
typename EntityT<DataT>::Data EntityT<DataT>::getId() const
{
  return m_data & ID_MASK;
}

//==============================================================================
template <typename DataT>
typename EntityT<DataT>::Data EntityT<DataT>::getVersion() const
{
  return (m_data >> ID_BITS) & VERSION_MASK;
}

//==============================================================================
template <typename DataT>
bool EntityT<DataT>::operator==(const EntityT& entity) const
{
  return entity.m_data == m_data;
}

//==============================================================================
template <typename DataT>
bool EntityT<DataT>::operator!=(const EntityT& entity) const
{
  return entity.m_data != m_data;
}

//==============================================================================
template <typename DataT>
constexpr typename EntityT<DataT>::Data EntityT<DataT>::getMaxId()
{
  return (DataT(1) << ID_BITS) - DataT(1);
}

//==============================================================================
template <typename DataT>
constexpr typename EntityT<DataT>::Data EntityT<DataT>::getMaxVersion()
{
  return (DataT(1) << VERSION_BITS) - DataT(1);
}

} // namespace dart::common
