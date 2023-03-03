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
#include <dart/common/ecs/entity.hpp>

#include <any>
#include <deque>
#include <typeindex>
#include <unordered_map>
#include <vector>

#ifndef NDEBUG
  #include <iostream>
#endif

namespace dart::common {

/// A class template for managing entities in a game or simulation.
template <typename EntityType_ = EntityT<std::uint32_t>>
class EntityManagerT
{
public:
  using This = EntityManagerT<EntityType_>;

  /// The entity type
  using EntityType = EntityType_;

  /// The entity data type
  using EntityDataType = typename EntityType::Data;

  /// The entity ID type
  using EntityIdType = typename EntityType::IdType;

  /// Then entity version type
  using EntityVersionType = typename EntityType::VersionType;

  /// Default constructor
  EntityManagerT() = default;

  /// @{ @name Entity Modifiers

  /// Creates a new entity.
  ///
  /// @return The new entity.
  [[nodiscard]] EntityType create();

  /// Destroys an entity.
  ///
  /// @param entity The entity to destroy.
  void destroy(EntityType entity);

  /// Checks if an entity is valid.
  ///
  /// An entity is valid if it has not been destroyed.
  ///
  /// @param entity The entity to check.
  /// @return True if the entity is valid, false otherwise.
  [[nodiscard]] bool isValid(EntityType entity) const;

  /// @}

  /// @{ @name Component Modifiers

  /// Adds a component of type T to the given entity.
  ///
  /// Returns a pointer to the added component, or nullptr if the component
  /// cannot be added. The component is constructed using the given arguments,
  /// if any. If the entity already has a component of the same type, the
  /// existing component is replaced. If the component type is not
  /// default-constructible, it must be constructed using arguments.
  ///
  /// @param entity The entity to add the component to.
  /// @param args The arguments to use to construct the component.
  /// @return A pointer to the added component, or nullptr if the component
  /// cannot be added.
  template <typename T, typename... Args>
  T& addComponent(EntityType entity, Args&&... args);

  /// Removes the component of type T associated with the given entity.
  ///
  /// Returns true if the component was successfully removed, or false if the
  /// entity did not have the component.
  ///
  /// This method removes the component from the entity-component map, updates
  /// the component vector, and removes the entity from the component-entity
  /// map. If the component was not the last element in the component vector,
  /// the method moves the last element of the vector to the position of the
  /// removed component to maintain the contiguous memory layout. This allows
  /// the component vector to be cache friendly and work well with GPGPU.
  ///
  /// @tparam T The type of the component.
  /// @param entity The entity to remove the component from.
  /// @return True if the component was successfully removed, or false if the
  /// entity did not have the component.
  template <typename T>
  bool removeComponent(EntityType entity);

  /// Returns a reference to the component of type T associated with the given
  /// entity, if it exists.
  ///
  /// Throws std::out_of_range if the component is not found.
  ///
  /// @param entity The entity to get the component from.
  /// @return A reference to the component.
  template <typename T>
  [[nodiscard]] T& getComponent(EntityType entity);

  /// Returns a const reference to the component of type T associated with the
  /// given entity, if it exists.
  ///
  /// Throws std::out_of_range if the component is not found.
  ///
  /// @param entity The entity to get the component from.
  /// @return A const reference to the component.
  template <typename T>
  [[nodiscard]] const T& getComponent(EntityType entity) const;

  /// Returns a pointer to the component of type T associated with the given
  /// entity, if it exists.
  ///
  /// Returns nullptr if the component is not found.
  ///
  /// @param entity The entity to get the component from.
  /// @return A pointer to the component, or nullptr if the component is not
  /// found.
  template <typename T>
  [[nodiscard]] T* tryComponent(EntityType entity);

  /// Returns a const pointer to the component of type T associated with the
  /// given entity, if it exists.
  ///
  /// Returns nullptr if the component is not found.
  ///
  /// @param entity The entity to get the component from.
  /// @return A const pointer to the component, or nullptr if the component is
  /// not found.
  template <typename T>
  [[nodiscard]] const T* tryComponent(EntityType entity) const;

  /// Returns true if the given entity has a component of type T.
  ///
  /// @param entity The entity to check.
  /// @return True if the entity has a component of type T, false otherwise.
  template <typename T>
  [[nodiscard]] bool hasComponent(EntityType entity) const;

  /// Returns true if the given entity has all of the given components.
  ///
  /// @param entity The entity to check.
  /// @return True if the entity has all of the given components, false
  /// otherwise.
  template <typename... Components>
  [[nodiscard]] bool hasComponents(EntityType entity) const;

  /// @}

  /// @{ @name Entity Iteration

  template <typename... Components>
  View<This, Components...> view();

  /// @}

#ifndef NDEBUG
  void print(std::ostream& os = std::cout, int indent = 0) const;
#endif

private:
  /// Returns a reference to the vector of components of type T with the given
  /// type_index, creating a new vector of components if it doesn't exist
  /// already.
  ///
  /// The components are stored in a contiguous memory layout in the vector to
  /// provide efficient access. If the vector does not exist, a new one is
  /// created and returned.
  ///
  /// @tparam T The type of the component.
  /// @param type_index The type index of the component.
  /// @return A reference to the vector of components of type T with the given
  /// type_index.
  template <typename T>
  std::vector<T>& getOrCreateComponentVector(std::type_index type_index);

  /// Returns an iterator to the vector of components of type T with the given
  /// type_index.
  ///
  /// The components are stored in a contiguous memory layout in the vector to
  /// provide efficient access. If the vector does not exist, undefined behavior
  /// may occur.
  ///
  /// @tparam T The type of the component.
  /// @param type_index The type index of the component.
  /// @return An iterator to the vector of components of type T with the given
  /// type_index.
  template <typename T>
  auto getComponentVectorIterator(std::type_index type_index);

  /// Returns a reference to the vector of components of type T with the given
  /// type_index.
  ///
  /// The components are stored in a contiguous memory layout in the vector to
  /// provide efficient access. If the vector does not exist, undefined behavior
  /// may occur.
  ///
  /// @tparam T The type of the component.
  /// @param type_index The type index of the component.
  /// @return A reference to the vector of components of type T with the given
  /// type_index.
  template <typename T>
  std::vector<T>& getComponentVector(std::type_index type_index);

  /// Removes the component at the given index from the vector of components.
  ///
  /// When a component is removed from the vector, the last element in the
  /// vector is moved to the index of the removed component to keep the
  /// components contiguous in memory. The last element is then removed from the
  /// vector.
  ///
  /// @tparam T The type of the component.
  /// @param type_index The type index of the component.
  /// @param comp_index The index of the component to remove.
  ///   template <typename T>
  /// @return True if component element data move happened
  template <typename T>
  bool removeComponentFromVector(
      std::type_index type_index, std::size_t comp_index);

  /// The minimum number of free indices in the queue before reusing one
  static constexpr uint32_t MINIMUM_FREE_INDICES = 1024u;

  std::vector<EntityType> m_entities;

  /// The entity versions
  ///
  /// The version is incremented every time an entity is destroyed.
  std::vector<EntityVersionType> m_versions;

  /// The free entity ids
  ///
  /// The ids are reused when possible to avoid allocating new ids every time
  /// an entity is destroyed.
  std::deque<EntityDataType> m_free_ids;

  /// Map from component type to vector of components.
  ///
  /// The vector stores components of the corresponding type for all entities.
  std::unordered_map<std::type_index, std::any> m_comp_vectors;

  /// Map from entity to indices of its components of each type.
  ///
  /// The outer map has the entity ID as its key, and the inner map has the
  /// component type index as its key. The corresponding value is the index of
  /// the component in the component vector for that type.
  std::unordered_map<
      EntityType,
      std::unordered_map<std::type_index, std::size_t>>
      m_entity_comp_map;

  /// Map from component type and index to set of entities that have the
  /// component.
  ///
  /// The outer map has the component type index as its key, and the inner map
  /// has the component index as its key. The corresponding value is a set of
  /// entity IDs that have the component of that type and index.
  std::unordered_map<
      std::type_index,
      std::unordered_map<std::size_t, EntityType>>
      m_comp_entity_map;
};

using EntityManager = EntityManagerT<EntityT<std::uint32_t>>;

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

#include <dart/common/ecs/view.hpp>

#include <typeinfo>

namespace dart::common {

//==============================================================================
template <typename EntityType>
typename EntityManagerT<EntityType>::EntityType
EntityManagerT<EntityType>::create()
{
  EntityDataType id;

  if (m_free_ids.size() > MINIMUM_FREE_INDICES) {
    id = m_free_ids.front();
    m_free_ids.pop_front();
  } else {
    id = static_cast<EntityIdType>(m_versions.size());
    DART_ASSERT(id <= EntityType::getMaxId());
    m_versions.push_back(0);
  }

  auto entity = Entity(id, m_versions[id]);

  m_entity_comp_map.insert({entity, {}});

  return entity;
}

//==============================================================================
template <typename EntityType>
void EntityManagerT<EntityType>::destroy(EntityType entity)
{
  auto it = m_entity_comp_map.find(entity);
  if (it == m_entity_comp_map.end()) {
    return;
  }

  // Erase components of the entity from the maps
  auto& comp_to_index = it->second;
  for (auto comp_it = comp_to_index.begin(); comp_it != comp_to_index.end();
       ++comp_it) {
    auto& comp_entity_map = m_comp_entity_map[comp_it->first];
    auto entity_set_it = comp_entity_map.find(comp_it->second);
    if (entity_set_it != comp_entity_map.end()) {
      entity_set_it->second = Entity();
    }
    m_entity_comp_map.erase(it);
  }

  // Mark the entity as destroyed and add its index to the free list
  const auto id = entity.getId();
  ++m_versions[id];
  m_free_ids.push_back(id);
}

//==============================================================================
template <typename EntityType>
bool EntityManagerT<EntityType>::isValid(EntityType entity) const
{
  return m_versions[entity.getId()] == entity.getVersion();
}

//==============================================================================
template <typename EntityType>
template <typename T, typename... Args>
T& EntityManagerT<EntityType>::addComponent(EntityType entity, Args&&... args)
{
  const auto type_index = std::type_index(typeid(T));

  // Get the component vector
  auto& comps = getOrCreateComponentVector<T>(type_index);
  const auto comp_index = comps.size();

  // Add the component to the component vector
  if constexpr (std::is_aggregate_v<T>) {
    comps.emplace_back();
    comps.back() = T{std::forward<Args>(args)...};
  } else {
    comps.emplace_back(std::forward<Args>(args)...);
  }

  // Update entity-comp-comp_index map
  m_entity_comp_map[entity][type_index] = comp_index;

  // Update comp-comp_index-entity map
  m_comp_entity_map[type_index][comp_index] = entity;

  return comps.back();
}

//==============================================================================
template <typename EntityType>
template <typename T>
bool EntityManagerT<EntityType>::removeComponent(EntityType entity)
{
  // Retrieve the type index of the component to remove
  const auto type_index = std::type_index(typeid(T));

  // Update m_entity_comp_map --------------------------------------------------

  auto it = m_entity_comp_map.find(entity);
  if (it == m_entity_comp_map.end()) {
    // The entity does not have any component of type T
    return false;
  }

  std::unordered_map<std::type_index, std::size_t>& comp_map = it->second;
  auto comp_it = comp_map.find(type_index);
  if (comp_it == comp_map.end()) {
    // The entity does not have any component of type T
    return false;
  }

  const std::size_t comp_index = comp_it->second;
  comp_map.erase(comp_it);

//  if (comp_map.empty()) {
//    // The entity no longer has any components, remove it from the map
//    m_entity_comp_map.erase(it);
//  }

  // Update m_comp_vectors -----------------------------------------------------

  auto comps_it = m_comp_vectors.find(type_index);
  DART_ASSERT(comps_it != m_comp_vectors.end());

  auto& comps = std::any_cast<std::vector<T>&>(comps_it->second);
  DART_ASSERT(!comps.empty());
  DART_ASSERT(comp_index < comps.size());
  const bool comp_index_changed = (comp_index != comps.size() - 1u);

  // Move the last component in the vector to the position of the component to
  // remove
  comps[comp_index] = std::move(comps.back());
  comps.pop_back();

  if (comps.empty()) {
    // The vector of components is now empty, remove it from the map
    m_comp_vectors.erase(comps_it);
  }

  // Update m_comp_entity_map --------------------------------------------------

  auto comp_entity_it = m_comp_entity_map.find(type_index);
  DART_ASSERT(comp_entity_it != m_comp_entity_map.end());

  std::unordered_map<std::size_t, EntityType>& comp_index_to_entity_map
      = comp_entity_it->second;
  auto comp_index_to_entity_it = comp_index_to_entity_map.find(comp_index);
  DART_ASSERT(comp_index_to_entity_it != comp_index_to_entity_map.end());

  // Update the mapping from the component index to the entity
  if (comp_index_changed) {
    const auto comp_index_to_update = comps.size();
    DART_ASSERT(
        comp_index_to_entity_map.find(comp_index_to_update)
        != comp_index_to_entity_map.end());
    const auto entity_to_update
        = comp_index_to_entity_map[comp_index_to_update];

    // Update the mapping for the component index being removed
    comp_index_to_entity_it->second = entity_to_update;
    comp_index_to_entity_map.erase(comp_index_to_update);

    // Update the mapping from the entity to the component index being removed
    auto entity_comp_it = m_entity_comp_map.find(entity_to_update);
    DART_ASSERT(entity_comp_it != m_entity_comp_map.end());

    std::unordered_map<std::type_index, std::size_t>& comp_to_index_map
        = entity_comp_it->second;
    auto comp_to_index_it = comp_to_index_map.find(type_index);
    DART_ASSERT(comp_to_index_it != comp_to_index_map.end());
    comp_to_index_it->second = comp_index;
  } else {
    // Remove the mapping for the component index being removed
    comp_index_to_entity_map.erase(comp_index_to_entity_it);
  }

  return true;
}

//==============================================================================
template <typename EntityType>
template <typename T>
T& EntityManagerT<EntityType>::getComponent(EntityType entity)
{
  auto comp = tryComponent<T>(entity);
  if (!comp) {
    throw std::runtime_error("Failed to get component for an entity");
  }
  return *comp;
}

//==============================================================================
template <typename EntityType>
template <typename T>
const T& EntityManagerT<EntityType>::getComponent(EntityType entity) const
{
  return const_cast<This*>(this)->getComponent<T>(std::move(entity));
}

//==============================================================================
template <typename EntityType>
template <typename T>
T* EntityManagerT<EntityType>::tryComponent(EntityType entity)
{
  const auto type_index = std::type_index(typeid(T));

  auto it = m_entity_comp_map.find(entity);
  if (it == m_entity_comp_map.end()) {
    return nullptr;
  }

  auto& comp_map = it->second;
  auto comp_it = comp_map.find(type_index);
  if (comp_it == comp_map.end()) {
    return nullptr;
  }

  std::vector<T>& comps = getComponentVector<T>(type_index);
  auto comp_index = comp_it->second;
  if (comp_index >= comps.size()) {
    return nullptr;
  }

  return &comps[comp_index];
}

//==============================================================================
template <typename EntityType>
template <typename T>
const T* EntityManagerT<EntityType>::tryComponent(EntityType entity) const
{
  return const_cast<This*>(this)->tryComponent<T>(std::move(entity));
}

//==============================================================================
template <typename EntityType>
template <typename T>
bool EntityManagerT<EntityType>::hasComponent(EntityType entity) const
{
  return tryComponent<T>(std::move(entity)) != nullptr;
}

//==============================================================================
template <typename EntityType>
template <typename... Components>
bool EntityManagerT<EntityType>::hasComponents(EntityType entity) const
{
  // Check if entity exists and has all required components
  auto it = m_entity_comp_map.find(entity);
  if (it == m_entity_comp_map.end()) {
    return false;
  }

  const std::unordered_map<std::type_index, std::size_t>& comp_to_index
      = it->second;
  bool has_all_components = true;

  // Check if entity has all required components
  ((has_all_components &= (comp_to_index.count(typeid(Components)) > 0)), ...);

  return has_all_components;
}

//==============================================================================
template <typename EntityType>
template <typename... Components>
View<EntityManagerT<EntityType>, Components...>
EntityManagerT<EntityType>::view()
{
  return View<This, Components...>(*this);
}

#ifndef NDEBUG
//==============================================================================
template <typename EntityType>
void EntityManagerT<EntityType>::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[EntityManager]\n";
  }
  const std::string spaces(indent, ' ');

  os << spaces << "versions:\n";
  os << spaces << "  size: " << m_versions.size() << "\n";
  os << spaces << "  data: [";
  for (const unsigned v : m_versions) {
    os << v << ", ";
  }
  os << "]\n";

  os << spaces << "free_ids:\n";
  os << spaces << "  size: " << m_free_ids.size() << "\n";
  os << spaces << "  data: [";
  for (const auto& v : m_free_ids) {
    os << v << ", ";
  }
  os << "]\n";

  os << spaces << "comp_vectors:\n";
  os << spaces << "  size: " << m_comp_vectors.size() << "\n";
  os << spaces << "  data: [";
  for (const auto& [type_index, any] : m_comp_vectors) {
    os << type_index.name() << ", ";
  }
  os << "]\n";

  os << spaces << "entity_comp_map:\n";
  os << spaces << "  size: " << m_entity_comp_map.size() << "\n";
  os << spaces << "  entities:\n";
  for (const auto& [entity, type_index_map] : m_entity_comp_map) {
    os << spaces << "  - id   : " << entity.getId() << "\n";
    os << spaces << "    ver  : " << entity.getVersion() << "\n";
    os << spaces << "    comps:\n";
    for (const auto& [type, index] : type_index_map) {
      os << spaces << "    - type : " << type.name() << "\n";
      os << spaces << "      index: " << index << "\n";
    }
  }

  os << spaces << "comp_entity_map:\n";
  os << spaces << "  size: " << m_comp_entity_map.size() << "\n";
  os << spaces << "  comps:\n";
  for (const auto& [type, index_entity_map] : m_comp_entity_map) {
    os << spaces << "  - id     : " << type.name() << "\n";
    os << spaces << "    indices:\n";
    for (const auto& [index, entity] : index_entity_map) {
      os << spaces << "    - index : " << index << "\n";
      os << spaces << "      entity: " << entity.getId() << "\n";
    }
  }
}
#endif

//==============================================================================
template <typename EntityType>
template <typename T>
std::vector<T>& EntityManagerT<EntityType>::getOrCreateComponentVector(
    std::type_index type_index)
{
  DART_ASSERT(type_index == std::type_index(typeid(T)));
  static_assert(
      std::is_default_constructible_v<T>,
      "Components must be default-constructible.");

  auto it = m_comp_vectors.find(type_index);
  if (it == m_comp_vectors.end()) {
    it = m_comp_vectors.insert({type_index, std::vector<T>()}).first;
  }

  return std::any_cast<std::vector<T>&>(it->second);
}

//==============================================================================
template <typename EntityType>
template <typename T>
auto EntityManagerT<EntityType>::getComponentVectorIterator(
    std::type_index type_index)
{
  DART_ASSERT(type_index == std::type_index(typeid(T)));
  static_assert(
      std::is_default_constructible_v<T>,
      "Components must be default-constructible.");

  auto it = m_comp_vectors.find(type_index);
  DART_ASSERT(it != m_comp_vectors.end());

  return it;
}

//==============================================================================
template <typename EntityType>
template <typename T>
std::vector<T>& EntityManagerT<EntityType>::getComponentVector(
    std::type_index type_index)
{
  return std::any_cast<std::vector<T>&>(
      getComponentVectorIterator<T>(type_index)->second);
}

//==============================================================================
template <typename EntityType>
template <typename T>
bool EntityManagerT<EntityType>::removeComponentFromVector(
    std::type_index type_index, std::size_t comp_index)
{
  auto it = getComponentVectorIterator<T>(type_index);

  std::vector<T>& comps = std::any_cast<std::vector<T>&>(it->second);
  DART_ASSERT(comp_index < comps.size());

  bool index_changed = false;
  const auto last_element_index = comps.size() - 1u;
  if (comp_index < last_element_index) {
    comps[comp_index] = std::move(comps[last_element_index]);
    index_changed = true;
  }
  comps.pop_back();

  if (comps.empty()) {
    m_comp_vectors.erase(it);
  }

  return index_changed;
}

} // namespace dart::common
