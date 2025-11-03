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

#include "dart7/world.hpp"

#include "dart7/body/rigid_body.hpp"
#include "dart7/common/ecs_utils.hpp"
#include "dart7/common/exceptions.hpp"
#include "dart7/comps/all.hpp"
#include "dart7/frame/fixed_frame.hpp"
#include "dart7/frame/frame.hpp"
#include "dart7/frame/free_frame.hpp"
#include "dart7/io/binary_io.hpp"
#include "dart7/io/serializer.hpp"
#include "dart7/multi_body/multi_body.hpp"

#include <algorithm>
#include <format>
#include <istream>
#include <ostream>
#include <string>
#include <utility>

namespace {

template <typename... Components>
std::size_t countEntities(const entt::registry& registry)
{
  std::size_t count = 0;
  auto view = registry.view<Components...>();
  for (auto entity : view) {
    (void)entity;
    ++count;
  }
  return count;
}

template <typename Component>
bool hasEntityWithName(const entt::registry& registry, std::string_view name)
{
  auto view = registry.view<Component, dart7::comps::Name>();
  for (auto entity : view) {
    const auto& info = view.template get<dart7::comps::Name>(entity);
    if (info.name == name) {
      return true;
    }
  }
  return false;
}

} // namespace

namespace dart7 {

World::World() = default;

//==============================================================================
entt::registry& World::getRegistry()
{
  return m_registry;
}

//==============================================================================
const entt::registry& World::getRegistry() const
{
  return m_registry;
}

//==============================================================================
void World::clear()
{
  m_registry.clear();
  m_simulationMode = false;
  m_freeFrameCounter = 0;
  m_fixedFrameCounter = 0;
  m_multiBodyCounter = 0;
  m_rigidBodyCounter = 0;
  m_linkCounter = 0;
  m_jointCounter = 0;
}

//==============================================================================
void World::ensureDesignMode() const
{
  DART7_THROW_T_IF(
      m_simulationMode,
      InvalidOperationException,
      "World modifications are not allowed while in simulation mode");
}

//==============================================================================
FreeFrame World::addFreeFrame()
{
  return addFreeFrame("", Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name)
{
  return addFreeFrame(name, Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name, const Frame& parent)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      Eigen::Isometry3d::Identity(),
      &m_freeFrameCounter,
      "free_frame",
      false,
      actualName);

  return FreeFrame(entity, this);
}

//==============================================================================
FixedFrame World::addFixedFrame(std::string_view name, const Frame& parent)
{
  return addFixedFrame(name, parent, Eigen::Isometry3d::Identity());
}

//==============================================================================
FixedFrame World::addFixedFrame(
    std::string_view name, const Frame& parent, const Eigen::Isometry3d& offset)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  DART7_THROW_T_IF(
      name.empty(),
      InvalidArgumentException,
      "FixedFrame requires a non-empty name");

  DART7_THROW_T_IF(
      parentFrame.isWorld(),
      InvalidArgumentException,
      "FixedFrame cannot be attached directly to the world frame");

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      offset,
      &m_fixedFrameCounter,
      "fixed_frame",
      true,
      actualName);

  return FixedFrame(entity, this);
}

//==============================================================================
entt::entity World::createFrameEntity(
    std::string_view name,
    const Frame& parentFrame,
    const Eigen::Isometry3d& localTransform,
    std::size_t* autoNameCounter,
    std::string_view autoNamePrefix,
    bool isFixedFrame,
    std::string& outName)
{
  std::string actualName;
  if (name.empty()) {
    if (autoNameCounter) {
      actualName
          = std::format("{}_{:03d}", autoNamePrefix, ++(*autoNameCounter));
    } else {
      actualName = std::string(autoNamePrefix);
    }
  } else {
    actualName = std::string(name);
  }

  auto entity = m_registry.create();
  m_registry.emplace<comps::Name>(entity, actualName);
  m_registry.emplace<comps::FrameTag>(entity);

  if (isFixedFrame) {
    m_registry.emplace<comps::FixedFrameTag>(entity);
  } else {
    m_registry.emplace<comps::FreeFrameTag>(entity);
  }

  auto& state = m_registry.emplace<comps::FrameState>(entity);
  state.parentFrame = parentFrame.getEntity();

  auto& cache = m_registry.emplace<comps::FrameCache>(entity);
  cache.worldTransform = Eigen::Isometry3d::Identity();
  cache.needTransformUpdate = true;

  if (isFixedFrame) {
    auto& props = m_registry.emplace<comps::FixedFrameProperties>(entity);
    props.localTransform = localTransform;
  } else {
    auto& props = m_registry.emplace<comps::FreeFrameProperties>(entity);
    props.localTransform = localTransform;
  }

  outName = actualName;
  return entity;
}

//==============================================================================
Frame World::resolveParentFrame(const Frame& parent) const
{
  if (parent.isWorld()) {
    return Frame(entt::null, const_cast<World*>(this));
  }

  DART7_THROW_T_IF(
      !parent.isValid(),
      InvalidArgumentException,
      "Parent frame is invalid or has been destroyed");

  DART7_THROW_T_IF(
      parent.getWorld() != this,
      InvalidArgumentException,
      "Parent frame belongs to a different world");

  return parent;
}

//==============================================================================
MultiBody World::addMultiBody(std::string_view name)
{
  ensureDesignMode();

  std::string candidateName
      = name.empty() ? std::format("multi_body_{:03d}", m_multiBodyCounter + 1)
                     : std::string(name);

  DART7_THROW_T_IF(
      hasEntityWithName<comps::MultiBodyTag>(m_registry, candidateName),
      InvalidArgumentException,
      "MultiBody '{}' already exists",
      candidateName);

  if (name.empty()) {
    ++m_multiBodyCounter;
  }

  auto entity = m_registry.create();
  m_registry.emplace<comps::Name>(entity, candidateName);
  m_registry.emplace<comps::MultiBodyTag>(entity);
  m_registry.emplace<comps::MultiBodyStructure>(entity);

  return MultiBody(entity, this);
}

//==============================================================================
std::optional<MultiBody> World::getMultiBody(std::string_view name)
{
  auto view = m_registry.view<comps::MultiBodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return MultiBody(entity, this);
    }
  }
  return std::nullopt;
}

//==============================================================================
std::size_t World::getMultiBodyCount() const
{
  return countEntities<comps::MultiBodyTag>(m_registry);
}

//==============================================================================
RigidBody World::addRigidBody(
    std::string_view name, const RigidBodyOptions& options)
{
  (void)options; // Placeholder for future use
  ensureDesignMode();

  std::string candidateName
      = name.empty() ? std::format("rigid_body_{:03d}", m_rigidBodyCounter + 1)
                     : std::string(name);

  DART7_THROW_T_IF(
      hasEntityWithName<comps::RigidBodyTag>(m_registry, candidateName),
      InvalidArgumentException,
      "RigidBody '{}' already exists",
      candidateName);

  Frame parent = Frame(entt::null, this);

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parent,
      Eigen::Isometry3d::Identity(),
      &m_rigidBodyCounter,
      "rigid_body",
      false,
      actualName);

  m_registry.emplace<comps::RigidBodyTag>(entity);
  return RigidBody(entity, this);
}

//==============================================================================
bool World::hasRigidBody(std::string_view name) const
{
  return hasEntityWithName<comps::RigidBodyTag>(m_registry, name);
}

//==============================================================================
std::size_t World::getRigidBodyCount() const
{
  return countEntities<comps::RigidBodyTag>(m_registry);
}

//==============================================================================
void World::enterSimulationMode()
{
  DART7_THROW_T_IF(
      m_simulationMode,
      InvalidArgumentException,
      "World is already in simulation mode");

  m_simulationMode = true;

  // Initial bake so that cached transforms are up-to-date.
  updateKinematics();
}

//==============================================================================
void World::updateKinematics()
{
  DART7_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "updateKinematics() requires simulation mode");

  auto cacheView = m_registry.view<comps::FrameTag, comps::FrameCache>();

  // Mark caches dirty
  for (auto entity : cacheView) {
    auto& cache = cacheView.get<comps::FrameCache>(entity);
    cache.needTransformUpdate = true;
  }

  // Recompute world transforms
  for (auto entity : cacheView) {
    Frame frame(entity, this);
    (void)frame.getTransform();
  }
}

//==============================================================================
void World::saveBinary(std::ostream& output) const
{
  io::writeFormatHeader(output);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().saveAllEntities(
      output, m_registry, entityMap);

  const std::uint8_t simulationFlag = m_simulationMode ? 1 : 0;
  io::writePOD(output, simulationFlag);
  io::writePOD(output, m_freeFrameCounter);
  io::writePOD(output, m_fixedFrameCounter);
  io::writePOD(output, m_multiBodyCounter);
  io::writePOD(output, m_rigidBodyCounter);
  io::writePOD(output, m_linkCounter);
  io::writePOD(output, m_jointCounter);
}

//==============================================================================
void World::loadBinary(std::istream& input)
{
  clear();

  io::readFormatHeader(input);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().loadAllEntities(
      input, m_registry, entityMap);

  // World metadata (optional for forward-compatibility)
  if (input.peek() != std::char_traits<char>::eof()) {
    std::uint8_t simulationFlag = 0;
    io::readPOD(input, simulationFlag);
    m_simulationMode = simulationFlag != 0;

    io::readPOD(input, m_freeFrameCounter);
    io::readPOD(input, m_fixedFrameCounter);
    io::readPOD(input, m_multiBodyCounter);
    io::readPOD(input, m_rigidBodyCounter);
    io::readPOD(input, m_linkCounter);
    io::readPOD(input, m_jointCounter);
  }

  // Ensure all frame entities have cache components (not serialized)
  auto frameView = m_registry.view<comps::FrameTag>();
  for (auto entity : frameView) {
    if (!m_registry.any_of<comps::FrameCache>(entity)) {
      auto& cache = m_registry.emplace<comps::FrameCache>(entity);
      cache.worldTransform = Eigen::Isometry3d::Identity();
      cache.needTransformUpdate = true;
    } else {
      auto& cache = m_registry.get<comps::FrameCache>(entity);
      cache.needTransformUpdate = true;
    }
  }

  resetCountersFromRegistry();

  if (m_simulationMode) {
    updateKinematics();
  }
}

//==============================================================================
void World::resetCountersFromRegistry()
{
  m_freeFrameCounter = std::max(
      m_freeFrameCounter, countEntities<comps::FreeFrameTag>(m_registry));
  m_fixedFrameCounter = std::max(
      m_fixedFrameCounter, countEntities<comps::FixedFrameTag>(m_registry));
  m_multiBodyCounter = std::max(
      m_multiBodyCounter, countEntities<comps::MultiBodyTag>(m_registry));
  m_rigidBodyCounter = std::max(
      m_rigidBodyCounter, countEntities<comps::RigidBodyTag>(m_registry));
  m_linkCounter
      = std::max(m_linkCounter, countEntities<comps::Link>(m_registry));
  m_jointCounter
      = std::max(m_jointCounter, countEntities<comps::Joint>(m_registry));
}

} // namespace dart7
