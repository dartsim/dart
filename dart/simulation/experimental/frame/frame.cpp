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

#include "dart/simulation/experimental/frame/frame.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/frame/free_frame.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <dart/simulation/experimental/comps/link.hpp>

#include <vector>

namespace {

void markSubtreeCacheDirty(entt::registry& registry, entt::entity root)
{
  if (root == entt::null) {
    return;
  }

  if (!registry.valid(root)) {
    return;
  }

  std::vector<entt::entity> stack;
  stack.push_back(root);

  auto frameStateView
      = registry.view<dart::simulation::experimental::comps::FrameState>();

  while (!stack.empty()) {
    auto entity = stack.back();
    stack.pop_back();

    if (!registry.valid(entity)) {
      continue;
    }

    if (auto* cache
        = registry.try_get<dart::simulation::experimental::comps::FrameCache>(
            entity)) {
      cache->needTransformUpdate = true;
    }

    for (auto child : frameStateView) {
      const auto& state
          = frameStateView
                .get<dart::simulation::experimental::comps::FrameState>(child);
      if (state.parentFrame == entity) {
        stack.push_back(child);
      }
    }
  }
}

} // namespace

namespace dart::simulation::experimental {

//==============================================================================
Frame::Frame(entt::entity entity, World* world)
  : EntityObjectWith<
        TagComps<comps::FrameTag>,
        ReadOnlyComps<>,
        WriteOnlyComps<>,
        ReadWriteComps<comps::FrameState, comps::FrameCache>>()
{
  // Initialize base EntityObject with entity/world
  // Virtual inheritance requires explicit initialization
  m_entity = entity;
  m_world = world;
}

//==============================================================================
const Eigen::Isometry3d& Frame::getLocalTransform() const
{
  // Default implementation: returns identity (for world frame)
  // Derived classes override this to provide frame-type-specific behavior
  static const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  return identity;
}

//==============================================================================
Frame Frame::getParentFrame() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid frame handle");

  // World frame: parent is itself
  if (m_entity == entt::null) {
    return Frame::world();
  }

  // Get parent from FrameState component (common to all frame types)
  auto* frameState = tryGetReadOnly<comps::FrameState>();
  DART_EXPERIMENTAL_THROW_T_IF(
      !frameState,
      InvalidOperationException,
      "Entity does not have FrameState component");

  return Frame(frameState->parentFrame, m_world);
}

//==============================================================================
void Frame::setParentFrame(const Frame& parent)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid frame handle");

  DART_EXPERIMENTAL_THROW_T_IF(
      !parent.isValid(), InvalidArgumentException, "Invalid parent frame");

  // World frame cannot change parent
  if (m_entity == entt::null) {
    DART_EXPERIMENTAL_THROW_T(
        InvalidOperationException, "Cannot set parent of world frame");
  }

  // Get FrameState component (common to FreeFrame and FixedFrame)
  auto* frameState = tryGetMutable<comps::FrameState>();

  DART_EXPERIMENTAL_THROW_T_IF(
      !frameState,
      InvalidOperationException,
      "Cannot change parent frame of Link. Links are connected through "
      "joints in a fixed tree structure.");

  if (m_world && m_entity != entt::null) {
    auto& registry = m_world->getRegistry();
    if (registry.valid(m_entity) && registry.all_of<comps::Link>(m_entity)) {
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "Cannot change parent frame of Link. Links are connected through "
          "joints in a fixed tree structure.");
    }
  }

  const auto parentEntity = parent.getEntity();

  if (parentEntity != entt::null) {
    DART_EXPERIMENTAL_THROW_T_IF(
        m_world != parent.m_world,
        InvalidArgumentException,
        "Parent frame belongs to a different world");
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      parentEntity == m_entity,
      InvalidArgumentException,
      "Cannot parent a frame to itself");

  // Prevent cycles by walking up the prospective parent's ancestry.
  Frame ancestor = parent;
  std::size_t depth = 0;
  while (ancestor.getEntity() != entt::null) {
    if (ancestor.getEntity() == m_entity) {
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException, "Cannot create cyclic frame hierarchy");
    }

    auto next = ancestor.getParentFrame();

    // Defensive break in case getParentFrame() returns self (shouldn't happen).
    if (next.getEntity() == ancestor.getEntity()) {
      break;
    }

    ancestor = next;

    if (++depth > 1024) {
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException, "Cycle detection depth limit exceeded");
    }
  }

  // Update parent in FrameState component
  frameState->parentFrame = parentEntity;

  if (m_world) {
    markSubtreeCacheDirty(m_world->getRegistry(), m_entity);
  }
}

//==============================================================================
const Eigen::Isometry3d& Frame::getTransform() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid frame handle");

  // World frame: return identity
  if (m_entity == entt::null) {
    static const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
    return identity;
  }

  // Frames with lazy evaluation (FreeFrame, FixedFrame) use FrameCache
  // Use getCacheMutable() for cache updates in const method
  auto* cache = getCacheMutable<comps::FrameCache>();
  DART_EXPERIMENTAL_THROW_T_IF(
      !cache,
      InvalidOperationException,
      "Entity does not have FrameCache component (expected for "
      "FreeFrame/FixedFrame). Links should override getTransform().");

  if (cache->needTransformUpdate) {
    auto parent = getParentFrame();
    cache->worldTransform = parent.getTransform() * getLocalTransform();
    cache->needTransformUpdate = false;
  }

  return cache->worldTransform;
}

//==============================================================================
Eigen::Vector3d Frame::getTranslation() const
{
  return getTransform().translation();
}

//==============================================================================
Eigen::Matrix3d Frame::getRotation() const
{
  return getTransform().linear();
}

//==============================================================================
Eigen::Quaterniond Frame::getQuaternion() const
{
  return Eigen::Quaterniond(getTransform().linear());
}

//==============================================================================
Eigen::Matrix4d Frame::getTransformMatrix() const
{
  return getTransform().matrix();
}

//==============================================================================
Eigen::Isometry3d Frame::getTransform(const Frame& relativeTo) const
{
  // Both frames must be valid
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid this frame");
  DART_EXPERIMENTAL_THROW_T_IF(
      !relativeTo.isValid(),
      InvalidArgumentException,
      "Invalid 'relativeTo' frame");

  // Both frames must be from the same World (or both are world frame)
  DART_EXPERIMENTAL_THROW_T_IF(
      m_world != relativeTo.m_world && m_world != nullptr
          && relativeTo.m_world != nullptr,
      InvalidArgumentException,
      "Frames are from different worlds");

  // Optimization: transform to world is just the world transform
  if (relativeTo.isWorld()) {
    return getTransform();
  }

  // Optimization: transform to self is identity
  if (relativeTo.m_entity == m_entity) {
    return Eigen::Isometry3d::Identity();
  }

  // Optimization: transform to parent is just the local transform
  auto parent = getParentFrame();
  if (!parent.isWorld() && relativeTo.m_entity == parent.m_entity) {
    return getLocalTransform();
  }

  // General case: compute via world transforms
  auto T_world_this = getTransform();
  auto T_world_to = relativeTo.getTransform();

  // Compute relative transform: T_to_this = T_world_to^{-1} * T_world_this
  return T_world_to.inverse() * T_world_this;
}

//==============================================================================
Eigen::Isometry3d Frame::getTransform(
    const Frame& to, const Frame& expressedIn) const
{
  // Validate frames
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid this frame");
  DART_EXPERIMENTAL_THROW_T_IF(
      !to.isValid(), InvalidArgumentException, "Invalid 'to' frame");
  DART_EXPERIMENTAL_THROW_T_IF(
      !expressedIn.isValid(),
      InvalidArgumentException,
      "Invalid expressedIn frame");

  // All frames must be from the same World (world frame is compatible with any)
  const bool toIsWorld = to.isWorld();
  const bool coordIsWorld = expressedIn.isWorld();

  DART_EXPERIMENTAL_THROW_T_IF(
      !toIsWorld && m_world != to.m_world,
      InvalidArgumentException,
      "Frames are from different worlds");
  DART_EXPERIMENTAL_THROW_T_IF(
      !coordIsWorld && m_world != expressedIn.m_world,
      InvalidArgumentException,
      "Frames are from different worlds");

  // Optimization: if to and expressedIn are the same, just use 2-parameter
  // version
  if (to.m_entity == expressedIn.m_entity) {
    return getTransform(to);
  }

  // Rotation from "expressedIn" to "to"
  // This transforms the coordinate system from expressedIn to to's coordinates
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = expressedIn.getTransform().linear().transpose()
               * to.getTransform().linear();

  // Apply coordinate transformation to the relative transform
  return T * getTransform(to);
}

//==============================================================================
Frame Frame::world()
{
  // Return a special frame representing the world
  // Uses entt::null entity and nullptr world as markers
  return Frame(entt::null, nullptr);
}

//==============================================================================
bool Frame::isValid() const
{
  // World frame is valid (entity is null, world can be null or non-null)
  if (m_entity == entt::null) {
    return true;
  }

  // Regular frames need both entity and world
  if (m_world == nullptr) {
    return false;
  }

  // Check if entity exists in registry
  auto& registry = m_world->getRegistry();
  if (!registry.valid(m_entity)) {
    return false;
  }

  // Check if entity has FrameTag
  if (!registry.all_of<comps::FrameTag>(m_entity)) {
    return false;
  }

  return true;
}

} // namespace dart::simulation::experimental
