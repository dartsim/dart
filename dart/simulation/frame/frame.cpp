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

#include "dart/simulation/frame/frame.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/ecs/component_access.hpp"
#include "dart/simulation/frame/free_frame.hpp"
#include "dart/simulation/world.hpp"

#include <dart/simulation/comps/frame_types.hpp>
#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/link.hpp>
#include <dart/simulation/comps/name.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <vector>

namespace {

void markSubtreeCacheDirty(
    auto& registry, entt::entity root, dart::common::MemoryAllocator& allocator)
{
  if (root == entt::null) {
    return;
  }

  if (!registry.valid(root)) {
    return;
  }

  std::vector<entt::entity, dart::common::StlAllocator<entt::entity>> stack(
      dart::common::StlAllocator<entt::entity>{allocator});
  stack.push_back(root);

  auto frameStateView
      = registry.template view<dart::simulation::comps::FrameState>();

  while (!stack.empty()) {
    auto entity = stack.back();
    stack.pop_back();

    if (!registry.valid(entity)) {
      continue;
    }

    if (auto* cache
        = registry.template try_get<dart::simulation::comps::FrameCache>(
            entity)) {
      cache->needTransformUpdate = true;
    }

    for (auto child : frameStateView) {
      const auto& state
          = frameStateView.template get<dart::simulation::comps::FrameState>(
              child);
      if (state.parentFrame == entity) {
        stack.push_back(child);
      }
    }
  }
}

} // namespace

namespace dart::simulation {

namespace {

//==============================================================================
Eigen::Isometry3d rotationVectorTransform(const Eigen::Vector3d& rotation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const double angle = rotation.norm();
  if (angle > 1e-12) {
    transform.linear()
        = Eigen::AngleAxisd(angle, rotation / angle).toRotationMatrix();
  }
  return transform;
}

//==============================================================================
Eigen::Isometry3d getJointTransform(const comps::Joint& joint)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  switch (joint.type) {
    case comps::JointType::Fixed:
      return transform;
    case comps::JointType::Revolute:
      transform.linear()
          = Eigen::AngleAxisd(joint.position[0], joint.axis).toRotationMatrix();
      return transform;
    case comps::JointType::Prismatic:
      transform.translation() = joint.axis * joint.position[0];
      return transform;
    case comps::JointType::Screw:
      transform.linear()
          = Eigen::AngleAxisd(joint.position[0], joint.axis).toRotationMatrix();
      transform.translation() = joint.axis * joint.pitch * joint.position[0];
      return transform;
    case comps::JointType::Universal:
      transform.linear() = (Eigen::AngleAxisd(joint.position[0], joint.axis)
                            * Eigen::AngleAxisd(joint.position[1], joint.axis2))
                               .toRotationMatrix();
      return transform;
    case comps::JointType::Spherical:
      return rotationVectorTransform(joint.position.head<3>());
    case comps::JointType::Planar: {
      const Eigen::Vector3d normal = joint.axis.normalized();
      const Eigen::Vector3d axis1 = joint.axis2.normalized();
      const Eigen::Vector3d axis2 = normal.cross(axis1).normalized();
      transform.translation()
          = axis1 * joint.position[0] + axis2 * joint.position[1];
      transform.linear()
          = Eigen::AngleAxisd(joint.position[2], normal).toRotationMatrix();
      return transform;
    }
    case comps::JointType::Floating:
      transform.translation() = joint.position.head<3>();
      transform.linear()
          = rotationVectorTransform(joint.position.tail<3>()).linear();
      return transform;
    case comps::JointType::Custom:
      DART_SIMULATION_THROW_T(
          InvalidOperationException,
          "Custom joints require a custom kinematics stage");
  }

  return transform;
}

//==============================================================================
Eigen::Isometry3d getFrameLocalTransform(
    const auto& registry, entt::entity entity)
{
  if (const auto* link = registry.template try_get<comps::Link>(entity)) {
    if (link->parentJoint != entt::null) {
      const auto* joint
          = registry.template try_get<comps::Joint>(link->parentJoint);
      DART_SIMULATION_THROW_T_IF(
          !joint,
          InvalidOperationException,
          "Link parent joint is missing a Joint component");

      return link->transformFromParentToJoint * getJointTransform(*joint)
             * link->transformFromParentJoint;
    }
  }

  if (const auto* props
      = registry.template try_get<comps::FreeFrameProperties>(entity)) {
    return props->localTransform;
  }

  if (const auto* props
      = registry.template try_get<comps::FixedFrameProperties>(entity)) {
    return props->localTransform;
  }

  if (const auto* link = registry.template try_get<comps::Link>(entity)) {
    return link->transformFromParentToJoint * link->transformFromParentJoint;
  }

  return Eigen::Isometry3d::Identity();
}

} // namespace

//==============================================================================
Frame::Frame(Entity entity, World* world) : m_entity(entity), m_world(world) {}

//==============================================================================
bool Frame::isWorld() const
{
  // The default-constructed Entity sentinel (value == 0xFFFFFFFF) marks world.
  return m_entity == Entity{};
}

//==============================================================================
Eigen::Isometry3d Frame::getLocalTransform() const
{
  if (isWorld()) {
    return Eigen::Isometry3d::Identity();
  }

  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid frame handle");

  return getFrameLocalTransform(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity));
}

//==============================================================================
Frame Frame::getParentFrame() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid frame handle");

  // World frame: parent is itself
  if (isWorld()) {
    return Frame::world();
  }

  // Get parent from FrameState component (common to all frame types)
  const auto* frameState = ecs::tryGetReadOnly<Frame, comps::FrameState>(*this);
  DART_SIMULATION_THROW_T_IF(
      !frameState,
      InvalidOperationException,
      "Entity does not have FrameState component");

  return Frame(detail::fromRegistryEntity(frameState->parentFrame), m_world);
}

//==============================================================================
std::string_view Frame::getName() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid frame handle");

  if (isWorld()) {
    return "world";
  }

  const auto enttEntity = detail::toRegistryEntity(m_entity);
  if (const auto* name
      = dart::simulation::detail::registryOf(*m_world).try_get<comps::Name>(
          enttEntity)) {
    return name->name;
  }

  return "";
}

//==============================================================================
void Frame::setParentFrame(const Frame& parent)
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid frame handle");

  DART_SIMULATION_THROW_T_IF(
      !parent.isValid(), InvalidArgumentException, "Invalid parent frame");

  // World frame cannot change parent
  if (isWorld()) {
    DART_SIMULATION_THROW_T(
        InvalidOperationException, "Cannot set parent of world frame");
  }

  const auto enttEntity = detail::toRegistryEntity(m_entity);
  auto& registry = dart::simulation::detail::registryOf(*m_world);

  // Get FrameState component (common to FreeFrame and FixedFrame)
  auto* frameState = ecs::tryGetMutable<Frame, comps::FrameState>(*this);

  DART_SIMULATION_THROW_T_IF(
      !frameState,
      InvalidOperationException,
      "Cannot change parent frame of Link. Links are connected through "
      "joints in a fixed tree structure.");

  if (registry.valid(enttEntity) && registry.all_of<comps::Link>(enttEntity)) {
    DART_SIMULATION_THROW_T(
        InvalidOperationException,
        "Cannot change parent frame of Link. Links are connected through "
        "joints in a fixed tree structure.");
  }

  const auto parentEntity = parent.getEntity();

  if (!parent.isWorld()) {
    DART_SIMULATION_THROW_T_IF(
        m_world != parent.m_world,
        InvalidArgumentException,
        "Parent frame belongs to a different world");
  }

  DART_SIMULATION_THROW_T_IF(
      parentEntity == m_entity,
      InvalidArgumentException,
      "Cannot parent a frame to itself");

  // Prevent cycles by walking up the prospective parent's ancestry.
  Frame ancestor = parent;
  std::size_t depth = 0;
  while (!ancestor.isWorld()) {
    if (ancestor.getEntity() == m_entity) {
      DART_SIMULATION_THROW_T(
          InvalidOperationException, "Cannot create cyclic frame hierarchy");
    }

    auto next = ancestor.getParentFrame();

    // Defensive break in case getParentFrame() returns self (shouldn't happen).
    if (next.getEntity() == ancestor.getEntity()) {
      break;
    }

    ancestor = next;

    if (++depth > 1024) {
      DART_SIMULATION_THROW_T(
          InvalidOperationException, "Cycle detection depth limit exceeded");
    }
  }

  // Update parent in FrameState component (parentFrame is entt::entity
  // internally; world frame is represented as entt::null)
  const auto newParentFrame
      = parent.isWorld() ? entt::null : detail::toRegistryEntity(parentEntity);
  if (frameState->parentFrame != newParentFrame) {
    frameState->parentFrame = newParentFrame;
    m_world->markFrameTopologyChanged();
  }

  markSubtreeTransformCacheDirty();
}

//==============================================================================
const Eigen::Isometry3d& Frame::getTransform() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid frame handle");

  // World frame: return identity
  if (isWorld()) {
    static const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
    return identity;
  }

  const auto enttEntity = detail::toRegistryEntity(m_entity);

  // Frames with lazy evaluation (FreeFrame, FixedFrame) use FrameCache
  // getCacheMutable const_casts internally: cache mutation does not change
  // observable state
  auto* cache = ecs::getCacheMutable<Frame, comps::FrameCache>(*this);
  DART_SIMULATION_THROW_T_IF(
      !cache,
      InvalidOperationException,
      "Entity does not have FrameCache component (expected for "
      "FreeFrame/FixedFrame). Links should override getTransform().");

  if (cache->needTransformUpdate) {
    auto parent = getParentFrame();
    cache->worldTransform
        = parent.getTransform()
          * getFrameLocalTransform(
              dart::simulation::detail::registryOf(*m_world), enttEntity);
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
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid this frame");
  DART_SIMULATION_THROW_T_IF(
      !relativeTo.isValid(),
      InvalidArgumentException,
      "Invalid 'relativeTo' frame");

  // Both frames must be from the same World (or both are world frame)
  DART_SIMULATION_THROW_T_IF(
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
    return getFrameLocalTransform(
        dart::simulation::detail::registryOf(*m_world),
        detail::toRegistryEntity(m_entity));
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
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid this frame");
  DART_SIMULATION_THROW_T_IF(
      !to.isValid(), InvalidArgumentException, "Invalid 'to' frame");
  DART_SIMULATION_THROW_T_IF(
      !expressedIn.isValid(),
      InvalidArgumentException,
      "Invalid expressedIn frame");

  // All frames must be from the same World (world frame is compatible with any)
  const bool toIsWorld = to.isWorld();
  const bool coordIsWorld = expressedIn.isWorld();

  DART_SIMULATION_THROW_T_IF(
      !toIsWorld && m_world != to.m_world,
      InvalidArgumentException,
      "Frames are from different worlds");
  DART_SIMULATION_THROW_T_IF(
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
  // Return a special frame representing the world.
  // The default-constructed Entity sentinel (0xFFFFFFFF) marks the world frame;
  // nullptr world is also a marker.
  return Frame(Entity{}, nullptr);
}

//==============================================================================
bool Frame::isValid() const
{
  // World frame is valid (entity sentinel, world can be null or non-null)
  if (isWorld()) {
    return true;
  }

  // Regular frames need both entity and world
  if (m_world == nullptr) {
    return false;
  }

  const auto enttEntity = detail::toRegistryEntity(m_entity);

  // Check if entity exists in registry
  auto& registry = dart::simulation::detail::registryOf(*m_world);
  if (!registry.valid(enttEntity)) {
    return false;
  }

  // Check if entity has FrameTag
  if (!registry.all_of<comps::FrameTag>(enttEntity)) {
    return false;
  }

  return true;
}

//==============================================================================
void Frame::markSubtreeTransformCacheDirty()
{
  if (!m_world) {
    return;
  }

  markSubtreeCacheDirty(
      dart::simulation::detail::registryOf(*m_world),
      detail::toRegistryEntity(m_entity),
      m_world->getMemoryManager().getFreeAllocator());
}

} // namespace dart::simulation
