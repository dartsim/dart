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

#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/comps/collision_object.hpp>

#include <limits>

#include <cstdint>

namespace dart::collision::experimental {

CollisionObject::CollisionObject(entt::entity entity, CollisionWorld* world)
  : m_entity(entity), m_world(world)
{
}

const Shape* CollisionObject::getShape() const
{
  if (!isValid()) {
    return nullptr;
  }
  auto& registry = m_world->getRegistry();
  auto* shapeComp = registry.try_get<comps::ShapeComponent>(m_entity);
  return shapeComp ? shapeComp->shape.get() : nullptr;
}

ShapeType CollisionObject::getShapeType() const
{
  const auto* shape = getShape();
  return shape ? shape->getType() : ShapeType::Sphere;
}

const Eigen::Isometry3d& CollisionObject::getTransform() const
{
  static const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  if (!isValid()) {
    return identity;
  }
  auto& registry = m_world->getRegistry();
  auto* transformComp = registry.try_get<comps::TransformComponent>(m_entity);
  return transformComp ? transformComp->transform : identity;
}

void CollisionObject::setTransform(const Eigen::Isometry3d& transform)
{
  if (!isValid()) {
    return;
  }
  auto& registry = m_world->getRegistry();
  auto* transformComp = registry.try_get<comps::TransformComponent>(m_entity);
  if (transformComp) {
    transformComp->transform = transform;
  }
  auto* aabbComp = registry.try_get<comps::AabbComponent>(m_entity);
  if (aabbComp) {
    aabbComp->dirty = true;
  }
}

Aabb CollisionObject::computeAabb() const
{
  if (!isValid()) {
    return Aabb();
  }
  const auto* shape = getShape();
  if (!shape) {
    return Aabb();
  }
  Aabb localAabb = shape->computeLocalAabb();
  return Aabb::transformed(localAabb, getTransform());
}

void CollisionObject::setUserData(void* data)
{
  if (!isValid()) {
    return;
  }
  auto& registry = m_world->getRegistry();
  auto* userDataComp = registry.try_get<comps::UserDataComponent>(m_entity);
  if (userDataComp) {
    userDataComp->userData = data;
  }
}

void* CollisionObject::getUserData() const
{
  if (!isValid()) {
    return nullptr;
  }
  auto& registry = m_world->getRegistry();
  auto* userDataComp = registry.try_get<comps::UserDataComponent>(m_entity);
  return userDataComp ? userDataComp->userData : nullptr;
}

std::uint32_t CollisionObject::getCollisionGroup() const
{
  if (!isValid()) {
    return kCollisionGroupAll;
  }
  auto& registry = m_world->getRegistry();
  auto* filterComp
      = registry.try_get<comps::CollisionFilterComponent>(m_entity);
  return filterComp ? filterComp->filterData.collisionGroup
                    : kCollisionGroupAll;
}

void CollisionObject::setCollisionGroup(std::uint32_t group)
{
  if (!isValid()) {
    return;
  }
  auto& registry = m_world->getRegistry();
  auto* filterComp
      = registry.try_get<comps::CollisionFilterComponent>(m_entity);
  if (filterComp) {
    filterComp->filterData.collisionGroup = group;
  }
}

std::uint32_t CollisionObject::getCollisionMask() const
{
  if (!isValid()) {
    return kCollisionMaskAll;
  }
  auto& registry = m_world->getRegistry();
  auto* filterComp
      = registry.try_get<comps::CollisionFilterComponent>(m_entity);
  return filterComp ? filterComp->filterData.collisionMask : kCollisionMaskAll;
}

void CollisionObject::setCollisionMask(std::uint32_t mask)
{
  if (!isValid()) {
    return;
  }
  auto& registry = m_world->getRegistry();
  auto* filterComp
      = registry.try_get<comps::CollisionFilterComponent>(m_entity);
  if (filterComp) {
    filterComp->filterData.collisionMask = mask;
  }
}

void CollisionObject::setCollisionFilter(
    std::uint32_t group, std::uint32_t mask)
{
  if (!isValid()) {
    return;
  }
  auto& registry = m_world->getRegistry();
  auto* filterComp
      = registry.try_get<comps::CollisionFilterComponent>(m_entity);
  if (filterComp) {
    filterComp->filterData.collisionGroup = group;
    filterComp->filterData.collisionMask = mask;
  }
}

const CollisionFilterData& CollisionObject::getCollisionFilterData() const
{
  static const CollisionFilterData defaultData = CollisionFilterData::all();
  if (!isValid()) {
    return defaultData;
  }
  auto& registry = m_world->getRegistry();
  auto* filterComp
      = registry.try_get<comps::CollisionFilterComponent>(m_entity);
  return filterComp ? filterComp->filterData : defaultData;
}

void CollisionObject::setCollisionFilterData(
    const CollisionFilterData& filterData)
{
  if (!isValid()) {
    return;
  }
  auto& registry = m_world->getRegistry();
  auto* filterComp
      = registry.try_get<comps::CollisionFilterComponent>(m_entity);
  if (filterComp) {
    filterComp->filterData = filterData;
  }
}

bool CollisionObject::isValid() const
{
  if (m_entity == entt::null || m_world == nullptr) {
    return false;
  }
  return m_world->getRegistry().valid(m_entity);
}

std::size_t CollisionObject::getId() const
{
  if (!isValid()) {
    return std::numeric_limits<std::size_t>::max();
  }
  auto& registry = m_world->getRegistry();
  auto* broadPhaseComp = registry.try_get<comps::BroadPhaseComponent>(m_entity);
  return broadPhaseComp ? broadPhaseComp->broadPhaseId
                        : std::numeric_limits<std::size_t>::max();
}

} // namespace dart::collision::experimental
