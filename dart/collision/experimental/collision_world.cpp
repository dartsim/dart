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

#include <dart/collision/experimental/collision_world.hpp>

#include <dart/collision/experimental/comps/collision_object.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>

#include <algorithm>
#include <limits>

namespace dart::collision::experimental {

CollisionWorld::CollisionWorld() = default;
CollisionWorld::~CollisionWorld() = default;

CollisionObject CollisionWorld::createObject(
    std::unique_ptr<Shape> shape,
    const Eigen::Isometry3d& transform)
{
  if (!shape) {
    return CollisionObject();
  }

  auto entity = m_registry.create();

  m_registry.emplace<comps::CollisionObjectTag>(entity);
  m_registry.emplace<comps::ShapeComponent>(entity, std::move(shape));
  m_registry.emplace<comps::TransformComponent>(entity, transform);
  m_registry.emplace<comps::AabbComponent>(entity);
  m_registry.emplace<comps::UserDataComponent>(entity);

  CollisionObject obj(entity, this);
  Aabb aabb = obj.computeAabb();

  auto& aabbComp = m_registry.get<comps::AabbComponent>(entity);
  aabbComp.aabb = aabb;
  aabbComp.dirty = false;

  m_broadPhase.add(static_cast<std::size_t>(entity), aabb);

  return obj;
}

void CollisionWorld::destroyObject(CollisionObject object)
{
  if (!object.isValid() || object.getWorld() != this) {
    return;
  }

  auto entity = object.getEntity();
  m_broadPhase.remove(static_cast<std::size_t>(entity));
  m_registry.destroy(entity);
}

void CollisionWorld::updateObject(CollisionObject object)
{
  if (!object.isValid() || object.getWorld() != this) {
    return;
  }

  auto entity = object.getEntity();
  auto* aabbComp = m_registry.try_get<comps::AabbComponent>(entity);

  if (aabbComp && aabbComp->dirty) {
    Aabb aabb = object.computeAabb();
    aabbComp->aabb = aabb;
    aabbComp->dirty = false;
    m_broadPhase.update(static_cast<std::size_t>(entity), aabb);
  }
}

std::size_t CollisionWorld::numObjects() const
{
  return m_registry.view<comps::CollisionObjectTag>().size();
}

CollisionObject CollisionWorld::getObject(std::size_t index)
{
  auto view = m_registry.view<comps::CollisionObjectTag>();
  std::size_t i = 0;
  for (auto entity : view) {
    if (i == index) {
      return CollisionObject(entity, this);
    }
    ++i;
  }
  return CollisionObject();
}

std::size_t CollisionWorld::updateAll()
{
  std::size_t updated = 0;
  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    auto* aabbComp = m_registry.try_get<comps::AabbComponent>(entity);
    if (aabbComp && aabbComp->dirty) {
      CollisionObject obj(entity, this);
      Aabb aabb = obj.computeAabb();
      aabbComp->aabb = aabb;
      aabbComp->dirty = false;
      m_broadPhase.update(static_cast<std::size_t>(entity), aabb);
      ++updated;
    }
  }
  return updated;
}

BroadPhaseSnapshot CollisionWorld::buildBroadPhaseSnapshot() const
{
  BroadPhaseSnapshot snapshot;
  snapshot.pairs = m_broadPhase.queryPairs();
  snapshot.numObjects = m_broadPhase.size();
  return snapshot;
}

bool CollisionWorld::collideAll(
    const BroadPhaseSnapshot& snapshot,
    const CollisionOption& option,
    CollisionResult& result,
    BatchStats* stats)
{
  result.clear();
  bool hasCollision = false;

  if (stats) {
    stats->numObjects = snapshot.numObjects;
    stats->numPairs = snapshot.pairs.size();
    stats->numPairsTested = 0;
    stats->numContacts = 0;
    stats->pairBytes = snapshot.pairs.size() * sizeof(BroadPhasePair);
    stats->contactBytes = 0;
    stats->tempBytes = stats->pairBytes;
  }

  for (const auto& pair : snapshot.pairs) {
    auto entity1 = static_cast<entt::entity>(pair.first);
    auto entity2 = static_cast<entt::entity>(pair.second);

    if (!m_registry.valid(entity1) || !m_registry.valid(entity2)) {
      continue;
    }

    CollisionObject obj1(entity1, this);
    CollisionObject obj2(entity2, this);

    if (stats) {
      ++stats->numPairsTested;
    }

    if (NarrowPhase::collide(obj1, obj2, option, result)) {
      hasCollision = true;
      if (option.enableContact == false
          || (option.maxNumContacts > 0
              && result.numContacts() >= option.maxNumContacts)) {
        break;
      }
    }
  }

  if (stats) {
    stats->numContacts = result.numContacts();
    stats->contactBytes = result.numContacts() * sizeof(ContactPoint);
    stats->tempBytes = stats->pairBytes + stats->contactBytes;
  }

  return hasCollision;
}

bool CollisionWorld::collide(const CollisionOption& option, CollisionResult& result)
{
  result.clear();
  bool hasCollision = false;

  auto pairs = m_broadPhase.queryPairs();

  for (const auto& pair : pairs) {
    auto entity1 = static_cast<entt::entity>(pair.first);
    auto entity2 = static_cast<entt::entity>(pair.second);

    if (!m_registry.valid(entity1) || !m_registry.valid(entity2)) {
      continue;
    }

    CollisionObject obj1(entity1, this);
    CollisionObject obj2(entity2, this);

    if (NarrowPhase::collide(obj1, obj2, option, result)) {
      hasCollision = true;
      if (option.enableContact == false
          || (option.maxNumContacts > 0 && result.numContacts() >= option.maxNumContacts)) {
        break;
      }
    }
  }

  return hasCollision;
}

bool CollisionWorld::collide(
    CollisionObject obj1,
    CollisionObject obj2,
    const CollisionOption& option,
    CollisionResult& result)
{
  if (!obj1.isValid() || !obj2.isValid()) {
    return false;
  }
  return NarrowPhase::collide(obj1, obj2, option, result);
}

bool CollisionWorld::raycast(
    const Ray& ray,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  RaycastResult closestResult;
  double closestDistance = std::numeric_limits<double>::max();

  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    CollisionObject obj(entity, this);
    RaycastResult tempResult;
    if (NarrowPhase::raycast(ray, obj, option, tempResult)) {
      if (tempResult.distance < closestDistance) {
        closestDistance = tempResult.distance;
        closestResult = tempResult;
      }
    }
  }

  if (closestResult.hit) {
    result = closestResult;
    return true;
  }

  return false;
}

bool CollisionWorld::raycastAll(
    const Ray& ray,
    const RaycastOption& option,
    std::vector<RaycastResult>& results)
{
  results.clear();

  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    CollisionObject obj(entity, this);
    RaycastResult tempResult;
    if (NarrowPhase::raycast(ray, obj, option, tempResult)) {
      results.push_back(tempResult);
    }
  }

  std::sort(results.begin(), results.end(), [](const RaycastResult& a, const RaycastResult& b) {
    return a.distance < b.distance;
  });

  return !results.empty();
}

bool CollisionWorld::sphereCast(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double radius,
    const CcdOption& option,
    CcdResult& result)
{
  result = CcdResult();

  CcdResult closestResult;
  double closestToi = std::numeric_limits<double>::max();

  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    CollisionObject obj(entity, this);
    CcdResult tempResult;
    if (NarrowPhase::sphereCast(start, end, radius, obj, option, tempResult)) {
      if (tempResult.timeOfImpact < closestToi) {
        closestToi = tempResult.timeOfImpact;
        closestResult = tempResult;
      }
    }
  }

  if (closestResult.hit) {
    result = closestResult;
    return true;
  }

  return false;
}

bool CollisionWorld::sphereCastAll(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double radius,
    const CcdOption& option,
    std::vector<CcdResult>& results)
{
  results.clear();

  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    CollisionObject obj(entity, this);
    CcdResult tempResult;
    if (NarrowPhase::sphereCast(start, end, radius, obj, option, tempResult)) {
      results.push_back(tempResult);
    }
  }

  std::sort(results.begin(), results.end(), [](const CcdResult& a, const CcdResult& b) {
    return a.timeOfImpact < b.timeOfImpact;
  });

  return !results.empty();
}

bool CollisionWorld::capsuleCast(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CcdOption& option,
    CcdResult& result)
{
  result = CcdResult();

  CcdResult closestResult;
  double closestToi = std::numeric_limits<double>::max();

  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    CollisionObject obj(entity, this);
    CcdResult tempResult;
    if (NarrowPhase::capsuleCast(capsuleStart, capsuleEnd, capsule, obj, option, tempResult)) {
      if (tempResult.timeOfImpact < closestToi) {
        closestToi = tempResult.timeOfImpact;
        closestResult = tempResult;
      }
    }
  }

  if (closestResult.hit) {
    result = closestResult;
    return true;
  }

  return false;
}

bool CollisionWorld::capsuleCastAll(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CcdOption& option,
    std::vector<CcdResult>& results)
{
  results.clear();

  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    CollisionObject obj(entity, this);
    CcdResult tempResult;
    if (NarrowPhase::capsuleCast(capsuleStart, capsuleEnd, capsule, obj, option, tempResult)) {
      results.push_back(tempResult);
    }
  }

  std::sort(results.begin(), results.end(), [](const CcdResult& a, const CcdResult& b) {
    return a.timeOfImpact < b.timeOfImpact;
  });

  return !results.empty();
}

void CollisionWorld::clear()
{
  m_broadPhase.clear();
  m_registry.clear();
}

} // namespace dart::collision::experimental
