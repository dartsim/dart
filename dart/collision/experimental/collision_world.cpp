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

#include <dart/collision/experimental/broad_phase/aabb_tree.hpp>
#include <dart/collision/experimental/broad_phase/brute_force.hpp>
#include <dart/collision/experimental/broad_phase/spatial_hash.hpp>
#include <dart/collision/experimental/broad_phase/sweep_and_prune.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/comps/collision_object.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>

#include <algorithm>
#include <limits>

namespace dart::collision::experimental {

CollisionWorld::CollisionWorld(BroadPhaseType broadPhaseType)
  : m_broadPhaseType(broadPhaseType),
    m_broadPhase(createBroadPhase(broadPhaseType))
{
}

CollisionWorld::~CollisionWorld() = default;

std::unique_ptr<BroadPhase> CollisionWorld::createBroadPhase(
    BroadPhaseType type)
{
  switch (type) {
    case BroadPhaseType::BruteForce:
      return std::make_unique<BruteForceBroadPhase>();
    case BroadPhaseType::AabbTree:
      return std::make_unique<AabbTreeBroadPhase>();
    case BroadPhaseType::SpatialHash:
      return std::make_unique<SpatialHashBroadPhase>();
    case BroadPhaseType::SweepAndPrune:
      return std::make_unique<SweepAndPruneBroadPhase>();
  }
  return std::make_unique<AabbTreeBroadPhase>();
}

CollisionObject CollisionWorld::createObject(
    std::unique_ptr<Shape> shape, const Eigen::Isometry3d& transform)
{
  if (!shape) {
    return CollisionObject();
  }

  auto entity = m_registry.create();

  m_registry.emplace<comps::CollisionObjectTag>(entity);
  m_registry.emplace<comps::ShapeComponent>(entity, std::move(shape));
  m_registry.emplace<comps::TransformComponent>(entity, transform);
  m_registry.emplace<comps::AabbComponent>(entity);
  auto& broadPhaseComp = m_registry.emplace<comps::BroadPhaseComponent>(entity);
  m_registry.emplace<comps::UserDataComponent>(entity);

  CollisionObject obj(entity, this);
  Aabb aabb = obj.computeAabb();

  auto& aabbComp = m_registry.get<comps::AabbComponent>(entity);
  aabbComp.aabb = aabb;
  aabbComp.dirty = false;

  const auto& shapeComp = m_registry.get<comps::ShapeComponent>(entity);
  const auto& transformComp = m_registry.get<comps::TransformComponent>(entity);

  broadPhaseComp.broadPhaseId = m_nextObjectId++;
  if (broadPhaseComp.broadPhaseId >= m_idToEntity.size()) {
    m_idToEntity.resize(broadPhaseComp.broadPhaseId + 1, entt::null);
  }
  m_idToEntity[broadPhaseComp.broadPhaseId] = entity;

  m_broadPhase->add(broadPhaseComp.broadPhaseId, aabb);
  m_batchStorage.add(
      broadPhaseComp.broadPhaseId,
      shapeComp.shape.get(),
      transformComp.transform,
      aabb);
  m_snapshotDirty = true;

  return obj;
}

void CollisionWorld::destroyObject(CollisionObject object)
{
  if (!object.isValid() || object.getWorld() != this) {
    return;
  }

  auto entity = object.getEntity();
  auto* broadPhaseComp = m_registry.try_get<comps::BroadPhaseComponent>(entity);
  if (broadPhaseComp) {
    m_broadPhase->remove(broadPhaseComp->broadPhaseId);
    if (broadPhaseComp->broadPhaseId < m_idToEntity.size()) {
      m_idToEntity[broadPhaseComp->broadPhaseId] = entt::null;
    }
    m_batchStorage.remove(broadPhaseComp->broadPhaseId);
  } else {
    m_broadPhase->remove(static_cast<std::size_t>(entity));
  }
  m_registry.destroy(entity);
  m_snapshotDirty = true;
}

void CollisionWorld::updateObject(CollisionObject object)
{
  if (!object.isValid() || object.getWorld() != this) {
    return;
  }

  auto entity = object.getEntity();
  auto* aabbComp = m_registry.try_get<comps::AabbComponent>(entity);
  auto* broadPhaseComp = m_registry.try_get<comps::BroadPhaseComponent>(entity);

  if (aabbComp && aabbComp->dirty) {
    Aabb aabb = object.computeAabb();
    aabbComp->aabb = aabb;
    aabbComp->dirty = false;
    if (broadPhaseComp) {
      m_broadPhase->update(broadPhaseComp->broadPhaseId, aabb);
      m_batchStorage.update(
          broadPhaseComp->broadPhaseId,
          object.getShape(),
          object.getTransform(),
          aabb);
    } else {
      m_broadPhase->update(static_cast<std::size_t>(entity), aabb);
    }
    m_snapshotDirty = true;
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

CollisionObject CollisionWorld::getObjectById(std::size_t id)
{
  if (id >= m_idToEntity.size()) {
    return CollisionObject();
  }
  auto entity = m_idToEntity[id];
  if (entity == entt::null || !m_registry.valid(entity)) {
    return CollisionObject();
  }
  return CollisionObject(entity, this);
}

BatchView CollisionWorld::getBatchView() const
{
  return m_batchStorage.view();
}

void CollisionWorld::reserveObjects(std::size_t count)
{
  m_batchStorage.reserve(count);
  if (count > m_idToEntity.size()) {
    m_idToEntity.resize(count, entt::null);
  }
}

std::size_t CollisionWorld::updateAll()
{
  BatchSettings settings;
  return updateAll(settings, nullptr);
}

std::size_t CollisionWorld::updateAll(
    const BatchSettings& settings, BatchStats* stats)
{
  (void)settings;
  std::size_t updated = 0;
  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    auto* aabbComp = m_registry.try_get<comps::AabbComponent>(entity);
    auto* broadPhaseComp
        = m_registry.try_get<comps::BroadPhaseComponent>(entity);
    if (aabbComp && aabbComp->dirty) {
      CollisionObject obj(entity, this);
      Aabb aabb = obj.computeAabb();
      aabbComp->aabb = aabb;
      aabbComp->dirty = false;
      if (broadPhaseComp) {
        m_broadPhase->update(broadPhaseComp->broadPhaseId, aabb);
        m_batchStorage.update(
            broadPhaseComp->broadPhaseId,
            obj.getShape(),
            obj.getTransform(),
            aabb);
      } else {
        m_broadPhase->update(static_cast<std::size_t>(entity), aabb);
      }
      ++updated;
    }
  }

  if (stats) {
    stats->numObjects = numObjects();
    stats->numAabbUpdates = updated;
  }

  if (updated > 0) {
    m_snapshotDirty = true;
  }

  return updated;
}

BroadPhaseSnapshot CollisionWorld::buildBroadPhaseSnapshot() const
{
  BatchSettings settings;
  return buildBroadPhaseSnapshot(settings);
}

BroadPhaseSnapshot CollisionWorld::buildBroadPhaseSnapshot(
    const BatchSettings& settings) const
{
  BroadPhaseSnapshot snapshot;
  buildBroadPhaseSnapshot(snapshot, settings);
  return snapshot;
}

void CollisionWorld::buildBroadPhaseSnapshot(BroadPhaseSnapshot& out) const
{
  BatchSettings settings;
  buildBroadPhaseSnapshot(out, settings);
}

void CollisionWorld::buildBroadPhaseSnapshot(
    BroadPhaseSnapshot& out, const BatchSettings& settings) const
{
  if (!m_snapshotDirty && m_cachedDeterministic == settings.deterministic) {
    out = m_cachedSnapshot;
    return;
  }

  out.pairs.clear();
  m_broadPhase->queryPairs(out.pairs);
  out.numObjects = m_broadPhase->size();

  if (settings.deterministic && out.pairs.size() > 1) {
    for (auto& pair : out.pairs) {
      if (pair.second < pair.first) {
        std::swap(pair.first, pair.second);
      }
    }
    std::sort(out.pairs.begin(), out.pairs.end());
  }

  m_cachedSnapshot = out;
  m_snapshotDirty = false;
  m_cachedDeterministic = settings.deterministic;
}

bool CollisionWorld::collideAll(
    const BroadPhaseSnapshot& snapshot,
    const CollisionOption& option,
    CollisionResult& result,
    BatchStats* stats)
{
  BatchSettings settings;
  return collideAll(snapshot, option, result, settings, stats);
}

bool CollisionWorld::collideAll(
    const BroadPhaseSnapshot& snapshot,
    const CollisionOption& option,
    CollisionResult& result,
    const BatchSettings& settings,
    BatchStats* stats)
{
  (void)settings;
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

  auto view = getBatchView();
  for (const auto& pair : snapshot.pairs) {
    const auto* shape1 = view.shape(pair.first);
    const auto* shape2 = view.shape(pair.second);
    const auto* tf1 = view.transform(pair.first);
    const auto* tf2 = view.transform(pair.second);

    if (!shape1 || !shape2 || !tf1 || !tf2) {
      continue;
    }

    if (stats) {
      ++stats->numPairsTested;
    }

    if (NarrowPhase::collide(shape1, *tf1, shape2, *tf2, option, result)) {
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

bool CollisionWorld::collideAll(
    const BroadPhaseSnapshot& snapshot,
    const CollisionOption& option,
    BatchOutput& out,
    const BatchSettings& settings,
    BatchStats* stats)
{
  out.clear();
  out.pairs = snapshot.pairs;
  return collideAll(snapshot, option, out.result, settings, stats);
}

bool CollisionWorld::collide(
    const CollisionOption& option, CollisionResult& result)
{
  result.clear();
  bool hasCollision = false;

  auto pairs = m_broadPhase->queryPairs();

  auto view = getBatchView();
  for (const auto& pair : pairs) {
    const auto* shape1 = view.shape(pair.first);
    const auto* shape2 = view.shape(pair.second);
    const auto* tf1 = view.transform(pair.first);
    const auto* tf2 = view.transform(pair.second);

    if (!shape1 || !shape2 || !tf1 || !tf2) {
      continue;
    }

    if (NarrowPhase::collide(shape1, *tf1, shape2, *tf2, option, result)) {
      hasCollision = true;
      if (option.enableContact == false
          || (option.maxNumContacts > 0
              && result.numContacts() >= option.maxNumContacts)) {
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
    const Ray& ray, const RaycastOption& option, RaycastResult& result)
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

  std::sort(
      results.begin(),
      results.end(),
      [](const RaycastResult& a, const RaycastResult& b) {
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

  std::sort(
      results.begin(),
      results.end(),
      [](const CcdResult& a, const CcdResult& b) {
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
    if (NarrowPhase::capsuleCast(
            capsuleStart, capsuleEnd, capsule, obj, option, tempResult)) {
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
    if (NarrowPhase::capsuleCast(
            capsuleStart, capsuleEnd, capsule, obj, option, tempResult)) {
      results.push_back(tempResult);
    }
  }

  std::sort(
      results.begin(),
      results.end(),
      [](const CcdResult& a, const CcdResult& b) {
        return a.timeOfImpact < b.timeOfImpact;
      });

  return !results.empty();
}

void CollisionWorld::clear()
{
  m_broadPhase->clear();
  m_registry.clear();
  m_idToEntity.clear();
  m_nextObjectId = 0;
  m_batchStorage.clear();
  m_cachedSnapshot.clear();
  m_snapshotDirty = true;
  m_cachedDeterministic = true;
}

} // namespace dart::collision::experimental
