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

#include <dart/collision/native/broad_phase/aabb_tree.hpp>
#include <dart/collision/native/broad_phase/brute_force.hpp>
#include <dart/collision/native/broad_phase/spatial_hash.hpp>
#include <dart/collision/native/broad_phase/sweep_and_prune.hpp>
#include <dart/collision/native/collision_filter.hpp>
#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/comps/collision_object.hpp>
#include <dart/collision/native/narrow_phase/box_box.hpp>
#include <dart/collision/native/narrow_phase/capsule_box.hpp>
#include <dart/collision/native/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/native/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/narrow_phase/sphere_box.hpp>
#include <dart/collision/native/narrow_phase/sphere_sphere.hpp>

#include <dart/common/parallel_for.hpp>

#include <algorithm>
#include <limits>
#include <utility>

namespace dart::collision::native {

namespace {

constexpr int pairKey(ShapeType type1, ShapeType type2)
{
  return (static_cast<int>(type1) << 8) | static_cast<int>(type2);
}

Aabb sweptSphereAabb(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end, double radius)
{
  Aabb aabb(start.cwiseMin(end), start.cwiseMax(end));
  aabb.expand(std::max(0.0, radius));
  return aabb;
}

Aabb sweptCapsuleAabb(
    const Eigen::Isometry3d& start,
    const Eigen::Isometry3d& end,
    const CapsuleShape& capsule)
{
  const double boundingRadius
      = std::max(0.0, capsule.getRadius() + 0.5 * capsule.getHeight());
  return sweptSphereAabb(
      start.translation(), end.translation(), boundingRadius);
}

template <typename CollideFn>
bool collideWithFlippedNormals(
    CollisionResult& result,
    const CollisionOption& option,
    CollideFn&& collideFn)
{
  const auto existingContacts = result.numContacts();
  if (option.enableContact && existingContacts >= option.maxNumContacts) {
    return false;
  }

  CollisionOption localOption = option;
  if (option.enableContact) {
    localOption.maxNumContacts = option.maxNumContacts - existingContacts;
  }

  CollisionResult localResult;
  const bool hit = collideFn(localResult, localOption);
  if (!hit) {
    return false;
  }

  const auto numContacts = localResult.numContacts();
  for (std::size_t i = 0; i < numContacts; ++i) {
    ContactPoint contact = localResult.getContact(i);
    contact.normal = -contact.normal;
    result.addContact(contact);
  }

  return true;
}

bool collideShapePair(
    const Shape* shape1,
    const Eigen::Isometry3d& tf1,
    const Shape* shape2,
    const Eigen::Isometry3d& tf2,
    const CollisionOption& option,
    CollisionResult& result)
{
  const ShapeType type1 = shape1->getType();
  const ShapeType type2 = shape2->getType();

  switch (pairKey(type1, type2)) {
    case pairKey(ShapeType::Sphere, ShapeType::Sphere):
      return collideSpheres(
          *static_cast<const SphereShape*>(shape1),
          tf1,
          *static_cast<const SphereShape*>(shape2),
          tf2,
          result,
          option);
    case pairKey(ShapeType::Box, ShapeType::Box):
      return collideBoxes(
          *static_cast<const BoxShape*>(shape1),
          tf1,
          *static_cast<const BoxShape*>(shape2),
          tf2,
          result,
          option);
    case pairKey(ShapeType::Sphere, ShapeType::Box):
      return collideWithFlippedNormals(
          result,
          option,
          [&](CollisionResult& local, const CollisionOption& opt) {
            return collideSphereBox(
                *static_cast<const SphereShape*>(shape1),
                tf1,
                *static_cast<const BoxShape*>(shape2),
                tf2,
                local,
                opt);
          });
    case pairKey(ShapeType::Box, ShapeType::Sphere):
      return collideSphereBox(
          *static_cast<const SphereShape*>(shape2),
          tf2,
          *static_cast<const BoxShape*>(shape1),
          tf1,
          result,
          option);
    case pairKey(ShapeType::Capsule, ShapeType::Capsule):
      return collideCapsules(
          *static_cast<const CapsuleShape*>(shape1),
          tf1,
          *static_cast<const CapsuleShape*>(shape2),
          tf2,
          result,
          option);
    case pairKey(ShapeType::Capsule, ShapeType::Sphere):
      return collideCapsuleSphere(
          *static_cast<const CapsuleShape*>(shape1),
          tf1,
          *static_cast<const SphereShape*>(shape2),
          tf2,
          result,
          option);
    case pairKey(ShapeType::Sphere, ShapeType::Capsule):
      return collideWithFlippedNormals(
          result,
          option,
          [&](CollisionResult& local, const CollisionOption& opt) {
            return collideCapsuleSphere(
                *static_cast<const CapsuleShape*>(shape2),
                tf2,
                *static_cast<const SphereShape*>(shape1),
                tf1,
                local,
                opt);
          });
    case pairKey(ShapeType::Capsule, ShapeType::Box):
      return collideWithFlippedNormals(
          result,
          option,
          [&](CollisionResult& local, const CollisionOption& opt) {
            return collideCapsuleBox(
                *static_cast<const CapsuleShape*>(shape1),
                tf1,
                *static_cast<const BoxShape*>(shape2),
                tf2,
                local,
                opt);
          });
    case pairKey(ShapeType::Box, ShapeType::Capsule):
      return collideCapsuleBox(
          *static_cast<const CapsuleShape*>(shape2),
          tf2,
          *static_cast<const BoxShape*>(shape1),
          tf1,
          result,
          option);
    default:
      return NarrowPhase::collide(shape1, tf1, shape2, tf2, option, result);
  }
}

} // namespace

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

void CollisionWorld::notifyCollisionFilterChanged()
{
  m_hasCustomCollisionFilters = true;
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
  m_registry.emplace<comps::CollisionFilterComponent>(entity);

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

void CollisionWorld::prepareQueryObjectCache()
{
  m_queryObjectCache.clear();
  m_queryObjectCache.reserve(numObjects());
}

const CollisionObject* CollisionWorld::cacheQueryObject(CollisionObject object)
{
  m_queryObjectCache.push_back(object);
  return &m_queryObjectCache.back();
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

std::size_t CollisionWorld::updateDirty(std::span<const ObjectId> ids)
{
  BatchSettings settings;
  return updateDirty(ids, settings, nullptr);
}

std::size_t CollisionWorld::updateDirty(
    std::span<const ObjectId> ids,
    const BatchSettings& settings,
    BatchStats* stats)
{
  (void)settings;
  std::size_t updated = 0;

  for (ObjectId id : ids) {
    if (id >= m_idToEntity.size()) {
      continue;
    }
    auto entity = m_idToEntity[id];
    if (entity == entt::null || !m_registry.valid(entity)) {
      continue;
    }

    auto* aabbComp = m_registry.try_get<comps::AabbComponent>(entity);
    if (!aabbComp || !aabbComp->dirty) {
      continue;
    }

    CollisionObject obj(entity, this);
    Aabb aabb = obj.computeAabb();
    aabbComp->aabb = aabb;
    aabbComp->dirty = false;

    auto* broadPhaseComp
        = m_registry.try_get<comps::BroadPhaseComponent>(entity);
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

BroadPhaseDebugSnapshot CollisionWorld::buildBroadPhaseDebugSnapshot() const
{
  BroadPhaseDebugSnapshot snapshot;
  buildBroadPhaseDebugSnapshot(snapshot);
  return snapshot;
}

void CollisionWorld::buildBroadPhaseDebugSnapshot(
    BroadPhaseDebugSnapshot& out) const
{
  m_broadPhase->buildDebugSnapshot(out);
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
  const bool needsFilterCheck
      = m_hasCustomCollisionFilters || option.collisionFilter != nullptr;
  if (settings.maxThreads <= 1 || snapshot.pairs.size() < settings.grainSize) {
    for (const auto& pair : snapshot.pairs) {
      const auto index1 = view.indexForId(pair.first);
      const auto index2 = view.indexForId(pair.second);
      if (index1 == view.invalidIndex || index2 == view.invalidIndex
          || index1 >= view.shapes.size() || index2 >= view.shapes.size()
          || index1 >= view.transforms.size()
          || index2 >= view.transforms.size()) {
        continue;
      }

      const auto* shape1 = view.shapes[index1];
      const auto* shape2 = view.shapes[index2];
      if (!shape1 || !shape2) {
        continue;
      }
      const auto& tf1 = view.transforms[index1];
      const auto& tf2 = view.transforms[index2];

      if (needsFilterCheck && pair.first < m_idToEntity.size()
          && pair.second < m_idToEntity.size()) {
        auto entity1 = m_idToEntity[pair.first];
        auto entity2 = m_idToEntity[pair.second];
        if (entity1 != entt::null && entity2 != entt::null) {
          auto* filter1
              = m_registry.try_get<comps::CollisionFilterComponent>(entity1);
          auto* filter2
              = m_registry.try_get<comps::CollisionFilterComponent>(entity2);
          if (filter1 && filter2) {
            if (m_hasCustomCollisionFilters
                && !shouldCollide(filter1->filterData, filter2->filterData)) {
              continue;
            }
            if (option.collisionFilter) {
              CollisionObject obj1(entity1, this);
              CollisionObject obj2(entity2, this);
              if (option.collisionFilter->ignoresCollision(obj1, obj2)) {
                continue;
              }
            }
          }
        }
      }

      if (stats) {
        ++stats->numPairsTested;
      }

      if (collideShapePair(shape1, tf1, shape2, tf2, option, result)) {
        hasCollision = true;
        if (option.enableContact == false
            || (option.maxNumContacts > 0
                && result.numContacts() >= option.maxNumContacts)) {
          break;
        }
      }
    }
  } else {
    const std::size_t numPairs = snapshot.pairs.size();
    const std::size_t chunkCount = dart::common::parallelForChunkCount(
        numPairs,
        settings.grainSize,
        static_cast<std::size_t>(settings.maxThreads));

    std::vector<CollisionResult> threadResults(chunkCount);
    std::vector<std::vector<BroadPhasePair>> threadPairs(chunkCount);
    std::vector<std::size_t> threadPairsTested(chunkCount, 0);
    std::vector<char> threadHasCollision(chunkCount, 0);

    dart::common::parallelForChunks(
        numPairs,
        settings.grainSize,
        static_cast<std::size_t>(settings.maxThreads),
        [&](std::size_t begin, std::size_t end, std::size_t t) {
          auto& localResult = threadResults[t];
          auto& localPairs = threadPairs[t];
          std::size_t localPairsTested = 0;
          bool localHasCollision = false;

          for (std::size_t i = begin; i < end; ++i) {
            const auto& pair = snapshot.pairs[i];
            const auto index1 = view.indexForId(pair.first);
            const auto index2 = view.indexForId(pair.second);
            if (index1 == view.invalidIndex || index2 == view.invalidIndex
                || index1 >= view.shapes.size() || index2 >= view.shapes.size()
                || index1 >= view.transforms.size()
                || index2 >= view.transforms.size()) {
              continue;
            }

            const auto* shape1 = view.shapes[index1];
            const auto* shape2 = view.shapes[index2];
            if (!shape1 || !shape2) {
              continue;
            }
            const auto& tf1 = view.transforms[index1];
            const auto& tf2 = view.transforms[index2];

            if (needsFilterCheck && pair.first < m_idToEntity.size()
                && pair.second < m_idToEntity.size()) {
              auto entity1 = m_idToEntity[pair.first];
              auto entity2 = m_idToEntity[pair.second];
              if (entity1 != entt::null && entity2 != entt::null) {
                auto* filter1
                    = m_registry.try_get<comps::CollisionFilterComponent>(
                        entity1);
                auto* filter2
                    = m_registry.try_get<comps::CollisionFilterComponent>(
                        entity2);
                if (filter1 && filter2) {
                  if (m_hasCustomCollisionFilters
                      && !shouldCollide(
                          filter1->filterData, filter2->filterData)) {
                    continue;
                  }
                  if (option.collisionFilter) {
                    CollisionObject obj1(entity1, this);
                    CollisionObject obj2(entity2, this);
                    if (option.collisionFilter->ignoresCollision(obj1, obj2)) {
                      continue;
                    }
                  }
                }
              }
            }

            ++localPairsTested;

            const std::size_t manifoldsBefore = localResult.numManifolds();
            if (collideShapePair(
                    shape1, tf1, shape2, tf2, option, localResult)) {
              localHasCollision = true;
              const std::size_t manifoldsAfter = localResult.numManifolds();
              for (std::size_t m = manifoldsBefore; m < manifoldsAfter; ++m) {
                localPairs.push_back(pair);
              }
              if (option.enableContact == false
                  || (option.maxNumContacts > 0
                      && localResult.numContacts() >= option.maxNumContacts)) {
                break;
              }
            }
          }

          threadPairsTested[t] = localPairsTested;
          threadHasCollision[t] = localHasCollision ? 1 : 0;
        });

    std::vector<std::pair<BroadPhasePair, ContactManifold>> merged;
    std::size_t mergedReserve = 0;
    for (const auto& pairs : threadPairs) {
      mergedReserve += pairs.size();
    }
    merged.reserve(mergedReserve);

    for (std::size_t t = 0; t < threadResults.size(); ++t) {
      if (threadHasCollision[t] != 0) {
        hasCollision = true;
      }
      if (stats) {
        stats->numPairsTested += threadPairsTested[t];
      }

      const auto manifolds = threadResults[t].getManifolds();
      const auto& pairs = threadPairs[t];
      const std::size_t count = std::min(manifolds.size(), pairs.size());
      for (std::size_t i = 0; i < count; ++i) {
        merged.emplace_back(pairs[i], manifolds[i]);
      }
    }

    if (settings.deterministic && merged.size() > 1) {
      std::stable_sort(
          merged.begin(), merged.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
          });
    }

    std::size_t contactCount = 0;
    BroadPhasePair currentPair;
    bool hasCurrentPair = false;
    for (std::size_t i = 0; i < merged.size(); ++i) {
      const auto& entry = merged[i];
      if (!hasCurrentPair) {
        currentPair = entry.first;
        hasCurrentPair = true;
      } else if (entry.first != currentPair) {
        if (option.enableContact == false) {
          break;
        }
        if (option.maxNumContacts > 0
            && contactCount >= option.maxNumContacts) {
          break;
        }
        currentPair = entry.first;
      }

      result.addManifold(entry.second);
      contactCount += entry.second.numContacts();
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
  if (!m_snapshotDirty) {
    return collideAll(m_cachedSnapshot, option, result);
  }

  result.clear();
  bool hasCollision = false;

  auto view = getBatchView();
  const bool needsFilterCheck
      = m_hasCustomCollisionFilters || option.collisionFilter != nullptr;
  auto visitPair = [&](std::size_t first, std::size_t second) {
    const auto index1 = view.indexForId(first);
    const auto index2 = view.indexForId(second);
    if (index1 == view.invalidIndex || index2 == view.invalidIndex
        || index1 >= view.shapes.size() || index2 >= view.shapes.size()
        || index1 >= view.transforms.size()
        || index2 >= view.transforms.size()) {
      return true;
    }

    const auto* shape1 = view.shapes[index1];
    const auto* shape2 = view.shapes[index2];
    if (!shape1 || !shape2) {
      return true;
    }
    const auto& tf1 = view.transforms[index1];
    const auto& tf2 = view.transforms[index2];

    if (needsFilterCheck && first < m_idToEntity.size()
        && second < m_idToEntity.size()) {
      auto entity1 = m_idToEntity[first];
      auto entity2 = m_idToEntity[second];
      if (entity1 != entt::null && entity2 != entt::null) {
        auto* filter1
            = m_registry.try_get<comps::CollisionFilterComponent>(entity1);
        auto* filter2
            = m_registry.try_get<comps::CollisionFilterComponent>(entity2);
        if (filter1 && filter2) {
          if (m_hasCustomCollisionFilters
              && !shouldCollide(filter1->filterData, filter2->filterData)) {
            return true;
          }
          if (option.collisionFilter) {
            CollisionObject obj1(entity1, this);
            CollisionObject obj2(entity2, this);
            if (option.collisionFilter->ignoresCollision(obj1, obj2)) {
              return true;
            }
          }
        }
      }
    }

    if (collideShapePair(shape1, tf1, shape2, tf2, option, result)) {
      hasCollision = true;
      if (option.enableContact == false
          || (option.maxNumContacts > 0
              && result.numContacts() >= option.maxNumContacts)) {
        return false;
      }
    }
    return true;
  };

  if (m_broadPhaseType == BroadPhaseType::AabbTree) {
    static_cast<const AabbTreeBroadPhase*>(m_broadPhase.get())
        ->visitPairsFast(visitPair);
  } else {
    m_broadPhase->visitPairs(visitPair);
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

  const auto& filter1 = obj1.getCollisionFilterData();
  const auto& filter2 = obj2.getCollisionFilterData();
  if (!shouldCollide(filter1, filter2)) {
    return false;
  }
  if (option.collisionFilter
      && option.collisionFilter->ignoresCollision(obj1, obj2)) {
    return false;
  }

  return NarrowPhase::collide(obj1, obj2, option, result);
}

bool CollisionWorld::raycast(
    const Ray& ray, const RaycastOption& option, RaycastResult& result)
{
  result.clear();
  prepareQueryObjectCache();

  RaycastResult closestResult;
  double closestDistance = std::numeric_limits<double>::max();

  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    CollisionObject obj(entity, this);
    RaycastResult tempResult;
    if (NarrowPhase::raycast(ray, obj, option, tempResult)) {
      tempResult.object = cacheQueryObject(obj);
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
  prepareQueryObjectCache();

  auto view = m_registry.view<comps::CollisionObjectTag>();
  for (auto entity : view) {
    CollisionObject obj(entity, this);
    RaycastResult tempResult;
    if (NarrowPhase::raycast(ray, obj, option, tempResult)) {
      tempResult.object = cacheQueryObject(obj);
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
  prepareQueryObjectCache();

  CcdResult closestResult;
  double closestToi = std::numeric_limits<double>::max();

  auto candidateIds
      = m_broadPhase->queryOverlapping(sweptSphereAabb(start, end, radius));
  std::sort(candidateIds.begin(), candidateIds.end());
  for (const auto id : candidateIds) {
    CollisionObject obj = getObjectById(id);
    if (!obj.isValid()) {
      continue;
    }

    CcdResult tempResult;
    if (NarrowPhase::sphereCast(start, end, radius, obj, option, tempResult)) {
      tempResult.object = cacheQueryObject(obj);
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
  prepareQueryObjectCache();

  auto candidateIds
      = m_broadPhase->queryOverlapping(sweptSphereAabb(start, end, radius));
  std::sort(candidateIds.begin(), candidateIds.end());
  for (const auto id : candidateIds) {
    CollisionObject obj = getObjectById(id);
    if (!obj.isValid()) {
      continue;
    }

    CcdResult tempResult;
    if (NarrowPhase::sphereCast(start, end, radius, obj, option, tempResult)) {
      tempResult.object = cacheQueryObject(obj);
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
  prepareQueryObjectCache();

  CcdResult closestResult;
  double closestToi = std::numeric_limits<double>::max();

  auto candidateIds = m_broadPhase->queryOverlapping(
      sweptCapsuleAabb(capsuleStart, capsuleEnd, capsule));
  std::sort(candidateIds.begin(), candidateIds.end());
  for (const auto id : candidateIds) {
    CollisionObject obj = getObjectById(id);
    if (!obj.isValid()) {
      continue;
    }

    CcdResult tempResult;
    if (NarrowPhase::capsuleCast(
            capsuleStart, capsuleEnd, capsule, obj, option, tempResult)) {
      tempResult.object = cacheQueryObject(obj);
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
  prepareQueryObjectCache();

  auto candidateIds = m_broadPhase->queryOverlapping(
      sweptCapsuleAabb(capsuleStart, capsuleEnd, capsule));
  std::sort(candidateIds.begin(), candidateIds.end());
  for (const auto id : candidateIds) {
    CollisionObject obj = getObjectById(id);
    if (!obj.isValid()) {
      continue;
    }

    CcdResult tempResult;
    if (NarrowPhase::capsuleCast(
            capsuleStart, capsuleEnd, capsule, obj, option, tempResult)) {
      tempResult.object = cacheQueryObject(obj);
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

} // namespace dart::collision::native
