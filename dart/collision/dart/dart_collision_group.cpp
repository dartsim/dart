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

#include "dart/collision/dart/dart_collision_group.hpp"

#include "dart/collision/collision_filter.hpp"
#include "dart/collision/collision_object.hpp"
#include "dart/collision/contact.hpp"
#include "dart/collision/dart/dart_collision_object.hpp"
#include "dart/collision/dart/shape_adapter.hpp"
#include "dart/collision/distance_filter.hpp"
#include "dart/collision/native/collision_filter.hpp"
#include "dart/collision/native/collision_world.hpp"
#include "dart/collision/native/narrow_phase/narrow_phase.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/signal.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/shape_frame.hpp"

#include <algorithm>
#include <atomic>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <cmath>

namespace dart {
namespace collision {

namespace {

bool isSameTransform(const Eigen::Isometry3d& lhs, const Eigen::Isometry3d& rhs)
{
  return lhs.matrix().isApprox(rhs.matrix(), 1e-12);
}

unsigned int getGeometryVariance(const dynamics::Shape& shape)
{
  return shape.getDataVariance()
         & (dynamics::Shape::DYNAMIC_VERTICES
            | dynamics::Shape::DYNAMIC_ELEMENTS);
}

bool requiresDynamicGeometrySync(unsigned int geometryVariance)
{
  return geometryVariance != 0u;
}

class DartCollisionFilterAdapter final : public native::CollisionFilter
{
public:
  explicit DartCollisionFilterAdapter(
      const dart::collision::CollisionFilter* filter)
    : mFilter(filter)
  {
  }

  bool ignoresCollision(
      const native::CollisionObject& object1,
      const native::CollisionObject& object2) const override
  {
    if (!mFilter) {
      return false;
    }

    const auto* dartObject1
        = static_cast<const CollisionObject*>(object1.getUserData());
    const auto* dartObject2
        = static_cast<const CollisionObject*>(object2.getUserData());
    if (!dartObject1 || !dartObject2) {
      return false;
    }

    return mFilter->ignoresCollision(dartObject1, dartObject2);
  }

private:
  const dart::collision::CollisionFilter* mFilter;
};

bool addContacts(
    const CollisionOption& option,
    CollisionObject* object1,
    CollisionObject* object2,
    const native::CollisionResult& nativeResult,
    CollisionResult& result)
{
  if (!option.enableContact) {
    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    result.addContact(contact);
    return result.getNumContacts() >= option.maxNumContacts;
  }

  for (const auto& manifold : nativeResult.getManifolds()) {
    for (const auto& cp : manifold.getContacts()) {
      if (!option.allowNegativePenetrationDepthContacts && cp.depth < 0.0) {
        continue;
      }

      if (Contact::isZeroNormal(cp.normal)) {
        continue;
      }

      Contact contact;
      contact.point = cp.position;
      contact.normal = cp.normal;
      contact.penetrationDepth = cp.depth;
      contact.collisionObject1 = object1;
      contact.collisionObject2 = object2;
      if (cp.featureIndex1 >= 0) {
        contact.triID1 = cp.featureIndex1;
      }
      if (cp.featureIndex2 >= 0) {
        contact.triID2 = cp.featureIndex2;
      }
      result.addContact(contact);

      if (result.getNumContacts() >= option.maxNumContacts) {
        return true;
      }
    }
  }

  return false;
}

native::CollisionOption makeNativeCollisionOption(
    const CollisionOption& option, const native::CollisionFilter* filter)
{
  native::CollisionOption nativeOption;
  nativeOption.enableContact = option.enableContact;
  nativeOption.maxNumContacts = option.maxNumContacts;
  nativeOption.collisionFilter = filter;
  return nativeOption;
}

native::DistanceOption makeNativeDistanceOption(
    const DistanceOption& option, double upperBound)
{
  native::DistanceOption nativeOption;
  nativeOption.upperBound = upperBound;
  nativeOption.enableNearestPoints = option.enableNearestPoints;
  return nativeOption;
}

double aabbDistance(const native::Aabb& a, const native::Aabb& b)
{
  double squaredDistance = 0.0;
  for (auto i = 0; i < 3; ++i) {
    double delta = 0.0;
    if (a.max[i] < b.min[i]) {
      delta = b.min[i] - a.max[i];
    } else if (b.max[i] < a.min[i]) {
      delta = a.min[i] - b.max[i];
    }
    squaredDistance += delta * delta;
  }
  return std::sqrt(squaredDistance);
}

std::size_t allocateManifoldCacheId()
{
  static std::atomic_size_t nextId{1u};
  return nextId.fetch_add(1u, std::memory_order_relaxed);
}

} // namespace

class DartCollisionScene
{
public:
  ~DartCollisionScene()
  {
    clear();
  }

  void sync(const std::vector<CollisionObject*>& objects)
  {
    const bool objectListUnchanged = hasMatchingObjectList(objects);
    if (!objectListUnchanged) {
      std::unordered_set<CollisionObject*> activeObjects;
      activeObjects.reserve(objects.size());
      for (auto* object : objects) {
        if (object) {
          activeObjects.insert(object);
        }
      }

      for (auto it = mEntries.begin(); it != mEntries.end();) {
        if (!activeObjects.contains(it->first)) {
          it = removeEntry(it);
        } else {
          ++it;
        }
      }

      mWorld.reserveObjects(activeObjects.size());
      mObjectsInOrder.clear();
      mObjectsInOrder.reserve(objects.size());
    }

    std::vector<native::ObjectId> dirtyIds;
    dirtyIds.reserve(objects.size());

    for (auto* object : objects) {
      if (syncObject(object, dirtyIds)) {
        if (!objectListUnchanged) {
          mObjectsInOrder.push_back(object);
        }
      } else if (objectListUnchanged) {
        std::erase(mObjectsInOrder, object);
      }
    }

    if (!dirtyIds.empty()) {
      (void)mWorld.updateDirty(dirtyIds);
    }
  }

  void remove(CollisionObject* object)
  {
    const auto it = mEntries.find(object);
    if (it != mEntries.end()) {
      removeEntry(it);
    }
    std::erase(mObjectsInOrder, object);
  }

  void clear()
  {
    for (auto& [object, entry] : mEntries) {
      (void)object;
      entry.transformConnection.disconnect();
    }
    mEntries.clear();
    mObjectsByNativeId.clear();
    mObjectsInOrder.clear();
    mWorld.clear();
  }

  bool collideSelf(
      const CollisionOption& option,
      CollisionResult* result,
      native::PersistentManifoldCache* /*manifoldCache*/)
  {
    if (!checkMaxContacts(option)) {
      return false;
    }

    if (mEntries.size() < 2u) {
      return false;
    }

    const DartCollisionFilterAdapter filterAdapter(
        option.collisionFilter.get());
    const auto nativeFilter = option.collisionFilter ? &filterAdapter : nullptr;
    const auto nativeOption = makeNativeCollisionOption(option, nativeFilter);

    if (mObjectsInOrder.size() == 2u) {
      const auto* entry1 = findEntry(mObjectsInOrder[0]);
      const auto* entry2 = findEntry(mObjectsInOrder[1]);
      if (entry1 && entry2) {
        auto& pairResult = nextPairResult();
        if (!collideNativePair(*entry1, *entry2, nativeOption, pairResult)) {
          return false;
        }

        if (result) {
          (void)addContacts(
              option, entry1->object, entry2->object, pairResult, *result);
        }
        return true;
      }
    }

    const auto snapshot = mWorld.buildBroadPhaseSnapshot();
    bool collisionFound = false;

    for (const auto& pair : snapshot.pairs) {
      const auto* entry1 = findEntry(pair.first);
      const auto* entry2 = findEntry(pair.second);
      if (!entry1 || !entry2) {
        continue;
      }

      auto& pairResult = nextPairResult();
      if (!collideNativePair(*entry1, *entry2, nativeOption, pairResult)) {
        continue;
      }

      collisionFound = true;
      if (!result) {
        return true;
      }

      if (addContacts(
              option, entry1->object, entry2->object, pairResult, *result)) {
        return true;
      }
    }

    return collisionFound;
  }

  bool collideAgainst(
      DartCollisionScene& other,
      const CollisionOption& option,
      CollisionResult* result,
      native::PersistentManifoldCache* manifoldCache)
  {
    if (&other == this) {
      return collideSelf(option, result, manifoldCache);
    }

    if (!checkMaxContacts(option)) {
      return false;
    }

    if (mEntries.empty() || other.mEntries.empty()) {
      return false;
    }

    const DartCollisionFilterAdapter filterAdapter(
        option.collisionFilter.get());
    const auto nativeFilter = option.collisionFilter ? &filterAdapter : nullptr;
    const auto nativeOption = makeNativeCollisionOption(option, nativeFilter);
    bool collisionFound = false;

    for (auto* object1 : mObjectsInOrder) {
      const auto* entry1 = findEntry(object1);
      if (!entry1) {
        continue;
      }

      auto candidates = other.mWorld.getBroadPhase().queryOverlapping(
          entry1->nativeObject.computeAabb());
      std::sort(candidates.begin(), candidates.end());

      for (const auto id2 : candidates) {
        const auto* entry2 = other.findEntry(id2);
        if (!entry2) {
          continue;
        }

        auto& pairResult = nextPairResult();
        if (!collideNativePair(*entry1, *entry2, nativeOption, pairResult)) {
          continue;
        }

        collisionFound = true;
        if (!result) {
          return true;
        }

        if (addContacts(
                option, entry1->object, entry2->object, pairResult, *result)) {
          return true;
        }
      }
    }

    return collisionFound;
  }

  double distanceSelf(const DistanceOption& option, DistanceResult* result)
  {
    return distanceImpl(*this, true, option, result);
  }

  double distanceAgainst(
      DartCollisionScene& other,
      const DistanceOption& option,
      DistanceResult* result)
  {
    if (&other == this) {
      return distanceSelf(option, result);
    }

    return distanceImpl(other, false, option, result);
  }

  bool raycast(
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option,
      RaycastResult* result)
  {
    const auto delta = to - from;
    const auto totalLength = delta.norm();
    if (totalLength <= 0.0) {
      return false;
    }

    const native::Ray ray(from, delta, totalLength);
    auto nativeOption = native::RaycastOption::unlimited();
    nativeOption.backfaceCulling = false;
    const native::Aabb rayAabb(from.cwiseMin(to), from.cwiseMax(to));
    const auto candidateIds = mWorld.getBroadPhase().queryOverlapping(rayAabb);
    if (candidateIds.empty()) {
      return false;
    }

    std::unordered_set<native::ObjectId> candidates;
    candidates.reserve(candidateIds.size());
    candidates.insert(candidateIds.begin(), candidateIds.end());

    bool hitFound = false;
    RayHit closestHit;
    auto closestFraction = std::numeric_limits<double>::max();
    std::vector<RayHit> hits;
    hits.reserve(mObjectsInOrder.size());

    for (auto* object : mObjectsInOrder) {
      if (!option.passesFilter(object)) {
        continue;
      }

      const auto* entry = findEntry(object);
      if (!entry) {
        continue;
      }
      if (!candidates.contains(entry->nativeObject.getId())) {
        continue;
      }

      native::RaycastResult pairResult;
      if (!native::NarrowPhase::raycast(
              ray, entry->nativeObject, nativeOption, pairResult)) {
        continue;
      }

      hitFound = true;

      RayHit hit;
      hit.mCollisionObject = object;
      hit.mPoint = pairResult.point;
      hit.mNormal = pairResult.normal;
      hit.mFraction = pairResult.distance / totalLength;

      if (option.mEnableAllHits) {
        hits.push_back(hit);
      } else if (hit.mFraction < closestFraction) {
        closestFraction = hit.mFraction;
        closestHit = hit;
      }
    }

    if (!hitFound) {
      return false;
    }

    if (!result) {
      return true;
    }

    if (option.mEnableAllHits) {
      if (option.mSortByClosest) {
        std::sort(
            hits.begin(), hits.end(), [](const RayHit& lhs, const RayHit& rhs) {
              return lhs.mFraction < rhs.mFraction;
            });
      }
      result->mRayHits = std::move(hits);
    } else {
      result->mRayHits.push_back(closestHit);
    }

    return result->hasHit();
  }

  std::size_t getManifoldCacheId(CollisionObject* object) const
  {
    const auto* entry = findEntry(object);
    return entry ? entry->manifoldCacheId : 0u;
  }

private:
  struct Entry
  {
    CollisionObject* object = nullptr;
    native::CollisionObject nativeObject;
    std::size_t manifoldCacheId = 0u;
    std::size_t shapeId = 0u;
    std::size_t shapeVersion = 0u;
    std::size_t shapeRevision = 0u;
    unsigned int geometryVariance = 0u;
    common::Connection transformConnection;
    bool dynamicGeometry = false;
    bool transformDirty = false;
  };

  using EntryMap = std::unordered_map<CollisionObject*, Entry>;

  bool hasMatchingObjectList(const std::vector<CollisionObject*>& objects) const
  {
    if (objects.size() != mObjectsInOrder.size()
        || objects.size() != mEntries.size()) {
      return false;
    }

    return std::equal(objects.begin(), objects.end(), mObjectsInOrder.begin());
  }

  bool syncObject(
      CollisionObject* object, std::vector<native::ObjectId>& dirtyIds)
  {
    if (!object) {
      remove(object);
      return false;
    }

    const auto* shapeFrame = object->getShapeFrame();
    if (!shapeFrame) {
      remove(object);
      return false;
    }

    const auto shape = object->getShape();
    if (!shape) {
      remove(object);
      return false;
    }

    auto* dartObject = static_cast<DartCollisionObject*>(object);
    const auto shapeId = shape->getID();
    const auto shapeVersion = shape->getVersion();
    const auto shapeRevision = dartObject->getNativeShapeRevision();
    const auto geometryVariance = getGeometryVariance(*shape);
    const bool dynamicGeometry = requiresDynamicGeometrySync(geometryVariance);

    auto it = mEntries.find(object);
    if (it != mEntries.end() && it->second.nativeObject.isValid()
        && !dynamicGeometry && !it->second.dynamicGeometry
        && it->second.shapeId == shapeId
        && it->second.shapeVersion == shapeVersion
        && it->second.shapeRevision == shapeRevision
        && it->second.geometryVariance == geometryVariance) {
      auto& entry = it->second;
      if (!entry.transformDirty) {
        return true;
      }

      auto& nativeObject = entry.nativeObject;
      const auto& transform = object->getTransform();
      if (!isSameTransform(nativeObject.getTransform(), transform)) {
        nativeObject.setTransform(transform);
        dirtyIds.push_back(nativeObject.getId());
      }
      entry.transformDirty = false;

      return true;
    }

    if (it == mEntries.end() || dynamicGeometry || it->second.dynamicGeometry
        || it->second.shapeId != shapeId
        || it->second.shapeVersion != shapeVersion
        || it->second.shapeRevision != shapeRevision
        || it->second.geometryVariance != geometryVariance
        || !it->second.nativeObject.isValid()) {
      if (it != mEntries.end()) {
        removeEntry(it);
      }

      auto nativeShape = adaptShape(shape);
      if (!nativeShape) {
        return false;
      }

      auto nativeObject
          = mWorld.createObject(std::move(nativeShape), object->getTransform());
      if (!nativeObject.isValid()) {
        return false;
      }

      nativeObject.setUserData(object);
      const auto nativeId = nativeObject.getId();
      mObjectsByNativeId[nativeId] = object;
      auto [entryIt, inserted] = mEntries.emplace(
          object,
          Entry{
              object,
              nativeObject,
              allocateManifoldCacheId(),
              shapeId,
              shapeVersion,
              shapeRevision,
              geometryVariance,
              common::Connection{},
              dynamicGeometry,
              false});
      (void)inserted;
      // Slot registration is observer state; ShapeFrame exposes it non-const.
      auto* transformNotifier = const_cast<dynamics::ShapeFrame*>(shapeFrame);
      entryIt->second.transformConnection
          = transformNotifier->onTransformUpdated.connect(
              [this, object](const dynamics::Entity*) {
                markObjectTransformDirty(object);
              });
      return true;
    }

    auto& entry = it->second;
    if (!entry.transformDirty) {
      return true;
    }

    auto& nativeObject = entry.nativeObject;
    const auto& transform = object->getTransform();
    if (!isSameTransform(nativeObject.getTransform(), transform)) {
      nativeObject.setTransform(transform);
      dirtyIds.push_back(nativeObject.getId());
    }
    entry.transformDirty = false;

    return true;
  }

  EntryMap::iterator removeEntry(EntryMap::iterator it)
  {
    it->second.transformConnection.disconnect();
    if (it->second.nativeObject.isValid()) {
      mObjectsByNativeId.erase(it->second.nativeObject.getId());
      mWorld.destroyObject(it->second.nativeObject);
    }
    return mEntries.erase(it);
  }

  void markObjectTransformDirty(CollisionObject* object)
  {
    const auto it = mEntries.find(object);
    if (it != mEntries.end()) {
      it->second.transformDirty = true;
    }
  }

  const Entry* findEntry(native::ObjectId id) const
  {
    const auto objectIt = mObjectsByNativeId.find(id);
    if (objectIt == mObjectsByNativeId.end()) {
      return nullptr;
    }
    return findEntry(objectIt->second);
  }

  const Entry* findEntry(CollisionObject* object) const
  {
    const auto entryIt = mEntries.find(object);
    if (entryIt == mEntries.end()) {
      return nullptr;
    }
    return &entryIt->second;
  }

  bool checkMaxContacts(const CollisionOption& option) const
  {
    if (0u == option.maxNumContacts) {
      DART_WARN(
          "CollisionOption::maxNumContacts is 0; skipping collision "
          "detection. Use maxNumContacts >= 1 for binary checks.");
      return false;
    }
    return true;
  }

  native::CollisionResult& nextPairResult()
  {
    mPairResultScratch.clear();
    return mPairResultScratch;
  }

  bool collideNativePair(
      const Entry& entry1,
      const Entry& entry2,
      const native::CollisionOption& option,
      native::CollisionResult& result) const
  {
    if (!native::shouldCollide(
            entry1.nativeObject.getCollisionFilterData(),
            entry2.nativeObject.getCollisionFilterData(),
            entry1.nativeObject,
            entry2.nativeObject,
            option.collisionFilter)) {
      return false;
    }

    return native::NarrowPhase::collide(
        entry1.nativeObject, entry2.nativeObject, option, result);
  }

  double distanceImpl(
      DartCollisionScene& other,
      bool selfQuery,
      const DistanceOption& option,
      DistanceResult* result)
  {
    if (mEntries.empty() || other.mEntries.empty()) {
      return 0.0;
    }

    double bestDistance = std::numeric_limits<double>::max();
    native::DistanceResult bestPairResult;
    CollisionObject* bestObject1 = nullptr;
    CollisionObject* bestObject2 = nullptr;
    bool found = false;

    auto shouldStop = [&]() {
      return found && bestDistance >= 0.0
             && bestDistance <= option.distanceLowerBound;
    };

    for (auto i = 0u; i < mObjectsInOrder.size(); ++i) {
      const auto* entry1 = findEntry(mObjectsInOrder[i]);
      if (!entry1) {
        continue;
      }
      const auto aabb1 = entry1->nativeObject.computeAabb();

      const auto jStart = selfQuery ? i + 1u : 0u;
      for (auto j = jStart; j < other.mObjectsInOrder.size(); ++j) {
        const auto* entry2 = other.findEntry(other.mObjectsInOrder[j]);
        if (!entry2) {
          continue;
        }

        if (option.distanceFilter
            && !option.distanceFilter->needDistance(
                entry1->object, entry2->object)) {
          continue;
        }

        if (found && bestDistance >= 0.0
            && aabbDistance(aabb1, entry2->nativeObject.computeAabb())
                   > bestDistance) {
          continue;
        }

        native::DistanceResult pairResult;
        const auto distance = native::NarrowPhase::distance(
            entry1->nativeObject,
            entry2->nativeObject,
            makeNativeDistanceOption(option, bestDistance),
            pairResult);

        if (distance < bestDistance) {
          bestDistance = distance;
          bestPairResult = pairResult;
          bestObject1 = entry1->object;
          bestObject2 = entry2->object;
          found = true;
        }

        if (shouldStop()) {
          break;
        }
      }

      if (shouldStop()) {
        break;
      }
    }

    if (!found) {
      return 0.0;
    }

    if (result) {
      result->unclampedMinDistance = bestDistance;
      result->minDistance = std::max(bestDistance, option.distanceLowerBound);
      result->shapeFrame1
          = bestObject1 ? bestObject1->getShapeFrame() : nullptr;
      result->shapeFrame2
          = bestObject2 ? bestObject2->getShapeFrame() : nullptr;
      result->nearestPoint1 = bestPairResult.pointOnObject1;
      result->nearestPoint2 = bestPairResult.pointOnObject2;
    }

    return std::max(bestDistance, option.distanceLowerBound);
  }

  native::CollisionWorld mWorld;
  EntryMap mEntries;
  std::unordered_map<native::ObjectId, CollisionObject*> mObjectsByNativeId;
  std::vector<CollisionObject*> mObjectsInOrder;
  native::CollisionResult mPairResultScratch;
};

//==============================================================================
DartCollisionGroup::DartCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mScene(std::make_unique<DartCollisionScene>())
{
  // Do nothing
}

//==============================================================================
DartCollisionGroup::~DartCollisionGroup() = default;

//==============================================================================
void DartCollisionGroup::initializeEngineData()
{
  mScene->clear();
}

//==============================================================================
void DartCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  if (std::ranges::find(mCollisionObjects, object) == mCollisionObjects.end()) {
    mCollisionObjects.push_back(object);
  }
}

//==============================================================================
void DartCollisionGroup::addCollisionObjectsToEngine(
    std::span<CollisionObject* const> collObjects)
{
  for (auto collObject : collObjects) {
    addCollisionObjectToEngine(collObject);
  }
}

//==============================================================================
void DartCollisionGroup::removeCollisionObjectFromEngine(
    CollisionObject* object)
{
  std::erase(mCollisionObjects, object);
  mScene->remove(object);
}

//==============================================================================
void DartCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  mCollisionObjects.clear();
  mScene->clear();
}

//==============================================================================
void DartCollisionGroup::updateCollisionGroupEngineData()
{
  mScene->sync(mCollisionObjects);
}

//==============================================================================
bool DartCollisionGroup::collideSelf(
    const CollisionOption& option,
    CollisionResult* result,
    native::PersistentManifoldCache* manifoldCache)
{
  mScene->sync(mCollisionObjects);
  return mScene->collideSelf(option, result, manifoldCache);
}

//==============================================================================
bool DartCollisionGroup::collideWith(
    DartCollisionGroup& other,
    const CollisionOption& option,
    CollisionResult* result,
    native::PersistentManifoldCache* manifoldCache)
{
  mScene->sync(mCollisionObjects);
  other.mScene->sync(other.mCollisionObjects);
  return mScene->collideAgainst(*other.mScene, option, result, manifoldCache);
}

//==============================================================================
double DartCollisionGroup::distanceSelf(
    const DistanceOption& option, DistanceResult* result)
{
  mScene->sync(mCollisionObjects);
  return mScene->distanceSelf(option, result);
}

//==============================================================================
double DartCollisionGroup::distanceWith(
    DartCollisionGroup& other,
    const DistanceOption& option,
    DistanceResult* result)
{
  mScene->sync(mCollisionObjects);
  other.mScene->sync(other.mCollisionObjects);
  return mScene->distanceAgainst(*other.mScene, option, result);
}

//==============================================================================
bool DartCollisionGroup::raycast(
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  mScene->sync(mCollisionObjects);
  return mScene->raycast(from, to, option, result);
}

//==============================================================================
std::size_t DartCollisionGroup::getManifoldCacheId(
    CollisionObject* object) const
{
  return mScene->getManifoldCacheId(object);
}

} // namespace collision
} // namespace dart
