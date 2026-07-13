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

#include "dart/collision/native/NativeCollisionDetector.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/DistanceFilter.hpp"
#include "dart/collision/dart/DARTCollide.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/native/NativeCollisionGroup.hpp"
#include "dart/collision/native/NativeCollisionObject.hpp"
#include "dart/collision/native/PersistentManifoldCache.hpp"
#include "dart/collision/native/narrow_phase/NarrowPhase.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Profile.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#include <cstdint>

namespace dart {
namespace collision {

static_assert(
    sizeof(NativeCollisionDetector) == sizeof(CollisionDetector),
    "NativeCollisionDetector must not add state to its exported ABI layout");
static_assert(
    alignof(NativeCollisionDetector) == alignof(CollisionDetector),
    "NativeCollisionDetector must preserve its exported ABI alignment");

namespace {

//==============================================================================
// Three non-collinear contacts define a stable planar patch while avoiding a
// fourth redundant solver row on contact-rich native scenes.
constexpr std::size_t kSolverFacingManifoldContactTarget = 3u;

class ScratchCollisionResult final : public CollisionResult
{
public:
  ScratchCollisionResult()
  {
    setCollidingObjectCacheEnabled(false);
  }
};

//==============================================================================
bool checkGroupValidity(
    const NativeCollisionDetector* detector, CollisionGroup* group)
{
  if (!group) {
    dterr << "[NativeCollisionDetector::collide] Attempting to check collision "
          << "with a nullptr collision group.\n";
    return false;
  }

  if (detector != group->getCollisionDetector().get()) {
    dterr << "[NativeCollisionDetector::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

//==============================================================================
native::CollisionOption makeNativeOption(
    const CollisionOption& option, const CollisionResult* result)
{
  native::CollisionOption nativeOption;
  nativeOption.enableContact = option.enableContact && result != nullptr;
  if (nativeOption.enableContact) {
    nativeOption.maxNumContacts = std::min(
        option.getEffectiveMaxNumContactsPerPair(),
        kSolverFacingManifoldContactTarget);
  } else {
    nativeOption.maxNumContacts = 1u;
  }
  nativeOption.collisionFilter = nullptr;

  return nativeOption;
}

//==============================================================================
bool shouldUseDartFallback(
    const NativeCollisionObject* object1, const NativeCollisionObject* object2)
{
  // DART's non-spherical ellipsoid fallback is implemented through the
  // soft-mesh kernels. Keep unsupported ellipsoid/primitive pairs off the hot
  // fallback bridge and let native-owned kernels handle primitive coverage.
  return (object1 != nullptr && object1->usesSoftMeshFallbackShape())
         || (object2 != nullptr && object2->usesSoftMeshFallbackShape());
}

//==============================================================================
bool hasSupportedCollisionPath(
    const NativeCollisionObject* object1, const NativeCollisionObject* object2)
{
  if (object1 == nullptr || object2 == nullptr)
    return false;

  if (shouldUseDartFallback(object1, object2))
    return true;

  return object1->getNativeShape() != nullptr
         && object2->getNativeShape() != nullptr;
}

//==============================================================================
bool shouldSkipPair(
    const NativeCollisionObject* object1,
    const NativeCollisionObject* object2,
    const CollisionOption& option)
{
  if (!hasSupportedCollisionPath(object1, object2))
    return true;

  if (option.collisionFilter
      && option.collisionFilter->ignoresCollision(object1, object2)) {
    return true;
  }

  return false;
}

//==============================================================================
bool shouldUseNativeManifoldContact(
    const CollisionObject* object1, const CollisionObject* object2)
{
  const auto* nativeObject1
      = static_cast<const NativeCollisionObject*>(object1);
  const auto* nativeObject2
      = static_cast<const NativeCollisionObject*>(object2);

  return nativeObject1->getNativeShape() != nullptr
         && nativeObject2->getNativeShape() != nullptr;
}

//==============================================================================
bool ownsNativeManifoldShape(const NativeCollisionObject* object)
{
  return object != nullptr && object->getNativeShape() != nullptr
         && !object->usesDartFallbackShape();
}

//==============================================================================
bool mayUseNativeManifoldContacts(
    const std::unordered_map<std::size_t, NativeCollisionObject*>& objects)
{
  std::size_t nativeShapeCount = 0u;
  for (const auto& entry : objects) {
    if (!ownsNativeManifoldShape(entry.second))
      continue;

    ++nativeShapeCount;
    if (nativeShapeCount >= 2u)
      return true;
  }

  return false;
}

bool mayUseNativeManifoldContacts(
    const std::unordered_map<std::size_t, NativeCollisionObject*>& objects1,
    const std::unordered_map<std::size_t, NativeCollisionObject*>& objects2)
{
  bool hasNativeShape1 = false;
  for (const auto& entry : objects1) {
    if (ownsNativeManifoldShape(entry.second)) {
      hasNativeShape1 = true;
      break;
    }
  }

  if (!hasNativeShape1)
    return false;

  for (const auto& entry : objects2) {
    if (ownsNativeManifoldShape(entry.second))
      return true;
  }

  return false;
}

//==============================================================================
bool shouldSkipDistancePair(
    const NativeCollisionObject* object1,
    const NativeCollisionObject* object2,
    const DistanceOption& option)
{
  if (!object1->getNativeShape() || !object2->getNativeShape())
    return true;

  if (option.distanceFilter
      && !option.distanceFilter->needDistance(object1, object2)) {
    return true;
  }

  return false;
}

//==============================================================================
native::DistanceOption makeNativeDistanceOption(
    const DistanceOption& option, double upperBound)
{
  native::DistanceOption nativeOption;
  nativeOption.upperBound = upperBound;
  nativeOption.enableNearestPoints = option.enableNearestPoints;

  return nativeOption;
}

//==============================================================================
struct NativeDistanceCandidate
{
  double distance = std::numeric_limits<double>::max();
  native::DistanceResult result;
  const NativeCollisionObject* object1 = nullptr;
  const NativeCollisionObject* object2 = nullptr;
  bool found = false;
};

//==============================================================================
struct NativeRayHitCandidate
{
  RayHit hit;
  double distance = std::numeric_limits<double>::max();
};

//==============================================================================
std::size_t getManifoldCacheId(const CollisionObject* object)
{
  return static_cast<std::size_t>(reinterpret_cast<std::uintptr_t>(object));
}

//==============================================================================
struct NativeDetectorState
{
  std::size_t numCollisionThreads{1u};
  std::unique_ptr<native::PersistentManifoldCache> manifoldCache;
};

using NativeDetectorStateMap
    = std::unordered_map<const NativeCollisionDetector*, NativeDetectorState>;

//==============================================================================
struct NativeDetectorStateRegistry
{
  NativeDetectorStateMap states;
  std::mutex mutex;
};

//==============================================================================
NativeDetectorStateRegistry& getNativeDetectorStateRegistry()
{
  // Detectors can be owned by static/global objects and therefore destroyed
  // after ordinary function-local objects. Keep this registry alive through
  // process teardown so every detector destructor can safely erase its state.
  static auto* const registry = new NativeDetectorStateRegistry;
  return *registry;
}

//==============================================================================
native::PersistentManifoldCache* findManifoldCache(
    const NativeCollisionDetector* detector)
{
  auto& registry = getNativeDetectorStateRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto it = registry.states.find(detector);
  if (it == registry.states.end())
    return nullptr;

  return it->second.manifoldCache.get();
}

//==============================================================================
native::PersistentManifoldCache& getOrCreateManifoldCache(
    const NativeCollisionDetector* detector)
{
  auto& registry = getNativeDetectorStateRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  auto& cache = registry.states[detector].manifoldCache;
  if (!cache)
    cache = std::make_unique<native::PersistentManifoldCache>();

  return *cache;
}

//==============================================================================
void setStoredNumCollisionThreads(
    const NativeCollisionDetector* detector, std::size_t numThreads)
{
  auto& registry = getNativeDetectorStateRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.states[detector].numCollisionThreads = numThreads;
}

//==============================================================================
std::size_t getStoredNumCollisionThreads(
    const NativeCollisionDetector* detector)
{
  auto& registry = getNativeDetectorStateRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto it = registry.states.find(detector);
  if (it == registry.states.end())
    return 1u;

  return it->second.numCollisionThreads;
}

//==============================================================================
void removeNativeDetectorState(const NativeCollisionDetector* detector)
{
  auto& registry = getNativeDetectorStateRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.states.erase(detector);
}

//==============================================================================
RayHit convertRayHit(
    const native::RaycastResult& nativeResult,
    const NativeCollisionObject* object,
    double rayLength)
{
  RayHit hit;
  hit.mCollisionObject = object;
  hit.mPoint = nativeResult.point;
  hit.mNormal = nativeResult.normal;
  hit.mFraction = nativeResult.distance / rayLength;
  return hit;
}

//==============================================================================
bool processNativeRaycastObject(
    const NativeCollisionObject* object,
    const native::Ray& ray,
    double rayLength,
    const RaycastOption& option,
    std::vector<NativeRayHitCandidate>& hits)
{
  const native::Shape* shape = object->getNativeShape();
  if (!shape)
    return false;

  if (!native::NarrowPhase::isRaycastSupported(shape->getType()))
    return false;

  native::RaycastResult nativeResult;
  native::RaycastOption nativeOption
      = native::RaycastOption::withMaxDistance(rayLength);
  nativeOption.backfaceCulling = false;
  const bool hit = native::NarrowPhase::raycast(
      ray, shape, object->getNativeTransform(), nativeOption, nativeResult);
  if (!hit || !option.passesFilter(object))
    return false;

  hits.push_back(NativeRayHitCandidate{
      convertRayHit(nativeResult, object, rayLength), nativeResult.distance});
  return true;
}

//==============================================================================
void convertRaycastResults(
    const std::vector<NativeRayHitCandidate>& hits,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  if (hits.empty())
    return;

  if (!option.mEnableAllHits) {
    const auto closest = std::min_element(
        hits.begin(),
        hits.end(),
        [](const NativeRayHitCandidate& a, const NativeRayHitCandidate& b) {
          return a.distance < b.distance;
        });
    result.mRayHits.emplace_back(closest->hit);
    return;
  }

  result.mRayHits.reserve(hits.size());
  for (const auto& hit : hits)
    result.mRayHits.emplace_back(hit.hit);

  if (option.mSortByClosest) {
    std::sort(
        result.mRayHits.begin(),
        result.mRayHits.end(),
        [](const RayHit& a, const RayHit& b) {
          return a.mFraction < b.mFraction;
        });
  }
}

//==============================================================================
bool processNativeDistancePair(
    const NativeCollisionObject* object1,
    const NativeCollisionObject* object2,
    const DistanceOption& option,
    NativeDistanceCandidate& best)
{
  if (shouldSkipDistancePair(object1, object2, option))
    return false;

  native::DistanceResult nativeResult;
  const native::DistanceOption nativeOption = makeNativeDistanceOption(
      option, best.found ? best.distance : std::numeric_limits<double>::max());

  const double distance = native::NarrowPhase::distance(
      object1->getNativeShape(),
      object1->getNativeTransform(),
      object2->getNativeShape(),
      object2->getNativeTransform(),
      nativeOption,
      nativeResult);

  if (!nativeResult.isValid())
    return false;

  if (!best.found || distance < best.distance) {
    best.distance = distance;
    best.result = nativeResult;
    best.object1 = object1;
    best.object2 = object2;
    best.found = true;
  }

  return best.distance <= option.distanceLowerBound;
}

//==============================================================================
double convertDistanceResult(
    const NativeDistanceCandidate& best,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (!best.found)
    return std::max(0.0, option.distanceLowerBound);

  const double minDistance = std::max(best.distance, option.distanceLowerBound);

  if (result) {
    result->minDistance = minDistance;
    result->unclampedMinDistance = best.distance;
    result->shapeFrame1 = best.object1->getShapeFrame();
    result->shapeFrame2 = best.object2->getShapeFrame();

    if (option.enableNearestPoints) {
      result->nearestPoint1 = best.result.pointOnObject1;
      result->nearestPoint2 = best.result.pointOnObject2;
    }
  }

  return minDistance;
}

//==============================================================================
void addPairOnlyContact(
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    CollisionResult& result)
{
  Contact contact;
  contact.collisionObject1 = object1;
  contact.collisionObject2 = object2;
  result.addContact(contact);
}

//==============================================================================
void attachCachedContactImpulses(
    CollisionResult* result, native::PersistentManifoldCache* manifoldCache)
{
  if (!result || !manifoldCache)
    return;

  if (result->getNumContacts() == 1u) {
    auto& contact = result->getContact(0);
    auto* object1 = contact.collisionObject1;
    auto* object2 = contact.collisionObject2;
    if (!shouldUseNativeManifoldContact(object1, object2))
      return;

    const auto id1 = getManifoldCacheId(object1);
    const auto id2 = getManifoldCacheId(object2);
    if (id1 == 0u || id2 == 0u)
      return;

    const auto tf1 = object1->getTransform();
    const auto tf2 = object2->getTransform();
    const auto tf1Inv = tf1.inverse();
    const auto tf2Inv = tf2.inverse();
    const bool swapped = id2 < id1;

    native::CachedContact cached;
    cached.localPointA
        = swapped ? tf2Inv * contact.point : tf1Inv * contact.point;
    cached.localPointB
        = swapped ? tf1Inv * contact.point : tf2Inv * contact.point;
    cached.normal = contact.normal;
    cached.penetrationDepth = contact.penetrationDepth;

    auto& manifold = manifoldCache->getOrCreate(id1, id2);
    manifold.addOrReplace(cached);
    if (manifold.numContacts == 1) {
      auto& manifoldContact = manifold.contacts[0];
      contact.userData = &manifoldContact;
      return;
    }

    manifold.refresh(swapped ? tf2 : tf1, swapped ? tf1 : tf2);
    if (manifold.numContacts == 0) {
      manifoldCache->remove(id1, id2);
      return;
    }

    const auto match = manifold.findMatch(cached.localPointA);
    if (match < 0)
      return;

    const auto matchIndex = static_cast<std::size_t>(match);
    auto& manifoldContact = manifold.contacts[matchIndex];
    contact.userData = &manifoldContact;
    return;
  }

  CollisionObject* cachedObject1 = nullptr;
  CollisionObject* cachedObject2 = nullptr;
  Eigen::Isometry3d tf1;
  Eigen::Isometry3d tf2;
  Eigen::Isometry3d tf1Inv;
  Eigen::Isometry3d tf2Inv;
  std::size_t id1 = 0u;
  std::size_t id2 = 0u;
  bool swapped = false;
  native::PersistentManifold* manifold = nullptr;
  std::vector<std::size_t> pendingContactIndices;
  std::vector<Eigen::Vector3d> pendingLocalPoints;
  pendingContactIndices.reserve(result->getNumContacts());
  pendingLocalPoints.reserve(result->getNumContacts());

  auto flushPair = [&]() {
    if (!manifold || pendingContactIndices.empty())
      return;

    manifold->refresh(swapped ? tf2 : tf1, swapped ? tf1 : tf2);
    if (manifold->numContacts == 0) {
      manifoldCache->remove(id1, id2);
      manifold = nullptr;
      pendingContactIndices.clear();
      pendingLocalPoints.clear();
      return;
    }

    std::array<bool, native::PersistentManifold::kMaxContacts> assignedSlots{};
    for (std::size_t pendingIndex = 0u;
         pendingIndex < pendingContactIndices.size();
         ++pendingIndex) {
      const auto match = manifold->findMatch(pendingLocalPoints[pendingIndex]);
      if (match < 0)
        continue;

      const auto matchIndex = static_cast<std::size_t>(match);
      if (assignedSlots[matchIndex])
        continue;
      assignedSlots[matchIndex] = true;

      auto& contact = result->getContact(pendingContactIndices[pendingIndex]);
      auto& manifoldContact = manifold->contacts[matchIndex];
      contact.userData = &manifoldContact;
    }

    manifold = nullptr;
    pendingContactIndices.clear();
    pendingLocalPoints.clear();
  };

  for (std::size_t i = 0u; i < result->getNumContacts(); ++i) {
    auto& contact = result->getContact(i);
    auto* object1 = contact.collisionObject1;
    auto* object2 = contact.collisionObject2;
    if (!shouldUseNativeManifoldContact(object1, object2))
      continue;

    if (object1 != cachedObject1 || object2 != cachedObject2) {
      flushPair();
      cachedObject1 = object1;
      cachedObject2 = object2;
      tf1 = object1->getTransform();
      tf2 = object2->getTransform();
      tf1Inv = tf1.inverse();
      tf2Inv = tf2.inverse();
      id1 = getManifoldCacheId(object1);
      id2 = getManifoldCacheId(object2);
      swapped = id2 < id1;
    }

    if (id1 == 0u || id2 == 0u) {
      flushPair();
      cachedObject1 = nullptr;
      cachedObject2 = nullptr;
      continue;
    }

    if (!manifold)
      manifold = &manifoldCache->getOrCreate(id1, id2);

    native::CachedContact cached;
    cached.localPointA
        = swapped ? tf2Inv * contact.point : tf1Inv * contact.point;
    cached.localPointB
        = swapped ? tf1Inv * contact.point : tf2Inv * contact.point;
    cached.normal = contact.normal;
    cached.penetrationDepth = contact.penetrationDepth;

    manifold->addOrReplace(cached);
    pendingContactIndices.push_back(i);
    pendingLocalPoints.push_back(cached.localPointA);
  }

  flushPair();
}

//==============================================================================
void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects,
    native::PersistentManifoldCache* manifoldCache)
{
  if (!manifoldCache || manifoldCache->size() == 0u)
    return;

  std::unordered_map<std::size_t, CollisionObject*> objectsById;
  objectsById.reserve(objects.size());
  for (auto* object : objects) {
    const auto id = getManifoldCacheId(object);
    if (id != 0u)
      objectsById[id] = object;
  }

  manifoldCache->refreshAll(
      [&](std::size_t idA, std::size_t idB)
          -> std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> {
        const auto itA = objectsById.find(idA);
        const auto itB = objectsById.find(idB);
        if (itA == objectsById.end() || itB == objectsById.end())
          return std::nullopt;

        return std::make_pair(
            itA->second->getTransform(), itB->second->getTransform());
      });
}

//==============================================================================
void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects1,
    const std::vector<CollisionObject*>& objects2,
    native::PersistentManifoldCache* manifoldCache)
{
  if (!manifoldCache || manifoldCache->size() == 0u)
    return;

  std::unordered_map<std::size_t, CollisionObject*> objectsById;
  objectsById.reserve(objects1.size() + objects2.size());
  for (auto* object : objects1) {
    const auto id = getManifoldCacheId(object);
    if (id != 0u)
      objectsById[id] = object;
  }
  for (auto* object : objects2) {
    const auto id = getManifoldCacheId(object);
    if (id != 0u)
      objectsById[id] = object;
  }

  manifoldCache->refreshAll(
      [&](std::size_t idA, std::size_t idB)
          -> std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> {
        const auto itA = objectsById.find(idA);
        const auto itB = objectsById.find(idB);
        if (itA == objectsById.end() || itB == objectsById.end())
          return std::nullopt;

        return std::make_pair(
            itA->second->getTransform(), itB->second->getTransform());
      });
}

//==============================================================================
bool emitContacts(
    const native::CollisionResult& nativeResult,
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult& result)
{
  const std::size_t maxPairContacts
      = option.getEffectiveMaxNumContactsPerPair();
  std::size_t emittedForPair = 0u;

  const std::size_t numContacts = nativeResult.numContacts();
  for (std::size_t i = 0u; i < numContacts; ++i) {
    if (result.getNumContacts() >= option.maxNumContacts)
      return true;

    if (emittedForPair >= maxPairContacts)
      return false;

    const auto& nativeContact = nativeResult.getContact(i);
    if (nativeContact.depth < 0.0
        && !option.allowNegativePenetrationDepthContacts) {
      continue;
    }

    if (Contact::isZeroNormal(nativeContact.normal))
      continue;

    Contact contact;
    contact.point = nativeContact.position;
    contact.normal = nativeContact.normal;
    contact.penetrationDepth = nativeContact.depth;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.triID1 = nativeContact.featureIndex1;
    contact.triID2 = nativeContact.featureIndex2;
    result.addContact(contact);
    ++emittedForPair;
  }

  return result.getNumContacts() >= option.maxNumContacts;
}

//==============================================================================
bool emitDartFallbackContacts(
    const CollisionResult& pairResult,
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult& result)
{
  DART_PROFILE_SCOPED_IF_N(
      dart::common::profile::isProfileRecordingEnabled(),
      "Native fallback emit");
  const std::size_t maxPairContacts
      = option.getEffectiveMaxNumContactsPerPair();
  std::size_t emittedForPair = 0u;

  const std::size_t numContacts = pairResult.getNumContacts();
  for (std::size_t i = 0u; i < numContacts; ++i) {
    if (result.getNumContacts() >= option.maxNumContacts)
      return true;

    if (emittedForPair >= maxPairContacts)
      return false;

    auto contact = pairResult.getContact(i);
    // DART fallback kernels currently report only penetrating contacts. Keep
    // the adapter check in case a future kernel violates that contract.
    // LCOV_EXCL_START: requires negative depth from a fallback kernel.
    if (contact.penetrationDepth < 0.0
        && !option.allowNegativePenetrationDepthContacts) {
      continue;
    }
    // LCOV_EXCL_STOP

    if (Contact::isZeroNormal(contact.normal))
      continue;

    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    result.addContact(contact);
    ++emittedForPair;
  }

  return result.getNumContacts() >= option.maxNumContacts;
}

//==============================================================================
bool processDartFallbackPair(
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    ScratchCollisionResult& pairResult)
{
  DART_PROFILE_SCOPED_IF_N(
      dart::common::profile::isProfileRecordingEnabled(),
      "Native soft fallback pair");
  pairResult.clear();
  auto* dartFallback1 = object1->getDartFallbackObject();
  auto* dartFallback2 = object2->getDartFallbackObject();
  const bool object1Plane = object1->isPlaneShape();
  const bool object2Plane = object2->isPlaneShape();
  bool hit = false;
  if (object1Plane && !object2Plane) {
    hit = collidePlaneShape(
              dartFallback1,
              dartFallback2,
              dartFallback1->getWorldTransformForCollision(),
              dartFallback2->getWorldTransformForCollision(),
              true,
              pairResult)
          != 0;
  } else if (!object1Plane && object2Plane) {
    hit = collidePlaneShape(
              dartFallback2,
              dartFallback1,
              dartFallback2->getWorldTransformForCollision(),
              dartFallback1->getWorldTransformForCollision(),
              false,
              pairResult)
          != 0;
  } else {
    hit = collide(dartFallback1, dartFallback2, pairResult) != 0;
  }
  if (!hit)
    return false;

  collisionFound = true;

  if (!result)
    return true;

  if (!option.enableContact) {
    addPairOnlyContact(object1, object2, *result);
    return result->getNumContacts() >= option.maxNumContacts;
  }

  return emitDartFallbackContacts(
      pairResult, object1, object2, option, *result);
}

//==============================================================================
bool processNativePair(
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    bool& nativeManifoldContactFound,
    ScratchCollisionResult& fallbackPairResult)
{
  if (shouldSkipPair(object1, object2, option))
    return false;

  if (shouldUseDartFallback(object1, object2)) {
    return processDartFallbackPair(
        object1, object2, option, result, collisionFound, fallbackPairResult);
  }

  if (result && result->getNumContacts() >= option.maxNumContacts)
    return true;

  native::CollisionResult nativeResult;
  const native::CollisionOption nativeOption = makeNativeOption(option, result);
  const bool hit = native::NarrowPhase::collide(
      object1->getNativeShape(),
      object1->getNativeTransform(),
      object2->getNativeShape(),
      object2->getNativeTransform(),
      nativeOption,
      nativeResult);

  if (!hit)
    return false;

  collisionFound = true;
  if (option.enableContact && result)
    nativeManifoldContactFound = true;

  if (!result)
    return true;

  if (!option.enableContact) {
    addPairOnlyContact(object1, object2, *result);
    return result->getNumContacts() >= option.maxNumContacts;
  }

  return emitContacts(nativeResult, object1, object2, option, *result);
}

} // namespace

//==============================================================================
NativeCollisionDetector::Registrar<NativeCollisionDetector>
    NativeCollisionDetector::mRegistrar{
        NativeCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<NativeCollisionDetector> {
          return NativeCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<NativeCollisionDetector> NativeCollisionDetector::create()
{
  return std::shared_ptr<NativeCollisionDetector>(
      new NativeCollisionDetector());
}

//==============================================================================
NativeCollisionDetector::~NativeCollisionDetector()
{
  removeNativeDetectorState(this);
}

//==============================================================================
native::CachedContact* NativeCollisionDetector::getCachedContact(
    const NativeCollisionObject* object1,
    const NativeCollisionObject* object2,
    void* userData) const
{
  if (!object1 || !object2 || !userData)
    return nullptr;

  if (object1->getCollisionDetector() != this
      || object2->getCollisionDetector() != this) {
    return nullptr;
  }

  const auto* manifoldCache = findManifoldCache(this);
  if (!manifoldCache)
    return nullptr;

  const auto id1 = getManifoldCacheId(object1);
  const auto id2 = getManifoldCacheId(object2);
  if (!manifoldCache->ownsContact(id1, id2, userData))
    return nullptr;

  return static_cast<native::CachedContact*>(userData);
}

//==============================================================================
void NativeCollisionDetector::notifyCollisionObjectDestroying(
    CollisionObject* object)
{
  CollisionDetector::notifyCollisionObjectDestroying(object);

  if (!object)
    return;

  auto* manifoldCache = findManifoldCache(this);
  if (!manifoldCache)
    return;

  manifoldCache->removeObject(getManifoldCacheId(object));
}

//==============================================================================
std::shared_ptr<CollisionDetector>
NativeCollisionDetector::cloneWithoutCollisionObjects() const
{
  auto clone = NativeCollisionDetector::create();
  clone->setNumCollisionThreads(getNumCollisionThreads());
  return clone;
}

//==============================================================================
const std::string& NativeCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& NativeCollisionDetector::getStaticType()
{
  static const std::string type = "native";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup> NativeCollisionDetector::createCollisionGroup()
{
  return std::make_unique<NativeCollisionGroup>(shared_from_this());
}

//==============================================================================
void NativeCollisionDetector::setNumCollisionThreads(std::size_t numThreads)
{
  if (numThreads == 0u) {
    numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0u)
      numThreads = 1u; // LCOV_EXCL_LINE: requires unavailable hardware count.
  }

  setStoredNumCollisionThreads(this, std::max<std::size_t>(1u, numThreads));
}

//==============================================================================
std::size_t NativeCollisionDetector::getNumCollisionThreads() const
{
  return getStoredNumCollisionThreads(this);
}

//==============================================================================
bool NativeCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  DART_PROFILE_SCOPED_IF_N(
      dart::common::profile::isProfileRecordingEnabled(),
      "NativeCollisionDetector::collide");
  if (result)
    result->clear();

  if (option.maxNumContacts == 0u)
    return false;

  if (!checkGroupValidity(this, group))
    return false;

  auto* nativeGroup = static_cast<NativeCollisionGroup*>(group);
  {
    DART_PROFILE_SCOPED_IF_N(
        dart::common::profile::isProfileRecordingEnabled(),
        "NativeCollisionGroup::updateObjects");
    nativeGroup->updateEngineData();
  }
  native::PersistentManifoldCache* manifoldCache = nullptr;
  const bool cacheNativeManifoldContacts
      = option.enableContact
        && mayUseNativeManifoldContacts(nativeGroup->mIdToObject);
  if (cacheNativeManifoldContacts) {
    manifoldCache = findManifoldCache(this);
    refreshManifoldCache(nativeGroup->mCollisionObjects, manifoldCache);
  }

  bool collisionFound = false;
  bool nativeManifoldContactFound = false;
  ScratchCollisionResult fallbackPairResult;
  const auto pairVisitor = [&](std::size_t id1, std::size_t id2) {
    auto* object1 = nativeGroup->mIdToObject.at(id1);
    auto* object2 = nativeGroup->mIdToObject.at(id2);
    return !processNativePair(
        object1,
        object2,
        option,
        result,
        collisionFound,
        nativeManifoldContactFound,
        fallbackPairResult);
  };
  if (result) {
    // Result-carrying queries need the sorted, deduplicated visitation
    // order: which contacts survive a maxNumContacts cap is part of the
    // observable behavior, and the ordered walk reproduces BruteForce
    // bit-for-bit.
    nativeGroup->mBroadPhase->visitPairs(pairVisitor);
  } else {
    // Boolean existence queries return no per-pair data, so traversal
    // order cannot leak; stream the tree walk and stop at the first hit.
    nativeGroup->mBroadPhase->visitPairsAnyOrder(pairVisitor);
  }

  if (cacheNativeManifoldContacts && nativeManifoldContactFound) {
    if (!manifoldCache) {
      manifoldCache = &getOrCreateManifoldCache(this);
    }
    attachCachedContactImpulses(result, manifoldCache);
  }

  return collisionFound;
}

//==============================================================================
bool NativeCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (option.maxNumContacts == 0u)
    return false;

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  if (group1 == group2)
    return collide(group1, option, result);

  auto* nativeGroup1 = static_cast<NativeCollisionGroup*>(group1);
  auto* nativeGroup2 = static_cast<NativeCollisionGroup*>(group2);
  nativeGroup1->updateEngineData();
  nativeGroup2->updateEngineData();
  native::PersistentManifoldCache* manifoldCache = nullptr;
  const bool cacheNativeManifoldContacts
      = option.enableContact
        && mayUseNativeManifoldContacts(
            nativeGroup1->mIdToObject, nativeGroup2->mIdToObject);
  if (cacheNativeManifoldContacts) {
    manifoldCache = findManifoldCache(this);
    refreshManifoldCache(
        nativeGroup1->mCollisionObjects,
        nativeGroup2->mCollisionObjects,
        manifoldCache);
  }

  bool collisionFound = false;
  bool nativeManifoldContactFound = false;
  ScratchCollisionResult fallbackPairResult;
  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      if (processNativePair(
              static_cast<NativeCollisionObject*>(object1),
              static_cast<NativeCollisionObject*>(object2),
              option,
              result,
              collisionFound,
              nativeManifoldContactFound,
              fallbackPairResult)) {
        if (cacheNativeManifoldContacts && nativeManifoldContactFound) {
          if (!manifoldCache) {
            manifoldCache = &getOrCreateManifoldCache(this);
          }
          attachCachedContactImpulses(result, manifoldCache);
        }
        return collisionFound;
      }
    }
  }

  if (cacheNativeManifoldContacts && nativeManifoldContactFound) {
    if (!manifoldCache) {
      manifoldCache = &getOrCreateManifoldCache(this);
    }
    attachCachedContactImpulses(result, manifoldCache);
  }

  return collisionFound;
}

//==============================================================================
double NativeCollisionDetector::distance(
    CollisionGroup* group, const DistanceOption& option, DistanceResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group))
    return 0.0;

  auto* nativeGroup = static_cast<NativeCollisionGroup*>(group);
  nativeGroup->updateEngineData();

  NativeDistanceCandidate best;
  for (std::size_t i = 0u; i < nativeGroup->mCollisionObjects.size(); ++i) {
    for (std::size_t j = i + 1u; j < nativeGroup->mCollisionObjects.size();
         ++j) {
      const auto* object1 = static_cast<const NativeCollisionObject*>(
          nativeGroup->mCollisionObjects[i]);
      const auto* object2 = static_cast<const NativeCollisionObject*>(
          nativeGroup->mCollisionObjects[j]);

      if (processNativeDistancePair(object1, object2, option, best))
        return convertDistanceResult(best, option, result);
    }
  }

  return convertDistanceResult(best, option, result);
}

//==============================================================================
double NativeCollisionDetector::distance(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group1))
    return 0.0;

  if (!checkGroupValidity(this, group2))
    return 0.0;

  if (group1 == group2)
    return distance(group1, option, result);

  auto* nativeGroup1 = static_cast<NativeCollisionGroup*>(group1);
  auto* nativeGroup2 = static_cast<NativeCollisionGroup*>(group2);
  nativeGroup1->updateEngineData();
  nativeGroup2->updateEngineData();

  NativeDistanceCandidate best;
  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      if (processNativeDistancePair(
              static_cast<const NativeCollisionObject*>(object1),
              static_cast<const NativeCollisionObject*>(object2),
              option,
              best)) {
        return convertDistanceResult(best, option, result);
      }
    }
  }

  return convertDistanceResult(best, option, result);
}

//==============================================================================
bool NativeCollisionDetector::raycast(
    CollisionGroup* group,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group))
    return false;

  const Eigen::Vector3d displacement = to - from;
  const double rayLength = displacement.norm();
  if (rayLength <= std::numeric_limits<double>::epsilon())
    return false;

  native::Ray ray(from, displacement, rayLength);

  auto* nativeGroup = static_cast<NativeCollisionGroup*>(group);
  nativeGroup->updateEngineData();

  std::vector<NativeRayHitCandidate> hits;
  for (auto* object : nativeGroup->mCollisionObjects) {
    const auto* nativeObject
        = static_cast<const NativeCollisionObject*>(object);
    if (processNativeRaycastObject(nativeObject, ray, rayLength, option, hits)
        && result == nullptr) {
      return true;
    }
  }

  if (hits.empty())
    return false;

  if (result)
    convertRaycastResults(hits, option, *result);

  return true;
}

//==============================================================================
NativeCollisionDetector::NativeCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
std::unique_ptr<CollisionObject> NativeCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<NativeCollisionObject>(
      new NativeCollisionObject(this, shapeFrame));
}

//==============================================================================
void NativeCollisionDetector::refreshCollisionObject(
    CollisionObject* /*object*/)
{
  // Do nothing. NativeCollisionObject refreshes lazily in updateEngineData().
}

} // namespace collision
} // namespace dart
