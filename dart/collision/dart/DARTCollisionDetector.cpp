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

#include "dart/collision/dart/DARTCollisionDetector.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/DistanceFilter.hpp"
#include "dart/collision/dart/DARTCollisionGroup.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/dart/PersistentManifoldCache.hpp"
#include "dart/collision/dart/narrow_phase/NarrowPhase.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Profile.hpp"

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

namespace {

//==============================================================================
// Three non-collinear contacts define a stable planar patch while avoiding a
// fourth redundant solver row on contact-rich native scenes.
constexpr std::size_t kSolverFacingManifoldContactTarget = 3u;

//==============================================================================
bool checkGroupValidity(
    const DARTCollisionDetector* detector, CollisionGroup* group)
{
  if (!group) {
    dterr << "[DARTCollisionDetector::collide] Attempting to check collision "
          << "with a nullptr collision group.\n";
    return false;
  }

  if (detector != group->getCollisionDetector().get()) {
    dterr << "[DARTCollisionDetector::collide] Attempting to check collision "
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
bool shouldSkipPair(
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
    const CollisionOption& option)
{
  if (!object1->getNativeShape() || !object2->getNativeShape())
    return true;

  if (option.collisionFilter
      && option.collisionFilter->ignoresCollision(object1, object2)) {
    return true;
  }

  return false;
}

//==============================================================================
bool shouldSkipDistancePair(
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
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
  const DARTCollisionObject* object1 = nullptr;
  const DARTCollisionObject* object2 = nullptr;
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
// Per-detector engine state: the persistent manifold cache plus scratch
// buffers reused across collide() calls so steady-state stepping performs no
// heap allocations (StepAllocation gate discipline). Guarded by the registry
// mutex only for lookup; concurrent collide() on the same detector instance
// is not supported, matching the manifold cache's existing contract.
struct DetectorEngineState
{
  native::PersistentManifoldCache manifoldCache;
  native::CollisionResult narrowphaseScratch;
  std::vector<std::pair<std::size_t, CollisionObject*>> objectsByIdScratch;
  std::vector<std::size_t> pendingContactIndicesScratch;
  std::vector<Eigen::Vector3d> pendingLocalPointsScratch;
};

using ManifoldCacheMap = std::unordered_map<
    const DARTCollisionDetector*,
    std::unique_ptr<DetectorEngineState>>;

//==============================================================================
struct ManifoldCacheRegistry
{
  ManifoldCacheMap caches;
  std::mutex mutex;
};

//==============================================================================
ManifoldCacheRegistry& getManifoldCacheRegistry()
{
  static ManifoldCacheRegistry registry;
  return registry;
}

//==============================================================================
native::PersistentManifoldCache* findManifoldCache(
    const DARTCollisionDetector* detector)
{
  auto& registry = getManifoldCacheRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto it = registry.caches.find(detector);
  if (it == registry.caches.end())
    return nullptr;

  return &it->second->manifoldCache;
}

//==============================================================================
DetectorEngineState& getOrCreateEngineState(
    const DARTCollisionDetector* detector)
{
  auto& registry = getManifoldCacheRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  auto& state = registry.caches[detector];
  if (!state)
    state = std::make_unique<DetectorEngineState>();

  return *state;
}

//==============================================================================
void removeManifoldCache(const DARTCollisionDetector* detector)
{
  auto& registry = getManifoldCacheRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.caches.erase(detector);
}

//==============================================================================
RayHit convertRayHit(
    const native::RaycastResult& nativeResult,
    const DARTCollisionObject* object,
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
    const DARTCollisionObject* object,
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
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
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
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    CollisionResult& result)
{
  Contact contact;
  contact.collisionObject1 = object1;
  contact.collisionObject2 = object2;
  result.addContact(contact);
}

//==============================================================================
void attachCachedContactImpulses(
    CollisionResult* result, DetectorEngineState& engineState)
{
  auto* manifoldCache = &engineState.manifoldCache;
  if (!result)
    return;

  if (result->getNumContacts() == 1u) {
    auto& contact = result->getContact(0);
    auto* object1 = contact.collisionObject1;
    auto* object2 = contact.collisionObject2;
    if (!object1 || !object2)
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
  auto& pendingContactIndices = engineState.pendingContactIndicesScratch;
  auto& pendingLocalPoints = engineState.pendingLocalPointsScratch;
  pendingContactIndices.clear();
  pendingLocalPoints.clear();
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
    if (!object1 || !object2)
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

namespace {

// Sorted id->object scratch lookup: a rebuilt std::unordered_map would
// allocate one hash node per object on every collide() and trip the
// StepAllocation gates; a capacity-retaining sorted vector allocates nothing
// in steady state.
using ObjectsByIdScratch
    = std::vector<std::pair<std::size_t, CollisionObject*>>;

void appendManifoldCacheIds(
    const std::vector<CollisionObject*>& objects, ObjectsByIdScratch& scratch)
{
  for (auto* object : objects) {
    const auto id = getManifoldCacheId(object);
    if (id != 0u)
      scratch.emplace_back(id, object);
  }
}

void refreshManifoldCacheImpl(
    native::PersistentManifoldCache& manifoldCache, ObjectsByIdScratch& scratch)
{
  std::sort(
      scratch.begin(), scratch.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
      });

  const auto findObject = [&scratch](std::size_t id) -> CollisionObject* {
    const auto it = std::lower_bound(
        scratch.begin(),
        scratch.end(),
        id,
        [](const auto& entry, std::size_t value) {
          return entry.first < value;
        });
    if (it == scratch.end() || it->first != id)
      return nullptr;

    return it->second;
  };

  manifoldCache.refreshAllWith(
      [&findObject](std::size_t idA, std::size_t idB)
          -> std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> {
        auto* objectA = findObject(idA);
        auto* objectB = findObject(idB);
        if (!objectA || !objectB)
          return std::nullopt;

        return std::make_pair(objectA->getTransform(), objectB->getTransform());
      },
      /*breakingThreshold=*/0.04);
}

} // namespace

//==============================================================================
void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects,
    DetectorEngineState& engineState)
{
  auto& scratch = engineState.objectsByIdScratch;
  scratch.clear();
  appendManifoldCacheIds(objects, scratch);
  refreshManifoldCacheImpl(engineState.manifoldCache, scratch);
}

//==============================================================================
void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects1,
    const std::vector<CollisionObject*>& objects2,
    DetectorEngineState& engineState)
{
  auto& scratch = engineState.objectsByIdScratch;
  scratch.clear();
  appendManifoldCacheIds(objects1, scratch);
  appendManifoldCacheIds(objects2, scratch);
  refreshManifoldCacheImpl(engineState.manifoldCache, scratch);
}

//==============================================================================
bool emitContacts(
    const native::CollisionResult& nativeResult,
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
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
bool processNativePair(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    native::CollisionResult& nativeResult)
{
  if (shouldSkipPair(object1, object2, option))
    return false;

  if (result && result->getNumContacts() >= option.maxNumContacts)
    return true;

  // Reused across pairs by the caller so steady-state stepping stays
  // allocation-free (StepAllocation gate discipline).
  nativeResult.clear();
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
DARTCollisionDetector::Registrar<DARTCollisionDetector>
    DARTCollisionDetector::mRegistrar{
        DARTCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<DARTCollisionDetector> {
          return DARTCollisionDetector::create();
        }};

//==============================================================================
// Transition alias: registers this same engine under the interim "native"
// key so in-tree and downstream callers that already select "native" keep
// working. Remove in 6.21.
DARTCollisionDetector::Registrar<DARTCollisionDetector>
    DARTCollisionDetector::mNativeAliasRegistrar{
        "native", []() -> std::shared_ptr<DARTCollisionDetector> {
          return DARTCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<DARTCollisionDetector> DARTCollisionDetector::create()
{
  return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
}

//==============================================================================
DARTCollisionDetector::~DARTCollisionDetector()
{
  removeManifoldCache(this);
}

//==============================================================================
native::CachedContact* DARTCollisionDetector::getCachedContact(
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
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
void DARTCollisionDetector::notifyCollisionObjectDestroying(
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
DARTCollisionDetector::cloneWithoutCollisionObjects() const
{
  auto clone = DARTCollisionDetector::create();
  clone->setNumCollisionThreads(mNumCollisionThreads);
  return clone;
}

//==============================================================================
const std::string& DARTCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& DARTCollisionDetector::getStaticType()
{
  static const std::string type = "dart";
  return type;
}

//==============================================================================
void DARTCollisionDetector::setNumCollisionThreads(std::size_t numThreads)
{
  if (numThreads == 0u) {
    numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0u)
      numThreads = 1u;
  }

  mNumCollisionThreads = std::max<std::size_t>(1u, numThreads);
}

//==============================================================================
std::size_t DARTCollisionDetector::getNumCollisionThreads() const
{
  return mNumCollisionThreads;
}

//==============================================================================
std::unique_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup()
{
  return std::make_unique<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (option.maxNumContacts == 0u)
    return false;

  if (!checkGroupValidity(this, group))
    return false;

  auto* nativeGroup = static_cast<DARTCollisionGroup*>(group);
  {
    DART_PROFILE_SCOPED_N("Native::updateEngineData");
    nativeGroup->updateEngineData();
  }
  auto& engineState = getOrCreateEngineState(this);
  {
    DART_PROFILE_SCOPED_N("Native::refreshManifoldCache");
    refreshManifoldCache(nativeGroup->mCollisionObjects, engineState);
  }

  native::CollisionResult& scratchResult = engineState.narrowphaseScratch;
  bool collisionFound = false;
  {
    DART_PROFILE_SCOPED_N("Native::visitPairs+narrowphase");
    // Bundle the visitor state behind one pointer so the std::function
    // conversion stays within its small-buffer optimization: a multi-capture
    // lambda heap-allocates on every collide and trips the StepAllocation
    // gates.
    struct PairVisitContext
    {
      DARTCollisionGroup* group;
      const CollisionOption* option;
      CollisionResult* result;
      bool* collisionFound;
      native::CollisionResult* scratchResult;
    } context{nativeGroup, &option, result, &collisionFound, &scratchResult};
    const auto pairVisitor = [&context](std::size_t id1, std::size_t id2) {
      auto* object1 = context.group->mIdToObject.at(id1);
      auto* object2 = context.group->mIdToObject.at(id2);
      return !processNativePair(
          object1,
          object2,
          *context.option,
          context.result,
          *context.collisionFound,
          *context.scratchResult);
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
  }

  if (option.enableContact) {
    DART_PROFILE_SCOPED_N("Native::attachCachedImpulses");
    attachCachedContactImpulses(result, engineState);
  }

  return collisionFound;
}

//==============================================================================
bool DARTCollisionDetector::collide(
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

  auto* nativeGroup1 = static_cast<DARTCollisionGroup*>(group1);
  auto* nativeGroup2 = static_cast<DARTCollisionGroup*>(group2);
  nativeGroup1->updateEngineData();
  nativeGroup2->updateEngineData();
  auto& engineState = getOrCreateEngineState(this);
  refreshManifoldCache(
      nativeGroup1->mCollisionObjects,
      nativeGroup2->mCollisionObjects,
      engineState);

  native::CollisionResult& scratchResult = engineState.narrowphaseScratch;
  bool collisionFound = false;
  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      if (processNativePair(
              static_cast<DARTCollisionObject*>(object1),
              static_cast<DARTCollisionObject*>(object2),
              option,
              result,
              collisionFound,
              scratchResult)) {
        if (option.enableContact)
          attachCachedContactImpulses(result, engineState);
        return collisionFound;
      }
    }
  }

  if (option.enableContact)
    attachCachedContactImpulses(result, engineState);

  return collisionFound;
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* group, const DistanceOption& option, DistanceResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group))
    return 0.0;

  auto* nativeGroup = static_cast<DARTCollisionGroup*>(group);
  nativeGroup->updateEngineData();

  NativeDistanceCandidate best;
  for (std::size_t i = 0u; i < nativeGroup->mCollisionObjects.size(); ++i) {
    for (std::size_t j = i + 1u; j < nativeGroup->mCollisionObjects.size();
         ++j) {
      const auto* object1 = static_cast<const DARTCollisionObject*>(
          nativeGroup->mCollisionObjects[i]);
      const auto* object2 = static_cast<const DARTCollisionObject*>(
          nativeGroup->mCollisionObjects[j]);

      if (processNativeDistancePair(object1, object2, option, best))
        return convertDistanceResult(best, option, result);
    }
  }

  return convertDistanceResult(best, option, result);
}

//==============================================================================
double DARTCollisionDetector::distance(
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

  auto* nativeGroup1 = static_cast<DARTCollisionGroup*>(group1);
  auto* nativeGroup2 = static_cast<DARTCollisionGroup*>(group2);
  nativeGroup1->updateEngineData();
  nativeGroup2->updateEngineData();

  NativeDistanceCandidate best;
  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      if (processNativeDistancePair(
              static_cast<const DARTCollisionObject*>(object1),
              static_cast<const DARTCollisionObject*>(object2),
              option,
              best)) {
        return convertDistanceResult(best, option, result);
      }
    }
  }

  return convertDistanceResult(best, option, result);
}

//==============================================================================
bool DARTCollisionDetector::raycast(
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

  auto* nativeGroup = static_cast<DARTCollisionGroup*>(group);
  nativeGroup->updateEngineData();

  std::vector<NativeRayHitCandidate> hits;
  for (auto* object : nativeGroup->mCollisionObjects) {
    const auto* nativeObject = static_cast<const DARTCollisionObject*>(object);
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
DARTCollisionDetector::DARTCollisionDetector() : CollisionDetector()
{
  getOrCreateEngineState(this);
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
std::unique_ptr<CollisionObject> DARTCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<DARTCollisionObject>(
      new DARTCollisionObject(this, shapeFrame));
}

//==============================================================================
void DARTCollisionDetector::refreshCollisionObject(CollisionObject* /*object*/)
{
  // Do nothing. DARTCollisionObject refreshes lazily in updateEngineData().
}

} // namespace collision
} // namespace dart
