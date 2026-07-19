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
#include "dart/collision/native/NativeCollisionGroup.hpp"
#include "dart/collision/native/NativeCollisionObject.hpp"
#include "dart/collision/native/PersistentManifoldCache.hpp"
#include "dart/collision/native/narrow_phase/NarrowPhase.hpp"
#include "dart/collision/native/shapes/Shape.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Profile.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart {
namespace collision {

namespace {

//==============================================================================
constexpr std::size_t kCompactManifoldContactTarget = 3u;
constexpr std::size_t kFourPointPlanarContactTarget = 4u;
constexpr std::size_t kPlaneBoxManifoldContactTarget = 4u;
constexpr double kConvexFaceOppositionCosine = 0.995;
constexpr double kConvexFaceContactNormalCosine = 0.95;
constexpr double kConvexFacePlaneToleranceScale = 1e-8;
constexpr double kConvexFacePolygonToleranceScale = 1e-10;
constexpr double kConvexFacePatchAreaToleranceScale = 1e-12;
constexpr double kConvexFacePatchAreaFractionTolerance = 1e-4;

//==============================================================================
std::size_t getSolverFacingManifoldContactTarget(
    NativeCollisionDetector::ContactManifoldMode mode)
{
  return mode == NativeCollisionDetector::ContactManifoldMode::FourPointPlanar
             ? kFourPointPlanarContactTarget
             : kCompactManifoldContactTarget;
}

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
    const CollisionOption& option,
    const CollisionResult* result,
    NativeCollisionDetector::ContactManifoldMode mode)
{
  native::CollisionOption nativeOption;
  nativeOption.enableContact = option.enableContact && result != nullptr;
  if (nativeOption.enableContact) {
    nativeOption.maxNumContacts = std::min(
        option.getEffectiveMaxNumContactsPerPair(),
        getSolverFacingManifoldContactTarget(mode));
  } else {
    nativeOption.maxNumContacts = 1u;
  }
  nativeOption.collisionFilter = nullptr;

  return nativeOption;
}

//==============================================================================
bool shouldSkipPair(
    const NativeCollisionObject* object1,
    const NativeCollisionObject* object2,
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
using ManifoldCacheMap = std::unordered_map<
    const NativeCollisionDetector*,
    std::unique_ptr<native::PersistentManifoldCache>>;

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
    const NativeCollisionDetector* detector)
{
  auto& registry = getManifoldCacheRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto it = registry.caches.find(detector);
  if (it == registry.caches.end())
    return nullptr;

  return it->second.get();
}

//==============================================================================
native::PersistentManifoldCache& getOrCreateManifoldCache(
    const NativeCollisionDetector* detector)
{
  auto& registry = getManifoldCacheRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  auto& cache = registry.caches[detector];
  if (!cache)
    cache = std::make_unique<native::PersistentManifoldCache>();

  return *cache;
}

//==============================================================================
void removeManifoldCache(const NativeCollisionDetector* detector)
{
  auto& registry = getManifoldCacheRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.caches.erase(detector);
}

//==============================================================================
using ContactManifoldModeMap = std::unordered_map<
    const NativeCollisionDetector*,
    NativeCollisionDetector::ContactManifoldMode>;

//==============================================================================
struct ContactManifoldModeRegistry
{
  ContactManifoldModeMap modes;
  std::mutex mutex;
};

//==============================================================================
ContactManifoldModeRegistry& getContactManifoldModeRegistry()
{
  static ContactManifoldModeRegistry registry;
  return registry;
}

//==============================================================================
NativeCollisionDetector::ContactManifoldMode lookupContactManifoldMode(
    const NativeCollisionDetector* detector)
{
  auto& registry = getContactManifoldModeRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto it = registry.modes.find(detector);
  if (it == registry.modes.end())
    return NativeCollisionDetector::ContactManifoldMode::Compact;

  return it->second;
}

//==============================================================================
void storeContactManifoldMode(
    const NativeCollisionDetector* detector,
    NativeCollisionDetector::ContactManifoldMode mode)
{
  auto& registry = getContactManifoldModeRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  if (mode == NativeCollisionDetector::ContactManifoldMode::Compact) {
    registry.modes.erase(detector);
    return;
  }

  registry.modes[detector] = mode;
}

//==============================================================================
void eraseContactManifoldMode(const NativeCollisionDetector* detector)
{
  auto& registry = getContactManifoldModeRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.modes.erase(detector);
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

//==============================================================================
void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects,
    native::PersistentManifoldCache* manifoldCache)
{
  if (!manifoldCache)
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
  if (!manifoldCache)
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
struct ConvexWorldFace
{
  std::size_t index = 0u;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> vertices;
};

//==============================================================================
double cross2d(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs)
{
  return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

//==============================================================================
double signedPolygonArea(const std::vector<Eigen::Vector2d>& polygon)
{
  double twiceArea = 0.0;
  for (std::size_t i = 0u; i < polygon.size(); ++i) {
    twiceArea += cross2d(polygon[i], polygon[(i + 1u) % polygon.size()]);
  }
  return 0.5 * twiceArea;
}

//==============================================================================
void removeAdjacentDuplicatePoints(
    std::vector<Eigen::Vector2d>& polygon, double tolerance)
{
  if (polygon.empty())
    return;

  const double toleranceSquared = tolerance * tolerance;
  std::vector<Eigen::Vector2d> unique;
  unique.reserve(polygon.size());
  for (const auto& point : polygon) {
    if (unique.empty()
        || (point - unique.back()).squaredNorm() > toleranceSquared) {
      unique.push_back(point);
    }
  }
  if (unique.size() > 1u
      && (unique.front() - unique.back()).squaredNorm() <= toleranceSquared) {
    unique.pop_back();
  }
  polygon = std::move(unique);
}

//==============================================================================
void orderConvexPolygon(std::vector<Eigen::Vector2d>& polygon, double tolerance)
{
  removeAdjacentDuplicatePoints(polygon, tolerance);
  if (polygon.size() < 3u)
    return;

  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& point : polygon)
    centroid += point;
  centroid /= static_cast<double>(polygon.size());

  std::sort(
      polygon.begin(),
      polygon.end(),
      [&](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
        const Eigen::Vector2d lhsDelta = lhs - centroid;
        const Eigen::Vector2d rhsDelta = rhs - centroid;
        const double lhsAngle = std::atan2(lhsDelta.y(), lhsDelta.x());
        const double rhsAngle = std::atan2(rhsDelta.y(), rhsDelta.x());
        if (lhsAngle != rhsAngle)
          return lhsAngle < rhsAngle;
        if (lhsDelta.squaredNorm() != rhsDelta.squaredNorm())
          return lhsDelta.squaredNorm() < rhsDelta.squaredNorm();
        if (lhs.x() != rhs.x())
          return lhs.x() < rhs.x();
        return lhs.y() < rhs.y();
      });
  removeAdjacentDuplicatePoints(polygon, tolerance);
  if (polygon.size() >= 3u && signedPolygonArea(polygon) < 0.0)
    std::reverse(polygon.begin(), polygon.end());
}

//==============================================================================
std::vector<Eigen::Vector2d> clipConvexPolygon(
    std::vector<Eigen::Vector2d> subject,
    const std::vector<Eigen::Vector2d>& clip,
    double tolerance)
{
  for (std::size_t clipIndex = 0u; clipIndex < clip.size(); ++clipIndex) {
    if (subject.empty())
      break;

    const Eigen::Vector2d& edgeStart = clip[clipIndex];
    const Eigen::Vector2d& edgeEnd = clip[(clipIndex + 1u) % clip.size()];
    const Eigen::Vector2d edge = edgeEnd - edgeStart;
    std::vector<Eigen::Vector2d> output;
    output.reserve(subject.size() + 1u);

    Eigen::Vector2d previous = subject.back();
    double previousDistance = cross2d(edge, previous - edgeStart);
    bool previousInside = previousDistance >= -tolerance;
    for (const auto& current : subject) {
      const double currentDistance = cross2d(edge, current - edgeStart);
      const bool currentInside = currentDistance >= -tolerance;
      if (currentInside != previousInside) {
        const double denominator = previousDistance - currentDistance;
        if (std::abs(denominator) > std::numeric_limits<double>::epsilon()) {
          const double ratio = previousDistance / denominator;
          output.push_back(previous + ratio * (current - previous));
        }
      }
      if (currentInside)
        output.push_back(current);

      previous = current;
      previousDistance = currentDistance;
      previousInside = currentInside;
    }
    removeAdjacentDuplicatePoints(output, tolerance);
    subject = std::move(output);
  }
  return subject;
}

//==============================================================================
template <typename DepthFunction>
std::vector<Eigen::Vector2d> clipPolygonByContactDepth(
    const std::vector<Eigen::Vector2d>& polygon,
    const DepthFunction& getDepth,
    double contactTolerance,
    double polygonTolerance)
{
  if (polygon.empty())
    return {};

  std::vector<Eigen::Vector2d> output;
  output.reserve(polygon.size() + 1u);
  Eigen::Vector2d previous = polygon.back();
  double previousValue = getDepth(previous) + contactTolerance;
  bool previousInside = previousValue >= 0.0;
  for (const auto& current : polygon) {
    const double currentValue = getDepth(current) + contactTolerance;
    const bool currentInside = currentValue >= 0.0;
    if (currentInside != previousInside) {
      const double denominator = previousValue - currentValue;
      if (std::abs(denominator) > std::numeric_limits<double>::epsilon()) {
        const double ratio = previousValue / denominator;
        output.push_back(previous + ratio * (current - previous));
      }
    }
    if (currentInside)
      output.push_back(current);

    previous = current;
    previousValue = currentValue;
    previousInside = currentInside;
  }
  removeAdjacentDuplicatePoints(output, polygonTolerance);
  return output;
}

//==============================================================================
double computeConvexWorldScale(
    const native::ConvexShape& shape, const Eigen::Isometry3d& transform)
{
  if (shape.getVertices().empty())
    return 1.0;

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& vertex : shape.getVertices())
    centroid += transform * vertex;
  centroid /= static_cast<double>(shape.getVertices().size());

  double scale = 0.0;
  for (const auto& vertex : shape.getVertices())
    scale = std::max(scale, (transform * vertex - centroid).norm());
  return std::max(1.0, scale);
}

//==============================================================================
std::vector<ConvexWorldFace> makeConvexWorldFaces(
    const native::ConvexShape& shape,
    const Eigen::Isometry3d& transform,
    double scale)
{
  std::vector<Eigen::Vector3d> worldVertices;
  worldVertices.reserve(shape.getVertices().size());
  for (const auto& vertex : shape.getVertices())
    worldVertices.push_back(transform * vertex);

  const double planeTolerance = kConvexFacePlaneToleranceScale * scale;
  const double duplicateToleranceSquared = planeTolerance * planeTolerance;
  std::vector<ConvexWorldFace> result;
  result.reserve(shape.getFaces().size());
  for (std::size_t faceIndex = 0u; faceIndex < shape.getFaces().size();
       ++faceIndex) {
    const auto& localFace = shape.getFaces()[faceIndex];
    ConvexWorldFace face;
    face.index = faceIndex;
    face.point = transform * localFace.point;
    face.normal = transform.linear() * localFace.normal;
    const double normalNorm = face.normal.norm();
    if (!(normalNorm > 0.0) || !std::isfinite(normalNorm))
      continue;
    face.normal /= normalNorm;

    for (const auto& vertex : worldVertices) {
      if (std::abs(face.normal.dot(vertex - face.point)) > planeTolerance)
        continue;

      bool duplicate = false;
      for (const auto& existing : face.vertices) {
        if ((vertex - existing).squaredNorm() <= duplicateToleranceSquared) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate)
        face.vertices.push_back(vertex);
    }
    if (face.vertices.size() >= 3u)
      result.push_back(std::move(face));
  }
  return result;
}

//==============================================================================
void makePlanarBasis(
    const Eigen::Vector3d& normal,
    Eigen::Vector3d& tangent1,
    Eigen::Vector3d& tangent2)
{
  int axis = 0;
  normal.cwiseAbs().minCoeff(&axis);
  tangent1 = normal.cross(Eigen::Vector3d::Unit(axis)).normalized();
  tangent2 = normal.cross(tangent1);
}

//==============================================================================
bool contactPointLess(
    const native::ContactPoint& lhs, const native::ContactPoint& rhs)
{
  for (Eigen::Index i = 0; i < 3; ++i) {
    if (lhs.position[i] != rhs.position[i])
      return lhs.position[i] < rhs.position[i];
  }
  return lhs.depth > rhs.depth;
}

//==============================================================================
struct ConvexFacePatchCandidate
{
  std::size_t faceIndex1 = 0u;
  std::size_t faceIndex2 = 0u;
  double score = std::numeric_limits<double>::max();
  std::vector<native::ContactPoint> contacts;
};

//==============================================================================
std::optional<ConvexFacePatchCandidate> makeConvexFacePatchCandidate(
    const ConvexWorldFace& face1,
    const ConvexWorldFace& face2,
    const native::ContactPoint& compactContact,
    double scale)
{
  if (face1.normal.dot(face2.normal) > -kConvexFaceOppositionCosine)
    return std::nullopt;

  Eigen::Vector3d outwardNormal = face1.normal - face2.normal;
  const double outwardNormalNorm = outwardNormal.norm();
  if (!(outwardNormalNorm > 0.0))
    return std::nullopt;
  outwardNormal /= outwardNormalNorm;
  const Eigen::Vector3d contactNormal = -outwardNormal;
  const double normalAlignment = contactNormal.dot(compactContact.normal);
  if (normalAlignment < kConvexFaceContactNormalCosine)
    return std::nullopt;

  const double denominator1 = face1.normal.dot(outwardNormal);
  const double denominator2 = face2.normal.dot(outwardNormal);
  if (denominator1 < kConvexFaceOppositionCosine
      || denominator2 > -kConvexFaceOppositionCosine) {
    return std::nullopt;
  }

  const Eigen::Vector3d origin = 0.5 * (face1.point + face2.point);
  Eigen::Vector3d tangent1;
  Eigen::Vector3d tangent2;
  makePlanarBasis(outwardNormal, tangent1, tangent2);
  const auto project = [&](const Eigen::Vector3d& point) {
    const Eigen::Vector3d delta = point - origin;
    return Eigen::Vector2d(delta.dot(tangent1), delta.dot(tangent2));
  };
  const auto unproject = [&](const Eigen::Vector2d& point) {
    return origin + point.x() * tangent1 + point.y() * tangent2;
  };

  const double polygonTolerance = kConvexFacePolygonToleranceScale * scale;
  std::vector<Eigen::Vector2d> polygon1;
  polygon1.reserve(face1.vertices.size());
  for (const auto& vertex : face1.vertices)
    polygon1.push_back(project(vertex));
  orderConvexPolygon(polygon1, polygonTolerance);

  std::vector<Eigen::Vector2d> polygon2;
  polygon2.reserve(face2.vertices.size());
  for (const auto& vertex : face2.vertices)
    polygon2.push_back(project(vertex));
  orderConvexPolygon(polygon2, polygonTolerance);
  if (polygon1.size() < 3u || polygon2.size() < 3u)
    return std::nullopt;

  auto overlap = clipConvexPolygon(polygon1, polygon2, polygonTolerance);
  if (overlap.size() < 3u)
    return std::nullopt;
  orderConvexPolygon(overlap, polygonTolerance);

  const auto getSurfaceData = [&](const Eigen::Vector2d& point,
                                  Eigen::Vector3d* position1,
                                  Eigen::Vector3d* position2) {
    const Eigen::Vector3d planePoint = unproject(point);
    const double offset1
        = face1.normal.dot(face1.point - planePoint) / denominator1;
    const double offset2
        = face2.normal.dot(face2.point - planePoint) / denominator2;
    if (position1)
      *position1 = planePoint + offset1 * outwardNormal;
    if (position2)
      *position2 = planePoint + offset2 * outwardNormal;
    return offset1 - offset2;
  };

  const double contactTolerance
      = std::max(1e-9, 64.0 * std::numeric_limits<double>::epsilon() * scale);
  overlap = clipPolygonByContactDepth(
      overlap,
      [&](const Eigen::Vector2d& point) {
        return getSurfaceData(point, nullptr, nullptr);
      },
      contactTolerance,
      polygonTolerance);
  if (overlap.size() < 3u)
    return std::nullopt;
  orderConvexPolygon(overlap, polygonTolerance);

  const double patchArea = std::abs(signedPolygonArea(overlap));
  const double faceArea1 = std::abs(signedPolygonArea(polygon1));
  const double faceArea2 = std::abs(signedPolygonArea(polygon2));
  const double areaTolerance = std::max(
      kConvexFacePatchAreaToleranceScale * scale * scale,
      kConvexFacePatchAreaFractionTolerance * std::min(faceArea1, faceArea2));
  if (patchArea <= areaTolerance)
    return std::nullopt;

  ConvexFacePatchCandidate candidate;
  candidate.faceIndex1 = face1.index;
  candidate.faceIndex2 = face2.index;
  candidate.contacts.reserve(overlap.size());
  double depthSum = 0.0;
  for (const auto& point : overlap) {
    Eigen::Vector3d position1;
    Eigen::Vector3d position2;
    const double depth = getSurfaceData(point, &position1, &position2);
    native::ContactPoint contact;
    contact.position = 0.5 * (position1 + position2);
    contact.normal = contactNormal;
    contact.depth = std::max(0.0, depth);
    candidate.contacts.push_back(contact);
    depthSum += contact.depth;
  }
  std::sort(
      candidate.contacts.begin(), candidate.contacts.end(), contactPointLess);

  const double meanDepth
      = depthSum / static_cast<double>(candidate.contacts.size());
  candidate.score = std::abs(meanDepth - std::max(0.0, compactContact.depth))
                    + (1.0 - normalAlignment) * scale;
  return candidate;
}

//==============================================================================
bool tryBuildFourPointPlanarConvexFacePatch(
    const native::ConvexShape& shape1,
    const Eigen::Isometry3d& transform1,
    const native::ConvexShape& shape2,
    const Eigen::Isometry3d& transform2,
    const native::CollisionOption& option,
    const native::ContactPoint& compactContact,
    native::CollisionResult& result)
{
  const double scale = std::max(
      computeConvexWorldScale(shape1, transform1),
      computeConvexWorldScale(shape2, transform2));
  const auto faces1 = makeConvexWorldFaces(shape1, transform1, scale);
  const auto faces2 = makeConvexWorldFaces(shape2, transform2, scale);

  std::optional<ConvexFacePatchCandidate> best;
  for (const auto& face1 : faces1) {
    for (const auto& face2 : faces2) {
      auto candidate
          = makeConvexFacePatchCandidate(face1, face2, compactContact, scale);
      if (!candidate)
        continue;

      const bool betterScore = !best || candidate->score < best->score;
      const bool deterministicTie
          = best && candidate->score == best->score
            && (candidate->faceIndex1 < best->faceIndex1
                || (candidate->faceIndex1 == best->faceIndex1
                    && candidate->faceIndex2 < best->faceIndex2));
      if (betterScore || deterministicTie)
        best = std::move(candidate);
    }
  }
  if (!best || best->contacts.size() < 3u)
    return false;

  native::ContactManifold reducer;
  for (const auto& contact : best->contacts)
    reducer.addContact(contact);
  std::vector<native::ContactPoint> reduced(
      reducer.getContacts().begin(), reducer.getContacts().end());
  std::sort(reduced.begin(), reduced.end(), contactPointLess);

  const std::size_t numContacts = std::min(
      {reduced.size(), option.maxNumContacts, kFourPointPlanarContactTarget});
  if (numContacts == 0u)
    return false;

  native::ContactManifold manifold;
  manifold.setType(
      numContacts >= 3u ? native::ContactType::Face
                        : (numContacts == 2u ? native::ContactType::Edge
                                             : native::ContactType::Point));
  for (std::size_t i = 0u; i < numContacts; ++i)
    manifold.addContact(reduced[i]);
  result.clear();
  result.addManifold(std::move(manifold));
  return true;
}

//==============================================================================
bool isConvexConvexPair(
    const native::Shape* shape1, const native::Shape* shape2)
{
  return shape1->getType() == native::ShapeType::Convex
         && shape2->getType() == native::ShapeType::Convex;
}

//==============================================================================
bool collideFourPointPlanarConvexConvex(
    const native::Shape* shape1,
    const Eigen::Isometry3d& transform1,
    const native::Shape* shape2,
    const Eigen::Isometry3d& transform2,
    const native::CollisionOption& option,
    native::CollisionResult& result)
{
  const bool hit = native::NarrowPhase::collide(
      shape1, transform1, shape2, transform2, option, result);
  if (!hit || !option.enableContact || result.numContacts() == 0u)
    return hit;

  const native::ContactPoint compactContact = result.getContact(0u);
  (void)tryBuildFourPointPlanarConvexFacePatch(
      static_cast<const native::ConvexShape&>(*shape1),
      transform1,
      static_cast<const native::ConvexShape&>(*shape2),
      transform2,
      option,
      compactContact,
      result);
  return true;
}

//==============================================================================
bool isPlaneBoxPair(const native::Shape* shape1, const native::Shape* shape2)
{
  return (shape1->getType() == native::ShapeType::Plane
          && shape2->getType() == native::ShapeType::Box)
         || (shape1->getType() == native::ShapeType::Box
             && shape2->getType() == native::ShapeType::Plane);
}

//==============================================================================
bool collideFourPointPlanarPlaneBox(
    const native::Shape* shape1,
    const Eigen::Isometry3d& transform1,
    const native::Shape* shape2,
    const Eigen::Isometry3d& transform2,
    const native::CollisionOption& option,
    native::CollisionResult& result)
{
  if (option.maxNumContacts == 0u)
    return false;

  const bool planeFirst = shape1->getType() == native::ShapeType::Plane;
  const auto* plane
      = static_cast<const native::PlaneShape*>(planeFirst ? shape1 : shape2);
  const auto* box
      = static_cast<const native::BoxShape*>(planeFirst ? shape2 : shape1);
  const auto& planeTransform = planeFirst ? transform1 : transform2;
  const auto& boxTransform = planeFirst ? transform2 : transform1;

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane->getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane->getOffset();
  const Eigen::Vector3d& halfExtents = box->getHalfExtents();

  struct Corner
  {
    Eigen::Vector3d position;
    double signedDistance;
  };

  std::array<Corner, 8u> corners;
  double minDistance = std::numeric_limits<double>::max();
  for (std::size_t i = 0u; i < corners.size(); ++i) {
    const Eigen::Vector3d localCorner(
        (i & 1u) ? halfExtents.x() : -halfExtents.x(),
        (i & 2u) ? halfExtents.y() : -halfExtents.y(),
        (i & 4u) ? halfExtents.z() : -halfExtents.z());
    const Eigen::Vector3d worldCorner = boxTransform * localCorner;
    const double signedDistance = worldNormal.dot(worldCorner - planePoint);
    corners[i] = Corner{worldCorner, signedDistance};
    minDistance = std::min(minDistance, signedDistance);
  }

  if (minDistance > 0.0)
    return false;

  if (!option.enableContact)
    return true;

  const double scale = std::max(
      {1.0,
       halfExtents.cwiseAbs().maxCoeff(),
       boxTransform.translation().cwiseAbs().maxCoeff(),
       planePoint.cwiseAbs().maxCoeff(),
       std::abs(minDistance)});
  const double contactTolerance
      = std::max(1e-9, 64.0 * std::numeric_limits<double>::epsilon() * scale);
  const Eigen::Vector3d contactNormal = planeFirst ? -worldNormal : worldNormal;

  std::array<std::size_t, 8u> candidateIndices{};
  std::size_t numCandidates = 0u;
  for (std::size_t i = 0u; i < corners.size(); ++i) {
    if (corners[i].signedDistance <= contactTolerance)
      candidateIndices[numCandidates++] = i;
  }
  std::sort(
      candidateIndices.begin(),
      candidateIndices.begin() + numCandidates,
      [&](std::size_t lhs, std::size_t rhs) {
        if (corners[lhs].signedDistance != corners[rhs].signedDistance)
          return corners[lhs].signedDistance < corners[rhs].signedDistance;
        return lhs < rhs;
      });

  const std::size_t maxContacts = std::min(
      {numCandidates, option.maxNumContacts, kPlaneBoxManifoldContactTarget});
  native::ContactManifold manifold;
  for (std::size_t i = 0u; i < maxContacts; ++i) {
    const Corner& corner = corners[candidateIndices[i]];
    native::ContactPoint contact;
    contact.position = corner.position - worldNormal * corner.signedDistance;
    contact.normal = contactNormal;
    contact.depth = std::max(0.0, -corner.signedDistance);
    manifold.addContact(contact);
  }

  if (manifold.numContacts() >= 3u)
    manifold.setType(native::ContactType::Face);
  else if (manifold.numContacts() == 2u)
    manifold.setType(native::ContactType::Edge);
  else
    manifold.setType(native::ContactType::Point);
  result.addManifold(std::move(manifold));
  return true;
}

//==============================================================================
bool processNativePair(
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult* result,
    NativeCollisionDetector::ContactManifoldMode manifoldMode,
    bool& collisionFound)
{
  if (shouldSkipPair(object1, object2, option))
    return false;

  if (result && result->getNumContacts() >= option.maxNumContacts)
    return true;

  native::CollisionResult nativeResult;
  const native::CollisionOption nativeOption
      = makeNativeOption(option, result, manifoldMode);
  const auto* shape1 = object1->getNativeShape();
  const auto* shape2 = object2->getNativeShape();
  const bool useFourPointPlaneBox
      = manifoldMode
            == NativeCollisionDetector::ContactManifoldMode::FourPointPlanar
        && isPlaneBoxPair(shape1, shape2);
  const bool useFourPointConvexFaces
      = manifoldMode
            == NativeCollisionDetector::ContactManifoldMode::FourPointPlanar
        && isConvexConvexPair(shape1, shape2);
  const bool hit = useFourPointPlaneBox ? collideFourPointPlanarPlaneBox(
                       shape1,
                       object1->getNativeTransform(),
                       shape2,
                       object2->getNativeTransform(),
                       nativeOption,
                       nativeResult)
                   : useFourPointConvexFaces
                       ? collideFourPointPlanarConvexConvex(
                           shape1,
                           object1->getNativeTransform(),
                           shape2,
                           object2->getNativeTransform(),
                           nativeOption,
                           nativeResult)
                       : native::NarrowPhase::collide(
                           shape1,
                           object1->getNativeTransform(),
                           shape2,
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
  eraseContactManifoldMode(this);
  removeManifoldCache(this);
}

//==============================================================================
void NativeCollisionDetector::setContactManifoldMode(ContactManifoldMode mode)
{
  storeContactManifoldMode(this, mode);
}

//==============================================================================
NativeCollisionDetector::ContactManifoldMode
NativeCollisionDetector::getContactManifoldMode() const
{
  return lookupContactManifoldMode(this);
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
  clone->setContactManifoldMode(getContactManifoldMode());
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
bool NativeCollisionDetector::collide(
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

  auto* nativeGroup = static_cast<NativeCollisionGroup*>(group);
  {
    DART_PROFILE_SCOPED_N("Native::updateEngineData");
    nativeGroup->updateEngineData();
  }
  auto& manifoldCache = getOrCreateManifoldCache(this);
  {
    DART_PROFILE_SCOPED_N("Native::refreshManifoldCache");
    refreshManifoldCache(nativeGroup->mCollisionObjects, &manifoldCache);
  }

  bool collisionFound = false;
  const auto manifoldMode = getContactManifoldMode();
  {
    DART_PROFILE_SCOPED_N("Native::visitPairs+narrowphase");
    const auto pairVisitor = [&](std::size_t id1, std::size_t id2) {
      auto* object1 = nativeGroup->mIdToObject.at(id1);
      auto* object2 = nativeGroup->mIdToObject.at(id2);
      return !processNativePair(
          object1, object2, option, result, manifoldMode, collisionFound);
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
    attachCachedContactImpulses(result, &manifoldCache);
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
  auto& manifoldCache = getOrCreateManifoldCache(this);
  refreshManifoldCache(
      nativeGroup1->mCollisionObjects,
      nativeGroup2->mCollisionObjects,
      &manifoldCache);

  bool collisionFound = false;
  const auto manifoldMode = getContactManifoldMode();
  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      if (processNativePair(
              static_cast<NativeCollisionObject*>(object1),
              static_cast<NativeCollisionObject*>(object2),
              option,
              result,
              manifoldMode,
              collisionFound)) {
        if (option.enableContact)
          attachCachedContactImpulses(result, &manifoldCache);
        return collisionFound;
      }
    }
  }

  if (option.enableContact)
    attachCachedContactImpulses(result, &manifoldCache);

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
  getOrCreateManifoldCache(this);
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
