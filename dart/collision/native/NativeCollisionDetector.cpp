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
#include "dart/collision/native/narrow_phase/NarrowPhase.hpp"
#include "dart/common/Console.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace dart {
namespace collision {

namespace {

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
  nativeOption.maxNumContacts = nativeOption.enableContact
                                    ? option.getEffectiveMaxNumContactsPerPair()
                                    : 1u;
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
bool processNativePair(
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound)
{
  if (shouldSkipPair(object1, object2, option))
    return false;

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
NativeCollisionDetector::~NativeCollisionDetector() = default;

//==============================================================================
std::shared_ptr<CollisionDetector>
NativeCollisionDetector::cloneWithoutCollisionObjects() const
{
  return NativeCollisionDetector::create();
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
  nativeGroup->updateEngineData();

  bool collisionFound = false;
  nativeGroup->mBroadPhase->visitPairs([&](std::size_t id1, std::size_t id2) {
    auto* object1 = nativeGroup->mIdToObject.at(id1);
    auto* object2 = nativeGroup->mIdToObject.at(id2);
    return !processNativePair(object1, object2, option, result, collisionFound);
  });

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

  bool collisionFound = false;
  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      if (processNativePair(
              static_cast<NativeCollisionObject*>(object1),
              static_cast<NativeCollisionObject*>(object2),
              option,
              result,
              collisionFound)) {
        return collisionFound;
      }
    }
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
