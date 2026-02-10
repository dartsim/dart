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

#include "dart/collision/dart/dart_query_helper.hpp"

#include "dart/collision/collision_filter.hpp"
#include "dart/collision/collision_object.hpp"
#include "dart/collision/contact.hpp"
#include "dart/collision/dart/shape_adapter.hpp"
#include "dart/collision/distance_filter.hpp"
#include "dart/collision/native/collision_world.hpp"
#include "dart/collision/native/narrow_phase/narrow_phase.hpp"
#include "dart/common/logging.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace dart {
namespace collision {

namespace {

struct TemporaryCollisionObject
{
  CollisionObject* mObject = nullptr;
  native::CollisionObject mNativeObject;
};

std::vector<TemporaryCollisionObject> buildTemporaryObjects(
    const std::vector<CollisionObject*>& objects, native::CollisionWorld& world)
{
  std::vector<TemporaryCollisionObject> temporaryObjects;
  temporaryObjects.reserve(objects.size());

  for (auto* object : objects) {
    if (!object) {
      continue;
    }

    if (!object->getShapeFrame()) {
      continue;
    }

    auto shape = adaptShape(object->getShape());
    if (!shape) {
      continue;
    }

    temporaryObjects.push_back(
        TemporaryCollisionObject{
            object,
            world.createObject(std::move(shape), object->getTransform())});
  }

  return temporaryObjects;
}

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

  const auto numContacts = nativeResult.numContacts();
  for (auto i = 0u; i < numContacts; ++i) {
    const auto& cp = nativeResult.getContact(i);

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
    result.addContact(contact);

    if (result.getNumContacts() >= option.maxNumContacts) {
      return true;
    }
  }

  return false;
}

} // namespace

bool dartCollide(
    const std::vector<CollisionObject*>& objects1,
    const std::vector<CollisionObject*>& objects2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result) {
    result->clear();
  }

  if (0u == option.maxNumContacts) {
    DART_WARN(
        "CollisionOption::maxNumContacts is 0; skipping collision detection. "
        "Use maxNumContacts >= 1 for binary checks.");
    return false;
  }

  native::CollisionWorld world(native::BroadPhaseType::BruteForce);
  auto temporaryObjects1 = buildTemporaryObjects(objects1, world);
  auto temporaryObjects2 = buildTemporaryObjects(objects2, world);

  if (temporaryObjects1.empty() || temporaryObjects2.empty()) {
    return false;
  }

  const auto isSameGroup = (&objects1 == &objects2);
  const auto& filter = option.collisionFilter;
  bool collisionFound = false;

  native::CollisionOption nativeOption;
  nativeOption.enableContact = option.enableContact;
  nativeOption.maxNumContacts = option.maxNumContacts;
  nativeOption.collisionFilter = nullptr;

  for (auto i = 0u; i < temporaryObjects1.size(); ++i) {
    const auto jStart = isSameGroup ? i + 1u : 0u;

    for (auto j = jStart; j < temporaryObjects2.size(); ++j) {
      const auto& object1 = temporaryObjects1[i];
      const auto& object2 = temporaryObjects2[j];

      if (filter
          && filter->ignoresCollision(object1.mObject, object2.mObject)) {
        continue;
      }

      native::CollisionResult pairResult;
      if (!native::NarrowPhase::collide(
              object1.mNativeObject,
              object2.mNativeObject,
              nativeOption,
              pairResult)) {
        continue;
      }

      collisionFound = true;

      if (!result) {
        return true;
      }

      if (addContacts(
              option, object1.mObject, object2.mObject, pairResult, *result)) {
        return true;
      }
    }
  }

  return collisionFound;
}

double dartDistance(
    const std::vector<CollisionObject*>& objects1,
    const std::vector<CollisionObject*>& objects2,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (result) {
    result->clear();
  }

  native::CollisionWorld world(native::BroadPhaseType::BruteForce);
  auto temporaryObjects1 = buildTemporaryObjects(objects1, world);
  auto temporaryObjects2 = buildTemporaryObjects(objects2, world);

  if (temporaryObjects1.empty() || temporaryObjects2.empty()) {
    return std::max(0.0, option.distanceLowerBound);
  }

  const auto isSameGroup = (&objects1 == &objects2);
  const auto& filter = option.distanceFilter;

  double bestDistance = std::numeric_limits<double>::max();
  native::DistanceResult bestPairResult;
  CollisionObject* bestObject1 = nullptr;
  CollisionObject* bestObject2 = nullptr;
  bool found = false;

  native::DistanceOption nativeOption;
  nativeOption.upperBound = std::numeric_limits<double>::max();
  nativeOption.enableNearestPoints = option.enableNearestPoints;

  auto shouldStop = [&]() {
    return found && bestDistance <= option.distanceLowerBound;
  };

  for (auto i = 0u; i < temporaryObjects1.size(); ++i) {
    const auto jStart = isSameGroup ? i + 1u : 0u;

    for (auto j = jStart; j < temporaryObjects2.size(); ++j) {
      const auto& object1 = temporaryObjects1[i];
      const auto& object2 = temporaryObjects2[j];

      if (filter && !filter->needDistance(object1.mObject, object2.mObject)) {
        continue;
      }

      native::DistanceResult pairResult;
      const auto distance = native::NarrowPhase::distance(
          object1.mNativeObject,
          object2.mNativeObject,
          nativeOption,
          pairResult);

      if (distance < bestDistance) {
        bestDistance = distance;
        bestPairResult = pairResult;
        bestObject1 = object1.mObject;
        bestObject2 = object2.mObject;
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
    return std::max(0.0, option.distanceLowerBound);
  }

  if (result) {
    result->unclampedMinDistance = bestDistance;
    result->minDistance = std::max(bestDistance, option.distanceLowerBound);
    result->shapeFrame1 = bestObject1 ? bestObject1->getShapeFrame() : nullptr;
    result->shapeFrame2 = bestObject2 ? bestObject2->getShapeFrame() : nullptr;
    result->nearestPoint1 = bestPairResult.pointOnObject1;
    result->nearestPoint2 = bestPairResult.pointOnObject2;
  }

  return std::max(bestDistance, option.distanceLowerBound);
}

bool dartRaycast(
    const std::vector<CollisionObject*>& objects,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (result) {
    result->clear();
  }

  const auto delta = to - from;
  const auto totalLength = delta.norm();
  if (totalLength <= 0.0) {
    return false;
  }

  native::CollisionWorld world(native::BroadPhaseType::BruteForce);
  auto temporaryObjects = buildTemporaryObjects(objects, world);
  if (temporaryObjects.empty()) {
    return false;
  }

  const native::Ray ray(from, delta, totalLength);
  const auto nativeOption = native::RaycastOption::unlimited();

  bool hitFound = false;
  RayHit closestHit;
  auto closestFraction = std::numeric_limits<double>::max();
  std::vector<RayHit> hits;
  hits.reserve(temporaryObjects.size());

  for (const auto& object : temporaryObjects) {
    if (!option.passesFilter(object.mObject)) {
      continue;
    }

    native::RaycastResult pairResult;
    if (!native::NarrowPhase::raycast(
            ray, object.mNativeObject, nativeOption, pairResult)) {
      continue;
    }

    hitFound = true;

    RayHit hit;
    hit.mCollisionObject = object.mObject;
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

} // namespace collision
} // namespace dart
