/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/collision/dart/DartCollisionEngine.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DARTCollide.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/dart/DartCollisionBroadphase.hpp"

#include <Eigen/Dense>

#include <cstdint>
#include <unordered_map>

namespace dart {
namespace collision {

namespace {

bool overlaps(const CoreObject& a, const CoreObject& b)
{
  return (a.worldAabbMin.array() <= b.worldAabbMax.array()).all()
         && (a.worldAabbMax.array() >= b.worldAabbMin.array()).all();
}

bool isClose(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, double tol)
{
  return (pos1 - pos2).norm() < tol;
}

void mergeContacts(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& totalResult,
    const CollisionResult& pairResult)
{
  if (!pairResult.isCollision()) [[likely]]
    return;

  const auto tol = 3.0e-12;

  for (auto pairContact : pairResult.getContacts()) {
    if (!option.allowNegativePenetrationDepthContacts
        && pairContact.penetrationDepth < 0.0) {
      continue;
    }

    bool foundClose = false;

    for (const auto& totalContact : totalResult.getContacts()) {
      if (isClose(pairContact.point, totalContact.point, tol)) {
        foundClose = true;
        break;
      }
    }

    if (foundClose)
      continue;

    auto contact = pairContact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    totalResult.addContact(contact);

    if (totalResult.getNumContacts() >= option.maxNumContacts)
      break;
  }
}

} // namespace

//==============================================================================
bool DartCollisionEngine::collide(
    const ObjectList& objects,
    const CollisionOption& option,
    CollisionResult* result) const
{
  if (objects.size() < 2u) [[unlikely]]
    return false;

  constexpr std::uint8_t kGroup = 1u;

  std::vector<CollisionObject*> collObjects;
  collObjects.reserve(objects.size());

  std::vector<CoreBroadphaseEntry> entries;
  entries.reserve(objects.size());
  for (auto* object : objects) {
    const auto* dartObject = static_cast<const DARTCollisionObject*>(object);
    collObjects.push_back(object);
    entries.push_back({&dartObject->getCoreObject(), kGroup});
  }

  std::vector<CoreBroadphasePair> pairs;
  pairs.reserve(objects.size());
  computeSweepPairs(entries, kGroup, kGroup, &pairs);

  bool collisionFound = false;
  const auto& filter = option.collisionFilter;

  for (const auto& pair : pairs) {
    auto* collObj1 = collObjects[pair.first];
    auto* collObj2 = collObjects[pair.second];

    if (filter && filter->ignoresCollision(collObj1, collObj2)) [[unlikely]]
      continue;

    const auto& aabb1 = *entries[pair.first].object;
    const auto& aabb2 = *entries[pair.second].object;
    if (!overlaps(aabb1, aabb2))
      continue;

    collisionFound = checkPair(collObj1, collObj2, option, result);

    if (result) {
      if (result->getNumContacts() >= option.maxNumContacts) [[unlikely]]
        return true;
    } else if (collisionFound) [[unlikely]] {
      return true;
    }
  }

  return collisionFound;
}

//==============================================================================
bool DartCollisionEngine::collide(
    const ObjectList& objects1,
    const ObjectList& objects2,
    const CollisionOption& option,
    CollisionResult* result) const
{
  if (objects1.empty() || objects2.empty()) [[unlikely]]
    return false;

  constexpr std::uint8_t kGroup1 = 1u;
  constexpr std::uint8_t kGroup2 = 1u << 1u;

  std::vector<CollisionObject*> collObjects;
  collObjects.reserve(objects1.size() + objects2.size());

  std::vector<CoreBroadphaseEntry> entries;
  entries.reserve(objects1.size() + objects2.size());

  std::unordered_map<CollisionObject*, std::size_t> entryMap;
  entryMap.reserve(objects1.size() + objects2.size());

  auto addEntry = [&](CollisionObject* object, std::uint8_t mask) {
    auto [it, inserted] = entryMap.emplace(object, entries.size());
    if (inserted) {
      const auto* dartObject = static_cast<const DARTCollisionObject*>(object);
      entries.push_back({&dartObject->getCoreObject(), mask});
      collObjects.push_back(object);
    } else {
      entries[it->second].mask |= mask;
    }
  };

  for (auto* object : objects1)
    addEntry(object, kGroup1);
  for (auto* object : objects2)
    addEntry(object, kGroup2);

  std::vector<CoreBroadphasePair> pairs;
  pairs.reserve(entries.size());
  computeSweepPairs(entries, kGroup1, kGroup2, &pairs);

  bool collisionFound = false;
  const auto& filter = option.collisionFilter;

  for (const auto& pair : pairs) {
    auto* collObj1 = collObjects[pair.first];
    auto* collObj2 = collObjects[pair.second];

    if (filter && filter->ignoresCollision(collObj1, collObj2))
      continue;

    const auto& aabb1 = *entries[pair.first].object;
    const auto& aabb2 = *entries[pair.second].object;
    if (!overlaps(aabb1, aabb2))
      continue;

    collisionFound = checkPair(collObj1, collObj2, option, result);

    if (result) {
      if (result->getNumContacts() >= option.maxNumContacts)
        return true;
    } else if (collisionFound) {
      return true;
    }
  }

  return collisionFound;
}

//==============================================================================
bool DartCollisionEngine::checkPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result) const
{
  CollisionResult pairResult;

  dart::collision::collide(o1, o2, pairResult);

  if (!result) {
    if (option.allowNegativePenetrationDepthContacts)
      return pairResult.isCollision();

    for (const auto& contact : pairResult.getContacts()) {
      if (contact.penetrationDepth >= 0.0)
        return true;
    }

    return false;
  }

  mergeContacts(o1, o2, option, *result, pairResult);

  return pairResult.isCollision();
}

} // namespace collision
} // namespace dart
