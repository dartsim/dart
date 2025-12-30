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

#include <Eigen/Dense>

#include <algorithm>

namespace dart {
namespace collision {

namespace {

struct Aabb
{
  Eigen::Vector3d min;
  Eigen::Vector3d max;
};

Aabb computeWorldAabb(const CollisionObject* object)
{
  const auto* dartObject = static_cast<const DARTCollisionObject*>(object);

  return {dartObject->getWorldAabbMin(), dartObject->getWorldAabbMax()};
}

bool overlaps(const Aabb& a, const Aabb& b)
{
  return (a.min.array() <= b.max.array()).all()
         && (a.max.array() >= b.min.array()).all();
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

  std::vector<Aabb> aabbs;
  aabbs.reserve(objects.size());
  for (const auto* object : objects)
    aabbs.push_back(computeWorldAabb(object));

  struct SweepEntry
  {
    double minX;
    double maxX;
    std::size_t index;
  };

  std::vector<SweepEntry> sweep;
  sweep.reserve(objects.size());
  for (std::size_t i = 0; i < aabbs.size(); ++i) {
    sweep.push_back({aabbs[i].min.x(), aabbs[i].max.x(), i});
  }

  std::stable_sort(
      sweep.begin(),
      sweep.end(),
      [](const SweepEntry& a, const SweepEntry& b) {
        return a.minX < b.minX;
      });

  bool collisionFound = false;
  const auto& filter = option.collisionFilter;

  for (std::size_t i = 0; i + 1u < sweep.size(); ++i) {
    const auto& entry = sweep[i];
    const auto maxX = entry.maxX;
    auto* collObj1 = objects[entry.index];

    for (std::size_t j = i + 1u; j < sweep.size(); ++j) {
      if (sweep[j].minX > maxX)
        break;

      const auto otherIndex = sweep[j].index;
      auto* collObj2 = objects[otherIndex];

      if (filter && filter->ignoresCollision(collObj1, collObj2)) [[unlikely]]
        continue;

      if (!overlaps(aabbs[entry.index], aabbs[otherIndex]))
        continue;

      collisionFound = checkPair(collObj1, collObj2, option, result);

      if (result) {
        if (result->getNumContacts() >= option.maxNumContacts) [[unlikely]]
          return true;
      } else if (collisionFound) [[unlikely]] {
        return true;
      }
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

  std::vector<Aabb> aabbs1;
  aabbs1.reserve(objects1.size());
  for (const auto* object : objects1)
    aabbs1.push_back(computeWorldAabb(object));

  std::vector<Aabb> aabbs2;
  aabbs2.reserve(objects2.size());
  for (const auto* object : objects2)
    aabbs2.push_back(computeWorldAabb(object));

  bool collisionFound = false;
  const auto& filter = option.collisionFilter;

  for (auto i = 0u; i < objects1.size(); ++i) {
    auto* collObj1 = objects1[i];

    for (auto j = 0u; j < objects2.size(); ++j) {
      auto* collObj2 = objects2[j];

      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;

      if (!overlaps(aabbs1[i], aabbs2[j]))
        continue;

      collisionFound = checkPair(collObj1, collObj2, option, result);

      if (result) {
        if (result->getNumContacts() >= option.maxNumContacts)
          return true;
      } else if (collisionFound) {
        return true;
      }
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

  if (!result)
    return pairResult.isCollision();

  mergeContacts(o1, o2, option, *result, pairResult);

  return pairResult.isCollision();
}

} // namespace collision
} // namespace dart
