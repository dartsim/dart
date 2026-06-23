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
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DARTCollide.hpp"
#include "dart/collision/dart/DARTCollisionGroup.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart {
namespace collision {

namespace {

struct BroadphaseEntry
{
  CollisionObject* object{nullptr};
  Eigen::Vector3d min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max{Eigen::Vector3d::Zero()};
  bool finite{false};
  bool plane{false};
};

constexpr double kContactDuplicateTolerance = 3.0e-12;
constexpr double kContactPointKeyLowerBound = -9223372036854775808.0;
constexpr double kContactPointKeyUpperBound = 9223372036854775808.0;
constexpr std::size_t kInvalidContactPointIndex
    = std::numeric_limits<std::size_t>::max();

struct ContactPointKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};
};

bool operator==(const ContactPointKey& lhs, const ContactPointKey& rhs);

struct ContactPointBucket
{
  ContactPointKey key;
  std::size_t head{kInvalidContactPointIndex};
  bool occupied{false};
};

class ContactPointIndex
{
public:
  void clear();
  void prepare(std::size_t maxContacts);
  bool containsClose(const Eigen::Vector3d& point) const;
  void add(const Eigen::Vector3d& point);

private:
  bool containsCloseLinear(const Eigen::Vector3d& point) const;
  void ensureBucketCapacity(std::size_t requiredPoints);
  void rehash(std::size_t bucketCount);
  std::size_t findBucketIndex(const ContactPointKey& key) const;
  std::size_t findOrCreateBucketIndex(const ContactPointKey& key);

  std::vector<ContactPointBucket> buckets;
  std::vector<std::size_t> usedBuckets;
  std::vector<Eigen::Vector3d> points;
  std::vector<std::size_t> nextPoint;
  std::size_t bucketMask{0u};
};

struct BroadphaseScratch
{
  std::vector<BroadphaseEntry> finiteEntries1;
  std::vector<BroadphaseEntry> planeEntries1;
  std::vector<BroadphaseEntry> otherEntries1;
  std::vector<BroadphaseEntry> finiteEntries2;
  std::vector<BroadphaseEntry> planeEntries2;
  std::vector<BroadphaseEntry> otherEntries2;
  std::vector<const BroadphaseEntry*> sortedEntries1;
  std::vector<const BroadphaseEntry*> sortedEntries2;
  CollisionResult pairResult;
  ContactPointIndex contactPointIndex;

  void clear();
};

BroadphaseScratch& getBroadphaseScratch();

bool checkPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& pairResult,
    CollisionResult* result = nullptr);

bool processPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

bool shouldStopAfterPair(
    bool pairCollision,
    const CollisionOption& option,
    const CollisionResult* result);

bool isClose(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, double tol);

void postProcess(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& totalResult,
    const CollisionResult& pairResult);

bool isPlaneShape(const CollisionObject* object);

BroadphaseEntry makeBroadphaseEntry(CollisionObject* object);

void buildBroadphaseEntries(
    const std::vector<CollisionObject*>& objects,
    std::vector<BroadphaseEntry>& finiteEntries,
    std::vector<BroadphaseEntry>& planeEntries,
    std::vector<BroadphaseEntry>& otherEntries);

bool overlaps(const BroadphaseEntry& entry1, const BroadphaseEntry& entry2);

bool processFinitePlanePairs(
    const std::vector<BroadphaseEntry>& finiteEntries,
    const std::vector<BroadphaseEntry>& planeEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult,
    bool planeFirst = true);

bool processPlanePlanePairs(
    const std::vector<BroadphaseEntry>& planeEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

bool processPlanePlanePairs(
    const std::vector<BroadphaseEntry>& planeEntries1,
    const std::vector<BroadphaseEntry>& planeEntries2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

bool processFiniteFinitePairs(
    const std::vector<BroadphaseEntry>& entries,
    std::vector<const BroadphaseEntry*>& sortedEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

bool processFiniteFinitePairs(
    const std::vector<BroadphaseEntry>& entries1,
    const std::vector<BroadphaseEntry>& entries2,
    std::vector<const BroadphaseEntry*>& sortedEntries2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

} // anonymous namespace

//==============================================================================
DARTCollisionDetector::Registrar<DARTCollisionDetector>
    DARTCollisionDetector::mRegistrar{
        DARTCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::DARTCollisionDetector> {
          return dart::collision::DARTCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<DARTCollisionDetector> DARTCollisionDetector::create()
{
  return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
}

//==============================================================================
std::shared_ptr<CollisionDetector>
DARTCollisionDetector::cloneWithoutCollisionObjects() const
{
  return DARTCollisionDetector::create();
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
std::unique_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup()
{
  return std::make_unique<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
static bool checkGroupValidity(DARTCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get()) {
    dterr << "[DARTCollisionDetector::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group))
    return false;

  auto casted = static_cast<DARTCollisionGroup*>(group);
  casted->updateEngineData();
  const auto& objects = casted->mCollisionObjects;

  if (objects.empty())
    return false;

  auto& scratch = getBroadphaseScratch();
  scratch.clear();
  if (result)
    scratch.contactPointIndex.prepare(option.maxNumContacts);
  buildBroadphaseEntries(
      objects,
      scratch.finiteEntries1,
      scratch.planeEntries1,
      scratch.otherEntries1);

  auto collisionFound = false;
  if (processFinitePlanePairs(
          scratch.finiteEntries1,
          scratch.planeEntries1,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  if (processFiniteFinitePairs(
          scratch.finiteEntries1,
          scratch.sortedEntries1,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  if (processPlanePlanePairs(
          scratch.planeEntries1,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  const auto& filter = option.collisionFilter;
  for (auto i = 0u; i < scratch.otherEntries1.size(); ++i) {
    for (auto j = i + 1u; j < scratch.otherEntries1.size(); ++j) {
      auto* collObj1 = scratch.otherEntries1[i].object;
      auto* collObj2 = scratch.otherEntries1[j].object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  for (const auto& otherEntry : scratch.otherEntries1) {
    for (const auto& finiteEntry : scratch.finiteEntries1) {
      auto* collObj1 = otherEntry.object;
      auto* collObj2 = finiteEntry.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }

    for (const auto& planeEntry : scratch.planeEntries1) {
      auto* collObj1 = otherEntry.object;
      auto* collObj2 = planeEntry.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  // Either no collision found or not reached the maximum number of contacts
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

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  auto casted1 = static_cast<DARTCollisionGroup*>(group1);
  auto casted2 = static_cast<DARTCollisionGroup*>(group2);

  casted1->updateEngineData();
  casted2->updateEngineData();

  const auto& objects1 = casted1->mCollisionObjects;
  const auto& objects2 = casted2->mCollisionObjects;

  if (objects1.empty() || objects2.empty())
    return false;

  auto& scratch = getBroadphaseScratch();
  scratch.clear();
  if (result)
    scratch.contactPointIndex.prepare(option.maxNumContacts);
  buildBroadphaseEntries(
      objects1,
      scratch.finiteEntries1,
      scratch.planeEntries1,
      scratch.otherEntries1);
  buildBroadphaseEntries(
      objects2,
      scratch.finiteEntries2,
      scratch.planeEntries2,
      scratch.otherEntries2);

  auto collisionFound = false;
  if (processFinitePlanePairs(
          scratch.finiteEntries1,
          scratch.planeEntries2,
          option,
          result,
          collisionFound,
          scratch.pairResult,
          false)) {
    return true;
  }

  if (processFinitePlanePairs(
          scratch.finiteEntries2,
          scratch.planeEntries1,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  if (processFiniteFinitePairs(
          scratch.finiteEntries1,
          scratch.finiteEntries2,
          scratch.sortedEntries2,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  if (processPlanePlanePairs(
          scratch.planeEntries1,
          scratch.planeEntries2,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  const auto& filter = option.collisionFilter;
  for (const auto& entry1 : scratch.otherEntries1) {
    for (const auto& entry2 : scratch.otherEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }

    for (const auto& entry2 : scratch.finiteEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }

    for (const auto& entry2 : scratch.planeEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  for (const auto& entry1 : scratch.finiteEntries1) {
    for (const auto& entry2 : scratch.otherEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  for (const auto& entry1 : scratch.planeEntries1) {
    for (const auto& entry2 : scratch.otherEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  // Either no collision found or not reached the maximum number of contacts
  return collisionFound;
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[DARTCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[DARTCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
DARTCollisionDetector::DARTCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
void warnUnsupportedShapeType(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  const auto& shape = shapeFrame->getShape();
  const auto& shapeType = shape->getType();

  if (shapeType == dynamics::SphereShape::getStaticType())
    return;

  if (shapeType == dynamics::BoxShape::getStaticType())
    return;

  if (shapeType == dynamics::PlaneShape::getStaticType())
    return;

  if (shapeType == dynamics::CylinderShape::getStaticType())
    return;

  if (shapeType == dynamics::CapsuleShape::getStaticType())
    return;

  if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto& ellipsoid
        = std::static_pointer_cast<const dynamics::EllipsoidShape>(shape);

    if (ellipsoid->isSphere())
      return;
  }

  dterr << "[DARTCollisionDetector] Attempting to create shape type ["
        << shapeType << "] that is not supported "
        << "by DARTCollisionDetector. Currently, only SphereShape, BoxShape, "
        << "CylinderShape, CapsuleShape, PlaneShape, and EllipsoidShape (only "
        << "when all the radii are equal) are supported. This shape will "
           "always get "
        << "penetrated by other "
        << "objects.\n";
}

//==============================================================================
std::unique_ptr<CollisionObject> DARTCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  warnUnsupportedShapeType(shapeFrame);

  return std::unique_ptr<DARTCollisionObject>(
      new DARTCollisionObject(this, shapeFrame));
}

//==============================================================================
void DARTCollisionDetector::refreshCollisionObject(CollisionObject* /*object*/)
{
  // Do nothing
}

namespace {

//==============================================================================
bool operator==(const ContactPointKey& lhs, const ContactPointKey& rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

//==============================================================================
std::size_t hashContactPointKey(const ContactPointKey& key)
{
  auto mix = [](std::uint64_t value) {
    value ^= value >> 30;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31;
    return value;
  };

  auto hash = mix(static_cast<std::uint64_t>(key.x));
  hash ^= mix(
      static_cast<std::uint64_t>(key.y) + 0x9e3779b97f4a7c15ULL + (hash << 6)
      + (hash >> 2));
  hash ^= mix(
      static_cast<std::uint64_t>(key.z) + 0x9e3779b97f4a7c15ULL + (hash << 6)
      + (hash >> 2));

  return static_cast<std::size_t>(hash);
}

//==============================================================================
std::size_t nextPowerOfTwo(std::size_t value)
{
  std::size_t power = 1u;
  while (power < value
         && power <= std::numeric_limits<std::size_t>::max() / 2u) {
    power <<= 1u;
  }

  return power;
}

//==============================================================================
bool makeContactPointKey(const Eigen::Vector3d& point, ContactPointKey& key)
{
  for (auto i = 0; i < 3; ++i) {
    if (!std::isfinite(point[i]))
      return false;

    const auto cell = std::floor(point[i] / kContactDuplicateTolerance);
    if (cell < kContactPointKeyLowerBound
        || cell >= kContactPointKeyUpperBound) {
      return false;
    }

    if (i == 0)
      key.x = static_cast<std::int64_t>(cell);
    else if (i == 1)
      key.y = static_cast<std::int64_t>(cell);
    else
      key.z = static_cast<std::int64_t>(cell);
  }

  return true;
}

//==============================================================================
bool offsetCell(std::int64_t cell, int offset, std::int64_t& result)
{
  if (offset > 0) {
    if (cell > std::numeric_limits<std::int64_t>::max() - offset)
      return false;
  } else if (offset < 0) {
    if (cell < std::numeric_limits<std::int64_t>::min() - offset)
      return false;
  }

  result = cell + offset;
  return true;
}

//==============================================================================
void ContactPointIndex::clear()
{
  for (const auto bucketIndex : usedBuckets)
    buckets[bucketIndex] = ContactPointBucket();

  usedBuckets.clear();
  points.clear();
  nextPoint.clear();
}

//==============================================================================
void ContactPointIndex::prepare(std::size_t maxContacts)
{
  clear();

  const auto reserveContacts
      = std::max<std::size_t>(16u, std::min<std::size_t>(maxContacts, 65536u));
  points.reserve(reserveContacts);
  nextPoint.reserve(reserveContacts);
  usedBuckets.reserve(reserveContacts);
  ensureBucketCapacity(reserveContacts);
}

//==============================================================================
bool ContactPointIndex::containsClose(const Eigen::Vector3d& point) const
{
  ContactPointKey key;
  if (buckets.empty() || !makeContactPointKey(point, key))
    return containsCloseLinear(point);

  for (auto dx = -1; dx <= 1; ++dx) {
    ContactPointKey neighbor;
    if (!offsetCell(key.x, dx, neighbor.x))
      continue;

    for (auto dy = -1; dy <= 1; ++dy) {
      if (!offsetCell(key.y, dy, neighbor.y))
        continue;

      for (auto dz = -1; dz <= 1; ++dz) {
        if (!offsetCell(key.z, dz, neighbor.z))
          continue;

        const auto bucketIndex = findBucketIndex(neighbor);
        if (bucketIndex == kInvalidContactPointIndex)
          continue;

        for (auto pointIndex = buckets[bucketIndex].head;
             pointIndex != kInvalidContactPointIndex;
             pointIndex = nextPoint[pointIndex]) {
          if (isClose(point, points[pointIndex], kContactDuplicateTolerance))
            return true;
        }
      }
    }
  }

  return false;
}

//==============================================================================
void ContactPointIndex::add(const Eigen::Vector3d& point)
{
  ensureBucketCapacity(points.size() + 1u);

  const auto pointIndex = points.size();
  points.push_back(point);
  nextPoint.push_back(kInvalidContactPointIndex);

  ContactPointKey key;
  if (!makeContactPointKey(point, key))
    return;

  const auto bucketIndex = findOrCreateBucketIndex(key);
  if (bucketIndex == kInvalidContactPointIndex)
    return;

  nextPoint[pointIndex] = buckets[bucketIndex].head;
  buckets[bucketIndex].head = pointIndex;
}

//==============================================================================
bool ContactPointIndex::containsCloseLinear(const Eigen::Vector3d& point) const
{
  for (const auto& existingPoint : points) {
    if (isClose(point, existingPoint, kContactDuplicateTolerance))
      return true;
  }

  return false;
}

//==============================================================================
void ContactPointIndex::ensureBucketCapacity(std::size_t requiredPoints)
{
  const auto minBucketCount = 64u;
  auto desiredBucketCount = minBucketCount;
  if (requiredPoints <= std::numeric_limits<std::size_t>::max() / 4u) {
    desiredBucketCount
        = std::max<std::size_t>(minBucketCount, requiredPoints * 4u);
  }

  desiredBucketCount = nextPowerOfTwo(desiredBucketCount);
  if (buckets.size() >= desiredBucketCount)
    return;

  rehash(desiredBucketCount);
}

//==============================================================================
void ContactPointIndex::rehash(std::size_t bucketCount)
{
  buckets.assign(bucketCount, ContactPointBucket());
  usedBuckets.clear();
  bucketMask = buckets.empty() ? 0u : buckets.size() - 1u;

  for (auto i = 0u; i < points.size(); ++i) {
    nextPoint[i] = kInvalidContactPointIndex;

    ContactPointKey key;
    if (!makeContactPointKey(points[i], key))
      continue;

    const auto bucketIndex = findOrCreateBucketIndex(key);
    if (bucketIndex == kInvalidContactPointIndex)
      continue;

    nextPoint[i] = buckets[bucketIndex].head;
    buckets[bucketIndex].head = i;
  }
}

//==============================================================================
std::size_t ContactPointIndex::findBucketIndex(const ContactPointKey& key) const
{
  if (buckets.empty())
    return kInvalidContactPointIndex;

  auto bucketIndex = hashContactPointKey(key) & bucketMask;
  for (auto probe = 0u; probe < buckets.size(); ++probe) {
    const auto& bucket = buckets[bucketIndex];
    if (!bucket.occupied)
      return kInvalidContactPointIndex;

    if (bucket.key == key)
      return bucketIndex;

    bucketIndex = (bucketIndex + 1u) & bucketMask;
  }

  return kInvalidContactPointIndex;
}

//==============================================================================
std::size_t ContactPointIndex::findOrCreateBucketIndex(
    const ContactPointKey& key)
{
  if (buckets.empty())
    return kInvalidContactPointIndex;

  auto bucketIndex = hashContactPointKey(key) & bucketMask;
  for (auto probe = 0u; probe < buckets.size(); ++probe) {
    auto& bucket = buckets[bucketIndex];
    if (!bucket.occupied) {
      bucket.occupied = true;
      bucket.key = key;
      bucket.head = kInvalidContactPointIndex;
      usedBuckets.push_back(bucketIndex);
      return bucketIndex;
    }

    if (bucket.key == key)
      return bucketIndex;

    bucketIndex = (bucketIndex + 1u) & bucketMask;
  }

  return kInvalidContactPointIndex;
}

//==============================================================================
void BroadphaseScratch::clear()
{
  finiteEntries1.clear();
  planeEntries1.clear();
  otherEntries1.clear();
  finiteEntries2.clear();
  planeEntries2.clear();
  otherEntries2.clear();
  sortedEntries1.clear();
  sortedEntries2.clear();
  pairResult.clear();
  contactPointIndex.clear();
}

//==============================================================================
BroadphaseScratch& getBroadphaseScratch()
{
  thread_local BroadphaseScratch scratch;
  return scratch;
}

//==============================================================================
bool checkPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& pairResult,
    CollisionResult* result)
{
  pairResult.clear();

  // Perform narrow-phase detection
  collide(
      static_cast<DARTCollisionObject*>(o1),
      static_cast<DARTCollisionObject*>(o2),
      pairResult);

  // Early return for binary check
  if (!result)
    return pairResult.isCollision();

  postProcess(o1, o2, option, *result, pairResult);

  return pairResult.isCollision();
}

//==============================================================================
bool processPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  const auto pairCollision = checkPair(o1, o2, option, pairResult, result);
  collisionFound = collisionFound || pairCollision;
  return shouldStopAfterPair(pairCollision, option, result);
}

//==============================================================================
bool shouldStopAfterPair(
    bool pairCollision,
    const CollisionOption& option,
    const CollisionResult* result)
{
  if (!result)
    return pairCollision;

  return result->getNumContacts() >= option.maxNumContacts;
}

//==============================================================================
bool isClose(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, double tol)
{
  return (pos1 - pos2).norm() < tol;
}

//==============================================================================
void postProcess(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& totalResult,
    const CollisionResult& pairResult)
{
  if (!pairResult.isCollision())
    return;

  auto& contactPointIndex = getBroadphaseScratch().contactPointIndex;
  const auto maxContactsPerPair = option.getEffectiveMaxNumContactsPerPair();
  std::size_t pairContacts = 0u;
  for (const auto& pairContact : pairResult.getContacts()) {
    if (pairContacts >= maxContactsPerPair)
      break;

    // Don't add repeated points.
    if (contactPointIndex.containsClose(pairContact.point))
      continue;

    auto contact = pairContact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    totalResult.addContact(contact);
    contactPointIndex.add(contact.point);
    ++pairContacts;

    if (totalResult.getNumContacts() >= option.maxNumContacts)
      break;
  }
}

//==============================================================================
bool isPlaneShape(const CollisionObject* object)
{
  if (!object)
    return false;

  return static_cast<const DARTCollisionObject*>(object)->isCachedPlaneShape();
}

//==============================================================================
BroadphaseEntry makeBroadphaseEntry(CollisionObject* object)
{
  BroadphaseEntry entry;
  entry.object = object;
  entry.plane = isPlaneShape(object);

  if (!object || entry.plane)
    return entry;

  const auto* dartObject = static_cast<const DARTCollisionObject*>(object);
  if (!dartObject->getCachedShape())
    return entry;

  const auto& localMin = dartObject->getCachedLocalBoundsMin();
  const auto& localMax = dartObject->getCachedLocalBoundsMax();
  if (!dartObject->hasFiniteCachedLocalBounds())
    return entry;

  const auto& transform = object->getTransform();
  entry.min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  entry.max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::max());

  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        const Eigen::Vector3d localCorner(
            x ? localMax.x() : localMin.x(),
            y ? localMax.y() : localMin.y(),
            z ? localMax.z() : localMin.z());
        const Eigen::Vector3d worldCorner = transform * localCorner;
        entry.min = entry.min.cwiseMin(worldCorner);
        entry.max = entry.max.cwiseMax(worldCorner);
      }
    }
  }

  entry.finite = entry.min.allFinite() && entry.max.allFinite();
  return entry;
}

//==============================================================================
void buildBroadphaseEntries(
    const std::vector<CollisionObject*>& objects,
    std::vector<BroadphaseEntry>& finiteEntries,
    std::vector<BroadphaseEntry>& planeEntries,
    std::vector<BroadphaseEntry>& otherEntries)
{
  finiteEntries.reserve(objects.size());
  planeEntries.reserve(objects.size());
  otherEntries.reserve(objects.size());

  for (auto* object : objects) {
    auto entry = makeBroadphaseEntry(object);
    if (entry.finite)
      finiteEntries.push_back(entry);
    else if (entry.plane)
      planeEntries.push_back(entry);
    else
      otherEntries.push_back(entry);
  }
}

//==============================================================================
bool overlaps(const BroadphaseEntry& entry1, const BroadphaseEntry& entry2)
{
  if (!entry1.finite || !entry2.finite)
    return true;

  for (int axis = 0; axis < 3; ++axis) {
    if (entry1.max[axis] < entry2.min[axis]
        || entry2.max[axis] < entry1.min[axis]) {
      return false;
    }
  }

  return true;
}

//==============================================================================
bool processFinitePlanePairs(
    const std::vector<BroadphaseEntry>& finiteEntries,
    const std::vector<BroadphaseEntry>& planeEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult,
    bool planeFirst)
{
  const auto& filter = option.collisionFilter;
  for (const auto& planeEntry : planeEntries) {
    for (const auto& finiteEntry : finiteEntries) {
      auto* collObj1 = planeFirst ? planeEntry.object : finiteEntry.object;
      auto* collObj2 = planeFirst ? finiteEntry.object : planeEntry.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

//==============================================================================
bool processPlanePlanePairs(
    const std::vector<BroadphaseEntry>& planeEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  const auto& filter = option.collisionFilter;
  for (std::size_t i = 0u; i + 1u < planeEntries.size(); ++i) {
    for (std::size_t j = i + 1u; j < planeEntries.size(); ++j) {
      auto* collObj1 = planeEntries[i].object;
      auto* collObj2 = planeEntries[j].object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

//==============================================================================
bool processPlanePlanePairs(
    const std::vector<BroadphaseEntry>& planeEntries1,
    const std::vector<BroadphaseEntry>& planeEntries2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  const auto& filter = option.collisionFilter;
  for (const auto& entry1 : planeEntries1) {
    for (const auto& entry2 : planeEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

//==============================================================================
bool processFiniteFinitePairs(
    const std::vector<BroadphaseEntry>& entries,
    std::vector<const BroadphaseEntry*>& sortedEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  sortedEntries.reserve(entries.size());
  for (const auto& entry : entries)
    sortedEntries.push_back(&entry);

  std::sort(
      sortedEntries.begin(),
      sortedEntries.end(),
      [](const BroadphaseEntry* lhs, const BroadphaseEntry* rhs) {
        return lhs->min.x() < rhs->min.x();
      });

  const auto& filter = option.collisionFilter;
  for (std::size_t i = 0u; i + 1u < sortedEntries.size(); ++i) {
    const auto& entry1 = *sortedEntries[i];
    for (std::size_t j = i + 1u; j < sortedEntries.size(); ++j) {
      const auto& entry2 = *sortedEntries[j];
      if (entry2.min.x() > entry1.max.x())
        break;

      if (!overlaps(entry1, entry2))
        continue;

      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

//==============================================================================
bool processFiniteFinitePairs(
    const std::vector<BroadphaseEntry>& entries1,
    const std::vector<BroadphaseEntry>& entries2,
    std::vector<const BroadphaseEntry*>& sortedEntries2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  sortedEntries2.reserve(entries2.size());
  for (const auto& entry : entries2)
    sortedEntries2.push_back(&entry);

  std::sort(
      sortedEntries2.begin(),
      sortedEntries2.end(),
      [](const BroadphaseEntry* lhs, const BroadphaseEntry* rhs) {
        return lhs->min.x() < rhs->min.x();
      });

  const auto& filter = option.collisionFilter;
  for (const auto& entry1 : entries1) {
    for (const auto* entry2Ptr : sortedEntries2) {
      const auto& entry2 = *entry2Ptr;
      if (entry2.max.x() < entry1.min.x())
        continue;
      if (entry2.min.x() > entry1.max.x())
        break;
      if (!overlaps(entry1, entry2))
        continue;

      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
