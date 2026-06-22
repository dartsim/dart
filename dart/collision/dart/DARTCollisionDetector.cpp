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
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <algorithm>
#include <limits>
#include <vector>

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
  const auto& objects = casted->mCollisionObjects;

  if (objects.empty())
    return false;

  auto& scratch = getBroadphaseScratch();
  scratch.clear();
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

  const auto& objects1 = casted1->mCollisionObjects;
  const auto& objects2 = casted2->mCollisionObjects;

  if (objects1.empty() || objects2.empty())
    return false;

  auto& scratch = getBroadphaseScratch();
  scratch.clear();
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

  if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto& ellipsoid
        = std::static_pointer_cast<const dynamics::EllipsoidShape>(shape);

    if (ellipsoid->isSphere())
      return;
  }

  dterr << "[DARTCollisionDetector] Attempting to create shape type ["
        << shapeType << "] that is not supported "
        << "by DARTCollisionDetector. Currently, only SphereShape, BoxShape, "
        << "CylinderShape, PlaneShape, and EllipsoidShape (only when all the "
        << "radii are equal) are supported. This shape will always get "
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
  collide(o1, o2, pairResult);

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

  // Don't add repeated points
  const auto tol = 3.0e-12;

  const auto maxContactsPerPair = option.getEffectiveMaxNumContactsPerPair();
  std::size_t pairContacts = 0u;
  for (auto pairContact : pairResult.getContacts()) {
    if (pairContacts >= maxContactsPerPair)
      break;

    auto foundClose = false;

    for (auto totalContact : totalResult.getContacts()) {
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

  const auto& shape = object->getShape();
  return shape && shape->getType() == dynamics::PlaneShape::getStaticType();
}

//==============================================================================
BroadphaseEntry makeBroadphaseEntry(CollisionObject* object)
{
  BroadphaseEntry entry;
  entry.object = object;
  entry.plane = isPlaneShape(object);

  if (!object || entry.plane)
    return entry;

  const auto& shape = object->getShape();
  if (!shape)
    return entry;

  const auto& localBox = shape->getBoundingBox();
  const auto& transform = object->getTransform();
  const auto& localMin = localBox.getMin();
  const auto& localMax = localBox.getMax();

  if (!localMin.allFinite() || !localMax.allFinite())
    return entry;

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
