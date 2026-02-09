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

#include "dart/collision/experimental_backend/experimental_collision_detector.hpp"

#include "dart/collision/collision_filter.hpp"
#include "dart/collision/collision_object.hpp"
#include "dart/collision/contact.hpp"
#include "dart/collision/distance_filter.hpp"
#include "dart/collision/experimental/collision_world.hpp"
#include "dart/collision/experimental/narrow_phase/distance.hpp"
#include "dart/collision/experimental/narrow_phase/narrow_phase.hpp"
#include "dart/collision/experimental/narrow_phase/raycast.hpp"
#include "dart/collision/experimental_backend/experimental_collision_group.hpp"
#include "dart/collision/experimental_backend/experimental_collision_object.hpp"
#include "dart/collision/experimental_backend/shape_adapter.hpp"
#include "dart/common/logging.hpp"
#include "dart/dynamics/shape_frame.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <memory>

namespace dart {
namespace collision {

namespace {

bool checkPair(
    ExperimentalCollisionObject* o1,
    ExperimentalCollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result = nullptr);

std::unique_ptr<experimental::Shape> cloneShape(
    const experimental::Shape* shape);

double distancePair(
    ExperimentalCollisionObject* o1,
    ExperimentalCollisionObject* o2,
    const DistanceOption& option,
    experimental::DistanceResult* result);

bool raycastShape(
    const experimental::Ray& ray,
    const experimental::Shape* shape,
    const Eigen::Isometry3d& transform,
    const experimental::RaycastOption& option,
    experimental::RaycastResult& result);

bool isClose(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, double tol);

void postProcess(
    ExperimentalCollisionObject* o1,
    ExperimentalCollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& totalResult,
    bool collisionFound,
    const experimental::CollisionResult& pairResult);

} // namespace

//==============================================================================
ExperimentalCollisionDetector::Registrar<ExperimentalCollisionDetector>
    ExperimentalCollisionDetector::mRegistrar{
        std::string(ExperimentalCollisionDetector::getStaticType()),
        []()
            -> std::shared_ptr<dart::collision::ExperimentalCollisionDetector> {
          return dart::collision::ExperimentalCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<ExperimentalCollisionDetector>
ExperimentalCollisionDetector::create()
{
  return std::shared_ptr<ExperimentalCollisionDetector>(
      new ExperimentalCollisionDetector());
}

//==============================================================================
std::shared_ptr<CollisionDetector>
ExperimentalCollisionDetector::cloneWithoutCollisionObjects() const
{
  return ExperimentalCollisionDetector::create();
}

//==============================================================================
const std::string& ExperimentalCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ExperimentalCollisionDetector::getStaticType()
{
  static const std::string type = "experimental";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup>
ExperimentalCollisionDetector::createCollisionGroup()
{
  return std::make_unique<ExperimentalCollisionGroup>(shared_from_this());
}

//==============================================================================
static bool checkGroupValidity(
    ExperimentalCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get()) {
    DART_ERROR(
        "Attempting to check collision for a collision group that is created "
        "from a different collision detector instance.");

    return false;
  }

  return true;
}

//==============================================================================
bool ExperimentalCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result) {
    result->clear();
  }

  if (0u == option.maxNumContacts) [[unlikely]] {
    DART_WARN(
        "CollisionOption::maxNumContacts is 0; skipping collision detection. "
        "Use maxNumContacts >= 1 for binary checks.");
    return false;
  }

  if (!checkGroupValidity(this, group)) {
    return false;
  }

  auto casted = static_cast<ExperimentalCollisionGroup*>(group);
  const auto& objects = casted->mCollisionObjects;

  if (objects.empty()) [[unlikely]] {
    return false;
  }

  auto collisionFound = false;
  const auto& filter = option.collisionFilter;

  for (auto i = 0u; i < objects.size() - 1; ++i) {
    auto* collObj1 = static_cast<ExperimentalCollisionObject*>(objects[i]);

    for (auto j = i + 1u; j < objects.size(); ++j) {
      auto* collObj2 = static_cast<ExperimentalCollisionObject*>(objects[j]);

      if (filter && filter->ignoresCollision(collObj1, collObj2)) [[unlikely]] {
        continue;
      }

      collisionFound = checkPair(collObj1, collObj2, option, result);

      if (result) {
        if (result->getNumContacts() >= option.maxNumContacts) [[unlikely]] {
          return true;
        }
      } else {
        // If no result is passed, stop checking when the first contact is found
        if (collisionFound) [[unlikely]] {
          return true;
        }
      }
    }
  }

  // Either no collision found or not reached the maximum number of contacts
  return collisionFound;
}

//==============================================================================
bool ExperimentalCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
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

  if (!checkGroupValidity(this, group1)) {
    return false;
  }

  if (!checkGroupValidity(this, group2)) {
    return false;
  }

  auto casted1 = static_cast<ExperimentalCollisionGroup*>(group1);
  auto casted2 = static_cast<ExperimentalCollisionGroup*>(group2);

  const auto& objects1 = casted1->mCollisionObjects;
  const auto& objects2 = casted2->mCollisionObjects;

  if (objects1.empty() || objects2.empty()) [[unlikely]] {
    return false;
  }

  auto collisionFound = false;
  const auto& filter = option.collisionFilter;

  for (auto i = 0u; i < objects1.size(); ++i) {
    auto* collObj1 = static_cast<ExperimentalCollisionObject*>(objects1[i]);

    for (auto j = 0u; j < objects2.size(); ++j) {
      auto* collObj2 = static_cast<ExperimentalCollisionObject*>(objects2[j]);

      if (filter && filter->ignoresCollision(collObj1, collObj2)) {
        continue;
      }

      collisionFound = checkPair(collObj1, collObj2, option, result);

      if (result) {
        if (result->getNumContacts() >= option.maxNumContacts) {
          return true;
        }
      } else {
        // If no result is passed, stop checking when the first contact is found
        if (collisionFound) {
          return true;
        }
      }
    }
  }

  // Either no collision found or not reached the maximum number of contacts
  return collisionFound;
}

//==============================================================================
double ExperimentalCollisionDetector::distance(
    CollisionGroup* group, const DistanceOption& option, DistanceResult* result)
{
  if (result) {
    result->clear();
  }

  if (!checkGroupValidity(this, group)) {
    return 0.0;
  }

  const auto* casted = static_cast<ExperimentalCollisionGroup*>(group);
  const auto& objects = casted->mCollisionObjects;
  if (objects.size() < 2u) {
    return std::max(0.0, option.distanceLowerBound);
  }

  const auto& filter = option.distanceFilter;
  double bestDistance = std::numeric_limits<double>::max();
  experimental::DistanceResult bestExpResult;
  const CollisionObject* bestObj1 = nullptr;
  const CollisionObject* bestObj2 = nullptr;
  bool found = false;

  auto shouldStop = [&]() {
    return found && bestDistance <= option.distanceLowerBound;
  };

  for (auto i = 0u; i + 1u < objects.size(); ++i) {
    auto* collObj1 = static_cast<ExperimentalCollisionObject*>(objects[i]);
    for (auto j = i + 1u; j < objects.size(); ++j) {
      auto* collObj2 = static_cast<ExperimentalCollisionObject*>(objects[j]);

      if (filter && !filter->needDistance(collObj1, collObj2)) {
        continue;
      }

      experimental::DistanceResult pairResult;
      const double distance
          = distancePair(collObj1, collObj2, option, &pairResult);
      if (distance < bestDistance) {
        bestDistance = distance;
        bestExpResult = pairResult;
        bestObj1 = collObj1;
        bestObj2 = collObj2;
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
    result->shapeFrame1 = bestObj1 ? bestObj1->getShapeFrame() : nullptr;
    result->shapeFrame2 = bestObj2 ? bestObj2->getShapeFrame() : nullptr;
    result->nearestPoint1 = bestExpResult.pointOnObject1;
    result->nearestPoint2 = bestExpResult.pointOnObject2;
  }

  return std::max(bestDistance, option.distanceLowerBound);
}

//==============================================================================
double ExperimentalCollisionDetector::distance(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (result) {
    result->clear();
  }

  if (!checkGroupValidity(this, group1)) {
    return 0.0;
  }

  if (!checkGroupValidity(this, group2)) {
    return 0.0;
  }

  const auto* casted1 = static_cast<ExperimentalCollisionGroup*>(group1);
  const auto* casted2 = static_cast<ExperimentalCollisionGroup*>(group2);
  const auto& objects1 = casted1->mCollisionObjects;
  const auto& objects2 = casted2->mCollisionObjects;

  if (objects1.empty() || objects2.empty()) {
    return std::max(0.0, option.distanceLowerBound);
  }

  const auto& filter = option.distanceFilter;
  double bestDistance = std::numeric_limits<double>::max();
  experimental::DistanceResult bestExpResult;
  const CollisionObject* bestObj1 = nullptr;
  const CollisionObject* bestObj2 = nullptr;
  bool found = false;

  auto shouldStop = [&]() {
    return found && bestDistance <= option.distanceLowerBound;
  };

  for (auto* object1 : objects1) {
    auto* collObj1 = static_cast<ExperimentalCollisionObject*>(object1);
    for (auto* object2 : objects2) {
      auto* collObj2 = static_cast<ExperimentalCollisionObject*>(object2);

      if (filter && !filter->needDistance(collObj1, collObj2)) {
        continue;
      }

      experimental::DistanceResult pairResult;
      const double distance
          = distancePair(collObj1, collObj2, option, &pairResult);
      if (distance < bestDistance) {
        bestDistance = distance;
        bestExpResult = pairResult;
        bestObj1 = collObj1;
        bestObj2 = collObj2;
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
    result->shapeFrame1 = bestObj1 ? bestObj1->getShapeFrame() : nullptr;
    result->shapeFrame2 = bestObj2 ? bestObj2->getShapeFrame() : nullptr;
    result->nearestPoint1 = bestExpResult.pointOnObject1;
    result->nearestPoint2 = bestExpResult.pointOnObject2;
  }

  return std::max(bestDistance, option.distanceLowerBound);
}

//==============================================================================
bool ExperimentalCollisionDetector::raycast(
    CollisionGroup* group,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (result) {
    result->clear();
  }

  if (!checkGroupValidity(this, group)) {
    return false;
  }

  auto* castedGroup = static_cast<ExperimentalCollisionGroup*>(group);
  const auto& objects = castedGroup->mCollisionObjects;
  if (objects.empty()) {
    return false;
  }

  const Eigen::Vector3d delta = to - from;
  const double totalLength = delta.norm();
  if (totalLength <= 0.0) {
    return false;
  }

  const experimental::Ray ray(from, delta, totalLength);
  const experimental::RaycastOption expOption
      = experimental::RaycastOption::unlimited();

  std::vector<RayHit> hits;
  hits.reserve(objects.size());
  double closestFraction = std::numeric_limits<double>::max();
  RayHit closestHit;
  bool foundHit = false;

  for (auto* object : objects) {
    auto* collObj = static_cast<ExperimentalCollisionObject*>(object);

    if (!option.passesFilter(collObj)) {
      continue;
    }

    experimental::RaycastResult expResult;
    if (!raycastShape(
            ray,
            collObj->getExperimentalShape(),
            collObj->getTransform(),
            expOption,
            expResult)) {
      continue;
    }

    foundHit = true;
    RayHit hit;
    hit.mCollisionObject = collObj;
    hit.mPoint = expResult.point;
    hit.mNormal = expResult.normal;
    hit.mFraction = expResult.distance / totalLength;

    if (option.mEnableAllHits) {
      hits.emplace_back(hit);
    } else if (hit.mFraction < closestFraction) {
      closestFraction = hit.mFraction;
      closestHit = hit;
    }
  }

  if (!foundHit) {
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
    result->mRayHits.emplace_back(closestHit);
  }

  return result->hasHit();
}

//==============================================================================
ExperimentalCollisionDetector::ExperimentalCollisionDetector()
  : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
std::unique_ptr<CollisionObject>
ExperimentalCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  auto shape = adaptShape(shapeFrame ? shapeFrame->getShape() : nullptr);
  return std::unique_ptr<ExperimentalCollisionObject>(
      new ExperimentalCollisionObject(this, shapeFrame, std::move(shape)));
}

//==============================================================================
void ExperimentalCollisionDetector::refreshCollisionObject(
    CollisionObject* object)
{
  if (!object) {
    return;
  }

  auto* casted = static_cast<ExperimentalCollisionObject*>(object);
  const auto* shapeFrame = casted->getShapeFrame();
  if (!shapeFrame) {
    return;
  }

  casted->setExperimentalShape(adaptShape(shapeFrame->getShape()));
}

namespace {

//==============================================================================
bool checkPair(
    ExperimentalCollisionObject* o1,
    ExperimentalCollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (!o1 || !o2) {
    return false;
  }

  const auto* shape1 = o1->getExperimentalShape();
  const auto* shape2 = o2->getExperimentalShape();
  if (!shape1 || !shape2) {
    return false;
  }

  experimental::CollisionResult pairResult;
  experimental::CollisionOption expOption;
  expOption.enableContact = option.enableContact;
  expOption.maxNumContacts = option.maxNumContacts;
  expOption.collisionFilter = nullptr;

  const bool collisionFound = experimental::NarrowPhase::collide(
      shape1,
      o1->getTransform(),
      shape2,
      o2->getTransform(),
      expOption,
      pairResult);

  // Early return for binary check
  if (!result) {
    return collisionFound;
  }

  postProcess(o1, o2, option, *result, collisionFound, pairResult);

  return collisionFound;
}

//==============================================================================
std::unique_ptr<experimental::Shape> cloneShape(
    const experimental::Shape* shape)
{
  if (!shape) {
    return nullptr;
  }

  switch (shape->getType()) {
    case experimental::ShapeType::Sphere: {
      const auto* sphere = static_cast<const experimental::SphereShape*>(shape);
      return std::make_unique<experimental::SphereShape>(sphere->getRadius());
    }
    case experimental::ShapeType::Box: {
      const auto* box = static_cast<const experimental::BoxShape*>(shape);
      return std::make_unique<experimental::BoxShape>(box->getHalfExtents());
    }
    case experimental::ShapeType::Capsule: {
      const auto* capsule
          = static_cast<const experimental::CapsuleShape*>(shape);
      return std::make_unique<experimental::CapsuleShape>(
          capsule->getRadius(), capsule->getHeight());
    }
    case experimental::ShapeType::Cylinder: {
      const auto* cylinder
          = static_cast<const experimental::CylinderShape*>(shape);
      return std::make_unique<experimental::CylinderShape>(
          cylinder->getRadius(), cylinder->getHeight());
    }
    case experimental::ShapeType::Plane: {
      const auto* plane = static_cast<const experimental::PlaneShape*>(shape);
      return std::make_unique<experimental::PlaneShape>(
          plane->getNormal(), plane->getOffset());
    }
    case experimental::ShapeType::Convex: {
      const auto* convex = static_cast<const experimental::ConvexShape*>(shape);
      return std::make_unique<experimental::ConvexShape>(convex->getVertices());
    }
    case experimental::ShapeType::Mesh: {
      const auto* mesh = static_cast<const experimental::MeshShape*>(shape);
      return std::make_unique<experimental::MeshShape>(
          mesh->getVertices(), mesh->getTriangles());
    }
    case experimental::ShapeType::Sdf: {
      const auto* sdf = static_cast<const experimental::SdfShape*>(shape);
      const auto* field = sdf->getField();
      if (!field) {
        return nullptr;
      }
      std::shared_ptr<const experimental::SignedDistanceField> fieldRef(
          field, [](const experimental::SignedDistanceField*) {});
      return std::make_unique<experimental::SdfShape>(std::move(fieldRef));
    }
    case experimental::ShapeType::Compound: {
      const auto* compound
          = static_cast<const experimental::CompoundShape*>(shape);
      auto clone = std::make_unique<experimental::CompoundShape>();
      for (const auto& child : compound->children()) {
        clone->addChild(cloneShape(child.shape.get()), child.localTransform);
      }
      return clone;
    }
    default:
      return nullptr;
  }
}

//==============================================================================
double distancePair(
    ExperimentalCollisionObject* o1,
    ExperimentalCollisionObject* o2,
    const DistanceOption& option,
    experimental::DistanceResult* result)
{
  if (!o1 || !o2) {
    return std::numeric_limits<double>::max();
  }

  const auto* shape1 = o1->getExperimentalShape();
  const auto* shape2 = o2->getExperimentalShape();
  if (!shape1 || !shape2) {
    return std::numeric_limits<double>::max();
  }

  auto shape1Clone = cloneShape(shape1);
  auto shape2Clone = cloneShape(shape2);
  if (!shape1Clone || !shape2Clone) {
    return std::numeric_limits<double>::max();
  }

  experimental::CollisionWorld world(experimental::BroadPhaseType::BruteForce);
  auto expObj1 = world.createObject(std::move(shape1Clone), o1->getTransform());
  auto expObj2 = world.createObject(std::move(shape2Clone), o2->getTransform());

  experimental::DistanceOption expOption;
  expOption.upperBound = std::numeric_limits<double>::max();
  expOption.enableNearestPoints = option.enableNearestPoints;

  experimental::DistanceResult pairResult;
  const auto distance = experimental::NarrowPhase::distance(
      expObj1, expObj2, expOption, pairResult);

  if (result) {
    *result = pairResult;
  }

  return distance;
}

//==============================================================================
bool raycastShape(
    const experimental::Ray& ray,
    const experimental::Shape* shape,
    const Eigen::Isometry3d& transform,
    const experimental::RaycastOption& option,
    experimental::RaycastResult& result)
{
  result.clear();

  if (!shape) {
    return false;
  }

  switch (shape->getType()) {
    case experimental::ShapeType::Sphere: {
      const auto* sphere = static_cast<const experimental::SphereShape*>(shape);
      return experimental::raycastSphere(
          ray, *sphere, transform, option, result);
    }
    case experimental::ShapeType::Box: {
      const auto* box = static_cast<const experimental::BoxShape*>(shape);
      return experimental::raycastBox(ray, *box, transform, option, result);
    }
    case experimental::ShapeType::Capsule: {
      const auto* capsule
          = static_cast<const experimental::CapsuleShape*>(shape);
      return experimental::raycastCapsule(
          ray, *capsule, transform, option, result);
    }
    case experimental::ShapeType::Cylinder: {
      const auto* cylinder
          = static_cast<const experimental::CylinderShape*>(shape);
      return experimental::raycastCylinder(
          ray, *cylinder, transform, option, result);
    }
    case experimental::ShapeType::Plane: {
      const auto* plane = static_cast<const experimental::PlaneShape*>(shape);
      return experimental::raycastPlane(ray, *plane, transform, option, result);
    }
    case experimental::ShapeType::Mesh: {
      const auto* mesh = static_cast<const experimental::MeshShape*>(shape);
      return experimental::raycastMesh(ray, *mesh, transform, option, result);
    }
    case experimental::ShapeType::Convex: {
      const auto* convex = static_cast<const experimental::ConvexShape*>(shape);
      return experimental::raycastConvex(
          ray, *convex, transform, option, result);
    }
    case experimental::ShapeType::Compound: {
      const auto* compound
          = static_cast<const experimental::CompoundShape*>(shape);
      bool hit = false;
      double bestDistance = std::min(ray.maxDistance, option.maxDistance);
      experimental::RaycastResult bestResult;

      for (const auto& child : compound->children()) {
        if (!child.shape) {
          continue;
        }

        experimental::RaycastOption childOption = option;
        childOption.maxDistance = bestDistance;
        experimental::RaycastResult childResult;
        if (raycastShape(
                ray,
                child.shape.get(),
                transform * child.localTransform,
                childOption,
                childResult)) {
          if (childResult.distance < bestDistance) {
            bestDistance = childResult.distance;
            bestResult = childResult;
          }
          hit = true;
        }
      }

      if (hit) {
        result = bestResult;
      }
      return hit;
    }
    default:
      return false;
  }
}

//==============================================================================
bool isClose(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, double tol)
{
  return (pos1 - pos2).norm() < tol;
}

//==============================================================================
void postProcess(
    ExperimentalCollisionObject* o1,
    ExperimentalCollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& totalResult,
    bool collisionFound,
    const experimental::CollisionResult& pairResult)
{
  if (!collisionFound) [[likely]] {
    return;
  }

  if (!option.enableContact) {
    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    totalResult.addContact(contact);
    return;
  }

  // Don't add repeated points
  const auto tol = 3.0e-12;

  const auto numContacts = pairResult.numContacts();
  for (auto i = 0u; i < numContacts; ++i) {
    const auto& cp = pairResult.getContact(i);
    if (!option.allowNegativePenetrationDepthContacts && cp.depth < 0.0) {
      continue;
    }

    Contact contact;
    contact.point = cp.position;
    contact.normal = cp.normal;
    if (Contact::isZeroNormal(contact.normal)) {
      continue;
    }
    contact.penetrationDepth = cp.depth;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;

    auto foundClose = false;
    for (const auto& totalContact : totalResult.getContacts()) {
      if (isClose(contact.point, totalContact.point, tol)) {
        foundClose = true;
        break;
      }
    }

    if (foundClose) {
      continue;
    }

    totalResult.addContact(contact);

    if (totalResult.getNumContacts() >= option.maxNumContacts) {
      break;
    }
  }
}

} // namespace

} // namespace collision
} // namespace dart
