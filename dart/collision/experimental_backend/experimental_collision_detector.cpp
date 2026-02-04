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
#include "dart/collision/experimental/narrow_phase/narrow_phase.hpp"
#include "dart/collision/experimental_backend/experimental_collision_group.hpp"
#include "dart/collision/experimental_backend/experimental_collision_object.hpp"
#include "dart/collision/experimental_backend/shape_adapter.hpp"
#include "dart/common/logging.hpp"
#include "dart/dynamics/shape_frame.hpp"

#include <Eigen/Dense>

namespace dart {
namespace collision {

namespace {

bool checkPair(
    ExperimentalCollisionObject* o1,
    ExperimentalCollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result = nullptr);

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
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  DART_WARN(
      "This collision detector does not support (signed) distance queries. "
      "Returning 0.0.");

  return 0.0;
}

//==============================================================================
double ExperimentalCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  DART_WARN(
      "This collision detector does not support (signed) distance queries. "
      "Returning 0.0.");

  return 0.0;
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
