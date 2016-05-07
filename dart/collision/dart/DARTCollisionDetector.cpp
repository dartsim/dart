/*
 * Copyright (c) 2013-2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/dart/DARTCollide.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/dart/DARTCollisionGroup.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"

namespace dart {
namespace collision {

namespace {

bool checkPair(CollisionObject* o1, CollisionObject* o2,
               const CollisionOption& option, CollisionResult* result = nullptr);

bool isClose(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2,
             double tol);

void postProcess(CollisionObject* o1, CollisionObject* o2, const CollisionOption& option,
                 CollisionResult& totalResult, const CollisionResult& pairResult);

} // anonymous namespace



//==============================================================================
std::shared_ptr<DARTCollisionDetector> DARTCollisionDetector::create()
{
  return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
}

//==============================================================================
std::shared_ptr<CollisionDetector>
DARTCollisionDetector::cloneWithoutCollisionObjects()
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
std::unique_ptr<CollisionGroup>
DARTCollisionDetector::createCollisionGroup()
{
  return common::make_unique<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
static void clearCollisionResult(
    const CollisionOption& option, CollisionResult* result)
{
  if (!result)
  {
    if (!option.binaryCheck)
    {
      dtwarn << "[DARTCollisionDetector::collide] nullptr of CollisionResult is "
             << "passed in when the binary check option is off. The collision "
             << "detector will run narrowphase collision checking over all the "
             << "feasible collision pairs without storing the contact "
             << "information, which would be meaningless work. Also, it would "
             << "ignore the maximum number of contacts since the number of "
             << "contact will not be counted. Please consider either of using "
             << "binary check or passsing CollisionResult pointer which is not "
             << "a nullptr.\n";
    }

    return;
  }

  result->clear();
}

//==============================================================================
static bool checkGroupValidity(DARTCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get())
  {
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
  clearCollisionResult(option, result);

  if (!checkGroupValidity(this, group))
    return false;

  const auto& binaryCheck = option.binaryCheck;

  auto casted = static_cast<DARTCollisionGroup*>(group);
  const auto& objects = casted->mCollisionObjects;

  if (objects.empty())
    return false;

  const auto& filter = option.collisionFilter;

  for (auto i = 0u; i < objects.size() - 1; ++i)
  {
    auto* collObj1 = objects[i];

    for (auto j = i + 1u; j < objects.size(); ++j)
    {
      auto* collObj2 = objects[j];

      if (filter && !filter->needCollision(collObj1, collObj2))
        continue;

      auto collisionFound = checkPair(collObj1, collObj2, option, result);

      if (binaryCheck && collisionFound)
        return true;

      if (result && result->getNumContacts() >= option.maxNumContacts)
        return result->isCollision();
    }
  }

  if (binaryCheck)
    return false;
  else
    return result->isCollision();
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  clearCollisionResult(option, result);

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  const auto& binaryCheck = option.binaryCheck;

  auto casted1 = static_cast<DARTCollisionGroup*>(group1);
  auto casted2 = static_cast<DARTCollisionGroup*>(group2);

  const auto& objects1 = casted1->mCollisionObjects;
  const auto& objects2 = casted2->mCollisionObjects;

  if (objects1.empty() || objects2.empty())
    return false;

  const auto& filter = option.collisionFilter;

  for (auto i = 0u; i < objects1.size(); ++i)
  {
    auto* collObj1 = objects1[i];

    for (auto j = 0u; j < objects2.size(); ++j)
    {
      auto* collObj2 = objects2[j];

      if (filter && !filter->needCollision(collObj1, collObj2))
        continue;

      auto collisionFound = checkPair(collObj1, collObj2, option, result);

      if (binaryCheck && collisionFound)
        return true;

      if (result && result->getNumContacts() >= option.maxNumContacts)
        return result->isCollision();
    }
  }

  if (binaryCheck)
    return false;
  else
    return result->isCollision();
}

//==============================================================================
DARTCollisionDetector::DARTCollisionDetector()
  : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
void warnUnsupportedShapeType(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  const auto& shape = shapeFrame->getShape();

  if (shape->getShapeType() == dynamics::Shape::BOX)
    return;

  if (shape->getShapeType() == dynamics::Shape::ELLIPSOID)
  {
    const auto& ellipsoid
        = std::static_pointer_cast<const dynamics::EllipsoidShape>(shape);
    if (ellipsoid->isSphere())
      return;
  }

  dterr << "[DARTCollisionDetector] Attempting to create shape type '"
        << shapeFrame->getShape()->getShapeType() << "' that is not supported "
        << "by DARTCollisionDetector. Currently, only BoxShape and "
        << "EllipsoidShape (only when all the radii are equal) are "
        << "supported. This shape will always get penetrated by other "
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




namespace {

//==============================================================================
bool checkPair(CollisionObject* o1, CollisionObject* o2,
               const CollisionOption& option, CollisionResult* result)
{
  CollisionResult pairResult;

  // Perform narrow-phase detection
  collide(o1, o2, pairResult);

  // Early return for binary check
  if (!result)
    return pairResult.isCollision();

  postProcess(o1, o2, option, *result, pairResult);

  return pairResult.isCollision();
}

//==============================================================================
bool isClose(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2,
             double tol)
{
  return (pos1 - pos2).norm() < tol;
}

//==============================================================================
void postProcess(CollisionObject* o1, CollisionObject* o2,
                 const CollisionOption& option,
                 CollisionResult& totalResult, const CollisionResult& pairResult)
{
  if (!pairResult.isCollision())
    return;

  // Don't add repeated points
  const auto tol = 3.0e-12;

  for (auto pairContact : pairResult.getContacts())
  {
    auto foundClose = false;

    for (auto totalContact : totalResult.getContacts())
    {
      if (isClose(pairContact.point, totalContact.point, tol))
      {
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

    if (option.binaryCheck)
      break;

    if (totalResult.getNumContacts() >= option.maxNumContacts)
      break;
  }
}

} // anonymous namespace

} // namespace collision
} // namespace dart
