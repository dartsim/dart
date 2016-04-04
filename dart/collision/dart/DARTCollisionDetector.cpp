/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include "dart/collision/dart/DARTCollisionDetector.h"

#include "dart/collision/CollisionObject.h"
#include "dart/collision/CollisionFilter.h"
#include "dart/collision/dart/DARTCollide.h"
#include "dart/collision/dart/DARTCollisionObject.h"
#include "dart/collision/dart/DARTCollisionGroup.h"
#include "dart/dynamics/ShapeFrame.h"
#include "dart/dynamics/EllipsoidShape.h"

namespace dart {
namespace collision {

namespace {

bool checkPair(CollisionObject* o1, CollisionObject* o2,
               const Option& option, Result& result);

bool isClose(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2,
             double tol);

void postProcess(CollisionObject* o1, CollisionObject* o2, const Option& option,
                 Result& totalResult, const Result& pairResult);

} // anonymous namespace



//==============================================================================
std::shared_ptr<DARTCollisionDetector> DARTCollisionDetector::create()
{
  return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
}

//==============================================================================
const std::string& DARTCollisionDetector::getTypeStatic()
{
  static const std::string& type("DART");
  return type;
}

//==============================================================================
const std::string& DARTCollisionDetector::getType() const
{
  return getTypeStatic();
}

//==============================================================================
std::unique_ptr<CollisionGroup>
DARTCollisionDetector::createCollisionGroup()
{
  return common::make_unique<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
bool DARTCollisionDetector::detect(
    CollisionGroup* group,
    const Option& option, Result& result)
{
  result.clear();

  assert(group);
  assert(group->getCollisionDetector()->getType()
         == DARTCollisionDetector::getTypeStatic());

  auto objects = group->getCollisionObjects();

  if (objects.empty())
    return false;

  auto done = false;
  const auto& filter = option.collisionFilter;

  for (auto i = 0u; i < objects.size() - 1; ++i)
  {
    auto collObj1 = objects[i];

    for (auto j = i + 1u; j < objects.size(); ++j)
    {
      auto collObj2 = objects[j];

      if (filter && !filter->needCollision(collObj1.get(), collObj2.get()))
        continue;

      checkPair(collObj1.get(), collObj2.get(), option, result);

      if ((option.binaryCheck && result.isCollision())
          || (result.getNumContacts() >= option.maxNumContacts))
      {
        done = true;
        break;
      }
    }

    if (done)
      break;
  }

  return result.isCollision();
}

//==============================================================================
bool DARTCollisionDetector::detect(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const Option& option, Result& result)
{
  result.clear();

  assert(group1);
  assert(group2);
  assert(group1->getCollisionDetector()->getType()
         == DARTCollisionDetector::getTypeStatic());
  assert(group2->getCollisionDetector()->getType()
         == DARTCollisionDetector::getTypeStatic());

  auto objects1 = group1->getCollisionObjects();
  auto objects2 = group2->getCollisionObjects();

  if (objects1.empty() || objects2.empty())
    return false;

  auto done = false;
  const auto& filter = option.collisionFilter;

  for (auto i = 0u; i < objects1.size(); ++i)
  {
    auto collObj1 = objects1[i];

    for (auto j = 0u; j < objects2.size(); ++j)
    {
      auto collObj2 = objects2[j];

      if (filter && !filter->needCollision(collObj1.get(), collObj2.get()))
        continue;

      checkPair(collObj1.get(), collObj2.get(), option, result);

      if (result.getNumContacts() >= option.maxNumContacts)
      {
        done = true;
        break;
      }
    }

    if (done)
      break;
  }

  return result.isCollision();
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
               const Option& option, Result& result)
{
  Result pairResult;

  // Perform narrow-phase detection
  auto colliding = collide(o1->getShape(), o1->getTransform(),
                           o2->getShape(), o2->getTransform(),
                           pairResult);

  postProcess(o1, o2, option, result, pairResult);

  return colliding != 0;
}

//==============================================================================
bool isClose(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2,
             double tol)
{
  return (pos1 - pos2).norm() < tol;
}

//==============================================================================
void postProcess(CollisionObject* o1, CollisionObject* o2,
                 const Option& option,
                 Result& totalResult, const Result& pairResult)
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
