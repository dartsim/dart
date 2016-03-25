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

#include <iostream>
#include "dart/collision/CollisionObject.h"
#include "dart/collision/dart/DARTCollide.h"
#include "dart/collision/dart/DARTCollisionObject.h"
#include "dart/collision/dart/DARTCollisionGroup.h"

namespace dart {
namespace collision {

namespace {

bool checkPair(CollisionObject* o1, CollisionObject* o2,
                       Result& result);

bool isClose(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2,
             double tol);

void postProcess(CollisionObject* o1, CollisionObject* o2,
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
std::shared_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup()
{
  return std::make_shared<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
std::shared_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::make_shared<DARTCollisionGroup>(shared_from_this(), shapeFrame);
}

//==============================================================================
std::shared_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames)
{
  return std::make_shared<DARTCollisionGroup>(shared_from_this(), shapeFrames);
}

//==============================================================================
bool DARTCollisionDetector::detect(
    CollisionGroup* group,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(group);
  assert(group->getCollisionDetector()->getType()
         == DARTCollisionDetector::getTypeStatic());

  auto objects = group->getCollisionObjects();

  if (objects.empty())
    return false;

  for (auto i = 0u; i < objects.size() - 1; ++i)
  {
    auto collObj1 = objects[i];

    for (auto j = i + 1u; j < objects.size(); ++j)
    {
      auto collObj2 = objects[j];

      checkPair(collObj1, collObj2, result);
    }
  }

  return !result.contacts.empty();
}

//==============================================================================
bool DARTCollisionDetector::detect(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

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

  for (auto i = 0u; i < objects1.size(); ++i)
  {
    auto collObj1 = objects1[i];

    for (auto j = 0u; j < objects2.size(); ++j)
    {
      auto collObj2 = objects2[j];

      checkPair(collObj1, collObj2, result);
    }
  }

  return !result.contacts.empty();
}

//==============================================================================
std::unique_ptr<CollisionObject> DARTCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  auto collObj = new DARTCollisionObject(this, shapeFrame);

  mDARTCollisionObjects.push_back(collObj);

  return std::unique_ptr<CollisionObject>(collObj);
}

//==============================================================================
void DARTCollisionDetector::notifyCollisionObjectDestorying(
    CollisionObject* collObj)
{
  if (!collObj)
    return;

  auto casted = static_cast<DARTCollisionObject*>(collObj);
  mDARTCollisionObjects.erase(
        std::remove(mDARTCollisionObjects.begin(), mDARTCollisionObjects.end(),
                    casted), mDARTCollisionObjects.end());
}



namespace {

//==============================================================================
bool checkPair(CollisionObject* o1, CollisionObject* o2, Result& result)
{
  // TODO(JS): filtering

  Result pairResult;

  // Perform narrow-phase detection
  auto colliding = collide(o1->getShape(), o1->getTransform(),
                           o2->getShape(), o2->getTransform(),
                           &pairResult.contacts);

  postProcess(o1, o2, result, pairResult);

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
                 Result& totalResult, const Result& pairResult)
{
  if (pairResult.contacts.empty())
    return;

  // Don't add repeated points
  const auto tol = 3.0e-12;

  for (auto pairContact : pairResult.contacts)
  {
    auto foundClose = false;

    for (auto totalContact : totalResult.contacts)
    {
      if (isClose(pairContact.point, totalContact.point, tol))
      {
        foundClose = true;
        break;
      }
    }

    if (foundClose)
      continue;

    totalResult.contacts.push_back(pairContact);
    totalResult.contacts.back().collisionObject1 = o1;
    totalResult.contacts.back().collisionObject2 = o2;
  }
}

} // anonymous namespace

} // namespace collision
} // namespace dart
