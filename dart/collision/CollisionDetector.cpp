/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>,
 *            Tobias Kunz <tobias@gatech.edu>
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

#include "dart/collision/CollisionDetector.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include "dart/common/Console.h"
#include "dart/collision/CollisionObject.h"
#include "dart/collision/CollisionGroup.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace collision {

//==============================================================================
CollisionDetector::~CollisionDetector()
{
  assert(mCollisionObjectMap.empty());
}

//==============================================================================
std::shared_ptr<CollisionGroup> CollisionDetector::createCollisionGroup(
    dynamics::Skeleton* skel)
{
  assert(skel);

  auto group = createCollisionGroup();

  auto numBodyNodes = skel->getNumBodyNodes();
  for (auto i = 0u; i < numBodyNodes; ++i)
  {
    auto bodyNode = skel->getBodyNode(i);
    auto collisionShapeNodes
        = bodyNode->getShapeNodesWith<dynamics::CollisionAddon>();

    for (auto& shapeNode : collisionShapeNodes)
      group->addShapeFrame(shapeNode);
  }

  return group;
}

////==============================================================================
//std::shared_ptr<CollisionGroup> CollisionDetector::createCollisionGroup(
//    const std::vector<CollisionObjectPtr>& objects)
//{
//  return std::make_shared<CollisionGroup>(shared_from_this(), objects);
//}

////==============================================================================
//bool CollisionDetector::detect(
//    CollisionObject* object1, CollisionObject* object2,
//    const Option& option, Result& result)
//{
//  assert(object1->getCollisionDetector() == object2->getCollisionDetector());

//  object1->updateEngineData();
//  object2->updateEngineData();

//  return detect(object1->getEngineData(), object2->getEngineData(),
//                option, result);
//}

////==============================================================================
//bool CollisionDetector::detect(
//    CollisionObject* object, CollisionGroup* group,
//    const Option& option, Result& result)
//{
//  assert(object->getCollisionDetector() == group->getCollisionDetector());

//  object->updateEngineData();
//  group->updateEngineData();

//  return detect(object->getEngineData(), group->getEngineData(),
//                option, result);
//}

////==============================================================================
//bool CollisionDetector::detect(
//    CollisionGroup* group, CollisionObject* object,
//    const Option& option, Result& result)
//{
//  return detect(object, group, option, result);
//}

////==============================================================================
//bool CollisionDetector::detect(
//    CollisionGroup* group, const Option& option, Result& result)
//{
//  group->update();

//  return detect(group, option, result);
//}

////==============================================================================
//bool CollisionDetector::detect(
//    CollisionGroup* group1, CollisionGroup* group2,
//    const Option& option, Result& result)
//{
//  assert(group1->getCollisionDetector() == group2->getCollisionDetector());

//  group1->updateEngineData();
//  group2->updateEngineData();

//  return detect(group1->getEngineData(), group2->getEngineData(),
//                option, result);
//}

//==============================================================================
struct is_collision_object_equal
    : public std::unary_function<WeakCollisionObjectPtr, bool>
{
  explicit is_collision_object_equal(const CollisionObject* object)
    : mObject(object) {}

  bool operator() (const WeakCollisionObjectPtr& arg)
  {
    auto locked = arg.lock();
    assert(locked);

    return mObject == locked.get();
  }

  const CollisionObject* mObject;
};

//==============================================================================
CollisionObject* CollisionDetector::claimCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  auto search = mCollisionObjectMap.find(shapeFrame);

  // Found existing collision object
  if (mCollisionObjectMap.end() != search)
  {
    auto& collObjAndCount = search->second;

    auto& collObj = collObjAndCount.first;
    auto& count   = collObjAndCount.second;
    assert(count != 0u);

    count++;

    return collObj.get();
  }

  auto newCollisionObject = createCollisionObject(shapeFrame);

  mCollisionObjectMap[shapeFrame]
      = std::make_pair(std::move(newCollisionObject), 1u);

  return mCollisionObjectMap[shapeFrame].first.get();
}

//==============================================================================
void CollisionDetector::reclaimCollisionObject(const CollisionObject* collObj)
{
  auto shapeFrame = collObj->getShapeFrame();
  auto search = mCollisionObjectMap.find(shapeFrame);

  if (mCollisionObjectMap.end() == search)
    return;

  auto& collObjAndCount = search->second;
  auto& count = collObjAndCount.second;
  assert(count != 0u);

  count--;

  if (0u == count)
  {
    auto& collisionObject = collObjAndCount.first;
    notifyDestroyingCollisionObject(collisionObject.get());

    mCollisionObjectMap.erase(search);
  }
}

//==============================================================================
bool CollisionDetector::hasCollisionObject(const CollisionObject* collObj) const
{
  if (!collObj)
    return false;

  return true;
//  return std::find_if(
//      mCollisionObjectMap.begin(), mCollisionObjectMap.end(),
//      [&](const CollisionObjectPair& pair)
//      { return pair.second.first.get() == collObj; })
//      != mCollisionObjectMap.end();
}

}  // namespace collision
}  // namespace dart
