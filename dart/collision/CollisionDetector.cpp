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

#include "dart/common/Console.h"
#include "dart/collision/CollisionObject.h"
#include "dart/collision/CollisionGroup.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace collision {

//==============================================================================
std::shared_ptr<CollisionGroup>
CollisionDetector::createCollisionGroupAsSharedPtr()
{
  return std::shared_ptr<CollisionGroup>(createCollisionGroup().release());
}

//==============================================================================
CollisionObject* CollisionDetector::claimCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  if (!mCollisionObjectManager)
    mCollisionObjectManager.reset(new NaiveCollisionObjectManager(this));

  return mCollisionObjectManager->claimCollisionObject(shapeFrame);
}

//==============================================================================
void CollisionDetector::reclaimCollisionObject(const CollisionObject* collObj)
{
  if (!mCollisionObjectManager)
    mCollisionObjectManager.reset(new NaiveCollisionObjectManager(this));

  mCollisionObjectManager->reclaimCollisionObject(collObj);
}

//==============================================================================
CollisionDetector::CollisionObjectManager::CollisionObjectManager(
    CollisionDetector* cd)
  : mCollisionDetector(cd)
{
  // Do nothing
}

//==============================================================================
CollisionDetector::
NaiveCollisionObjectManager::NaiveCollisionObjectManager(
    CollisionDetector* cd)
  : CollisionDetector::CollisionObjectManager(cd)
{
  // Do nothing
}

//==============================================================================
CollisionDetector::
NaiveCollisionObjectManager::~NaiveCollisionObjectManager()
{
  assert(mCollisionObjects.empty());
}

//==============================================================================
CollisionObject*
CollisionDetector::NaiveCollisionObjectManager::claimCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  mCollisionObjects.push_back(
        std::move(mCollisionDetector->createCollisionObject(shapeFrame)));

  return mCollisionObjects.back().get();
}

//==============================================================================
void CollisionDetector::NaiveCollisionObjectManager::reclaimCollisionObject(
    const CollisionObject* object)
{
  auto search = std::find_if(mCollisionObjects.begin(), mCollisionObjects.end(),
      [&](const std::unique_ptr<CollisionObject>& it)
      { return it.get() == object; });

  if (mCollisionObjects.end() == search)
    return;

  mCollisionDetector->notifyCollisionObjectDestorying(search->get());
  mCollisionObjects.erase(search);
}

//==============================================================================
CollisionDetector::
RefCountingCollisionObjectManager::RefCountingCollisionObjectManager(
    CollisionDetector* cd)
  : CollisionDetector::CollisionObjectManager(cd)
{
  // Do nothing
}

//==============================================================================
CollisionDetector::
RefCountingCollisionObjectManager::~RefCountingCollisionObjectManager()
{
  assert(mCollisionObjectMap.empty());
}

//==============================================================================
CollisionObject*
CollisionDetector::RefCountingCollisionObjectManager::claimCollisionObject(
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

  auto newCollisionObject
      = mCollisionDetector->createCollisionObject(shapeFrame);

  mCollisionObjectMap[shapeFrame]
      = std::make_pair(std::move(newCollisionObject), 1u);

  return mCollisionObjectMap[shapeFrame].first.get();
}

//==============================================================================
void CollisionDetector::
RefCountingCollisionObjectManager::reclaimCollisionObject(
    const CollisionObject* object)
{
  auto shapeFrame = object->getShapeFrame();
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
    mCollisionDetector->notifyCollisionObjectDestorying(collisionObject.get());

    mCollisionObjectMap.erase(search);
  }
}

}  // namespace collision
}  // namespace dart
