/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#include "dart/collision/bullet/BulletCollisionGroupData.h"

#include "dart/collision/CollisionObject.h"
#include "dart/collision/bullet/BulletCollisionObjectData.h"

namespace dart {
namespace collision {

//==============================================================================
BulletCollisionGroupData::BulletCollisionGroupData(
    CollisionDetector* collisionDetector,
    CollisionGroup* parent,
    const BulletCollisionGroupData::CollisionObjects& collObjects)
  : CollisionGroupData(collisionDetector, parent),
    mBulletProadphaseAlg(
      new btDbvtBroadphase()),
    mBulletCollisionConfiguration(
      new btDefaultCollisionConfiguration()),
    mBulletDispatcher(
      new btCollisionDispatcher(mBulletCollisionConfiguration.get())),
    mBulletCollisionWorld(
      new btCollisionWorld(mBulletDispatcher.get(),
                           mBulletProadphaseAlg.get(),
                           mBulletCollisionConfiguration.get()))
{
  btOverlapFilterCallback* filterCallback = new CollisionFilter();
  btOverlappingPairCache* pairCache = mBulletCollisionWorld->getPairCache();
  assert(pairCache != nullptr);
  pairCache->setOverlapFilterCallback(filterCallback);

  addCollisionObjects(collObjects, true);
}

//==============================================================================
std::unique_ptr<CollisionGroupData> BulletCollisionGroupData::clone(
    CollisionGroup* newParent,
    const CollisionObjectPtrs& collObjects) const
{
  return std::unique_ptr<CollisionGroupData>(
        new BulletCollisionGroupData(mCollisionDetector, newParent, collObjects));
}

//==============================================================================
void BulletCollisionGroupData::init()
{
  btDispatcherInfo& dispatchInfo = mBulletCollisionWorld->getDispatchInfo();
  // dispatchInfo.m_timeStep  = 0.001;
  dispatchInfo.m_stepCount = 0;
}

//==============================================================================
void BulletCollisionGroupData::addCollisionObject(
    const CollisionObjectPtr& object, const bool init)
{
  auto data = static_cast<BulletCollisionObjectData*>(object->getEngineData());
  mBulletCollisionWorld->addCollisionObject(data->getBulletCollisionObject());

  if (init)
    this->init();
}

//==============================================================================
void BulletCollisionGroupData::addCollisionObjects(
    const BulletCollisionGroupData::CollisionObjectPtrs& collObjects,
    const bool init)
{
  for (auto collObj : collObjects)
  {
    auto data = static_cast<BulletCollisionObjectData*>(
          collObj->getEngineData());

    mBulletCollisionWorld->addCollisionObject(data->getBulletCollisionObject());
  }

  if (init)
    this->init();
}

//==============================================================================
void BulletCollisionGroupData::removeCollisionObject(
    const CollisionObjectPtr& object, const bool init)
{
  auto data = static_cast<BulletCollisionObjectData*>(object->getEngineData());
  mBulletCollisionWorld->removeCollisionObject(
        data->getBulletCollisionObject());

  if (init)
    this->init();
}

//==============================================================================
void BulletCollisionGroupData::removeAllCollisionObjects(bool init)
{
  auto collisionObjects = mParent->getCollisionObjects();
  for (auto collisionObject : collisionObjects)
    removeCollisionObject(collisionObject, false);

  if (init)
    this->init();
}

//==============================================================================
void BulletCollisionGroupData::update()
{
  // Setting up broadphase collision detection options
  btDispatcherInfo& dispatchInfo = mBulletCollisionWorld->getDispatchInfo();
  // dispatchInfo.m_timeStep  = 0.001;
  dispatchInfo.m_stepCount = 0;

  mBulletCollisionWorld->updateAabbs();
}

//==============================================================================
btCollisionWorld* BulletCollisionGroupData::getBulletCollisionWorld() const
{
  return mBulletCollisionWorld.get();
}

//==============================================================================
bool BulletCollisionGroupData::CollisionFilter::needBroadphaseCollision(
    btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
{
  assert((proxy0 != nullptr && proxy1 != nullptr) &&
         "Bullet broadphase overlapping pair proxies are nullptr");

  bool collide = (proxy0->m_collisionFilterGroup &
                  proxy1->m_collisionFilterMask) != 0;
  collide = collide && (proxy1->m_collisionFilterGroup &
                        proxy0->m_collisionFilterMask);

  if (collide)
  {
//    auto collObj0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
//    auto collObj1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);

//    auto userData0 = static_cast<BulletCollisionObjectUserData*>(collObj0->getUserPointer());
//    auto userData1 = static_cast<BulletCollisionObjectUserData*>(collObj1->getUserPointer());

//    // Assume single collision detector
//    assert(userData0->collisionDetector == userData1->collisionDetector);

//    auto collGroup0 = userData0->group;
//    auto collGroup1 = userData1->group;

//    if (!collGroup0 || !)

    // TODO(JS): check collision pair if they are enabled.
  }

  return collide;
}

}  // namespace collision
}  // namespace dart
