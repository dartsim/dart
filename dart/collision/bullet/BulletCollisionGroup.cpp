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

#include "dart/collision/bullet/BulletCollisionGroup.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/bullet/BulletCollisionObject.hpp"
#include "dart/collision/bullet/detail/BulletCollisionDispatcher.hpp"
#include "dart/common/Profile.hpp"

#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

namespace dart {
namespace collision {

//==============================================================================
BulletCollisionGroup::BulletCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mBulletProadphaseAlg(new btDbvtBroadphase()),
    mBulletCollisionConfiguration(new btDefaultCollisionConfiguration()),
    mBulletDispatcher(new detail::BulletCollisionDispatcher(
        mBulletCollisionConfiguration.get())),
    mBulletCollisionWorld(new btCollisionWorld(
        mBulletDispatcher.get(),
        mBulletProadphaseAlg.get(),
        mBulletCollisionConfiguration.get()))
{
  btGImpactCollisionAlgorithm::registerAlgorithm(
      static_cast<btCollisionDispatcher*>(mBulletDispatcher.get()));
}

//==============================================================================
void BulletCollisionGroup::initializeEngineData()
{
  // Do nothing
}

//==============================================================================
void BulletCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  auto casted = static_cast<BulletCollisionObject*>(object);

  mBulletCollisionWorld->addCollisionObject(casted->getBulletCollisionObject());

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::addCollisionObjectsToEngine(
    const std::vector<CollisionObject*>& collObjects)
{
  for (auto collObj : collObjects) {
    auto casted = static_cast<BulletCollisionObject*>(collObj);

    mBulletCollisionWorld->addCollisionObject(
        casted->getBulletCollisionObject());
  }

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::removeCollisionObjectFromEngine(
    CollisionObject* object)
{
  auto casted = static_cast<BulletCollisionObject*>(object);
  auto* btObject = casted->getBulletCollisionObject();
  auto* proxy = btObject->getBroadphaseHandle();
  auto* pairCache
      = mBulletCollisionWorld->getBroadphase()->getOverlappingPairCache();

  if (proxy && pairCache) {
    pairCache->cleanProxyFromPairs(proxy, mBulletDispatcher.get());
    pairCache->removeOverlappingPairsContainingProxy(
        proxy, mBulletDispatcher.get());
  }

  mBulletCollisionWorld->removeCollisionObject(btObject);

  if (mBulletProadphaseAlg) {
    auto* dbvt = static_cast<btDbvtBroadphase*>(mBulletProadphaseAlg.get());
    dbvt->resetPool(mBulletDispatcher.get());
  }

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  for (const auto& info : mObjectInfoList)
    removeCollisionObjectFromEngine(info->mObject.get());

  if (mBulletProadphaseAlg) {
    auto* dbvt = static_cast<btDbvtBroadphase*>(mBulletProadphaseAlg.get());
    dbvt->resetPool(mBulletDispatcher.get());
  }

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::updateCollisionGroupEngineData()
{
  DART_PROFILE_SCOPED_N("Bullet update AABBs");
  mBulletCollisionWorld->updateAabbs();
}

//==============================================================================
void BulletCollisionGroup::updateEngineDataForCollide()
{
  {
    DART_PROFILE_SCOPED_N("CollisionGroup update objects");
    for (const auto& info : mObjectInfoList) {
      auto* object = static_cast<BulletCollisionObject*>(info->mObject.get());
      object->BulletCollisionObject::updateEngineData();
    }
  }

  {
    DART_PROFILE_SCOPED_N("CollisionGroup update backend");
    updateCollisionGroupEngineData();
  }
}

//==============================================================================
btCollisionWorld* BulletCollisionGroup::getBulletCollisionWorld()
{
  return mBulletCollisionWorld.get();
}

//==============================================================================
const btCollisionWorld* BulletCollisionGroup::getBulletCollisionWorld() const
{
  return mBulletCollisionWorld.get();
}

//==============================================================================
bool BulletCollisionGroup::shouldFilterPersistentPairs(
    const CollisionFilter* filter, std::size_t revision)
{
  if (mLastPersistentPairFilter == filter
      && mLastPersistentPairFilterRevision == revision) {
    return false;
  }

  mLastPersistentPairFilter = filter;
  mLastPersistentPairFilterRevision = revision;
  return true;
}

//==============================================================================
void BulletCollisionGroup::resetPersistentPairFilterCache()
{
  mLastPersistentPairFilter = nullptr;
  mLastPersistentPairFilterRevision = 0u;
}

} // namespace collision
} // namespace dart
