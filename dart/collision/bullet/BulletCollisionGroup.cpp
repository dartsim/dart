/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

namespace dart {
namespace collision {

//==============================================================================
BulletCollisionGroup::BulletCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mBulletProadphaseAlg(new btDbvtBroadphase()),
    mBulletCollisionConfiguration(new btDefaultCollisionConfiguration()),
    mBulletDispatcher(
      new btCollisionDispatcher(mBulletCollisionConfiguration.get())),
    mBulletCollisionWorld(
      new btCollisionWorld(mBulletDispatcher.get(),
                           mBulletProadphaseAlg.get(),
                           mBulletCollisionConfiguration.get()))
{
  // Do nothing
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
  for (auto collObj : collObjects)
  {
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

  mBulletCollisionWorld->removeCollisionObject(
        casted->getBulletCollisionObject());

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  for (const auto& pair : mShapeFrameMap)
    removeCollisionObjectFromEngine(pair.second.get());

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::updateCollisionGroupEngineData()
{
  mBulletCollisionWorld->updateAabbs();
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

}  // namespace collision
}  // namespace dart
