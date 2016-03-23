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

#include "dart/collision/bullet/BulletCollisionGroup.h"

#include "dart/collision/CollisionObject.h"
#include "dart/collision/bullet/BulletCollisionObject.h"

namespace dart {
namespace collision {

//==============================================================================
BulletCollisionGroup::BulletCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
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
  assert(mCollisionDetector);
}

//==============================================================================
BulletCollisionGroup::BulletCollisionGroup(
    const CollisionDetectorPtr& collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : CollisionGroup(collisionDetector),
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
  assert(mCollisionDetector);
  assert(shapeFrame);

  addShapeFrame(shapeFrame);
}

//==============================================================================
BulletCollisionGroup::BulletCollisionGroup(
    const CollisionDetectorPtr& collisionDetector,
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames)
  : CollisionGroup(collisionDetector),
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
  assert(mCollisionDetector);

  addShapeFrames(shapeFrames);
}

//==============================================================================
BulletCollisionGroup::~BulletCollisionGroup()
{
  removeAllShapeFrames();
}

//==============================================================================
void BulletCollisionGroup::initializeEngineData()
{
  btDispatcherInfo& dispatchInfo = mBulletCollisionWorld->getDispatchInfo();
  // dispatchInfo.m_timeStep  = 0.001;
  dispatchInfo.m_stepCount = 0;
}

//==============================================================================
void BulletCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  auto casted = static_cast<BulletCollisionObject*>(object);

  mBulletCollisionWorld->addCollisionObject(casted->getBulletCollisionObject());

  this->initializeEngineData();
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

  this->initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::removeCollisionObjectFromEngine(
    CollisionObject* object)
{
  auto casted = static_cast<BulletCollisionObject*>(object);

  mBulletCollisionWorld->removeCollisionObject(
        casted->getBulletCollisionObject());

  this->initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  for (const auto& collisionObject : getCollisionObjects())
    removeCollisionObjectFromEngine(collisionObject);

  this->initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::updateEngineData()
{
  // Setting up broadphase collision detection options
  btDispatcherInfo& dispatchInfo = mBulletCollisionWorld->getDispatchInfo();
  // dispatchInfo.m_timeStep  = 0.001;
  dispatchInfo.m_stepCount = 0;

  mBulletCollisionWorld->updateAabbs();
}

//==============================================================================
btCollisionWorld* BulletCollisionGroup::getBulletCollisionWorld() const
{
  return mBulletCollisionWorld.get();
}

}  // namespace collision
}  // namespace dart
