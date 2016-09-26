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

#include "dart/collision/fcl/FCLCollisionGroup.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/fcl/FCLCollisionObject.hpp"

namespace dart {
namespace collision {

//==============================================================================
FCLCollisionGroup::FCLCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mBroadPhaseAlg(new fcl::DynamicAABBTreeCollisionManager())
{
  // Do nothing
}

//==============================================================================
void FCLCollisionGroup::initializeEngineData()
{
  mBroadPhaseAlg->setup();
}

//==============================================================================
void FCLCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  auto casted = static_cast<FCLCollisionObject*>(object);
  mBroadPhaseAlg->registerObject(casted->getFCLCollisionObject());

  initializeEngineData();
}

//==============================================================================
void FCLCollisionGroup::addCollisionObjectsToEngine(
    const std::vector<CollisionObject*>& collObjects)
{
  for (auto collObj : collObjects)
  {
    auto casted = static_cast<FCLCollisionObject*>(collObj);

    mBroadPhaseAlg->registerObject(casted->getFCLCollisionObject());
  }

  initializeEngineData();
}

//==============================================================================
void FCLCollisionGroup::removeCollisionObjectFromEngine(CollisionObject* object)
{
  auto casted = static_cast<FCLCollisionObject*>(object);

  mBroadPhaseAlg->unregisterObject(casted->getFCLCollisionObject());

  initializeEngineData();
}

//==============================================================================
void FCLCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  mBroadPhaseAlg->clear();

  initializeEngineData();
}

//==============================================================================
void FCLCollisionGroup::updateCollisionGroupEngineData()
{
  mBroadPhaseAlg->update();
}

//==============================================================================
FCLCollisionGroup::FCLCollisionManager*
FCLCollisionGroup::getFCLCollisionManager()
{
  return mBroadPhaseAlg.get();
}

//==============================================================================
const FCLCollisionGroup::FCLCollisionManager*
FCLCollisionGroup::getFCLCollisionManager() const
{
  return mBroadPhaseAlg.get();
}

}  // namespace collision
}  // namespace dart
