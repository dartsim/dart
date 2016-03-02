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

#include "dart/collision/fcl/FCLMeshCollisionGroupData.h"

#include "dart/collision/CollisionObject.h"
#include "dart/collision/fcl/FCLMeshCollisionObjectData.h"

namespace dart {
namespace collision {

//==============================================================================
FCLMeshCollisionGroupData::FCLMeshCollisionGroupData(
    CollisionDetector* collisionDetector,
    CollisionGroup* parent,
    const FCLMeshCollisionGroupData::CollisionObjects& collObjects)
  : CollisionGroupData(collisionDetector, parent),
    mBroadPhaseAlg(new FCLCollisionManager())
{
  for (auto collObj : collObjects)
  {
    auto data = static_cast<FCLMeshCollisionObjectData*>(collObj->getEngineData());
    mBroadPhaseAlg->registerObject(data->getFCLCollisionObject());
  }

  mBroadPhaseAlg->setup();
}

//==============================================================================
std::unique_ptr<CollisionGroupData>
FCLMeshCollisionGroupData::clone(
    CollisionGroup* newParent,
    const CollisionGroupData::CollisionObjectPtrs& collObjects) const
{
  return std::unique_ptr<CollisionGroupData>(
        new FCLMeshCollisionGroupData(mCollisionDetector, newParent, collObjects));
}

//==============================================================================
void FCLMeshCollisionGroupData::init()
{
  mBroadPhaseAlg->setup();
}

//==============================================================================
void FCLMeshCollisionGroupData::addCollisionObject(
    const CollisionObjectPtr& object, const bool init)
{
  auto data = static_cast<FCLMeshCollisionObjectData*>(
        object->getEngineData());
  mBroadPhaseAlg->registerObject(data->getFCLCollisionObject());

  if (init)
    this->init();
}

//==============================================================================
void FCLMeshCollisionGroupData::addCollisionObjects(
    const CollisionGroupData::CollisionObjectPtrs& collObjects, const bool init)
{
  for (auto collObj : collObjects)
  {
    auto data = static_cast<FCLMeshCollisionObjectData*>(
          collObj->getEngineData());

    mBroadPhaseAlg->registerObject(data->getFCLCollisionObject());
  }

  if (init)
    this->init();
}

//==============================================================================
void FCLMeshCollisionGroupData::removeCollisionObject(
    const CollisionObjectPtr& object, const bool init)
{
  auto data = static_cast<FCLMeshCollisionObjectData*>(
        object->getEngineData());
  mBroadPhaseAlg->unregisterObject(data->getFCLCollisionObject());

  if (init)
    this->init();
}

//==============================================================================
void FCLMeshCollisionGroupData::removeAllCollisionObjects(bool init)
{
  mBroadPhaseAlg->clear();

  if (init)
    this->init();
}

//==============================================================================
void FCLMeshCollisionGroupData::update()
{
  mBroadPhaseAlg->setup();
  mBroadPhaseAlg->update();
}

//==============================================================================
FCLMeshCollisionGroupData::FCLCollisionManager*
FCLMeshCollisionGroupData::getFCLCollisionManager() const
{
  return mBroadPhaseAlg.get();
}


}  // namespace collision
}  // namespace dart
