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

#include "dart/collision/CollisionGroup.h"

#include "dart/collision/CollisionObject.h"
#include "dart/collision/CollisionGroupEngineData.h"

#include <cassert>

namespace dart {
namespace collision {

//==============================================================================
CollisionGroup::CollisionGroup(
    const EnginePtr engine,
    const CollisionGroup::CollisionObjects& collObjects)
  : mEngine(engine),
    mCollisionObjects(collObjects),
    mEngineData(mEngine->createCollisionGroupData(mCollisionObjects))
{
  assert(mEngine);
}

//==============================================================================
CollisionGroup::CollisionGroup(
    const EnginePtr& engine,
    const CollisionObjectPtr& collObject)
  : mEngine(engine),
    mCollisionObjects{collObject}
{
  assert(mEngine);
  assert(collObject);
}

//==============================================================================
CollisionGroup::~CollisionGroup()
{
}

//==============================================================================
Engine* CollisionGroup::getEngine() const
{
  return mEngine.get();
}

//==============================================================================
void CollisionGroup::addCollisionObject(const CollisionObjectPtr& object)
{
  auto found = std::find(mCollisionObjects.begin(),
                         mCollisionObjects.end(), object)
      != mCollisionObjects.end();

  if (found)
  {
    // TODO(JS): print warning message
    return;
  }

  mCollisionObjects.push_back(object);
  mEngineData->notifyCollisionObjectAdded(object.get());
}

//==============================================================================
bool CollisionGroup::detect(const Option& option, Result& result)
{
  return mEngine->detect(this, option, result);
}

//==============================================================================
bool CollisionGroup::detect(CollisionObject* object,
                            const Option& option, Result& result)
{
  return mEngine->detect(object, this, option, result);
}

//==============================================================================
bool CollisionGroup::detect(CollisionGroup* otherGroup,
                            const Option& option, Result& result)
{
  return mEngine->detect(this, otherGroup, option, result);
}

//==============================================================================
CollisionGroupEngineData* CollisionGroup::getEngineData()
{
  return mEngineData.get();
}

//==============================================================================
void CollisionGroup::updateEngineData()
{
  for (auto collObj : mCollisionObjects)
    collObj->updateEngineData();

  mEngineData->update();
}

}  // namespace collision
}  // namespace dart
