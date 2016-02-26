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
#include "dart/collision/CollisionGroupData.h"
#include "dart/collision/CollisionDetector.h"

#include <cassert>

namespace dart {
namespace collision {

//==============================================================================
CollisionGroup::CollisionGroup(
    const CollisionDetectorPtr& collisionDetector,
    const CollisionGroup::CollisionObjectPtrs& collObjects)
  : mCollisionDetector(collisionDetector),
    mCollisionObjects(collObjects),
    mEngineData(mCollisionDetector->getEngine()->createCollisionGroupData(
        mCollisionObjects).release())
{
  assert(mCollisionDetector);
}

//==============================================================================
CollisionGroup::CollisionGroup(const CollisionGroup& other)
  : mCollisionDetector(other.mCollisionDetector),
    mCollisionObjects(other.mCollisionObjects),
    mEngineData(other.mEngineData->clone(mCollisionObjects))
{
  // Do nothing
}

//==============================================================================
CollisionGroup& CollisionGroup::operator=(const CollisionGroup& other)
{
  copy(other);
  return *this;
}

//==============================================================================
CollisionGroup::~CollisionGroup()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::copy(const CollisionGroup& other)
{
  mCollisionDetector = other.mCollisionDetector;
  mCollisionObjects = other.mCollisionObjects;
  mEngineData.reset(other.mEngineData->clone(mCollisionObjects).release());
}

//==============================================================================
void CollisionGroup::changeDetector(
    const CollisionDetectorPtr& collisionDetector)
{
  if (mCollisionDetector == collisionDetector)
    return;

//  mEngineData.reset(
//        engine->createCollisionGroupData(getCollisionObjects()).release());
  // TODO(JS): Implement this!
}

//==============================================================================
CollisionDetector* CollisionGroup::getCollisionDetector() const
{
  return mCollisionDetector.get();
}

//==============================================================================
bool CollisionGroup::hasCollisionObject(
    const CollisionGroup::CollisionObjectPtr& object) const
{
  return std::find(mCollisionObjects.begin(),
                   mCollisionObjects.end(), object)
      != mCollisionObjects.end();
}

//==============================================================================
void CollisionGroup::addCollisionObject(const CollisionObjectPtr& object)
{
  if (hasCollisionObject(object))
  {
    // TODO(JS): print warning message
    return;
  }

  mCollisionObjects.push_back(object);
  mEngineData->addCollisionObject(object);
}

//==============================================================================
void CollisionGroup::addCollisionObjects(
    const CollisionGroup::CollisionObjectPtrs& objects)
{
  bool added = false;

  for (auto object : objects)
  {
    if (!hasCollisionObject(object))
    {
      mCollisionObjects.push_back(object);
      mEngineData->addCollisionObject(object, false);
      added = true;
    }
  }

  if (added)
    mEngineData->init();
}

//==============================================================================
void CollisionGroup::removeCollisionObject(
    const CollisionGroup::CollisionObjectPtr& object)
{
  if (!object)
    return;

  auto result
      = std::find(mCollisionObjects.begin(), mCollisionObjects.end(), object);

  if (mCollisionObjects.end() != result)
  {
    mCollisionObjects.erase(result);
    mEngineData->removeCollisionObject(*result);
  }
}

//==============================================================================
void CollisionGroup::removeCollisionObjects(
    const CollisionGroup::CollisionObjectPtrs& objects)
{
  for (auto object : objects)
    removeCollisionObject(object);
  // TODO(JS): there is a room for improving the perfomance
}

//==============================================================================
void CollisionGroup::removeAllCollisionObjects()
{
  mCollisionObjects.clear();
  mEngineData->removeAllCollisionObjects();
}

//==============================================================================
const CollisionGroup::CollisionObjectPtrs&
CollisionGroup::getCollisionObjects()
{
  return mCollisionObjects;
}

//==============================================================================
const CollisionGroup::ConstCollisionObjectPtrs
CollisionGroup::getCollisionObjects() const
{
  ConstCollisionObjectPtrs vec(mCollisionObjects.size());

  for (auto i = 0u; i < mCollisionObjects.size(); ++i)
  {
    vec[i] = std::const_pointer_cast<const CollisionObject>(
          mCollisionObjects[i]);
  }

  return vec;
}

//==============================================================================
void CollisionGroup::unionGroup(const CollisionGroupPtr& other)
{
  if (!other)
    return;

  this->addCollisionObjects(other->getCollisionObjects());
}

//==============================================================================
void CollisionGroup::subtractGroup(const CollisionGroupPtr& other)
{
  if (!other)
    return;

  this->removeCollisionObjects(other->getCollisionObjects());
}

//==============================================================================
bool CollisionGroup::detect(const Option& option, Result& result)
{
  return mCollisionDetector->detect(this, option, result);
}

//==============================================================================
bool CollisionGroup::detect(CollisionObject* object,
                            const Option& option, Result& result)
{
  return mCollisionDetector->detect(object, this, option, result);
}

//==============================================================================
bool CollisionGroup::detect(CollisionGroup* otherGroup,
                            const Option& option, Result& result)
{
  return mCollisionDetector->detect(this, otherGroup, option, result);
}

//==============================================================================
CollisionGroupData* CollisionGroup::getEngineData()
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
