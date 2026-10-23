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

#include "dart/collision/fcl/FCLCollisionGroup.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/detail/CollisionHistory.hpp"
#include "dart/collision/fcl/FCLCollisionObject.hpp"

#include <algorithm>

namespace dart {
namespace collision {

namespace {

//==============================================================================
bool isUnboundedObject(const FCLCollisionObject* object)
{
  if (!object)
    return false;

  const auto* fclObject = object->getFCLCollisionObject();
  if (!fclObject)
    return false;

  return dynamic_cast<const fcl::Halfspace*>(
             fclObject->collisionGeometry().get())
         != nullptr;
}

//==============================================================================
void eraseObject(
    std::vector<fcl::CollisionObject*>& objects, fcl::CollisionObject* object)
{
  objects.erase(
      std::remove(objects.begin(), objects.end(), object), objects.end());
}

//==============================================================================
bool containsObject(
    const std::vector<fcl::CollisionObject*>& objects,
    fcl::CollisionObject* object)
{
  return std::find(objects.begin(), objects.end(), object) != objects.end();
}

} // namespace

//==============================================================================
void detail::CollisionHistory::clear(FCLCollisionGroup& group)
{
  // Incremental balancing retains tree topology and traversal history even at
  // unchanged poses, and a replaced shape's object moves behind the others.
  // Reinsert every object in group order at the restored poses; FCL's public
  // tree reset requires allocating new nodes. Pick up shapes replaced since
  // the last step first, as collide() would. Without automatic updates the
  // caller synchronizes replaced shapes, so reinsert the engine objects as
  // they are and let the next collision check refresh their poses.
  const bool automaticUpdate = group.getAutomaticUpdate();
  if (automaticUpdate)
    group.update();
  group.removeAllCollisionObjectsFromEngine();
  if (automaticUpdate)
    group.updateEngineData();
  for (const auto& info : group.mObjectInfoList)
    group.addCollisionObjectToEngine(info->mObject.get());
}

//==============================================================================
FCLCollisionGroup::FCLCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mBroadPhaseAlg(new dart::collision::fcl::DynamicAABBTreeCollisionManager())
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
  auto fclObject = casted->getFCLCollisionObject();

  if (isUnboundedObject(casted)) {
    mUnboundedObjects.push_back(fclObject);
  } else {
    mFiniteObjects.push_back(fclObject);
    try {
      mBroadPhaseAlg->registerObject(fclObject);
    } catch (...) {
      // FCL may have inserted a tree leaf without its lookup-table entry.
      mFiniteObjects.pop_back();
      mBroadPhaseAlg->clear();
      mBroadPhaseAlg->registerObjects(mFiniteObjects);
      initializeEngineData();
      throw;
    }
  }

  initializeEngineData();
}

//==============================================================================
void FCLCollisionGroup::addCollisionObjectsToEngine(
    const std::vector<CollisionObject*>& collObjects)
{
  for (auto collObj : collObjects) {
    auto casted = static_cast<FCLCollisionObject*>(collObj);
    auto fclObject = casted->getFCLCollisionObject();

    if (isUnboundedObject(casted)) {
      mUnboundedObjects.push_back(fclObject);
    } else {
      mFiniteObjects.push_back(fclObject);
      mBroadPhaseAlg->registerObject(fclObject);
    }
  }

  initializeEngineData();
}

//==============================================================================
void FCLCollisionGroup::removeCollisionObjectFromEngine(CollisionObject* object)
{
  auto casted = static_cast<FCLCollisionObject*>(object);
  auto fclObject = casted->getFCLCollisionObject();

  if (containsObject(mFiniteObjects, fclObject))
    mBroadPhaseAlg->unregisterObject(fclObject);

  eraseObject(mFiniteObjects, fclObject);
  eraseObject(mUnboundedObjects, fclObject);

  initializeEngineData();
}

//==============================================================================
void FCLCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  mBroadPhaseAlg->clear();
  mFiniteObjects.clear();
  mUnboundedObjects.clear();

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

//==============================================================================
const std::vector<fcl::CollisionObject*>&
FCLCollisionGroup::getFiniteFCLCollisionObjects() const
{
  return mFiniteObjects;
}

//==============================================================================
const std::vector<fcl::CollisionObject*>&
FCLCollisionGroup::getUnboundedFCLCollisionObjects() const
{
  return mUnboundedObjects;
}

} // namespace collision
} // namespace dart
