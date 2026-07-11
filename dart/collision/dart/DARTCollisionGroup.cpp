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

#include "dart/collision/dart/DARTCollisionGroup.hpp"

#include "dart/collision/dart/DARTCollisionObject.hpp"

#include <algorithm>

namespace dart {
namespace collision {

//==============================================================================
DARTCollisionGroup::DARTCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mBroadPhase(std::make_unique<native::AabbTreeBroadPhase>())
{
  // Do nothing
}

//==============================================================================
void DARTCollisionGroup::initializeEngineData()
{
  // Do nothing
}

//==============================================================================
void DARTCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  if (mObjectToId.find(object) != mObjectToId.end())
    return;

  auto* nativeObject = static_cast<DARTCollisionObject*>(object);
  nativeObject->updateEngineData();

  const std::size_t id = assignId(nativeObject);
  mObjectToId[object] = id;
  mIdToObject[id] = nativeObject;
  mCollisionObjects.push_back(object);
  mBroadPhase->add(id, nativeObject->getNativeAabb());
}

//==============================================================================
void DARTCollisionGroup::addCollisionObjectsToEngine(
    const std::vector<CollisionObject*>& collObjects)
{
  for (auto* collObject : collObjects)
    addCollisionObjectToEngine(collObject);
}

//==============================================================================
void DARTCollisionGroup::removeCollisionObjectFromEngine(
    CollisionObject* object)
{
  const auto search = mObjectToId.find(object);
  if (search == mObjectToId.end())
    return;

  const std::size_t id = search->second;
  mBroadPhase->remove(id);
  mObjectToId.erase(search);
  mIdToObject.erase(id);
  mFreeIds.push_back(id);
  mCollisionObjects.erase(
      std::remove(mCollisionObjects.begin(), mCollisionObjects.end(), object),
      mCollisionObjects.end());
}

//==============================================================================
void DARTCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  mBroadPhase->clear();
  mCollisionObjects.clear();
  mIdToObject.clear();
  mObjectToId.clear();
  mFreeIds.clear();
  mNextId = 0u;
}

//==============================================================================
void DARTCollisionGroup::updateCollisionGroupEngineData()
{
  // Iterate the id map directly instead of hashing every object through
  // mObjectToId each step; broadphase update order cannot leak into results
  // because pair queries are normalized and sorted before visitation.
  for (const auto& entry : mIdToObject) {
    mBroadPhase->update(entry.first, entry.second->getNativeAabb());
  }
}

//==============================================================================
std::size_t DARTCollisionGroup::assignId(DARTCollisionObject* /*object*/)
{
  if (!mFreeIds.empty()) {
    const std::size_t id = mFreeIds.back();
    mFreeIds.pop_back();
    return id;
  }

  return mNextId++;
}

} // namespace collision
} // namespace dart
