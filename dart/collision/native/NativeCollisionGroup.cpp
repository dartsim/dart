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

#include "dart/collision/native/NativeCollisionGroup.hpp"

#include "dart/collision/native/NativeCollisionObject.hpp"

#include <algorithm>

namespace dart {
namespace collision {

//==============================================================================
NativeCollisionGroup::NativeCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mBroadPhase(std::make_unique<native::BruteForceBroadPhase>())
{
  // Do nothing
}

//==============================================================================
void NativeCollisionGroup::initializeEngineData()
{
  // Do nothing
}

//==============================================================================
void NativeCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  if (mObjectToId.find(object) != mObjectToId.end())
    return;

  auto* nativeObject = static_cast<NativeCollisionObject*>(object);
  nativeObject->updateEngineData();

  const std::size_t id = assignId(nativeObject);
  mObjectToId[object] = id;
  if (id >= mIdToObject.size())
    mIdToObject.resize(id + 1u, nullptr);
  mIdToObject[id] = nativeObject;
  mCollisionObjects.push_back(object);
  mNativeObjects.push_back(nativeObject);
  mCollisionObjectIds.push_back(id);
  mCollisionObjectAabbs.push_back(nativeObject->getNativeAabb());
  mBroadPhase->add(id, nativeObject->getNativeAabb());
}

//==============================================================================
void NativeCollisionGroup::addCollisionObjectsToEngine(
    const std::vector<CollisionObject*>& collObjects)
{
  mCollisionObjects.reserve(mCollisionObjects.size() + collObjects.size());
  mNativeObjects.reserve(mNativeObjects.size() + collObjects.size());
  mCollisionObjectIds.reserve(mCollisionObjectIds.size() + collObjects.size());
  mCollisionObjectAabbs.reserve(
      mCollisionObjectAabbs.size() + collObjects.size());
  for (auto* collObject : collObjects)
    addCollisionObjectToEngine(collObject);
}

//==============================================================================
void NativeCollisionGroup::removeCollisionObjectFromEngine(
    CollisionObject* object)
{
  const auto search = mObjectToId.find(object);
  if (search == mObjectToId.end())
    return;

  const std::size_t id = search->second;
  mBroadPhase->remove(id);
  mObjectToId.erase(search);
  if (id < mIdToObject.size())
    mIdToObject[id] = nullptr;
  mFreeIds.push_back(id);
  const auto objectIt
      = std::find(mCollisionObjects.begin(), mCollisionObjects.end(), object);
  if (objectIt != mCollisionObjects.end()) {
    const auto index = static_cast<std::size_t>(
        std::distance(mCollisionObjects.begin(), objectIt));
    mCollisionObjects.erase(objectIt);
    mNativeObjects.erase(
        mNativeObjects.begin() + static_cast<std::ptrdiff_t>(index));
    mCollisionObjectIds.erase(
        mCollisionObjectIds.begin() + static_cast<std::ptrdiff_t>(index));
    mCollisionObjectAabbs.erase(
        mCollisionObjectAabbs.begin() + static_cast<std::ptrdiff_t>(index));
  }
}

//==============================================================================
void NativeCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  mBroadPhase->clear();
  mCollisionObjects.clear();
  mNativeObjects.clear();
  mCollisionObjectIds.clear();
  mCollisionObjectAabbs.clear();
  mIdToObject.clear();
  mObjectToId.clear();
  mFreeIds.clear();
  mNextId = 0u;
}

//==============================================================================
void NativeCollisionGroup::updateCollisionGroupEngineData()
{
  const auto numObjects = mNativeObjects.size();
  for (std::size_t i = 0u; i < numObjects; ++i)
    mCollisionObjectAabbs[i] = mNativeObjects[i]->getNativeAabb();

  mBroadPhase->updateRange(mCollisionObjectIds, mCollisionObjectAabbs);
}

//==============================================================================
void NativeCollisionGroup::updateEngineDataForCollide()
{
  const auto numObjects = mNativeObjects.size();
  for (std::size_t i = 0u; i < numObjects; ++i) {
    auto* nativeObject = mNativeObjects[i];
    nativeObject->NativeCollisionObject::updateEngineData();
    mCollisionObjectAabbs[i] = nativeObject->getNativeAabb();
  }

  mBroadPhase->updateRange(mCollisionObjectIds, mCollisionObjectAabbs);
}

//==============================================================================
std::size_t NativeCollisionGroup::assignId(NativeCollisionObject* /*object*/)
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
