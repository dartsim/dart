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

#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/dart/detail/DARTCollisionGroupEngineData.hpp"
#include "dart/collision/dart/detail/DARTCollisionObjectEngineData.hpp"

#include <algorithm>

namespace dart {
namespace collision {

namespace {

//==============================================================================
std::size_t assignId(detail::DARTCollisionGroupEngineData& engineData)
{
  if (!engineData.freeIds.empty()) {
    const std::size_t id = engineData.freeIds.back();
    engineData.freeIds.pop_back();
    return id;
  }

  return engineData.nextId++;
}

} // namespace

//==============================================================================
DARTCollisionGroup::DARTCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector)
{
  auto* detector = static_cast<DARTCollisionDetector*>(collisionDetector.get());
  // Anchor the external broadphase state in the released detector pointer
  // member. Old DART 6.20 headers inline this class's destructor, but they
  // still destroy mCollisionDetector and therefore release this owner.
  mCollisionDetector
      = detector->attachCollisionGroupEngineData(this, collisionDetector);
}

//==============================================================================
void DARTCollisionGroup::initializeEngineData()
{
  // Do nothing
}

//==============================================================================
void DARTCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  auto& engineData = getEngineData();
  if (engineData.objectToId.find(object) != engineData.objectToId.end())
    return;

  auto* nativeObject = static_cast<DARTCollisionObject*>(object);
  nativeObject->updateEngineData();

  const std::size_t id = assignId(engineData);
  engineData.objectToId[object] = id;
  engineData.idToObject[id] = nativeObject;
  mCollisionObjects.push_back(object);
  engineData.broadPhase.add(
      id, detail::DARTCollisionObjectAccessor::getAabb(nativeObject));
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
  auto& engineData = getEngineData();
  const auto search = engineData.objectToId.find(object);
  if (search == engineData.objectToId.end())
    return;

  const std::size_t id = search->second;
  engineData.broadPhase.remove(id);
  engineData.objectToId.erase(search);
  engineData.idToObject.erase(id);
  engineData.freeIds.push_back(id);
  mCollisionObjects.erase(
      std::remove(mCollisionObjects.begin(), mCollisionObjects.end(), object),
      mCollisionObjects.end());
}

//==============================================================================
void DARTCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  auto& engineData = getEngineData();
  engineData.broadPhase.clear();
  mCollisionObjects.clear();
  engineData.idToObject.clear();
  engineData.objectToId.clear();
  engineData.freeIds.clear();
  engineData.nextId = 0u;
}

//==============================================================================
void DARTCollisionGroup::updateCollisionGroupEngineData()
{
  auto& engineData = getEngineData();
  // Iterate the id map directly instead of hashing every object through
  // objectToId each step; broadphase update order cannot leak into results
  // because pair queries are normalized and sorted before visitation.
  for (const auto& entry : engineData.idToObject) {
    engineData.broadPhase.update(
        entry.first,
        detail::DARTCollisionObjectAccessor::getAabb(entry.second));
  }
}

//==============================================================================
void DARTCollisionGroup::updateEngineDataForCollide()
{
  updateEngineData();
}

//==============================================================================
detail::DARTCollisionGroupEngineData& DARTCollisionGroup::getEngineData()
{
  auto* detector
      = static_cast<DARTCollisionDetector*>(getCollisionDetector().get());
  return detector->getCollisionGroupEngineData(this);
}

} // namespace collision
} // namespace dart
