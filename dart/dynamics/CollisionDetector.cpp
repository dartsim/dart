/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/dynamics/CollisionDetector.hpp"

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/CollisionGroup.hpp"
#include "dart/dynamics/CollisionObject.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include <algorithm>

namespace dart {
namespace collision {

//==============================================================================
CollisionDetector::Factory* CollisionDetector::getFactory()
{
  return SingletonFactory::getSingletonPtr();
}

//==============================================================================
std::shared_ptr<CollisionGroup>
CollisionDetector::createCollisionGroupAsSharedPtr()
{
  return std::shared_ptr<CollisionGroup>(createCollisionGroup().release());
}

//==============================================================================
bool CollisionDetector::raycast(
    CollisionGroup* /*group*/,
    const Eigen::Vector3d& /*from*/,
    const Eigen::Vector3d& /*to*/,
    const RaycastOption& /*option*/,
    RaycastResult* /*result*/)
{
  dtwarn << "[CollisionDetector] Raycast is not supported by '" << getType()
         << "'\n";
  return false;
}

//==============================================================================
std::shared_ptr<CollisionObject> CollisionDetector::claimCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  if (!mCollisionObjectManager)
    mCollisionObjectManager.reset(
        new ManagerForUnsharableCollisionObjects(this));

  return mCollisionObjectManager->claimCollisionObject(shapeFrame);
}

//==============================================================================
void CollisionDetector::notifyCollisionObjectDestroying(
    CollisionObject* /*object*/)
{
  // Do nothing
}

//==============================================================================
CollisionDetector::CollisionObjectManager::CollisionObjectManager(
    CollisionDetector* cd)
  : mCollisionDetector(cd)
{
  assert(cd);
}

//==============================================================================
CollisionDetector*
CollisionDetector::CollisionObjectManager::getCollisionDetector()
{
  return mCollisionDetector;
}

//==============================================================================
CollisionDetector::ManagerForUnsharableCollisionObjects::
    ManagerForUnsharableCollisionObjects(CollisionDetector* cd)
  : CollisionDetector::CollisionObjectManager(cd), mCollisionObjectDeleter(this)
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<CollisionObject>
CollisionDetector::ManagerForUnsharableCollisionObjects::claimCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  auto uniqueObject = mCollisionDetector->createCollisionObject(shapeFrame);
  auto sharedObject = std::shared_ptr<CollisionObject>(
      uniqueObject.release(), mCollisionObjectDeleter);

  return sharedObject;
}

//==============================================================================
CollisionDetector::ManagerForUnsharableCollisionObjects::
    CollisionObjectDeleter::CollisionObjectDeleter(
        ManagerForUnsharableCollisionObjects* mgr)
  : mCollisionObjectManager(mgr)
{
  assert(mgr);
}

//==============================================================================
void CollisionDetector::ManagerForUnsharableCollisionObjects ::
    CollisionObjectDeleter::operator()(CollisionObject* object) const
{
  mCollisionObjectManager->getCollisionDetector()
      ->notifyCollisionObjectDestroying(object);

  delete object;
}

//==============================================================================
CollisionDetector::ManagerForSharableCollisionObjects::
    ManagerForSharableCollisionObjects(CollisionDetector* cd)
  : CollisionDetector::CollisionObjectManager(cd), mCollisionObjectDeleter(this)
{
  // Do nothing
}

//==============================================================================
CollisionDetector::ManagerForSharableCollisionObjects::
    ~ManagerForSharableCollisionObjects()
{
  assert(mCollisionObjectMap.empty());
}

//==============================================================================
std::shared_ptr<CollisionObject>
CollisionDetector::ManagerForSharableCollisionObjects::claimCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  const auto search = mCollisionObjectMap.find(shapeFrame);

  const auto found = mCollisionObjectMap.end() != search;
  if (found)
  {
    const auto& collObj = search->second;
    assert(collObj.lock());
    // Ensure all the collision objects in the map are valid pointers

    return collObj.lock();
  }

  auto uniqueObject = mCollisionDetector->createCollisionObject(shapeFrame);
  auto sharedObject = std::shared_ptr<CollisionObject>(
      uniqueObject.release(), mCollisionObjectDeleter);

  mCollisionObjectMap[shapeFrame] = sharedObject;

  return sharedObject;
}

//==============================================================================
CollisionDetector::ManagerForSharableCollisionObjects::CollisionObjectDeleter::
    CollisionObjectDeleter(ManagerForSharableCollisionObjects* mgr)
  : mCollisionObjectManager(mgr)
{
  assert(mgr);
}

//==============================================================================
void CollisionDetector::ManagerForSharableCollisionObjects ::
    CollisionObjectDeleter::operator()(CollisionObject* object) const
{
  mCollisionObjectManager->getCollisionDetector()
      ->notifyCollisionObjectDestroying(object);
  mCollisionObjectManager->mCollisionObjectMap.erase(object->getShapeFrame());

  delete object;
}

} // namespace collision
} // namespace dart
