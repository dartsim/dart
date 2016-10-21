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

#include "dart/collision/dart/DARTCollisionGroup.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/dart/BruteForceBroadPhase.hpp"

namespace dart {
namespace collision {

//==============================================================================
DARTCollisionGroup::DARTCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mBroadPhaseAlgorithm(common::make_unique<BruteForceBroadPhase>())
{
  // Do nothing
}

//==============================================================================
void DARTCollisionGroup::collideInGroup()
{
  for (auto& obj : mCollisionObjects)
    obj->updateAabb();
  // TODO(JS): improve; an AABB could get updated more than once

  mBroadPhaseAlgorithm->updateOverlappingPairs();

  const auto& overlappingPairs = mBroadPhaseAlgorithm->getOverlappingPairs();

  mOverlappingPairsInContact.clear();

  for (const auto& pair : overlappingPairs)
  {
    CollisionObject* objA = pair.first;
    CollisionObject* objB = pair.second;

    // TODO(JS): filtering

    const auto& algorithms = getDARTCollisionDetector()->getNarrowPhaseAlgorithms();
    const auto& algorithm = algorithms->getAlgorithm(
          objA->getShape()->getShapeType(),
          objB->getShape()->getShapeType());

    if (!algorithm)
    {
      // TODO(JS): print warning
      return;
    }

    algorithm(
          objA->getShape().get(), objA->getTransform(),
          objB->getShape().get(), objB->getTransform(), nullptr);
  }
}

//==============================================================================
DARTCollisionDetectorPtr DARTCollisionGroup::getDARTCollisionDetector()
{
  return std::static_pointer_cast<DARTCollisionDetector>(
        getCollisionDetector());
}

//==============================================================================
ConstDARTCollisionDetectorPtr
DARTCollisionGroup::getDARTCollisionDetector() const
{
  return std::static_pointer_cast<const DARTCollisionDetector>(
        getCollisionDetector());
}

//==============================================================================
struct ContactPointBinaryChecker : NarrowPhaseCallback
{
  bool mFound{false};

  bool notifyContact(const dynamics::Shape* /*shapeA*/,
                     const dynamics::Shape* /*shapeB*/,
                     const Eigen::Vector3d& /*point*/,
                     const Eigen::Vector3d& /*normal*/,
                     double /*depth*/) override
  {
    if (!mFound)
      mFound = true;

    return true;
  }
};

//==============================================================================
struct ContactPointCollector : NarrowPhaseCallback
{
  std::vector<Contact> mContacts;

  std::size_t mMaxContacts;

  ContactPointCollector(std::size_t maxContacts = 1000)
    : mMaxContacts(maxContacts)
  {
    // Do nothing
  }

  bool notifyContact(const dynamics::Shape* /*shapeA*/,
                     const dynamics::Shape* /*shapeB*/,
                     const Eigen::Vector3d& point,
                     const Eigen::Vector3d& normal,
                     double depth) override
  {
    Contact contact;
    contact.point = point;
    contact.normal = normal;
    contact.penetrationDepth = depth;

    mContacts.emplace_back(std::move(contact));

    return false;
  }
};

//==============================================================================
bool DARTCollisionGroup::collide(
    const CollisionOption& /*option*/, CollisionResult* result)
{
  for (auto& obj : mCollisionObjects)
    obj->updateAabb();
  // TODO(JS): improve; an AABB could get updated more than once

  mBroadPhaseAlgorithm->updateOverlappingPairs();

  const auto& overlappingPairs = mBroadPhaseAlgorithm->getOverlappingPairs();

  mOverlappingPairsInContact.clear();

  ContactPointBinaryChecker binaryChecker;
  ContactPointCollector collector;

  bool found = false;

  for (const auto& pair : overlappingPairs)
  {
    collector.mContacts.clear();

    CollisionObject* objA = pair.first;
    CollisionObject* objB = pair.second;

    // TODO(JS): filtering

    const auto& algorithms = getDARTCollisionDetector()->getNarrowPhaseAlgorithms();
    const auto& algorithm = algorithms->getAlgorithm(
          objA->getShape()->getShapeType(),
          objB->getShape()->getShapeType());

    if (!algorithm)
    {
      // TODO(JS): print warning
      continue;
    }

    if (!result)
    {
      algorithm(
            objA->getShape().get(), objA->getTransform(),
            objB->getShape().get(), objB->getTransform(), &binaryChecker);

      if (binaryChecker.mFound)
        return true;
      else
        continue;
    }

    algorithm(
          objA->getShape().get(), objA->getTransform(),
          objB->getShape().get(), objB->getTransform(), &collector);

    for (auto& contact : collector.mContacts)
    {
      found = true;

      if (!result)
        continue;

      contact.collisionObject1 = objA;
      contact.collisionObject2 = objB;

      result->addContact(contact);
    }

    // TODO(JS): improve the logic for early termination according to option
  }

  return found;
}

//==============================================================================
void DARTCollisionGroup::initializeEngineData()
{
  // Do nothing
}

//==============================================================================
void DARTCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  if (std::find(mCollisionObjects.begin(), mCollisionObjects.end(), object)
      == mCollisionObjects.end())
  {
    mCollisionObjects.push_back(object);
  }
  // TODO(JS): remove above

  mBroadPhaseAlgorithm->addObject(object);
}

//==============================================================================
void DARTCollisionGroup::addCollisionObjectsToEngine(
    const std::vector<CollisionObject*>& collObjects)
{
  for (auto collObject : collObjects)
    addCollisionObjectToEngine(collObject);
}

//==============================================================================
void DARTCollisionGroup::removeCollisionObjectFromEngine(
    CollisionObject* object)
{
  mCollisionObjects.erase(
      std::remove(mCollisionObjects.begin(), mCollisionObjects.end(), object));
}

//==============================================================================
void DARTCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  mCollisionObjects.clear();
}

//==============================================================================
void DARTCollisionGroup::updateCollisionGroupEngineData()
{
  // Do nothing
}

}  // namespace collision
}  // namespace dart
