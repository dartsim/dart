/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#include "dart/collision/bullet/BulletCollisionDetector.h"

#include <iostream>

#include "dart/collision/CollisionObject.h"
#include "dart/collision/bullet/BulletTypes.h"
#include "dart/collision/bullet/BulletCollisionObjectData.h"
#include "dart/collision/bullet/BulletCollisionGroupData.h"

namespace dart {
namespace collision {

namespace {

struct BulletContactResultCallback : btCollisionWorld::ContactResultCallback
{
  BulletContactResultCallback(Result& result)
    : ContactResultCallback(),
      mResult(result)
  {
  }

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;

  Result& mResult;
};

} // anonymous namespace



//==============================================================================
std::shared_ptr<BulletCollisionDetector> BulletCollisionDetector::create()
{
  return std::shared_ptr<BulletCollisionDetector>(
        new BulletCollisionDetector());
}

//==============================================================================
const std::string& BulletCollisionDetector::getTypeStatic()
{
  static const std::string& type("Bullet");
  return type;
}

//==============================================================================
const std::string& BulletCollisionDetector::getType() const
{
  return getTypeStatic();
}

//==============================================================================
std::unique_ptr<CollisionObjectData>
BulletCollisionDetector::createCollisionObjectData(CollisionObject* parent,
                                        const dynamics::ShapePtr& shape)
{
  return std::unique_ptr<CollisionObjectData>(
        new BulletCollisionObjectData(this, parent, shape));
}

//==============================================================================
std::unique_ptr<CollisionGroupData>
BulletCollisionDetector::createCollisionGroupData(
    CollisionGroup* parent,
    const CollisionObjectPtrs& collObjects)
{
  return std::unique_ptr<CollisionGroupData>(
        new BulletCollisionGroupData(this, parent, collObjects));
}

//==============================================================================
Contact convertContact(const btManifoldPoint& bulletManifoldPoint,
                       const BulletCollisionObjectUserData* userData1,
                       const BulletCollisionObjectUserData* userData2)
{
  assert(userData1);
  assert(userData2);

  Contact contact;

  contact.point = convertVector3(bulletManifoldPoint.getPositionWorldOnA());
  contact.normal = convertVector3(bulletManifoldPoint.m_normalWorldOnB);
  contact.penetrationDepth = -bulletManifoldPoint.m_distance1;
  contact.collisionObject1 = userData1->collisionObject;
  contact.collisionObject2 = userData2->collisionObject;

  return contact;
}

//==============================================================================
void convertContacts(btCollisionWorld* collWorld, Result& result)
{
  assert(collWorld);

  auto dispatcher = collWorld->getDispatcher();
  assert(dispatcher);

  auto numManifolds = dispatcher->getNumManifolds();

  for (auto i = 0; i < numManifolds; ++i)
  {
    auto contactManifold = dispatcher->getManifoldByIndexInternal(i);
    const auto bulletCollObj0 = contactManifold->getBody0();
    const auto bulletCollObj1 = contactManifold->getBody1();

    auto userPointer0 = bulletCollObj0->getUserPointer();
    auto userPointer1 = bulletCollObj1->getUserPointer();

    auto userDataA = static_cast<BulletCollisionObjectUserData*>(userPointer1);
    auto userDataB = static_cast<BulletCollisionObjectUserData*>(userPointer0);

    auto numContacts = contactManifold->getNumContacts();

    for (auto j = 0; j < numContacts; ++j)
    {
      auto& cp = contactManifold->getContactPoint(j);

      result.contacts.push_back(convertContact(cp, userDataA, userDataB));
    }
  }
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionObjectData* objectData1,
    CollisionObjectData* objectData2,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(objectData1->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());
  assert(objectData2->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());

  auto castedData1 = static_cast<const BulletCollisionObjectData*>(objectData1);
  auto castedData2 = static_cast<const BulletCollisionObjectData*>(objectData2);

  auto bulletCollObj1 = castedData1->getBulletCollisionObject();
  auto bulletCollObj2 = castedData2->getBulletCollisionObject();

  if (!mBulletCollisionGroupForSinglePair)
    mBulletCollisionGroupForSinglePair = createCollisionGroup();

  auto cb = BulletContactResultCallback(result);
  auto collisionGroupData = static_cast<BulletCollisionGroupData*>(
        mBulletCollisionGroupForSinglePair->getEngineData());
  auto bulletCollisionWorld = collisionGroupData->getBulletCollisionWorld();
  bulletCollisionWorld->contactPairTest(bulletCollObj1, bulletCollObj2, cb);

  return !result.contacts.empty();
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionObjectData* objectData,
    CollisionGroupData* groupData,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(objectData);
  assert(groupData);
  assert(objectData->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());
  assert(groupData->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());

  auto castedObjData = static_cast<BulletCollisionObjectData*>(objectData);
  auto castedGrpData = static_cast<BulletCollisionGroupData*>(groupData);

  auto bulletCollObj = castedObjData->getBulletCollisionObject();
  auto bulletCollisionWorld = castedGrpData->getBulletCollisionWorld();

  auto cb = BulletContactResultCallback(result);
  bulletCollisionWorld->contactTest(bulletCollObj, cb);

  return !result.contacts.empty();
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionGroupData* groupData,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(groupData);
  assert(groupData->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());

  auto castedData = static_cast<BulletCollisionGroupData*>(groupData);

  auto bulletCollisionWorld = castedData->getBulletCollisionWorld();

  bulletCollisionWorld->performDiscreteCollisionDetection();

  convertContacts(bulletCollisionWorld, result);

  return !result.contacts.empty();
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionGroupData* groupData1,
    CollisionGroupData* groupData2,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(groupData1);
  assert(groupData2);
  assert(groupData1->getCollisionDetector()->getType() == BulletCollisionDetector::getTypeStatic());
  assert(groupData2->getCollisionDetector()->getType() == BulletCollisionDetector::getTypeStatic());

  auto castedData1 = static_cast<BulletCollisionGroupData*>(groupData1);
  auto castedData2 = static_cast<BulletCollisionGroupData*>(groupData2);

  auto bulletCollisionWorld1 = castedData1->getBulletCollisionWorld();
  auto bulletCollisionWorld2 = castedData2->getBulletCollisionWorld();

//  BulletCollisionData collData(&option, &result);
//  bulletCollisionWorld1->collide(bulletCollisionWorld2, &collData, checkPair);

  return !result.contacts.empty();
}




namespace {

btScalar BulletContactResultCallback::addSingleResult(
    btManifoldPoint& cp,
    const btCollisionObjectWrapper* colObj0Wrap,
    int /*partId0*/,
    int /*index0*/,
    const btCollisionObjectWrapper* colObj1Wrap,
    int /*partId1*/,
    int /*index1*/)
{
  auto userPointer0 = colObj0Wrap->getCollisionObject()->getUserPointer();
  auto userPointer1 = colObj1Wrap->getCollisionObject()->getUserPointer();

  auto userDataA = static_cast<BulletCollisionObjectUserData*>(userPointer1);
  auto userDataB = static_cast<BulletCollisionObjectUserData*>(userPointer0);

  mResult.contacts.push_back(convertContact(cp, userDataA, userDataB));

  return 1.0f;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
