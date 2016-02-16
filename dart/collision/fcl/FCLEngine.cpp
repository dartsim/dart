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

#include "dart/collision/fcl/FCLEngine.h"

#include <fcl/collision.h>
#include <fcl/collision_object.h>
#include <fcl/collision_data.h>
#include <fcl/broadphase/broadphase.h>

#include "dart/collision/CollisionObject.h"
#include "dart/collision/fcl/FCLTypes.h"
#include "dart/collision/fcl/FCLCollisionNode.h"
#include "dart/collision/fcl/FCLCollisionObjectEngineData.h"
#include "dart/collision/fcl/FCLCollisionGroupEngineData.h"

namespace dart {
namespace collision {

namespace {

// Collision data stores the collision request and the result given by
// collision algorithm.
struct CollisionData
{
  // Collision request
  fcl::CollisionRequest request;

  // Collision result
  fcl::CollisionResult result;

  // FCL collision detector
//  FCLEngine* collisionDetector;

  // Whether the collision iteration can stop
  bool done;

  bool checkAllCollisions;

  CollisionData();
};

bool collisionCallBack(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata);

void convert(const Option& fclResult, fcl::CollisionRequest& request);

void convert(const fcl::CollisionResult& fclResult, Result& result);

void convert(const fcl::Contact& fclContact, Contact& contact);

} // anonymous namespace



//==============================================================================
const std::string& FCLEngine::getType() const
{
  return getTypeStatic();
}

//==============================================================================
FCLEnginePtr FCLEngine::create()
{
  return FCLEnginePtr(new FCLEngine());
}

//==============================================================================
const std::string& FCLEngine::getTypeStatic()
{
  static const std::string& type("FCL");
  return type;
}

//==============================================================================
CollisionObjectEngineDataPtr FCLEngine::createCollisionObjectData(
    CollisionObject* parent,
    const dynamics::ShapePtr& shape)
{
  return std::make_shared<FCLCollisionObjectEngineData>(parent, shape);
}

//==============================================================================
CollisionGroupEngineDataPtr FCLEngine::createCollisionGroupData(
    const CollisionObjects& collObjects)
{
  return std::make_shared<FCLCollisionGroupEngineData>(collObjects);
}

//==============================================================================
bool FCLEngine::detect(CollisionObject* object1,
                       CollisionObject* object2,
                       const Option& option,
                       Result& result)
{
  result.contacts.clear();

  assert(object1->getEngine()->getType() == FCLEngine::getTypeStatic());
  assert(object2->getEngine()->getType() == FCLEngine::getTypeStatic());

  object1->updateEngineData();
  object2->updateEngineData();

  auto data1 = static_cast<FCLCollisionObjectEngineData*>(object1->getEngineData());
  auto data2 = static_cast<FCLCollisionObjectEngineData*>(object2->getEngineData());

  auto fclCollObj1 = data1->getFCLCollisionObject();
  auto fclCollObj2 = data2->getFCLCollisionObject();

  CollisionData collData;
  convert(option, collData.request);

  collisionCallBack(fclCollObj1, fclCollObj2, &collData);

  convert(collData.result, result);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLEngine::detect(CollisionObject* object, CollisionGroup* group,
                       const Option& option, Result& result)
{
  result.contacts.clear();

  assert(object);
  assert(group);
  assert(object->getEngine()->getType() == FCLEngine::getTypeStatic());
  assert(group->getEngine()->getType() == FCLEngine::getTypeStatic());

  object->updateEngineData();
  group->updateEngineData();

  auto objData = static_cast<FCLCollisionObjectEngineData*>(object->getEngineData());
  auto groupData = static_cast<FCLCollisionGroupEngineData*>(group->getEngineData());

  auto fclObject = objData->getFCLCollisionObject();
  auto broadPhaseAlg = groupData->getFCLCollisionManager();

  CollisionData collData;
  convert(option, collData.request);

  broadPhaseAlg->collide(fclObject, &collData, collisionCallBack);

  convert(collData.result, result);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLEngine::detect(CollisionGroup* group,
                       const Option& option, Result& result)
{
  result.contacts.clear();

  assert(group);
  assert(group->getEngine()->getType() == FCLEngine::getTypeStatic());

  group->updateEngineData();

  auto data = static_cast<FCLCollisionGroupEngineData*>(group->getEngineData());

  auto broadPhaseAlg = data->getFCLCollisionManager();

  CollisionData collData;
  convert(option, collData.request);

  broadPhaseAlg->collide(&collData, collisionCallBack);

  convert(collData.result, result);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLEngine::detect(CollisionGroup* group1, CollisionGroup* group2,
                       const Option& option, Result& result)
{
  result.contacts.clear();

  assert(group1);
  assert(group2);
  assert(group1->getEngine()->getType() == FCLEngine::getTypeStatic());
  assert(group2->getEngine()->getType() == FCLEngine::getTypeStatic());

  group1->updateEngineData();
  group2->updateEngineData();

  auto data1 = static_cast<FCLCollisionGroupEngineData*>(group1->getEngineData());
  auto data2 = static_cast<FCLCollisionGroupEngineData*>(group2->getEngineData());

  auto broadPhaseAlg1 = data1->getFCLCollisionManager();
  auto broadPhaseAlg2 = data2->getFCLCollisionManager();

  CollisionData collData;
  convert(option, collData.request);

  broadPhaseAlg1->collide(broadPhaseAlg2, &collData, collisionCallBack);

  convert(collData.result, result);

  return !result.contacts.empty();
}



namespace {

//==============================================================================
CollisionData::CollisionData()
{
  done = false;
}

//==============================================================================
bool collisionCallBack(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata)
{
  CollisionData* collData = static_cast<CollisionData*>(cdata);

  const fcl::CollisionRequest& request = collData->request;
        fcl::CollisionResult&  result  = collData->result;
//  FCLEngine*        cd      = collData->collisionDetector;
  // TODO(JS): take filter object instead of collision detector

  if (collData->done)
    return true;

  // Filtering
//  if (!cd->isCollidable(cd->findCollisionNode(o1), cd->findCollisionNode(o2)))
//    return collData->done;
  // TODO(JS): disabled until other functionalities are implemented

  // Perform narrow-phase detection
  fcl::collide(o1, o2, request, result);

  if (!request.enable_cost
      && (result.isCollision())
      && ((result.numContacts() >= request.num_max_contacts)
          || !collData->checkAllCollisions))
  {
    collData->done = true;
  }

  return collData->done;
}

//==============================================================================
void convert(const Option& fclResult, fcl::CollisionRequest& request)
{
  request.num_max_contacts = fclResult.maxNumContacts;
  request.enable_contact   = fclResult.enableContact;
#if FCL_VERSION_AT_LEAST(0,3,0)
  request.gjk_solver_type  = fcl::GST_LIBCCD;
#endif
}

//==============================================================================
void convert(const fcl::CollisionResult& fclResult, Result& result)
{
  result.contacts.resize(fclResult.numContacts());

  // TODO(JS): Check if there is contact point sufficiently close to the new
  // contact point

  for (auto i = 0u; i < result.contacts.size(); ++i)
    convert(fclResult.getContact(i), result.contacts[i]);
}

//==============================================================================
void convert(const fcl::Contact& fclContact, Contact& contact)
{
  Eigen::Vector3d point = FCLTypes::convertVector3(fclContact.pos);

  contact.point = point;
  contact.normal = -FCLTypes::convertVector3(fclContact.normal);
  contact.penetrationDepth = fclContact.penetration_depth;
  contact.triID1 = fclContact.b1;
  contact.triID2 = fclContact.b2;

  FCLCollisionGeometryUserData* userData1
      = static_cast<FCLCollisionGeometryUserData*>(fclContact.o1->getUserData());
  FCLCollisionGeometryUserData* userData2
      = static_cast<FCLCollisionGeometryUserData*>(fclContact.o2->getUserData());
  assert(userData1);
  assert(userData2);
  contact.shape1 = userData1->mShape;
  contact.shape2 = userData2->mShape;
  contact.collisionObject1 = userData1->mCollisionObject;
  contact.collisionObject2 = userData2->mCollisionObject;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
