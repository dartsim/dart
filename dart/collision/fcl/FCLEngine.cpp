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

bool collisionCallBack(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata);

void postProcess(const fcl::CollisionResult& fclResult,
                 fcl::CollisionObject* o1,
                 fcl::CollisionObject* o2,
                 Result& result);

void convertOption(const Option& fclOption, fcl::CollisionRequest& request);

Contact convertContact(const fcl::Contact& fclContact,
                       fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2);

/// Collision data stores the collision request and the result given by
/// collision algorithm.
struct FCLCollisionData
{
  /// FCL collision request
  fcl::CollisionRequest mFclRequest;

  /// FCL collision result
  fcl::CollisionResult mFclResult;

  /// Collision option of DART
  const Option* mOption;

  /// Collision result of DART
  Result* mResult;

  /// Whether the collision iteration can stop
  bool done;

  /// Constructor
  FCLCollisionData(
      const Option* option = nullptr,
      Result* result = nullptr)
    : mOption(option),
      mResult(result),
      done(false)
  {
    if (mOption)
      convertOption(*mOption, mFclRequest);
  }
};

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
std::unique_ptr<CollisionObjectEngineData> FCLEngine::createCollisionObjectData(
    CollisionObject* parent,
    const dynamics::ShapePtr& shape)
{
  return std::unique_ptr<CollisionObjectEngineData>(
        new FCLCollisionObjectEngineData(parent, shape));
}

//==============================================================================
std::unique_ptr<CollisionGroupEngineData> FCLEngine::createCollisionGroupData(
    const CollisionObjectPtrs& collObjects)
{
  return std::unique_ptr<CollisionGroupEngineData>(
        new FCLCollisionGroupEngineData(collObjects));
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

  FCLCollisionData collData(&option, &result);
  collisionCallBack(fclCollObj1, fclCollObj2, &collData);

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

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg->collide(fclObject, &collData, collisionCallBack);

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

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg->collide(&collData, collisionCallBack);

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

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg1->collide(broadPhaseAlg2, &collData, collisionCallBack);

  return !result.contacts.empty();
}



namespace {

//==============================================================================
bool collisionCallBack(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata)
{
  FCLCollisionData* collData = static_cast<FCLCollisionData*>(cdata);

  const fcl::CollisionRequest& fclRequest = collData->mFclRequest;
        fcl::CollisionResult&  fclResult  = collData->mFclResult;
        Result&                result     = *(collData->mResult);
//  FCLEngine*        cd      = collData->collisionDetector;
  // TODO(JS): take filter object instead of collision detector

  if (collData->done)
    return true;

  // Filtering
//  if (!cd->isCollidable(cd->findCollisionNode(o1), cd->findCollisionNode(o2)))
//    return collData->done;
  // TODO(JS): disabled until other functionalities are implemented

  // Perform narrow-phase detection
  fcl::collide(o1, o2, fclRequest, fclResult);

  if (!fclRequest.enable_cost
      && (fclResult.isCollision())
      && ((fclResult.numContacts() >= fclRequest.num_max_contacts)))
          //|| !collData->checkAllCollisions))
    // TODO(JS): checkAllCollisions should be in FCLCollisionData
  {
    collData->done = true;
  }

  postProcess(fclResult, o1, o2, result);

  return collData->done;
}

//==============================================================================
void postProcess(const fcl::CollisionResult& fclResult,
                 fcl::CollisionObject* o1,
                 fcl::CollisionObject* o2,
                 Result& result)
{
  auto numContacts = fclResult.numContacts();
  for (auto i = 0u; i < numContacts; ++i)
  {
    const auto fclContact = fclResult.getContact(i);
    result.contacts.push_back(convertContact(fclContact, o1, o2));
  }
}

//==============================================================================
void convertOption(const Option& fclOption, fcl::CollisionRequest& request)
{
  request.num_max_contacts = fclOption.maxNumContacts;
  request.enable_contact   = fclOption.enableContact;
#if FCL_VERSION_AT_LEAST(0,3,0)
  request.gjk_solver_type  = fcl::GST_LIBCCD;
#endif
}

//==============================================================================
Contact convertContact(const fcl::Contact& fclContact,
                       fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2)
{
  Contact contact;

  Eigen::Vector3d point = FCLTypes::convertVector3(fclContact.pos);

  contact.point = point;
  contact.normal = -FCLTypes::convertVector3(fclContact.normal);
  contact.penetrationDepth = fclContact.penetration_depth;
  contact.triID1 = fclContact.b1;
  contact.triID2 = fclContact.b2;

  FCLCollisionObjectUserData* userData1
      = static_cast<FCLCollisionObjectUserData*>(o1->getUserData());
  FCLCollisionObjectUserData* userData2
      = static_cast<FCLCollisionObjectUserData*>(o2->getUserData());
  assert(userData1);
  assert(userData2);
  contact.shape1 = userData1->mShape;
  contact.shape2 = userData2->mShape;
  contact.collisionObject1 = userData1->mCollisionObject;
  contact.collisionObject2 = userData2->mCollisionObject;

  return contact;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
