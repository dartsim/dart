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
#include "dart/collision/fcl/FCLCollisionObjectData.h"
#include "dart/collision/fcl/FCLCollisionGroupData.h"

namespace dart {
namespace collision {

namespace {

bool checkPair(fcl::CollisionObject* o1,
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
FCLEnginePtr FCLEngine::create()
{
  return FCLEnginePtr(new FCLEngine());
}

//==============================================================================
FCLEngine::FCLEngine()
{
  // Do nothing
}

//==============================================================================
FCLEngine::~FCLEngine()
{
  // Do nothing
}

//==============================================================================
const std::string& FCLEngine::getTypeStatic()
{
  static const std::string& type("FCL");
  return type;
}

//==============================================================================
const std::string& FCLEngine::getType() const
{
  return getTypeStatic();
}

//==============================================================================
std::unique_ptr<CollisionObjectData> FCLEngine::createCollisionObjectData(
    CollisionObject* parent,
    const dynamics::ShapePtr& shape)
{
  return std::unique_ptr<CollisionObjectData>(
        new FCLCollisionObjectData(this, parent, shape));
}

//==============================================================================
std::unique_ptr<CollisionGroupData> FCLEngine::createCollisionGroupData(
    CollisionGroup* parent,
    const CollisionObjectPtrs& collObjects)
{
  return std::unique_ptr<CollisionGroupData>(
        new FCLCollisionGroupData(this, parent, collObjects));
}

//==============================================================================
bool FCLEngine::detect(
    CollisionObjectData* objectData1,
    CollisionObjectData* objectData2,
    const Option& option, Result& result)
{
  result.contacts.clear();

  assert(objectData1->getEngine()->getType() == FCLEngine::getTypeStatic());
  assert(objectData2->getEngine()->getType() == FCLEngine::getTypeStatic());

  auto castedData1 = static_cast<const FCLCollisionObjectData*>(objectData1);
  auto castedData2 = static_cast<const FCLCollisionObjectData*>(objectData2);

  auto fclCollObj1 = castedData1->getFCLCollisionObject();
  auto fclCollObj2 = castedData2->getFCLCollisionObject();

  FCLCollisionData collData(&option, &result);
  checkPair(fclCollObj1, fclCollObj2, &collData);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLEngine::detect(
    CollisionObjectData* objectData,
    CollisionGroupData* groupData,
    const Option& option, Result& result)
{
  result.contacts.clear();

  assert(objectData);
  assert(groupData);
  assert(objectData->getEngine()->getType() == FCLEngine::getTypeStatic());
  assert(groupData->getEngine()->getType() == FCLEngine::getTypeStatic());

  auto castedObjData = static_cast<FCLCollisionObjectData*>(objectData);
  auto castedGrpData = static_cast<FCLCollisionGroupData*>(groupData);

  auto fclObject = castedObjData->getFCLCollisionObject();
  auto broadPhaseAlg = castedGrpData->getFCLCollisionManager();

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg->collide(fclObject, &collData, checkPair);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLEngine::detect(CollisionGroupData* groupData,
                       const Option& option, Result& result)
{
  result.contacts.clear();

  assert(groupData);
  assert(groupData->getEngine()->getType() == FCLEngine::getTypeStatic());

  auto castedData = static_cast<FCLCollisionGroupData*>(groupData);

  auto broadPhaseAlg = castedData->getFCLCollisionManager();

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg->collide(&collData, checkPair);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLEngine::detect(CollisionGroupData* groupData1,
                       CollisionGroupData* groupData2,
                       const Option& option, Result& result)
{
  result.contacts.clear();

  assert(groupData1);
  assert(groupData2);
  assert(groupData1->getEngine()->getType() == FCLEngine::getTypeStatic());
  assert(groupData2->getEngine()->getType() == FCLEngine::getTypeStatic());

  auto castedData1 = static_cast<FCLCollisionGroupData*>(groupData1);
  auto castedData2 = static_cast<FCLCollisionGroupData*>(groupData2);

  auto broadPhaseAlg1 = castedData1->getFCLCollisionManager();
  auto broadPhaseAlg2 = castedData2->getFCLCollisionManager();

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg1->collide(broadPhaseAlg2, &collData, checkPair);

  return !result.contacts.empty();
}



namespace {

//==============================================================================
bool checkPair(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata)
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

  // Clear previous results
  fclResult.clear();

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

  if (0 == numContacts)
    return;

  const double ZERO = 0.000001;
  const double ZERO2 = ZERO*ZERO;

  std::vector<bool> markForDeletion(numContacts, false);

  // mark all the repeated points
  for (auto i = 0u; i < numContacts - 1; ++i)
  {
    const auto& contact1 = fclResult.getContact(i);

    for (auto j = i + 1u; j < numContacts; ++j)
    {
      const auto& contact2 = fclResult.getContact(j);

      const auto diff = contact1.pos - contact2.pos;

      if (diff.length() < 3 * ZERO2)
      {
        markForDeletion[i] = true;
        break;
      }
    }
  }

  // remove all the co-linear contact points
  for (auto i = 0u; i < numContacts; ++i)
  {
    if (markForDeletion[i])
      continue;

    const auto& contact1 = fclResult.getContact(i);

    for (auto j = i + 1u; j < numContacts; ++j)
    {
      if (markForDeletion[j])
        continue;

      const auto& contact2 = fclResult.getContact(j);

      for (auto k = j + 1u; k < numContacts; ++k)
      {
        if (markForDeletion[k])
          continue;

        const auto& contact3 = fclResult.getContact(k);

        const auto va = contact1.pos - contact2.pos;
        const auto vb = contact1.pos - contact3.pos;
        const auto v = va.cross(vb);

        if (v.length() < ZERO2)
        {
          markForDeletion[i] = true;
          break;
        }
      }
    }
  }

  for (size_t i = 0; i < numContacts; ++i)
  {
    if (!markForDeletion[i])
    {
      const auto fclContact = fclResult.getContact(i);
      result.contacts.push_back(convertContact(fclContact, o1, o2));
    }
  }

  std::cout << "=================" << std::endl;
  std::cout << "# of contacts:" << result.contacts.size() << std::endl;
  std::cout << "=================" << std::endl;
  for (auto i = 0u; i < result.contacts.size(); ++i)
  {
    auto contact = result.contacts[i];

    std::cout << "Contact (" << i << ")" << std::endl;
    std::cout << "Point : " << contact.point.transpose() << std::endl;
    std::cout << "Normal: " << contact.normal.transpose() << std::endl;
    std::cout << "Depth : " << contact.penetrationDepth << std::endl;
    std::cout << std::endl;
  }
  std::cout << "=================" << std::endl;
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
  contact.collisionObject1 = userData1->mCollisionObject;
  contact.collisionObject2 = userData2->mCollisionObject;

  return contact;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
