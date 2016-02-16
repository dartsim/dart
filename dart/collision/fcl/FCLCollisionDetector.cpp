/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>,
 *            Tobias Kunz <tobias@gatech.edu>
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

#include "dart/collision/fcl/FCLCollisionDetector.h"

#include <vector>

#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/collision/fcl/FCLCollisionNode.h"
#include "dart/collision/fcl/FCLTypes.h"

#define FCL_VERSION_AT_LEAST(x,y,z) \
  (FCL_MAJOR_VERSION > x || (FCL_MAJOR_VERSION >= x && \
  (FCL_MINOR_VERSION > y || (FCL_MINOR_VERSION >= y && \
  FCL_PATCH_VERSION >= z))))

namespace dart {
namespace collision {

//==============================================================================
// Collision data stores the collision request and the result given by
// collision algorithm.
struct CollisionData
{
  CollisionData() { done = false; }

  // Collision request
  fcl::CollisionRequest request;

  // Collision result
  fcl::CollisionResult result;

  // FCL collision detector
  FCLCollisionDetector* collisionDetector;

  // Whether the collision iteration can stop
  bool done;
};

//==============================================================================
bool collisionCallBack(fcl::CollisionObject* _o1,
                       fcl::CollisionObject* _o2,
                       void* _cdata)
{
  CollisionData* cdata = static_cast<CollisionData*>(_cdata);
  const fcl::CollisionRequest& request = cdata->request;
  fcl::CollisionResult& result = cdata->result;
  FCLCollisionDetector* cd = cdata->collisionDetector;

  if(cdata->done)
    return true;

  // Filtering
  if (!cd->isCollidable(cd->findCollisionNode(_o1), cd->findCollisionNode(_o2)))
    return cdata->done;

  // Perform narrow-phase detection
  fcl::collide(_o1, _o2, request, result);

  if (!request.enable_cost
      && (result.isCollision())
      && (result.numContacts() >= request.num_max_contacts))
  {
    cdata->done = true;
  }

  return cdata->done;
}

//==============================================================================
FCLCollisionDetector::FCLCollisionDetector()
  : CollisionDetector(),
    mBroadPhaseAlg(new fcl::DynamicAABBTreeCollisionManager())
{
}

//==============================================================================
FCLCollisionDetector::~FCLCollisionDetector()
{
  delete mBroadPhaseAlg;
}

//==============================================================================
CollisionNode* FCLCollisionDetector::createCollisionNode(
    dynamics::BodyNode* _bodyNode)
{
  // This collision node will be removed at destructor of CollisionDetector.
  FCLCollisionNode* collNode = new FCLCollisionNode(_bodyNode);

  for (size_t i = 0; i < collNode->getNumCollisionObjects(); ++i)
  {
    fcl::CollisionObject* collObj = collNode->getCollisionObject(i);
    mBroadPhaseAlg->registerObject(collObj);
  }

  // We do initial setup here for FCL's broad-phase algorithm since we don't
  // have any other place to put this before performing collision detection
  // frequently.
  mBroadPhaseAlg->setup();

  return collNode;
}

//==============================================================================
bool isClose(const Eigen::Vector3d& _point1, const Eigen::Vector3d& _point2)
{
  if ((_point1 - _point2).squaredNorm() < 1e-12)
    return true;
  return false;
}

//==============================================================================
bool hasClosePoint(const std::vector<Contact>& _contacts,
                   const Eigen::Vector3d& _point)
{
  for (const auto& contact : _contacts)
  {
    if (isClose(contact.point, _point))
      return true;
  }

  return false;
}

//==============================================================================
bool FCLCollisionDetector::detectCollision(bool /*_checkAllCollisions*/,
                                           bool _calculateContactPoints)
{
  // TODO(JS): _checkAllCollisions
  clearAllContacts();

  // Set all the body nodes are not in colliding
  for (size_t i = 0; i < mCollisionNodes.size(); i++)
    mCollisionNodes[i]->getBodyNode()->setColliding(false);

  // Update all the transformations of the collision nodes
  for (auto& collNode : mCollisionNodes)
    static_cast<FCLCollisionNode*>(collNode)->updateFCLCollisionObjects();
  mBroadPhaseAlg->update();

  CollisionData collData;
  collData.request.enable_contact = _calculateContactPoints;
  // TODO: Uncomment below once we strict to use fcl 0.3.0 or greater
  // collData.request.gjk_solver_type = fcl::GST_LIBCCD;
  collData.request.num_max_contacts = getNumMaxContacts();
  collData.collisionDetector = this;

  // Perform broad-phase collision detection. Narrow-phase collision detection
  // will be handled by the collision callback function.
  mBroadPhaseAlg->collide(&collData, collisionCallBack);

  const size_t numContacts = collData.result.numContacts();
  for (size_t m = 0; m < numContacts; ++m)
  {
    const fcl::Contact& contact = collData.result.getContact(m);

    Eigen::Vector3d point = FCLTypes::convertVector3(contact.pos);

    if (hasClosePoint(mContacts, point))
      continue;

    Contact contactPair;
    contactPair.point = point;
    contactPair.normal = -FCLTypes::convertVector3(contact.normal);
    contactPair.bodyNode1 = findCollisionNode(contact.o1)->getBodyNode();
    contactPair.bodyNode2 = findCollisionNode(contact.o2)->getBodyNode();
    contactPair.triID1 = contact.b1;
    contactPair.triID2 = contact.b2;
    contactPair.penetrationDepth = contact.penetration_depth;
    assert(contactPair.bodyNode1.lock());
    assert(contactPair.bodyNode2.lock());

    mContacts.push_back(contactPair);
  }

  for (size_t i = 0; i < mContacts.size(); ++i)
  {
    // Set these two bodies are in colliding
    mContacts[i].bodyNode1.lock()->setColliding(true);
    mContacts[i].bodyNode2.lock()->setColliding(true);
  }

  return !mContacts.empty();
}

//==============================================================================
bool FCLCollisionDetector::detectCollision(CollisionNode* _node1,
                                           CollisionNode* _node2,
                                           bool _calculateContactPoints)
{
  // TODO(JS): function not implemented
  assert(false);
  return false;
}

//==============================================================================
CollisionNode* FCLCollisionDetector::findCollisionNode(
    const fcl::CollisionGeometry* _fclCollGeom) const
{
  int numCollNodes = mCollisionNodes.size();
  for (int i = 0; i < numCollNodes; ++i)
  {
    FCLCollisionNode* collisionNode =
        static_cast<FCLCollisionNode*>(mCollisionNodes[i]);
    for (size_t j = 0; j < collisionNode->getNumCollisionObjects(); j++)
    {
#if FCL_VERSION_AT_LEAST(0,3,0)
      if (collisionNode->getCollisionObject(j)->collisionGeometry().get()
          == _fclCollGeom)
#else
      if (collisionNode->getCollisionObject(j)->getCollisionGeometry()
          == _fclCollGeom)
#endif
        return mCollisionNodes[i];
    }
  }
  return NULL;
}

//==============================================================================
FCLCollisionNode* FCLCollisionDetector::findCollisionNode(
    const fcl::CollisionObject* _fclCollObj) const
{
  FCLCollisionGeometryUserData* userData = static_cast<FCLCollisionGeometryUserData*>(_fclCollObj->getUserData());
  return userData->mFclCollNode;
}

}  // namespace collision
}  // namespace dart
