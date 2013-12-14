/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include <vector>

#include "dart/collision/bullet/BulletCollisionNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace collision {

struct CollisionFilter : public btOverlapFilterCallback {
  // return true when pairs need collision
  virtual bool needBroadphaseCollision(btBroadphaseProxy *_proxy0,
                                       btBroadphaseProxy *_proxy1) const   {
    assert((_proxy0 != NULL && _proxy1 != NULL) &&
           "Bullet broadphase overlapping pair proxies are NULL");

    bool collide = (_proxy0->m_collisionFilterGroup &
                    _proxy1->m_collisionFilterMask) != 0;
    collide = collide && (_proxy1->m_collisionFilterGroup &
                          _proxy0->m_collisionFilterMask);

    btCollisionObject* collObj0 =
        static_cast<btCollisionObject*>(_proxy0->m_clientObject);
    btCollisionObject* collObj1 =
        static_cast<btCollisionObject*>(_proxy1->m_clientObject);

    btUserData* userData0 =
        static_cast<btUserData*>(collObj0->getUserPointer());
    btUserData* userData1 =
        static_cast<btUserData*>(collObj1->getUserPointer());

//    if (!userData0->btCollDet->isCollidable(userData0->btCollNode,
//                                            userData1->btCollNode)) {
//      std::cout<< "false.\n";
//      return false;
//    }

    if (userData0->bodyNode == userData1->bodyNode)
      return false;

    if (!userData0->bodyNode->isCollidable())
      return false;

    if (!userData1->bodyNode->isCollidable())
      return false;

    if (userData0->bodyNode->getSkeleton() ==
        userData1->bodyNode->getSkeleton())
      if (!userData0->bodyNode->getSkeleton()->isSelfCollidable())
        return false;

    return collide;
  }
};

BulletCollisionDetector::BulletCollisionDetector() : CollisionDetector() {
  btVector3 worldAabbMin(-1000, -1000, -1000);
  btVector3 worldAabbMax(1000, 1000, 1000);
//  btBroadphaseInterface* broadphasePairCache =
//      new btAxisSweep3(worldAabbMin, worldAabbMax);
  btBroadphaseInterface* broadphasePairCache = new btDbvtBroadphase();
  btCollisionConfiguration* collisionConfiguration =
      new btDefaultCollisionConfiguration();
  btDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  mBulletCollisionWorld = new btCollisionWorld(dispatcher,
                                               broadphasePairCache,
                                               collisionConfiguration);

  btOverlapFilterCallback* filterCallback = new CollisionFilter();
  btOverlappingPairCache* pairCache = mBulletCollisionWorld->getPairCache();
  assert(pairCache != NULL);
  pairCache->setOverlapFilterCallback(filterCallback);
}

BulletCollisionDetector::~BulletCollisionDetector() {
}

CollisionNode* BulletCollisionDetector::createCollisionNode(
    dynamics::BodyNode* _bodyNode) {
  BulletCollisionNode* newBTCollNode = new BulletCollisionNode(_bodyNode);

  for (int i = 0; i < newBTCollNode->getNumBTCollisionObjects(); ++i) {
    btUserData* userData =
        static_cast<btUserData*>(
          newBTCollNode->getBTCollisionObject(i)->getUserPointer());
    userData->btCollDet = this;
    mBulletCollisionWorld->addCollisionObject(
          newBTCollNode->getBTCollisionObject(i));
  }

  return newBTCollNode;
}

struct btContactResultCB : public btCollisionWorld::ContactResultCallback {
  virtual btScalar addSingleResult(
      btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
      const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) {
    Contact contactPair;
    contactPair.point = convertVector3(cp.getPositionWorldOnA());
    contactPair.normal = convertVector3(cp.m_normalWorldOnB);
    contactPair.penetrationDepth = -cp.m_distance1;

    mContacts.push_back(contactPair);

    return 0;
  }

  std::vector<Contact> mContacts;
};

bool BulletCollisionDetector::detectCollision(bool _checkAllCollisions,
                                              bool _calculateContactPoints) {
  for (int i = 0; i < mCollisionNodes.size(); ++i)
    static_cast<BulletCollisionNode*>(
        mCollisionNodes[i])->updateBTCollisionObjects();

  btDispatcherInfo& dispatchInfo = mBulletCollisionWorld->getDispatchInfo();

  dispatchInfo.m_timeStep = 0.001;
  dispatchInfo.m_stepCount = 0;
  // dispatchInfo.m_debugDraw = getDebugDrawer();

  mBulletCollisionWorld->performDiscreteCollisionDetection();

//  std::cout << "Number of collision objects: "
//            << collWorld->getNumCollisionObjects() << std::endl;

  // TODO(Unknown): _checkAllCollisions
  clearAllContacts();

  int numManifolds = mBulletCollisionWorld->getDispatcher()->getNumManifolds();
  for (int i = 0; i < numManifolds; ++i) {
    btPersistentManifold* contactManifold =
        mBulletCollisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject* obA = contactManifold->getBody0();
    const btCollisionObject* obB = contactManifold->getBody1();

    btUserData* userDataA = static_cast<btUserData*>(obA->getUserPointer());
    btUserData* userDataB = static_cast<btUserData*>(obB->getUserPointer());

    int numContacts = contactManifold->getNumContacts();
    for (int j = 0; j < numContacts; j++) {
      btManifoldPoint& cp = contactManifold->getContactPoint(j);

      Contact contactPair;
      contactPair.point = convertVector3(cp.getPositionWorldOnA());
      contactPair.normal = convertVector3(cp.m_normalWorldOnB);
      contactPair.penetrationDepth = -cp.m_distance1;
      contactPair.collisionNode1 = userDataA->btCollNode;
      contactPair.collisionNode2 = userDataB->btCollNode;

      mContacts.push_back(contactPair);
    }
  }


//  for(int i = 0; i < mCollisionNodes.size(); i++) {
//    for(int j = i + 1; j < mCollisionNodes.size(); j++) {
//      //result.clear();
//      BulletCollisionNode* collNode1 =
//          static_cast<BulletCollisionNode*>(mCollisionNodes[i]);
//      BulletCollisionNode* collNode2 =
//          static_cast<BulletCollisionNode*>(mCollisionNodes[j]);

//      if (!isCollidable(collNode1, collNode2))
//        continue;

//      for(int k = 0; k < collNode1->getNumBTCollisionObjects(); k++) {
//        for(int l = 0; l < collNode2->getNumBTCollisionObjects(); l++) {
//          int currContactNum = mContacts.size();

//          btContactResultCB result;

//          //------------------------------------------------------------------
//          // Collide
//          //------------------------------------------------------------------
//          mBulletCollisionWorld->contactPairTest(
//                collNode1->getBTCollisionObject(k),
//                collNode2->getBTCollisionObject(l),
//                result);
//          //------------------------------------------------------------------

//          unsigned int numContacts = result.mContacts.size();

//          if (numContacts > 0)
//            std::cout << "Contact number: " << numContacts << std::endl;

//          for (int i = 0; i < numContacts; ++i) {
//            result.mContacts[i].collisionNode1 = collNode1;
//            result.mContacts[i].collisionNode2 = collNode2;
//            mContacts.push_back(result.mContacts[i]);
//          }

//          std::vector<bool> markForDeletion(numContacts, false);
//          for (int m = 0; m < numContacts; m++) {
//            for (int n = m + 1; n < numContacts; n++) {
//              Eigen::Vector3d diff =
//                  mContacts[currContactNum + m].point -
//                  mContacts[currContactNum + n].point;
//              if (diff.dot(diff) < 1e-6) {
//                markForDeletion[m] = true;
//                break;
//              }
//            }
//          }
//          for (int m = numContacts - 1; m >= 0; m--) {
//            if (markForDeletion[m])
//              mContacts.erase(mContacts.begin() + currContactNum + m);
//          }
//        }
//      }
//    }
//  }

  return !mContacts.empty();
}

bool BulletCollisionDetector::detectCollision(CollisionNode* _node1,
                                              CollisionNode* _node2,
                                              bool _calculateContactPoints) {
  assert(false);  // function not implemented
  return false;
}

}  // namespace collision
}  // namespace dart
