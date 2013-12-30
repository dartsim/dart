/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

namespace dart {
namespace collision {

FCLCollisionDetector::FCLCollisionDetector()
  : CollisionDetector() {
}

FCLCollisionDetector::~FCLCollisionDetector() {
}

CollisionNode* FCLCollisionDetector::createCollisionNode(
    dynamics::BodyNode* _bodyNode) {
  return new FCLCollisionNode(_bodyNode);
}

bool FCLCollisionDetector::detectCollision(bool _checkAllCollisions,
                                           bool _calculateContactPoints) {
  // TODO(JS): _checkAllCollisions
  clearAllContacts();

  fcl::CollisionResult result;

  // only evaluate contact points if data structure for returning the contact
  // points was provided
  fcl::CollisionRequest request;
  request.enable_contact = _calculateContactPoints;
  request.num_max_contacts = mNumMaxContacts;
  //    request.enable_cost;
  //    request.num_max_cost_sources;
  //    request.use_approximate_cost;

  for (int i = 0; i < mCollisionNodes.size(); i++) {
    for (int j = i + 1; j < mCollisionNodes.size(); j++) {
      result.clear();
      FCLCollisionNode* collNode1 =
          static_cast<FCLCollisionNode*>(mCollisionNodes[i]);
      FCLCollisionNode* collNode2 =
          static_cast<FCLCollisionNode*>(mCollisionNodes[j]);

      if (!isCollidable(collNode1, collNode2))
        continue;

      for (int k = 0; k < collNode1->getNumCollisionGeometries(); k++) {
        for (int l = 0; l < collNode2->getNumCollisionGeometries(); l++) {
          int currContactNum = mContacts.size();
          fcl::collide(collNode1->getCollisionGeometry(k),
                       collNode1->getFCLTransform(k),
                       collNode2->getCollisionGeometry(l),
                       collNode2->getFCLTransform(l),
                       request, result);

          unsigned int numContacts = result.numContacts();

          for (unsigned int m = 0; m < numContacts; ++m) {
            const fcl::Contact& contact = result.getContact(m);

            Contact contactPair;
            contactPair.point(0) = contact.pos[0];
            contactPair.point(1) = contact.pos[1];
            contactPair.point(2) = contact.pos[2];
            contactPair.normal(0) = contact.normal[0];
            contactPair.normal(1) = contact.normal[1];
            contactPair.normal(2) = contact.normal[2];
            contactPair.collisionNode1 = findCollisionNode(contact.o1);
            contactPair.collisionNode2 = findCollisionNode(contact.o2);
            assert(contactPair.collisionNode1 != NULL);
            assert(contactPair.collisionNode2 != NULL);
//            contactPair.bdID1 =
//                collisionNodePair.collisionNode1->getBodyNodeID();
//            contactPair.bdID2 =
//                collisionNodePair.collisionNode2->getBodyNodeID();
            contactPair.penetrationDepth = contact.penetration_depth;

            mContacts.push_back(contactPair);
          }

          std::vector<bool>markForDeletion(numContacts, false);
          for (int m = 0; m < numContacts; m++) {
            for (int n = m + 1; n < numContacts; n++) {
              Eigen::Vector3d diff =
                  mContacts[currContactNum + m].point -
                  mContacts[currContactNum + n].point;
              if (diff.dot(diff) < 1e-6) {
                markForDeletion[m] = true;
                break;
              }
            }
          }
          for (int m = numContacts - 1; m >= 0; m--) {
            if (markForDeletion[m])
              mContacts.erase(mContacts.begin() + currContactNum + m);
          }
        }
      }
    }
  }
  return !mContacts.empty();
}

bool FCLCollisionDetector::detectCollision(CollisionNode* _node1,
                                           CollisionNode* _node2,
                                           bool _calculateContactPoints) {
  // TODO(JS): function not implemented
  assert(false);
  return false;
}

CollisionNode* FCLCollisionDetector::findCollisionNode(
    const fcl::CollisionGeometry* _fclCollGeom) const {
  int numCollNodes = mCollisionNodes.size();
  for (int i = 0; i < numCollNodes; ++i) {
    FCLCollisionNode* collisionNode =
        static_cast<FCLCollisionNode*>(mCollisionNodes[i]);
    for (int j = 0; j < collisionNode->getNumCollisionGeometries(); j++) {
      if (collisionNode->getCollisionGeometry(j) == _fclCollGeom)
        return mCollisionNodes[i];
    }
  }
  return NULL;
}

}  // namespace collision
}  // namespace dart
