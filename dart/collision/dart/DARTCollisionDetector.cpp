/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include "dart/collision/dart/KIDOCollisionDetector.h"

#include <vector>

#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/collision/dart/KIDOCollide.h"

namespace kido {
namespace collision {

KIDOCollisionDetector::KIDOCollisionDetector()
  : CollisionDetector() {
}

KIDOCollisionDetector::~KIDOCollisionDetector() {
}

CollisionNode* KIDOCollisionDetector::createCollisionNode(
    dynamics::BodyNode* _bodyNode) {
  return new CollisionNode(_bodyNode);
}

bool KIDOCollisionDetector::detectCollision(bool /*_checkAllCollisions*/,
                                            bool /*_calculateContactPoints*/) {
  clearAllContacts();

  // Set all the body nodes are not in colliding
  for (size_t i = 0; i < mCollisionNodes.size(); i++)
    mCollisionNodes[i]->getBodyNode()->setColliding(false);

  std::vector<Contact> contacts;

  for (size_t i = 0; i < mCollisionNodes.size(); i++) {
    for (size_t j = i + 1; j < mCollisionNodes.size(); j++) {
      CollisionNode* collNode1 = mCollisionNodes[i];
      CollisionNode* collNode2 = mCollisionNodes[j];
      dynamics::BodyNode* BodyNode1 = collNode1->getBodyNode();
      dynamics::BodyNode* BodyNode2 = collNode2->getBodyNode();

      if (!isCollidable(collNode1, collNode2))
        continue;

      for (size_t k = 0; k < BodyNode1->getNumCollisionShapes(); k++) {
        for (size_t l = 0; l < BodyNode2->getNumCollisionShapes(); l++) {
          int currContactNum = mContacts.size();

          contacts.clear();
          collide(BodyNode1->getCollisionShape(k),
                  BodyNode1->getTransform()
                  * BodyNode1->getCollisionShape(k)->getLocalTransform(),
                  BodyNode2->getCollisionShape(l),
                  BodyNode2->getTransform()
                  * BodyNode2->getCollisionShape(l)->getLocalTransform(),
                  &contacts);

          size_t numContacts = contacts.size();

          for (unsigned int m = 0; m < numContacts; ++m) {
            Contact contactPair;
            contactPair = contacts[m];
            contactPair.bodyNode1 = BodyNode1;
            contactPair.bodyNode2 = BodyNode2;
            assert(contactPair.bodyNode1.lock() != nullptr);
            assert(contactPair.bodyNode2.lock() != nullptr);

            mContacts.push_back(contactPair);
          }

          std::vector<bool> markForDeletion(numContacts, false);
          for (size_t m = 0; m < numContacts; m++) {
            for (size_t n = m + 1; n < numContacts; n++) {
              Eigen::Vector3d diff =
                  mContacts[currContactNum + m].point -
                  mContacts[currContactNum + n].point;
              if (diff.dot(diff) < 1e-6) {
                markForDeletion[m] = true;
                break;
              }
            }
          }

          for (int m = numContacts - 1; m >= 0; m--)
          {
            if (markForDeletion[m])
              mContacts.erase(mContacts.begin() + currContactNum + m);
          }
        }
      }
    }
  }

  for (size_t i = 0; i < mContacts.size(); ++i)
  {
    // Set these two bodies are in colliding
    mContacts[i].bodyNode1.lock()->setColliding(true);
    mContacts[i].bodyNode2.lock()->setColliding(true);
  }

  return !mContacts.empty();
}

bool KIDOCollisionDetector::detectCollision(CollisionNode* _collNode1,
                                            CollisionNode* _collNode2,
                                            bool /*_calculateContactPoints*/) {
  std::vector<Contact> contacts;
  dynamics::BodyNode* BodyNode1 = _collNode1->getBodyNode();
  dynamics::BodyNode* BodyNode2 = _collNode2->getBodyNode();

  for (size_t i = 0; i < BodyNode1->getNumCollisionShapes(); i++) {
    for (size_t j = 0; j < BodyNode2->getNumCollisionShapes(); j++) {
      collide(BodyNode1->getCollisionShape(i),
              BodyNode1->getTransform()
              * BodyNode1->getCollisionShape(i)->getLocalTransform(),
              BodyNode2->getCollisionShape(j),
              BodyNode2->getTransform()
              * BodyNode2->getCollisionShape(j)->getLocalTransform(),
              &contacts);
    }
  }

  return contacts.size() > 0 ? true : false;
}

}  // namespace collision
}  // namespace kido
