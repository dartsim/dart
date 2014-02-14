/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Chen Tang <ctang40@gatech.edu>
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

#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"

#include <algorithm>
#include <cmath>

#include <fcl/collision.h>

#include "dart/renderer/LoadOpengl.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/collision/CollisionNode.h"
#include "dart/collision/fcl_mesh/CollisionShapes.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionNode.h"

namespace dart {
namespace collision {

FCLMeshCollisionDetector::FCLMeshCollisionDetector() {
}

FCLMeshCollisionDetector::~FCLMeshCollisionDetector() {
}

CollisionNode*FCLMeshCollisionDetector::createCollisionNode(
    dynamics::BodyNode* _bodyNode) {
  return new FCLMeshCollisionNode(_bodyNode);
}

bool FCLMeshCollisionDetector::detectCollision(bool _checkAllCollisions,
                                               bool _calculateContactPoints) {
  clearAllContacts();
  bool collision = false;

  FCLMeshCollisionNode* FCLMeshCollisionNode1 = NULL;
  FCLMeshCollisionNode* FCLMeshCollisionNode2 = NULL;

  for (int i = 0; i < mCollisionNodes.size(); i++)
    mCollisionNodes[i]->getBodyNode()->setColliding(false);

  for (int i = 0; i < mCollisionNodes.size(); i++) {
    FCLMeshCollisionNode1 =
        static_cast<FCLMeshCollisionNode*>(mCollisionNodes[i]);
    for (int j = i + 1; j < mCollisionNodes.size(); j++) {
      FCLMeshCollisionNode2 =
          static_cast<FCLMeshCollisionNode*>(mCollisionNodes[j]);
      if (!isCollidable(FCLMeshCollisionNode1, FCLMeshCollisionNode2))
        continue;
      if (FCLMeshCollisionNode1->detectCollision(
           FCLMeshCollisionNode2,
           _calculateContactPoints ? &mContacts : NULL,
           mNumMaxContacts)) {
        collision = true;
        mCollisionNodes[i]->getBodyNode()->setColliding(true);
        mCollisionNodes[j]->getBodyNode()->setColliding(true);

        if (!_checkAllCollisions)
          return true;
      }
    }
  }

  return collision;
}

bool FCLMeshCollisionDetector::detectCollision(CollisionNode* _node1,
                                               CollisionNode* _node2,
                                               bool _calculateContactPoints) {
  FCLMeshCollisionNode* collisionNode1 =
      static_cast<FCLMeshCollisionNode*>(_node1);
  FCLMeshCollisionNode* collisionNode2 =
      static_cast<FCLMeshCollisionNode*>(_node2);
  return collisionNode1->detectCollision(
        collisionNode2,
        _calculateContactPoints ? &mContacts : NULL,
        mNumMaxContacts);
}

void FCLMeshCollisionDetector::draw() {
  for (int i = 0; i < mCollisionNodes.size(); i++)
    static_cast<FCLMeshCollisionNode*>(
        mCollisionNodes[i])->drawCollisionSkeletonNode();
}

}  // namespace collision
}  // namespace dart
