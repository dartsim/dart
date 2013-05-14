/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/11/2013
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

#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"

#include "collision/CollisionNode.h"
#include "collision/CollisionDetector.h"

namespace collision
{

CollisionDetector::CollisionDetector() {
}

CollisionDetector::~CollisionDetector() {
    for(int i = 0; i < mCollisionNodes.size(); i++)
        if (mCollisionNodes[i])
            delete mCollisionNodes[i];
}

void CollisionDetector::addCollisionSkeletonNode(kinematics::BodyNode* _bodyNode,
                                                 bool _bRecursive) {
    if (_bRecursive == false || _bodyNode->getNumChildJoints() == 0) {
        CollisionNode* collNode = createCollisionNode(_bodyNode);
        collNode->setBodyNodeID(mCollisionNodes.size());
        mCollisionNodes.push_back(collNode);
    }
    else {
        addCollisionSkeletonNode(_bodyNode, false);

        for (int i = 0; i < _bodyNode->getNumChildJoints(); i++)
            addCollisionSkeletonNode(_bodyNode->getChildNode(i), true);
    }

    _rebuildBodyNodePairs();
    updateSkeletonSelfCollidableState();
    updateBodyNodeCollidableState();
}

void CollisionDetector::_rebuildBodyNodePairs() {
    // TODO: Need better way
    mCollisionNodePairs.clear();

    CollisionNodePair collisionNodePair;
    unsigned int numCollisionNodes = mCollisionNodes.size();

    for (unsigned int i = 0; i < numCollisionNodes; i++) {
        collisionNodePair.collisionNode1 = mCollisionNodes[i];

        for (unsigned int j = i + 1; j < numCollisionNodes; j++) {
            collisionNodePair.collisionNode2 = mCollisionNodes[j];
            collisionNodePair.collidable = true;
            mCollisionNodePairs.push_back(collisionNodePair);
        }
    }
}

void CollisionDetector::_setAllBodyNodePairsCollidable(bool _collidable) {
    unsigned int numCollisionNodes = mCollisionNodes.size();

    for (unsigned int i = 0; i < numCollisionNodes; ++i) {
        mCollisionNodePairs[i].collidable = _collidable;
    }
}

void CollisionDetector::updateSkeletonSelfCollidableState() {
    unsigned int numCollisionNodes = mCollisionNodePairs.size();
    CollisionNodePair itrCollisionNodePair;

    for (unsigned int i = 0; i < numCollisionNodes; ++i) {
        itrCollisionNodePair = mCollisionNodePairs[i];

        if (itrCollisionNodePair.collisionNode1->getBodyNode()->getSkel()
                == itrCollisionNodePair.collisionNode2->getBodyNode()->getSkel()) {
            if (itrCollisionNodePair.collisionNode1->getBodyNode()->getSkel()->getSelfCollidable() == false) {
                itrCollisionNodePair.collidable = false;
            }
        }
    }
}

void CollisionDetector::updateBodyNodeCollidableState() {
    unsigned int numCollisionNodes = mCollisionNodePairs.size();
    CollisionNodePair itrCollisionNodePair;

    for (unsigned int i = 0; i < numCollisionNodes; ++i) {
        itrCollisionNodePair = mCollisionNodePairs[i];

        if (itrCollisionNodePair.collisionNode1->getBodyNode()->getCollideState() == false
                || itrCollisionNodePair.collisionNode2->getBodyNode()->getCollideState() == false)
            itrCollisionNodePair.collidable = false;
    }
}

} // namespace collision
