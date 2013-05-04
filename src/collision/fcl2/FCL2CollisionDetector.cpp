/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/01/2013
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

#include "kinematics/Shape.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"

#include "collision/fcl2/FCL2CollisionNode.h"
#include "collision/fcl2/FCL2CollisionDetector.h"

namespace collision
{

FCL2CollisionDetector::FCL2CollisionDetector()
    : CollisionDetector()
{
}

FCL2CollisionDetector::~FCL2CollisionDetector()
{
}

CollisionNode*FCL2CollisionDetector::createCollisionNode(kinematics::BodyNode* _bodyNode)
{
    CollisionNode* collisionNode = NULL;

    collisionNode = new FCL2CollisionNode(_bodyNode);

    return collisionNode;
}

bool FCL2CollisionDetector::checkCollision(bool _checkAllCollisions,
                                          bool _calculateContactPoints)
{
    clearAllContacts();

    //    evalRT();
    //    _otherNode->evalRT();

    fcl::CollisionResult result;

    // only evaluate contact points if data structure for returning the contact
    // points was provided
    fcl::CollisionRequest request;
    request.enable_contact = true;
    request.num_max_contacts = 10;
//    request.enable_cost;
//    request.num_max_cost_sources;
//    request.use_approximate_cost;

    unsigned int numCollisionNodePairs = mCollisionNodePairs.size();
    fcl::CollisionGeometry* collGeom1 = NULL;
    fcl::CollisionGeometry* collGeom2 = NULL;
    fcl::Transform3f transf1;
    fcl::Transform3f transf2;

    for (unsigned int i = 0; i < numCollisionNodePairs; ++i)
    {
        const CollisionNodePair& collisionNodePair = mCollisionNodePairs[i];

        if (collisionNodePair.collidable == false)
            continue;

        collGeom1 = static_cast<FCL2CollisionNode*>(collisionNodePair.collisionNode1)->mCollisionGeometry;
        collGeom2 = static_cast<FCL2CollisionNode*>(collisionNodePair.collisionNode2)->mCollisionGeometry;

        Eigen::Matrix4d mWorldTrans1 = collisionNodePair.collisionNode1->mBodyNode->getWorldTransform();
        mWorldTrans1 = mWorldTrans1 * collisionNodePair.collisionNode1->mBodyNode->getCollisionShape()->getTransform().matrix();
        transf1 = fcl::Transform3f(fcl::Matrix3f(mWorldTrans1(0,0), mWorldTrans1(0,1), mWorldTrans1(0,2),
                                                 mWorldTrans1(1,0), mWorldTrans1(1,1), mWorldTrans1(1,2),
                                                 mWorldTrans1(2,0), mWorldTrans1(2,1), mWorldTrans1(2,2)),
                                   fcl::Vec3f(mWorldTrans1(0,3), mWorldTrans1(1,3), mWorldTrans1(2,3)));

        Eigen::Matrix4d mWorldTrans2 = collisionNodePair.collisionNode2->mBodyNode->getWorldTransform();
        mWorldTrans2 = mWorldTrans2 * collisionNodePair.collisionNode2->mBodyNode->getCollisionShape()->getTransform().matrix();
        transf2 = fcl::Transform3f(fcl::Matrix3f(mWorldTrans2(0,0), mWorldTrans2(0,1), mWorldTrans2(0,2),
                                                 mWorldTrans2(1,0), mWorldTrans2(1,1), mWorldTrans2(1,2),
                                                 mWorldTrans2(2,0), mWorldTrans2(2,1), mWorldTrans2(2,2)),
                                   fcl::Vec3f(mWorldTrans2(0,3), mWorldTrans2(1,3), mWorldTrans2(2,3)));

        fcl::collide(collGeom1, transf1, collGeom2, transf2, request, result);

        unsigned int numContacts = result.numContacts();
        for (unsigned int j = 0; j < numContacts; ++j)
        {
            const fcl::Contact& contact = result.getContact(j);

            Contact contactPair;
            contactPair.point(0) = contact.pos[0];
            contactPair.point(1) = contact.pos[1];
            contactPair.point(2) = contact.pos[2];
            contactPair.normal(0) = contact.normal[0];
            contactPair.normal(1) = contact.normal[1];
            contactPair.normal(2) = contact.normal[2];
//            contactPair.bodyNode1 = bodyNodePair.bodyNode1;
//            contactPair.bodyNode2 = bodyNodePair.bodyNode2;
            contactPair.collisionNode1 = collisionNodePair.collisionNode1;
            contactPair.collisionNode2 = collisionNodePair.collisionNode2;
            contactPair.bdID1 = collisionNodePair.collisionNode1->mBodyNodeID;
            contactPair.bdID2 = collisionNodePair.collisionNode2->mBodyNodeID;
            contactPair.penetrationDepth = contact.penetration_depth;

            mContacts.push_back(contactPair);
        }
    }

    return !mContacts.empty();
}

} // namespace collision
