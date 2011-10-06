/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Chen Tang <ctang40@gatech.edu>
 * Date: 09/30/2011
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

#ifndef COLLISION_SKELTON_H
#define COLLISION_SKELTON_H

#include "collision.h"
#include "../kinematics/BodyNode.h"
#include "../../thirdparty/eigen-3.0.1/Eigen/Dense"
#include <vector>

#define ODE_STYLE 1

namespace collision_checking {
    struct ContactPoint {
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        kinematics::BodyNode *bd1;
        kinematics::BodyNode *bd2;
    };

    struct CollisionSkeletonNode{
        BVHModel<RSS>* cdmesh;
        kinematics::BodyNode *bodyNode;
        AABB aabb;
        


        CollisionSkeletonNode(kinematics::BodyNode* _bodyNode);
        virtual ~CollisionSkeletonNode();

        int checkCollision(CollisionSkeletonNode* otherNode, std::vector<ContactPoint>& result, int max_num_contact);
        void evalRT(Eigen::MatrixXd mat, Vec3f R[3], Vec3f& T);
    };

   

    class SkeletonCollision {
    public:
        SkeletonCollision() {}
        virtual ~SkeletonCollision();
        void addCollisionSkeletonNode(kinematics::BodyNode *_bd, bool _bRecursive = false);
        inline void clearAllContacts(){ mContactPointList.clear(); }
        inline void clearAllCollisionSkeletonNode() {mCollisionSkeletonNodeList.clear();}
        inline ContactPoint& getContact(int idx) { return mContactPointList[idx]; }
        inline int getNumContact(){ return mContactPointList.size();}
        void checkCollision(bool bConsiderGround = false);
        

    public:
        
        std::vector<ContactPoint> mContactPointList;
        std::vector<CollisionSkeletonNode*> mCollisionSkeletonNodeList; 
    };

    
} // namespace collision

#endif