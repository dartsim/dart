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

#define ODE_STYLE 0

namespace collision_checking {
    struct ContactPoint {
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        kinematics::BodyNode *bd1;
        kinematics::BodyNode *bd2;
        int bdID1;
        int bdID2;
    };

    struct CollisionSkeletonNode{
        BVHModel<RSS>* cdmesh;
        kinematics::BodyNode *bodyNode;
        AABB aabb;
        int bodynodeID;

        CollisionSkeletonNode(kinematics::BodyNode* _bodyNode);
        virtual ~CollisionSkeletonNode();

        int checkCollision(CollisionSkeletonNode* otherNode, std::vector<ContactPoint>& result, int max_num_contact);
        void evalRT(Eigen::MatrixXd mat, Vec3f R[3], Vec3f& T);
        Eigen::Vector3d evalContactPosition(BVH_CollideResult& result, CollisionSkeletonNode* other, int idx);

    private:
        inline bool FFtest(Vec3f& r1, Vec3f& r2, Vec3f& r3, Vec3f& R1, Vec3f& R2, Vec3f& R3, Vec3f& res);
        inline bool EFtest(Vec3f& p0, Vec3f&p1, Vec3f& r1, Vec3f& r2, Vec3f& r3, Vec3f& p);
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

   
    inline bool CollisionSkeletonNode::EFtest(Vec3f& p0, Vec3f&p1, Vec3f& r1, Vec3f& r2, Vec3f& r3, Vec3f& p)
    {
        printf("ok\n");
        const double ZERO = 0.00000001;
        Vec3f n = (r2-r1).cross(r3-r1);
        double s = (p1 - p0).dot(n);
        if(std::abs(s)<ZERO)return false;
        double t = (r1-p0).dot(n)/s;
        Vec3f tmp=p0+(p1-p0)*t;
        //printf("t %lf, %lf %lf %lf \n", t, dot(cross(r2-r1, tmp-r1), n), dot(cross(r3-r2, tmp-r2), n), dot(cross(r1-r3, tmp-r3), n));
        if(t>=0&&t<=1&&
            (((r2-r1).cross(tmp-r1)).dot(n)>0)&&
            (((r3-r2).cross(tmp-r2)).dot(n)>0)&&
            (((r1-r3).cross(tmp-r3)).dot(n)>0)
            )
        {
            p=tmp;
            return true;
        }
        return false;


    }

    inline bool CollisionSkeletonNode::FFtest(Vec3f& r1, Vec3f& r2, Vec3f& r3, Vec3f& R1, Vec3f& R2, Vec3f& R3, Vec3f& res)
    {
        int count = 0;
        Vec3f p = Vec3f(0);
        Vec3f tmp;

        if(EFtest(r1, r2, R1, R2, R3, tmp))
        {
            p+=tmp;
            count++;
        }
        if(EFtest(r2, r3, R1, R2, R3, tmp))
        {
            p+=tmp;
            count++;
        }
        if(EFtest(r3, r1, R1, R2, R3, tmp))
        {
            p+=tmp;
            count++;
        }
        if(EFtest(R1, R2, r1, r2, r3, tmp))
        {
            p+=tmp;
            count++;
        }
        if(EFtest(R2, R3, r1, r2, r3, tmp))
        {
            p+=tmp;
            count++;
        }
        if(EFtest(R3, R1, r1, r2, r3, tmp))
        {
            p+=tmp;
            count++;
        }
        if(count!=0){
            res = p*(1.0/count);
            return true;
        }

        else return false;

    }  
    
} // namespace collision




#endif
