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
#include "kinematics/BodyNode.h"
#include "Eigen/Dense"
#include <vector>
#include "tri_tri_intersection_test.h"
#include <map>

namespace collision_checking {
    class CollisionSkeletonNode;
    struct ContactTriangle
    {
        Vec3f v1, v2, v3, u1, u2, u3;
    };
    struct ContactPoint {
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        Eigen::Vector3d force;
        kinematics::BodyNode *bd1;
        kinematics::BodyNode *bd2;
        CollisionSkeletonNode *collisionSkeletonNode1;
        CollisionSkeletonNode *collisionSkeletonNode2;
        int bdID1;
        int bdID2;
        int triID1;
        int triID2;
        ContactTriangle contactTri;

        double penetrationDepth;
        /*        bool isAdjacent(ContactPoint &otherPt){
        //  return (((((((bd1==otherPt.bd1 && triID1==otherPt.triID1) || bd2==otherPt.bd2) && triID2==otherPt.triID2) || bd1==otherPt.bd2) && triID1==otherPt.triID2) || bd2==otherPt.bd1) && triID2==otherPt.triID1);
        }
        */
    };

    class CollisionSkeletonNode {
    public:
        BVHModel<RSS>* mMesh;
        kinematics::BodyNode *mBodyNode;
        AABB mAABB;
        int mBodynodeID;
        

        Vec3f mR[3];
        Vec3f mT;
        Eigen::MatrixXd mWorldTrans;
        CollisionSkeletonNode(kinematics::BodyNode* _bodyNode);
        virtual ~CollisionSkeletonNode();

        int checkCollision(CollisionSkeletonNode* _otherNode, std::vector<ContactPoint>& _result, int _max_num_contact);
        void evalRT();
        
        int evalContactPosition(BVH_CollideResult& _result, CollisionSkeletonNode* _other, int _idx, Eigen::Vector3d& _contactPosition1, Eigen::Vector3d& _contactPosition2, ContactTriangle& _contactTri);
        void drawCollisionSkeletonNode(bool _bTrans = true);
        void drawCollisionTriangle(int _tri);

    private:
        inline int FFtest(Vec3f& r1, Vec3f& r2, Vec3f& r3, Vec3f& R1, Vec3f& R2, Vec3f& R3, Vec3f& res1, Vec3f& res2);
        inline bool EFtest(Vec3f& p0, Vec3f&p1, Vec3f& r1, Vec3f& r2, Vec3f& r3, Vec3f& p);
        inline Vec3f TransformVertex(Vec3f& _v);
        inline double triArea(Vec3f p1, Vec3f p2, Vec3f p3);
    };

    class SkeletonCollision {
    public:
        SkeletonCollision() { mNumTriIntersection=0;}
        virtual ~SkeletonCollision();
        void addCollisionSkeletonNode(kinematics::BodyNode *_bd, bool _bRecursive = false);
        inline void clearAllContacts(){ mContactPointList.clear(); }
        inline void clearAllCollisionSkeletonNode() {mCollisionSkeletonNodeList.clear();}
        inline ContactPoint& getContact(int idx) { return mContactPointList[idx]; }
        inline int getNumContact(){ return mContactPointList.size();}
        inline int getNumTriangleIntersection(){return mNumTriIntersection;}
        void checkCollision(bool bConsiderGround = false);
        void draw();
        CollisionSkeletonNode* getCollisionSkeletonNode(kinematics::BodyNode *_bodyNode){
            if(mBodyNodeHash.find(_bodyNode)!=mBodyNodeHash.end())
                return mBodyNodeHash[_bodyNode];
            else
                return NULL;
        }
        
    public:
        int mNumTriIntersection;
        std::vector<ContactPoint> mContactPointList;
        std::vector<CollisionSkeletonNode*> mCollisionSkeletonNodeList; 
        std::map<kinematics::BodyNode*, CollisionSkeletonNode*> mBodyNodeHash;
    };

    inline bool CollisionSkeletonNode::EFtest(Vec3f& p0, Vec3f&p1, Vec3f& r1, Vec3f& r2, Vec3f& r3, Vec3f& p)
    {
        double ZERO1 = 0.00000001;
        Vec3f n = (r3-r1).cross(r2-r1);
        double s = (p1 - p0).dot(n);
        if(std::abs(s)<ZERO1)return false;
        double t = (r1-p0).dot(n)/s;
        Vec3f tmp=p0+(p1-p0)*t;
        if(t>=0&&t<=1&&
            (((tmp-r1).cross(r2-r1)).dot(n)>=0)&&
            (((tmp-r2).cross(r3-r2)).dot(n)>=0)&&
            (((tmp-r3).cross(r1-r3)).dot(n)>=0)
            )
        {
             p=tmp;
            return true;
        }
        return false;
    }

    inline bool Vec3fCmp(Vec3f& v1, Vec3f& v2)
    {
        if(v1[0]!=v2[0])
            return v1[0]<v2[0];
        else if(v1[1]!=v2[1])
            return v1[1]<v2[1];
        else
            return v1[2]<v2[2];
    }

    inline int CollisionSkeletonNode::FFtest(Vec3f& r1, Vec3f& r2, Vec3f& r3, Vec3f& R1, Vec3f& R2, Vec3f& R3, Vec3f& res1, Vec3f& res2)
    {
        float U0[3], U1[3], U2[3], V0[3], V1[3], V2[3], RES1[3], RES2[3];
        SET(U0, r1);
        SET(U1, r2);
        SET(U2, r3);
        SET(V0, R1);
        SET(V1, R2);
        SET(V2, R3);
        
        int contactResult = tri_tri_intersect(V0, V1, V2, U0, U1, U2, RES1, RES2);
        
        SET(res1, RES1);
        SET(res2, RES2);
        return contactResult;
        /*
        int count1 = 0, count2 = 0, count = 0;
        //Vec3f p1 = Vec3f(0, 0, 0), p2 = Vec3f(0, 0, 0);
        Vec3f tmp;
        Vec3f p[6], p1[4], p2[4];

        if(EFtest(r1, r2, R1, R2, R3, tmp))
        {
            p[count] = tmp;
            count++;
        }
        if(EFtest(r2, r3, R1, R2, R3, tmp))
        {
            p[count] = tmp;
            count++;
        }
        if(EFtest(r3, r1, R1, R2, R3, tmp))
        {
            p[count] = tmp;
            count++;
        }
        
        if(EFtest(R1, R2, r1, r2, r3, tmp))
        {
            p[count] = tmp;
            count++;
        }
        if(EFtest(R2, R3, r1, r2, r3, tmp))
        {
            p[count] = tmp;
            count++;
        }
        if(EFtest(R3, R1, r1, r2, r3, tmp))
        {
            p[count] = tmp;
            count++;
        }
        if(count==0) return false;
        else
        {
           
            res1 = Vec3f(100000, 1000000, 1000000);
            res2 = -res1;
            for(int i=0; i<count;i++)
            {
                if(Vec3fCmp(p[i], res1))res1 = p[i];
                if(!Vec3fCmp(p[i], res2))res2 = p[i];
            }
            return true;
        }
        */
    }  

    inline Vec3f CollisionSkeletonNode::TransformVertex( Vec3f& _v )
    {
        Eigen::Vector3d vv(_v[0], _v[1], _v[2]);
        Eigen::Vector3d res = mWorldTrans.topLeftCorner(3, 3) * vv + mWorldTrans.col(3).head(3);
        return Vec3f(res[0], res[1], res[2]);
    }
    
    inline double CollisionSkeletonNode::triArea(Vec3f p1, Vec3f p2, Vec3f p3) {
        Vec3f a = p2 - p1;
        Vec3f b = p3 - p1;
        double aMag = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
        double bMag = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
        double dp = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        double area =  0.5 * sqrt(aMag * bMag - dp * dp);
        return area;
    }

} // namespace collision

#endif
