/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Chen Tang <ctang40@gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef COLLISION_FCL_MESH_CONLLISION_NODE_H
#define COLLISION_FCL_MESH_CONLLISION_NODE_H

#include <vector>
#include <Eigen/Dense>
#include <fcl/collision.h>

#include "collision/CollisionNode.h"
#include "collision/CollisionDetector.h"
#include "collision/fcl_mesh/tri_tri_intersection_test.h"

namespace kinematics { class BodyNode; }

namespace collision
{

/// @brief
class FCLMESHCollisionNode : public CollisionNode
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FCLMESHCollisionNode(kinematics::BodyNode* _bodyNode);
    virtual ~FCLMESHCollisionNode();

    fcl::BVHModel<fcl::OBBRSS>* mMesh;
    fcl::AABB mAABB;

    fcl::Transform3f mFclWorldTrans;
    Eigen::Matrix4d mWorldTrans;

    int checkCollision(FCLMESHCollisionNode* _otherNode, std::vector<Contact>* _contactPoints, int _max_num_contact);
    void evalRT();

    int evalContactPosition(fcl::CollisionResult& _result, FCLMESHCollisionNode* _other, int _idx, Eigen::Vector3d& _contactPosition1, Eigen::Vector3d& _contactPosition2);

    void drawCollisionSkeletonNode(bool _bTrans = true);
    void drawCollisionTriangle(int _tri);

private:
    inline int FFtest(fcl::Vec3f& r1, fcl::Vec3f& r2, fcl::Vec3f& r3, fcl::Vec3f& R1, fcl::Vec3f& R2, fcl::Vec3f& R3, fcl::Vec3f& res1, fcl::Vec3f& res2);
    inline bool EFtest(fcl::Vec3f& p0, fcl::Vec3f&p1, fcl::Vec3f& r1, fcl::Vec3f& r2, fcl::Vec3f& r3, fcl::Vec3f& p);
    inline fcl::Vec3f TransformVertex(fcl::Vec3f& _v);
    inline double triArea(fcl::Vec3f p1, fcl::Vec3f p2, fcl::Vec3f p3);
};

inline bool FCLMESHCollisionNode::EFtest(fcl::Vec3f& p0,
                                     fcl::Vec3f& p1,
                                     fcl::Vec3f& r1,
                                     fcl::Vec3f& r2,
                                     fcl::Vec3f& r3,
                                     fcl::Vec3f& p)
{
    double ZERO1 = 0.00000001;
    fcl::Vec3f n = (r3-r1).cross(r2-r1);
    double s = (p1 - p0).dot(n);
    if(std::abs(s)<ZERO1)return false;
    double t = (r1-p0).dot(n)/s;
    fcl::Vec3f tmp=p0+(p1-p0)*t;
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

inline int FCLMESHCollisionNode::FFtest(fcl::Vec3f& r1, fcl::Vec3f& r2, fcl::Vec3f& r3, fcl::Vec3f& R1, fcl::Vec3f& R2, fcl::Vec3f& R3, fcl::Vec3f& res1, fcl::Vec3f& res2)
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
        //fcl::Vec3f p1 = fcl::Vec3f(0, 0, 0), p2 = fcl::Vec3f(0, 0, 0);
        fcl::Vec3f tmp;
        fcl::Vec3f p[6], p1[4], p2[4];

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

            res1 = fcl::Vec3f(100000, 1000000, 1000000);
            res2 = -res1;
            for(int i=0; i<count;i++)
            {
                if(fcl::Vec3fCmp(p[i], res1))res1 = p[i];
                if(!fcl::Vec3fCmp(p[i], res2))res2 = p[i];
            }
            return true;
        }
        */
}

inline fcl::Vec3f FCLMESHCollisionNode::TransformVertex( fcl::Vec3f& _v )
{
    Eigen::Vector3d vv(_v[0], _v[1], _v[2]);
    Eigen::Vector3d res = mWorldTrans.topLeftCorner<3,3>() * vv + mWorldTrans.col(3).head(3);
    return fcl::Vec3f(res[0], res[1], res[2]);
}

inline double FCLMESHCollisionNode::triArea(fcl::Vec3f p1, fcl::Vec3f p2, fcl::Vec3f p3) {
    fcl::Vec3f a = p2 - p1;
    fcl::Vec3f b = p3 - p1;
    double aMag = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    double bMag = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
    double dp = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    double area =  0.5 * sqrt(aMag * bMag - dp * dp);
    return area;
}


} // namespace collision

#endif // COLLISION_FCL2_CONLLISION_NODE_H
