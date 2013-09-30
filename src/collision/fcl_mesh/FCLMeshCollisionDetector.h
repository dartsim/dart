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

#ifndef DART_COLLISION_FCL_MESH_COLLISION_DETECTOR_H
#define DART_COLLISION_FCL_MESH_COLLISION_DETECTOR_H

#include <vector>
#include <map>
#include <fcl/BVH/BVH_model.h>

#include "collision/CollisionDetector.h"
#include "collision/fcl_mesh/tri_tri_intersection_test.h"

namespace fcl { class CollisionResult; }

namespace dart {
namespace dynamics { class BodyNode; }
namespace collision {

class FCLMeshCollisionNode;

//class FCLContact : public Contact
//{
//public:
//    dynamics::BodyNode *bd1;
//    dynamics::BodyNode *bd2;
//    CollisionSkeletonNode *collisionSkeletonNode1;
//    CollisionSkeletonNode *collisionSkeletonNode2;
//    int triID1;
//    int triID2;
//    /*        bool isAdjacent(ContactPoint &otherPt){
//        //  return (((((((bd1==otherPt.bd1 && triID1==otherPt.triID1) || bd2==otherPt.bd2) && triID2==otherPt.triID2) || bd1==otherPt.bd2) && triID1==otherPt.triID2) || bd2==otherPt.bd1) && triID2==otherPt.triID1);
//        }
//        */
//};


class FCLMeshCollisionDetector : public CollisionDetector
{
public:
    /// @brief
    FCLMeshCollisionDetector() {}

    /// @brief
    virtual ~FCLMeshCollisionDetector();

    virtual CollisionNode* createCollisionNode(dynamics::BodyNode* _bodyNode);

    // Documentation inherited
    virtual bool detectCollision(bool _checkAllCollisions, bool _calculateContactPoints);

    virtual bool detectCollision(CollisionNode* _node1,
                                CollisionNode* _node2,
                                bool _calculateContactPoints);

    /// @brief
    void draw();
};



inline bool Vec3fCmp(fcl::Vec3f& v1, fcl::Vec3f& v2)
{
    if(v1[0]!=v2[0])
        return v1[0]<v2[0];
    else if(v1[1]!=v2[1])
        return v1[1]<v2[1];
    else
        return v1[2]<v2[2];
}

} // namespace collision
} // namespace dart

#endif // #ifndef DART_COLLISION_FCL_MESH_COLLISION_DETECTOR_H
