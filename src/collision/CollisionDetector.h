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

#ifndef COLLISION_CONLLISION_DETECTOR_H
#define COLLISION_CONLLISION_DETECTOR_H

#include <vector>
#include <Eigen/Dense>
#include "collision/CollisionNode.h"

namespace kinematics { class BodyNode; }

namespace collision {

class CollisionNode;

/// @brief
struct Contact {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief
    Eigen::Vector3d point;

    /// @brief
    Eigen::Vector3d normal;

    /// @brief
    Eigen::Vector3d force;

    /// @brief
    CollisionNode* collisionNode1;

    /// @brief
    CollisionNode* collisionNode2;

    /// @brief
    double penetrationDepth;

    // TODO: triID1 will be deprecated when we don't use fcl_mesh
    /// @brief
    int triID1;

    // TODO: triID2 will be deprecated when we don't use fcl_mesh
    /// @brief
    int triID2;
};

/// @brief
class CollisionDetector {
    // CONSTRUCTORS AND DESTRUCTOR ---------------------------------------------
public:
    /// @brief
    CollisionDetector();

    /// @brief
    virtual ~CollisionDetector();

public:
    /// @brief
    virtual void addCollisionSkeletonNode(kinematics::BodyNode *_bd,
                                          bool _bRecursive = false);

    /// @brief
    virtual CollisionNode* createCollisionNode(
            kinematics::BodyNode* _bodyNode) = 0;

    /// @brief
    virtual bool checkCollision(bool _checkAllCollisions,
                                bool _calculateContactPoints) = 0;

    /// @brief
    unsigned int getNumContacts() { return mContacts.size(); }

    /// @brief
    Contact& getContact(int _idx) { return mContacts[_idx]; }

    /// @brief
    void clearAllContacts() { mContacts.clear(); }

    /// @brief
    void updateSkeletonSelfCollidableState();

    /// @brief
    void updateBodyNodeCollidableState();

protected:
    /// @brief
    void _rebuildBodyNodePairs();

    /// @brief
    void _setAllBodyNodePairsCollidable(bool _collidable);

    /// @brief
    std::vector<Contact> mContacts;

    /// @brief
    std::vector<CollisionNode*> mCollisionNodes;

    /// @brief
    std::vector<CollisionNodePair> mCollisionNodePairs;

private:

};

} // namespace collision

#endif // COLLISION_CONLLISION_DETECTOR_H
