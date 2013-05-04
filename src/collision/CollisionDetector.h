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

#ifndef COLLISION_CONLLISION_DETECTOR_H
#define COLLISION_CONLLISION_DETECTOR_H

#include <vector>
#include <map>
#include <Eigen/Dense>

namespace kinematics { class BodyNode; }

namespace collision
{

class CollisionNode;

/// @brief BodyNode pair
///
/// CollisionDetector creates all possible BodyNode pairs for collision
/// checking.
class CollisionNodePair
{
public:
    /// @brief
    CollisionNode* collisionNode1;

    /// @brief
    CollisionNode* collisionNode2;

    /// @brief Collision detector conduct collision detection if collidable is
    /// true.
    bool collidable;
};

/// @brief
class Contact
{
public:
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
    int bdID1;

    /// @brief
    int bdID2;

    /// @brief
    double penetrationDepth;

    // TODO: IN TEST
    kinematics::BodyNode *bd1;
    kinematics::BodyNode *bd2;
//    CollisionSkeletonNode *collisionSkeletonNode1;
//    CollisionSkeletonNode *collisionSkeletonNode2;
    int triID1;
    int triID2;
};

/// @brief
class CollisionDetector
{
public:
    /// @brief
    CollisionDetector();

    /// @brief
    virtual ~CollisionDetector();

    /// @brief
    virtual void addCollisionSkeletonNode(kinematics::BodyNode *_bd,
                                  bool _bRecursive = false);

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

protected:
    /// @brief
    void rebuildBodyNodePairs();

    /// @brief
    void setAllBodyNodePairsCollidable(bool _collidable);

    /// @brief
    void eliminateSelfCollision();

    /// @brief
    std::vector<Contact> mContacts;

    /// @brief
    std::vector<CollisionNode*> mCollisionNodes;

    /// @brief
    std::vector<CollisionNodePair> mCollisionNodePairs;

    /// @brief
    //std::map<const kinematics::BodyNode*, CollisionNode*> mBodyCollisionMap;
    //std::vector<std::vector<bool> > mActiveMatrix;

private:

};

} // namespace collision

#endif // COLLISION_CONLLISION_DETECTOR_H
