/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_COLLISION_BULLET_BULLETCOLLISIONNODE_H_
#define DART_COLLISION_BULLET_BULLETCOLLISIONNODE_H_

// Must be included before any Bullet headers.
#include "dart/config.h"

#include <vector>

#include <assimp/scene.h>
#include <btBulletCollisionCommon.h>
#include <bullet/BulletCollision/Gimpact/btGImpactShape.h>
#include <Eigen/Dense>

#include "dart/dynamics/SmartPointer.h"
#include "dart/dynamics/Shape.h"
#include "dart/collision/CollisionNode.h"

namespace dart {
namespace dynamics {
class BodyNode;
}  // namespace dynamics
}  // namespace dart

namespace dart {
namespace collision {

class BulletCollisionNode;
class BulletCollisionDetector;

struct BulletUserData {
    dynamics::BodyNode* bodyNode;
    dynamics::ConstShapePtr shape;
    BulletCollisionNode* btCollNode;
    BulletCollisionDetector* btCollDet;
};

/// @brief class BulletCollisionNode
class BulletCollisionNode : public CollisionNode
{
public:
    /// @brief Constructor
    explicit BulletCollisionNode(dynamics::BodyNode* _bodyNode);

    /// @brief Destructor
    virtual ~BulletCollisionNode();

    /// @brief Update transformation of all the bullet collision objects.
    void updateBulletCollisionObjects();

    /// @brief Get number of bullet collision objects
    int getNumBulletCollisionObjects() const;

    /// @brief Get bullet collision object whose index is _i
    btCollisionObject* getBulletCollisionObject(int _i);

private:
    /// @brief Bullet collision objects
    std::vector<btCollisionObject*> mbtCollsionObjects;
};

/// @brief Create Bullet mesh from assimp3 mesh
btGImpactMeshShape* _createMesh(
    const Eigen::Vector3d& _scale, const aiScene* _mesh);

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_BULLET_BULLETCOLLISIONNODE_H_
