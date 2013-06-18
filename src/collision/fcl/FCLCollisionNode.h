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

#ifndef COLLISION_FCL_CONLLISION_NODE_H
#define COLLISION_FCL_CONLLISION_NODE_H

#include <Eigen/Dense>
#include <fcl/collision.h>
#include <fcl/BVH/BVH_model.h>

#include "collision/CollisionNode.h"

namespace kinematics { class BodyNode; }

namespace collision {

/// @brief
class FCLCollisionNode : public CollisionNode
{
public:
    /// @brief
    FCLCollisionNode(kinematics::BodyNode* _bodyNode);

    /// @brief
    virtual ~FCLCollisionNode();

    /// @brief
    int getNumCollisionGeometries() const {
        return mCollisionGeometries.size();
    }

    /// @brief
    fcl::CollisionGeometry* getCollisionGeometry(int _idx) const {
        return mCollisionGeometries[_idx];
    }

    /// @brief
    fcl::Transform3f getFCLTransform(int _idx) const;

protected:

private:
    /// @brief
    std::vector<fcl::CollisionGeometry*> mCollisionGeometries;
    std::vector<kinematics::Shape*> mShapes;
};

/// @brief
template<class BV>
fcl::BVHModel<BV>* createMesh(float _sizeX, float _sizeY, float _sizeZ,
                              const aiScene *_mesh);

/// @brief
template<class BV>
fcl::BVHModel<BV>* createEllipsoid(float _sizeX, float _sizeY, float _sizeZ);

} // namespace collision

#endif // COLLISION_FCL2_CONLLISION_NODE_H
