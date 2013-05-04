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

#include <fcl/shape/geometric_shapes.h>
#include <fcl/BVH/BVH_model.h>

#include "kinematics/BodyNode.h"
#include "kinematics/Shape.h"

#include "collision/fcl2/FCL2CollisionNode.h"

namespace collision {

template<class BV>
fcl::BVHModel<BV>* createCube(float _sizeX, float _sizeY, float _sizeZ) //create a cube mesh for collision detection
{
    float n[6][3] =
    {
        {-1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 0.0, -1.0}
    };
    int faces[6][4] =
    {
        {0, 1, 2, 3},
        {3, 2, 6, 7},
        {7, 6, 5, 4},
        {4, 5, 1, 0},
        {5, 6, 2, 1},
        {7, 4, 0, 3}
    };
    float v[8][3];

    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -_sizeX / 2;
    v[4][0] = v[5][0] = v[6][0] = v[7][0] = _sizeX / 2;
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -_sizeY / 2;
    v[2][1] = v[3][1] = v[6][1] = v[7][1] = _sizeY / 2;
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = -_sizeZ / 2;
    v[1][2] = v[2][2] = v[5][2] = v[6][2] = _sizeZ / 2;

    fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
    fcl::Vec3f p1, p2, p3;
    model->beginModel();

    for (int i = 0; i < 6; i++) {
        p1 = fcl::Vec3f(v[faces[i][0]][0], v[faces[i][0]][1], v[faces[i][0]][2]);
        p2 = fcl::Vec3f(v[faces[i][1]][0], v[faces[i][1]][1], v[faces[i][1]][2]);
        p3 = fcl::Vec3f(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
        model->addTriangle(p1, p2, p3);
        p1 = fcl::Vec3f(v[faces[i][0]][0], v[faces[i][0]][1], v[faces[i][0]][2]);
        p2 = fcl::Vec3f(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
        p3 = fcl::Vec3f(v[faces[i][3]][0], v[faces[i][3]][1], v[faces[i][3]][2]);
        model->addTriangle(p1, p2, p3);
    }
    model->endModel();
    return model;
}//end createCube

FCL2CollisionNode::FCL2CollisionNode(kinematics::BodyNode* _bodyNode)
    : CollisionNode(_bodyNode)
{
    kinematics::Shape* shape = _bodyNode->getCollisionShape();

    switch (shape->getShapeType())
    {
        case kinematics::Shape::P_ELLIPSOID:
            mCollisionGeometry = new fcl::Sphere(shape->getDim()[0]*0.5);
            break;
        case kinematics::Shape::P_BOX:
//            mCollisionGeometry = new fcl::Box(shape->getDim()[0],
//                    shape->getDim()[1], shape->getDim()[2]);
            mCollisionGeometry = createCube<fcl::OBBRSS>(shape->getDim()[0],
                    shape->getDim()[1],
                    shape->getDim()[2]);
            break;
//        case kinematics::Shape::P_CYLINDER:
//        {
//            kinematics::ShapeCylinder *cylinder = dynamic_cast<kinematics::ShapeCylinder *>(shape);
//            if(cylinder) {
//                double radius = cylinder->getRadius();
//                double height = cylinder->getHeight();
//                mMesh = createCylinder<fcl::OBBRSS>(radius, radius, height, 16, 16);
//            }
//            break;
//        }
        case kinematics::Shape::P_MESH:
        {
//            kinematics::ShapeMesh *shapeMesh = dynamic_cast<kinematics::ShapeMesh *>(shape);
//            if(shapeMesh) {
//                mMesh = createMesh<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2], shapeMesh->getMesh());
//            }
            break;
        }
        default:
            std::cout << "ERROR: Collision checking does not support "
                      << _bodyNode->getName()
                      << "'s Shape type\n";
            break;
    }
}

FCL2CollisionNode::~FCL2CollisionNode()
{
}



} // namespace collision
