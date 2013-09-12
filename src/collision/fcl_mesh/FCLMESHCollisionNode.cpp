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

#include <iostream>

#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/BVH/BVH_model.h>

#include "dynamics/BodyNode.h"
#include "dynamics/Shape.h"
#include "dynamics/MeshShape.h"
#include "dynamics/EllipsoidShape.h"
#include "dynamics/CylinderShape.h"
#include "dynamics/BodyNode.h"

#include "renderer/LoadOpengl.h"

#include "collision/fcl_mesh/CollisionShapes.h"
#include "collision/fcl_mesh/FCLMESHCollisionDetector.h"
#include "collision/fcl_mesh/FCLMESHCollisionNode.h"

namespace dart {
namespace collision {

FCLMESHCollisionNode::FCLMESHCollisionNode(dynamics::BodyNode* _bodyNode)
    : CollisionNode(_bodyNode)
{
    for(int i = 0; i < _bodyNode->getNumCollisionShapes(); i++) {

        dynamics::Shape *shape = _bodyNode->getCollisionShape(i);
        fcl::Transform3f shapeTransform = getFclTransform(shape->getLocalTransform());

        switch (shape->getShapeType()) {
            case dynamics::Shape::P_ELLIPSOID:
            {
                dynamics::EllipsoidShape* ellipsoid = dynamic_cast<dynamics::EllipsoidShape*>(shape);

                if (ellipsoid->isSphere())
                {
                    fcl::BVHModel<fcl::OBBRSS>* mesh = new fcl::BVHModel<fcl::OBBRSS>;
                    fcl::generateBVHModel<fcl::OBBRSS>(*mesh, fcl::Sphere(shape->getDim()[0]*0.5), shapeTransform, 10, 10);
                    mMeshes.push_back(mesh);
                }
                else {
                    mMeshes.push_back(createEllipsoid<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2], shapeTransform));
                }
                break;
            }
            case dynamics::Shape::P_BOX:
                mMeshes.push_back(createCube<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2], shapeTransform));
                break;
            case dynamics::Shape::P_CYLINDER:
            {
                dynamics::CylinderShape *cylinder = dynamic_cast<dynamics::CylinderShape *>(shape);
                if(cylinder) {
                    double radius = cylinder->getRadius();
                    double height = cylinder->getHeight();
                    mMeshes.push_back(createCylinder<fcl::OBBRSS>(radius, radius, height, 16, 16, shapeTransform));
                }
                break;
            }
            case dynamics::Shape::P_MESH:
            {
                dynamics::MeshShape *shapeMesh = dynamic_cast<dynamics::MeshShape *>(shape);
                if(shapeMesh) {
                    mMeshes.push_back(createMesh<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2], shapeMesh->getMesh(), shapeTransform));
                }
                break;
            }
            default:
                std::cout << "ERROR: Collision checking does not support " << _bodyNode->getName() << "'s Shape type\n";
                break;
        }
    }
}

FCLMESHCollisionNode::~FCLMESHCollisionNode()
{
    for(int i = 0; i < mMeshes.size(); i++)
        delete mMeshes[i];
}

bool FCLMESHCollisionNode::checkCollision(
        FCLMESHCollisionNode* _otherNode,
        std::vector<Contact>* _contactPoints,
        int _num_max_contact)
{
    evalRT();
    _otherNode->evalRT();
    bool collision = false;

    for(int i = 0; i < mMeshes.size(); i++)
    for(int j = 0; j < _otherNode->mMeshes.size(); j++) {

        fcl::CollisionResult res;
        fcl::CollisionRequest req;

        req.enable_contact = _contactPoints; // only evaluate contact points if data structure for returning the contact points was provided
        req.num_max_contacts = _num_max_contact;
        fcl::collide(mMeshes[i],
                     mFclWorldTrans,
                     _otherNode->mMeshes[j],
                     _otherNode->mFclWorldTrans,
                     req, res);

        if(res.isCollision()) {
            collision = true;
        }
        if(!_contactPoints) {
            return collision;
        }

        int numCoplanarContacts = 0;
        int numNoContacts = 0;
        int numContacts = 0;

        std::vector<Contact> unfilteredContactPoints;
        unfilteredContactPoints.reserve(res.numContacts());

        for(int k = 0; k < res.numContacts(); k++) {
            // for each pair of intersecting triangles, we create two contact points
            Contact pair1, pair2;
//            pair1.bd1 = mBodyNode;
//            pair1.bd2 = _otherNode->mBodyNode;
//            pair1.bdID1 = this->mBodyNodeID;
//            pair1.bdID2 = _otherNode->mBodyNodeID;
            pair1.collisionNode1 = this;
            pair1.collisionNode2 = _otherNode;
            fcl::Vec3f v;
            pair1.triID1 = res.getContact(k).b1;
            pair1.triID2 = res.getContact(k).b2;
            pair1.penetrationDepth = res.getContact(k).penetration_depth;
            pair2 = pair1;
            int contactResult = evalContactPosition(res.getContact(k), mMeshes[i], _otherNode->mMeshes[j],
                                                    mFclWorldTrans, _otherNode->mFclWorldTrans, pair1.point, pair2.point);
            if(contactResult == COPLANAR_CONTACT) {
                numCoplanarContacts++;
                //                if(numContacts != 0 || numCoplanarContacts > 1)
                if (numContacts > 2)
                    continue;
            } else if(contactResult == NO_CONTACT) {
                numNoContacts++;
                continue;
            } else {
                numContacts++;
            }
            v = -res.getContact(k).normal;
            pair1.normal = Eigen::Vector3d(v[0], v[1], v[2]);
            pair2.normal = Eigen::Vector3d(v[0], v[1], v[2]);

            unfilteredContactPoints.push_back(pair1);
            unfilteredContactPoints.push_back(pair2);
        }

        const double ZERO = 0.000001;
        const double ZERO2 = ZERO*ZERO;

        std::vector<bool> markForDeletion(unfilteredContactPoints.size(), false);

        // mark all the repeated points
        for (unsigned int k = 0; k < unfilteredContactPoints.size(); k++) {
            for (unsigned int l = k + 1; l < unfilteredContactPoints.size(); l++) {
                Eigen::Vector3d diff = unfilteredContactPoints[k].point - unfilteredContactPoints[l].point;
                if (diff.dot(diff) < 3 * ZERO2) {
                    markForDeletion[k] = true;
                    break;
                }
            }
        }

        // remove all the co-linear contact points
        bool bremove;
        for (unsigned int k = 0; k < unfilteredContactPoints.size(); k++) {
            if(markForDeletion[k])
                continue;
            for (unsigned int l = 0; l < unfilteredContactPoints.size(); l++) {
                if (l == k || markForDeletion[l])
                    continue;
                if (markForDeletion[k])
                    break;
                for (int m = l + 1; m < unfilteredContactPoints.size(); m++) {
                    if (k == m)
                        continue;
                    Eigen::Vector3d  v = (unfilteredContactPoints[k].point - unfilteredContactPoints[l].point).cross(unfilteredContactPoints[k].point - unfilteredContactPoints[m].point);
                    if (v.dot(v) < ZERO2 && ((unfilteredContactPoints[k].point - unfilteredContactPoints[l].point).dot(unfilteredContactPoints[k].point - unfilteredContactPoints[m].point) < 0)) {
                        markForDeletion[k] = true;
                        break;
                    }
                }
            }
        }

        for (unsigned int k = 0; k < unfilteredContactPoints.size(); k++) {
            if(!markForDeletion[k]) {
                _contactPoints->push_back(unfilteredContactPoints[k]);
            }
        }
    }

    return collision;
}

void FCLMESHCollisionNode::evalRT() {
    mWorldTrans = mBodyNode->getWorldTransform();
    mFclWorldTrans = getFclTransform(mWorldTrans);
}

fcl::Transform3f FCLMESHCollisionNode::getFclTransform(const Eigen::Isometry3d& _m) {
    return fcl::Transform3f(fcl::Matrix3f(_m(0,0), _m(0,1), _m(0,2),
                                          _m(1,0), _m(1,1), _m(1,2),
                                          _m(2,0), _m(2,1), _m(2,2)),
                            fcl::Vec3f(_m(0,3), _m(1,3), _m(2,3)));
}

int FCLMESHCollisionNode::evalContactPosition(
        const fcl::Contact& _fclContact,
        fcl::BVHModel<fcl::OBBRSS>* _mesh1,
        fcl::BVHModel<fcl::OBBRSS>* _mesh2,
        const fcl::Transform3f& _transform1,
        const fcl::Transform3f& _transform2,
        Eigen::Vector3d& _contactPosition1,
        Eigen::Vector3d& _contactPosition2) {
    int id1 = _fclContact.b1;
    int id2 = _fclContact.b2;
    fcl::Triangle tri1 = _mesh1->tri_indices[id1];
    fcl::Triangle tri2 = _mesh2->tri_indices[id2];

    fcl::Vec3f v1, v2, v3, p1, p2, p3;
    v1 = _mesh1->vertices[tri1[0]];
    v2 = _mesh1->vertices[tri1[1]];
    v3 = _mesh1->vertices[tri1[2]];

    p1 = _mesh2->vertices[tri2[0]];
    p2 = _mesh2->vertices[tri2[1]];
    p3 = _mesh2->vertices[tri2[2]];

    fcl::Vec3f contact1, contact2;
    v1 = _transform1.transform(v1);
    v2 = _transform1.transform(v2);
    v3 = _transform1.transform(v3);
    p1 = _transform2.transform(p1);
    p2 = _transform2.transform(p2);
    p3 = _transform2.transform(p3);
    int testRes = FFtest(v1, v2, v3, p1, p2, p3, contact1, contact2);

    if (testRes == COPLANAR_CONTACT) {
        double area1 = triArea(v1, v2, v3);
        double area2 = triArea(p1, p2, p3);
        //            cout << "this node = " << this->mBodynodeID << " other node = " << _other->mBodynodeID << endl;
        if (area1 < area2)
            contact1 = v1 + v2 + v3;
        else
            contact1 = p1 + p2 + p3;
        contact1[0] /= 3.0;
        contact1[1] /= 3.0;
        contact1[2] /= 3.0;
        contact2 = contact1;
        //            cout << contact1[0] << " " << contact1[1] << " " << contact1[2] << endl;
    }
    _contactPosition1 = Eigen::Vector3d(contact1[0], contact1[1], contact1[2]);
    _contactPosition2 = Eigen::Vector3d(contact2[0], contact2[1], contact2[2]);
    return testRes;
}

void FCLMESHCollisionNode::drawCollisionSkeletonNode(bool _bTrans) {
    evalRT();
    double M[16];
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            M[j*4+i] = mWorldTrans(i, j);
    fcl::Vec3f v1, v2, v3;
    glPushMatrix();
    if(_bTrans)
        glMultMatrixd(M);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_TRIANGLES);
    for(int i = 0; i < mMeshes.size(); i++) {
        for(int j = 0; j < mMeshes[i]->num_tris; j++) {
            fcl::Triangle tri = mMeshes[i]->tri_indices[j];
            glVertex3f(mMeshes[i]->vertices[tri[0]][0], mMeshes[i]->vertices[tri[0]][1], mMeshes[i]->vertices[tri[0]][2]);
            glVertex3f(mMeshes[i]->vertices[tri[1]][0], mMeshes[i]->vertices[tri[1]][1], mMeshes[i]->vertices[tri[1]][2]);
            glVertex3f(mMeshes[i]->vertices[tri[2]][0], mMeshes[i]->vertices[tri[2]][1], mMeshes[i]->vertices[tri[2]][2]);
        }
    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopMatrix();
}

} // namespace collision
} // namespace dart
