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
#include <fcl/BVH/BVH_model.h>

#include "kinematics/BodyNode.h"
#include "kinematics/Shape.h"
#include "kinematics/ShapeMesh.h"
#include "kinematics/ShapeCylinder.h"
#include "kinematics/BodyNode.h"

#include "renderer/LoadOpengl.h"

#include "collision/fcl_mesh/CollisionShapes.h"
#include "collision/fcl_mesh/FCLMESHCollisionDetector.h"
#include "collision/fcl_mesh/FCLMESHCollisionNode.h"

namespace collision {

FCLMESHCollisionNode::FCLMESHCollisionNode(kinematics::BodyNode* _bodyNode)
    : CollisionNode(_bodyNode),
      mMesh(NULL)
{
    kinematics::Shape *shape = _bodyNode->getCollisionShape();

    switch (shape->getShapeType()) {
        case kinematics::Shape::P_ELLIPSOID:
            mMesh = createEllipsoid<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2]);
            break;
        case kinematics::Shape::P_BOX:
            mMesh = createCube<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2]);
            break;
        case kinematics::Shape::P_CYLINDER:
        {
            kinematics::ShapeCylinder *cylinder = dynamic_cast<kinematics::ShapeCylinder *>(shape);
            if(cylinder) {
                double radius = cylinder->getRadius();
                double height = cylinder->getHeight();
                mMesh = createCylinder<fcl::OBBRSS>(radius, radius, height, 16, 16);
            }
            break;
        }
        case kinematics::Shape::P_MESH:
        {
            kinematics::ShapeMesh *shapeMesh = dynamic_cast<kinematics::ShapeMesh *>(shape);
            if(shapeMesh) {
                mMesh = createMesh<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2], shapeMesh->getMesh());
            }
            break;
        }
        default:
            std::cout << "ERROR: Collision checking does not support " << _bodyNode->getName() << "'s Shape type\n";
            break;
    }
}

FCLMESHCollisionNode::~FCLMESHCollisionNode()
{
    if (mMesh)
        delete mMesh;
}

int FCLMESHCollisionNode::checkCollision(
        FCLMESHCollisionNode* _otherNode,
        std::vector<Contact>* _contactPoints,
        int _num_max_contact)
{
    evalRT();
    _otherNode->evalRT();
    fcl::CollisionResult res;
    fcl::CollisionRequest req;

    req.enable_contact = _contactPoints; // only evaluate contact points if data structure for returning the contact points was provided
    req.num_max_contacts = _num_max_contact;
    fcl::collide(mMesh,
                 mFclWorldTrans,
                 _otherNode->mMesh,
                 _otherNode->mFclWorldTrans,
                 req, res);

    if(!_contactPoints) {
        return res.isCollision() ? 1 : 0;
    }

    int numCoplanarContacts = 0;
    int numNoContacts = 0;
    int numContacts = 0;

    std::vector<Contact> unfilteredContactPoints;
    unfilteredContactPoints.reserve(res.numContacts());

    for(int i = 0; i < res.numContacts();i++) {
        // for each pair of intersecting triangles, we create two contact points
        Contact pair1, pair2;
//        pair1.bd1 = mBodyNode;
//        pair1.bd2 = _otherNode->mBodyNode;
//        pair1.bdID1 = this->mBodyNodeID;
//        pair1.bdID2 = _otherNode->mBodyNodeID;
        pair1.collisionNode1 = this;
        pair1.collisionNode2 = _otherNode;
        fcl::Vec3f v;
        pair1.triID1 = res.getContact(i).b1;
        pair1.triID2 = res.getContact(i).b2;
        pair1.penetrationDepth = res.getContact(i).penetration_depth;
        pair2 = pair1;
        int contactResult = evalContactPosition(res, _otherNode, i,
                                                pair1.point, pair2.point);
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
        v = -res.getContact(i).normal;
        pair1.normal = Eigen::Vector3d(v[0], v[1], v[2]);
        pair2.normal = Eigen::Vector3d(v[0], v[1], v[2]);

        unfilteredContactPoints.push_back(pair1);
        unfilteredContactPoints.push_back(pair2);
    }

    const double ZERO = 0.000001;
    const double ZERO2 = ZERO*ZERO;

    std::vector<bool> markForDeletion(unfilteredContactPoints.size(), false);

    // mark all the repeated points
    for (unsigned int i = 0; i < unfilteredContactPoints.size(); i++) {
        for (unsigned int j = i + 1; j < unfilteredContactPoints.size(); j++) {
            Eigen::Vector3d diff = unfilteredContactPoints[i].point - unfilteredContactPoints[j].point;
            if (diff.dot(diff) < 3 * ZERO2) {
                markForDeletion[i] = true;
                break;
            }
        }
    }

    // remove all the co-linear contact points
    bool bremove;
    for (unsigned int i = 0; i < unfilteredContactPoints.size(); i++) {
        if(markForDeletion[i])
            continue;
        for (unsigned int j = 0; j < unfilteredContactPoints.size(); j++) {
            if (j == i || markForDeletion[j])
                continue;
            if (markForDeletion[i])
                break;
            for (int k = j + 1; k < unfilteredContactPoints.size(); k++) {
                if (i == k)
                    continue;
                Eigen::Vector3d  v = (unfilteredContactPoints[i].point - unfilteredContactPoints[j].point).cross(unfilteredContactPoints[i].point - unfilteredContactPoints[k].point);
                if (v.dot(v) < ZERO2 && ((unfilteredContactPoints[i].point - unfilteredContactPoints[j].point).dot(unfilteredContactPoints[i].point - unfilteredContactPoints[k].point) < 0)) {
                    markForDeletion[i] = true;
                    break;
                }
            }
        }
    }

    for (unsigned int i = 0; i < unfilteredContactPoints.size(); i++) {
        if(!markForDeletion[i]) {
            _contactPoints->push_back(unfilteredContactPoints[i]);
        }
    }

    ////////////////////////////////////////////////////////////////////////
    // TODO: SPECIAL TEST CODE FOR GAZEBO SUPPORT BRANCH !!!
    ////////////////////////////////////////////////////////////////////////
    kinematics::Shape* collShape = mBodyNode->getCollisionShape();
    if (!_contactPoints->empty()
            && collShape->getShapeType() == kinematics::Shape::P_ELLIPSOID)
    {
        //            ContactPoint everageContactPoint;
        //            unsigned int numContactPoint = _contactPoints->size();
        //            for (unsigned int i = 0; i < numContactPoint; ++i)
        //            {
        //                everageContactPoint.point += _contactPoints->at(i).point;
        //                everageContactPoint.normal += _contactPoints->at(i).normal;
        //            }
        //            everageContactPoint.point /= numContactPoint;
        //            everageContactPoint.normal.normalize();

        //            _contactPoints->clear();
        //            _contactPoints->push_back(everageContactPoint);
        //_contactPoints->resize();
    }
    ////////////////////////////////////////////////////////////////////////

    return res.numContacts();
}

void FCLMESHCollisionNode::evalRT() {
    mWorldTrans = mBodyNode->getWorldTransform();
    //Vector3d p = xformHom(mWorldTrans, mBodyNode->getCollisionShape()->getOffset());
    //mWorldTrans.block(0, 3, 3, 1) = p;
    mWorldTrans = mWorldTrans * mBodyNode->getCollisionShape()->getTransform().matrix();
    mFclWorldTrans = fcl::Transform3f(fcl::Matrix3f(mWorldTrans(0,0), mWorldTrans(0,1), mWorldTrans(0,2),
                                                    mWorldTrans(1,0), mWorldTrans(1,1), mWorldTrans(1,2),
                                                    mWorldTrans(2,0), mWorldTrans(2,1), mWorldTrans(2,2)),
                                      fcl::Vec3f(mWorldTrans(0,3), mWorldTrans(1,3), mWorldTrans(2,3)));
}

int FCLMESHCollisionNode::evalContactPosition(
        fcl::CollisionResult& _result,
        FCLMESHCollisionNode* _other,
        int _idx,
        Eigen::Vector3d& _contactPosition1,
        Eigen::Vector3d& _contactPosition2) {
    int id1, id2;
    fcl::Triangle tri1, tri2;
    FCLMESHCollisionNode* node1 = this;
    FCLMESHCollisionNode* node2 = _other;
    id1 = _result.getContact(_idx).b1;
    id2 = _result.getContact(_idx).b2;
    tri1 = node1->mMesh->tri_indices[id1];
    tri2 = node2->mMesh->tri_indices[id2];

    fcl::Vec3f v1, v2, v3, p1, p2, p3;
    v1 = node1->mMesh->vertices[tri1[0]];
    v2 = node1->mMesh->vertices[tri1[1]];
    v3 = node1->mMesh->vertices[tri1[2]];

    p1 = node2->mMesh->vertices[tri2[0]];
    p2 = node2->mMesh->vertices[tri2[1]];
    p3 = node2->mMesh->vertices[tri2[2]];

    fcl::Vec3f contact1, contact2;
    v1 = node1->TransformVertex(v1);
    v2 = node1->TransformVertex(v2);
    v3 = node1->TransformVertex(v3);
    p1 = node2->TransformVertex(p1);
    p2 = node2->TransformVertex(p2);
    p3 = node2->TransformVertex(p3);
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

void FCLMESHCollisionNode::drawCollisionTriangle(int _tri) {
    fcl::Triangle Tri = mMesh->tri_indices[_tri];
    glVertex3f(mMesh->vertices[Tri[0]][0], mMesh->vertices[Tri[0]][1], mMesh->vertices[Tri[0]][2]);
    glVertex3f(mMesh->vertices[Tri[1]][0], mMesh->vertices[Tri[1]][1], mMesh->vertices[Tri[1]][2]);
    glVertex3f(mMesh->vertices[Tri[2]][0], mMesh->vertices[Tri[2]][1], mMesh->vertices[Tri[2]][2]);
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
    for(int i = 0;i < mMesh->num_tris; i++) {
        fcl::Triangle tri = mMesh->tri_indices[i];
        glVertex3f(mMesh->vertices[tri[0]][0], mMesh->vertices[tri[0]][1], mMesh->vertices[tri[0]][2]);
        glVertex3f(mMesh->vertices[tri[1]][0], mMesh->vertices[tri[1]][1], mMesh->vertices[tri[1]][2]);
        glVertex3f(mMesh->vertices[tri[2]][0], mMesh->vertices[tri[2]][1], mMesh->vertices[tri[2]][2]);

    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopMatrix();
}



} // namespace collision
