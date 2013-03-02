#include "CollisionSkeleton.h"
#include "CollisionShapes.h"
#include "kinematics/Shape.h"
#include <cmath>
#include "utils/LoadOpengl.h"
#include "utils/UtilsMath.h"
#include "kinematics/BodyNode.h"
#include "fcl/collision.h"
#include <algorithm>

using namespace std;
using namespace Eigen;
using namespace utils;

namespace collision_checking{
    
    CollisionSkeletonNode::CollisionSkeletonNode(kinematics::BodyNode* _bodyNode)
    {
        mBodyNode = _bodyNode;

        kinematics::Shape *shape = _bodyNode->getShape();

        switch (shape->getShapeType()) {
        case kinematics::Shape::P_ELLIPSOID:
        	mMesh = createEllipsoid<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2]);
        	break;
        case kinematics::Shape::P_BOX:
        	mMesh = createCube<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2]);
        	break;
        case kinematics::Shape::P_MESH:
        	mMesh = createMesh<fcl::OBBRSS>(shape->getDim()[0], shape->getDim()[1], shape->getDim()[2], shape->getCollisionMesh());
        	break;
        default:
        	cout << "ERROR: Collision checking does not support " << _bodyNode->getName() << "'s Shape type\n";
        	break;
        }
    }

    CollisionSkeletonNode::~CollisionSkeletonNode()
    {
        delete mMesh;
    }

    int CollisionSkeletonNode::checkCollision(CollisionSkeletonNode* _otherNode, vector<ContactPoint>* _contactPoints, int _num_max_contact)
    {
        evalRT();
        _otherNode->evalRT();
        fcl::CollisionResult res;
        fcl::CollisionRequest req;

        req.enable_contact = _contactPoints; // only evaluate contact points if data structure for returning the contact points was provided
        req.num_max_contacts = _num_max_contact;
        fcl::collide(mMesh, mFclWorldTrans, _otherNode->mMesh, _otherNode->mFclWorldTrans, req, res);

        if(!_contactPoints) {
            return res.isCollision() ? 1 : 0;
        }

        int numCoplanarContacts = 0;
        int numNoContacts = 0;
        int numContacts = 0;

        vector<ContactPoint> unfilteredContactPoints;
        unfilteredContactPoints.reserve(res.numContacts());

        for(int i = 0; i < res.numContacts();i++) {
            // for each pair of intersecting triangles, we create two contact points
            ContactPoint pair1, pair2;
            pair1.bd1 = mBodyNode;
            pair1.bd2 = _otherNode->mBodyNode;
            pair1.bdID1 = this->mBodynodeID;
            pair1.bdID2 = _otherNode->mBodynodeID;
            pair1.collisionSkeletonNode1 = this;
            pair1.collisionSkeletonNode2 = _otherNode;
            fcl::Vec3f v;
            pair1.triID1 = res.getContact(i).b1;
            pair1.triID2 = res.getContact(i).b2;
            pair1.penetrationDepth = res.getContact(i).penetration_depth;
            pair2 = pair1;
            int contactResult = evalContactPosition(res, _otherNode, i, pair1.point, pair2.point);
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
            pair1.normal = Vector3d(v[0], v[1], v[2]);
            pair2.normal = Vector3d(v[0], v[1], v[2]);
        
            unfilteredContactPoints.push_back(pair1);
            unfilteredContactPoints.push_back(pair2);
        }

        const double ZERO = 0.000001;
        const double ZERO2 = ZERO*ZERO;

        vector<bool> markForDeletion(unfilteredContactPoints.size(), false);

        // mark all the repeated points
        for (unsigned int i = 0; i < unfilteredContactPoints.size(); i++) {
            for (unsigned int j = i + 1; j < unfilteredContactPoints.size(); j++) {
                Vector3d diff = unfilteredContactPoints[i].point - unfilteredContactPoints[j].point;
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
                    Vector3d  v = (unfilteredContactPoints[i].point - unfilteredContactPoints[j].point).cross(unfilteredContactPoints[i].point - unfilteredContactPoints[k].point);
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

        return res.numContacts();
    }

    void CollisionSkeletonNode::evalRT() {
        mWorldTrans = mBodyNode->getWorldTransform();
        Vector3d p = xformHom(mWorldTrans, mBodyNode->getShape()->getOffset());
        mWorldTrans.block(0, 3, 3, 1) = p;
        mFclWorldTrans = fcl::Transform3f(fcl::Matrix3f(mWorldTrans(0,0), mWorldTrans(0,1), mWorldTrans(0,2), 
                                                        mWorldTrans(1,0), mWorldTrans(1,1), mWorldTrans(1,2), 
                                                        mWorldTrans(2,0), mWorldTrans(2,1), mWorldTrans(2,2)),
                                          fcl::Vec3f(mWorldTrans(0,3), mWorldTrans(1,3), mWorldTrans(2,3)));
    }

    int CollisionSkeletonNode::evalContactPosition(fcl::CollisionResult& _result,  CollisionSkeletonNode* _other, int _idx, Vector3d& _contactPosition1, Vector3d& _contactPosition2) {
        int id1, id2;
        fcl::Triangle tri1, tri2;
        CollisionSkeletonNode* node1 = this;
        CollisionSkeletonNode* node2 = _other;
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
        _contactPosition1 = Vector3d(contact1[0], contact1[1], contact1[2]);
        _contactPosition2 = Vector3d(contact2[0], contact2[1], contact2[2]);
        return testRes;
    }

    void CollisionSkeletonNode::drawCollisionTriangle(int _tri) {
        fcl::Triangle Tri = mMesh->tri_indices[_tri];
        glVertex3f(mMesh->vertices[Tri[0]][0], mMesh->vertices[Tri[0]][1], mMesh->vertices[Tri[0]][2]);
        glVertex3f(mMesh->vertices[Tri[1]][0], mMesh->vertices[Tri[1]][1], mMesh->vertices[Tri[1]][2]);
        glVertex3f(mMesh->vertices[Tri[2]][0], mMesh->vertices[Tri[2]][1], mMesh->vertices[Tri[2]][2]);
    }

    void CollisionSkeletonNode::drawCollisionSkeletonNode(bool _bTrans) {
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

    SkeletonCollision::~SkeletonCollision() {
        for(int i=0;i<mCollisionSkeletonNodeList.size();i++)
            delete mCollisionSkeletonNodeList[i];
    }

    void SkeletonCollision::addCollisionSkeletonNode(kinematics::BodyNode *_bd, bool _bRecursive) {
        if (_bRecursive == false || _bd->getNumChildJoints() == 0) {
            CollisionSkeletonNode* csnode = new CollisionSkeletonNode(_bd);
            csnode->mBodynodeID = mCollisionSkeletonNodeList.size();
            mCollisionSkeletonNodeList.push_back(csnode);
            mBodyNodeHash[_bd] = csnode;
            mActiveMatrix.push_back(vector<bool>(mCollisionSkeletonNodeList.size() - 1));
            for(unsigned int i = 0; i < mCollisionSkeletonNodeList.size() - 1; i++) {
                //if(mCollisionSkeletonNodeList[i]->mBodyNode->getParentNode() == _bd || _bd->getParentNode() == mCollisionSkeletonNodeList[i]->mBodyNode) {
                if(mCollisionSkeletonNodeList[i]->mBodyNode->getSkel() == _bd->getSkel()) {
                    mActiveMatrix.back()[i] = false;
                }
                else {
                    mActiveMatrix.back()[i] = true;
                }
            }
        } else {
            addCollisionSkeletonNode(_bd, false);
            for (int i = 0; i < _bd->getNumChildJoints(); i++)
                addCollisionSkeletonNode(_bd->getChildNode(i), true);
        }
    }

    bool SkeletonCollision::checkCollision(bool _checkAllCollisions, bool _calculateContactPoints) {
        int num_max_contact = 100;
        clearAllContacts();
        mNumTriIntersection = 0;
        for (int i = 0; i < mCollisionSkeletonNodeList.size(); i++) {
            mCollisionSkeletonNodeList[i]->mBodyNode->setColliding(false);
        }
        for (int i = 0; i < mCollisionSkeletonNodeList.size(); i++) {
            for (int j = i + 1; j < mCollisionSkeletonNodeList.size(); j++) {
                if (!mActiveMatrix[j][i]) {
                    continue;
                }
                const int numTriIntersection = mCollisionSkeletonNodeList[i]->checkCollision(mCollisionSkeletonNodeList[j], _calculateContactPoints ? &mContactPointList : NULL, num_max_contact);
                mNumTriIntersection += numTriIntersection;
                if(numTriIntersection > 0) {
                    mCollisionSkeletonNodeList[i]->mBodyNode->setColliding(true);
                    mCollisionSkeletonNodeList[j]->mBodyNode->setColliding(true);
                }
                if(!_checkAllCollisions && mNumTriIntersection > 0) {
                    return true;
                }
            }
        }
        return (mNumTriIntersection > 0);
    }

    void SkeletonCollision::draw() {
        for(int i=0;i<mCollisionSkeletonNodeList.size();i++)
            mCollisionSkeletonNodeList[i]->drawCollisionSkeletonNode();
    }

    void SkeletonCollision::activatePair(const kinematics::BodyNode* node1, const kinematics::BodyNode* node2) {
        int nodeId1 = getCollisionSkeletonNode(node1)->mBodynodeID;
        int nodeId2 = getCollisionSkeletonNode(node2)->mBodynodeID;
        if(nodeId1 < nodeId2) {
            swap(nodeId1, nodeId2);
        }
        mActiveMatrix[nodeId1][nodeId2] = true;
    }

    void SkeletonCollision::deactivatePair(const kinematics::BodyNode* node1, const kinematics::BodyNode* node2) {
        int nodeId1 = getCollisionSkeletonNode(node1)->mBodynodeID;
        int nodeId2 = getCollisionSkeletonNode(node2)->mBodynodeID;
        if(nodeId1 < nodeId2) {
            swap(nodeId1, nodeId2);
        }
        mActiveMatrix[nodeId1][nodeId2] = false;
    }
}
