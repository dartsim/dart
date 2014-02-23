#include "CollisionInterface.h"
#include "collision/fcl_mesh/FCLMESHCollisionDetector.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Shape.h"
#include "rigidBody.h"

using namespace Eigen;
using namespace dart::dynamics;
using namespace dart::collision;

CollisionInterface::CollisionInterface() {
	mCollisionChecker = new dart::collision::FCLMeshCollisionDetector();
    //mCollisionChecker->setNumMaxContacts(2);
}

CollisionInterface::~CollisionInterface() {
    if (mCollisionChecker)
        delete mCollisionChecker;
}


void CollisionInterface::addSkeleton(dart::dynamics::Skeleton* _skel) {
	int nNodes = _skel->getNumBodyNodes();
    for (int i = 0; i < nNodes; i++) {
		BodyNode *bn = _skel->getBodyNode(i);
        mCollisionChecker->addCollisionSkeletonNode(bn);
        mNodeMap[_skel->getBodyNode(i)] = NULL;
    }
}

void CollisionInterface::addRigidBody(RigidBody *_rb) {
    BodyNode *bn = new BodyNode();
	bn->addCollisionShape(_rb->mShape);
    /*Skeleton *skel = new Skeleton();    
    bn->setSkel(skel);*/
    mCollisionChecker->addCollisionSkeletonNode(bn);
    mNodeMap[bn] = _rb;
}

void CollisionInterface::checkCollision() {
    updateBodyNodes();
    mCollisionChecker->clearAllContacts();    
    mCollisionChecker->detectCollision(true, true);
    postProcess();
}

void CollisionInterface::updateBodyNodes() {
    int numNodes = mNodeMap.size();
    for (std::map<BodyNode*, RigidBody*>::iterator it = mNodeMap.begin(); it != mNodeMap.end(); ++it) {
        BodyNode *bn = it->first;
        RigidBody *rb = it->second;
        if (rb == NULL)
            continue;
        Matrix4d W;
        W.setIdentity();
        W.topLeftCorner(3, 3) = rb->mOrientation;
        W.topRightCorner(3, 1) = rb->mPosition;
        //bn->setWorldTransform(W);
    }
}

void CollisionInterface::postProcess() {
    mContacts.clear();
    int numContacts = mCollisionChecker->getNumContacts();
    mContacts.resize(numContacts);
    for (int i = 0; i < numContacts; i++) {
        mContacts[i].point = mCollisionChecker->getContact(i).point;
        mContacts[i].normal = mCollisionChecker->getContact(i).normal;
        mContacts[i].rb1 = mNodeMap[mCollisionChecker->getContact(i).collisionNode1->getBodyNode()];
        mContacts[i].rb2 = mNodeMap[mCollisionChecker->getContact(i).collisionNode2->getBodyNode()];
    }
}
