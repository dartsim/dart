#include "CollisionInterface.h"
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "RigidBody.h"
#include "dart/dynamics/FreeJoint.h"

using namespace Eigen;
using namespace dart::dynamics;
using namespace dart::collision;

CollisionInterface::CollisionInterface() {
    mCollisionChecker = new dart::collision::DARTCollisionDetector();
    mCollisionChecker->setNumMaxContacs(10);
}

CollisionInterface::~CollisionInterface() {
    if (mCollisionChecker)
        delete mCollisionChecker;
    for (int i = 0; i < mSkeletons.size(); i++)
    {
        delete mSkeletons[i];
    }
}


void CollisionInterface::addSkeleton(dart::dynamics::Skeleton* _skel) {
    int nNodes = _skel->getNumBodyNodes();
    for (int i = 0; i < nNodes; i++) {
        BodyNode *bn = _skel->getBodyNode(i);
        mCollisionChecker->addCollisionSkeletonNode(bn);
        mNodeMap[_skel->getBodyNode(i)] = NULL;
    }    
}

void CollisionInterface::addRigidBody(RigidBody *_rb, const std::string& name) {
    Skeleton *skel = new Skeleton();
    BodyNode *bn = new BodyNode();
    bn->setParentJoint( new dart::dynamics::FreeJoint("freeJoint") );
    bn->addCollisionShape(_rb->mShape);
    skel->addBodyNode( bn );
    skel->setName( name );
    skel->disableSelfCollision();
    skel->init();
    mCollisionChecker->addCollisionSkeletonNode(bn);
    mNodeMap[bn] = _rb;
}

void CollisionInterface::checkCollision() {
    updateBodyNodes();
    mCollisionChecker->detectCollision(true, true);
    postProcess();
}

void CollisionInterface::updateBodyNodes() {
    int numNodes = mNodeMap.size();
    for (std::map<BodyNode*, RigidBody*>::iterator it = mNodeMap.begin(); it != mNodeMap.end(); ++it) {
        BodyNode *bn = it->first;
        RigidBody *rb = it->second;
        if (bn->getSkeleton()->getName() == "pinata")
            continue;
        Isometry3d W;
        W.setIdentity();
        W.linear() = rb->mOrientation;
        W.translation() = rb->mPosition;
        W.makeAffine();
        bn->getSkeleton()->getJoint("freeJoint")->setTransformFromParentBodyNode(W);
        //bn->updateTransform();
        bn->getSkeleton()->computeForwardKinematics(true, false, false);
    }
}

void CollisionInterface::postProcess() {
    mContacts.clear();
    int numContacts = mCollisionChecker->getNumContacts();
    mContacts.resize(numContacts);
    for (int i = 0; i < numContacts; i++) {
        mContacts[i].point = mCollisionChecker->getContact(i).point;
        mContacts[i].normal = mCollisionChecker->getContact(i).normal;
        mContacts[i].rb1 = mNodeMap[mCollisionChecker->getContact(i).bodyNode1];
        mContacts[i].rb2 = mNodeMap[mCollisionChecker->getContact(i).bodyNode2];
        if(mContacts[i].rb1 == NULL) {
          BodyNode* bd = mCollisionChecker->getContact(i).bodyNode1;
          Vector3d localPoint = bd->getTransform().inverse() * mContacts[i].point;
          mContacts[i].pinataVelocity = bd->getWorldLinearVelocity(localPoint);
        } else if(mContacts[i].rb2 == NULL) {
          BodyNode* bd = mCollisionChecker->getContact(i).bodyNode2;
          Vector3d localPoint = bd->getTransform().inverse() * mContacts[i].point;
          mContacts[i].pinataVelocity = bd->getWorldLinearVelocity(localPoint);
        } else {
          mContacts[i].pinataVelocity.setZero();
        }        
    }
}
