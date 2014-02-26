#include "CollisionInterface.h"
#include "collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Shape.h"
#include "RigidBody.h"
#include "dynamics/FreeJoint.h"

using namespace Eigen;
using namespace dart::dynamics;
using namespace dart::collision;

CollisionInterface::CollisionInterface() {
    mCollisionChecker = new dart::collision::FCLMeshCollisionDetector();
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
    skel->setSelfCollidable(false);
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
        if (rb == NULL)
            continue;
        Eigen::Isometry3d W;
        W.setIdentity();
        W.linear() = rb->mOrientation;
        W.translation() = rb->mPosition;
        W.makeAffine();
        bn->getSkeleton()->getJoint("freeJoint")->setTransformFromParentBodyNode(W);
        bn->updateTransform();
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
        if ( mContacts[i].rb2 == NULL ) //Colliding with something static. Reflect
        {
            Eigen::Vector3d vApproachPreImpulsePointOfContact = ( mContacts[i].rb1->mMomentum / mContacts[i].rb1->mMass ).dot(mContacts[i].normal)*mContacts[i].normal;

            if (vApproachPreImpulsePointOfContact.dot(mContacts[i].normal) < 0) //If approaching
            {
                Eigen::Vector3d vTangential = ( mContacts[i].rb1->mMomentum / mContacts[i].rb1->mMass ) - vApproachPreImpulsePointOfContact;
                Eigen::Vector3d vReflected = vTangential - vApproachPreImpulsePointOfContact;
                mContacts[i].rb1->mMomentum = mContacts[i].rb1->mMass * (vTangential + vReflected);
            }
        }
    }
}
