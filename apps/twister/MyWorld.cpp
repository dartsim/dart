#include "MyWorld.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Marker.h"
#include <iostream>

using namespace Eigen;
using namespace dart::dynamics;

MyWorld::MyWorld() {
    // Load a skeleton from file
    mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/human.skel");
    // Assume that there is only one constraint
    mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
    mConstrainedMarker = -1;
}

MyWorld::~MyWorld() {
    delete mSkel;
}

void MyWorld::solve() {
    if (mConstrainedMarker == -1)
        return; 
    int numIter = 300;
    double alpha = 0.01;
    int nDof = mSkel->getNumDofs();
    VectorXd gradients(nDof);
    VectorXd newPose(nDof);
    for (int i = 0; i < numIter; i++) {
        gradients = updateGradients();
        newPose = mSkel->getPositions() - alpha * gradients;
        mSkel->setPositions(newPose); 
        mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
    }
}

// Current code only works for the left ankle with only one constraint
VectorXd MyWorld::updateGradients() {
    // compute c(q)
    mC = mSkel->getMarker(mConstrainedMarker)->getWorldPosition() - mTarget;

    // compute J(q)
    Vector4d offset;
    offset << mSkel->getMarker(mConstrainedMarker)->getLocalPosition(), 1; // Create a vector in homogeneous coordinates
    // w.r.t ankle
    BodyNode *node = mSkel->getMarker(mConstrainedMarker)->getBodyNode();
    Joint *joint = node->getParentJoint();
    Matrix4d worldToParent = node->getParentBodyNode()->getTransform().matrix();
    Matrix4d parentToJoint = joint->getTransformFromParentBodyNode().matrix();
    Matrix4d dR = joint->getTransformDerivative(0); // Doesn't need .matrix() because it returns a Matrix4d instead of Isometry3d
    Matrix4d R = joint->getTransform(1).matrix();
    Matrix4d jointToChild =joint->getTransformFromChildBodyNode().inverse().matrix();
    Vector4d jCol = worldToParent * parentToJoint * dR * R * jointToChild * offset;
    int colIndex = joint->getGenCoord(0)->getSkeletonIndex();
    mJ.col(colIndex) = jCol.head(3); // Take the first 3 elelemtns of J
    dR = joint->getTransformDerivative(1);
    R = joint->getTransform(0).matrix();
    jCol = worldToParent * parentToJoint * R * dR * jointToChild * offset;
    colIndex = joint->getGenCoord(1)->getSkeletonIndex();
    mJ.col(colIndex) = jCol.head(3);
    
    // compute gradients
    VectorXd gradients = 2 * mJ.transpose() * mC;
    return gradients;
}

// Current code only handlse one constraint on the left foot.
void MyWorld::createConstraint(int _index) {
    if (_index == 3) {
        mTarget = mSkel->getMarker(_index)->getWorldPosition();
        mConstrainedMarker = 3;
    } else {
        mConstrainedMarker = -1;
    }
}

void MyWorld::modifyConstraint(Vector3d _deltaP) {
    if (mConstrainedMarker == 3)
        mTarget += _deltaP;
}

void MyWorld::removeConstraint(int _index) {
    mConstrainedMarker = -1;
}


