#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/Shape.h"
#include "math/UtilsMath.h"
#include "collision/CollisionDetector.h"

using namespace kinematics;
using namespace dynamics;
using namespace dart_math;

Controller::Controller(dynamics::SkeletonDynamics *_skel, double _t) {
    mSkel = _skel;
    mTimestep = _t;
    mFrame = 0;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
        
    mTorques = VectorXd::Zero(nDof);

    initializeTargetPose();
    
    // using SPD results in simple Kp coefficients
    for (int i = 0; i < nDof; i++) {
        mKp(i, i) = 500.0;
        mKd(i, i) = 50.0;
    }
}

void Controller::initializeTargetPose() {
    int nDof = mSkel->getNumDofs();
    // set desired joint angles for grasping
    double graspPose[] = { 0, 0, 3.11, 0, 0.0, 0.15, 0.28, -0.87, -0.26, -0.8, -0.12, 0.15, -0.49, -0.51, -0.15, -1.24, -0.62, -0.31, -0.03, -1.44, -0.54, -0.33, -0.11, -1.56, -0.56, -0.37, -0.34, -1.74, -0.58};

    mDesiredDofs.resize(nDof);

    for (int i = 0; i < nDof; i++){
        mDesiredDofs[i] = graspPose[i];
    }
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel, const VectorXd& _constrForce) {

    // SPD tracking
    int nDof = mSkel->getNumDofs();
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + _constrForce);
    mTorques = p + d - mKd * qddot * mTimestep;
    
    mFrame++;
}

void Controller::getReady() {
    mDesiredDofs[2] = 0.0;
    mDesiredDofs[3] = 1.0;
}

void Controller::throwBall() {
    double releasePose[] = { 0, 0, 0, 1.0, 0.0, 0.15, 0.28, -0.87, -0.26, -0.8, -0.12, 0.15, -0.49, -0.21, -0.15, -0.54, -0.32, -0.11, -0.03, -0.44, -0.24, -0.13, -0.11, -0.56, -0.26, -0.17, -0.14, -0.74, -0.28};

    for (int i = 0; i < mDesiredDofs.size(); i++)
        mDesiredDofs[i] = releasePose[i];
}
