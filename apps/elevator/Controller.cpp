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
    mDesiredDofs.resize(nDof);
    
    mDesiredDofs = mSkel->getPose();
    
    // using SPD results in simple Kp coefficients
    for (int i = 0; i < nDof; i++) {
        mKp(i, i) = 500.0;
        mKd(i, i) = 50.0;
    }

    // using SPD results in simple Kp coefficients
    for (int i = 0; i < 6; i++) {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
    for (int i = 6; i < 22; i++)
        mKp(i, i) = 200.0; // lower body + lower back
    for (int i = 22; i < nDof; i++)
        mKp(i, i) = 20.0;
    for (int i = 6; i < 22; i++) 
        mKd(i, i) = 100.0;
    for (int i = 22; i < nDof; i++) 
        mKd(i, i) = 10.0;

}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel, const VectorXd& _constrForce) {

    // SPD tracking
    int nDof = mSkel->getNumDofs();
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + _constrForce);
    mTorques = p + d - mKd * qddot * mTimestep;
    /*   
    if (mFrame == 3000) {
        mDesiredDofs[9] = -2.5;
        mDesiredDofs[16] = -2.5;
    }
    */

    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;

    mFrame++;
}


