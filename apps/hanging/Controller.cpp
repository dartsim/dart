#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/Shape.h"
#include "math/UtilsMath.h"
#include "collision/CollisionDetector.h"

using namespace kinematics;
using namespace dynamics;
using namespace dart_math;

Controller::Controller(dynamics::SkeletonDynamics *_skel, dynamics::ConstraintDynamics *_constraintHandle, double _t) {
    mSkel = _skel;
    mConstraintHandle = _constraintHandle;
    mTimestep = _t;
    mFrame = 0;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
    mConstrForces = VectorXd::Zero(nDof);
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
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
        
    mPreOffset = 0.0;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    // SPD tracking
    int nDof = mSkel->getNumDofs();
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
    mTorques = p + d - mKd * qddot * mTimestep;

    for (int i = 0; i < 6; i++){        
        mTorques[i] = 0.0;
    }
    mFrame++;
}
