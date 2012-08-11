#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/FileInfoDof.h"

using namespace Eigen;

Controller::Controller(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, double _t) {
    mMotion = _motion;
    mSkel = _skel;
    mTimestep = _t;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);;
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }

#ifdef SPD
    for (int i = 0; i < nDof; i++) {
        mKp(i, i) = 15.0;
        mKd(i, i) = 2.0;
    }
#else
    for (int i = 0; i < nDof; i++) {
        mKp(i, i) = 800.0;
        mKd(i, i) = 15;
    }
#endif

    mFrame = 0;
    mInterval = (1.0 / mMotion->getFPS()) / mTimestep;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    // feedback control force
    int frame = mFrame / mInterval;    
    if (frame >= mMotion->getNumFrames())
        frame = mMotion->getNumFrames() - 1;
    
    mDesiredDofs = mMotion->getPoseAtFrame(frame);
#ifdef SPD
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
    mTorques = p + d - mKd * qddot * mTimestep;
#else
    mTorques = -mKp * (_dof - mDesiredDofs) - mKd * _dofVel;
    mTorques = mSkel->getMassMatrix() * mTorques; // scaled by accumulated mass
#endif

    mFrame++; 
}
