#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"

Controller::Controller(dynamics::SkeletonDynamics *_skel) {
    mSkel = _skel;
    int nDof = mSkel->getNumDofs();
    
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    mKd.resize(nDof);
    mKs.resize(nDof);
    for (int i = 0; i < 6; i++) {
        mKs[i] = 0.0;
        mKd[i] = 0.0;
    }
    for (int i = 6; i < nDof; i++) {
        mKs[i] = 250.0;
        mKd[i] = 2 * sqrt(250.0);
    }
}

void Controller::computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel) {
    for (unsigned int i = 0; i < mTorques.size(); i++) {
        mTorques[i] = -mKs[i] * (_dof[i] - mDesiredDofs[i])  -mKd[i] * _dofVel[i];
    }
}
