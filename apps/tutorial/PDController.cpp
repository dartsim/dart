#include "PDController.h"

using namespace Eigen;
using namespace dynamics;

PDController::PDController(SkeletonDynamics* _skel) : Controller(_skel) {

    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
            
    mDesiredDofs = mSkel->getPose();
    
    mDesiredDofs[9] = -1.5;
    mDesiredDofs[13] = 1.5;
    mDesiredDofs[27] = -1.5;
    mDesiredDofs[28] = 0.75;
    mDesiredDofs[29] = 1.5;
    
    for (int i = 0; i < 6; i++) {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
    for (int i = 6; i < 22; i++)
        mKp(i, i) = 200.0; // lower body + lower back
    for (int i = 22; i < nDof; i++)
        mKp(i, i) = 20.0;
    for (int i = 6; i < 22; i++) 
        mKd(i, i) = 10.0;
    for (int i = 22; i < nDof; i++) 
        mKd(i, i) = 1.0;
}

void PDController::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {    
    mTorques = -mKp * (_dof - mDesiredDofs) -mKd * _dofVel;
    //mTorques = mSkel->getMassMatrix() * mTorques; // scaled by accumulated mass
    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;
}
