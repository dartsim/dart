#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"

using namespace Eigen;

Controller::Controller(dynamics::SkeletonDynamics *_skel) 
{
    mSkel = _skel;
    int nDof = mSkel->getNumDofs();
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    mKp.resize(nDof);
    mKd.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();    
        mKp[i] = 200.0;
        mKd[i] = 10;
    }
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) 
{ 
    for (int i = 0; i < _dof.size(); i++) 
        mTorques[i] = -mKp[i] * (_dof[i] - mDesiredDofs[i]) - mKd[i] * _dofVel[i];
    mTorques = mSkel->getMassMatrix() * mTorques; // scaled by accumulated mass
}
