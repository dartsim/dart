#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "dynamics/BodyNodeDynamics.h"
#include "math/UtilsMath.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;
using namespace dart_math;


Controller::Controller(dynamics::SkeletonDynamics *_skel) {

    mSkel = _skel;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    mTorques.setZero();
    for (int i = 0; i < nDof; i++)
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    
    for (int i = 3; i < nDof; i++) {
        mKp(i, i) = 500.0;
        mKd(i, i) = 50.0;
    }
    mFrame = 0;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    // track a pose using PD
    mTorques.setZero();
    mTorques = -mKp * (_dof - mDesiredDofs) - mKd * _dofVel;
    for (int i = 0; i < 3; i++)
        mTorques[i] = 0.0;
    mTorques = mSkel->getMassMatrix() * mTorques; // scaled by accumulated mass

    
    mFrame++;
}
