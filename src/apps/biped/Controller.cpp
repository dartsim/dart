#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/UtilsMath.h"
#include "kinematics/BodyNode.h"

using namespace kinematics;
using namespace Eigen;
using namespace utils;

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
    for (int i = 6; i < 18; i++) {
        mKs[i] = 200.0;
        mKd[i] = 2 * sqrt(200.0);
    }
    for (int i = 18; i < nDof; i++) {
        mKs[i] = 200.0;
        mKd[i] = 2 * sqrt(200.0);
    }

}

void Controller::computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel) {
    for (unsigned int i = 18; i < mTorques.size(); i++) 
        mTorques[i] = -mKs[i] * (_dof[i] - mDesiredDofs[i])  -mKd[i] * _dofVel[i];
    
    Vector3d com = mSkel->getWorldCOM();
    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");
    Vector3d cp = (lFoot->getWorldCOM() + rFoot->getWorldCOM()) /2.0;
    Vector3d vf = com - cp;
    vf[1] = -10;

    int nDofs = mSkel->getNumDofs();
    MatrixXd J(MatrixXd::Zero(3, nDofs));
    Vector3d lHeel = lFoot->getWorldTransform().block(0, 3, 3, 1);
    Vector3d rHeel = rFoot->getWorldTransform().block(0, 3, 3, 1);

    for (int i = 0; i < lFoot->getNumDependentDofs(); i++) {
        int index = lFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(lFoot->getDerivWorldTransform(i), lHeel);
        J.col(index) = jCol;
    }
    mTorques += J.transpose() * vf / 2.0;
        
    J.setZero();
    for (int i = 0; i < rFoot->getNumDependentDofs(); i++) {
        int index = rFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(rFoot->getDerivWorldTransform(i), rHeel);
        J.col(index) = jCol;
    }
    mTorques += J.transpose() * vf / 2.0;
   
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;
}
