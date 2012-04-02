#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Joint.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"

using namespace kinematics;
using namespace Eigen;
using namespace utils;

Controller::Controller(dynamics::SkeletonDynamics *_skel, double _t) {
    mSkel = _skel;
    mTimestep = _t;
    int nDof = mSkel->getNumDofs();
    //    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    //    mKd.resize(nDof);
    //    mKs.resize(nDof);

    mKp = MatrixXd::Identity(nDof, nDof) * 10.0;
    mKd = MatrixXd::Identity(nDof, nDof) * 1.0;

        //    for (int i = 0; i < nDof; i++) {
    //    mKs[i] = 50.0;
    //  mKd[i] = 5;//2 * sqrt(0.0);
    //}

    //    mMassTree = VectorXd::Zero(nDof);
    //    for (int i = 0; i < nDof; i++)
    //  mMassTree[i] = computeMassTree(mSkel->getDof(i)->getJoint()->getChildNode());
    
    //    mMassTree /= mMassTree.norm();
}

void Controller::computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel) {

    int nDof = mSkel->getNumDofs();        
    VectorXd torque(nDof);

    for (int i = 0; i < nDof; i++)
        torque[i] = -mKp(i, i) * (_dof[i] + _dofVel[i] * mTimestep - mDesiredDofs[i]) - mSkel->getKd()(i, i) * _dofVel[i];
    
    mSkel->setInternalForces(torque);

    //    for (unsigned int i = 0; i < mTorques.size(); i++) {
    //  mTorques[i] = -mKs[i] * (_dof[i] - mDesiredDofs[i])  -mKd[i] * _dofVel[i];
    //  mTorques[i] *= mMassTree[i];        
    //}
}

/*
double Controller::computeMassTree(BodyNode *_bd) {
    if (_bd->getNumChildJoints() == 0) {
        return _bd->getMass();
    }else{
        double sum = _bd->getMass();
        for (int i = 0; i < _bd->getNumChildJoints(); i++)
            sum += computeMassTree(_bd->getChildJoint(i)->getChildNode());
        return sum;
    }
}
*/
