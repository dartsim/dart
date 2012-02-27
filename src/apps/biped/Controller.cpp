#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/UtilsMath.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Dof.h"
#include "kinematics/Joint.h"

using namespace kinematics;
using namespace Eigen;
using namespace utils;

Controller::Controller(dynamics::SkeletonDynamics *_skel) {
    mSkel = _skel;
    int nDof = mSkel->getNumDofs();
    mFrame = 0;
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }

    mKd.resize(nDof);
    mKs.resize(nDof);
    for (int i = 0; i < 6; i++) {
        mKs[i] = 0.0;
        mKd[i] = 0.0;
    }

    
    for (int i = 6; i < 18; i++) {
        mKs[i] = 100.0;
        mKd[i] = 2 * sqrt(20.0);
    }
    
    mKs[6] = 600.0;
    mKs[12] = 600.0;
    mKd[6] = 2 * sqrt(400.0);
    mKd[12] = 2 * sqrt(400.0);
    
    mKs[9] = 800.0;
    mKs[15] = 800.0;
    mKd[9] = 2 * sqrt(mKs[9]);
    mKd[15] = 2 * sqrt(mKs[15]);
    
    mKs[10] = 1000.0;
    mKd[10] = 2 * sqrt(mKs[10]);
    mKs[11] = 1000.0;
    mKd[11] = 2 * sqrt(mKs[11]);
    mKs[16] = 1000.0;
    mKd[16] = 2 * sqrt(mKs[16]);
    mKs[17] = 1000.0;
    mKd[17] = 2 * sqrt(mKs[17]);

    for (int i = 18; i < nDof; i++) {
        mKs[i] = 100.0;
        mKd[i] = 2 * sqrt(100.0);
    }
    mKs[19] = 600.0;
    mKd[19] = 2 * sqrt(mKs[19]);

    mMassTree = VectorXd::Zero(nDof);
    for (int i = 6; i < nDof; i++)
        mMassTree[i] = computeMassTree(mSkel->getDof(i)->getJoint()->getChildNode());
    
    mMassTree /= mMassTree.norm();
}

void Controller::computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel) {
    for (unsigned int i = 0; i < mTorques.size(); i++) 
        mTorques[i] = -mKs[i] * (_dof[i] - mDesiredDofs[i])  -mKd[i] * _dofVel[i];

    int nDof = mSkel->getNumDofs();
    //MatrixXd M = mSkel->getMassMatrix();
    //    int n = M.cols();
    for (int i = 6; i < nDof; i++)
      mTorques[i] *= mMassTree[i];
    //    VectorXd scaled = M * mTorques;
    //    for (int i = 6; i < nDof; i++)
    //        cout << scaled[i] / mTorques[i] << endl;
    //    mTorques = scaled;
    
    Vector3d com = mSkel->getWorldCOM();
    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");
    Vector3d cp = (lFoot->getWorldCOM() + rFoot->getWorldCOM()) /2.0;
    Vector3d vf = com - cp;
    double k = 5;
    
    mFrame++;
    
    mDesiredDofs[10] = 0.1 - k * vf[0];
    mDesiredDofs[16] = 0.1 - k * vf[0];
       
    vf[1] = -2000;
    int nDofs = mSkel->getNumDofs();
    MatrixXd J(MatrixXd::Zero(3, nDofs));
    Vector3d lHeel = lFoot->getWorldTransform().block(0, 3, 3, 1);
    Vector3d rHeel = rFoot->getWorldTransform().block(0, 3, 3, 1);

    for (int i = 0; i < lFoot->getNumDependentDofs(); i++) {
        int index = lFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(lFoot->getDerivWorldTransform(i), lHeel);
        J.col(index) = jCol;
    }
    //  mTorques += J.transpose() * vf / 2.0;
        
    J.setZero();
    for (int i = 0; i < rFoot->getNumDependentDofs(); i++) {
        int index = rFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(rFoot->getDerivWorldTransform(i), rHeel);
        J.col(index) = jCol;
    }
    //    mTorques += J.transpose() * vf / 2.0;
    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;
}

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
