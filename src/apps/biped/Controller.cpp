#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/UtilsMath.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Dof.h"
#include "kinematics/Joint.h"

using namespace kinematics;
using namespace Eigen;
using namespace utils;

Controller::Controller(dynamics::SkeletonDynamics *_skel, double _t) {
    mSkel = _skel;
    mTimestep = _t;
    int nDof = mSkel->getNumDofs();
    mFrame = 0;
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }

    for (int i = 0; i < 6; i++) {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
       
    for (int i = 6; i < nDof; i++) {
        mKp(i, i) = 25.0;
        mKd(i, i) = 2 * sqrt(25.0);
    }

    // hips
    mKp(6, 6) = 500.0;
    mKd(6, 6) = 2 * sqrt(mKp(6, 6));
    mKp(12, 12) = 500.0;
    mKd(12, 12) = 2 * sqrt(mKp(12, 12));
    
    // knees
    mKp(9, 9) = 500.0;
    mKd(9, 9) = 2 * sqrt(mKp(9, 9));
    mKp(15, 15) = 500.0;
    mKd(15, 15) = 2 * sqrt(mKd(15, 15));
    
    // ankles
    mKp(10, 10) = 500.0;
    mKd(10, 10) = 2 * sqrt(mKp(10, 10));
    mKp(11, 11) = 500.0;
    mKd(11, 11) = 2 * sqrt(mKp(11, 11));
    mKp(16, 16) = 500.0;
    mKd(16, 16) = 2 * sqrt(mKp(16, 16));
    mKp(17, 17) = 500.0;
    mKd(17, 17) = 2 * sqrt(mKp(17, 17));
    
    // lower back
    //    mKp(18, 18) = 500.0;
    //    mKd(18, 18) = 2 * sqrt(mKp(18, 18));
    //    mKp(19, 19) = 500.0;
    //    mKd(19, 19) = 2 * sqrt(mKp(19, 19));

    // mass tree is only used when SPD is not activated
    mMassTree = VectorXd::Zero(nDof);
    for (int i = 6; i < nDof; i++)
        mMassTree[i] = computeMassTree(mSkel->getDof(i)->getJoint()->getChildNode());
    
    mMassTree /= mMassTree.norm();
    
}

void Controller::computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel) {
    mFrame++;
    int nDof = mSkel->getNumDofs();
    mTorques.setZero();
    MatrixXd J(MatrixXd::Zero(3, nDof));
    
    // PD tracking
    /*  
    for (unsigned int i = 6; i < mTorques.size(); i++) 
        mTorques[i] = -mKp(i, i) * (_dof[i] - mDesiredDofs[i])  -mKd(i, i) * _dofVel[i];

    for (int i = 6; i < nDof; i++)
        mTorques[i] *= mMassTree[i];
    */

    // ankle strategy
    /*
    Vector3d com = mSkel->getWorldCOM();
    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");

    //Vector3d cp = (lFoot->getWorldCOM() + rFoot->getWorldCOM()) / 2.0;
    Vector3d cp = (lFoot->evalWorldPos(Vector3d::Zero()) + rFoot->evalWorldPos(Vector3d::Zero())) /2.0;
    double k1 = 10.0;
    Vector3d vf = (com - cp) * k1;
    vf[0] = -100;
    vf[1] = -mSkel->getMass() * 9.8;
    // ankle strategy
    //double k2 = 5;
    //    mDesiredDofs[10] = 0.1 - k2 * vf[0];
    //    mDesiredDofs[16] = 0.1 - k2 * vf[0];
    */

    // virtual force on lower body
    /*
    Vector3d lHeel = lFoot->getLocalCOM(); //lFoot->getWorldTransform().block(0, 3, 3, 1);
    Vector3d rHeel = rFoot->getLocalCOM(); //rFoot->getWorldTransform().block(0, 3, 3, 1);

    for (int i = 0; i < lFoot->getNumDependentDofs(); i++) {
        int index = lFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(lFoot->getDerivWorldTransform(i), lHeel);
        J.col(index) = jCol;
    }
    //mTorques += J.transpose() * vf / 2.0;
        
    J.setZero();
    for (int i = 0; i < rFoot->getNumDependentDofs(); i++) {
        int index = rFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(rFoot->getDerivWorldTransform(i), rHeel);
        J.col(index) = jCol;
    }
    //mTorques += J.transpose() * vf / 2.0;
    */

    // gravity compensation for upper body
    /*    
    VectorXd torque(nDof);
    torque.setZero();
    for (int i = 7; i < mSkel->getNumNodes(); i++) { // loop over each node in upperbody
        BodyNode *node = mSkel->getNode(i);
        Vector3d com = node->getLocalCOM();
        J.setZero();

        for (int j = 0; j < node->getNumDependentDofs(); j++) {
            int index = node->getDependentDof(j);
            VectorXd jCol = utils::xformHom(node->getDerivWorldTransform(j), com);
            J.col(index) = jCol;
        }
        torque += node->getMass() * J.transpose() * Vector3d(0, 9.8, 0);
    }

    mTorques += torque;
    */

    // virtual force on COM
      
    Vector3d vf(0.0, 0.0, 0.0);
    Vector3d com = mSkel->getWorldCOM();
    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");
    Vector3d cp = (lFoot->evalWorldPos(Vector3d::Zero()) + rFoot->evalWorldPos(Vector3d::Zero())) /2.0;
    vf = (cp - com) * 1000;
    vf[0] += 18;
    vf[1] = 0.0;
    J.setZero();
    int nNode = mSkel->getNumNodes();
    for (int i = 0; i < nNode; i++) {
        BodyNode *bNode = mSkel->getNode(i);
        MatrixXd localJ = bNode->getJacobianLinear() * bNode->getMass();
        for (int j = 0; j < bNode->getNumDependentDofs(); j++) {
            int dofIndex = bNode->getDependentDof(j);
            J.col(dofIndex) += localJ.col(j);
        }
    }
    mTorques += J.transpose() / mSkel->getMass() * vf;
    
    /*
    // virtual force from ankles
    Vector3d vf(0.0, 0.0, 0.0);
    Vector3d com = mSkel->getWorldCOM();
    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");
    Vector3d cp = (lFoot->evalWorldPos(Vector3d::Zero()) + rFoot->evalWorldPos(Vector3d::Zero())) /2.0;
    vf = (cp - com) * 1000;
    vf[0] += 18;
    vf[1] = 0.0;
    J.setZero();
    //    BodyNode *hand = mSkel->getNode("fullbody_h_hand_left");
    //    BodyNode *abd = mSkel->getNode("fullbody_h_abdomen");
    BodyNode *node = mSkel->getNode("fullbody_h_spine");
    BodyNode *foot = mSkel->getNode("fullbody_h_foot_left");
    //    Vector3d handCOM = hand->getLocalCOM();
    //    Vector3d abdCOM = abd->getLocalCOM();
    Vector3d nodeCOM = node->getLocalCOM();
    for (int i = 0; i < foot->getNumDependentDofs(); i++) {
        int index = foot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(foot->getDerivWorldTransform(i).transpose() * node->getWorldTransform(), nodeCOM);
        if (index == 8)
            J.col(index) = -jCol;
        else
            J.col(index) = jCol;
    }
    for (int i = 0; i < node->getNumDependentDofs(); i++) {
        int index = node->getDependentDof(i);
        VectorXd jCol = utils::xformHom(foot->getWorldTransform().transpose() * node->getDerivWorldTransform(i), nodeCOM);
        J.col(index) = jCol;
    }
    
    foot = mSkel->getNode("fullbody_h_foot_right");
    for (int i = 0; i < foot->getNumDependentDofs(); i++) {
        int index = foot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(foot->getDerivWorldTransform(i).transpose() * node->getWorldTransform(), nodeCOM);
        if (index == 14)
            J.col(index) -= jCol;
        else
            J.col(index) += jCol;
    }
    for (int i = 0; i < node->getNumDependentDofs(); i++) {
        int index = node->getDependentDof(i);
        VectorXd jCol = utils::xformHom(foot->getWorldTransform().transpose() * node->getDerivWorldTransform(i), nodeCOM);
        J.col(index) += jCol;
    }
        //    vf[2] = -vf[2];
    mTorques += J.transpose() * vf;
    */  
    // SPD control force
    for (int i = 0; i < nDof; i++)
        mTorques[i] += -mKp(i, i) * (_dof[i] + _dofVel[i] * mTimestep - mDesiredDofs[i]) - mSkel->getKd()(i, i) * _dofVel[i];

    // Just to make sure no illegal torque is used
    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;
    
    mSkel->setInternalForces(mTorques);
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

