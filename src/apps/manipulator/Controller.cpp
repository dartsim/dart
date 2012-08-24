#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "dynamics/BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;
using namespace utils;

//#define HAND_SHAPE

Controller::Controller(dynamics::SkeletonDynamics *_skel, double _t) {

    mSkel = _skel;
    mTimestep = _t;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);;
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    mTorques.setZero();
    for (int i = 0; i < nDof; i++)
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    
    mKp(0, 0) = 50;
    mKp(1, 1) = 50;
    mKd(0, 0) = 5;
    mKd(1, 1) = 5;
    for (int i = 2; i < nDof; i++) {
        mKp(i, i) = 15.0;
        mKd(i, i) = 2.0;
    }
    mFrame = 0;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel, const VectorXd& _objVel) {
    // track a pose using SPD
    mTorques.setZero();
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
    mTorques = p + d - mKd * qddot * mTimestep;

    // change hand shape
#ifdef HAND_SHAPE    
    if (mFrame > 1015 && mFrame < 1035) {
        BodyNode *midFinger = mSkel->getNode("manipulator_middlePIP");
        BodyNode *ringFinger = mSkel->getNode("manipulator_ringPIP");
        //        Vector3d xAxis = -_objVel.head(3).normalized();
        //        Vector3d yAxis = xformHom(midFinger->getWorldTransform(), Vector3d(0, 1, 0));
        //  Vector3d zAxis = xAxis.cross(yAxis);
        Vector3d zAxis(0, 0, 1);
        
        double alpha = 1.0;
        Vector3d omega = zAxis * alpha;
        MatrixXd Jw = midFinger->getJacobianAngular();
        //        VectorXd vel = Jw.transpose() * (Jw * Jw.transpose()).inverse() * omega;
        VectorXd tau = Jw.transpose() * omega;
        VectorXd torques(mTorques.size());
        torques.setZero();
        for (int i = 0; i < tau.size(); i++) {
            int dofIndex = midFinger->getDependentDof(i);
            torques[dofIndex] += tau[i] * 3;
        }
        Jw = ringFinger->getJacobianAngular();
        tau = Jw.transpose() * omega;
        for (int i = 0; i < tau.size(); i++) {
            int dofIndex = ringFinger->getDependentDof(i);
            torques[dofIndex] += tau[i] * 3;
        }
        mTorques += torques;
    }
#endif
    // mimic joint interdependency
    if (abs(mTorques[16]) > 0.2)
        mTorques[17] = mTorques[16] * 0.3;
    if (abs(mTorques[20]) > 0.1)
        mTorques[21] = mTorques[20] * 0.3;
    mFrame++;
}
