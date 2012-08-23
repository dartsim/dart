#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "dynamics/BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;
using namespace utils;


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
    VectorXd damping = d - mKd * invM * (-mSkel->getCombinedVector() + d) * mTimestep;
    VectorXd tracking = p - mKd * invM * p * mTimestep;
    //    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
    //    mTorques = p + d - mKd * qddot * mTimestep;
    mTorques += tracking;
    // change plam orientation 
    
    if (mFrame > 1015 && mFrame < 1035) {
        BodyNode *midFinger = mSkel->getNode("manipulator_middlePIP");
        BodyNode *ringFinger = mSkel->getNode("manipulator_ringPIP");
        //        Vector3d xAxis = -_objVel.head(3).normalized();
        //        Vector3d yAxis = xformHom(midFinger->getWorldTransform(), Vector3d(0, 1, 0));
        //  Vector3d zAxis = xAxis.cross(yAxis);
        Vector3d zAxis(0, 0, 1);
        
        //        cout << "x " << xAxis << endl;
        //        cout << "y " << yAxis << endl;
        //        cout << "z " << zAxis << endl;
        double alpha = 1.0;
        Vector3d omega = zAxis * alpha;
        MatrixXd Jw = midFinger->getJacobianAngular();
        //        VectorXd vel = Jw.transpose() * (Jw * Jw.transpose()).inverse() * omega;
        VectorXd tau = Jw.transpose() * omega;
        VectorXd torques(mTorques.size());
        torques.setZero();
        //        cout << "vel " << vel << endl;
        //        VectorXd tau = (vel - _dofVel.head(4)) / mTimestep;
        //        cout << "SPD " << mTorques << endl;
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
        // approximate joint interdependency
        //        torques[17] = torques[16] * 0.3;
        //        torques[21] = torques[20] * 0.3;
        mTorques += torques;
    }
    mTorques += damping;
    if (abs(mTorques[16]) > 0.2)
        mTorques[17] = mTorques[16] * 0.3;
    if (abs(mTorques[20]) > 0.1)
        mTorques[21] = mTorques[20] * 0.3;
    mFrame++;
}
