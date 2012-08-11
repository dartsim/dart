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
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
    mTorques = p + d - mKd * qddot * mTimestep;

    // change plam orientation 
    /*
    if (mFrame == 1010) {
        BodyNode *palm = mSkel->getNode("manipulator_wrist");
        Vector3d xAxis = -_objVel.head(3).normalized();
        Vector3d yAxis = xformHom(palm->getWorldTransform(), Vector3d(0, 1, 0));
        Vector3d zAxis = xAxis.cross(yAxis);
        //        cout << "x " << xAxis << endl;
        //        cout << "y " << yAxis << endl;
        //        cout << "z " << zAxis << endl;
        double alpha = 1.0;
        Vector3d omega = zAxis * alpha;
        MatrixXd Jw = palm->getJacobianAngular();
        VectorXd vel = Jw.transpose() * (Jw * Jw.transpose()).inverse() * omega;
        //        cout << "vel " << vel << endl;
        int nDof = mSkel->getNumDofs();
        VectorXd tau = (vel - _dofVel.head(4)) / mTimestep;
        cout << "SPD " << mTorques << endl;
        mTorques.head(4) += tau * 0.1;
        cout << "palm " << tau << endl;
        }*/
    
    mFrame++;
}
