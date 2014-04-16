#include "JumpController.h"
#include "kinematics/BodyNode.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

JumpController::JumpController(SkeletonDynamics* _skel, double _timestep) : Controller(_skel) {
    mFrame = 0;    
    mTimestep = _timestep;

    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);

    double crouchPose[] =  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.572453, 0.0, 0.0, -1.80614, 0.523637 ,0.0, 0.403953, 0.572453, 0.0, 0.0, -1.80614, 0.523637, 0.0, 0.403953, 0.0, -0.4, 0.0, 0.0, 0.0, -0.421909, -0.0693028, -0.0821567, 0.00587502, 0.273915, 0.227518, 0.421909, 0.0693028, 0.0821567, 0.00587502, -0.273915, 0.227518};

    mDesiredDofs = Eigen::VectorXd::Zero(nDof);
    for (int i = 0; i < nDof; i++)
        mDesiredDofs[i] = crouchPose[i];
            

    // Using SPD results in simple Kp coefficients
    for (int i = 0; i < 6; i++) {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
    for (int i = 6; i < 22; i++)
        mKp(i, i) = 200.0; // lower body gain
    for (int i = 22; i < nDof; i++)
        mKp(i, i) = 20.0; // upper body gain
    for (int i = 6; i < 22; i++) 
        mKd(i, i) = 20.0; // lower body damping
    for (int i = 22; i < nDof; i++) 
        mKd(i, i) = 2.0; // upper body damping
}

void JumpController::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    mTorques.setZero();
    if (mFrame < 600) {
        int nDof = mSkel->getNumDofs();
        MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
        VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
        VectorXd d = -mKd * _dofVel;
        VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mSkel->getConstraintForces());
        mTorques = p + d - mKd * qddot * mTimestep;
    }
        
    if (mFrame > 800 && mFrame < 1000) {
        Vector3d localPoint = mSkel->getNode("fullbody1_h_heel_left")->getLocalCOM();
        Vector3d virtualForce(200, -1385, 0.0);
        MatrixXd jacobian = mSkel->getJacobian(mSkel->getNode("fullbody1_h_heel_left"), localPoint);
        mTorques = jacobian.transpose() * virtualForce;

        jacobian = mSkel->getJacobian(mSkel->getNode("fullbody1_h_heel_right"), localPoint);
        mTorques += jacobian.transpose() * virtualForce;
    }    
    
    if (mFrame > 1000) {
        mDesiredDofs[6] = 1.0;
        mDesiredDofs[9] = -2.0;
        mDesiredDofs[13] = 1.0;
        mDesiredDofs[16] = -2.0;
        mDesiredDofs[21] = -1.5;


        int nDof = mSkel->getNumDofs();
        MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
        VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
        VectorXd d = -mKd * _dofVel;
        VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mSkel->getConstraintForces());
        mTorques = p + d - mKd * qddot * mTimestep;
    }

    if (mFrame > 2000) {
        mDesiredDofs[6] = 0.5;
        mDesiredDofs[9] = -0.5;
        mDesiredDofs[13] = 0.5;
        mDesiredDofs[16] = -0.5;
        mDesiredDofs[21] = -0.5;


        int nDof = mSkel->getNumDofs();
        MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
        VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
        VectorXd d = -mKd * _dofVel;
        VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mSkel->getConstraintForces());
        mTorques = p + d - mKd * qddot * mTimestep;
    }

    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;

    mFrame++;
}
