#include "SwingController.h"
#include "kinematics/BodyNode.h"
#include "dynamics/PointConstraint.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ConstraintDynamics.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

SwingController::SwingController(SkeletonDynamics* _skel, ConstraintDynamics* _constraintHandle, double _timestep) : Controller(_skel) {
    mFrame = 0;    
    mTimestep = _timestep;
    mConstraintHandle = _constraintHandle;

    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);

    double crouchPose[] =  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.572453, 0.0, 0.0, -1.80614, 0.523637 ,0.0, 0.403953, 0.572453, 0.0, 0.0, -1.80614, 0.523637, 0.0, 0.403953, 0.0, 0.0, 0.0, 0.0, 0.0, -0.421909, -0.0693028, -2.5, 0.75, 0.3, 0.227518, 0.421909, 0.0693028, 2.5, -0.75, 0.3, 0.227518};

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

void SwingController::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    mTorques.setZero();
    if (mFrame < 600) {
        int nDof = mSkel->getNumDofs();
        MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
        VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
        VectorXd d = -mKd * _dofVel;
        VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mSkel->getConstraintForces());
        mTorques = p + d - mKd * qddot * mTimestep;
    }
        
    if (mFrame > 600 && mFrame < 800) {
        Vector3d localPoint = mSkel->getNode("fullbody1_h_heel_left")->getLocalCOM();
        Vector3d virtualForce(-600, -1400, 0.0);
        MatrixXd jacobian = mSkel->getJacobian(mSkel->getNode("fullbody1_h_heel_left"), localPoint);
        mTorques = jacobian.transpose() * virtualForce;

        jacobian = mSkel->getJacobian(mSkel->getNode("fullbody1_h_heel_right"), localPoint);
        mTorques += jacobian.transpose() * virtualForce;

        int nDof = mSkel->getNumDofs();
        MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
        VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
        VectorXd d = -mKd * _dofVel;
        VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mSkel->getConstraintForces());
        VectorXd temp = p + d - mKd * qddot * mTimestep;
        for (int i = 20; i < nDof; i++)
            mTorques[i] += temp[i];
    }    
    
    if (mFrame > 800 && mFrame < 1000) {
        mDesiredDofs[6] = 1.0;
        mDesiredDofs[9] = -2.0;
        mDesiredDofs[13] = 1.0;
        mDesiredDofs[16] = -2.0;
 
        int nDof = mSkel->getNumDofs();
        MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
        VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
        VectorXd d = -mKd * _dofVel;
        VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mSkel->getConstraintForces());
        mTorques = p + d - mKd * qddot * mTimestep;
    }

    if (mFrame == 1000) {
        BodyNodeDynamics *bd = (BodyNodeDynamics*)mSkel->getNode("fullbody1_h_hand_left");
        PointConstraint* point1 = new PointConstraint(bd, bd->getLocalCOM(), bd->getWorldCOM(), mSkel->getIndex());
        mConstraintHandle->addConstraint(point1);
        bd = (BodyNodeDynamics*)mSkel->getNode("fullbody1_h_hand_right");
        PointConstraint* point2 = new PointConstraint(bd, bd->getLocalCOM(), bd->getWorldCOM(), mSkel->getIndex());
        mConstraintHandle->addConstraint(point2);
    }
    
    if (mFrame >= 1000 ) {
       int nDof = mSkel->getNumDofs();
        MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
        VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
        VectorXd d = -mKd * _dofVel;
        VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mSkel->getConstraintForces());
        VectorXd temp = p + d - mKd * qddot * mTimestep;
        for (int i = 6; i < nDof; i++)
            mTorques[i] += temp[i];
    }

    if (mFrame == 1500) {
        int nConstr = mConstraintHandle->getNumConstraints();
        for (int i = nConstr - 1; i >= 0; i--)
            mConstraintHandle->deleteConstraint(mConstraintHandle->getConstraint(i));
    }
    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;

    mFrame++;
}
