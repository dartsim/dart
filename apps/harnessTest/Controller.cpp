#include "Controller.h"

#include "math/Helpers.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"
#include "dynamics/GenCoord.h"
#include "dynamics/Shape.h"
#include "constraint/ConstraintDynamics.h"
#include "collision/CollisionDetector.h"

using namespace dart;
using namespace dynamics;
using namespace math;

Controller::Controller(dynamics::Skeleton* _skel, constraint::ConstraintDynamics* _collisionHandle, double _t) {
    mSkel = _skel;
    mCollisionHandle = _collisionHandle;
    mTimestep = _t;
    mFrame = 0;
    int nDof = mSkel->getNumGenCoords();
    mKp = Eigen::MatrixXd::Identity(nDof, nDof);
    mKd = Eigen::MatrixXd::Identity(nDof, nDof);
    mConstrForces = Eigen::VectorXd::Zero(nDof);
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getGenCoord(i)->get_q();
    }

    // using SPD results in simple Kp coefficients
    for (int i = 0; i < 6; i++) {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
    for (int i = 6; i < 22; i++)
        mKp(i, i) = 200.0; // lower body + lower back
    for (int i = 22; i < nDof; i++)
        mKp(i, i) = 20.0;
    for (int i = 6; i < 22; i++) 
        mKd(i, i) = 100.0;
    for (int i = 22; i < nDof; i++) 
        mKd(i, i) = 10.0;
        
    mPreOffset = 0.0;
}

void Controller::computeTorques(const Eigen::VectorXd& _dof,
                                const Eigen::VectorXd& _dofVel) {
    // SPD tracking
    int nDof = mSkel->getNumGenCoords();
    Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    Eigen::VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    Eigen::VectorXd d = -mKd * _dofVel;
    Eigen::VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
    mTorques = p + d - mKd * qddot * mTimestep;
    
    // ankle strategy for sagital plane
    Eigen::Vector3d com = mSkel->getWorldCOM();
    Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getWorldTransform() * Eigen::Vector3d(0.05, 0, 0);
    Eigen::Vector2d diff(com[0] - cop[0], com[2] - cop[2]);
    if (diff[0] < 0.1) {
        double offset = com[0] - cop[0];
        double k1 = 20.0;
        double k2 = 10.0;
        double kd = 100.0;
        mTorques[10] += -k1 * offset + kd * (mPreOffset - offset);
        mTorques[12] += -k2 * offset + kd * (mPreOffset - offset);
        mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
        mTorques[19] += -k2 * offset + kd * (mPreOffset - offset);
        mPreOffset = offset;
    }

    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++){        
        mTorques[i] = 0.0;
    }
    mFrame++;
}

