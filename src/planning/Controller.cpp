#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "planning/Trajectory.h"
#include "kinematics/Dof.h"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace planning {

Controller::Controller(dynamics::SkeletonDynamics* _skel, const vector<int> &_actuatedDofs,
                       const VectorXd &_kI, const VectorXd &_kP, const VectorXd &_kD) :
    mSkel(_skel),
    mIntegratedError(VectorXd::Zero(mSkel->getNumDofs())),
    mKi(_kI.asDiagonal()),
    mKp(_kP.asDiagonal()),
    mKd(_kD.asDiagonal()),
    mTrajectory(NULL)
{
    const int nDof = mSkel->getNumDofs();

    mSelectionMatrix = MatrixXd::Zero(nDof, nDof);
    for (int i = 0; i < _actuatedDofs.size(); i++) {
        mSelectionMatrix(_actuatedDofs[i], _actuatedDofs[i]) = 1.0;
    }

    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++) {
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }
}


void Controller::setTrajectory(const Trajectory* _trajectory, double _startTime, const std::vector<int> &_dofs) {
    mTrajectoryDofs = _dofs;
    mTrajectory = _trajectory;
    mStartTime = _startTime;
}


VectorXd Controller::getTorques(const VectorXd& _dof, const VectorXd& _dofVel, double _time) {
    Eigen::VectorXd desiredDofVels = VectorXd::Zero(mSkel->getNumDofs());

    if(mTrajectory && _time - mStartTime >= 0.0 & _time - mStartTime <= mTrajectory->getDuration()) {
        for(unsigned int i = 0; i < mTrajectoryDofs.size(); i++) {
            mDesiredDofs[mTrajectoryDofs[i]] = mTrajectory->getPosition(_time - mStartTime)[i];
            desiredDofVels[mTrajectoryDofs[i]] = mTrajectory->getVelocity(_time - mStartTime)[i];
        }
    }
    
    VectorXd torques;
    const double mTimestep = 0.001;
    mIntegratedError += mTimestep * (_dof - mDesiredDofs);
    torques = -mKi * mIntegratedError - mKp * (_dof - mDesiredDofs) - mKd * (_dofVel - desiredDofVels);
    VectorXd max = 1000.0 * VectorXd::Ones(torques.size());
    return mSelectionMatrix * torques.cwiseMax(-max).cwiseMin(max);
}

}