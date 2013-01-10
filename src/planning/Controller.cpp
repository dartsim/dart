#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "planning/Trajectory.h"
#include "kinematics/Dof.h"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace planning {

const int mode = 0;

Controller::Controller(dynamics::SkeletonDynamics* _skel, const vector<int> &_actuatedDofs) :
    mSkel(_skel),
    mTrajectory(NULL)
{
    const int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Zero(nDof, nDof);
    mKd = MatrixXd::Zero(nDof, nDof);
    mSelectionMatrix = MatrixXd::Zero(nDof, nDof);

	if(mode == 0) {
		for (int i = 0; i < nDof; i++) {
			mKp(i, i) = 15.0;
			mKd(i, i) = 2.0;
		}
	}
	else if(mode == 1) {
		for (int i = 0; i < nDof; i++) {
			mKp(i, i) = 800.0;
			mKd(i, i) = 1.5;
		}
	}
	else {
	    //for (int i = 0; i < _actuatedDofs.size(); i++) {
	    //    mKp(_actuatedDofs[i], _actuatedDofs[i]) = 800.0;
	    //    mKd(_actuatedDofs[i], _actuatedDofs[i]) = 15.0;
	    //}
		for (int i = 0; i < nDof; i++) {
			mKp(i, i) = 15.0;
			mKd(i, i) = 2.0;
		}
	}

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
	if(_time - mStartTime > mTrajectory->getDuration()) {
		cout << "end" << endl;
	}
    if(mTrajectory && _time - mStartTime >= 0.0) {
        for(unsigned int i = 0; i < mTrajectoryDofs.size(); i++) {
            mDesiredDofs[mTrajectoryDofs[i]] = mTrajectory->getPosition(_time - mStartTime)[i];
            desiredDofVels[mTrajectoryDofs[i]] = mTrajectory->getVelocity(_time - mStartTime)[i];
        }
    }
	//cout << "sdfadf " << mTrajectory << "  " << _time << "  " << mStartTime << endl;
    
	VectorXd torques;
	if(mode == 0) {
		const double mTimestep = 0.001;
	    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
	    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
	    VectorXd d = -mKd * _dofVel;
	    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
		torques = p + d - mKd * qddot * mTimestep;
	}
	else if(mode == 1) {
	    torques = -mKp * (_dof - mDesiredDofs) - mKd * (_dofVel - desiredDofVels);
	    //torques = mSkel->getMassMatrix() * torques; // scaled by accumulated mass
	}
	else {
	    const VectorXd accelerations = -mKp * (_dof - mDesiredDofs) - mKd * (_dofVel - desiredDofVels);
	    torques = mSelectionMatrix * mSkel->getMassMatrix() * (accelerations + mSkel->getCombinedVector());
	}
	//cout << torques[28] << endl;
	VectorXd max = 1000.0 * VectorXd::Ones(torques.size());
    return mSelectionMatrix * torques.cwiseMax(-max).cwiseMin(max);
}

}