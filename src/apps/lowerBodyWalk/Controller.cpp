#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/FileInfoDof.h"

using namespace Eigen;

Controller::Controller(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, double _t) {
	mMotion = _motion;
	mSkel = _skel;
	mTimestep = _t;
	int nDof = mSkel->getNumDofs();
//	int nDof = mSkel->getNumDofs() - 6;
	mKp = MatrixXd::Identity(nDof, nDof);
	mKd = MatrixXd::Identity(nDof, nDof);

	mTorques.resize(nDof);
	mDesiredDofs.resize(nDof);
	for(int i = 0; i < nDof; i++) {
		mTorques[i] = 0.0;
		mDesiredDofs[i] = mSkel->getDof(i)->getValue();
//		mDesiredDofs[i] = mSkel->getDof(i+6)->getValue();
	}

	for(int i = 0; i < nDof; i++) {
		mKp(i, i) = 150.0;
		mKd(i, i) = 15.0;
	}

	mFrame = 0;
	mInterval = (1.0 / mMotion->getFPS()) / mTimestep;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel, VectorXd* optionalGoal) {

	// Get the desired DOFs for the current frame
	int frame = mFrame / mInterval;
	if(frame >= mMotion->getNumFrames()) frame = mMotion->getNumFrames() - 1;
	mDesiredDofs = mMotion->getPoseAtFrame(frame);
	if(optionalGoal != NULL) {
		mDesiredDofs += *optionalGoal;
	}

	// Print debug statements
	static const bool debug = true;
	if(debug) {
		std::cout << "\n\n===================================================" << std::endl;
		std::cout << "_dof: " << _dof.transpose() << std::endl;
		std::cout << "_dofVel: " << _dofVel.transpose() << std::endl;
		std::cout << "mDesiredDofs: " << mDesiredDofs.transpose() << std::endl;
		std::cout << "mTorques: " << mTorques.transpose() << std::endl;
	}

	// Compute the torques
	MatrixXd temp = mSkel->getMassMatrix();
	MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
	VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
	VectorXd d = -mKd * _dofVel;
	VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
	mTorques = p + d - mKd * qddot * mTimestep;

	for(size_t i = 0; i < 6; i++)
		mTorques(i) = 0;

	// Increment the frame counter
	mFrame++;
}
