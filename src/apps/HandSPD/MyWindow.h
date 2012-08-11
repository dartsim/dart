#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include <stdio.h>
#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoDof.h"

class Controller;

/// The main interface
class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:

	/// The constructor
	MyWindow(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics* _mList = 0, ...) :
			Win3D() {

		// Set the background color
		mBackground[0] = 1.0;
		mBackground[1] = 1.0;
		mBackground[2] = 1.0;
		mBackground[3] = 1.0;

		// Set the simulator and replay options
		mSim = false;
		mPlay = false;
		mSimFrame = 0;
		mPlayFrame = 0;

		// Set the camera view angle and translation from the origin (OpenGL)
		mPersp = 60.f;
		mTrans[2] = -1.f;

		// Set the simulation properties: gravity, time step and ?
		mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
		mTimeStep = 1.0 / 1000.0;
		mForce = Eigen::Vector3d::Zero();

		// Initialize the motion of the skeleton
		mMotion = _motion;
		if(_mList) {

			// Get the skeletons set in main.cpp - the hand skeleton in this case.
			mSkels.push_back(_mList);
			va_list ap;
			va_start(ap, _mList);
			while (true) {
				dynamics::SkeletonDynamics *skel = va_arg(ap, dynamics::SkeletonDynamics*);
				if(skel)
					mSkels.push_back(skel);
				else break;
			}
			va_end(ap);
		}

		// Get the number of degrees in the world - as indices to the state (?)
		// Each index in mIndices creates a range for the nDofs of the skeleton it corresponds to.
		int sumNDofs = 0;
		mIndices.push_back(sumNDofs);
		for (unsigned int i = 0; i < mSkels.size(); i++) {
			int nDofs = mSkels[i]->getNumDofs();
			sumNDofs += nDofs;
			mIndices.push_back(sumNDofs);
		}

		// Initialize the dynamics
		initDyn();
	}

public:

	virtual void draw();
	virtual void keyboard(unsigned char key, int x, int y);
	virtual void displayTimer(int _val);

	// Needed for integration
	virtual Eigen::VectorXd getState();
	virtual Eigen::VectorXd evalDeriv();
	virtual void setState(Eigen::VectorXd state);

protected:

	int mSimFrame;				///< The frame the simulation is in
	bool mSim;						///< Indicates whether the simulation is on
	int mPlayFrame;				///< The frame the replay is in
	bool mPlay;						///< Indicates whether the replay is on

	kinematics::FileInfoDof *mMotion;							///< Describes the predefined motion of the skeleton
	integration::EulerIntegrator mIntegrator;
	std::vector <Eigen::VectorXd> mBakedStates;

	std::vector <dynamics::SkeletonDynamics*> mSkels;			///< The skeletons in the world
	std::vector <Eigen::VectorXd> mDofVels;								///< The velocities at dofs of each skeleton
	std::vector <Eigen::VectorXd> mDofs;									///< The positions at dofs of each skeleton

	double mTimeStep;											///< The simulation time step
	Eigen::Vector3d mGravity;							///< The gravity in the world
	Eigen::Vector3d mForce;								///< The force with which the finger is moved

	std::vector <int> mIndices;						///< Indices to the state
	Controller *mController;							///< The spd controller

	void initDyn();
	void setPose();
	void bake();
};

#endif
