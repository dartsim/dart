#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "collision/collision_skeleton.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoDof.h"

#include "Controller.h"

using namespace std;

namespace dynamics {
class SkeletonDynamics;
class ContactDynamics;
}

namespace integration {
class IntegrableSystem;
}

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
	//    MyWindow(dynamics::SkeletonDynamics* _m1, dynamics::SkeletonDynamics* _m2)
	MyWindow(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics* _mList = 0, ...) :
			Win3D() {

		mBackground[0] = 1.0;
		mBackground[1] = 1.0;
		mBackground[2] = 1.0;
		mBackground[3] = 1.0;

		mSim = false;
		mPlay = false;
		mSimFrame = 0;
		mPlayFrame = 0;
		mShowMarkers = true;

		mPersp = 45.f;
		mTrans[1] = 200.f;
		mTrans[0] = -200.f;
		mTrans[2] = -1000.f;

		mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
		mTimeStep = 1.0 / 1000.0;
		mForce = Eigen::Vector3d::Zero();
		mImpulseDuration = 0;
		mSelectedNode = 1;

		// Initialize the motion of the skeleton
		mMotion = _motion;

		if(_mList) {
			mSkels.push_back(_mList);
			va_list ap;
			va_start(ap, _mList);
			while(true) {
				dynamics::SkeletonDynamics *skel = va_arg(ap, dynamics::SkeletonDynamics*);
				if(skel) mSkels.push_back(skel);
				else break;
			}
			va_end(ap);
		}

		int sumNDofs = 0;
		mIndices.push_back(sumNDofs);
		for(unsigned int i = 0; i < mSkels.size(); i++) {
			int nDofs = mSkels[i]->getNumDofs();
			sumNDofs += nDofs;
			mIndices.push_back(sumNDofs);
		}
		initDyn();
	}

	virtual void draw();
	virtual void keyboard(unsigned char key, int x, int y);
	virtual void displayTimer(int _val);
  bool takeScreenshot();

	// Needed for integration
	virtual Eigen::VectorXd getState();
	virtual Eigen::VectorXd evalDeriv();
	virtual void setState(Eigen::VectorXd state);
protected:

	kinematics::BodyNode* rightFoot;

	// TO TEST DOFS
	double dDOF;
	////////////////

	Controller *mController;											///< The spd controller
	kinematics::FileInfoDof *mMotion;							///< Describes the predefined motion of the skeleton

	int mSimFrame;
	bool mSim;
	int mPlayFrame;
	bool mPlay;
	bool mShowMarkers;
	int mSelectedNode;
	integration::EulerIntegrator mIntegrator;
	std::vector <Eigen::VectorXd> mBakedStates;

	std::vector <dynamics::SkeletonDynamics*> mSkels;
	dynamics::ContactDynamics *mCollisionHandle;
	std::vector <Eigen::VectorXd> mDofVels;
	std::vector <Eigen::VectorXd> mDofs;
	double mTimeStep;
	Eigen::Vector3d mGravity;
	Eigen::Vector3d mForce;
	std::vector <int> mIndices;
	int mImpulseDuration;

	void initDyn();
	void setPose();
	void bake();
};

#endif
