#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include "yui/Win3D.h"
#include "Controller.h"
#include "integration/RK4Integrator.h"
#include "collision/collision_skeleton.h"
#include "dynamics/SkeletonDynamics.h"

namespace dynamics{
    class SkeletonDynamics;
	class ContactDynamics;
}

namespace integration{
    class IntegrableSystem;
}


class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
	//Constructor
	MyWindow(dynamics::SkeletonDynamics* _m, dynamics::SkeletonDynamics* _m2);

	// inherited from Win3D
	virtual void draw();
	virtual void keyboard(unsigned char key, int x, int y);
	virtual void displayTimer(int _val);

	// inherited from IntegrableSystem
	virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(Eigen::VectorXd state);

protected:

	//Environment
	Eigen::Vector3d mGravity;
	Eigen::Vector3d mForce, mForce2;
	double mTimeStep;
	int mSimFrame, mPlayFrame;
	bool mSim, mPlay, mDrop;
	std::vector<Eigen::VectorXd> mBakedStates;

	//Integrators
	integration::RK4Integrator mIntegrator;

	//Models
	std::vector<dynamics::SkeletonDynamics*> mSkels;
	dynamics::SkeletonDynamics* mModel, *mModel2;

	//Degrees of freedom
	std::vector<Eigen::VectorXd> mDofs;
	std::vector<Eigen::VectorXd> mDofVels;
	std::vector<int> mIndices;

	//Collision Handlers and Controllers
	dynamics::ContactDynamics* mCollisionHandle;
	Controller *mController;


	void initDyn();
	void setPose();
	void bake();
};
#endif
