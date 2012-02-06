#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include "yui/Win3D.h"
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
	double mTimeStep;
	int mFrame;
	bool mSim, mPlay;

	//Integrators
	integration::RK4Integrator mIntegrator;

	//Models
	std::vector<dynamics::SkeletonDynamics*> mSkels;
	dynamics::SkeletonDynamics* mModel, *mModel2;

	//Degrees of freedom
	std::vector<Eigen::VectorXd> mDofs;
	std::vector<Eigen::VectorXd> mDofVels;

	//Collision
	dynamics::ContactDynamics* mCollisionHandle;



	void initDyn();
	void setPose();
};
#endif
