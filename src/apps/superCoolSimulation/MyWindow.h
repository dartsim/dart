//
#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "integration/RK4Integrator.h"

namespace dynamics{
    class SkeletonDynamics;
}

namespace integration{
    class IntegrableSystem;
}


class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
	//Constructor
	MyWindow(dynamics::SkeletonDynamics* _m);

	// inherited from Win3D
	virtual void draw();
	virtual void keyboard(unsigned char key, int x, int y);
	virtual void displayTimer(int _val);

	// inherited from IntegrableSystem
	virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(Eigen::VectorXd state);

protected:


	//Integrators
	integration::RK4Integrator mIntegrator;

	//Models
	dynamics::SkeletonDynamics* mModel;

	//Degrees of freedom
	Eigen::VectorXd mDofs;
	Eigen::VectorXd mDofVels;

	//Environment
	Eigen::Vector3d mGravity;
	double mTimeStep;
	int mFrame;

	void initDyn();
	void setPose();
};
#endif
