#ifndef MYWINDOW_H
#define MYWINDOW_H

#include "yui/Win3D.h"
#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"

namespace kinematics {
    class FileInfoDof;
}

namespace integration{
    class IntegrableSystem;
}

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
 MyWindow(dynamics::SkeletonDynamics* _m): Win3D(), mModel(_m) {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mRunning = false;
        mShowMarker = false;

        mPersp = 30.f;
        mTrans[2] = -1.f;
        mFrame = 0;

        mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
        mTimeStep = 1.0/5000.0;
        mController = new Controller(mModel);
        mForce = Eigen::Vector3d::Zero();
        initDyn();
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);
	
    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(Eigen::VectorXd state);	
    
protected:
    bool mRunning;
    int mFrame;
    bool mShowMarker;
    dynamics::SkeletonDynamics* mModel;
    Eigen::VectorXd mDofVels;
    Eigen::VectorXd mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    Controller *mController;
    //integration::EulerIntegrator mIntegrator;
    integration::RK4Integrator mIntegrator;
    Eigen::Vector3d mForce;

    void initDyn();

    //    model3d::FileInfoDof& mMotion;
};

#endif // #ifndef MYWINDOW_H
