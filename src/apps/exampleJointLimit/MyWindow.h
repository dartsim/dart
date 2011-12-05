#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"

namespace dynamics{
    class SkeletonDynamics;
    class JointLimitDynamics;
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

        mPersp = 45.f;
        mTrans[1] = 300.f;
        mFrame = 0;
        mForce = Eigen::Vector3d::Zero();
    
        mGravity = Eigen::Vector3d(0.0,-9.8, 0.0);
        mTimeStep = 1.0/5000.0;
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
    Eigen::Vector3d mForce;
    // Select an integrator
    integration::EulerIntegrator mIntegrator;
    //    integration::RK4Integrator mIntegrator;
    
    dynamics::SkeletonDynamics* mModel;
    dynamics::JointLimitDynamics *mJointLimitConstr;
    Eigen::VectorXd mDofVels;
    Eigen::VectorXd mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    void initDyn();
    void setPose();
};

#endif
