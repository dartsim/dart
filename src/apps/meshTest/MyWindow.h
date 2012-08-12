#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include "yui/Win3D.h"
#include "dynamics/JointLimitDynamics.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"

using namespace std;

namespace dynamics{
    class SkeletonDynamics;
    class ContactDynamics;
}

namespace integration{
    class IntegrableSystem;
}

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
    //    MyWindow(dynamics::SkeletonDynamics* _m1, dynamics::SkeletonDynamics* _m2)
 MyWindow(dynamics::SkeletonDynamics* _m): Win3D() {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mSim = false;
        mPlay = false;
        mFrame = 0;
        mSimFrame = 0;
        mPlayFrame = 0;
        mShowMarkers = false;

        mPersp = 60.f;
        mTrans[1] = 250.f;
    
        mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
        mTimeStep = 1.0/1000.0;
        mForce = Eigen::Vector3d::Zero();
        mImpulseDuration = 0;
        mSelectedNode = 1;

        mModel = _m; 
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
    int mSimFrame;
    bool mSim;
    int mPlayFrame;
    int mFrame;
    bool mPlay;
    bool mShowMarkers;
    int mSelectedNode;
    double mImpulseDuration;
    Eigen::Vector3d mForce;
    integration::EulerIntegrator mIntegrator;

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
