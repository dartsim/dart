#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"

#define DAMPING

    namespace dynamics {
        class SkeletonDynamics;
        class ConstraintDynamics;
    }

    namespace integration {
        class IntegrableSystem;
    }

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
    MyWindow(dynamics::SkeletonDynamics* _m): Win3D() {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mRunning = false;

        mPersp = 45.f;
        mTrans[1] = 300.f;
        mFrame = 0;
    
        mSkels.push_back(_m);

        mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
        mTimeStep = 1.0/2000.0;
        initDyn();
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(const Eigen::VectorXd& state);	
protected:	
    bool mRunning;
    int mFrame;
    // Select an integrator
    integration::EulerIntegrator mIntegrator;
    //integration::RK4Integrator mIntegrator;
    
    std::vector<dynamics::SkeletonDynamics*> mSkels;
    dynamics::ConstraintDynamics *mConstraintHandle;
    Eigen::VectorXd mDofVels;
    Eigen::VectorXd mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    void initDyn();
#ifdef DAMPING    
    Eigen::VectorXd computeDamping();
#endif
};

#endif
