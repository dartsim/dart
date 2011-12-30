#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "collision/collision_shapes.h"
#include "collision/collision_skeleton.h"

using namespace collision_checking;
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
    MyWindow(dynamics::SkeletonDynamics* _m, dynamics::SkeletonDynamics* _m2): Win3D(), mModel(_m), mModel2(_m2) {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mRunning = false;
        mShowMarker = false;
        mPlayBack = false;
        mCurrFrame = 0;

        mPersp = 45.f;
        mTrans[1] = 300.f;
        mFrame = 0;
    
        mGravity = Eigen::Vector3d(0.0,-9.8, 0.0);
        mTimeStep = 1.0/1000.0;
        mSkels.push_back(mModel);
        mSkels.push_back(mModel2);
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
    bool mPlayBack;
    int mCurrFrame;
    std::vector<Eigen::VectorXd> mBakedStates;

    integration::EulerIntegrator mIntegrator;
    
    std::vector<dynamics::SkeletonDynamics*> mSkels;
    dynamics::ContactDynamics *mCollisionHandle;
    dynamics::SkeletonDynamics* mModel;
    dynamics::SkeletonDynamics* mModel2;
    Eigen::VectorXd mDofVels;
    Eigen::VectorXd mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    SkeletonCollision mContactCheck;


    void initDyn();
    void setPose();
    void bake();

    
};

#endif
