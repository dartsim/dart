#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include "yui/Win3D.h"
#include "Controller.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
<<<<<<< HEAD
#include "collision/collision_skeleton.h"
=======
#include "collision/CollisionSkeleton.h"
>>>>>>> karen
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
    //    MyWindow(dynamics::SkeletonDynamics* _m1, dynamics::SkeletonDynamics* _m2)
 MyWindow(dynamics::SkeletonDynamics* _mList = 0, ...): Win3D() {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mSim = false;
        mPlay = false;
        mSimFrame = 0;
        mPlayFrame = 0;
        mShowMarkers = true;
        mImpulseDuration = 0;


        mPersp = 45.f;
        mTrans[1] = 300.f;
    
        mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
        mTimeStep = 1.0/1000.0;
        mForce = Eigen::Vector3d::Zero();

        if (_mList) {
            mSkels.push_back(_mList);
            va_list ap;
            va_start(ap, _mList);
            while (true) {
                dynamics::SkeletonDynamics *skel = va_arg(ap, dynamics::SkeletonDynamics*);
                if(skel)
                    mSkels.push_back(skel);
                else
                    break;
            }
            va_end(ap);
        }
        
        int sumNDofs = 0;
        mIndices.push_back(sumNDofs);
        for (unsigned int i = 0; i < mSkels.size(); i++) {
            int nDofs = mSkels[i]->getNumDofs();
            sumNDofs += nDofs;
            mIndices.push_back(sumNDofs);
        }
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
    bool mPlay;
    bool mShowMarkers;
<<<<<<< HEAD
    integration::EulerIntegrator mIntegrator;
    //    integration::RK4Integrator mIntegrator;
    std::vector<Eigen::VectorXd> mBakedStates;
=======
    //integration::EulerIntegrator mIntegrator;
    integration::RK4Integrator mIntegrator;
    std::vector<Eigen::VectorXd> mBakedStates;
    void pushState();
    void popState();
    bool evaluateSample(Eigen::VectorXd& _sample);
    void sampleControl();
>>>>>>> karen

    std::vector<dynamics::SkeletonDynamics*> mSkels;
    dynamics::ContactDynamics *mCollisionHandle;
    std::vector<Eigen::VectorXd> mDofVels;
    std::vector<Eigen::VectorXd> mDofs;
<<<<<<< HEAD
=======
    std::vector<Eigen::VectorXd> mStoredDofVels;
    std::vector<Eigen::VectorXd> mStoredDofs;
>>>>>>> karen
    double mTimeStep;
    Eigen::Vector3d mGravity;
    Eigen::Vector3d mForce;
    std::vector<int> mIndices;
    Controller *mController;
    int mImpulseDuration;
<<<<<<< HEAD
=======
    Eigen::VectorXd mControlBias;
    Eigen::VectorXd mBestTorques;
    double mBestScore;
    double mBestAlpha;
>>>>>>> karen

    void initDyn();
    void setPose();
    void bake();
<<<<<<< HEAD
=======
    Eigen::Vector3d evalLinMomentum(dynamics::SkeletonDynamics* _skel, const Eigen::VectorXd& _dofVel);
    Eigen::Vector3d evalAngMomentum(dynamics::SkeletonDynamics* _skel, const Eigen::VectorXd& _dofVel);

>>>>>>> karen
};

#endif
