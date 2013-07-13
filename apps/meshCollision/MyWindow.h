#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include <iostream>
#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "collision/CollisionDetector.h"
#include "dynamics/SkeletonDynamics.h"

using namespace std;
using namespace Eigen;

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
		
        mDisplayFrequency = 16;
        mPlayState = PAUSED;
        mPlayStateLast = SIMULATE;
        mSimFrame = 0;
        mPlayFrame = -mDisplayFrequency;
        mMovieFrame = -mDisplayFrequency;
        mScreenshotScheduled = false;

        mShowMarkers = true;

        mPersp = 45.f;
        mTrans[1] = 300.f;
    
        mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
        mTimeStep = 1.0/1000.0;
        mForce = Eigen::Vector3d::Zero();
        mImpulseDuration = 0;
        mSelectedNode = 1;

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

        std::cout << 
            "\nKeybindings:\n" <<
            "\n" <<
            "s: start or continue simulating.\n" <<
            "\n" <<
            "p: start or continue playback.\n" <<
            "r, t: move to start or end of playback.\n" <<
            "[, ]: step through playback by one frame.\n" <<
            "\n" <<
            "m: start or continue movie recording.\n" <<
            "\n" <<
            "space: pause/unpause whatever is happening.\n" <<
            "\n" <<
            "q, escape: quit.\n" <<
            std::endl;
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(const Eigen::VectorXd &state);	
 protected:	
    enum playstate_enum {
        SIMULATE = 3,
        RECORD = 2,
        PLAYBACK = 1,
        PAUSED = 0
    };
    playstate_enum mPlayState;
    playstate_enum mPlayStateLast;
    int mSimFrame;
    int mPlayFrame;
    int mMovieFrame;
    bool mScreenshotScheduled;
    int mDisplayFrequency;

    bool mShowMarkers;
    int mSelectedNode;
    integration::EulerIntegrator mIntegrator;
    std::vector<Eigen::VectorXd> mBakedStates;

    std::vector<dynamics::SkeletonDynamics*> mSkels;
    dynamics::ContactDynamics *mCollisionHandle;
    std::vector<Eigen::VectorXd> mDofVels;
    std::vector<Eigen::VectorXd> mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    Eigen::Vector3d mForce;
    std::vector<int> mIndices;
    int mImpulseDuration;

    void drawContact(Vector3d vertex, Vector3d force, Vector3d penColor, Vector3d ellipsoidColor);
    void drawText();
    void initDyn();
    void setPose();
    void bake();
    void retrieveBakedState(int frame);
};

#endif
