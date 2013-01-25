#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include "yui/Win3D.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoDof.h"

class Analyzer;

class MyWindow : public yui::Win3D {
public:
 MyWindow(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, ...): Win3D() {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mPlay = false;
        mPlayFrame = 0;

        mPersp = 30.f;
        mTrans[2] = -1.f;
    
        mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
        mTimeStep = 1.0/1000.0;

        mMotion = _motion;
        mSkel = _skel;
        
        initDyn();
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

 protected:	
    kinematics::FileInfoDof *mMotion;
    int mPlayFrame;
    bool mPlay;

    dynamics::SkeletonDynamics* mSkel;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    Analyzer *mAnalyzer;

    void initDyn();
};

#endif
