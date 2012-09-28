#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "kinematics/FileInfoDof.h"

class MyWindow : public yui::Win3D {
public:
 MyWindow(kinematics::FileInfoDof& _mot): Win3D(), mMainMotion(_mot), mCompareMotion(_mot)
 {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mPlaying = false;
        mShowMarker = false;
        mShowProgress = false;

        mPersp = 30.f;
        mTrans[2] = -1.f;
        mFrame = 0;
        mDisplayTimeout = 5;        
    }

 MyWindow(kinematics::FileInfoDof& _mot1, kinematics::FileInfoDof& _mot2): Win3D(), mMainMotion(_mot1), mCompareMotion(_mot2)
 {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mPlaying = false;
        mShowMarker = false;
        mShowProgress = false;

        mPersp = 30.f;
        mTrans[2] = -1.f;
        mFrame = 0;
        mDisplayTimeout = 5;        
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);
    virtual void move(int _x, int _y);
    void computeMax();
	
protected:
    int mMaxFrame;
    bool mPlaying;
    int mFrame;
    bool mShowMarker;
    bool mShowProgress;
    kinematics::FileInfoDof& mMainMotion;
    kinematics::FileInfoDof& mCompareMotion;
};

#endif
