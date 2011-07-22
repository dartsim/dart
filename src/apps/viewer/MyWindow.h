#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "YUI/Win3D.h"
#include "model3d/FileInfoDof.h"

class MyWindow : public Win3D {
public:
    MyWindow(model3d::FileInfoDof& _mot): Win3D(), mMotion(_mot) {
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
    model3d::FileInfoDof& mMotion;
};

#endif
