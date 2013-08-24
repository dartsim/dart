#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "MyWorld.h"

class MyWindow : public yui::Win3D {
public:
 MyWindow(): Win3D()
 {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mPlaying = false;

        mPersp = 30.f;
        mTrans[2] = -1.f;
        mFrame = 0;
        mDisplayTimeout = 5;
    }

    virtual ~MyWindow() {};
    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);
	
protected:
    bool mPlaying;
    int mFrame;
    
    MyWorld mWorld1;
    MyWorld mWorld2;
    MyWorld mWorld3;
};

#endif
