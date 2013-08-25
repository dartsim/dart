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
            mTrans[1] = 0.f;
            mTrans[2] = -1500.0;
            mFrame = 0;
            mDisplayTimeout = 5;
        }

    virtual ~MyWindow() {};
    
    // Override these virtual functions defined in yui::Win3D
    virtual void displayTimer(int _val);
    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void click(int button, int state, int x, int y);
    virtual void drag(int x, int y);
    
    MyWorld* getWorld() {
        return mWorld;
    }

    void setWorld(MyWorld *_world) {
        mWorld = _world;
    }

protected:
    bool mPlaying;
    int mFrame;
    
    MyWorld *mWorld;
};

#endif
