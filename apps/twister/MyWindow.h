#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "dart/gui/Win3D.h"
#include "MyWorld.h"

class MyWindow : public dart::gui::Win3D {
public:
 MyWindow(): Win3D() {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;		
        mPersp = 30.f;
        mTrans[1] = 200.f;
        mZoom = 0.3;

        mActiveMarker = -1;
    }

    virtual void draw();
    virtual void keyboard(unsigned char _key, int _x, int _y);
    virtual void click(int _button, int _state, int _x, int _y);
    virtual void drag(int _x, int _y);
    virtual void displayTimer(int _val);

    MyWorld* getWorld() {
        return mWorld;
    }

    void setWorld(MyWorld *_world) {
        mWorld = _world;
    }

 protected:
    MyWorld* mWorld;

    // mActiveMarker is the index of the marker currently selected.
    int mActiveMarker;

    int coordsToMarker(int _x, int _y);
    Eigen::Vector3d reverseProjection(double _x, double _y);
    int processHits(GLint _hits, GLuint _buffer[]);
};

#endif
