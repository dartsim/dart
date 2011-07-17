#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "YUI/Win3D.h"
#include "model3d/Skeleton.h"
#include "utils/LoadOpengl.h";

class MyWindow : public Win3D {
public:
    MyWindow(model3d::Skeleton* _model) : mModel(_model){
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
    
        mDrawMarker = false;
    }

    void draw(){
        mModel->draw(mRI);
        if(mDrawMarker) mModel->drawHandles(mRI);
    }

    void keyboard(unsigned char key, int x, int y){
        switch(key){
        case 'h':
        case 'H':
            mDrawMarker = !mDrawMarker;
            break;
        default:
            Win3D::keyboard(key,x,y);
        }
        glutPostRedisplay();
    }

protected:
    model3d::Skeleton* mModel;
    bool mDrawMarker;
};

#endif
