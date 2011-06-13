#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "YUI/Win3D.h"
#include "renderer/OpenGLRenderInterface.h"
#include "model3d/Skeleton.h"

class MyWindow : public Win3D {
public:
MyWindow(model3d::Skeleton* skel) : mSkel(skel){
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
    }

    virtual void draw(){
        mSkel->draw(&mRenderer, Vector4d(0.0,0.0,0.0,0.0), false);
    }

protected:
    model3d::Skeleton* mSkel;
    Renderer::OpenGLRenderInterface mRenderer;
};

#endif
