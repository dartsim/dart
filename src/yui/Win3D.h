/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author    Sumit Jain
  Date      07/21/2011
*/

#ifndef YUI_WIN3D_H
#define YUI_WIN3D_H

#include "GlutWindow.h"
#include "Trackball.h"
#include <Eigen/Eigen>

namespace yui {

    class Win3D : public GlutWindow {
    public:
        Win3D();

        virtual void initWindow(int w, int h, const char* name);
        virtual void resize(int w, int h);
        virtual void render();

        virtual void keyboard(unsigned char key, int x, int y);
        virtual void click(int button, int state, int x, int y);
        virtual void drag(int x, int y);

        virtual void capturing();
        virtual void initGL();
        virtual void initLights();

        virtual void draw()=0;

    protected:
        Trackball mTrackBall;
        Eigen::Vector3d mTrans;
        Eigen::Vector3d mEye;
        float mZoom;
        float mPersp;

        bool mRotate;
        bool mTranslate;
        bool mZooming;
    };

}   // namespace yui

#endif
