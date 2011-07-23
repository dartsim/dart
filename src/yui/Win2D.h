/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author    Sumit Jain
  Date      07/21/2011
*/

#ifndef YUI_WIN2D_H
#define YUI_WIN2D_H

#include "GlutWindow.h"

namespace yui {

    class Win2D : public GlutWindow
    {
    protected:
        bool mTranslate;
        double mTransX;
        double mTransY;
    public:
        Win2D();

        virtual void resize(int w, int h);
        virtual void render();

        virtual void keyboard(unsigned char key, int x, int y);
        virtual void click(int button, int state, int x, int y);
        virtual void drag(int x, int y);

        virtual void initGL();
        virtual void draw()=0;
    };

}   // namespace yui

#endif
