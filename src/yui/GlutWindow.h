/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author    Sumit Jain
  Date      07/21/2011
*/

#ifndef YUI_GLUTWINDOW_H
#define YUI_GLUTWINDOW_H

#include "utils/LoadOpengl.h"
#include "renderer/RenderInterface.h"
#include <vector>

namespace yui {
    class GlutWindow
    {
    public:
        GlutWindow();
        virtual ~GlutWindow();
        virtual void initWindow(int w, int h, const char* name);

        // callback functions	
        static void reshape(int w, int h);
        static void keyEvent(unsigned char key, int x, int y);
        static void specKeyEvent( int key, int x, int y );
        static void mouseClick(int button, int state, int x, int y);
        static void mouseDrag(int x, int y);
        static void mouseMove(int x, int y);
        static void refresh();
        static void refreshTimer(int);
        static void runTimer(int);

        static GlutWindow* current();
        static std::vector<GlutWindow*> mWindows;
        static std::vector<int> mWinIDs;

    protected:
        // callback implementation	
        virtual void resize(int w, int h)=0;
        virtual void render()=0;
        virtual void keyboard(unsigned char key, int x, int y){}
        virtual void specKey( int key, int x, int y){}
        virtual void click(int button, int state, int x, int y){}
        virtual void drag(int x, int y){}
        virtual void move(int x, int y){}
        virtual void displayTimer(int);
        virtual void simTimer(int){}

        virtual bool screenshot();

        int mWinWidth;
        int mWinHeight;
        int mMouseX;
        int mMouseY;
        double mDisplayTimeout;
        bool mMouseDown;
        bool mMouseDrag;
        bool mCapture;
        double mBackground[4];
        renderer::RenderInterface* mRI;
    };

} // namespace yui

#endif // YUI_GLUTWINDOW_H
