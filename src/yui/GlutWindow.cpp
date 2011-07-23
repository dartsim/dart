/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author    Sumit Jain
  Date      07/21/2011
*/

#if !WIN32
#include <dirent.h>
#endif

#include <cstdio>
#include <iostream>
using namespace std;
#include "GlutWindow.h"
#include "yui/GLFuncs.h"
#include "renderer/OpenGLRenderInterface.h"

namespace yui {

    std::vector<GlutWindow*> GlutWindow::mWindows;
    std::vector<int> GlutWindow::mWinIDs;

    GlutWindow::GlutWindow()
    {
        mWinWidth = 0;
        mWinHeight = 0;
        mMouseX = 0;
        mMouseY = 0;
        mDisplayTimeout = 1000.0/30.0;
        mMouseDown = false;
        mMouseDrag = false;
        mCapture = false;
        mBackground[0] = 0.3;
        mBackground[1] = 0.3;
        mBackground[2] = 0.3;
        mBackground[3] = 1.0;
        mRI = NULL;
    }

    GlutWindow::~GlutWindow()
    {
        if (mRI)
            delete mRI;
    }

    void GlutWindow::initWindow(int w, int h, const char* name)
    {
        mWindows.push_back(this);

        mWinWidth = w;
        mWinHeight = h;

        glutInitDisplayMode( GLUT_DEPTH | GLUT_DOUBLE |GLUT_RGBA  | GLUT_STENCIL | GLUT_ACCUM);
        glutInitWindowPosition( 150, 100 );
        glutInitWindowSize( w, h );
        mWinIDs.push_back(glutCreateWindow( name ));

        glutDisplayFunc( refresh );
        glutReshapeFunc( reshape );
        glutKeyboardFunc( keyEvent );
        glutSpecialFunc( specKeyEvent );
        glutMouseFunc( mouseClick );
        glutMotionFunc( mouseDrag );
        glutPassiveMotionFunc( mouseMove );

        if (mRI)
            delete mRI;
        mRI = new renderer::OpenGLRenderInterface();
        mRI->initialize();
        //glutTimerFunc ( mDisplayTimeout, refreshTimer, 0 );
        //glutTimerFunc ( mDisplayTimeout, runTimer, 0 );
    }

    void GlutWindow::reshape(int w, int h)
    {
        current()->resize(w,h);
    }

    void GlutWindow::keyEvent(unsigned char key, int x, int y)
    {
        current()->keyboard(key, x, y);
    }

    void GlutWindow::specKeyEvent(int key, int x, int y)
    {
        current()->specKey(key, x, y);
    }

    void GlutWindow::mouseClick(int button, int state, int x, int y)
    {
        current()->click(button, state, x, y);
    }

    void GlutWindow::mouseDrag(int x, int y)
    {
        current()->drag(x, y);
    }

    void GlutWindow::mouseMove(int x, int y)
    {
        current()->move(x, y);
    }

    void GlutWindow::refresh()
    {
        current()->render();
    }

    void GlutWindow::refreshTimer(int val)
    {
        current()->displayTimer(val);
    }

    void GlutWindow::displayTimer(int val)
    {
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, val);
    }

    void GlutWindow::runTimer(int val)
    {
        current()->simTimer(val);
    }

    bool GlutWindow::screenshot(){
        static int count=0;
        char fileBase[32]="frames/Capture";
        char fileName[64];
        // png
        sprintf(fileName, "%s%.4d.png", fileBase, count++); 
        int tw = glutGet(GLUT_WINDOW_WIDTH);
        int th = glutGet(GLUT_WINDOW_HEIGHT);
        bool antiAlias = true;
        if(yui::screenShot(FIF_PNG, tw, th, fileName, antiAlias)){
            cout<<fileName<<" generated\n";
            return true;
        }
        //sprintf(fileName, "%s%.4d.tga", fileBase, count++); 
        //screenShot(tw, th, fileName, antiAlias);
        return false;
    }

    inline GlutWindow* GlutWindow::current()
    {
        int id = glutGetWindow();
        for(unsigned int i=0; i<mWinIDs.size(); i++){
            if(mWinIDs.at(i) == id){
                return mWindows.at(i);
            }
        }
        std::cout<<"An unknown error occured!"<<std::endl;
        exit(0);
    }
}   // namespace  yui
