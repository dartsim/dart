/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
              Saul Reynolds-Haertle <saulrh@gatech.edu>
 * Date: 07/21/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#if !WIN32
#include <dirent.h>
#endif

#include <cstdio>
#include <iostream>
#include "GlutWindow.h"
#include "yui/GLFuncs.h"
#include "renderer/OpenGLRenderInterface.h"

namespace dart {
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
    current()->mScreenshotTemp = std::vector<unsigned char>(w*h*4);
    current()->mScreenshotTemp2 = std::vector<unsigned char>(w*h*4);
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

    glReadPixels( 0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

    // reverse temp2 temp1
    for (int row = 0; row < th; row++) {
        memcpy(&mScreenshotTemp2[row * tw * 4], &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
    }

    unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

    //if there's an error, display it
    if(result) {
        std::cout << "lodepng error " << result << ": "<< lodepng_error_text(result) << std::endl;
        return false;
    }
    else {
        std::cout << "wrote screenshot " << fileName << "\n";
        return true;
    }
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

} // namespace yui
} // namespace dart
