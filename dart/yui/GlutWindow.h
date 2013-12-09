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

#ifndef DART_YUI_GLUTWINDOW_H
#define DART_YUI_GLUTWINDOW_H

#include "renderer/LoadOpengl.h"
#include "renderer/RenderInterface.h"
#include <vector>
#include "lodepng.h"

namespace dart {
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
    std::vector<unsigned char> mScreenshotTemp;
    std::vector<unsigned char> mScreenshotTemp2;
};

} // namespace yui
} // namespace dart

#endif // #ifndef DART_YUI_GLUTWINDOW_H
