/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/gui/GlutWindow.hpp"

#include "dart/external/lodepng/lodepng.h"

#ifdef _WIN32
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <direct.h>
#else
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <dirent.h>
#endif
#include <cstdio>
#include <iostream>
#include <vector>

#include "dart/common/Console.hpp"
#include "dart/gui/LoadGlut.hpp"
#include "dart/gui/GLFuncs.hpp"
#include "dart/gui/OpenGLRenderInterface.hpp"

namespace dart {
namespace gui {

std::vector<GlutWindow*> GlutWindow::mWindows;
std::vector<int> GlutWindow::mWinIDs;

GlutWindow::GlutWindow() {
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
  mRI = nullptr;
}

GlutWindow::~GlutWindow() {
  delete mRI;
}

void GlutWindow::initWindow(int _w, int _h, const char* _name) {
  mWindows.push_back(this);

  mWinWidth = _w;
  mWinHeight = _h;

  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE);
  glutInitWindowPosition(150, 100);
  glutInitWindowSize(_w, _h);
  mWinIDs.push_back(glutCreateWindow(_name));

  glutDisplayFunc(refresh);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyEvent);
  glutSpecialFunc(specKeyEvent);
  glutMouseFunc(mouseClick);
  glutMotionFunc(mouseDrag);
  glutPassiveMotionFunc(mouseMove);

  delete mRI;
  mRI = new gui::OpenGLRenderInterface();
  mRI->initialize();
  // glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
  // glutTimerFunc(mDisplayTimeout, runTimer, 0);

#ifndef _WIN32
  glDisable(GL_MULTISAMPLE);
#endif
  // TODO: Disabled use of GL_MULTISAMPLE for Windows. Please see #411 for the
  // detail.

  glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
  // Note: We book the timer id 0 for the main rendering purpose.
}

void GlutWindow::reshape(int _w, int _h) {
  current()->mScreenshotTemp = std::vector<unsigned char>(_w*_h*4);
  current()->mScreenshotTemp2 = std::vector<unsigned char>(_w*_h*4);
  current()->resize(_w, _h);
}

void GlutWindow::keyEvent(unsigned char _key, int _x, int _y) {
  current()->keyboard(_key, _x, _y);
}

void GlutWindow::specKeyEvent(int _key, int _x, int _y) {
  current()->specKey(_key, _x, _y);
}

void GlutWindow::mouseClick(int _button, int _state, int _x, int _y) {
  current()->click(_button, _state, _x, _y);
}

void GlutWindow::mouseDrag(int _x, int _y) {
  current()->drag(_x, _y);
}

void GlutWindow::mouseMove(int _x, int _y) {
  current()->move(_x, _y);
}

void GlutWindow::refresh() {
  current()->render();
}

void GlutWindow::refreshTimer(int _val) {
  current()->displayTimer(_val);
}

void GlutWindow::displayTimer(int _val) {
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void GlutWindow::simTimer(int /*_val*/) {
}

void GlutWindow::runTimer(int _val) {
  current()->simTimer(_val);
}

bool GlutWindow::screenshot() {
  static int count = 0;
  const char directory[8] = "frames";
  const char fileBase[8] = "Capture";
  char fileName[32];

  // create frames directory if not exists
  using Stat = struct stat;
  Stat buff;

#ifdef _WIN32
#define __S_ISTYPE(mode, mask) (((mode) & _S_IFMT) == (mask))
#define S_ISDIR(mode) __S_ISTYPE((mode), _S_IFDIR)
  if (stat(directory, &buff) != 0)
    _mkdir(directory);
#else
  if (stat(directory, &buff) != 0)
    mkdir(directory, 0777);
#endif

  if (!S_ISDIR(buff.st_mode))
  {
    dtwarn << "[GlutWindow::screenshot] 'frames' is not a directory, "
           << "cannot write a screenshot\n";
    return false;
  }

  // png
#ifdef _WIN32
  _snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png",
            directory, "\\", fileBase, count++);
#else
  std::snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png",
                directory, "/", fileBase, count++);
#endif
  int tw = glutGet(GLUT_WINDOW_WIDTH);
  int th = glutGet(GLUT_WINDOW_HEIGHT);

  glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

  // reverse temp2 temp1
  for (int row = 0; row < th; row++) {
    memcpy(&mScreenshotTemp2[row * tw * 4],
           &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
  }

  unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

  // if there's an error, display it
  if (result) {
    std::cout << "lodepng error " << result << ": "
              << lodepng_error_text(result) << std::endl;
    return false;
  } else {
    std::cout << "wrote screenshot " << fileName << "\n";
    return true;
  }
}

inline GlutWindow* GlutWindow::current() {
  int id = glutGetWindow();
  for (unsigned int i = 0; i < mWinIDs.size(); i++) {
    if (mWinIDs.at(i) == id) {
      return mWindows.at(i);
    }
  }
  std::cout << "An unknown error occurred!" << std::endl;
  exit(0);
}

void GlutWindow::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
  // TODO(JS): Is 2d point information necessary for keyboard event?
}

void GlutWindow::specKey(int /*_key*/, int /*_x*/, int /*_y*/) {
}

void GlutWindow::click(int /*_button*/, int /*_state*/, int /*_x*/, int /*_y*/) {
}

void GlutWindow::drag(int /*_x*/, int /*_y*/) {
}

void GlutWindow::move(int /*_x*/, int /*_y*/) {
}

}  // namespace gui
}  // namespace dart
