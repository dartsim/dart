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

#include "dart/gui/Win2D.hpp"

#include <iostream>

#include "dart/gui/LoadGlut.hpp"

namespace dart {
namespace gui {

Win2D::Win2D() : GlutWindow() {
  mTransX = 0;
  mTransY = 0;
  mTranslate = false;
}

void Win2D::resize(int _w, int _h) {
  mWinWidth = _w;
  mWinHeight = _h;

  glViewport(0, 0, _w, _h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-_w/2, _w/2-1, -_h/2, _h/2-1, -1, 1);

  glMatrixMode(GL_MODELVIEW);  // Select The Modelview Matrix
  glLoadIdentity();  // Reset The Modelview Matrix

  glutPostRedisplay();
}

void Win2D::keyboard(unsigned char _key, int /*_x*/, int /*_y*/) {
  switch (_key) {
    case ',':  // slow down
      mDisplayTimeout += 2;
      break;
    case '.':  // speed up
      mDisplayTimeout -= 2;
      if (mDisplayTimeout < 1)
        mDisplayTimeout = 1;
      break;
    case 'c':
    case 'C':  // screen capture
      mCapture = !mCapture;
      break;
    case 27:  // ESC
      exit(0);
  }

  glutPostRedisplay();
  // printf("ascii key: %lu\n", key);
}

void Win2D::click(int /*_button*/, int /*_state*/, int _x, int _y) {
  mMouseDown = !mMouseDown;
  if (mMouseDown) {
    mTranslate = true;

    mMouseX = _x;
    mMouseY = _y;
  } else {
    mTranslate = false;
  }
  glutPostRedisplay();
}

void Win2D::drag(int _x, int _y) {
  if (mMouseDown) {
    mTransX += (_x - mMouseX);
    mTransY += (_y - mMouseY);

    mMouseX = _x;
    mMouseY = _y;
  }
  glutPostRedisplay();
}

void Win2D::render() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-mWinWidth/2, mWinWidth/2-1, -mWinHeight/2, mWinHeight/2-1, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  initGL();

  // transformation
  glTranslatef(mTransX, -mTransY, 0.0);
  draw();

  // draw axis
  // translate back to the center
  glTranslatef(-mTransX, mTransY, 0.0);
  if (mTranslate) {
    glLineWidth(2.0);

    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex2f(-40.f, 0.0f);
    glVertex2f(40.f, 0.0f);
    glEnd();

    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex2f(0.0f, -40.f);
    glVertex2f(0.0f, 40.f);
    glEnd();
  }

  if (mCapture)
    screenshot();

  glutSwapBuffers();
}

void Win2D::initGL() {
  glClearColor(mBackground[0], mBackground[1], mBackground[2], mBackground[3]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
}

}  // namespace gui
}  // namespace dart
