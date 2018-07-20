/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/gui/glfw/WorldWindow.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
WorldWindow::WorldWindow()
{
  // Do nothing
}

//==============================================================================
WorldWindow::~WorldWindow()
{
  // Do nothing
}

//==============================================================================
void WorldWindow::renderScene()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(
      static_cast<GLdouble>(mPersp),
      static_cast<GLdouble>(mWindowWidth)
          / static_cast<GLdouble>(mWindowHeight),
      0.1,
      10.0);
  gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  glShadeModel(GL_SMOOTH);
  glPolygonMode(GL_FRONT, GL_FILL);

  mTrackBall.applyGLRotation();

  // Draw world origin indicator
  if (!mCapture)
  {
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glLineWidth(2.0);
    if (mRotate || mTranslate || mZooming) {
      glColor3f(1.0f, 0.0f, 0.0f);
      glBegin(GL_LINES);
      glVertex3f(-0.1f, 0.0f, -0.0f);
      glVertex3f(0.15f, 0.0f, -0.0f);
      glEnd();

      glColor3f(0.0f, 1.0f, 0.0f);
      glBegin(GL_LINES);
      glVertex3f(0.0f, -0.1f, 0.0f);
      glVertex3f(0.0f, 0.15f, 0.0f);
      glEnd();

      glColor3f(0.0f, 0.0f, 1.0f);
      glBegin(GL_LINES);
      glVertex3f(0.0f, 0.0f, -0.1f);
      glVertex3f(0.0f, 0.0f, 0.15f);
      glEnd();
    }
  }

  glScalef(mZoom, mZoom, mZoom);
  glTranslated(mTrans[0]*0.001, mTrans[1]*0.001, mTrans[2]*0.001);

  // TODO: Set lights

  renderWorld();

  // Draw trackball indicator
//  if (mRotate && !mCapture)
  //    mTrackBall.draw(mWinWidth, mWinHeight);
}

//==============================================================================
void WorldWindow::renderWorld()
{
  // TODO(JS): Not implemented
}

} // namespace glfw
} // namespace gui
} // namespace dart
