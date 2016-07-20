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

#include "dart/gui/Win3D.hpp"

#include <algorithm>

#include "dart/gui/LoadGlut.hpp"

namespace dart {
namespace gui {

Win3D::Win3D()
  : GlutWindow(),
    mTrans(0.0, 0.0, 0.0),
    mEye(0.0, 0.0, 1.0),
    mUp(0.0, 1.0, 0.0),
    mZoom(1.0),
    mPersp(45.0),
    mRotate(false),
    mTranslate(false),
    mZooming(false) {
}

void Win3D::initWindow(int _w, int _h, const char* _name) {
  GlutWindow::initWindow(_w, _h, _name);

  int smaller = _w < _h ? _w : _h;
  mTrackBall.setTrackball(Eigen::Vector2d(_w*0.5, _h*0.5), smaller/2.5);
}

void Win3D::resize(int _w, int _h) {
  mWinWidth = _w;
  mWinHeight = _h;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, mWinWidth, mWinHeight);
  gluPerspective(mPersp,
                 static_cast<double>(mWinWidth)/static_cast<double>(mWinHeight),
                 0.1, 10.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  mTrackBall.setCenter(Eigen::Vector2d(_w*0.5, _h*0.5));
  mTrackBall.setRadius(std::min(_w, _h)/2.5);

  glutPostRedisplay();
}

void Win3D::keyboard(unsigned char _key, int /*_x*/, int /*_y*/) {
  switch (_key) {
    case ',':  // slow down
      mDisplayTimeout +=2;
      break;
    case '.':  // speed up
      mDisplayTimeout -= 2;
      if (mDisplayTimeout < 1)
        mDisplayTimeout = 1;
      break;
    case 'c':
    case 'C':  // screen capture
      mCapture = !mCapture;
#ifndef _WIN32
      if (mCapture)
        glEnable(GL_MULTISAMPLE);
      else
        glDisable(GL_MULTISAMPLE);
#endif
      // TODO: Disabled use of GL_MULTISAMPLE for Windows. Please see #411 for
      // the detail.
      break;
    case 27:  // ESC
      exit(0);
  }

  glutPostRedisplay();
  // printf("ascii key: %lu\n", key);
}

void Win3D::click(int _button, int _state, int _x, int _y) {
  mMouseDown = !mMouseDown;
  int mask = glutGetModifiers();
  if (mMouseDown) {
    if (_button == GLUT_LEFT_BUTTON) {
      if (mask == GLUT_ACTIVE_SHIFT) {
        mZooming = true;
      } else {
        mRotate = true;
        mTrackBall.startBall(_x, mWinHeight - _y);
      }
    } else if (_button == GLUT_RIGHT_BUTTON || _button == GLUT_MIDDLE_BUTTON) {
      mTranslate = true;
    } else if (_button == 3 && _state == GLUT_DOWN) {  // mouse wheel up
      // each scroll generates a down and an immediate up,
      // so ignore ups
      mZoom += 0.1;
    } else if (_button == 4 && _state == GLUT_DOWN) {  // mouse wheel down?
      // each scroll generates a down and an immediate up,
      // so ignore ups
      mZoom -= 0.1;
    }
    mMouseX = _x;
    mMouseY = _y;
  } else {
    mTranslate = false;
    mRotate = false;
    mZooming = false;
  }
  glutPostRedisplay();
}

void Win3D::drag(int _x, int _y) {
  double deltaX = _x - mMouseX;
  double deltaY = _y - mMouseY;

  mMouseX = _x;
  mMouseY = _y;

  if (mRotate) {
    if (deltaX != 0 || deltaY != 0)
      mTrackBall.updateBall(_x, mWinHeight - _y);
  }
  if (mTranslate) {
    Eigen::Matrix3d rot = mTrackBall.getRotationMatrix();
    mTrans += rot.transpose()*Eigen::Vector3d(deltaX, -deltaY, 0.0);
  }
  if (mZooming) {
    mZoom += deltaY*0.01;
  }
  glutPostRedisplay();
}

void Win3D::render() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(mPersp,
                 static_cast<double>(mWinWidth)/static_cast<double>(mWinHeight),
                 0.1, 10.0);
  gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  initGL();

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
  glTranslatef(mTrans[0]*0.001, mTrans[1]*0.001, mTrans[2]*0.001);

  initLights();
  draw();

  // Draw trackball indicator
  if (mRotate && !mCapture)
    mTrackBall.draw(mWinWidth, mWinHeight);

  glutSwapBuffers();

  if (mCapture)
    screenshot();
}

void Win3D::initGL() {
  glClearColor(mBackground[0], mBackground[1], mBackground[2], mBackground[3]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_POLYGON_SMOOTH);
  glShadeModel(GL_SMOOTH);
  glPolygonMode(GL_FRONT, GL_FILL);
}

void Win3D::initLights() {
  static float ambient[]             = {0.2, 0.2, 0.2, 1.0};
  static float diffuse[]             = {0.6, 0.6, 0.6, 1.0};
  static float front_mat_shininess[] = {60.0};
  static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
  static float front_mat_diffuse[]   = {0.5, 0.28, 0.38, 1.0};
  static float lmodel_ambient[]      = {0.2, 0.2,  0.2,  1.0};
  static float lmodel_twoside[]      = {GL_FALSE};

  GLfloat position[] = {1.0, 0.0, 0.0, 0.0};
  GLfloat position1[] = {-1.0, 0.0, 0.0, 0.0};

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, position);

  glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, position1);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glDisable(GL_CULL_FACE);
  glEnable(GL_NORMALIZE);
}

// Remove once deprecated function, capturing(), is removed
void accFrustum(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top,
                GLdouble nearPlane, GLdouble farPlane,
                GLdouble pixdx, GLdouble pixdy, GLdouble eyedx, GLdouble eyedy,
                GLdouble focus) {
  GLdouble xwsize, ywsize;
  GLdouble dx, dy;
  GLint viewport[4];

  glGetIntegerv(GL_VIEWPORT, viewport);

  xwsize = right - left;
  ywsize = top - bottom;
  dx = -(pixdx * xwsize / static_cast<GLdouble>(viewport[2])
       + eyedx * nearPlane / focus);
  dy = -(pixdy * ywsize / static_cast<GLdouble>(viewport[3])
       + eyedy * nearPlane / focus);

  glFrustum(left + dx, right + dx, bottom + dy, top + dy, nearPlane, farPlane);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(-eyedx, -eyedy, 0.0);
}

// Remove once deprecated function, capturing(), is removed
void accPerspective(GLdouble fovy, GLdouble aspect,
                    GLdouble nearPlane, GLdouble farPlane,
                    GLdouble pixdx, GLdouble pixdy,
                    GLdouble eyedx, GLdouble eyedy, GLdouble focus) {
  GLdouble fov2 = ((fovy*M_PI) / 180.0) / 2.0;
  GLdouble top = nearPlane / (cosf(fov2) / sinf(fov2));
  GLdouble bottom = -top;
  GLdouble right = top * aspect;
  GLdouble left = -right;

  accFrustum(left, right, bottom, top, nearPlane, farPlane,
             pixdx, pixdy, eyedx, eyedy, focus);
}

}  // namespace gui
}  // namespace dart
