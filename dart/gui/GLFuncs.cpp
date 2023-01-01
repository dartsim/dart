/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/gui/GLFuncs.hpp"

#include "dart/common/Console.hpp"
#include "dart/gui/LoadOpengl.hpp"
#include "dart/gui/glut/GLUTFuncs.hpp"
#include "dart/math/Constants.hpp"

#include <Eigen/Eigen>

#include <iostream>
#include <string>

#include <cstdio>
// TODO(JS): remove once glut become an optional dependency

namespace dart {
namespace gui {

void drawStringOnScreen(float x, float y, const std::string& s, bool bigFont)
{
  glut::drawStringOnScreen(x, y, s, bigFont);
}

// draw a 3D arrow starting from pt along dir, the arrowhead is on the other end
void drawArrow3D(
    const Eigen::Vector3d& _pt,
    const Eigen::Vector3d& _dir,
    const double _length,
    const double _thickness,
    const double _arrowThickness)
{
  const double pi = math::constantsd::pi();

  Eigen::Vector3d normDir = _dir;
  normDir.normalize();

  double arrowLength;
  if (_arrowThickness == -1)
    arrowLength = 4 * _thickness;
  else
    arrowLength = 2 * _arrowThickness;

  // draw the arrow body as a cylinder
  GLUquadricObj* c;
  c = gluNewQuadric();
  gluQuadricDrawStyle(c, GLU_FILL);
  gluQuadricNormals(c, GLU_SMOOTH);

  glPushMatrix();
  glTranslatef(_pt[0], _pt[1], _pt[2]);
  glRotated(acos(normDir[2]) * 180 / pi, -normDir[1], normDir[0], 0);
  gluCylinder(c, _thickness, _thickness, _length - arrowLength, 16, 16);

  // draw the arrowhed as a cone
  glPushMatrix();
  glTranslatef(0, 0, _length - arrowLength);
  gluCylinder(c, arrowLength * 0.5, 0.0, arrowLength, 10, 10);
  glPopMatrix();

  glPopMatrix();

  gluDeleteQuadric(c);
}

// draw a 2D arrow starting from pt along vec, the arrow head is on the other
// end
void drawArrow2D(
    const Eigen::Vector2d& _pt, const Eigen::Vector2d& _vec, double _thickness)
{
  const double pi = math::constantsd::pi();

  // draw the arrow body as a thick line
  glLineWidth(_thickness);
  glBegin(GL_LINES);
  glVertex2f(_pt[0], _pt[1]);
  glVertex2f(_pt[0] + _vec[0], _pt[1] + _vec[1]);
  glEnd();

  // draw arrowhead as a triangle
  double theta = atan2(_vec[1], _vec[0]);
  glPushMatrix();
  glTranslatef(_pt[0] + _vec[0], _pt[1] + _vec[1], 0.0);
  glRotatef(theta * 180.0 / pi, 0.0, 0.0, 1.0);
  glTranslatef(_thickness, 0.0, 0.0);
  glBegin(GL_TRIANGLES);
  glVertex2f(0.0, _thickness);
  glVertex2f(2 * _thickness, 0.0);
  glVertex2f(0.0, -_thickness);
  glEnd();
  glPopMatrix();
}

void drawProgressBar(int _currFrame, int _totalFrame)
{
  GLint oldMode;
  glGetIntegerv(GL_MATRIX_MODE, &oldMode);
  glMatrixMode(GL_PROJECTION);

  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0.0, 1.0, 0.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glPolygonMode(GL_FRONT, GL_LINE);
  glColor4d(0.0, 0.0, 0.0, 0.5);
  glBegin(GL_QUADS);
  glVertex2f(0.15f, 0.02f);
  glVertex2f(0.85f, 0.02f);
  glVertex2f(0.85f, 0.08f);
  glVertex2f(0.15f, 0.08f);
  glEnd();

  float portion = static_cast<float>(_currFrame) / _totalFrame;
  float end = 0.15f + portion * 0.7f;
  glPolygonMode(GL_FRONT, GL_FILL);
  glColor4d(0.3, 0.3, 0.3, 0.5);
  glBegin(GL_QUADS);
  glVertex2f(0.15f, 0.02f);
  glVertex2f(end, 0.02f);
  glVertex2f(end, 0.08f);
  glVertex2f(0.15f, 0.08f);
  glEnd();

  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(oldMode);
}

} // namespace gui
} // namespace dart
