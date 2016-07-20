/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/gui/GraphWindow.hpp"

#include <cstdio>
#include <iostream>
#include <string>

#include "dart/gui/GLFuncs.hpp"
#include "dart/gui/LoadGlut.hpp"

namespace dart {
namespace gui {

GraphWindow::GraphWindow()
  : Win2D() {
  mBackground[0] = 1.0;
  mBackground[1] = 1.0;
  mBackground[2] = 1.0;
  mBackground[3] = 1.0;
}

GraphWindow::~GraphWindow() {
}

void GraphWindow::draw() {
  mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
  glPointSize(2);
  glMatrixMode(GL_MODELVIEW);

  int nPoints = mData.size();

  double upperBound = +1.0;
  double lowerBound = -1.0;
  if (nPoints > 0) {
    upperBound = mData.maxCoeff();
    lowerBound = mData.minCoeff();
  }

  for (int i = 0; i < nPoints; i++) {
    glPushMatrix();
    glLoadIdentity();
    glBegin(GL_POINTS);
    glVertex2f(i / (double)nPoints * mWinWidth - mWinWidth / 2.0, mWinHeight * (mData[i] - lowerBound) / (upperBound - lowerBound) - mWinHeight / 2.0);
    glEnd();
    glPopMatrix();
  }    
  glMatrixMode(GL_PROJECTION);

  double xPos = 0.1;
  while (xPos < 1.0) {
    char buff[64];
    int v = xPos * nPoints;
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%d", v);
#else
    std::snprintf(buff, sizeof(buff), "%d", v);
#endif
    std::string frame(buff);
    glColor3f(0.0, 0.0, 0.0);
    gui::drawStringOnScreen(xPos, 0.01f, frame, false);
    xPos += 0.2;
  }

  double yPos = 0.1;
  while (yPos < 1.0) {
    char buff[64];
    double v = yPos * (upperBound - lowerBound) + lowerBound;
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%.2e", v);
#else
    std::snprintf(buff, sizeof(buff), "%.2e", v);
#endif
    std::string frame(buff);
    glColor3f(0.0, 0.0, 0.0);
    gui::drawStringOnScreen(0.01f, yPos, frame, false);
    yPos += 0.2;
  }
}

void GraphWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    default:
      Win2D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void GraphWindow::setData(Eigen::VectorXd _data) {
  mData = _data;
}

}  // namespace gui
}  // namespace dart
