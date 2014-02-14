/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Can Erdogan
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
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

/**
 * @file MyWindow.h
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#include "apps/hardcodedDesign/MyWindow.h"

void MyWindow::draw() {
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  skel->draw(mRI);
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  static bool inverse = false;
  static const double dDOF = 0.1;
  switch (_key) {
    case '-': {
      inverse = !inverse;
    } break;
    case '1':
    case '2':
    case '3': {
      size_t dofIdx = _key - 49;
      Eigen::VectorXd pose = skel->get_q();
      pose(dofIdx) = pose(dofIdx) + (inverse ? -dDOF : dDOF);
      skel->setConfig(pose);
      std::cout << "Updated pose DOF " << dofIdx << ": " << pose.transpose()
                << std::endl;
      glutPostRedisplay();
    } break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

