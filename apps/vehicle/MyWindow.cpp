/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "apps/vehicle/MyWindow.h"

MyWindow::MyWindow()
  : SimWindow() {
  mBackWheelVelocity = 0.0;
  mSteeringWheelAngle = 0.0;
  mK = 0.01;
  mD = 0.005;
}

MyWindow::~MyWindow() {
}

void MyWindow::timeStepping() {
  dart::dynamics::SkeletonPtr vehicle = mWorld->getSkeleton("car_skeleton");
  assert(vehicle != 0);

  size_t dof = vehicle->getNumDofs();

  Eigen::VectorXd q   = vehicle->getPositions();
  Eigen::VectorXd dq  = vehicle->getVelocities();
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(dof);

  tau[ 6] = -mK * (q[6] - mSteeringWheelAngle) - mD * dq[6];
  tau[ 8] = -mK * (q[8] - mSteeringWheelAngle) - mD * dq[8];
  tau[ 7] = -mD * (dq[ 7] - mBackWheelVelocity);
  tau[ 9] = -mD * (dq[ 9] - mBackWheelVelocity);
  tau[10] = -mD * (dq[10] - mBackWheelVelocity);
  tau[11] = -mD * (dq[11] - mBackWheelVelocity);

  vehicle->setForces(tau);

  mWorld->step();
}

void MyWindow::drawWorld() const {
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawWorld();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating) {
        mPlay = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay) {
        mSimulating = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[':  // step backward
      if (!mSimulating) {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating) {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case 'w':  // move forward
      mBackWheelVelocity = DART_RADIAN * -420.0;
      break;
    case 's':  // stop
      mBackWheelVelocity = DART_RADIAN * 0.0;
      break;
    case 'x':  // move backward
      mBackWheelVelocity = DART_RADIAN * +420.0;
      break;
    case 'a':  // rotate steering wheels to left
      mSteeringWheelAngle += DART_RADIAN * +10;
      if (mSteeringWheelAngle > DART_RADIAN * 30.0)
        mSteeringWheelAngle = DART_RADIAN * 30.0;
      break;
    case 'd':  // rotate steering wheels to right
      mSteeringWheelAngle += DART_RADIAN * -10;
      if (mSteeringWheelAngle < DART_RADIAN * -30.0)
        mSteeringWheelAngle = DART_RADIAN * -30.0;
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}
