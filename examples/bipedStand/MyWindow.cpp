/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include "MyWindow.hpp"

#include <dart/gui/GLFuncs.hpp>

MyWindow::MyWindow(): SimWindow() {
  mForce = Eigen::Vector3d::Zero();
  mController = nullptr;
  mImpulseDuration = 0;
}

MyWindow::~MyWindow() {
}

void MyWindow::timeStepping() {
  mWorld->getSkeleton(1)->getBodyNode("h_spine")->addExtForce(mForce);

  mController->computeTorques();
  mController->getSkel()->setForces(mController->getTorques());

  mWorld->step();

  // for perturbation test
  mImpulseDuration--;
  if (mImpulseDuration <= 0) {
    mImpulseDuration = 0;
    mForce.setZero();
  }
}

void MyWindow::drawWorld() const {

  SimWindow::drawWorld();

  // draw arrow
  if (mImpulseDuration > 0) {
    Eigen::Vector3d poa =
        mWorld->getSkeleton(1)->getBodyNode("h_spine")->getTransform()
        * Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d start = poa - mForce / 10.0;
    double len = mForce.norm() / 10.0;
    dart::gui::drawArrow3D(start, mForce, len, 0.05, 0.1);
  }
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating)
        mPlay = false;
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay)
        mSimulating = false;
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
    case '1':
      mForce[0] = 50;
      mImpulseDuration = 100.0;
      std::cout << "push forward" << std::endl;
      break;
    case '2':
      mForce[0] = -50;
      mImpulseDuration = 100.0;
      std::cout << "push backward" << std::endl;
      break;
    case '3':
      mForce[2] = 50;
      mImpulseDuration = 100.0;
      std::cout << "push right" << std::endl;
      break;
    case '4':
      mForce[2] = -50;
      mImpulseDuration = 100.0;
      std::cout << "push left" << std::endl;
      break;
    case 'g':
      plotCOMX();
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void MyWindow::setController(Controller* _controller) {
  mController = _controller;
}

void MyWindow::plotCOMX() {
  int nFrame = mWorld->getRecording()->getNumFrames();
  Eigen::VectorXd data(nFrame);
  for (int i = 0; i < nFrame; i++) {
    Eigen::VectorXd pose = mWorld->getRecording()->getConfig(i, 1);
    mWorld->getSkeleton(1)->setPositions(pose);
    data[i] = mWorld->getSkeleton(1)->getCOM()[0];
  }
  if (nFrame != 0)
  {
    Eigen::VectorXd pose = mWorld->getRecording()->getConfig(mPlayFrame, 1);
    mWorld->getSkeleton(1)->setPositions(pose);
  }

  plot(data);
}
