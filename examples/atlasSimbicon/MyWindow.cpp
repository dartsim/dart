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

//==============================================================================
MyWindow::MyWindow(Controller* _controller)
  : SimWindow(),
    mController(_controller)
{
  mForce = Eigen::Vector3d::Zero();
  mImpulseDuration = 0.0;
}

//==============================================================================
MyWindow::~MyWindow()
{
  delete mController;
}

//==============================================================================
void MyWindow::timeStepping()
{
  // External force
  mWorld->getSkeleton("drc_skeleton")->getBodyNode("pelvis")->addExtForce(
        mForce);

  // Internal force
  mController->update(mWorld->getTime());

  // simulate one step
  mWorld->step();

  // for perturbation test
  mImpulseDuration--;
  if (mImpulseDuration <= 0)
  {
    mImpulseDuration = 0;
    mForce.setZero();
  }
}

//==============================================================================
void MyWindow::drawSkels()
{
//  glEnable(GL_LIGHTING);
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
    drawSkeleton(mWorld->getSkeleton(i).get());

  // draw arrow
  if (mImpulseDuration > 0)
  {
    Eigen::Vector3d poa
        =  mWorld->getSkeleton("drc_skeleton")->getBodyNode(
             "pelvis")->getTransform()
           * Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d start = poa - mForce / 500.0;
    double len = mForce.norm() / 500.0;
    dart::gui::drawArrow3D(start, mForce, len, 0.05, 0.1);
  }
}

//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  switch (_key)
  {
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
    if (!mSimulating)
    {
      mPlayFrame--;
      if (mPlayFrame < 0)
        mPlayFrame = 0;
      glutPostRedisplay();
    }
    break;
  case ']':  // step forwardward
    if (!mSimulating)
    {
      mPlayFrame++;
      if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
        mPlayFrame = 0;
      glutPostRedisplay();
    }
    break;
  case 'i':  // print debug information
    mController->printDebugInfo();
    break;
  case 'v':  // show or hide markers
    mShowMarkers = !mShowMarkers;
    break;
  case 'a':  // upper right force
    mForce[0] = 500;
    mImpulseDuration = 100;
    std::cout << "push forward" << std::endl;
    break;
  case 's':  // upper right force
    mForce[0] = -500;
    mImpulseDuration = 100;
    std::cout << "push backward" << std::endl;
    break;
  case 'd':  // upper right force
    mForce[2] = 500;
    mImpulseDuration = 100;
    std::cout << "push right" << std::endl;
    break;
  case 'f':  // upper right force
    mForce[2] = -500;
    mImpulseDuration = 100;
    std::cout << "push left" << std::endl;
    break;
  default:
    Win3D::keyboard(_key, _x, _y);
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y, mWorld->getTime());

  glutPostRedisplay();
}

