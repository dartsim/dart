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

//==============================================================================
MyWindow::MyWindow()
  : SimWindow(),
    mHarnessOn(false)
{
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{
  dart::dynamics::SkeletonPtr skel  = mWorld->getSkeleton(1);

  std::size_t index0 = skel->getJoint("j_scapula_left")->getIndexInSkeleton(0);
  std::size_t index1 = skel->getJoint("j_scapula_right")->getIndexInSkeleton(0);
  std::size_t index2 = skel->getJoint("j_forearm_left")->getIndexInSkeleton(0);
  std::size_t index3 = skel->getJoint("j_forearm_right")->getIndexInSkeleton(0);

  std::size_t index6 = skel->getJoint("j_shin_left")->getIndexInSkeleton(0);
  std::size_t index7 = skel->getJoint("j_shin_right")->getIndexInSkeleton(0);

  skel->setCommand(index0,  1.0 * std::sin(mWorld->getTime() * 4.0));
  skel->setCommand(index1, -1.0 * std::sin(mWorld->getTime() * 4.0));
  skel->setCommand(index2,  0.8 * std::sin(mWorld->getTime() * 4.0));
  skel->setCommand(index3,  0.8 * std::sin(mWorld->getTime() * 4.0));

  skel->setCommand(index6,  0.1 * std::sin(mWorld->getTime() * 2.0));
  skel->setCommand(index7,  0.1 * std::sin(mWorld->getTime() * 2.0));

  mWorld->step();
}

//==============================================================================
void MyWindow::drawWorld() const
{
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawWorld();
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
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case 'h':
      mHarnessOn = !mHarnessOn;
      if (mHarnessOn)
      {
        dart::dynamics::Joint* joint
            = mWorld->getSkeleton(1)->getBodyNode("h_pelvis")->getParentJoint();
        joint->setActuatorType(dart::dynamics::Joint::LOCKED);
        std::cout << "The pelvis is locked." << std::endl;
      }
      else
      {
        dart::dynamics::Joint* joint
            = mWorld->getSkeleton(1)->getBodyNode("h_pelvis")->getParentJoint();
        joint->setActuatorType(dart::dynamics::Joint::PASSIVE);
        std::cout << "The pelvis is unlocked." << std::endl;
      }
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}
