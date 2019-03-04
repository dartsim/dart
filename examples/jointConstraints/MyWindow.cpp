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

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace gui;
using namespace dart::constraint;

void MyWindow::timeStepping()
{
  mWorld->getSkeleton(1)->getBodyNode("h_spine")->addExtForce(mForce);

  //    mController->setConstrForces(mWorld->getConstraintHandler()->getTotalConstraintForce(1));
  mController->computeTorques(mWorld->getSkeleton(1)->getPositions(),
                              mWorld->getSkeleton(1)->getVelocities());
  mWorld->getSkeleton(1)->setForces(mController->getTorques());

  mWorld->step();

  // for perturbation test
  mImpulseDuration--;
  if (mImpulseDuration <= 0)
  {
    mImpulseDuration = 0;
    mForce.setZero();
  }
}

void MyWindow::drawSkels()
{
  for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
    drawSkeleton(mWorld->getSkeleton(i).get());

  // draw arrow
  if (mImpulseDuration > 0)
  {
    Eigen::Vector3d poa = mWorld->getSkeleton(1)->getBodyNode("h_spine")->getTransform() * Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d start = poa - mForce / 10.0;
    double len = mForce.norm() / 10.0;
    drawArrow3D(start, mForce, len, 0.05, 0.1);
  }
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
  switch(key){
    case ' ': // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if(mSimulating)
        mPlay = false;
      break;
    case 'p': // playBack
      mPlay = !mPlay;
      if (mPlay)
        mSimulating = false;
      break;
    case '[': // step backward
      if (!mSimulating)
      {
        mPlayFrame--;
        if(mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']': // step forwardward
      if (!mSimulating) {
        mPlayFrame++;
        if(mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v': // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case '1':
      mForce[0] = 40;
      mImpulseDuration = 100.0;
      std::cout << "push forward" << std::endl;
      break;
    case '2':
      mForce[0] = -40;
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

    case 'h':
      mHarnessOn = !mHarnessOn;
      if (mHarnessOn)
      {
        BodyNode* bd = mWorld->getSkeleton(1)->getBodyNode("h_pelvis");
        mWeldJoint = std::make_shared<WeldJointConstraint>(bd);
        mWorld->getConstraintSolver()->addConstraint(mWeldJoint);
      }
      else
      {
        mWorld->getConstraintSolver()->removeConstraint(mWeldJoint);
      }
      break;
    default:
      Win3D::keyboard(key,x,y);

  }
  glutPostRedisplay();
}

