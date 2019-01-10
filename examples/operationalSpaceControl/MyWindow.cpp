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

#include <iostream>

//==============================================================================
MyWindow::MyWindow(Controller* _controller)
  : SimWindow(),
    mController(_controller),
    mCircleTask(false)
{
  assert(_controller != nullptr);

  // Set the initial target positon to the initial position of the end effector
  mTargetPosition = mController->getEndEffector()->getTransform().translation();
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{
  if (mCircleTask)
  {
    static double time = 0.0;
    const double dt = 0.0005;
    const double radius = 0.6;
    Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.1, 0.0);

    mTargetPosition = center;
    mTargetPosition[0] = radius * std::sin(time);
    mTargetPosition[1] = 0.25 * radius * std::sin(time);
    mTargetPosition[2] = radius * std::cos(time);

    time += dt;
  }

  // Update the controller and apply control force to the robot
  mController->update(mTargetPosition);

  // Step forward the simulation
  mWorld->step();
}

//==============================================================================
void MyWindow::drawWorld() const
{
  // Draw the target position
  if (mRI)
  {
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(mTargetPosition);
    mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    mRI->popMatrix();
  }

  // Draw world
  SimWindow::drawWorld();
}

//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  double incremental = 0.01;

  switch (_key)
  {
    case 'c':  // print debug information
      if (mCircleTask)
      {
        std::cout << "Circle task [off]." << std::endl;
        mCircleTask = false;
      }
      else
      {
        std::cout << "Circle task [on]." << std::endl;
        mCircleTask = true;
      }
      break;
    case 'q':
      mTargetPosition[0] -= incremental;
      break;
    case 'w':
      mTargetPosition[0] += incremental;
      break;
    case 'a':
      mTargetPosition[1] -= incremental;
      break;
    case 's':
      mTargetPosition[1] += incremental;
      break;
    case 'z':
      mTargetPosition[2] -= incremental;
      break;
    case 'x':
      mTargetPosition[2] += incremental;
      break;
    default:
      // Default keyboard control
      SimWindow::keyboard(_key, _x, _y);
      break;
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y);

  glutPostRedisplay();
}

