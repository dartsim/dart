/*
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
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

#include <iostream>
#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include "MyWindow.hpp"

int main(int argc, char* argv[])
{
  // create and initialize the world
  dart::simulation::WorldPtr myWorld
      = dart::utils::SkelParser::readWorld(
        "dart://sample/skel/fullbody1.skel");
  assert(myWorld != nullptr);
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  dart::dynamics::SkeletonPtr skel  = myWorld->getSkeleton(1);

  std::vector<std::size_t> genCoordIds;
  genCoordIds.push_back(1);
  genCoordIds.push_back(6);   // left hip
  genCoordIds.push_back(9);   // left knee
  genCoordIds.push_back(10);  // left ankle
  genCoordIds.push_back(13);  // right hip
  genCoordIds.push_back(16);  // right knee
  genCoordIds.push_back(17);  // right ankle
  genCoordIds.push_back(21);  // lower back
  Eigen::VectorXd initConfig(8);
  initConfig << -0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;
  skel->setPositions(genCoordIds, initConfig);

  dart::dynamics::Joint* joint0 = skel->getJoint(0);
  joint0->setActuatorType(dart::dynamics::Joint::PASSIVE);
  for (std::size_t i = 1; i < skel->getNumBodyNodes(); ++i)
  {
    dart::dynamics::Joint* joint = skel->getJoint(i);
    joint->setActuatorType(dart::dynamics::Joint::VELOCITY);
  }

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'1'--'4': programmed interaction" << std::endl;
  std::cout << "'h': harness on/off" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Hybrid Dynamics");
  glutMainLoop();

  return 0;
}
