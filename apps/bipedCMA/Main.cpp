/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include <iostream>
#include <vector>

#include "dart/utils/Paths.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "MyWindow.h"

int main(int argc, char* argv[]) {
  srand( (unsigned int) time (NULL) );
  // create and initialize the world
  dart::simulation::World* myWorld
      = dart::utils::SkelParser::readWorld(
          DART_DATA_PATH"skel/fullbody1.skel");
  assert(myWorld != NULL);

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  int NDOFS = myWorld->getSkeleton(1)->getDof();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(NDOFS);
  q[1] = -0.1;
  q[6] = 0.2;
  q[14] = -0.5;
  q[17] = 0.3;
  q[9] = 0.2;
  q[15] = -0.5;
  q[19] = 0.3;
  myWorld->getSkeleton(1)->setPositions(q);
  myWorld->getSkeleton(1)->computeForwardKinematics(true, true, false);

  // create controller
  Controller* myController = new Controller(myWorld->getSkeleton(1),
                                            myWorld->getConstraintSolver(),
                                            myWorld->getTimeStep());

  Eigen::VectorXd initParam = Eigen::VectorXd::Random(3);
  std::cout << "initial parameters = " << initParam.transpose() << std::endl;
  myController->setParams( initParam );

  Eigen::Vector3d COM = myWorld->getSkeleton(1)->getWorldCOM();
  myController->setTargetCOM( COM - Eigen::Vector3d(0.0, 0.05, 0.0 ) );
  std::cout << "Target COM = " << myController->getTargetCOM().transpose() << std::endl;

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);
  window.setController(myController);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'1'--'4': programmed interaction" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Optimal Balance Controller");
  glutMainLoop();

  return 0;
}

