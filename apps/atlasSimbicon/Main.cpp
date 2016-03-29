/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#include "dart/dart.hpp"

#include "apps/atlasSimbicon/MyWindow.hpp"
#include "apps/atlasSimbicon/Controller.hpp"

using namespace std;
using namespace Eigen;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

int main(int argc, char* argv[])
{
  // Create empty soft world
  WorldPtr myWorld(new World);

  // Load ground and Atlas robot and add them to the world
  DartLoader urdfLoader;
  SkeletonPtr ground = urdfLoader.parseSkeleton(
        DART_DATA_PATH"sdf/atlas/ground.urdf");
//  SkeletonPtr atlas = SoftSdfParser::readSkeleton(
//        DART_DATA_PATH"sdf/atlas/atlas_v3_no_head.sdf");
  SkeletonPtr atlas = SdfParser::readSkeleton(
        DART_DATA_PATH"sdf/atlas/atlas_v3_no_head_soft_feet.sdf");
  myWorld->addSkeleton(atlas);
  myWorld->addSkeleton(ground);

  // Set initial configuration for Atlas robot
  VectorXd q = atlas->getPositions();
  q[0] = -0.5 * DART_PI;
  atlas->setPositions(q);

  // Set gravity of the world
  myWorld->setGravity(Vector3d(0.0, -9.81, 0.0));

  // Create a window and link it to the world
  MyWindow window(new Controller(atlas, myWorld->getConstraintSolver()));
  window.setWorld(myWorld);

  // Print manual
  cout << "space bar: simulation on/off" << endl;
  cout << "'p': playback/stop" << endl;
  cout << "'[' and ']': play one frame backward and forward" << endl;
  cout << "'v': visualization on/off" << endl;
  cout << endl;
  cout << "'h': harness pelvis on/off" << endl;
  cout << "'j': harness left foot on/off" << endl;
  cout << "'k': harness right foot on/off" << endl;
  cout << "'r': reset robot" << endl;
  cout << "'n': transite to the next state manually" << endl;
  cout << endl;
  cout << "'1': standing controller" << endl;
  cout << "'2': walking controller" << endl;

  // Run glut loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Atlas Robot");
  glutMainLoop();

  return 0;
}
