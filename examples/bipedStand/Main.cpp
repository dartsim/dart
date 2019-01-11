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

#include <iostream>
#include <vector>

#include <dart/dart.hpp>
#include <dart/io/io.hpp>

#include "MyWindow.hpp"

int main(int argc, char* argv[]) {
  // create and initialize the world
  dart::simulation::WorldPtr myWorld
      = dart::io::SkelParser::readWorld("dart://sample/skel/fullbody1.skel");
  assert(myWorld != nullptr);

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  dart::dynamics::SkeletonPtr biped = myWorld->getSkeleton("fullbody1");

  biped->getDof("j_pelvis_rot_y")->setPosition( -0.20);
  biped->getDof("j_thigh_left_z")->setPosition(  0.15);
  biped->getDof("j_shin_left")->setPosition(    -0.40);
  biped->getDof("j_heel_left_1")->setPosition(   0.25);
  biped->getDof("j_thigh_right_z")->setPosition( 0.15);
  biped->getDof("j_shin_right")->setPosition(   -0.40);
  biped->getDof("j_heel_right_1")->setPosition(  0.25);
  biped->getDof("j_abdomen_2")->setPosition(     0.00);

  // create controller
  Controller* myController = new Controller(biped, myWorld->getTimeStep());

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
  window.initWindow(640, 480, "Balance");
  glutMainLoop();

  return 0;
}

