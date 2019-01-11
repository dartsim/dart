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

#include <dart/dart.hpp>
#include <dart/io/io.hpp>

#include "MyWindow.hpp"

int main(int argc, char* argv[])
{
  using namespace dart;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  // create and initialize the world
  WorldPtr myWorld = SkelParser::readWorld("dart://sample/skel/vehicle.skel");
  assert(myWorld != nullptr);
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'1'--'4': programmed interaction" << std::endl;
  std::cout << "'w': move forward" << std::endl;
  std::cout << "'s': stop" << std::endl;
  std::cout << "'x': move backward" << std::endl;
  std::cout << "'a': rotate steering wheels to left" << std::endl;
  std::cout << "'d': rotate steering wheels to right" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Vehicle");
  glutMainLoop();

  return 0;
}
