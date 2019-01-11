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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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
  // load a skeleton file
  // create and initialize the world
  dart::simulation::WorldPtr myWorld
      = dart::io::SkelParser::readWorld(
        "dart://sample/skel/softBodies.skel");
  assert(myWorld != nullptr);

  for(std::size_t i=0; i<myWorld->getNumSkeletons(); ++i)
  {
    dart::dynamics::SkeletonPtr skel = myWorld->getSkeleton(i);
    for(std::size_t j=0; j<skel->getNumBodyNodes(); ++j)
    {
      dart::dynamics::BodyNode* bn = skel->getBodyNode(j);
      Eigen::Vector3d color = dart::Color::Random();
      auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
      for(auto shapeNode : shapeNodes)
        shapeNode->getVisualAspect(true)->setColor(color);
    }
  }

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'1'--'6': programmed interaction" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Soft Bodies");
  glutMainLoop();

  return 0;
}
