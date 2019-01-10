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
#include <dart/gui/osg/osg.hpp>

#include "Helpers.hpp"
#include "WamWorld.hpp"
#include "InputHandler.hpp"

int main()
{
  // Create the world.
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Create an Wam arm
  SkeletonPtr wam = createWam();
  setStartupConfiguration(wam);
  setupEndEffectors(wam);
  world->addSkeleton(wam);

  // Create the ground
  world->addSkeleton(createGround());

  ::osg::ref_ptr<WamWorld> node = new WamWorld(world, wam);

  dart::gui::osg::Viewer viewer;
  viewer.allowSimulation(false);
  viewer.addWorldNode(node);

  enableDragAndDrops(viewer, wam);

  viewer.addEventHandler(new InputHandler(&viewer, node, wam, world));

  std::cout << viewer.getInstructions() << std::endl;
  std::cout << "Alt + Click:   Try to translate a body without changing its orientation\n"
            << "Ctrl + Click:  Try to rotate a body without changing its translation\n"
            << "Shift + Click: Move a body using only its parent joint\n"
            << "1:             Toggle the interactive target of an EndEffector\n"
            << "P:             Print the current joint values\n"
            << "T:             Reset the robot to its relaxed posture\n"
            << "The blue/red ball is the robot's center of mass.\n"
            << "\n"
            << "Note that this is purely kinematic. Physical simulation is not allowed in this app.\n"
            << std::endl;

  // Set up the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set up the default viewing position
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3( 5.34,  3.00, 1.91),
      ::osg::Vec3( 0.00,  0.00, 0.50),
      ::osg::Vec3(-0.00, -0.00, 0.98));

  // Reset the camera manipulator so that it starts in the new viewing position
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();
}
