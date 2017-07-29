/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "MinitaurWorldNode.hpp"
#include "MinitaurEventHandler.hpp"
#include "MinitaurWidget.hpp"

int main()
{
  dart::simulation::WorldPtr world(new dart::simulation::World());

  // Load ground and Atlas robot and add them to the world
  dart::utils::DartLoader loader;
  auto minitaur = loader.parseSkeleton("dart://sample/urdf/minitaur/minitaur.urdf");
  auto ground = loader.parseSkeleton("dart://sample/urdf/minitaur/ground.urdf");

  world->addSkeleton(minitaur);
  //world->addSkeleton(ground);

  // Set initial configuration for Minitaur robot
  using namespace dart::math::suffixes;
  minitaur->setPosition(0, 180_deg);
  minitaur->setPosition(5, 0.5);

  auto frontLL = minitaur->getBodyNode("lower_leg_front_leftL_link");
  auto frontLR = minitaur->getBodyNode("lower_leg_front_leftR_link");
  auto frontRL = minitaur->getBodyNode("lower_leg_front_rightL_link");
  auto frontRR = minitaur->getBodyNode("lower_leg_front_rightR_link");
  auto backLL = minitaur->getBodyNode("lower_leg_back_leftL_link");
  auto backLR = minitaur->getBodyNode("lower_leg_back_leftR_link");
  auto backRL = minitaur->getBodyNode("lower_leg_back_rightL_link");
  auto backRR = minitaur->getBodyNode("lower_leg_back_rightR_link");
  Eigen::Vector3d offset(0.0, 0.0, 0.2);
  auto constFrontLeft = std::make_shared<dart::constraint::BallJointConstraint>(
      frontLL, frontLL->getTransform() * offset);
  auto constFrontLeft2 = std::make_shared<dart::constraint::BallJointConstraint>(
      frontLR, frontLR->getTransform() * offset);
  auto constFrontRight = std::make_shared<dart::constraint::BallJointConstraint>(
      frontRL, frontRL->getTransform() * offset);
  auto constFrontRight2 = std::make_shared<dart::constraint::BallJointConstraint>(
      frontRR, frontRR->getTransform() * offset);
  auto constBackLeft = std::make_shared<dart::constraint::BallJointConstraint>(
      backLL, backLL->getTransform() * offset);
  auto constBackRight = std::make_shared<dart::constraint::BallJointConstraint>(
      backRL, backRL->getTransform() * offset);
  world->getConstraintSolver()->addConstraint(constFrontLeft);
  world->getConstraintSolver()->addConstraint(constFrontLeft2);
  world->getConstraintSolver()->addConstraint(constFrontRight);
  world->getConstraintSolver()->addConstraint(constFrontRight2);
  //world->getConstraintSolver()->addConstraint(constBackLeft);
  //world->getConstraintSolver()->addConstraint(constBackRight);

  // Set gravity of the world
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  // Wrap a WorldNode around it
  osg::ref_ptr<MinitaurWorldNode> node
      = new MinitaurWorldNode(world, minitaur);
  node->setNumStepsPerCycle(20);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Add control widget for atlas
  viewer.getImGuiHandler()->addWidget(
        std::make_shared<MinitaurWidget>(&viewer, node.get()));

  // Pass in the custom event handler
  viewer.addEventHandler(new MinitaurEventHandler(node));

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set the window name
  viewer.realize();
  osgViewer::Viewer::Windows windows;
  viewer.getWindows(windows);
  windows.front()->setWindowName("Ghost Robotics - Minitaur");

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 2.0, 2.0, 2.0),
        ::osg::Vec3d( 0.0, 0.0, 0.0),
        ::osg::Vec3d( 0.0, 0.0, 1.0));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}
