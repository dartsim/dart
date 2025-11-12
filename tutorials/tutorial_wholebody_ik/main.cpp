/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

/*
 * Whole-Body Inverse Kinematics Tutorial
 *
 * This tutorial will teach you how to:
 * - Lesson 1: Load a humanoid robot and set up a standing pose
 * - Lesson 2: Create end effectors with proper offsets
 * - Lesson 3: Configure IK with proper error method bounds
 * - Lesson 4: Enable drag-and-drop interaction for IK targets
 *
 * Follow the instructions in each lesson to complete the tutorial.
 */

#include <dart/gui/osg/all.hpp>
#include <dart/utils/all.hpp>
#include <dart/all.hpp>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::osg;
using namespace dart::utils;
using namespace dart::math;

class WholeBodyIKEventHandler : public ::osgGA::GUIEventHandler
{
public:
  WholeBodyIKEventHandler(
      WorldNode* worldNode, const SkeletonPtr& robot, EndEffector* leftHand)
    : mWorldNode(worldNode), mRobot(robot), mLeftHand(leftHand)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      if (ea.getKey() == 'r' || ea.getKey() == 'R') {
        resetToStandingPose();
        return true;
      }
    }
    return false;
  }

  void resetToStandingPose()
  {
    // Lesson 1: Set the robot to a standing configuration
    // TODO: Set the following joint angles to create a standing pose:
    //   - r_leg_hpy: -45 degrees
    //   - r_leg_kny: 90 degrees
    //   - r_leg_aky: -45 degrees
    //   - l_leg_hpy: -45 degrees
    //   - l_leg_kny: 90 degrees
    //   - l_leg_aky: -45 degrees
    // Hint: Convert degrees to radians using constantsd::pi() / 180.0

    std::cout << "Reset to standing pose" << std::endl;
  }

protected:
  WorldNode* mWorldNode;
  SkeletonPtr mRobot;
  EndEffector* mLeftHand;
};

class WholeBodyIKWorldNode : public RealTimeWorldNode
{
public:
  WholeBodyIKWorldNode(const WorldPtr& world, const SkeletonPtr& robot)
    : RealTimeWorldNode(world), mRobot(robot)
  {
  }

  void customPreStep() override
  {
    // The IK will be solved automatically when targets are moved
  }

protected:
  SkeletonPtr mRobot;
};

SkeletonPtr loadAtlasRobot()
{
  // Lesson 1: Load the Atlas robot and configure it for a standing pose
  DartLoader loader;
  SkeletonPtr atlas
      = loader.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");

  if (!atlas) {
    std::cerr << "Failed to load Atlas robot!" << std::endl;
    return nullptr;
  }

  // TODO: Set up initial standing pose by setting the following joint positions:
  //   - r_leg_hpy: -45 degrees
  //   - r_leg_kny: 90 degrees
  //   - r_leg_aky: -45 degrees
  //   - l_leg_hpy: -45 degrees
  //   - l_leg_kny: 90 degrees
  //   - l_leg_aky: -45 degrees
  // Hint: Use atlas->getDof("joint_name")->setPosition(angle_in_radians)

  // TODO: Set knee joint lower limits to prevent backward bending (10 degrees)
  // Hint: Use atlas->getDof("r_leg_kny")->setPositionLowerLimit(...)

  return atlas;
}

EndEffector* createHandEndEffector(BodyNode* handBodyNode, const std::string& name)
{
  // Lesson 2: Create an end effector with proper offset
  // TODO: Create an end effector for the hand with an offset
  // The offset should move the end effector point to the palm center
  // Left hand offset: (0.0, 0.12, 0.0)
  // Right hand offset: (0.0, -0.12, 0.0)
  // Hint: Use Eigen::Isometry3d for the offset transformation

  EndEffector* hand = handBodyNode->createEndEffector(name);
  // TODO: Set the default relative transform with the offset

  return hand;
}

void setupHandIK(EndEffector* hand)
{
  // Lesson 3: Configure inverse kinematics with proper settings
  // Get or create the IK module for this end effector
  auto ik = hand->getIK(true);

  // TODO: Set tight bounds for the error method
  // This is CRITICAL! The error method only produces non-zero error when
  // displacement is outside the bounds. With infinite bounds, error is always zero!
  // Use bounds of [-1e-8, 1e-8] for both linear and angular errors
  // Hint: Use ik->getErrorMethod().setBounds(lower, upper)
  //       where lower and upper are Eigen::Vector6d

  // TODO: Use whole-body IK to allow all dependent DOFs to be used
  // Hint: Use ik->useWholeBody()

  // TODO: Configure the solver parameters:
  //   - Max iterations: 100
  //   - Tolerance: 1e-4
  // Hint: Use ik->getSolver()->setNumMaxIterations(...)
  //       and ik->getSolver()->setTolerance(...)

  // TODO: Activate the IK module
  // Hint: Use ik->setActive(true)

  std::cout << "Set up IK for end effector: " << hand->getName() << std::endl;
}

int main()
{
  // Load the Atlas humanoid robot
  SkeletonPtr atlas = loadAtlasRobot();
  if (!atlas) {
    return 1;
  }

  std::cout << "============================================" << std::endl;
  std::cout << "  Whole-Body IK Tutorial" << std::endl;
  std::cout << "============================================" << std::endl;
  std::cout << "Loaded robot: " << atlas->getName() << std::endl;
  std::cout << "Number of DOFs: " << atlas->getNumDofs() << std::endl;
  std::cout << std::endl;

  // Create end effectors for both hands
  BodyNode* leftHandBody = atlas->getBodyNode("l_hand");
  BodyNode* rightHandBody = atlas->getBodyNode("r_hand");

  if (!leftHandBody || !rightHandBody) {
    std::cerr << "Failed to find hand body nodes!" << std::endl;
    return 1;
  }

  EndEffector* leftHand = createHandEndEffector(leftHandBody, "l_hand");
  EndEffector* rightHand = createHandEndEffector(rightHandBody, "r_hand");

  // Set up IK for both hands
  std::cout << "Setting up whole-body IK..." << std::endl;
  setupHandIK(leftHand);
  setupHandIK(rightHand);
  std::cout << std::endl;

  // Create the world and add the robot
  WorldPtr world = std::make_shared<World>();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(atlas);

  // Create the world node
  ::osg::ref_ptr<WholeBodyIKWorldNode> worldNode
      = new WholeBodyIKWorldNode(world, atlas);

  // Create the viewer
  Viewer viewer;
  viewer.addWorldNode(worldNode);

  // Lesson 4: Enable drag-and-drop for interactive IK
  // TODO: Enable drag-and-drop for the left and right hand end effectors
  // This allows you to interactively move the hands and see the robot adjust
  // Hint: Use viewer.enableDragAndDrop(end_effector)

  // Create event handler
  auto handler = new WholeBodyIKEventHandler(worldNode, atlas, leftHand);
  viewer.addEventHandler(handler);

  // Print instructions
  std::cout << "============================================" << std::endl;
  std::cout << "  Interactive Controls" << std::endl;
  std::cout << "============================================" << std::endl;
  std::cout << "Left-click and drag the colored spheres to move the hands" << std::endl;
  std::cout << "The robot will automatically adjust its posture using whole-body IK" << std::endl;
  std::cout << std::endl;
  std::cout << "Press 'R' to reset to standing pose" << std::endl;
  std::cout << "Press Space to pause/resume simulation" << std::endl;
  std::cout << "============================================" << std::endl;

  // Set up the viewer window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set camera position
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0, 2.0, 2.0),
      ::osg::Vec3(0.0, 0.5, 0.0),
      ::osg::Vec3(0.0, 0.0, 1.0));

  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Run the application
  viewer.run();

  return 0;
}
