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
 * This tutorial demonstrates how to use whole-body inverse kinematics (IK)
 * to control a humanoid robot's posture. You will learn how to:
 *
 * 1. Load a complex humanoid robot (Atlas)
 * 2. Create end effectors for hands and feet
 * 3. Set up IK targets and constraints
 * 4. Use whole-body IK to achieve desired poses
 * 5. Understand the importance of proper error method bounds
 *
 * The tutorial is interactive - you can drag and drop end effectors
 * to see the robot adjust its posture in real-time.
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
    // Reset to standing configuration
    mRobot->getDof("r_leg_hpy")
        ->setPosition(-45.0 * constantsd::pi() / 180.0);
    mRobot->getDof("r_leg_kny")->setPosition(90.0 * constantsd::pi() / 180.0);
    mRobot->getDof("r_leg_aky")
        ->setPosition(-45.0 * constantsd::pi() / 180.0);
    mRobot->getDof("l_leg_hpy")
        ->setPosition(-45.0 * constantsd::pi() / 180.0);
    mRobot->getDof("l_leg_kny")->setPosition(90.0 * constantsd::pi() / 180.0);
    mRobot->getDof("l_leg_aky")
        ->setPosition(-45.0 * constantsd::pi() / 180.0);

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
    // because we're using drag-and-drop interaction
  }

protected:
  SkeletonPtr mRobot;
};

SkeletonPtr loadAtlasRobot()
{
  DartLoader loader;
  SkeletonPtr atlas
      = loader.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");

  if (!atlas) {
    std::cerr << "Failed to load Atlas robot!" << std::endl;
    return nullptr;
  }

  // Set up initial standing pose
  atlas->getDof("r_leg_hpy")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("r_leg_kny")->setPosition(90.0 * constantsd::pi() / 180.0);
  atlas->getDof("r_leg_aky")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_hpy")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_kny")->setPosition(90.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_aky")->setPosition(-45.0 * constantsd::pi() / 180.0);

  // Prevent knees from bending backwards
  atlas->getDof("r_leg_kny")->setPositionLowerLimit(10.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_kny")->setPositionLowerLimit(10.0 * constantsd::pi() / 180.0);

  return atlas;
}

EndEffector* createHandEndEffector(BodyNode* handBodyNode, const std::string& name)
{
  // Create an end effector for the hand
  // The offset moves the end effector point to the palm center
  Eigen::Isometry3d handOffset = Eigen::Isometry3d::Identity();
  handOffset.translation() = Eigen::Vector3d(0.0, (name == "l_hand" ? 0.12 : -0.12), 0.0);

  EndEffector* hand = handBodyNode->createEndEffector(name);
  hand->setDefaultRelativeTransform(handOffset, true);

  return hand;
}

void setupHandIK(EndEffector* hand)
{
  // Get or create the IK module for this end effector
  auto ik = hand->getIK(true);

  // CRITICAL: Set tight bounds for the error method
  // This ensures that any displacement from the target produces a non-zero error,
  // which allows the optimizer to find the gradient direction.
  // With infinite bounds (the default), the error would be zero, causing IK to fail.
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));

  // Use whole-body IK: allows all dependent DOFs to be used
  ik->useWholeBody();

  // Configure the solver
  ik->getSolver()->setNumMaxIterations(100);
  ik->getSolver()->setTolerance(1e-4);

  // Activate the IK module
  ik->setActive(true);

  std::cout << "Set up IK for end effector: " << hand->getName() << std::endl;
  std::cout << "  - Using whole-body IK (all dependent DOFs)" << std::endl;
  std::cout << "  - Error method bounds: tight (1e-8)" << std::endl;
  std::cout << "  - Max iterations: 100" << std::endl;
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

  // Enable drag-and-drop for the end effectors
  // This allows you to interactively move the hands
  viewer.enableDragAndDrop(leftHand);
  viewer.enableDragAndDrop(rightHand);

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
