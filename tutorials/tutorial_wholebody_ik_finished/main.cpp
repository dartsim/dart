/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/gui/all.hpp>
#include <dart/utils/All.hpp>
#include <dart/All.hpp>

#include <dart/io/read.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui;
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

// snippet:cpp-load-atlas-start
SkeletonPtr loadAtlasRobot()
{
  SkeletonPtr atlas
      = dart::io::readSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");

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
// snippet:cpp-load-atlas-end

// snippet:cpp-create-hand-start
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
// snippet:cpp-create-hand-end

// snippet:cpp-setup-ik-start
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
// snippet:cpp-setup-ik-end

// snippet:cpp-smooth-motion-start
void configureSmoothMotion(dart::dynamics::InverseKinematics* ik)
{
  if (!ik)
    return;

  auto* gradient = ik->getGradientMethod();
  if (!gradient)
    return;

  const auto& dofs = ik->getDofs();
  Eigen::VectorXd weights
      = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(dofs.size()));
  const std::size_t rootCount = std::min<std::size_t>(6, dofs.size());
  if (rootCount > 0) {
    weights.head(static_cast<Eigen::Index>(rootCount)).setConstant(0.25);
  }
  gradient->setComponentWeights(weights);
  gradient->setComponentWiseClamp(0.15);

  if (auto* dls
      = dynamic_cast<dart::dynamics::InverseKinematics::JacobianDLS*>(
          gradient)) {
    dls->setDampingCoefficient(2.0);
  }
}
// snippet:cpp-smooth-motion-end

struct HandTarget
{
  std::shared_ptr<SimpleFrame> mFrame;
  Eigen::Vector3d mRestTranslation;
};

HandTarget createHandTarget(EndEffector* hand)
{
  auto target = std::make_shared<SimpleFrame>(
      Frame::World(), hand->getName() + "_target", hand->getWorldTransform());
  hand->getIK()->setTarget(target.get());
  return HandTarget{target, target->getTransform().translation()};
}

void updateTargetPose(const HandTarget& target, const Eigen::Vector3d& offset)
{
  Eigen::Isometry3d tf = target.mFrame->getTransform();
  tf.translation() = target.mRestTranslation + offset;
  target.mFrame->setTransform(tf);
}

// snippet:cpp-headless-start
void runHeadlessDemo(
    const WorldPtr& world,
    const SkeletonPtr& atlas,
    EndEffector* leftHand,
    EndEffector* rightHand,
    std::size_t steps,
    double radius)
{
  auto leftTarget = createHandTarget(leftHand);
  auto rightTarget = createHandTarget(rightHand);
  world->addSimpleFrame(leftTarget.mFrame);
  world->addSimpleFrame(rightTarget.mFrame);

  auto atlasIK = atlas->getIK(true);
  atlasIK->setActive(true);
  configureSmoothMotion(atlasIK.get());
  auto solver = atlasIK->getSolver();
  auto* gradientSolver
      = dynamic_cast<dart::optimizer::GradientDescentSolver*>(solver.get());

  std::cout << "Running headless trajectory tracking for " << steps
            << " samples..." << std::endl;
  for (std::size_t i = 0; i < steps; ++i) {
    const double phase
        = static_cast<double>(i) / static_cast<double>(steps)
          * 2.0 * constantsd::pi();
    Eigen::Vector3d leftOffset(
        radius * std::cos(phase), 0.0, radius * std::sin(phase));
    Eigen::Vector3d rightOffset(
        0.5 * radius * std::cos(phase),
        0.5 * radius * std::sin(phase),
        0.0);

    updateTargetPose(leftTarget, leftOffset);
    updateTargetPose(rightTarget, rightOffset);

    const auto start = std::chrono::steady_clock::now();
    const bool solved = atlasIK->solveAndApply(true);
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start);
    world->step();

    const auto& lTrans = leftHand->getWorldTransform().translation();
    const auto& rTrans = rightHand->getWorldTransform().translation();
    const Eigen::Vector3d leftError
        = leftTarget.mFrame->getTransform().translation() - lTrans;
    const Eigen::Vector3d rightError
        = rightTarget.mFrame->getTransform().translation() - rTrans;
    const Eigen::VectorXd q = atlas->getPositions();
    const Eigen::Index previewSize = std::min<Eigen::Index>(6, q.size());
    const Eigen::VectorXd qPreview = q.head(previewSize);
    const std::size_t iterations
        = gradientSolver ? gradientSolver->getLastNumIterations() : 0;

    std::cout << "[headless] step " << i << " solved=" << std::boolalpha
              << solved << " | left_err=" << leftError.norm()
              << "m right_err=" << rightError.norm() << "m | iter="
              << iterations << " time=" << duration.count() / 1000.0
              << "ms | q[0:6]=" << qPreview.transpose() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cout << "Headless run complete." << std::endl;
}
// snippet:cpp-headless-end

// snippet:cpp-gui-start
void runGuiDemo(
    const WorldPtr& world,
    const SkeletonPtr& atlas,
    EndEffector* leftHand,
    EndEffector* rightHand)
{
  auto worldNode = new WholeBodyIKWorldNode(world, atlas);

  Viewer viewer;
  viewer.addWorldNode(worldNode);
  viewer.enableDragAndDrop(leftHand);
  viewer.enableDragAndDrop(rightHand);

  auto handler = new WholeBodyIKEventHandler(worldNode, atlas, leftHand);
  viewer.addEventHandler(handler);

  std::cout << "============================================" << std::endl;
  std::cout << "  Interactive Controls" << std::endl;
  std::cout << "============================================" << std::endl;
  std::cout << "Use the viewer gizmos to drag the hands." << std::endl;
  std::cout << "Press 'R' to reset the standing pose." << std::endl;
  std::cout << "============================================" << std::endl;

  viewer.setUpViewInWindow(0, 0, 1280, 960);
  viewer.setCameraHomePosition(
      {3.0, 2.0, 2.0}, {0.0, 0.5, 0.0}, {0.0, 0.0, 1.0});
  viewer.run();
}
// snippet:cpp-gui-end

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
  configureSmoothMotion(leftHand->getIK());
  configureSmoothMotion(rightHand->getIK());

  // Create the world and add the robot
  WorldPtr world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(atlas);

  bool headless = false;
  std::size_t headlessSteps = 120;
  double trajectoryRadius = 0.08;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--headless") {
      headless = true;
    } else if (arg.rfind("--steps=", 0) == 0) {
      headlessSteps = std::stoul(arg.substr(8));
    } else if (arg.rfind("--radius=", 0) == 0) {
      trajectoryRadius = std::stod(arg.substr(9));
    }
  }

  if (headless) {
    runHeadlessDemo(
        world, atlas, leftHand, rightHand, headlessSteps, trajectoryRadius);
  } else {
    runGuiDemo(world, atlas, leftHand, rightHand);
  }

  return 0;
}
