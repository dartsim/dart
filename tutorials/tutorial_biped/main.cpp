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

#include "dart/common/macros.hpp"

#include <dart/gui/all.hpp>

#include <dart/utils/All.hpp>

#include <dart/all.hpp>
#include <dart/io/read.hpp>

const double default_speed_increment = 0.5;

[[maybe_unused]] const int default_ik_iterations = 4500;

const double default_force = 50.0; // N
const int default_countdown = 100; // Number of timesteps for applying force

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;

class Controller
{
public:
  /// Constructor
  Controller(const SkeletonPtr& biped)
    : mBiped(biped), mPreOffset(0.0), mSpeed(0.0)
  {
    int nDofs = mBiped->getNumDofs();

    mForces = Eigen::VectorXd::Zero(nDofs);

    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);

    for (std::size_t i = 0; i < 6; ++i) {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }

    for (std::size_t i = 6; i < biped->getNumDofs(); ++i) {
      mKp(i, i) = 1000;
      mKd(i, i) = 50;
    }

    setTargetPositions(mBiped->getPositions());
  }

  /// Reset the desired dof position to the current position
  void setTargetPositions(const Eigen::VectorXd& pose)
  {
    mTargetPositions = pose;
  }

  /// Clear commanding forces
  void clearForces()
  {
    mForces.setZero();
  }

  /// Add commanding forces from PD controllers
  void addPDForces()
  {
    // Lesson 2
  }

  /// Add commanind forces from Stable-PD controllers
  void addSPDForces()
  {
    // Lesson 3
  }

  /// add commanding forces from ankle strategy
  void addAnkleStrategyForces()
  {
    // Lesson 4
  }

  // Send velocity commands on wheel actuators
  void setWheelCommands()
  {
    // Lesson 6
  }

  void changeWheelSpeed(double increment)
  {
    mSpeed += increment;
    std::cout << "wheel speed = " << mSpeed << std::endl;
  }

protected:
  /// The biped Skeleton that we will be controlling
  SkeletonPtr mBiped;

  /// Joint forces for the biped (output of the Controller)
  Eigen::VectorXd mForces;

  /// Control gains for the proportional error terms in the PD controller
  Eigen::MatrixXd mKp;

  /// Control gains for the derivative error terms in the PD controller
  Eigen::MatrixXd mKd;

  /// Target positions for the PD controllers
  Eigen::VectorXd mTargetPositions;

  /// For ankle strategy: Error in the previous timestep
  double mPreOffset;

  /// For velocity actuator: Current speed of the skateboard
  double mSpeed;
};

class BipedEventHandler : public ::osgGA::GUIEventHandler
{
public:
  BipedEventHandler(const WorldPtr& world, Controller* controller)
    : mWorld(world),
      mController(controller),
      mForceCountDown(0),
      mPositiveSign(true)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case ',':
          mForceCountDown = default_countdown;
          mPositiveSign = false;
          return true;
        case '.':
          mForceCountDown = default_countdown;
          mPositiveSign = true;
          return true;
        case 'a':
        case 'A':
          mController->changeWheelSpeed(default_speed_increment);
          return true;
        case 's':
        case 'S':
          mController->changeWheelSpeed(-default_speed_increment);
          return true;
        default:
          return false;
      }
    }
    return false;
  }

  void update()
  {
    mController->clearForces();

    // Lesson 3
    mController->addSPDForces();

    // Lesson 4
    mController->addAnkleStrategyForces();

    // Lesson 6
    mController->setWheelCommands();

    // Apply body forces based on user input, and color the body shape red
    if (mForceCountDown > 0) {
      BodyNode* bn = mWorld->getSkeleton("biped")->getBodyNode("h_abdomen");
      bn->setColor(dart::Color::Red());

      if (mPositiveSign) {
        bn->addExtForce(
            default_force * Eigen::Vector3d::UnitX(),
            bn->getCOM(),
            false,
            false);
      } else {
        bn->addExtForce(
            -default_force * Eigen::Vector3d::UnitX(),
            bn->getCOM(),
            false,
            false);
      }

      --mForceCountDown;
    }
  }

protected:
  WorldPtr mWorld;
  Controller* mController;

  /// Number of iterations before clearing a force entry
  int mForceCountDown;

  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;
};

class CustomWorldNode : public RealTimeWorldNode
{
public:
  CustomWorldNode(const WorldPtr& world, BipedEventHandler* handler)
    : RealTimeWorldNode(world), mHandler(handler)
  {
  }

  void customPreStep() override
  {
    mHandler->update();
  }

protected:
  BipedEventHandler* mHandler;
};

// Load a biped model and enable joint limits and self-collision
SkeletonPtr loadBiped()
{
  // Lesson 1

  // Create the world with a skeleton
  WorldPtr world = dart::io::readWorld("dart://sample/skel/biped.skel");
  DART_ASSERT(world != nullptr);

  SkeletonPtr biped = world->getSkeleton("biped");

  return biped;
}

// Load a skateboard model and connect it to the biped model via an Euler joint
void modifyBipedWithSkateboard(SkeletonPtr /*biped*/)
{
  // Lesson 5
}

// Set the actuator type for four wheel joints to "VELOCITY"
void setVelocityActuators(SkeletonPtr /*biped*/)
{
  // Lesson 6
}

// Solve for a balanced pose using IK
Eigen::VectorXd solveIK(SkeletonPtr biped)
{
  // Lesson 7
  Eigen::VectorXd newPose = biped->getPositions();
  return newPose;
}

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body
      = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

int main()
{
  SkeletonPtr floor = createFloor();

  // Lesson 1
  SkeletonPtr biped = loadBiped();

  // Lesson 5
  modifyBipedWithSkateboard(biped);

  // Lesson 6
  setVelocityActuators(biped);

  // Lesson 7
  Eigen::VectorXd balancedPose = solveIK(biped);
  biped->setPositions(balancedPose);

  WorldConfig config;
  if (auto* factory = dart::collision::CollisionDetector::getFactory();
      factory && factory->canCreate("bullet")) {
    config.collisionDetector = CollisionDetectorType::Bullet;
  }
  WorldPtr world = std::make_shared<World>(config);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  world->addSkeleton(floor);
  world->addSkeleton(biped);

  // Create controller and event handler
  auto controller = std::make_unique<Controller>(biped);
  auto handler = new BipedEventHandler(world, controller.get());

  // Create a WorldNode and wrap it around the world
  ::osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world, handler);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  // Print instructions
  viewer.addInstructionText("'.': forward push\n");
  viewer.addInstructionText("',': backward push\n");
  viewer.addInstructionText("'s': increase skateboard forward speed\n");
  viewer.addInstructionText("'a': increase skateboard backward speed\n");
  viewer.addInstructionText("space bar: simulation on/off\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.0f, 3.0f, 3.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
