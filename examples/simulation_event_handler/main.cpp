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

#include "SimulationEventHandler.hpp"

#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/dart.hpp>

#include <iostream>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui::osg;
using namespace dart::utils;
using namespace dart::math;

/// \brief Create a simple box rigid body
SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& size = Eigen::Vector3d(0.5, 0.5, 0.5),
    const Eigen::Vector3d& color = Eigen::Vector3d(0.7, 0.7, 0.9),
    double mass = 1.0)
{
  SkeletonPtr skeleton = Skeleton::create(name);

  // Create the joint and body pair
  std::pair<FreeJoint*, BodyNode*> pair
      = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr);
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  // Set joint position
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  joint->setTransformFromParentBodyNode(tf);

  // Set body properties
  body->setName(name + "_body");

  // Create shape
  ShapePtr boxShape = std::make_shared<BoxShape>(size);

  // Create shape node with visual, collision, and dynamics aspects
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(boxShape);

  shapeNode->getVisualAspect()->setColor(color);

  // Set mass properties
  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(boxShape->computeInertia(mass));
  body->setInertia(inertia);

  return skeleton;
}

/// \brief Create a simple sphere rigid body
SkeletonPtr createSphere(
    const std::string& name,
    const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
    double radius = 0.25,
    const Eigen::Vector3d& color = Eigen::Vector3d(0.9, 0.7, 0.7),
    double mass = 1.0)
{
  SkeletonPtr skeleton = Skeleton::create(name);

  // Create the joint and body pair
  std::pair<FreeJoint*, BodyNode*> pair
      = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr);
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  // Set joint position
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  joint->setTransformFromParentBodyNode(tf);

  // Set body properties
  body->setName(name + "_body");

  // Create shape
  ShapePtr sphereShape = std::make_shared<SphereShape>(radius);

  // Create shape node with visual, collision, and dynamics aspects
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(sphereShape);

  shapeNode->getVisualAspect()->setColor(color);

  // Set mass properties
  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(sphereShape->computeInertia(mass));
  body->setInertia(inertia);

  return skeleton;
}

/// \brief Create a ground plane
SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  // Create the ground body with a WeldJoint (fixed to world)
  std::pair<WeldJoint*, BodyNode*> pair
      = ground->createJointAndBodyNodePair<WeldJoint>(nullptr);
  BodyNode* groundBody = pair.second;

  // Set ground position slightly below origin
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  pair.first->setTransformFromParentBodyNode(tf);

  groundBody->setName("ground_body");

  // Create large box shape for ground
  double groundSize = 10.0;
  double groundHeight = 0.1;
  ShapePtr groundShape = std::make_shared<BoxShape>(
      Eigen::Vector3d(groundSize, groundSize, groundHeight));

  // Create shape node
  auto shapeNode = groundBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(groundShape);

  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.3, 0.3, 0.3));

  return ground;
}

/// \brief Custom world node that calls our event handler's update function
class CustomWorldNode : public RealTimeWorldNode
{
public:
  CustomWorldNode(const WorldPtr& world, SimulationEventHandler* handler)
    : RealTimeWorldNode(world), mHandler(handler)
  {
  }

  void customPreStep() override
  {
    if (mHandler) {
      mHandler->update();
    }
  }

protected:
  SimulationEventHandler* mHandler;
};

int main(int argc, char* argv[])
{
  // Create the physics world
  WorldPtr world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);

  // Create some rigid bodies for testing
  auto ground = createGround();
  world->addSkeleton(ground);

  auto box1 = createBox("box1", Eigen::Vector3d(-1.0, 0.0, 2.0),
                        Eigen::Vector3d(0.4, 0.4, 0.4),
                        Eigen::Vector3d(0.8, 0.2, 0.2));
  world->addSkeleton(box1);

  auto box2 = createBox("box2", Eigen::Vector3d(0.0, 0.0, 3.0),
                        Eigen::Vector3d(0.6, 0.3, 0.3),
                        Eigen::Vector3d(0.2, 0.8, 0.2));
  world->addSkeleton(box2);

  auto sphere1 = createSphere("sphere1", Eigen::Vector3d(1.0, 0.0, 2.5), 0.3,
                              Eigen::Vector3d(0.2, 0.2, 0.8));
  world->addSkeleton(sphere1);

  auto box3 = createBox("box3", Eigen::Vector3d(0.5, 1.0, 2.2),
                        Eigen::Vector3d(0.3, 0.5, 0.4),
                        Eigen::Vector3d(0.9, 0.9, 0.2));
  world->addSkeleton(box3);

  // Create OSG viewer
  Viewer viewer;

  // Create the simulation event handler
  SimulationEventHandler* eventHandler = new SimulationEventHandler(world, &viewer);

  // Create custom world node that will call our event handler's update
  ::osg::ref_ptr<CustomWorldNode> worldNode = new CustomWorldNode(world, eventHandler);

  // Set up viewer
  viewer.addWorldNode(worldNode);
  viewer.addEventHandler(eventHandler);

  // Configure the viewer
  viewer.setUpViewInWindow(0, 0, 1280, 960);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.0, 5.0, 3.0),
      ::osg::Vec3(0.0, 0.0, 1.0),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Add instructional text
  viewer.addInstructionText("=== Simulation Event Handler Demo ===\n");
  viewer.addInstructionText("This example demonstrates a comprehensive event handler\n");
  viewer.addInstructionText("that replaces MyWindow.cpp functionality.\n\n");
  viewer.addInstructionText("Press 'H' or '?' for detailed controls\n");
  viewer.addInstructionText("Press Space to start/pause simulation\n");
  viewer.addInstructionText("Use Tab to select different rigid bodies\n");
  viewer.addInstructionText("Use arrow keys to apply forces\n");
  viewer.addInstructionText("Use Q/W/E/A/Z/C for torques\n");
  viewer.addInstructionText("Press 'V' to toggle force visualization\n");

  std::cout << "\n=== Simulation Event Handler Demo ===" << std::endl;
  std::cout << "World created with " << world->getNumSkeletons() << " skeletons:" << std::endl;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    std::cout << "  " << (i + 1) << ". " << world->getSkeleton(i)->getName() << std::endl;
  }
  std::cout << "\nPress 'H' for help or start interacting!" << std::endl;

  // Start the application
  return viewer.run();
}
