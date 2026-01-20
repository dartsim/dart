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

#include <dart/collision/bullet/All.hpp>

#include <dart/all.hpp>
#include <dart/io/read.hpp>

#include <iostream>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;

class AddDeleteSkelsEventHandler : public ::osgGA::GUIEventHandler
{
public:
  AddDeleteSkelsEventHandler(const WorldPtr& world) : mWorld(world) {}

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'q':
        case 'Q':
          spawnCube();
          return true;
        case 'w':
        case 'W':
          if (mWorld->getNumSkeletons() > 1)
            mWorld->removeSkeleton(
                mWorld->getSkeleton(mWorld->getNumSkeletons() - 1));
          return true;
        default:
          return false;
      }
    }
    return false;
  }

  void spawnCube(
      const Eigen::Vector3d& position = Eigen::Vector3d(
          Random::uniform(-1.0, 1.0),
          Random::uniform(0.5, 1.0),
          Random::uniform(-1.0, 1.0)),
      const Eigen::Vector3d& size = Eigen::Vector3d(
          Random::uniform(0.1, 0.5),
          Random::uniform(0.1, 0.5),
          Random::uniform(0.1, 0.5)),
      double mass = 0.1)
  {
    SkeletonPtr newCubeSkeleton = Skeleton::create();

    BodyNode::Properties body;
    body.mName = "cube_link";
    body.mInertia.setMass(mass);
    body.mInertia.setMoment(BoxShape::computeInertia(size, mass));
    ShapePtr newBoxShape(new BoxShape(size));

    FreeJoint::Properties joint;
    joint.mName = "cube_joint";
    joint.mT_ParentBodyToJoint = Eigen::Translation3d(position);

    auto pair = newCubeSkeleton->createJointAndBodyNodePair<FreeJoint>(
        nullptr, joint, body);
    auto shapeNode = pair.second->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(newBoxShape);
    shapeNode->getVisualAspect()->setColor(
        Random::uniform<Eigen::Vector3d>(0.0, 1.0));

    mWorld->addSkeleton(newCubeSkeleton);
  }

protected:
  WorldPtr mWorld;
};

int main()
{
  // Create and initialize the world
  WorldPtr myWorld = dart::io::readWorld("dart://sample/skel/ground.skel");
  DART_ASSERT(myWorld != nullptr);
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  // Set collision detector type
  if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet")) {
    myWorld->setCollisionDetector(CollisionDetectorType::Bullet);
  }

  // Create event handler
  auto handler = new AddDeleteSkelsEventHandler(myWorld);

  // Create a WorldNode and wrap it around the world
  ::osg::ref_ptr<RealTimeWorldNode> node = new RealTimeWorldNode(myWorld);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  // Print instructions
  viewer.addInstructionText("'q': spawn a random cube\n");
  viewer.addInstructionText("'w': delete a spawned cube\n");
  viewer.addInstructionText("space bar: simulation on/off\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.0f, 3.0f, 3.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
