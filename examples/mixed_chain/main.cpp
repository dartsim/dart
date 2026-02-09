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

#include <iostream>

#define FORCE_ON_RIGIDBODY 500.0

class MixedChainEventHandler : public osgGA::GUIEventHandler
{
public:
  MixedChainEventHandler(dart::simulation::WorldPtr world)
    : mWorld(world), mImpulseDuration(0)
  {
    mForceOnRigidBody.setZero();
  }

  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'q':
          mForceOnRigidBody[0] = -FORCE_ON_RIGIDBODY;
          mImpulseDuration = 100;
          return true;
        case 'w':
          mForceOnRigidBody[0] = FORCE_ON_RIGIDBODY;
          mImpulseDuration = 100;
          return true;
        case 'e':
          mForceOnRigidBody[1] = -FORCE_ON_RIGIDBODY;
          mImpulseDuration = 100;
          return true;
        case 'r':
          mForceOnRigidBody[1] = FORCE_ON_RIGIDBODY;
          mImpulseDuration = 100;
          return true;
        case 't':
          mForceOnRigidBody[2] = -FORCE_ON_RIGIDBODY;
          mImpulseDuration = 100;
          return true;
        case 'y':
          mForceOnRigidBody[2] = FORCE_ON_RIGIDBODY;
          mImpulseDuration = 100;
          return true;
        default:
          return false;
      }
    }
    return false;
  }

  void applyForces()
  {
    if (mImpulseDuration > 0) {
      dart::dynamics::SkeletonPtr skeleton = mWorld->getSkeleton(1);
      if (skeleton && skeleton->getNumSoftBodyNodes() > 3) {
        dart::dynamics::SoftBodyNode* softBodyNode
            = skeleton->getSoftBodyNode(3);
        if (softBodyNode) {
          softBodyNode->addExtForce(mForceOnRigidBody);
        }
      }
      mImpulseDuration--;
      if (mImpulseDuration <= 0) {
        mImpulseDuration = 0;
        mForceOnRigidBody.setZero();
      }
    }
  }

private:
  dart::simulation::WorldPtr mWorld;
  Eigen::Vector3d mForceOnRigidBody;
  int mImpulseDuration;
};

class MixedChainWorldNode : public dart::gui::RealTimeWorldNode
{
public:
  MixedChainWorldNode(
      dart::simulation::WorldPtr world,
      ::osg::ref_ptr<MixedChainEventHandler> eventHandler)
    : dart::gui::RealTimeWorldNode(world), mEventHandler(eventHandler)
  {
  }

protected:
  void customPreStep() override
  {
    // Apply forces from the event handler
    mEventHandler->applyForces();
  }

private:
  ::osg::ref_ptr<MixedChainEventHandler> mEventHandler;
};

int main()
{
  // Load the skeleton file
  dart::simulation::WorldPtr myWorld = dart::io::readWorld(
      "dart://sample/skel/test/test_articulated_bodies_10bodies.skel");
  DART_ASSERT(myWorld != nullptr);

  // Set initial pose for the skeleton
  int dof = myWorld->getSkeleton(1)->getNumDofs();
  Eigen::VectorXd initPose = Eigen::VectorXd::Zero(dof);
  for (int i = 0; i < 3; i++) {
    initPose[i] = dart::math::Random::uniform(-0.5, 0.5);
  }
  myWorld->getSkeleton(1)->setPositions(initPose);

  // Create event handler for keyboard input (using osg::ref_ptr for OSG
  // compatibility)
  ::osg::ref_ptr<MixedChainEventHandler> eventHandler
      = new MixedChainEventHandler(myWorld);

  // Create a WorldNode and wrap it around the world
  ::osg::ref_ptr<MixedChainWorldNode> node
      = new MixedChainWorldNode(myWorld, eventHandler);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::Viewer viewer;
  viewer.addWorldNode(node);

  // Add the event handler to the viewer
  viewer.addEventHandler(eventHandler);

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.0f, 1.0f, 2.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Print controls
  std::cout << "Mixed Chain Example Controls:" << std::endl;
  std::cout << "'q'/'w': Apply force in -X/+X direction" << std::endl;
  std::cout << "'e'/'r': Apply force in -Y/+Y direction" << std::endl;
  std::cout << "'t'/'y': Apply force in -Z/+Z direction" << std::endl;
  std::cout << "Space: Toggle simulation" << std::endl;

  // Begin running the application
  viewer.run();

  return 0;
}
