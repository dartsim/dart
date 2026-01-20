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

#include <dart/All.hpp>
#include <dart/io/read.hpp>

#include <iostream>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui;
using namespace dart::utils;

class HybridDynamicsEventHandler : public ::osgGA::GUIEventHandler
{
public:
  HybridDynamicsEventHandler(const WorldPtr& world)
    : mWorld(world), mHarnessOn(false)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'h':
        case 'H':
          toggleHarness();
          return true;
        default:
          return false;
      }
    }
    return false;
  }

  void toggleHarness()
  {
    mHarnessOn = !mHarnessOn;
    if (mHarnessOn) {
      Joint* joint
          = mWorld->getSkeleton(1)->getBodyNode("h_pelvis")->getParentJoint();
      joint->setActuatorType(Joint::LOCKED);
      std::cout << "The pelvis is locked." << std::endl;
    } else {
      Joint* joint
          = mWorld->getSkeleton(1)->getBodyNode("h_pelvis")->getParentJoint();
      joint->setActuatorType(Joint::PASSIVE);
      std::cout << "The pelvis is unlocked." << std::endl;
    }
  }

protected:
  WorldPtr mWorld;
  bool mHarnessOn;
};

class HybridDynamicsWorld : public RealTimeWorldNode
{
public:
  HybridDynamicsWorld(const WorldPtr& world) : RealTimeWorldNode(world) {}

  void customPreStep() override
  {
    SkeletonPtr skel = mWorld->getSkeleton(1);

    std::size_t index0
        = skel->getJoint("j_scapula_left")->getIndexInSkeleton(0);
    std::size_t index1
        = skel->getJoint("j_scapula_right")->getIndexInSkeleton(0);
    std::size_t index2
        = skel->getJoint("j_forearm_left")->getIndexInSkeleton(0);
    std::size_t index3
        = skel->getJoint("j_forearm_right")->getIndexInSkeleton(0);

    std::size_t index6 = skel->getJoint("j_shin_left")->getIndexInSkeleton(0);
    std::size_t index7 = skel->getJoint("j_shin_right")->getIndexInSkeleton(0);

    skel->setCommand(index0, 1.0 * std::sin(mWorld->getTime() * 4.0));
    skel->setCommand(index1, -1.0 * std::sin(mWorld->getTime() * 4.0));
    skel->setCommand(index2, 0.8 * std::sin(mWorld->getTime() * 4.0));
    skel->setCommand(index3, 0.8 * std::sin(mWorld->getTime() * 4.0));

    skel->setCommand(index6, 0.1 * std::sin(mWorld->getTime() * 2.0));
    skel->setCommand(index7, 0.1 * std::sin(mWorld->getTime() * 2.0));
  }
};

int main(int /*argc*/, char* /*argv*/[])
{
  // Create and initialize the world
  WorldPtr myWorld = dart::io::readWorld("dart://sample/skel/fullbody1.skel");
  DART_ASSERT(myWorld != nullptr);
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  SkeletonPtr skel = myWorld->getSkeleton(1);

  std::vector<std::size_t> genCoordIds;
  genCoordIds.push_back(1);
  genCoordIds.push_back(6);  // left hip
  genCoordIds.push_back(9);  // left knee
  genCoordIds.push_back(10); // left ankle
  genCoordIds.push_back(13); // right hip
  genCoordIds.push_back(16); // right knee
  genCoordIds.push_back(17); // right ankle
  genCoordIds.push_back(21); // lower back
  Eigen::VectorXd initConfig(8);
  initConfig << -0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;
  skel->setPositions(genCoordIds, initConfig);

  Joint* joint0 = skel->getJoint(0);
  joint0->setActuatorType(Joint::PASSIVE);
  for (std::size_t i = 1; i < skel->getNumBodyNodes(); ++i) {
    Joint* joint = skel->getJoint(i);
    joint->setActuatorType(Joint::VELOCITY);
  }

  // Create event handler
  auto handler = new HybridDynamicsEventHandler(myWorld);

  // Create a custom WorldNode with hybrid dynamics behavior
  ::osg::ref_ptr<HybridDynamicsWorld> node = new HybridDynamicsWorld(myWorld);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  // Print instructions
  viewer.addInstructionText("'h': toggle harness on/off\n");
  viewer.addInstructionText("space bar: simulation on/off\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.0f, 3.0f, 3.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 1.0f, 0.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
