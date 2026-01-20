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

#include "Controller.hpp"
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
using namespace dart::constraint;

class JointConstraintsEventHandler : public ::osgGA::GUIEventHandler
{
public:
  JointConstraintsEventHandler(const WorldPtr& world, Controller* controller)
    : mWorld(world),
      mController(controller),
      mHarnessOn(false),
      mForce(Eigen::Vector3d::Zero()),
      mImpulseDuration(0)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case '1':
          mForce[0] = 40;
          mImpulseDuration = 100.0;
          std::cout << "push forward" << std::endl;
          return true;
        case '2':
          mForce[0] = -40;
          mImpulseDuration = 100.0;
          std::cout << "push backward" << std::endl;
          return true;
        case '3':
          mForce[2] = 50;
          mImpulseDuration = 100.0;
          std::cout << "push right" << std::endl;
          return true;
        case '4':
          mForce[2] = -50;
          mImpulseDuration = 100.0;
          std::cout << "push left" << std::endl;
          return true;
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
      BodyNode* bd = mWorld->getSkeleton(1)->getBodyNode("h_pelvis");
      mWeldJoint = std::make_shared<WeldJointConstraint>(bd);
      mWorld->getConstraintSolver()->addConstraint(mWeldJoint);
      std::cout << "Harness on" << std::endl;
    } else {
      mWorld->getConstraintSolver()->removeConstraint(mWeldJoint);
      std::cout << "Harness off" << std::endl;
    }
  }

  Eigen::Vector3d getForce() const
  {
    return mForce;
  }

  int getImpulseDuration() const
  {
    return mImpulseDuration;
  }

  void decrementImpulseDuration()
  {
    mImpulseDuration--;
    if (mImpulseDuration <= 0) {
      mImpulseDuration = 0;
      mForce.setZero();
    }
  }

protected:
  WorldPtr mWorld;
  Controller* mController;
  bool mHarnessOn;
  std::shared_ptr<WeldJointConstraint> mWeldJoint;
  Eigen::Vector3d mForce;
  int mImpulseDuration;
};

class JointConstraintsWorld : public RealTimeWorldNode
{
public:
  JointConstraintsWorld(
      const WorldPtr& world,
      Controller* controller,
      JointConstraintsEventHandler* handler)
    : RealTimeWorldNode(world), mController(controller), mHandler(handler)
  {
  }

  void customPreStep() override
  {
    mWorld->getSkeleton(1)->getBodyNode("h_spine")->addExtForce(
        mHandler->getForce());

    mController->computeTorques(
        mWorld->getSkeleton(1)->getPositions(),
        mWorld->getSkeleton(1)->getVelocities());
    mWorld->getSkeleton(1)->setForces(mController->getTorques());

    mHandler->decrementImpulseDuration();
  }

  void customPostStep() override
  {
    // Note: Arrow visualization for force could be added through OSG mechanisms
    // For now, we just apply the force in customPreStep
  }

protected:
  Controller* mController;
  JointConstraintsEventHandler* mHandler;
};

int main(int /*argc*/, char* /*argv*/[])
{
  // Create and initialize the world
  WorldPtr myWorld = dart::io::readWorld("dart://sample/skel/fullbody1.skel");
  DART_ASSERT(myWorld != nullptr);

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  std::vector<std::size_t> genCoordIds;
  genCoordIds.push_back(1);  // global orientation y
  genCoordIds.push_back(4);  // global position y
  genCoordIds.push_back(6);  // left hip
  genCoordIds.push_back(9);  // left knee
  genCoordIds.push_back(10); // left ankle
  genCoordIds.push_back(13); // right hip
  genCoordIds.push_back(16); // right knee
  genCoordIds.push_back(17); // right ankle
  genCoordIds.push_back(21); // lower back
  Eigen::VectorXd initConfig(9);
  initConfig << -0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1;
  myWorld->getSkeleton(1)->setPositions(genCoordIds, initConfig);

  // Create controller
  Controller* myController = new Controller(
      myWorld->getSkeleton(1),
      myWorld->getConstraintSolver(),
      myWorld->getTimeStep());

  // Create event handler
  auto handler = new JointConstraintsEventHandler(myWorld, myController);

  // Create a custom WorldNode with joint constraints behavior
  ::osg::ref_ptr<JointConstraintsWorld> node
      = new JointConstraintsWorld(myWorld, myController, handler);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  // Print instructions
  viewer.addInstructionText("'1'-'4': programmed perturbations\n");
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

  delete myController;

  return 0;
}
