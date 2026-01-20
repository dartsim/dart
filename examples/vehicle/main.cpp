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
using namespace dart::math::suffixes;

class VehicleEventHandler : public ::osgGA::GUIEventHandler
{
public:
  VehicleEventHandler() : mBackWheelVelocity(0.0), mSteeringWheelAngle(0.0) {}

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'w':
        case 'W':
          mBackWheelVelocity = -420.0_deg;
          std::cout << "Moving forward" << std::endl;
          return true;
        case 's':
        case 'S':
          mBackWheelVelocity = 0.0_deg;
          std::cout << "Stop" << std::endl;
          return true;
        case 'x':
        case 'X':
          mBackWheelVelocity = +420.0_deg;
          std::cout << "Moving backward" << std::endl;
          return true;
        case 'a':
        case 'A':
          mSteeringWheelAngle += +10_deg;
          if (mSteeringWheelAngle > 30.0_deg)
            mSteeringWheelAngle = 30.0_deg;
          std::cout << "Steering left, angle: " << mSteeringWheelAngle
                    << std::endl;
          return true;
        case 'd':
        case 'D':
          mSteeringWheelAngle += -10_deg;
          if (mSteeringWheelAngle < -30.0_deg)
            mSteeringWheelAngle = -30.0_deg;
          std::cout << "Steering right, angle: " << mSteeringWheelAngle
                    << std::endl;
          return true;
        default:
          return false;
      }
    }
    return false;
  }

  double getBackWheelVelocity() const
  {
    return mBackWheelVelocity;
  }

  double getSteeringWheelAngle() const
  {
    return mSteeringWheelAngle;
  }

protected:
  double mBackWheelVelocity;
  double mSteeringWheelAngle;
};

class VehicleWorld : public RealTimeWorldNode
{
public:
  VehicleWorld(const WorldPtr& world, VehicleEventHandler* handler)
    : RealTimeWorldNode(world), mHandler(handler), mK(0.01), mD(0.005)
  {
  }

  void customPreStep() override
  {
    SkeletonPtr vehicle = mWorld->getSkeleton("car_skeleton");
    DART_ASSERT(vehicle != nullptr);

    std::size_t dof = vehicle->getNumDofs();

    Eigen::VectorXd q = vehicle->getPositions();
    Eigen::VectorXd dq = vehicle->getVelocities();
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dof);

    double steeringAngle = mHandler->getSteeringWheelAngle();
    double wheelVelocity = mHandler->getBackWheelVelocity();

    tau[6] = -mK * (q[6] - steeringAngle) - mD * dq[6];
    tau[8] = -mK * (q[8] - steeringAngle) - mD * dq[8];
    tau[7] = -mD * (dq[7] - wheelVelocity);
    tau[9] = -mD * (dq[9] - wheelVelocity);
    tau[10] = -mD * (dq[10] - wheelVelocity);
    tau[11] = -mD * (dq[11] - wheelVelocity);

    vehicle->setForces(tau);
  }

protected:
  VehicleEventHandler* mHandler;
  double mK;
  double mD;
};

int main(int /*argc*/, char* /*argv*/[])
{
  // Create and initialize the world
  WorldPtr myWorld = dart::io::readWorld("dart://sample/skel/vehicle.skel");
  DART_ASSERT(myWorld != nullptr);
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  // Create event handler
  auto handler = new VehicleEventHandler();

  // Create a custom WorldNode with vehicle behavior
  ::osg::ref_ptr<VehicleWorld> node = new VehicleWorld(myWorld, handler);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  // Print instructions
  viewer.addInstructionText("'w': move forward\n");
  viewer.addInstructionText("'s': stop\n");
  viewer.addInstructionText("'x': move backward\n");
  viewer.addInstructionText("'a': rotate steering wheels to left\n");
  viewer.addInstructionText("'d': rotate steering wheels to right\n");
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
