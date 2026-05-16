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

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <cmath>
#include <cstddef>

namespace {

constexpr const char* kWorldUri = "dart://sample/skel/vehicle.skel";
constexpr const char* kVehicleCarName = "visual_vehicle_car";
constexpr const char* kVehicleGroundName = "visual_vehicle_ground";
constexpr const char* kVehicleObstaclePrefix = "visual_vehicle_obstacle_";
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kWheelSpeedCommand = 420.0 * kDegreesToRadians;
constexpr double kSteeringStep = 10.0 * kDegreesToRadians;
constexpr double kMaxSteeringAngle = 30.0 * kDegreesToRadians;

std::string makeStatusLine(const char* label, double value)
{
  std::ostringstream stream;
  stream << label << value;
  return stream.str();
}

class VehicleController
{
public:
  explicit VehicleController(dart::dynamics::SkeletonPtr vehicle)
    : mVehicle(std::move(vehicle))
  {
    if (mVehicle == nullptr) {
      throw std::runtime_error(
          "vehicle controller requires a vehicle skeleton");
    }
  }

  void preStep()
  {
    if (mVehicle->getNumDofs() < 12) {
      return;
    }

    const auto positions = mVehicle->getPositions();
    const auto velocities = mVehicle->getVelocities();
    Eigen::VectorXd forces = Eigen::VectorXd::Zero(mVehicle->getNumDofs());

    forces[6] = -mSteeringStiffness * (positions[6] - mSteeringWheelAngle)
                - mSteeringDamping * velocities[6];
    forces[8] = -mSteeringStiffness * (positions[8] - mSteeringWheelAngle)
                - mSteeringDamping * velocities[8];
    forces[7] = -mWheelDamping * (velocities[7] - mBackWheelVelocity);
    forces[9] = -mWheelDamping * (velocities[9] - mBackWheelVelocity);
    forces[10] = -mWheelDamping * (velocities[10] - mBackWheelVelocity);
    forces[11] = -mWheelDamping * (velocities[11] - mBackWheelVelocity);

    mVehicle->setForces(forces);
  }

  void moveForward()
  {
    mBackWheelVelocity = -kWheelSpeedCommand;
  }

  void stop()
  {
    mBackWheelVelocity = 0.0;
  }

  void moveBackward()
  {
    mBackWheelVelocity = kWheelSpeedCommand;
  }

  void steerLeft()
  {
    mSteeringWheelAngle
        = std::min(mSteeringWheelAngle + kSteeringStep, kMaxSteeringAngle);
  }

  void steerRight()
  {
    mSteeringWheelAngle
        = std::max(mSteeringWheelAngle - kSteeringStep, -kMaxSteeringAngle);
  }

  double& wheelVelocity()
  {
    return mBackWheelVelocity;
  }

  double& steeringAngle()
  {
    return mSteeringWheelAngle;
  }

private:
  dart::dynamics::SkeletonPtr mVehicle;
  double mBackWheelVelocity = 0.0;
  double mSteeringWheelAngle = 0.0;
  double mSteeringStiffness = 0.01;
  double mSteeringDamping = 0.005;
  double mWheelDamping = 0.005;
};

dart::dynamics::SkeletonPtr requireSkeleton(
    const dart::simulation::WorldPtr& world, const std::string& name)
{
  auto skeleton = world->getSkeleton(name);
  if (skeleton == nullptr) {
    throw std::runtime_error("vehicle world is missing skeleton: " + name);
  }
  return skeleton;
}

dart::simulation::WorldPtr createVehicleWorld()
{
  auto world = dart::io::readWorld(kWorldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load vehicle world from " + std::string(kWorldUri));
  }

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto ground = requireSkeleton(world, "ground skeleton");
  ground->setName(kVehicleGroundName);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.45, 0.50, 0.45));
  }

  auto car = requireSkeleton(world, "car_skeleton");
  car->setName(kVehicleCarName);
  if (auto* body = car->getBodyNode("main_body")) {
    body->setColor(Eigen::Vector3d(0.18, 0.36, 0.82));
  }

  const std::array<const char*, 4> wheelNames{
      "wheel_front_left",
      "wheel_front_right",
      "wheel_back_left",
      "wheel_back_right"};
  for (const char* wheelName : wheelNames) {
    if (auto* wheel = car->getBodyNode(wheelName)) {
      wheel->setColor(Eigen::Vector3d(0.06, 0.06, 0.07));
    }
  }

  const std::array<const char*, 2> obstacles{"skeleton_box1", "skeleton_box2"};
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    auto obstacle = requireSkeleton(world, obstacles[i]);
    obstacle->setName(std::string(kVehicleObstaclePrefix) + std::to_string(i));
    if (auto* body = obstacle->getBodyNode("box")) {
      body->setColor(
          i == 0 ? Eigen::Vector3d(0.86, 0.42, 0.20)
                 : Eigen::Vector3d(0.78, 0.64, 0.18));
    }
  }

  return world;
}

dart::gui::Panel createVehiclePanel(
    const std::shared_ptr<VehicleController>& controller)
{
  dart::gui::Panel panel;
  panel.title = "Vehicle";
  panel.buildWithContext = [controller](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Vehicle steering and wheel commands");
    builder.text(
        makeStatusLine("wheel velocity: ", controller->wheelVelocity()));
    builder.text(
        makeStatusLine("steering angle: ", controller->steeringAngle()));
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    if (builder.button("Forward")) {
      controller->moveForward();
    }
    builder.sameLine();
    if (builder.button("Stop")) {
      controller->stop();
    }
    builder.sameLine();
    if (builder.button("Reverse")) {
      controller->moveBackward();
    }
    if (builder.button("Left")) {
      controller->steerLeft();
    }
    builder.sameLine();
    if (builder.button("Right")) {
      controller->steerRight();
    }
    builder.slider(
        "wheel velocity",
        controller->wheelVelocity(),
        -kWheelSpeedCommand,
        kWheelSpeedCommand);
    builder.slider(
        "steering angle",
        controller->steeringAngle(),
        -kMaxSteeringAngle,
        kMaxSteeringAngle);
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  auto world = createVehicleWorld();
  auto controller = std::make_shared<VehicleController>(
      requireSkeleton(world, kVehicleCarName));

  dart::gui::ApplicationOptions options;
  options.world = std::move(world);
  options.preStep = [controller]() {
    controller->preStep();
  };
  options.panels.push_back(createVehiclePanel(controller));

  return dart::gui::runApplication(argc, argv, options);
}
