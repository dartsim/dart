/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"
#include "z_up.hpp"

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
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart::examples::demos {

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

  // vehicle.skel is authored Y-up; reorient to the canonical Z-up convention.
  reorientWorldToZUp(world);

  return world;
}

dart::gui::OrbitCamera makeVehicleCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.5404195002705842;
  camera.pitch = 0.4758822496604165;
  camera.distance = 6.557438524302;
  return camera;
}

dart::gui::KeyboardAction makeVehicleAction(
    const std::shared_ptr<VehicleController>& controller,
    char key,
    std::string label,
    void (VehicleController::*command)())
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.callback = [controller, command](dart::gui::KeyboardActionContext&) {
    ((*controller).*command)();
  };
  return action;
}

std::vector<dart::gui::KeyboardAction> createVehicleKeyboardActions(
    const std::shared_ptr<VehicleController>& controller)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(5);
  actions.push_back(makeVehicleAction(
      controller,
      'w',
      "Move vehicle forward",
      &VehicleController::moveForward));
  actions.push_back(makeVehicleAction(
      controller, 's', "Stop vehicle", &VehicleController::stop));
  actions.push_back(makeVehicleAction(
      controller,
      'x',
      "Move vehicle backward",
      &VehicleController::moveBackward));
  actions.push_back(makeVehicleAction(
      controller, 'a', "Steer vehicle left", &VehicleController::steerLeft));
  actions.push_back(makeVehicleAction(
      controller, 'd', "Steer vehicle right", &VehicleController::steerRight));
  return actions;
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
    builder.text("'w': move forward");
    builder.text("'s': stop");
    builder.text("'x': move backward");
    builder.text("'a': rotate steering wheels to left");
    builder.text("'d': rotate steering wheels to right");
    builder.text("space bar: simulation on/off");
    builder.separator();
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

dart::gui::ApplicationOptions makeVehicleScene()
{
  auto world = createVehicleWorld();
  auto controller = std::make_shared<VehicleController>(
      requireSkeleton(world, kVehicleCarName));

  dart::gui::ApplicationOptions options;
  options.world = std::move(world);
  options.camera = makeVehicleCamera();
  options.preStep = [controller]() {
    controller->preStep();
  };
  options.keyboardActions = createVehicleKeyboardActions(controller);
  options.panels.push_back(createVehiclePanel(controller));
  return options;
}

} // namespace dart::examples::demos
