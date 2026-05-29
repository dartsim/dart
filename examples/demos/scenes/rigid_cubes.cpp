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

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

constexpr const char* kRigidCubesUri = "dart://sample/skel/cubes.skel";
constexpr double kRigidCubesForceMagnitude = 500.0;

struct RigidCubesState
{
  Eigen::Vector3d pendingForce = Eigen::Vector3d::Zero();
  bool visualizationMarkersVisible = true;
};

dart::simulation::WorldPtr createRigidCubesWorld()
{
  auto world = dart::io::readWorld(kRigidCubesUri);
  if (world == nullptr) {
    throw std::runtime_error("Failed to load dart://sample/skel/cubes.skel");
  }

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  return world;
}

dart::dynamics::BodyNode* getForceTargetBody(dart::simulation::World* world)
{
  if (world == nullptr) {
    return nullptr;
  }

  if (world->getNumSkeletons() <= 1) {
    return nullptr;
  }

  const auto cube = world->getSkeleton(1);
  if (cube == nullptr || cube->getNumBodyNodes() == 0) {
    return nullptr;
  }

  return cube->getBodyNode(0);
}

void applyPendingForce(
    const dart::simulation::WorldPtr& world,
    const std::shared_ptr<RigidCubesState>& state)
{
  if (state == nullptr) {
    return;
  }

  auto* body = getForceTargetBody(world.get());
  if (body != nullptr) {
    body->addExtForce(state->pendingForce);
  }

  state->pendingForce /= 2.0;
}

void setDirectionalForce(
    const std::shared_ptr<RigidCubesState>& state,
    const Eigen::Vector3d& force,
    const char* label)
{
  if (state == nullptr) {
    return;
  }

  state->pendingForce = force;
  std::cout << "Applied " << label << " force\n";
}

void toggleVisualizationMarkers(const std::shared_ptr<RigidCubesState>& state)
{
  if (state == nullptr) {
    return;
  }

  state->visualizationMarkersVisible = !state->visualizationMarkersVisible;
  std::cout << "Visualization markers "
            << (state->visualizationMarkersVisible ? "on" : "off") << "\n";
}

dart::gui::Panel createControlsPanel(
    const std::shared_ptr<RigidCubesState>& state)
{
  dart::gui::Panel controls;
  controls.title = "Rigid Cube Forces";
  controls.buildWithContext = [state](
                                  dart::gui::PanelBuilder& panel,
                                  dart::gui::PanelContext& context) mutable {
    panel.text("Rigid Cubes Example");
    panel.text("1/2: -X/+X force, 3/4: -Z/+Z force");
    panel.text("P: playback/stop, V: visualization on/off");
    if (state != nullptr) {
      panel.text(
          std::string("markers: ")
          + (state->visualizationMarkersVisible ? "on" : "off"));
    }
    panel.separator();
    panel.text("Apply one-frame force to the center cube");
    panel.separator();
    if (context.lifecycle != nullptr) {
      if (panel.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      panel.sameLine();
      if (panel.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    if (panel.button("-X")) {
      setDirectionalForce(
          state, Eigen::Vector3d(-kRigidCubesForceMagnitude, 0.0, 0.0), "-X");
    }
    panel.sameLine();
    if (panel.button("+X")) {
      setDirectionalForce(
          state, Eigen::Vector3d(kRigidCubesForceMagnitude, 0.0, 0.0), "+X");
    }
    panel.sameLine();
    if (panel.button("-Z")) {
      setDirectionalForce(
          state, Eigen::Vector3d(0.0, 0.0, -kRigidCubesForceMagnitude), "-Z");
    }
    panel.sameLine();
    if (panel.button("+Z")) {
      setDirectionalForce(
          state, Eigen::Vector3d(0.0, 0.0, kRigidCubesForceMagnitude), "+Z");
    }
    panel.text("time: " + std::to_string(context.simulationTime));
  };
  return controls;
}

dart::gui::KeyboardAction makeRigidCubesAction(
    std::string label,
    dart::gui::KeyboardShortcut shortcut,
    std::function<void(dart::gui::KeyboardActionContext&)> callback)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = shortcut;
  action.callback = std::move(callback);
  return action;
}

std::vector<dart::gui::KeyboardAction> createRigidCubesKeyboardActions(
    const std::shared_ptr<RigidCubesState>& state)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.push_back(makeRigidCubesAction(
      "Toggle playback",
      dart::gui::KeyboardShortcut::characterKey('p'),
      [](dart::gui::KeyboardActionContext& context) {
        if (context.lifecycle != nullptr) {
          dart::gui::togglePaused(*context.lifecycle);
        }
      }));
  actions.push_back(makeRigidCubesAction(
      "Toggle visualization markers",
      dart::gui::KeyboardShortcut::characterKey('v'),
      [state](dart::gui::KeyboardActionContext&) {
        toggleVisualizationMarkers(state);
      }));
  actions.push_back(makeRigidCubesAction(
      "Apply -X force",
      dart::gui::KeyboardShortcut::characterKey('1'),
      [state](dart::gui::KeyboardActionContext&) {
        setDirectionalForce(
            state, Eigen::Vector3d(-kRigidCubesForceMagnitude, 0.0, 0.0), "-X");
      }));
  actions.push_back(makeRigidCubesAction(
      "Apply +X force",
      dart::gui::KeyboardShortcut::characterKey('2'),
      [state](dart::gui::KeyboardActionContext&) {
        setDirectionalForce(
            state, Eigen::Vector3d(kRigidCubesForceMagnitude, 0.0, 0.0), "+X");
      }));
  actions.push_back(makeRigidCubesAction(
      "Apply -Z force",
      dart::gui::KeyboardShortcut::characterKey('3'),
      [state](dart::gui::KeyboardActionContext&) {
        setDirectionalForce(
            state, Eigen::Vector3d(0.0, 0.0, -kRigidCubesForceMagnitude), "-Z");
      }));
  actions.push_back(makeRigidCubesAction(
      "Apply +Z force",
      dart::gui::KeyboardShortcut::characterKey('4'),
      [state](dart::gui::KeyboardActionContext&) {
        setDirectionalForce(
            state, Eigen::Vector3d(0.0, 0.0, kRigidCubesForceMagnitude), "+Z");
      }));
  return actions;
}

dart::gui::OrbitCamera makeRigidCubesCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.7853981633974483;
  camera.pitch = 0.6154797086703874;
  camera.distance = 8.660254037844387;
  return camera;
}

} // namespace

dart::gui::ApplicationOptions makeRigidCubesScene()
{
  auto state = std::make_shared<RigidCubesState>();
  auto world = createRigidCubesWorld();

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.camera = makeRigidCubesCamera();
  options.preStep = [world, state]() {
    applyPendingForce(world, state);
  };
  options.panels.push_back(createControlsPanel(state));
  options.keyboardActions = createRigidCubesKeyboardActions(state);
  return options;
}

} // namespace dart::examples::demos
