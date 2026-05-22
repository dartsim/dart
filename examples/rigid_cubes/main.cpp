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

#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr const char* kRigidCubesUri = "dart://sample/skel/cubes.skel";
constexpr double kRigidCubesForceMagnitude = 500.0;

struct RigidCubesState
{
  Eigen::Vector3d pendingForce = Eigen::Vector3d::Zero();
  bool visualizationMarkersVisible = true;
};

void printRigidCubesInstructions()
{
  std::cout << "Rigid Cubes Example\n"
            << "Controls:\n"
            << "  space bar: simulation on/off\n"
            << "  'p': playback/stop\n"
            << "  'v': visualization on/off\n"
            << "  '1'-'4': apply directional forces\n"
            << "    '1': -X force    '2': +X force\n"
            << "    '3': -Z force    '4': +Z force\n";
}

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

dart::gui::RunOptions makeRigidCubesRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
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

int main(int argc, char* argv[])
{
  try {
    printRigidCubesInstructions();

    auto state = std::make_shared<RigidCubesState>();
    auto world = createRigidCubesWorld();

    dart::gui::ApplicationOptions options;
    options.world = world;
    options.runDefaults = makeRigidCubesRunDefaults();
    options.camera = makeRigidCubesCamera();
    options.preStep = [world, state]() {
      applyPendingForce(world, state);
    };
    options.panels.push_back(createControlsPanel(state));
    options.keyboardActions = createRigidCubesKeyboardActions(state);
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "rigid_cubes: " << e.what() << "\n";
    return 1;
  }
}
