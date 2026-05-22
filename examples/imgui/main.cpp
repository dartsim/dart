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
 *     copyright notice, this list of conditions and the disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
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
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr const char* kPanelTargetName = "panel extension target";

struct PanelExtensionScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<dart::dynamics::SimpleFrame> target;
  std::vector<dart::gui::Gizmo> gizmos;
  std::shared_ptr<int> preStepCount;
  std::shared_ptr<int> postStepCount;
  std::shared_ptr<int> preRenderCount;
  std::shared_ptr<int> postRenderCount;
  std::shared_ptr<bool> gravityEnabled;
};

std::shared_ptr<dart::dynamics::SimpleFrame> createPanelTarget()
{
  return dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kPanelTargetName);
}

PanelExtensionScene createPanelExtensionScene()
{
  PanelExtensionScene scene;
  scene.world = dart::simulation::World::create("panel_extension");
  scene.world->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
  scene.target = createPanelTarget();
  scene.world->addSimpleFrame(scene.target);
  dart::gui::Gizmo gizmo;
  gizmo.label = kPanelTargetName;
  gizmo.target = scene.target;
  gizmo.size = 0.24;
  scene.gizmos.push_back(std::move(gizmo));
  scene.preStepCount = std::make_shared<int>(0);
  scene.postStepCount = std::make_shared<int>(0);
  scene.preRenderCount = std::make_shared<int>(0);
  scene.postRenderCount = std::make_shared<int>(0);
  scene.gravityEnabled = std::make_shared<bool>(true);
  return scene;
}

dart::gui::RunOptions makePanelExtensionRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

dart::gui::OrbitCamera makePanelExtensionCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.up = Eigen::Vector3d(-0.24, -0.25, 0.94);
  camera.yaw = 0.8848934155088675;
  camera.pitch = 0.38410042777133657;
  camera.distance = 4.376539729055364;
  return camera;
}

void setPanelWorldGravity(
    const dart::simulation::WorldPtr& world, bool gravityEnabled)
{
  if (world == nullptr) {
    return;
  }

  if (gravityEnabled) {
    world->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
  } else {
    world->setGravity(Eigen::Vector3d::Zero());
  }
}

std::string formatCameraVector(const Eigen::Vector3d& value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << "(" << value.x() << ", "
         << value.y() << ", " << value.z() << ")";
  return stream.str();
}

dart::gui::KeyboardAction makePrintAction(
    std::string label,
    dart::gui::KeyboardShortcut shortcut,
    std::string message,
    dart::gui::KeyboardActionTrigger trigger
    = dart::gui::KeyboardActionTrigger::Press)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = shortcut;
  action.trigger = trigger;
  action.callback
      = [message = std::move(message)](dart::gui::KeyboardActionContext&) {
          std::cout << message << std::endl;
        };
  return action;
}

std::vector<dart::gui::KeyboardAction> createPanelExtensionKeyboardActions()
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.push_back(makePrintAction(
      "Lowercase q pressed",
      dart::gui::KeyboardShortcut::characterKey('q'),
      "Lowercase q pressed"));
  actions.push_back(makePrintAction(
      "Capital Q pressed",
      dart::gui::KeyboardShortcut::characterKey('Q'),
      "Capital Q pressed"));
  actions.push_back(makePrintAction(
      "Left arrow key pressed",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Left),
      "Left arrow key pressed"));
  actions.push_back(makePrintAction(
      "Right arrow key pressed",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Right),
      "Right arrow key pressed"));
  actions.push_back(makePrintAction(
      "Lowercase q released",
      dart::gui::KeyboardShortcut::characterKey('q'),
      "Lowercase q released",
      dart::gui::KeyboardActionTrigger::Release));
  actions.push_back(makePrintAction(
      "Capital Q released",
      dart::gui::KeyboardShortcut::characterKey('Q'),
      "Capital Q released",
      dart::gui::KeyboardActionTrigger::Release));
  actions.push_back(makePrintAction(
      "Left arrow key released",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Left),
      "Left arrow key released",
      dart::gui::KeyboardActionTrigger::Release));
  actions.push_back(makePrintAction(
      "Right arrow key released",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Right),
      "Right arrow key released",
      dart::gui::KeyboardActionTrigger::Release));
  return actions;
}

dart::gui::Panel createPanelExtensionControls(const PanelExtensionScene& scene)
{
  dart::gui::Panel controls;
  controls.title = "Tinkertoy Control";
  controls.buildWithContext = [scene](
                                  dart::gui::PanelBuilder& panel,
                                  dart::gui::PanelContext& context) {
    panel.text("Promoted panel extension example");
    panel.separator();

    panel.text("Simulation");
    if (context.lifecycle != nullptr) {
      if (panel.button("Play")) {
        context.lifecycle->paused = false;
      }
      panel.sameLine();
      if (panel.button("Pause")) {
        context.lifecycle->paused = true;
      }
      panel.sameLine();
      if (panel.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
      panel.sameLine();
      if (panel.button("Exit")) {
        dart::gui::requestExit(*context.lifecycle);
      }
    }
    panel.text("Time: " + std::to_string(context.simulationTime));
    if (scene.preStepCount != nullptr) {
      panel.text("Pre-step callbacks: " + std::to_string(*scene.preStepCount));
    }
    if (scene.postStepCount != nullptr) {
      panel.text(
          "Post-step callbacks: " + std::to_string(*scene.postStepCount));
    }
    if (scene.preRenderCount != nullptr) {
      panel.text(
          "Pre-render callbacks: " + std::to_string(*scene.preRenderCount));
    }
    if (scene.postRenderCount != nullptr) {
      panel.text(
          "Post-render callbacks: " + std::to_string(*scene.postRenderCount));
    }

    panel.separator();
    panel.text("World Options");
    if (scene.gravityEnabled != nullptr) {
      bool gravity = *scene.gravityEnabled;
      panel.checkbox("Gravity On/Off", gravity);
      if (gravity != *scene.gravityEnabled) {
        *scene.gravityEnabled = gravity;
        setPanelWorldGravity(scene.world, gravity);
      }
    }
    if (context.lighting.headlightsEnabled != nullptr) {
      bool headlights = *context.lighting.headlightsEnabled;
      if (panel.checkbox("Headlights On/Off", headlights)) {
        *context.lighting.headlightsEnabled = headlights;
      }
    }

    panel.separator();
    panel.text("View");
    panel.text("Eye   : " + formatCameraVector(context.camera.eye));
    panel.text("Center: " + formatCameraVector(context.camera.target));
    panel.text("Up    : " + formatCameraVector(context.camera.up));

    panel.separator();
    panel.text("Help");
    panel.text("User Guide");
    panel.text("Left drag orbits; right or middle drag pans; wheel zooms.");
    panel.text("Left-drag target gizmo arrows/planes/rings.");
    panel.text("q, Q, Left, and Right demonstrate keydown/release callbacks.");
    panel.text("About DART: project and libdart simulation libraries.");
    panel.text("selected: " + context.selectedLabel);
  };
  return controls;
}

} // namespace

int main(int argc, char* argv[])
{
  PanelExtensionScene scene = createPanelExtensionScene();

  dart::gui::ApplicationOptions options;
  options.world = scene.world;
  options.runDefaults = makePanelExtensionRunDefaults();
  options.camera = makePanelExtensionCamera();
  options.gizmos = scene.gizmos;
  options.preStep = [count = scene.preStepCount]() {
    if (count != nullptr) {
      ++*count;
    }
  };
  options.postStep = [count = scene.postStepCount]() {
    if (count != nullptr) {
      ++*count;
    }
  };
  options.preRender = [count = scene.preRenderCount]() {
    if (count != nullptr) {
      ++*count;
    }
  };
  options.postRender = [count = scene.postRenderCount]() {
    if (count != nullptr) {
      ++*count;
    }
  };
  options.keyboardActions = createPanelExtensionKeyboardActions();
  options.panels.push_back(createPanelExtensionControls(scene));

  return dart::gui::runApplication(argc, argv, options);
}
