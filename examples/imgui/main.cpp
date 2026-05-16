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
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

namespace {

constexpr const char* kPanelTargetName = "panel extension target";

struct PanelExtensionScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<dart::dynamics::SimpleFrame> target;
  std::shared_ptr<int> preStepCount;
  std::shared_ptr<bool> gravityEnabled;
};

std::shared_ptr<dart::dynamics::LineSegmentShape> createPanelTargetShape()
{
  auto handle = std::make_shared<dart::dynamics::LineSegmentShape>(6.0f);
  const std::size_t center = handle->addVertex(Eigen::Vector3d::Zero());
  handle->addVertex(Eigen::Vector3d(0.35, 0.0, 0.0), center);
  handle->addVertex(Eigen::Vector3d(-0.35, 0.0, 0.0), center);
  handle->addVertex(Eigen::Vector3d(0.0, 0.35, 0.0), center);
  handle->addVertex(Eigen::Vector3d(0.0, -0.35, 0.0), center);
  handle->addVertex(Eigen::Vector3d(0.0, 0.0, 0.35), center);
  handle->addVertex(Eigen::Vector3d(0.0, 0.0, -0.35), center);

  const auto a = handle->addVertex(Eigen::Vector3d(0.25, 0.25, 0.0));
  const auto b = handle->addVertex(Eigen::Vector3d(-0.25, 0.25, 0.0));
  const auto c = handle->addVertex(Eigen::Vector3d(-0.25, -0.25, 0.0));
  const auto d = handle->addVertex(Eigen::Vector3d(0.25, -0.25, 0.0));
  handle->addConnection(a, b);
  handle->addConnection(b, c);
  handle->addConnection(c, d);
  handle->addConnection(d, a);

  return handle;
}

std::shared_ptr<dart::dynamics::SimpleFrame> createPanelTarget()
{
  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kPanelTargetName);
  target->setShape(createPanelTargetShape());
  target->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.94, 0.78, 0.22, 0.8));
  return target;
}

PanelExtensionScene createPanelExtensionScene()
{
  PanelExtensionScene scene;
  scene.world = dart::simulation::World::create("panel_extension");
  scene.world->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
  scene.target = createPanelTarget();
  scene.world->addSimpleFrame(scene.target);
  scene.preStepCount = std::make_shared<int>(0);
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

dart::gui::KeyboardAction makePrintAction(
    std::string label,
    dart::gui::KeyboardShortcut shortcut,
    std::string message)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = shortcut;
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
      "Left arrow key pressed",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Left),
      "Left arrow key pressed"));
  actions.push_back(makePrintAction(
      "Right arrow key pressed",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Right),
      "Right arrow key pressed"));
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
    panel.text("Headlights need a public lighting control API.");

    panel.separator();
    panel.text("View");
    panel.text("Camera readout needs a public viewer-inspection API.");

    panel.separator();
    panel.text("Help");
    panel.text("User Guide");
    panel.text("Left drag orbits; right or middle drag pans; wheel zooms.");
    panel.text("Select the target, then Ctrl-left drag it.");
    panel.text("Arrow/PageUp/PageDown nudge the selected target.");
    panel.text("q, Left, and Right demonstrate keydown callbacks.");
    panel.text("Key release and shifted Q need public shortcut APIs.");
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
  options.preStep = [count = scene.preStepCount]() {
    if (count != nullptr) {
      ++*count;
    }
  };
  options.keyboardActions = createPanelExtensionKeyboardActions();
  options.panels.push_back(createPanelExtensionControls(scene));

  return dart::gui::runApplication(argc, argv, options);
}
