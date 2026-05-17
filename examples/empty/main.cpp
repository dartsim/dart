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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

void colorFrame(
    const std::shared_ptr<dart::dynamics::SimpleFrame>& frame,
    const Eigen::Vector3d& color)
{
  frame->getVisualAspect(true)->setColor(color);
}

std::shared_ptr<dart::dynamics::SimpleFrame> createBoxFrame(
    const std::string& name,
    dart::dynamics::Frame* parent,
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  auto frame = dart::dynamics::SimpleFrame::createShared(
      parent == nullptr ? dart::dynamics::Frame::World() : parent,
      name,
      transform);
  frame->setShape(std::make_shared<dart::dynamics::BoxShape>(size));
  colorFrame(frame, color);
  return frame;
}

dart::simulation::WorldPtr createEmptyWorld()
{
  auto world = dart::simulation::World::create("dartsim_empty");
  world->setGravity(Eigen::Vector3d::Zero());

  auto anchor = createBoxFrame(
      "interactive frame",
      nullptr,
      Eigen::Vector3d(4.0, -4.0, 0.0),
      Eigen::Vector3d(0.45, 0.45, 0.45),
      Eigen::Vector3d(0.95, 0.70, 0.15));
  world->addSimpleFrame(anchor);

  auto draggable = createBoxFrame(
      "draggable",
      anchor.get(),
      Eigen::Vector3d(-4.0, 4.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 1.0),
      Eigen::Vector3d(0.90, 0.00, 0.00));
  world->addSimpleFrame(draggable);

  world->addSimpleFrame(createBoxFrame(
      "X",
      nullptr,
      Eigen::Vector3d(8.0, 0.0, 0.0),
      Eigen::Vector3d(0.25, 0.25, 0.25),
      Eigen::Vector3d(0.90, 0.00, 0.00)));
  world->addSimpleFrame(createBoxFrame(
      "Y",
      nullptr,
      Eigen::Vector3d(0.0, 8.0, 0.0),
      Eigen::Vector3d(0.25, 0.25, 0.25),
      Eigen::Vector3d(0.00, 0.90, 0.00)));
  world->addSimpleFrame(createBoxFrame(
      "Z",
      nullptr,
      Eigen::Vector3d(0.0, 0.0, 8.0),
      Eigen::Vector3d(0.25, 0.25, 0.25),
      Eigen::Vector3d(0.00, 0.00, 0.90)));

  return world;
}

dart::gui::Panel createEmptyPanel()
{
  dart::gui::Panel panel;
  panel.title = "Empty";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Minimal public frame viewer scaffold");
    builder.text("selectable anchor, draggable child, and axis markers");
    builder.text("Keyboard actions print q, Q, Left, and Right keydown events");
    builder.text(
        "key-release callbacks and pre/post render hooks require public API "
        "follow-up.");
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
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

void emptyPreStepScaffold()
{
  // Preserves the promoted pre-step hook shape from the historical scaffold.
}

dart::gui::RunOptions makeEmptyRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

dart::gui::OrbitCamera makeEmptyCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.up = Eigen::Vector3d(-0.24, -0.25, 0.94);
  camera.yaw = 0.8848934155088675;
  camera.pitch = 0.38410042777133657;
  camera.distance = 4.376539729055364;
  return camera;
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

std::vector<dart::gui::KeyboardAction> createEmptyKeyboardActions()
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
  return actions;
}

} // namespace

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createEmptyWorld();
  options.preStep = emptyPreStepScaffold;
  options.runDefaults = makeEmptyRunDefaults();
  options.camera = makeEmptyCamera();
  options.panels.push_back(createEmptyPanel());
  options.keyboardActions = createEmptyKeyboardActions();
  return dart::gui::runApplication(argc, argv, options);
}
