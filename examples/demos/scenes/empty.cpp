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

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

struct EmptyHookState
{
  int preStepCount = 0;
  int postStepCount = 0;
  int preRenderCount = 0;
  int postRenderCount = 0;
};

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

dart::gui::Panel createEmptyPanel(const std::shared_ptr<EmptyHookState>& state)
{
  dart::gui::Panel panel;
  panel.title = "Empty";
  panel.buildWithContext = [state](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Minimal public frame viewer scaffold");
    builder.text("selectable anchor, draggable child, and axis markers");
    builder.text("Keyboard actions print q, Q, Left, and Right keydown events");
    builder.text(
        "Key-release actions print q, Q, Left, and Right release events.");
    if (state != nullptr) {
      builder.text(
          "Pre-step callbacks: " + std::to_string(state->preStepCount));
      builder.text(
          "Post-step callbacks: " + std::to_string(state->postStepCount));
      builder.text(
          "Pre-render callbacks: " + std::to_string(state->preRenderCount));
      builder.text(
          "Post-render callbacks: " + std::to_string(state->postRenderCount));
    }
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

void emptyPreStepScaffold(const std::shared_ptr<EmptyHookState>& state)
{
  if (state != nullptr) {
    ++state->preStepCount;
  }
}

void emptyPostStepScaffold(const std::shared_ptr<EmptyHookState>& state)
{
  if (state != nullptr) {
    ++state->postStepCount;
  }
}

void emptyPreRenderScaffold(const std::shared_ptr<EmptyHookState>& state)
{
  if (state != nullptr) {
    ++state->preRenderCount;
  }
}

void emptyPostRenderScaffold(const std::shared_ptr<EmptyHookState>& state)
{
  if (state != nullptr) {
    ++state->postRenderCount;
  }
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

} // namespace

dart::gui::ApplicationOptions makeEmptyScene()
{
  auto hookState = std::make_shared<EmptyHookState>();

  dart::gui::ApplicationOptions options;
  options.world = createEmptyWorld();
  options.preStep = [hookState]() {
    emptyPreStepScaffold(hookState);
  };
  options.postStep = [hookState]() {
    emptyPostStepScaffold(hookState);
  };
  options.preRender = [hookState]() {
    emptyPreRenderScaffold(hookState);
  };
  options.postRender = [hookState]() {
    emptyPostRenderScaffold(hookState);
  };
  options.camera = makeEmptyCamera();
  options.panels.push_back(createEmptyPanel(hookState));
  options.keyboardActions = createEmptyKeyboardActions();
  return options;
}

} // namespace dart::examples::demos
