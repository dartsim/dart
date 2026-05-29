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

#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/helpers.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>
#include <dart/math/random.hpp>

#include <Eigen/Geometry>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

namespace dart::examples::demos {

namespace {

enum class SolverType
{
  Dantzig,
  Pgs,
};

struct BoxStackingConfig
{
  SolverType solver = SolverType::Dantzig;
};

std::string solverName(SolverType solver)
{
  return solver == SolverType::Pgs ? "pgs" : "dantzig";
}

void applyLcpSolver(
    dart::simulation::World* world,
    SolverType solverType,
    bool splitImpulseEnabled)
{
  auto* solver = world == nullptr ? nullptr : world->getConstraintSolver();
  if (solver == nullptr) {
    return;
  }

  if (solverType == SolverType::Pgs) {
    solver->setLcpSolver(std::make_shared<dart::math::PgsSolver>());
    solver->setSecondaryLcpSolver(nullptr);
  } else {
    solver->setLcpSolver(std::make_shared<dart::math::DantzigSolver>());
    solver->setSecondaryLcpSolver(std::make_shared<dart::math::PgsSolver>());
  }
  solver->setSplitImpulseEnabled(splitImpulseEnabled);
}

dart::dynamics::SkeletonPtr createBox(
    std::size_t index, const Eigen::Vector3d& position)
{
  auto skeleton
      = dart::dynamics::Skeleton::create("stack_box_" + std::to_string(index));
  auto jointAndBody
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 0.5));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(
      dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  constexpr double mass = 1.0;
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), shape->computeInertia(mass)));

  return skeleton;
}

dart::dynamics::SkeletonPtr createFloor()
{
  auto floor = dart::dynamics::Skeleton::create("floor");
  auto jointAndBody
      = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* joint = jointAndBody.first;
  auto* body = jointAndBody.second;

  constexpr double floorHeight = 0.01;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 * floorHeight);
  joint->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(10.0, 10.0, floorHeight));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  return floor;
}

dart::simulation::WorldPtr createBoxStackingWorld(
    const BoxStackingConfig& config)
{
  auto world = dart::simulation::World::create("box_stacking");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createFloor());

  constexpr int numBoxes = 5;
  for (int i = 0; i < numBoxes; ++i) {
    world->addSkeleton(createBox(
        static_cast<std::size_t>(i),
        Eigen::Vector3d(0.0, 0.0, 0.5 + 0.25 + i * 0.5)));
  }

  applyLcpSolver(world.get(), config.solver, false);
  return world;
}

dart::gui::OrbitCamera makeBoxStackingCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 2.0);
  camera.yaw = 0.7853981633974483;
  camera.pitch = 0.39121759480759044;
  camera.distance = 18.35755975068582;
  return camera;
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

std::vector<dart::gui::KeyboardAction> createBoxStackingKeyboardActions()
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

dart::gui::Panel createControlsPanel(const BoxStackingConfig& config)
{
  bool gravityEnabled = true;
  bool splitImpulseEnabled = false;
  SolverType solverType = config.solver;

  dart::gui::Panel controls;
  controls.title = "Box Stacking";
  controls.buildWithContext = [gravityEnabled, splitImpulseEnabled, solverType](
                                  dart::gui::PanelBuilder& panel,
                                  dart::gui::PanelContext& context) mutable {
    panel.text("Box stacking demo");
    panel.text("Stacked rigid-body contact demo");
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

    if (context.world != nullptr) {
      if (panel.checkbox("Gravity On/Off", gravityEnabled)) {
        context.world->setGravity(
            gravityEnabled ? Eigen::Vector3d(0.0, 0.0, -9.81)
                           : Eigen::Vector3d::Zero());
      }
      panel.text("LCP solver:");
      panel.text("selected: " + solverName(solverType));
      if (panel.button(
              solverType == SolverType::Dantzig ? "Dantzig (selected)"
                                                : "Dantzig")) {
        solverType = SolverType::Dantzig;
        applyLcpSolver(context.world, solverType, splitImpulseEnabled);
      }
      panel.sameLine();
      if (panel.button(
              solverType == SolverType::Pgs ? "PGS (selected)" : "PGS")) {
        solverType = SolverType::Pgs;
        applyLcpSolver(context.world, solverType, splitImpulseEnabled);
      }
      if (auto* solver = context.world->getConstraintSolver()) {
        if (panel.checkbox("Split impulse", splitImpulseEnabled)) {
          solver->setSplitImpulseEnabled(splitImpulseEnabled);
        }
      }
      if (context.lighting.headlightsEnabled != nullptr) {
        bool headlights = *context.lighting.headlightsEnabled;
        if (panel.checkbox("Headlights On/Off", headlights)) {
          *context.lighting.headlightsEnabled = headlights;
        }
      }
    }

    panel.text("Time: " + std::to_string(context.simulationTime));
    panel.text("Contacts: " + std::to_string(context.contactCount));
    panel.separator();
    panel.text("View");
    panel.text("Eye   : " + formatCameraVector(context.camera.eye));
    panel.text("Center: " + formatCameraVector(context.camera.target));
    panel.text("Up    : " + formatCameraVector(context.camera.up));
    panel.separator();
    panel.text("User Guide:");
    panel.text("Space toggles simulation; n steps once while paused.");
    panel.text("Left drag orbits, right/middle drag pans, wheel zooms.");
    panel.text("q, Q, Left, and Right print keydown/release messages.");
  };
  return controls;
}

} // namespace

dart::gui::ApplicationOptions makeBoxStackingScene()
{
  BoxStackingConfig config;

  dart::gui::ApplicationOptions options;
  options.world = createBoxStackingWorld(config);
  options.camera = makeBoxStackingCamera();
  options.panels.push_back(createControlsPanel(config));
  options.keyboardActions = createBoxStackingKeyboardActions();
  return options;
}

} // namespace dart::examples::demos
