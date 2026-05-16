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

#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include <cctype>
#include <cstddef>

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

std::string toLowerAscii(std::string_view value)
{
  std::string lowered(value);
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](char c) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  });
  return lowered;
}

std::optional<std::string_view> getOptionValue(
    std::string_view argument,
    std::string_view option,
    int& index,
    int argc,
    char* argv[])
{
  if (argument == option) {
    if (index + 1 >= argc) {
      throw std::runtime_error("Missing value for " + std::string(option));
    }
    return std::string_view(argv[++index]);
  }

  const std::string prefix = std::string(option) + "=";
  if (argument.starts_with(prefix)) {
    return argument.substr(prefix.size());
  }

  return std::nullopt;
}

std::optional<SolverType> parseSolver(std::string_view value)
{
  const std::string lowered = toLowerAscii(value);
  if (lowered == "dantzig") {
    return SolverType::Dantzig;
  }
  if (lowered == "pgs" || lowered == "projected-gauss-seidel") {
    return SolverType::Pgs;
  }
  return std::nullopt;
}

std::string solverName(SolverType solver)
{
  return solver == SolverType::Pgs ? "pgs" : "dantzig";
}

void printBoxStackingUsage(const char* executable)
{
  std::cout << "Usage: " << executable << " [--solver dantzig|pgs]\n"
            << "       [common dart::gui flags such as --headless, --frames,\n"
            << "        --screenshot, --out, --width, --height, --gui-scale]\n";
}

enum class ParseResult
{
  Ok,
  Help,
};

ParseResult parseBoxStackingConfig(
    int argc, char* argv[], BoxStackingConfig& config)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i] == nullptr ? "" : argv[i]);
    if (argument == "--help" || argument == "-h") {
      printBoxStackingUsage(argv[0]);
      return ParseResult::Help;
    }
    if (auto value = getOptionValue(argument, "--solver", i, argc, argv)) {
      if (auto solver = parseSolver(*value)) {
        config.solver = *solver;
      } else {
        throw std::runtime_error(
            "Unknown --solver value: " + std::string(*value));
      }
    }
  }

  return ParseResult::Ok;
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

dart::gui::RunOptions makeBoxStackingRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 800;
  options.height = 640;
  return options;
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
    }

    panel.text("Time: " + std::to_string(context.simulationTime));
    panel.text("Contacts: " + std::to_string(context.contactCount));
    panel.separator();
    panel.text("User Guide:");
    panel.text("Space toggles simulation; n steps once while paused.");
    panel.text("Left drag orbits, right/middle drag pans, wheel zooms.");
    panel.text("Use --gui-scale to scale this panel.");
  };
  return controls;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    BoxStackingConfig config;
    if (parseBoxStackingConfig(argc, argv, config) == ParseResult::Help) {
      return 0;
    }

    dart::gui::ApplicationOptions options;
    options.world = createBoxStackingWorld(config);
    options.runDefaults = makeBoxStackingRunDefaults();
    options.camera = makeBoxStackingCamera();
    options.panels.push_back(createControlsPanel(config));
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "box_stacking: " << e.what() << "\n";
    return 1;
  }
}
