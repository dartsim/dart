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

#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <charconv>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <cctype>
#include <cmath>

namespace {

constexpr const char* kGroundName = "lcp_physics_ground";
constexpr const char* kBoxPrefix = "lcp_physics_contact_box_";
constexpr const char* kSpherePrefix = "lcp_physics_ball_drop_sphere_";
constexpr int kSphereCount = 12;

enum class Scenario
{
  MassRatio,
  BoxStack,
  BallDrop,
  Dominos,
  InclinedPlane,
};

enum class SolverType
{
  Dantzig,
  Pgs,
};

struct ScenarioInfo
{
  Scenario type;
  std::string name;
  std::string description;
};

struct SolverInfo
{
  SolverType type;
  std::string name;
  std::string description;
};

struct LcpPhysicsConfig
{
  Scenario scenario = Scenario::MassRatio;
  SolverType solver = SolverType::Dantzig;
};

const std::vector<ScenarioInfo>& getScenarios()
{
  static const std::vector<ScenarioInfo> scenarios = {
      {Scenario::MassRatio,
       "mass_ratio",
       "heavy box on light box with a 1000:1 mass ratio"},
      {Scenario::BoxStack, "box_stack", "stacked box pyramid"},
      {Scenario::BallDrop, "ball_drop", "many spheres dropping and settling"},
      {Scenario::Dominos, "dominos", "sequential domino impulse propagation"},
      {Scenario::InclinedPlane, "inclined_plane", "sliding block on a ramp"},
  };
  return scenarios;
}

const std::vector<SolverInfo>& getSolvers()
{
  static const std::vector<SolverInfo> solvers = {
      {SolverType::Dantzig, "dantzig", "pivoting LCP solver"},
      {SolverType::Pgs, "pgs", "projected Gauss-Seidel LCP solver"},
  };
  return solvers;
}

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

const ScenarioInfo& scenarioInfo(Scenario scenario)
{
  for (const auto& info : getScenarios()) {
    if (info.type == scenario) {
      return info;
    }
  }
  return getScenarios().front();
}

const SolverInfo& solverInfo(SolverType solver)
{
  for (const auto& info : getSolvers()) {
    if (info.type == solver) {
      return info;
    }
  }
  return getSolvers().front();
}

std::optional<Scenario> parseScenario(std::string_view value)
{
  const std::string lowered = toLowerAscii(value);
  if (lowered == "mass" || lowered == "mass_ratio" || lowered == "mass-ratio") {
    return Scenario::MassRatio;
  }
  if (lowered == "box" || lowered == "stack" || lowered == "box_stack"
      || lowered == "box-stack") {
    return Scenario::BoxStack;
  }
  if (lowered == "ball" || lowered == "balls" || lowered == "ball_drop"
      || lowered == "ball-drop") {
    return Scenario::BallDrop;
  }
  if (lowered == "domino" || lowered == "dominos") {
    return Scenario::Dominos;
  }
  if (lowered == "incline" || lowered == "inclined"
      || lowered == "inclined_plane" || lowered == "inclined-plane") {
    return Scenario::InclinedPlane;
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

void printLcpUsage(const char* executable)
{
  std::cout << "Usage: " << executable
            << " [--scenario mass_ratio|box_stack|ball_drop|dominos|"
               "inclined_plane]\n"
            << "       [--solver dantzig|pgs] [--list]\n"
            << "       [common dart::gui flags such as --headless, --frames,\n"
            << "        --screenshot, --out, --width, --height, --gui-scale]\n";
}

void printLcpList()
{
  std::cout << "Available scenarios:\n";
  for (const auto& scenario : getScenarios()) {
    std::cout << "  " << scenario.name << ": " << scenario.description << "\n";
  }
  std::cout << "\nAvailable solvers:\n";
  for (const auto& solver : getSolvers()) {
    std::cout << "  " << solver.name << ": " << solver.description << "\n";
  }
}

enum class ParseResult
{
  Ok,
  HelpOrList,
};

ParseResult parseLcpPhysicsConfig(
    int argc, char* argv[], LcpPhysicsConfig& config)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i] == nullptr ? "" : argv[i]);
    if (argument == "--help" || argument == "-h") {
      printLcpUsage(argv[0]);
      return ParseResult::HelpOrList;
    }
    if (argument == "--list") {
      printLcpList();
      return ParseResult::HelpOrList;
    }
    if (auto value = getOptionValue(argument, "--scenario", i, argc, argv)) {
      if (auto scenario = parseScenario(*value)) {
        config.scenario = *scenario;
      } else {
        throw std::runtime_error(
            "Unknown --scenario value: " + std::string(*value));
      }
      continue;
    }
    if (auto value = getOptionValue(argument, "--solver", i, argc, argv)) {
      if (auto solver = parseSolver(*value)) {
        config.solver = *solver;
      } else {
        throw std::runtime_error(
            "Unknown --solver value: " + std::string(*value));
      }
      continue;
    }
  }

  return ParseResult::Ok;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  constexpr double thickness = 0.08;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(7.0, thickness, 4.5)));
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.72, 0.74, 0.72));
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.3);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().y() = -0.5 * thickness;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);
  return ground;
}

dart::dynamics::SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& color)
{
  auto box = dart::dynamics::Skeleton::create(name);
  auto [joint, body]
      = box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  joint->setTransformFromParentBodyNode(transform);

  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(size);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.3);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);
  shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(0.0);
  shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(0.0);
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), boxShape->computeInertia(mass)));
  return box;
}

dart::dynamics::SkeletonPtr createTranslatedBox(
    const std::string& suffix,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  return createBox(
      std::string(kBoxPrefix) + suffix, size, mass, transform, color);
}

dart::dynamics::SkeletonPtr createSphere(
    const std::string& suffix,
    double radius,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double mass)
{
  auto sphere
      = dart::dynamics::Skeleton::create(std::string(kSpherePrefix) + suffix);
  auto [joint, body]
      = sphere->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  joint->setTransformFromParentBodyNode(transform);

  auto sphereShape = std::make_shared<dart::dynamics::SphereShape>(radius);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(sphereShape);
  shapeNode->getVisualAspect()->setColor(color);
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), sphereShape->computeInertia(mass)));
  return sphere;
}

dart::simulation::WorldPtr createMassRatioScenario()
{
  auto world = dart::simulation::World::create("lcp_physics_mass_ratio");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(createGround());

  constexpr double massBoxSize = 0.36;
  world->addSkeleton(createTranslatedBox(
      "light_mass",
      Eigen::Vector3d::Constant(massBoxSize),
      1.0,
      Eigen::Vector3d(-2.0, 0.5 * massBoxSize, -0.85),
      Eigen::Vector3d(0.30, 0.72, 0.36)));
  world->addSkeleton(createTranslatedBox(
      "heavy_mass",
      Eigen::Vector3d::Constant(massBoxSize),
      1000.0,
      Eigen::Vector3d(-2.0, 1.5 * massBoxSize, -0.85),
      Eigen::Vector3d(0.84, 0.24, 0.20)));

  return world;
}

dart::simulation::WorldPtr createBoxStackScenario()
{
  auto world = dart::simulation::World::create("lcp_physics_box_stack");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(createGround());

  constexpr int pyramidLayers = 4;
  constexpr double pyramidBoxSize = 0.28;
  int pyramidBoxIndex = 0;
  for (int layer = 0; layer < pyramidLayers; ++layer) {
    const int boxesInLayer = pyramidLayers - layer;
    const double y = 0.5 * pyramidBoxSize + layer * pyramidBoxSize * 1.05;
    const double startX = -0.5 * (boxesInLayer - 1) * pyramidBoxSize * 1.12;
    for (int i = 0; i < boxesInLayer; ++i) {
      const double x = startX + i * pyramidBoxSize * 1.12;
      const double t
          = static_cast<double>(pyramidBoxIndex)
            / static_cast<double>(pyramidLayers * (pyramidLayers + 1) / 2);
      world->addSkeleton(createTranslatedBox(
          "stack_" + std::to_string(pyramidBoxIndex),
          Eigen::Vector3d::Constant(pyramidBoxSize),
          1.0,
          Eigen::Vector3d(x, y, -0.55),
          Eigen::Vector3d(0.28 + 0.42 * t, 0.52, 0.82 - 0.34 * t)));
      ++pyramidBoxIndex;
    }
  }

  return world;
}

dart::simulation::WorldPtr createDominosScenario()
{
  auto world = dart::simulation::World::create("lcp_physics_dominos");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(createGround());

  constexpr int dominoCount = 8;
  constexpr double dominoSpacing = 0.16;
  for (int i = 0; i < dominoCount; ++i) {
    const double x
        = (static_cast<double>(i) - 0.5 * (dominoCount - 1)) * dominoSpacing;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, 0.18, 0.85);
    if (i == 0) {
      transform.rotate(Eigen::AngleAxisd(0.30, Eigen::Vector3d::UnitZ()));
    }
    const double t = static_cast<double>(i) / static_cast<double>(dominoCount);
    world->addSkeleton(createBox(
        std::string(kBoxPrefix) + "domino_" + std::to_string(i),
        Eigen::Vector3d(0.055, 0.32, 0.15),
        0.5,
        transform,
        Eigen::Vector3d(0.18 + 0.52 * t, 0.34, 0.84 - 0.42 * t)));
  }

  return world;
}

dart::simulation::WorldPtr createBallDropScenario()
{
  auto world = dart::simulation::World::create("lcp_physics_ball_drop");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(createGround());

  int sphereIndex = 0;
  for (int xIndex = 0; xIndex < 4; ++xIndex) {
    for (int zIndex = 0; zIndex < 3; ++zIndex) {
      const double t = static_cast<double>(sphereIndex)
                       / static_cast<double>(kSphereCount);
      world->addSkeleton(createSphere(
          std::to_string(sphereIndex),
          0.11,
          Eigen::Vector3d(
              1.45 + (static_cast<double>(xIndex) - 1.5) * 0.28,
              0.65 + 0.10 * static_cast<double>(zIndex),
              -0.55 + (static_cast<double>(zIndex) - 1.0) * 0.28),
          Eigen::Vector3d(0.86 - 0.35 * t, 0.42 + 0.35 * t, 0.36),
          0.5));
      ++sphereIndex;
    }
  }

  return world;
}

dart::simulation::WorldPtr createInclinedPlaneScenario()
{
  auto world = dart::simulation::World::create("lcp_physics_inclined_plane");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  Eigen::Isometry3d rampTransform = Eigen::Isometry3d::Identity();
  constexpr double rampAngle = 0.4;
  rampTransform.translation() = Eigen::Vector3d(0.0, 0.5, 0.0);
  rampTransform.rotate(Eigen::AngleAxisd(rampAngle, Eigen::Vector3d::UnitZ()));
  auto ramp = createBox(
      "lcp_physics_ramp",
      Eigen::Vector3d(3.0, 0.1, 1.0),
      1.0,
      rampTransform,
      Eigen::Vector3d(0.50, 0.50, 0.50));
  ramp->setMobile(false);
  if (auto* rampShape = ramp->getBodyNode(0)->getShapeNode(0)) {
    rampShape->getDynamicsAspect()->setFrictionCoeff(0.5);
  }
  world->addSkeleton(ramp);

  constexpr double boxSize = 0.2;
  world->addSkeleton(createTranslatedBox(
      "sliding_box",
      Eigen::Vector3d::Constant(boxSize),
      1.0,
      Eigen::Vector3d(
          -1.0 * std::cos(rampAngle) + 0.5 * std::sin(rampAngle),
          1.0 * std::sin(rampAngle) + 0.5 * std::cos(rampAngle) + boxSize / 2.0,
          0.0),
      Eigen::Vector3d(0.82, 0.58, 0.18)));
  world->addSkeleton(createGround());
  return world;
}

dart::simulation::WorldPtr createScenario(Scenario scenario)
{
  switch (scenario) {
    case Scenario::MassRatio:
      return createMassRatioScenario();
    case Scenario::BoxStack:
      return createBoxStackScenario();
    case Scenario::BallDrop:
      return createBallDropScenario();
    case Scenario::Dominos:
      return createDominosScenario();
    case Scenario::InclinedPlane:
      return createInclinedPlaneScenario();
  }
  return createMassRatioScenario();
}

void applySolver(const dart::simulation::WorldPtr& world, SolverType solverType)
{
  auto* solver = world == nullptr ? nullptr : world->getConstraintSolver();
  if (solver == nullptr) {
    return;
  }

  switch (solverType) {
    case SolverType::Dantzig:
      solver->setLcpSolver(std::make_shared<dart::math::DantzigSolver>());
      break;
    case SolverType::Pgs:
      solver->setLcpSolver(std::make_shared<dart::math::PgsSolver>());
      break;
  }
}

dart::simulation::WorldPtr createLcpPhysicsWorld(const LcpPhysicsConfig& config)
{
  auto world = createScenario(config.scenario);
  applySolver(world, config.solver);
  return world;
}

dart::gui::RunOptions makeLcpRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 720;
  return options;
}

dart::gui::OrbitCamera makeLcpCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.3, 0.0);
  camera.yaw = 0.5880026035475675;
  camera.pitch = 0.7179841485128485;
  camera.distance = 4.358898943540674;
  return camera;
}

dart::gui::Panel createLcpPanel(const LcpPhysicsConfig& config)
{
  dart::gui::Panel panel;
  panel.title = "LCP Physics";
  panel.buildWithContext = [config](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("LCP contact solver benchmark scene");
    builder.text("scenario: " + scenarioInfo(config.scenario).name);
    builder.text("solver: " + solverInfo(config.solver).name);
    builder.text(scenarioInfo(config.scenario).description);
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
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  LcpPhysicsConfig config;
  try {
    if (parseLcpPhysicsConfig(argc, argv, config) == ParseResult::HelpOrList) {
      return 0;
    }
  } catch (const std::exception& e) {
    std::cerr << "lcp_physics: " << e.what() << "\n";
    return 1;
  }

  dart::gui::ApplicationOptions options;
  options.world = createLcpPhysicsWorld(config);
  options.runDefaults = makeLcpRunDefaults();
  options.camera = makeLcpCamera();
  options.panels.push_back(createLcpPanel(config));
  return dart::gui::runApplication(argc, argv, options);
}
