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
#include <array>
#include <chrono>
#include <deque>
#include <iomanip>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <cmath>

namespace dart::examples::demos {

namespace {

constexpr int kBallDropSphereCount = 75;
constexpr int kBoxStackLayers = 5;
constexpr int kDominoCount = 20;
constexpr int kMaxMetricHistorySize = 120;
constexpr double kDefaultTimeStepHz = 1000.0;
constexpr double kDefaultGravityMagnitude = 9.81;

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
  std::string explanation;
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
  double timeStepHz = kDefaultTimeStepHz;
  double gravityMagnitude = kDefaultGravityMagnitude;
};

struct MetricSummary
{
  std::size_t samples = 0;
  double latest = 0.0;
  double average = 0.0;
  double maximum = 0.0;
};

const std::vector<ScenarioInfo>& getScenarios()
{
  static const std::vector<ScenarioInfo> scenarios = {
      {Scenario::MassRatio,
       "mass_ratio",
       "heavy box on light box with a 1000:1 mass ratio",
       "Tests solver stability with extreme mass ratios. A 1000kg box sits on "
       "a 1kg box; unstable solvers may jitter or let the light box sink."},
      {Scenario::BoxStack,
       "box_stack",
       "stacked box pyramid",
       "Tests shock propagation through a contact graph. The five-layer "
       "pyramid must distribute loads through many simultaneous contacts."},
      {Scenario::BallDrop,
       "ball_drop",
       "75 spheres dropping and settling",
       "Tests many-contact performance and stability with a seeded 75-ball "
       "drop, inspired by SimBenchmark contact-heavy scenes."},
      {Scenario::Dominos,
       "dominos",
       "sequential domino impulse propagation",
       "Tests impulse propagation accuracy as a 20-domino chain transfers "
       "energy from the first tilted block."},
      {Scenario::InclinedPlane,
       "inclined_plane",
       "sliding block on a ramp",
       "Tests friction model behavior as a block slides down a ramp with "
       "angle greater than the friction threshold."},
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

std::string formatFixed(double value, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string formatPair(const Eigen::Vector2d& value, int precision)
{
  return formatFixed(value.x(), precision) + " x "
         + formatFixed(value.y(), precision);
}

double elapsedMilliseconds(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

void appendMetricSample(std::deque<double>& history, double value)
{
  history.push_back(value);
  while (history.size() > kMaxMetricHistorySize) {
    history.pop_front();
  }
}

MetricSummary summarizeMetrics(const std::deque<double>& history)
{
  MetricSummary summary;
  summary.samples = history.size();
  if (history.empty()) {
    return summary;
  }

  summary.latest = history.back();
  summary.average = std::accumulate(history.begin(), history.end(), 0.0)
                    / static_cast<double>(history.size());
  summary.maximum = *std::max_element(history.begin(), history.end());
  return summary;
}

std::string formatMetricCell(
    const MetricSummary& summary, double value, std::string_view suffix)
{
  if (summary.samples == 0) {
    return "n/a";
  }
  return formatFixed(value, 3) + std::string(suffix);
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

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("ground");
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  constexpr double thickness = 0.1;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(20.0, thickness, 20.0)));
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.3);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);
  shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(0.0);
  shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(0.0);

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
    const std::string& name,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  return createBox(name, size, mass, transform, color);
}

dart::dynamics::SkeletonPtr createWeldedBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& color)
{
  auto box = dart::dynamics::Skeleton::create(name);
  auto [joint, body]
      = box->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
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
  return box;
}

dart::dynamics::SkeletonPtr createSphere(
    const std::string& name,
    double radius,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double mass)
{
  auto sphere = dart::dynamics::Skeleton::create(name);
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
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.5);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.6);
  shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(0.0);
  shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(0.0);
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), sphereShape->computeInertia(mass)));
  return sphere;
}

dart::simulation::WorldPtr createMassRatioScenario()
{
  auto world = dart::simulation::World::create("mass_ratio");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(createGround());

  constexpr double massBoxSize = 0.5;
  world->addSkeleton(createTranslatedBox(
      "light_box",
      Eigen::Vector3d::Constant(massBoxSize),
      1.0,
      Eigen::Vector3d(0.0, 0.5 * massBoxSize, 0.0),
      Eigen::Vector3d(0.4, 0.8, 0.4)));
  world->addSkeleton(createTranslatedBox(
      "heavy_box",
      Eigen::Vector3d::Constant(massBoxSize),
      1000.0,
      Eigen::Vector3d(0.0, massBoxSize * 1.6, 0.0),
      Eigen::Vector3d(0.8, 0.2, 0.2)));

  return world;
}

dart::simulation::WorldPtr createBoxStackScenario()
{
  auto world = dart::simulation::World::create("box_stack");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(createGround());

  constexpr double pyramidBoxSize = 0.3;
  int pyramidBoxIndex = 0;
  for (int layer = 0; layer < kBoxStackLayers; ++layer) {
    const int boxesInLayer = kBoxStackLayers - layer;
    const double y = 0.5 * pyramidBoxSize + layer * pyramidBoxSize * 1.05;
    const double startX = -(boxesInLayer - 1) * pyramidBoxSize * 0.55;
    for (int i = 0; i < boxesInLayer; ++i) {
      const double x = startX + i * pyramidBoxSize * 1.1;
      const double t
          = static_cast<double>(pyramidBoxIndex)
            / static_cast<double>(kBoxStackLayers * (kBoxStackLayers + 1) / 2);
      world->addSkeleton(createTranslatedBox(
          "box_" + std::to_string(pyramidBoxIndex),
          Eigen::Vector3d::Constant(pyramidBoxSize),
          1.0,
          Eigen::Vector3d(x, y, 0.0),
          Eigen::Vector3d(0.3 + 0.5 * t, 0.5, 0.8 - 0.5 * t)));
      ++pyramidBoxIndex;
    }
  }

  return world;
}

dart::simulation::WorldPtr createDominosScenario()
{
  auto world = dart::simulation::World::create("dominos");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(createGround());

  constexpr double dominoSpacing = 0.12;
  for (int i = 0; i < kDominoCount; ++i) {
    const double x
        = (static_cast<double>(i) - 0.5 * kDominoCount) * dominoSpacing;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, 0.15, 0.0);
    if (i == 0) {
      transform.rotate(Eigen::AngleAxisd(0.30, Eigen::Vector3d::UnitZ()));
    }
    const double t = static_cast<double>(i) / static_cast<double>(kDominoCount);
    world->addSkeleton(createBox(
        "domino_" + std::to_string(i),
        Eigen::Vector3d(0.05, 0.3, 0.15),
        0.5,
        transform,
        Eigen::Vector3d(0.2 + 0.6 * t, 0.3, 0.8 - 0.5 * t)));
  }

  return world;
}

dart::simulation::WorldPtr createBallDropScenario()
{
  auto world = dart::simulation::World::create("ball_drop");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(createGround());

  constexpr double radius = 0.1;
  constexpr int gridSize = 5;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> jitter(-0.02, 0.02);

  int sphereIndex = 0;
  for (int xIndex = 0; xIndex < gridSize; ++xIndex) {
    for (int zIndex = 0; zIndex < gridSize; ++zIndex) {
      for (int yIndex = 0; yIndex < 3; ++yIndex) {
        const double t = static_cast<double>(sphereIndex)
                         / static_cast<double>(kBallDropSphereCount);
        world->addSkeleton(createSphere(
            "ball_" + std::to_string(sphereIndex),
            radius,
            Eigen::Vector3d(
                (static_cast<double>(xIndex) - gridSize / 2.0) * radius * 2.5
                    + jitter(rng),
                radius + yIndex * radius * 2.2 + 0.5,
                (static_cast<double>(zIndex) - gridSize / 2.0) * radius * 2.5
                    + jitter(rng)),
            Eigen::Vector3d(0.8 - 0.4 * t, 0.4 + 0.4 * t, 0.4),
            0.5));
        ++sphereIndex;
      }
    }
  }

  return world;
}

dart::simulation::WorldPtr createInclinedPlaneScenario()
{
  auto world = dart::simulation::World::create("inclined_plane");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  Eigen::Isometry3d rampTransform = Eigen::Isometry3d::Identity();
  constexpr double rampAngle = 0.4;
  rampTransform.translation() = Eigen::Vector3d(0.0, 0.5, 0.0);
  rampTransform.rotate(Eigen::AngleAxisd(rampAngle, Eigen::Vector3d::UnitZ()));
  auto ramp = createWeldedBox(
      "ramp",
      Eigen::Vector3d(3.0, 0.1, 1.0),
      rampTransform,
      Eigen::Vector3d(0.50, 0.50, 0.50));
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

void applyParameters(
    const dart::simulation::WorldPtr& world, const LcpPhysicsConfig& config)
{
  if (world == nullptr) {
    return;
  }

  world->setTimeStep(1.0 / std::max(1.0, config.timeStepHz));
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -config.gravityMagnitude));
}

dart::simulation::WorldPtr createLcpPhysicsWorld(const LcpPhysicsConfig& config)
{
  auto world = createScenario(config.scenario);
  applyParameters(world, config);
  applySolver(world, config.solver);
  // The scenario skeletons are authored Y-up; reorient each to the canonical
  // Z-up convention (gravity is already set along -Z in applyParameters). This
  // also covers the runtime scenario switch, which rebuilds via this function.
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    reorientSkeletonToZUp(world->getSkeleton(i));
  }
  return world;
}

struct LcpPhysicsState
{
  using Clock = std::chrono::steady_clock;

  LcpPhysicsConfig config;
  dart::simulation::WorldPtr world;
  std::size_t stepCount = 0;
  Clock::time_point stepStart = Clock::now();
  Clock::time_point fpsWindowStart = Clock::now();
  int lastRenderedFrameCount = 0;
  int framesInFpsWindow = 0;
  double renderFps = 0.0;
  std::deque<double> stepTimeHistory;

  void beginStep()
  {
    stepStart = Clock::now();
  }

  void finishStep()
  {
    appendMetricSample(stepTimeHistory, elapsedMilliseconds(stepStart));
    ++stepCount;
  }

  void resetStepMetrics()
  {
    stepCount = 0;
    stepTimeHistory.clear();
    stepStart = Clock::now();
  }

  void updateRenderFps(const dart::gui::ViewerLifecycleState* lifecycle)
  {
    if (lifecycle == nullptr) {
      return;
    }

    const int renderedFrameCount = lifecycle->renderedFrames;
    if (renderedFrameCount < lastRenderedFrameCount) {
      lastRenderedFrameCount = renderedFrameCount;
      framesInFpsWindow = 0;
      fpsWindowStart = Clock::now();
      return;
    }

    framesInFpsWindow += renderedFrameCount - lastRenderedFrameCount;
    lastRenderedFrameCount = renderedFrameCount;

    const auto now = Clock::now();
    const double elapsedSeconds
        = std::chrono::duration<double>(now - fpsWindowStart).count();
    if (elapsedSeconds >= 0.5) {
      renderFps = static_cast<double>(framesInFpsWindow) / elapsedSeconds;
      framesInFpsWindow = 0;
      fpsWindowStart = now;
    }
  }
};

void addMetricTableRow(
    dart::gui::PanelBuilder& builder,
    std::string_view label,
    const MetricSummary& summary,
    std::string_view suffix)
{
  builder.tableNextRow();
  if (builder.tableNextColumn()) {
    builder.text(label);
  }
  if (builder.tableNextColumn()) {
    builder.text(formatMetricCell(summary, summary.latest, suffix));
  }
  if (builder.tableNextColumn()) {
    builder.text(formatMetricCell(summary, summary.average, suffix));
  }
  if (builder.tableNextColumn()) {
    builder.text(formatMetricCell(summary, summary.maximum, suffix));
  }
}

void addMetricFallbackText(
    dart::gui::PanelBuilder& builder,
    std::string_view label,
    const MetricSummary& summary,
    std::string_view suffix)
{
  builder.text(
      std::string(label)
      + " last=" + formatMetricCell(summary, summary.latest, suffix)
      + " avg=" + formatMetricCell(summary, summary.average, suffix)
      + " max=" + formatMetricCell(summary, summary.maximum, suffix));
}

std::vector<double> copyMetricHistory(const std::deque<double>& history)
{
  return {history.begin(), history.end()};
}

void replaceWorldContents(
    dart::simulation::World& target, const dart::simulation::WorldPtr& source)
{
  target.removeAllSensors();
  target.removeAllSimpleFrames();
  target.removeAllSkeletons();
  target.setName(source->getName());
  target.setTime(0.0);
  while (source->getNumSkeletons() > 0) {
    auto skeleton = source->getSkeleton(0);
    source->removeSkeleton(skeleton);
    target.addSkeleton(skeleton);
  }
}

void resetLcpWorld(LcpPhysicsState& state)
{
  auto nextWorld = createLcpPhysicsWorld(state.config);
  replaceWorldContents(*state.world, nextWorld);
  applyParameters(state.world, state.config);
  applySolver(state.world, state.config.solver);
  state.resetStepMetrics();
}

dart::gui::OrbitCamera makeLcpCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.3);
  camera.yaw = 0.5880026035475675;
  camera.pitch = 0.7179841485128485;
  camera.distance = 4.358898943540674;
  return camera;
}

dart::gui::Panel createLcpPanel(const std::shared_ptr<LcpPhysicsState>& state)
{
  dart::gui::Panel panel;
  panel.title = "LCP Physics Control";
  panel.initialPosition = std::array<double, 2>{10.0, 10.0};
  panel.initialSize = std::array<double, 2>{340.0, 600.0};
  panel.backgroundAlpha = 0.85;
  panel.autoResize = false;
  panel.horizontalScrollbar = true;
  panel.buildWithContext = [state](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    state->updateRenderFps(context.lifecycle);
    builder.text("LCP contact solver benchmark scene");
    builder.text("scenario: " + scenarioInfo(state->config.scenario).name);
    builder.text("solver: " + solverInfo(state->config.solver).name);

    if (builder.collapsingHeader("Simulation", true)) {
      if (context.lifecycle != nullptr) {
        if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
          dart::gui::togglePaused(*context.lifecycle);
        }
        builder.sameLine();
        if (builder.button("Step")) {
          dart::gui::requestSingleStep(*context.lifecycle);
        }
        builder.sameLine();
      }
      if (builder.button("Reset")) {
        resetLcpWorld(*state);
      }
      builder.text("time: " + std::to_string(context.simulationTime));
      builder.text("steps: " + std::to_string(state->stepCount));
    }

    if (builder.collapsingHeader("Scenario", true)) {
      for (const auto& scenario : getScenarios()) {
        const std::string label
            = (state->config.scenario == scenario.type ? "* " : "")
              + scenario.name;
        if (builder.button(label)) {
          state->config.scenario = scenario.type;
          resetLcpWorld(*state);
        }
        builder.text(scenario.description);
      }
    }

    if (builder.collapsingHeader("LCP Solver", true)) {
      for (const auto& solver : getSolvers()) {
        const std::string label
            = (state->config.solver == solver.type ? "* " : "") + solver.name;
        if (builder.button(label)) {
          state->config.solver = solver.type;
          applySolver(state->world, state->config.solver);
        }
        builder.text(solver.description);
      }
    }

    if (builder.collapsingHeader("Parameters", true)) {
      double timestepHz = state->config.timeStepHz;
      if (builder.slider("Timestep (Hz)", timestepHz, 100.0, 2000.0)) {
        state->config.timeStepHz = std::max(1.0, timestepHz);
        applyParameters(state->world, state->config);
      }

      double gravity = state->config.gravityMagnitude;
      if (builder.slider("Gravity (m/s^2)", gravity, 0.0, 20.0)) {
        state->config.gravityMagnitude = std::max(0.0, gravity);
        applyParameters(state->world, state->config);
      }
    }

    if (builder.collapsingHeader("Performance", true)) {
      builder.text("render fps: " + formatFixed(state->renderFps, 1));
      if (context.lifecycle != nullptr) {
        builder.text(
            "rendered frames: "
            + std::to_string(context.lifecycle->renderedFrames));
        builder.text(
            "skipped frames: "
            + std::to_string(context.lifecycle->skippedFrames));
      }
      builder.text("contacts: " + std::to_string(context.contactCount));
      if (context.world != nullptr) {
        builder.text(
            "bodies: " + std::to_string(context.world->getNumSkeletons()));
      }

      const MetricSummary stepTime = summarizeMetrics(state->stepTimeHistory);
      static constexpr std::array<std::string_view, 4> kMetricColumns
          = {"metric", "last", "avg", "max"};
      if (builder.beginTable("lcp_performance", kMetricColumns)) {
        addMetricTableRow(builder, "step time", stepTime, " ms");
        builder.endTable();
      } else {
        addMetricFallbackText(builder, "step time", stepTime, " ms");
      }
      const std::vector<double> stepTimeValues
          = copyMetricHistory(state->stepTimeHistory);
      builder.plotLines("Step time (ms)", stepTimeValues);
    }

    if (builder.collapsingHeader("UI Debug")) {
      builder.text("DisplaySize: " + formatPair(context.ui.displaySize, 0));
      builder.text(
          "FramebufferScale: " + formatPair(context.ui.framebufferScale, 2));
      builder.text("FontSize: " + formatFixed(context.ui.fontSize, 1));
      builder.text(
          "FontGlobalScale: " + formatFixed(context.ui.fontGlobalScale, 3));
      builder.text("UiScale: " + formatFixed(context.ui.uiScale, 2));
      if (context.ui.fontTextureSize.has_value()) {
        builder.text(
            "FontTex: " + std::to_string((*context.ui.fontTextureSize)[0])
            + " x " + std::to_string((*context.ui.fontTextureSize)[1]));
      }
    }

    if (builder.collapsingHeader("About This Scenario")) {
      builder.text(scenarioInfo(state->config.scenario).explanation);
    }
  };
  return panel;
}

} // namespace

dart::gui::ApplicationOptions makeLcpPhysicsScene()
{
  const LcpPhysicsConfig config;

  auto state = std::make_shared<LcpPhysicsState>();
  state->config = config;
  state->world = createLcpPhysicsWorld(config);

  dart::gui::ApplicationOptions options;
  options.world = state->world;
  options.camera = makeLcpCamera();
  options.preStep = [state]() {
    state->beginStep();
  };
  options.postStep = [state]() {
    state->finishStep();
  };
  options.panels.push_back(createLcpPanel(state));
  return options;
}

} // namespace dart::examples::demos
