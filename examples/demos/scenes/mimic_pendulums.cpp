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

#include <dart/config.hpp>

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/helpers.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace dart::examples::demos {

namespace {

constexpr const char* kWorldUri
    = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";
constexpr const char* kGridName = "mimic_pendulums_xy_grid";
constexpr const char* kGridBodyName = "mimic_pendulums_xy_grid_body";
constexpr const char* kGridShapeName = "mimic_pendulums_xy_grid_lines";

enum class SolverType
{
  Dantzig,
  Pgs,
};

struct MimicPendulumsConfig
{
  SolverType solver = SolverType::Dantzig;
  std::string collisionDetector = "file";
};

struct MimicPairView
{
  std::string label;
  const dart::dynamics::Joint* follower = nullptr;
  const dart::dynamics::Joint* reference = nullptr;
  std::size_t followerDof = 0;
  std::size_t referenceDof = 0;
  const dart::dynamics::BodyNode* base = nullptr;
  Eigen::Vector3d baseStart = Eigen::Vector3d::Zero();
};

struct PaletteEntry
{
  std::string model;
  Eigen::Vector3d color;
  std::string label;
};

struct MimicPendulumsScene
{
  dart::simulation::WorldPtr world;
  std::vector<MimicPairView> pairs;
  MimicPendulumsConfig config;
};

struct MimicPendulumsState
{
  dart::simulation::WorldPtr world;
  std::vector<MimicPairView> pairs;
  MimicPendulumsConfig config;
};

const std::vector<PaletteEntry>& getPalette()
{
  static const std::vector<PaletteEntry> palette = {
      {"pendulum_with_base", Eigen::Vector3d(0.7, 0.7, 0.7), "uncoupled"},
      {"pendulum_with_base_mimic_slow_follows_fast",
       Eigen::Vector3d(0.9, 0.35, 0.35),
       "slow follows fast"},
      {"pendulum_with_base_mimic_fast_follows_slow",
       Eigen::Vector3d(0.35, 0.5, 0.95),
       "fast follows slow"},
  };
  return palette;
}

std::string formatDouble(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string formatAngle(double radians)
{
  return formatDouble(radians) + " rad ("
         + formatDouble(dart::math::toDegree(radians), 1) + " deg)";
}

std::string solverName(SolverType solver)
{
  return solver == SolverType::Pgs ? "pgs" : "dantzig";
}

Eigen::Vector3d translationOf(const dart::dynamics::BodyNode* body)
{
  if (body == nullptr) {
    return Eigen::Vector3d::Zero();
  }
  return body->getWorldTransform().translation();
}

void applyCollisionDetector(
    const MimicPendulumsConfig& config, const dart::simulation::WorldPtr& world)
{
  if (world == nullptr || config.collisionDetector == "file") {
    return;
  }
  if (config.collisionDetector == "dart") {
    world->setCollisionDetector(dart::simulation::CollisionDetectorType::Dart);
    return;
  }
  if (config.collisionDetector == "fcl") {
    world->setCollisionDetector(dart::simulation::CollisionDetectorType::Fcl);
    return;
  }
#if DART_HAVE_BULLET
  if (config.collisionDetector == "bullet") {
    world->setCollisionDetector(
        dart::simulation::CollisionDetectorType::Bullet);
    return;
  }
#endif
#if DART_HAVE_ODE
  if (config.collisionDetector == "ode") {
    world->setCollisionDetector(dart::simulation::CollisionDetectorType::Ode);
    return;
  }
#endif
  throw std::runtime_error(
      "Requested collision detector is not available: "
      + config.collisionDetector);
}

void applyLcpSolver(
    const MimicPendulumsConfig& config, const dart::simulation::WorldPtr& world)
{
  auto* solver = world == nullptr ? nullptr : world->getConstraintSolver();
  if (solver == nullptr) {
    return;
  }
  if (config.solver == SolverType::Pgs) {
    solver->setLcpSolver(std::make_shared<dart::math::PgsSolver>());
    solver->setSecondaryLcpSolver(nullptr);
  } else {
    solver->setLcpSolver(std::make_shared<dart::math::DantzigSolver>());
    solver->setSecondaryLcpSolver(std::make_shared<dart::math::PgsSolver>());
  }
}

void retargetMimicsToBaseline(
    const dart::simulation::WorldPtr& world, const std::string& baselineName)
{
  const auto baseline
      = world == nullptr ? nullptr : world->getSkeleton(baselineName);
  if (baseline == nullptr) {
    throw std::runtime_error(
        "mimic_pendulums world is missing baseline skeleton: " + baselineName);
  }

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr) {
      continue;
    }

    for (std::size_t jointIndex = 0; jointIndex < skeleton->getNumJoints();
         ++jointIndex) {
      auto* joint = skeleton->getJoint(jointIndex);
      if (joint == nullptr) {
        continue;
      }

      const auto props = joint->getMimicDofProperties();
      if (props.empty()) {
        continue;
      }

      if (skeleton == baseline) {
        std::vector<dart::dynamics::MimicDofProperties> clearedProps(
            joint->getNumDofs());
        joint->setMimicJointDofs(clearedProps);
        joint->setActuatorType(dart::dynamics::Joint::FORCE);
        joint->setUseCouplerConstraint(false);
        continue;
      }

      bool updated = false;
      for (std::size_t dofIndex = 0; dofIndex < props.size(); ++dofIndex) {
        auto prop = props[dofIndex];
        if (prop.mReferenceJoint == nullptr) {
          continue;
        }

        auto* reference = baseline->getJoint(prop.mReferenceJoint->getName());
        if (reference == nullptr) {
          continue;
        }

        prop.mReferenceJoint = reference;
        joint->setMimicJointDof(dofIndex, prop);
        updated = true;
      }

      if (updated) {
        joint->setActuatorType(dart::dynamics::Joint::MIMIC);
        joint->setUseCouplerConstraint(false);
      }
    }
  }
}

std::vector<MimicPairView> collectMimicPairs(
    const dart::simulation::WorldPtr& world)
{
  std::vector<MimicPairView> pairs;
  if (world == nullptr) {
    return pairs;
  }

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr) {
      continue;
    }

    const auto* base = skeleton->getBodyNode("base");
    for (std::size_t jointIndex = 0; jointIndex < skeleton->getNumJoints();
         ++jointIndex) {
      const auto* follower = skeleton->getJoint(jointIndex);
      if (follower == nullptr) {
        continue;
      }

      const auto props = follower->getMimicDofProperties();
      for (std::size_t dofIndex = 0; dofIndex < props.size(); ++dofIndex) {
        const auto& prop = props[dofIndex];
        if (prop.mReferenceJoint == nullptr) {
          continue;
        }

        MimicPairView view;
        view.label = skeleton->getName() + ": " + follower->getName() + "["
                     + std::to_string(dofIndex) + "] -> "
                     + prop.mReferenceJoint->getName() + "["
                     + std::to_string(prop.mReferenceDofIndex) + "]";
        view.follower = follower;
        view.reference = prop.mReferenceJoint;
        view.followerDof = dofIndex;
        view.referenceDof = prop.mReferenceDofIndex;
        view.base = base;
        view.baseStart = translationOf(base);
        pairs.push_back(std::move(view));
      }
    }
  }

  return pairs;
}

void tintBases(const dart::simulation::WorldPtr& world)
{
  for (const auto& entry : getPalette()) {
    const auto skeleton = world->getSkeleton(entry.model);
    if (skeleton == nullptr) {
      continue;
    }

    auto* base = skeleton->getBodyNode("base");
    if (base != nullptr) {
      base->setColor(entry.color);
    }
  }
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createMimicGridShape()
{
  auto grid = std::make_shared<dart::dynamics::LineSegmentShape>(1.0f);
  constexpr double halfExtent = 5.0;
  constexpr int cells = 20;
  constexpr double spacing = (2.0 * halfExtent) / cells;
  for (int index = 0; index <= cells; ++index) {
    const double coordinate = -halfExtent + index * spacing;
    const auto xStart
        = grid->addVertex(Eigen::Vector3d(-halfExtent, coordinate, 0.0));
    grid->addVertex(Eigen::Vector3d(halfExtent, coordinate, 0.0), xStart);
    const auto yStart
        = grid->addVertex(Eigen::Vector3d(coordinate, -halfExtent, 0.0));
    grid->addVertex(Eigen::Vector3d(coordinate, halfExtent, 0.0), yStart);
  }
  return grid;
}

dart::dynamics::SkeletonPtr createMimicGrid()
{
  auto ground = dart::dynamics::Skeleton::create(kGridName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  body->setName(kGridBodyName);
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      createMimicGridShape());
  shapeNode->setName(kGridShapeName);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.45, 0.48, 0.45));
  return ground;
}

MimicPendulumsScene createMimicPendulumsScene(
    const MimicPendulumsConfig& config)
{
  MimicPendulumsScene scene;
  scene.config = config;
  auto world = dart::io::readWorld(kWorldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load mimic_pendulums world from " + std::string(kWorldUri));
  }
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  retargetMimicsToBaseline(world, "pendulum_with_base");
  applyCollisionDetector(config, world);
  applyLcpSolver(config, world);
  scene.pairs = collectMimicPairs(world);
  tintBases(world);

  auto ground = world->getSkeleton("ground_plane");
  if (ground == nullptr) {
    throw std::runtime_error("mimic_pendulums world is missing ground_plane");
  }
  world->addSkeleton(createMimicGrid());

  for (const auto& entry : getPalette()) {
    if (world->getSkeleton(entry.model) == nullptr) {
      throw std::runtime_error(
          "mimic_pendulums world is missing skeleton: " + entry.model);
    }
  }

  scene.world = std::move(world);
  return scene;
}

dart::gui::OrbitCamera makeMimicPendulumsCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.5, 0.0, 1.5);
  camera.yaw = -0.7509290623979403;
  camera.pitch = 0.2391161269830301;
  camera.distance = 10.559356040971437;
  return camera;
}

void refreshBaseAnchors(MimicPendulumsState& state)
{
  for (auto& pair : state.pairs) {
    pair.baseStart = translationOf(pair.base);
  }
}

dart::gui::Panel createMimicPendulumsPanel(
    const std::shared_ptr<MimicPendulumsState>& state)
{
  dart::gui::Panel panel;
  panel.title = "Mimic constraint debugger";
  panel.initialPosition = std::array<double, 2>{10.0, 10.0};
  panel.initialSize = std::array<double, 2>{800.0, 600.0};
  panel.autoResize = false;
  panel.buildWithContext = [state](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("SDF mimic-joint pendulum comparison");
    builder.text("gray: uncoupled, red/blue: opposite mimic mappings");
    builder.text("SDF: " + std::string(kWorldUri));
    builder.text("solver: " + solverName(state->config.solver));
    builder.text("collision: " + state->config.collisionDetector);
    builder.separator();
    if (context.lifecycle != nullptr) {
      bool simulating = !context.lifecycle->paused;
      if (builder.checkbox("Run simulation", simulating)) {
        context.lifecycle->paused = !simulating;
      }
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step 1")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Reset world")) {
        if (context.world != nullptr) {
          context.world->reset();
        }
        refreshBaseAnchors(*state);
      }
    }

    builder.separator();
    builder.text("Collision / solver");
#if DART_HAVE_ODE
    bool useOdeCollision = state->config.collisionDetector == "ode";
    if (builder.checkbox(
            "Use ODE collision (closer to Gazebo repro)", useOdeCollision)) {
      state->config.collisionDetector = useOdeCollision ? "ode" : "file";
      applyCollisionDetector(state->config, state->world);
    }
#else
    builder.text("ODE collision detector not built");
#endif

    bool forcePgsSolver = state->config.solver == SolverType::Pgs;
    if (builder.checkbox("Force PGS solver", forcePgsSolver)) {
      state->config.solver
          = forcePgsSolver ? SolverType::Pgs : SolverType::Dantzig;
      applyLcpSolver(state->config, state->world);
    }

    builder.text(
        "World time: " + formatDouble(context.simulationTime) + " s | dt "
        + formatDouble(state->world->getTimeStep(), 4));
    builder.text("Contacts last step: " + std::to_string(context.contactCount));
    builder.text("mimic pairs: " + std::to_string(state->pairs.size()));
    builder.separator();

    builder.text("Rigs:");
    for (const auto& entry : getPalette()) {
      builder.colorSwatch(
          entry.model + " (" + entry.label + ")",
          Eigen::Vector4d(
              entry.color.x(), entry.color.y(), entry.color.z(), 1.0));
    }
    builder.separator();

    if (state->pairs.empty()) {
      builder.text(
          "No mimic joints were parsed from " + std::string(kWorldUri));
    }

    constexpr std::array<std::string_view, 6> kMimicColumns{
        "Pair",
        "Reference (rad)",
        "Follower (rad)",
        "Error (rad)",
        "Velocity error (rad/s)",
        "Base drift (m)"};
    const bool useTable = builder.beginTable("mimic_table", kMimicColumns);
    if (!useTable) {
      builder.text(
          "Pair | Reference (rad) | Follower (rad) | Error (rad) | Velocity "
          "error (rad/s) | Base drift (m)");
    }
    for (const auto& pair : state->pairs) {
      if (pair.follower == nullptr || pair.reference == nullptr) {
        continue;
      }
      const double referencePosition
          = pair.reference->getPosition(pair.referenceDof);
      const double followerPosition
          = pair.follower->getPosition(pair.followerDof);
      const double positionError = followerPosition - referencePosition;
      const double referenceVelocity
          = pair.reference->getVelocity(pair.referenceDof);
      const double followerVelocity
          = pair.follower->getVelocity(pair.followerDof);
      const double velocityError = followerVelocity - referenceVelocity;
      const double baseDrift
          = (translationOf(pair.base) - pair.baseStart).norm();

      if (useTable) {
        builder.tableNextRow();
        builder.tableNextColumn();
        builder.text(pair.label);
        builder.tableNextColumn();
        builder.text(formatAngle(referencePosition));
        builder.tableNextColumn();
        builder.text(formatAngle(followerPosition));
        builder.tableNextColumn();
        builder.text(formatDouble(positionError));
        builder.tableNextColumn();
        builder.text(formatDouble(velocityError));
        builder.tableNextColumn();
        builder.text(formatDouble(baseDrift));
      } else {
        builder.text(
            pair.label + ": reference " + formatAngle(referencePosition)
            + ", follower " + formatAngle(followerPosition)
            + ", position error " + formatDouble(positionError)
            + " rad, velocity error " + formatDouble(velocityError)
            + " rad/s, base drift " + formatDouble(baseDrift) + " m");
      }
    }
    if (useTable) {
      builder.endTable();
    }
  };
  return panel;
}

} // namespace

dart::gui::ApplicationOptions makeMimicPendulumsScene()
{
  MimicPendulumsConfig config;
  auto scene = createMimicPendulumsScene(config);
  auto state = std::make_shared<MimicPendulumsState>();
  state->world = scene.world;
  state->pairs = scene.pairs;
  state->config = scene.config;

  dart::gui::ApplicationOptions options;
  options.world = state->world;
  options.camera = makeMimicPendulumsCamera();
  options.panels.push_back(createMimicPendulumsPanel(state));
  return options;
}

} // namespace dart::examples::demos
