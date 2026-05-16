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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <cctype>

namespace {

constexpr const char* kWorldUri
    = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";
constexpr const char* kVisualGroundName = "visual_mimic_pendulums_ground";
constexpr const char* kSkeletonPrefix = "visual_mimic_pendulum_";

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

struct MimicPendulumsScene
{
  dart::simulation::WorldPtr world;
  std::vector<MimicPairView> pairs;
  MimicPendulumsConfig config;
};

std::string formatDouble(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
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

void printMimicUsage(const char* executable)
{
  std::cout
      << "Usage: " << executable
      << " [--solver dantzig|pgs] [--collision file|dart|fcl|bullet|ode]\n"
      << "       [common dart::gui flags such as --headless, --frames,\n"
      << "        --screenshot, --out, --width, --height, --gui-scale]\n";
}

enum class ParseResult
{
  Ok,
  Help,
};

ParseResult parseMimicPendulumsConfig(
    int argc, char* argv[], MimicPendulumsConfig& config)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i] == nullptr ? "" : argv[i]);
    if (argument == "--help" || argument == "-h") {
      printMimicUsage(argv[0]);
      return ParseResult::Help;
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
    if (auto value = getOptionValue(argument, "--collision", i, argc, argv)) {
      const std::string collision = toLowerAscii(*value);
      if (collision == "file" || collision == "dart" || collision == "fcl"
          || collision == "bullet" || collision == "ode") {
        config.collisionDetector = collision;
      } else {
        throw std::runtime_error(
            "Unknown --collision value: " + std::string(*value));
      }
      continue;
    }
  }

  return ParseResult::Ok;
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

dart::dynamics::SkeletonPtr createVisualGround()
{
  auto ground = dart::dynamics::Skeleton::create(kVisualGroundName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 3.0, -0.02);
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(2.2, 7.2, 0.04)));
  shapeNode->setRelativeTransform(transform);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.47, 0.50, 0.47));
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

  auto ground = world->getSkeleton("ground_plane");
  if (ground == nullptr) {
    throw std::runtime_error("mimic_pendulums world is missing ground_plane");
  }
  ground->setName("visual_mimic_pendulums_imported_ground");
  world->addSkeleton(createVisualGround());

  const std::array<std::pair<const char*, Eigen::Vector3d>, 3> rigs{{
      {"pendulum_with_base", Eigen::Vector3d(0.62, 0.62, 0.62)},
      {"pendulum_with_base_mimic_slow_follows_fast",
       Eigen::Vector3d(0.90, 0.34, 0.34)},
      {"pendulum_with_base_mimic_fast_follows_slow",
       Eigen::Vector3d(0.28, 0.48, 0.92)},
  }};

  for (std::size_t i = 0; i < rigs.size(); ++i) {
    auto skeleton = world->getSkeleton(rigs[i].first);
    if (skeleton == nullptr) {
      throw std::runtime_error(
          "mimic_pendulums world is missing skeleton: "
          + std::string(rigs[i].first));
    }
    skeleton->setName(std::string(kSkeletonPrefix) + std::to_string(i));
    for (std::size_t bodyIndex = 0; bodyIndex < skeleton->getNumBodyNodes();
         ++bodyIndex) {
      if (auto* body = skeleton->getBodyNode(bodyIndex)) {
        body->setColor(rigs[i].second);
      }
    }
  }

  scene.world = std::move(world);
  return scene;
}

dart::gui::RunOptions makeMimicPendulumsRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 720;
  return options;
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

dart::gui::Panel createMimicPendulumsPanel(const MimicPendulumsScene& scene)
{
  dart::gui::Panel panel;
  panel.title = "Mimic Pendulums";
  panel.buildWithContext = [pairs = scene.pairs, config = scene.config](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("SDF mimic-joint pendulum comparison");
    builder.text("gray: uncoupled, red/blue: opposite mimic mappings");
    builder.text("solver: " + solverName(config.solver));
    builder.text("collision: " + config.collisionDetector);
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
    builder.text("mimic pairs: " + std::to_string(pairs.size()));
    builder.separator();
    for (const auto& pair : pairs) {
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
      builder.text(
          pair.label + ": pos err " + formatDouble(positionError) + ", vel err "
          + formatDouble(velocityError) + ", base drift "
          + formatDouble(baseDrift));
    }
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    MimicPendulumsConfig config;
    if (parseMimicPendulumsConfig(argc, argv, config) == ParseResult::Help) {
      return 0;
    }
    auto scene = createMimicPendulumsScene(config);

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.runDefaults = makeMimicPendulumsRunDefaults();
    options.camera = makeMimicPendulumsCamera();
    options.panels.push_back(createMimicPendulumsPanel(scene));
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "mimic_pendulums: " << e.what() << "\n";
    return 1;
  }
}
