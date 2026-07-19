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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORCARDHOUSESPEC_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORCARDHOUSESPEC_HPP_

#include <dart/simulation/World.hpp>

#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>

#include <Eigen/Geometry>

#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

#ifndef DART_FBF_AUTHOR_CARD_HOUSE_SPEC_SHA256
  #error "FBF author-card-house consumers must bind the shared spec source hash"
#endif

namespace fbf_author_card_house {

inline constexpr const char* kContractSchema
    = "dart.fbf_author_card_house_configuration_contract/v1";
inline constexpr const char* kContractKind = "configuration_only";
inline constexpr const char* kDemoSceneId
    = "fbf_author_card_house_5_construction";
inline constexpr const char* kAuthorRepository
    = "https://github.com/matthcsong/fbf-sca-2026";
inline constexpr const char* kAuthorCommit
    = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0";
inline constexpr const char* kAuthorTree
    = "ffcdafb61adeda2239c8366d054b548b50d26685e";
inline constexpr const char* kAuthorCardHouseBlob
    = "35f33651bc9674a259071ac723e47755504152db";
inline constexpr const char* kAuthorCardHouseRunSha256
    = "18c58c85eaad865aeef480b46e880a52088f266b79c90226f624637221ee36f8";
inline constexpr const char* kAuthorConfigSha256
    = "88f3f9ffd758eccce8496f7897192587a05907109e313c7a86bcf8f9de8cc248";
inline constexpr const char* kAuthorSolverSha256
    = "8ec32aa20bf8d6c1173ed6c7f3735e2926fbb4b5059ee2236e26ad27eb22f941";
inline constexpr const char* kSpecSourceSha256
    = DART_FBF_AUTHOR_CARD_HOUSE_SPEC_SHA256;

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kGravity = 9.81;
inline constexpr double kFriction = 0.8;
inline constexpr double kContactGap = 0.005;
inline constexpr std::size_t kDefaultLevelCount = 5u;
inline constexpr std::size_t kCubeCount = 4u;
inline constexpr double kLeanFromHorizontalDegrees = 65.0;
inline constexpr double kLeanFromVerticalDegrees = 25.0;
inline constexpr double kBridgeAngleDegrees = -1.0;
inline constexpr double kCardHalfLength = 1.25;
inline constexpr double kCardHalfDepth = 0.625;
inline constexpr double kCardHalfThickness = 0.02;
inline constexpr double kCardDensity = 200.0;
inline constexpr double kCardMass = 25.0;
inline constexpr double kTentHalfGap = 0.55;
inline constexpr double kTentWidth = 2.2;
// Evaluated from the author's exact formula:
//   (tan(65 deg) * 2.2 + tan(3 deg) * 2.2) / 2.
inline constexpr double kTentHeight = 2.41660616977186;
inline constexpr double kCubeHalfSize = 0.4;
inline constexpr double kCubeDensity = 500.0;
inline constexpr double kCubeMass = 256.0;
inline constexpr double kDropHeight = 1.0;
inline constexpr double kCubeInitialHeight = 13.0830308488593;
inline constexpr double kDisplayTimeStep = 1.0 / 60.0;
inline constexpr std::size_t kSubstepsPerFrame = 4u;
inline constexpr double kRuntimeTimeStep = 1.0 / 240.0;
inline constexpr std::size_t kReleaseFrame = 400u;
inline constexpr std::size_t kReleaseSubstep = 1600u;
inline constexpr std::size_t kTotalFrames = 800u;
inline constexpr std::size_t kTotalSubsteps = 3200u;

inline constexpr std::size_t kSourceMaxContacts = 4096u;
inline constexpr int kSourceMaxOuterIterations = 200;
inline constexpr double kSourceOuterTolerance = 1e-6;
inline constexpr int kSourceResidualCheckInterval = 5;
inline constexpr const char* kSourceInnerSolver = "block_gs";
inline constexpr int kSourceInnerGaussSeidelSweeps = 10;
inline constexpr int kSourceInnerMaxIterations = 200;
inline constexpr double kSourceInnerTolerance = 1e-6;
inline constexpr double kSourceGammaC = 5.0;
inline constexpr double kSourceGammaMax = 1e6;
inline constexpr double kSourceArmijoRhoHigh = 0.9;
inline constexpr double kSourceArmijoShrink = 0.7;
inline constexpr int kSourceArmijoMaxBacktracks = 8;
inline constexpr int kSourcePlateauPatience = 5;
inline constexpr double kSourcePlateauRelativeTolerance = 0.01;
inline constexpr double kSourceWarmStartMatchRadius = 0.02;
inline constexpr double kSourceWarmStartNormalCosine = 0.9;
inline constexpr int kSourceWarmStartMaxAge = 3;
inline constexpr double kSourceWarmStartGammaCap = 1e4;
inline constexpr double kSourceBaumgarteErp = 0.0;
inline constexpr const char* kSourceTerminationResidual = "coulomb_rel";
inline constexpr double kSourceTerminationTolerance = 1e-6;

inline constexpr bool kTrajectoryEquivalence = false;
inline constexpr bool kSolverEquivalence = false;
inline constexpr bool kPhysicalOutcomeEquivalence = false;
inline constexpr bool kFig06Parity = false;
inline constexpr bool kVideo06Parity = false;
inline constexpr bool kTimingComparability = false;

//==============================================================================
constexpr std::size_t leaningCardCount(std::size_t levelCount)
{
  return levelCount * (levelCount + 1u);
}

//==============================================================================
constexpr std::size_t bridgeCardCount(std::size_t levelCount)
{
  return levelCount == 0u ? 0u : levelCount * (levelCount - 1u) / 2u;
}

//==============================================================================
constexpr std::size_t cardCount(std::size_t levelCount)
{
  return leaningCardCount(levelCount) + bridgeCardCount(levelCount);
}

enum class CardKind
{
  Bridge,
  LeanLeft,
  LeanRight,
};

//==============================================================================
inline const char* cardKindLabel(CardKind kind)
{
  switch (kind) {
    case CardKind::Bridge:
      return "bridge";
    case CardKind::LeanLeft:
      return "tent_left";
    case CardKind::LeanRight:
      return "tent_right";
  }
  return "unknown";
}

struct CardSpec
{
  std::string name;
  CardKind kind = CardKind::Bridge;
  std::size_t level = 0u;
  std::size_t tent = 0u;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

struct CubeSpec
{
  std::string name;
  std::size_t index = 0u;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

//==============================================================================
inline double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

//==============================================================================
inline Eigen::Matrix3d rotationAboutY(double angleDegrees)
{
  return Eigen::AngleAxisd(
             degreesToRadians(angleDegrees), Eigen::Vector3d::UnitY())
      .toRotationMatrix();
}

//==============================================================================
inline std::vector<CardSpec> makeCardSpecs(std::size_t levelCount)
{
  if (levelCount == 0u)
    throw std::invalid_argument(
        "author card house requires at least one level");

  std::vector<CardSpec> specs;
  specs.reserve(cardCount(levelCount));
  for (std::size_t level = 0u; level < levelCount; ++level) {
    const std::size_t tentCount = levelCount - level;
    for (std::size_t tent = 0u; tent < tentCount; ++tent) {
      const double xOffset = (static_cast<double>(tent)
                              - static_cast<double>(tentCount) / 2.0 + 0.5)
                             * kTentWidth;

      if (level != 0u) {
        CardSpec bridge;
        bridge.name
            = "bridge_L" + std::to_string(level) + "_T" + std::to_string(tent);
        bridge.kind = CardKind::Bridge;
        bridge.level = level;
        bridge.tent = tent;
        bridge.size = Eigen::Vector3d(
            2.0 * kCardHalfLength,
            2.0 * kCardHalfDepth,
            2.0 * kCardHalfThickness);
        bridge.transform.linear() = rotationAboutY(kBridgeAngleDegrees);
        bridge.transform.translation()
            = Eigen::Vector3d(xOffset, 0.0, level * kTentHeight);
        specs.push_back(bridge);
      }

      CardSpec left;
      left.name
          = "tent_left_L" + std::to_string(level) + "_T" + std::to_string(tent);
      left.kind = CardKind::LeanLeft;
      left.level = level;
      left.tent = tent;
      left.size = Eigen::Vector3d(
          2.0 * kCardHalfThickness,
          2.0 * kCardHalfDepth,
          2.0 * kCardHalfLength);
      left.transform.linear() = rotationAboutY(kLeanFromVerticalDegrees);
      left.transform.translation() = Eigen::Vector3d(
          xOffset - kTentHalfGap, 0.0, (level + 0.5) * kTentHeight);
      specs.push_back(left);

      CardSpec right = left;
      right.name = "tent_right_L" + std::to_string(level) + "_T"
                   + std::to_string(tent);
      right.kind = CardKind::LeanRight;
      right.transform.linear() = rotationAboutY(-kLeanFromVerticalDegrees);
      right.transform.translation().x() = xOffset + kTentHalfGap;
      specs.push_back(right);
    }
  }
  return specs;
}

//==============================================================================
inline std::vector<CubeSpec> makeCubeSpecs(
    std::size_t levelCount = kDefaultLevelCount)
{
  if (levelCount == 0u)
    throw std::invalid_argument(
        "author card house requires at least one level");

  std::vector<CubeSpec> specs;
  specs.reserve(kCubeCount);
  const double topZ = levelCount * kTentHeight + kDropHeight;
  const double spacing = 3.0 * kCubeHalfSize;
  for (std::size_t index = 0u; index < kCubeCount; ++index) {
    CubeSpec cube;
    cube.name = "cube_" + std::to_string(index);
    cube.index = index;
    cube.size = Eigen::Vector3d::Constant(2.0 * kCubeHalfSize);
    cube.transform.translation() = Eigen::Vector3d(
        (static_cast<double>(index) - 0.5 * (kCubeCount - 1u)) * spacing,
        0.0,
        topZ);
    specs.push_back(cube);
  }
  return specs;
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfConstraintSolverOptions
dartConstructionSolverOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.useContactRowDelassusOperator = true;
  options.assembleDenseContactRowSnapshot = false;
  options.enableWarmStart = true;
  options.enableStepSizePersistence = true;
  options.stepSizeRecoveryGrowthFactor = 1.0 / kSourceArmijoShrink;
  options.warmStartMatchDistance = kSourceWarmStartMatchRadius;
  options.useMatrixFreeDelassusSeed = true;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.maxOuterIterations = kSourceMaxOuterIterations;
  options.tolerance = kSourceOuterTolerance;
  // DART's conservative spectral factor is 0.5. A scale of ten represents
  // the author's gamma_c=5 configuration, subject to DART's own line search.
  options.stepSizeScale = 2.0 * kSourceGammaC;
  options.couplingVariationTolerance = kSourceArmijoRhoHigh;
  options.shrinkFactor = kSourceArmijoShrink;
  options.maxStepShrinkIterations = kSourceArmijoMaxBacktracks;
  options.enableAdaptiveStepSize = true;
  options.innerMaxSweeps = kSourceInnerGaussSeidelSweeps;
  options.runFixedInnerSweeps = true;
  options.acceptInnerMaxIterations = true;
  options.innerTolerance = kSourceInnerTolerance;
  return options;
}

struct BodyContract
{
  std::string name;
  std::string role;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double friction = 0.0;
  double mass = 0.0;
  bool mobile = true;
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
};

struct ConfigurationContract
{
  std::string binaryRole;
  std::string binarySourceSha256;
  std::size_t levelCount = 0u;
  double timeStep = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::size_t simulationThreads = 0u;
  bool deactivationEnabled = true;
  std::string collisionDetector;
  std::string contactManifold;
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  bool splitImpulseEnabled = false;
  dart::constraint::ExactCoulombFbfConstraintSolverOptions solverOptions;
  std::string groundShape;
  double groundFriction = 0.0;
  bool groundMobile = true;
  std::vector<BodyContract> cards;
  std::vector<BodyContract> cubes;
};

//==============================================================================
inline BodyContract inspectBox(
    const std::shared_ptr<dart::simulation::World>& world,
    const std::string& name,
    const std::string& role)
{
  const auto skeleton = world->getSkeleton(name);
  if (!skeleton || skeleton->getNumBodyNodes() != 1u)
    throw std::runtime_error("author card-house body is missing: " + name);

  const auto* body = skeleton->getBodyNode(0u);
  const auto* node
      = body->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (node == nullptr || node->getDynamicsAspect() == nullptr)
    throw std::runtime_error(
        "author card-house collision shape is missing: " + name);

  const auto shape = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
      node->getShape());
  if (!shape)
    throw std::runtime_error("author card-house body is not a box: " + name);

  BodyContract contract;
  contract.name = name;
  contract.role = role;
  contract.size = shape->getSize();
  contract.pose = body->getWorldTransform();
  contract.friction = node->getDynamicsAspect()->getFrictionCoeff();
  contract.mass = body->getInertia().getMass();
  contract.mobile = skeleton->isMobile();
  contract.linearVelocity = body->getLinearVelocity();
  contract.angularVelocity = body->getAngularVelocity();
  return contract;
}

//==============================================================================
inline ConfigurationContract inspectConfigurationContract(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t levelCount,
    const std::string& binaryRole,
    const std::string& binarySourceSha256)
{
  if (!world)
    throw std::runtime_error("author card-house contract has no world");

  auto* solver
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  if (solver == nullptr)
    throw std::runtime_error("author card-house contract requires exact FBF");

  const auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          solver->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author card-house contract requires Native collision");

  const auto ground = world->getSkeleton("ground_plane");
  if (!ground || ground->getNumBodyNodes() != 1u)
    throw std::runtime_error("author card-house ground plane is missing");
  const auto* groundNode
      = ground->getBodyNode(0u)
            ->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (groundNode == nullptr || groundNode->getDynamicsAspect() == nullptr
      || !std::dynamic_pointer_cast<const dart::dynamics::PlaneShape>(
          groundNode->getShape())) {
    throw std::runtime_error("author card-house ground is not a plane");
  }

  const auto cardSpecs = makeCardSpecs(levelCount);
  const auto cubeSpecs = makeCubeSpecs(levelCount);
  if (world->getNumSkeletons() != 1u + cardSpecs.size() + cubeSpecs.size()) {
    throw std::runtime_error(
        "author card-house contract has an unexpected skeleton count");
  }

  ConfigurationContract contract;
  contract.binaryRole = binaryRole;
  contract.binarySourceSha256 = binarySourceSha256;
  contract.levelCount = levelCount;
  contract.timeStep = world->getTimeStep();
  contract.gravity = world->getGravity();
  contract.simulationThreads = world->getNumSimulationThreads();
  contract.deactivationEnabled = world->getDeactivationOptions().mEnabled;
  contract.collisionDetector = detector->getType();
  contract.contactManifold
      = detector->getContactManifoldMode()
                == dart::collision::NativeCollisionDetector::
                    ContactManifoldMode::FourPointPlanar
            ? "four_point_planar"
            : "compact";
  const auto& collisionOption = solver->getCollisionOption();
  contract.maxContacts = collisionOption.maxNumContacts;
  contract.maxContactsPerPair = collisionOption.maxNumContactsPerPair;
  contract.splitImpulseEnabled = solver->isSplitImpulseEnabled();
  contract.solverOptions = solver->getExactCoulombOptions();
  contract.groundShape = "plane";
  contract.groundFriction = groundNode->getDynamicsAspect()->getFrictionCoeff();
  contract.groundMobile = ground->isMobile();
  contract.cards.reserve(cardSpecs.size());
  for (const auto& spec : cardSpecs)
    contract.cards.push_back(
        inspectBox(world, spec.name, cardKindLabel(spec.kind)));
  contract.cubes.reserve(cubeSpecs.size());
  for (const auto& spec : cubeSpecs)
    contract.cubes.push_back(inspectBox(world, spec.name, "cube"));
  return contract;
}

//==============================================================================
inline void writeJsonString(std::ostream& out, const std::string& value)
{
  out << '"';
  for (const char c : value) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  out << '"';
}

//==============================================================================
inline void writeJsonVector(std::ostream& out, const Eigen::Vector3d& value)
{
  out << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

//==============================================================================
inline void writeJsonMatrix(std::ostream& out, const Eigen::Matrix3d& value)
{
  out << '[';
  for (int row = 0; row < 3; ++row) {
    if (row != 0)
      out << ',';
    out << '[';
    for (int column = 0; column < 3; ++column) {
      if (column != 0)
        out << ',';
      out << value(row, column);
    }
    out << ']';
  }
  out << ']';
}

//==============================================================================
inline void writeJsonPose(std::ostream& out, const Eigen::Isometry3d& value)
{
  out << "{\"translation\":";
  writeJsonVector(out, value.translation());
  out << ",\"rotation\":";
  writeJsonMatrix(out, value.linear());
  out << '}';
}

//==============================================================================
inline void writeJsonBodies(
    std::ostream& out, const std::vector<BodyContract>& bodies)
{
  out << '[';
  for (std::size_t index = 0u; index < bodies.size(); ++index) {
    if (index != 0u)
      out << ',';
    const auto& body = bodies[index];
    out << "{\"name\":";
    writeJsonString(out, body.name);
    out << ",\"role\":";
    writeJsonString(out, body.role);
    out << ",\"size_m\":";
    writeJsonVector(out, body.size);
    out << ",\"mass_kg\":" << body.mass << ",\"friction\":" << body.friction
        << ",\"mobile\":" << (body.mobile ? "true" : "false")
        << ",\"initial_pose\":";
    writeJsonPose(out, body.pose);
    out << ",\"initial_linear_velocity_m_s\":";
    writeJsonVector(out, body.linearVelocity);
    out << ",\"initial_angular_velocity_rad_s\":";
    writeJsonVector(out, body.angularVelocity);
    out << '}';
  }
  out << ']';
}

//==============================================================================
inline std::string configurationContractJson(
    const ConfigurationContract& contract)
{
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema_version\":";
  writeJsonString(out, kContractSchema);
  out << ",\"kind\":";
  writeJsonString(out, kContractKind);
  out << ",\"author_source\":{\"repository\":";
  writeJsonString(out, kAuthorRepository);
  out << ",\"commit\":";
  writeJsonString(out, kAuthorCommit);
  out << ",\"tree\":";
  writeJsonString(out, kAuthorTree);
  out << ",\"card_house_run_blob\":";
  writeJsonString(out, kAuthorCardHouseBlob);
  out << ",\"card_house_run_py_sha256\":";
  writeJsonString(out, kAuthorCardHouseRunSha256);
  out << ",\"fbf_config_py_sha256\":";
  writeJsonString(out, kAuthorConfigSha256);
  out << ",\"solver_fbf_py_sha256\":";
  writeJsonString(out, kAuthorSolverSha256);
  out << "},\"configuration_spec_source_sha256\":";
  writeJsonString(out, kSpecSourceSha256);
  out << ",\"binary_binding\":{\"role\":";
  writeJsonString(out, contract.binaryRole);
  out << ",\"implementation_source_sha256\":";
  writeJsonString(out, contract.binarySourceSha256);
  out << "},\"source_configuration\":{\"scene\":{\"demo_scene_id\":";
  writeJsonString(out, kDemoSceneId);
  out << ",\"levels\":" << contract.levelCount
      << ",\"leaning_cards\":" << leaningCardCount(contract.levelCount)
      << ",\"bridges\":" << bridgeCardCount(contract.levelCount)
      << ",\"cards\":" << cardCount(contract.levelCount)
      << ",\"cubes\":" << kCubeCount << "},\"cards\":{\"lean_size_m\":["
      << 2.0 * kCardHalfThickness << ',' << 2.0 * kCardHalfDepth << ','
      << 2.0 * kCardHalfLength << "],\"bridge_size_m\":["
      << 2.0 * kCardHalfLength << ',' << 2.0 * kCardHalfDepth << ','
      << 2.0 * kCardHalfThickness << "],\"density_kg_m3\":" << kCardDensity
      << ",\"mass_kg\":" << kCardMass
      << ",\"lean_from_horizontal_degrees\":" << kLeanFromHorizontalDegrees
      << ",\"lean_from_vertical_degrees\":" << kLeanFromVerticalDegrees
      << ",\"bridge_angle_degrees\":" << kBridgeAngleDegrees
      << ",\"tent_half_gap_m\":" << kTentHalfGap
      << ",\"tent_width_m\":" << kTentWidth
      << ",\"tent_height_m\":" << kTentHeight
      << ",\"source_mobile\":true},\"cubes\":{\"size_m\":["
      << 2.0 * kCubeHalfSize << ',' << 2.0 * kCubeHalfSize << ','
      << 2.0 * kCubeHalfSize << "],\"density_kg_m3\":" << kCubeDensity
      << ",\"mass_kg\":" << kCubeMass << ",\"initial_height_m\":"
      << contract.levelCount * kTentHeight + kDropHeight
      << ",\"source_initially_kinematic\":true}"
      << ",\"contact\":{\"friction\":" << kFriction
      << ",\"gap_m\":" << kContactGap
      << "},\"schedule\":{\"display_time_step_seconds\":" << kDisplayTimeStep
      << ",\"substeps_per_frame\":" << kSubstepsPerFrame
      << ",\"runtime_time_step_seconds\":" << kRuntimeTimeStep
      << ",\"release_frame\":" << kReleaseFrame
      << ",\"release_substep\":" << kReleaseSubstep
      << ",\"total_frames\":" << kTotalFrames
      << ",\"total_substeps\":" << kTotalSubsteps
      << "},\"solver\":{\"type\":\"fbf_exact_coulomb\""
      << ",\"max_contacts\":" << kSourceMaxContacts
      << ",\"max_outer\":" << kSourceMaxOuterIterations
      << ",\"outer_tol\":" << kSourceOuterTolerance
      << ",\"residual_check_interval\":" << kSourceResidualCheckInterval
      << ",\"inner_solver\":";
  writeJsonString(out, kSourceInnerSolver);
  out << ",\"inner_gs_sweeps\":" << kSourceInnerGaussSeidelSweeps
      << ",\"inner_max_iter\":" << kSourceInnerMaxIterations
      << ",\"inner_tol\":" << kSourceInnerTolerance
      << ",\"gamma\":null,\"adaptive_gamma\":true,\"gamma_c\":" << kSourceGammaC
      << ",\"gamma_max\":" << kSourceGammaMax
      << ",\"armijo_rho_high\":" << kSourceArmijoRhoHigh
      << ",\"armijo_shrink\":" << kSourceArmijoShrink
      << ",\"armijo_max_backtracks\":" << kSourceArmijoMaxBacktracks
      << ",\"plateau_patience\":" << kSourcePlateauPatience
      << ",\"plateau_rtol\":" << kSourcePlateauRelativeTolerance
      << ",\"warm_start\":true,\"warm_start_match_radius\":"
      << kSourceWarmStartMatchRadius
      << ",\"warm_start_normal_cosine\":" << kSourceWarmStartNormalCosine
      << ",\"warm_start_max_age\":" << kSourceWarmStartMaxAge
      << ",\"warm_start_gamma_cap\":" << kSourceWarmStartGammaCap
      << ",\"baumgarte_erp\":" << kSourceBaumgarteErp
      << ",\"project_after_correction\":false"
      << ",\"termination_residual\":";
  writeJsonString(out, kSourceTerminationResidual);
  out << ",\"termination_tol\":" << kSourceTerminationTolerance
      << "}},\"dart_observation\":{\"world\":{\"time_step_seconds\":"
      << contract.timeStep << ",\"gravity_m_s2\":";
  writeJsonVector(out, contract.gravity);
  out << ",\"simulation_threads\":" << contract.simulationThreads
      << ",\"deactivation_enabled\":"
      << (contract.deactivationEnabled ? "true" : "false")
      << "},\"collision\":{\"detector\":";
  writeJsonString(out, contract.collisionDetector);
  out << ",\"contact_manifold\":";
  writeJsonString(out, contract.contactManifold);
  out << ",\"max_contacts\":" << contract.maxContacts
      << ",\"max_contacts_per_pair\":" << contract.maxContactsPerPair
      << "},\"solver_adapter\":{\"type\":\"exact_fbf\""
      << ",\"split_impulse_enabled\":"
      << (contract.splitImpulseEnabled ? "true" : "false")
      << ",\"max_outer_iterations\":"
      << contract.solverOptions.maxOuterIterations
      << ",\"tolerance\":" << contract.solverOptions.tolerance
      << ",\"inner_max_sweeps\":" << contract.solverOptions.innerMaxSweeps
      << ",\"inner_tolerance\":" << contract.solverOptions.innerTolerance
      << ",\"step_size_scale\":" << contract.solverOptions.stepSizeScale
      << ",\"warm_start_enabled\":"
      << (contract.solverOptions.enableWarmStart ? "true" : "false")
      << ",\"fallback_to_boxed_lcp_enabled\":"
      << (contract.solverOptions.fallbackToBoxedLcp ? "true" : "false")
      << ",\"projected_gradient_retry_enabled\":"
      << (contract.solverOptions.enableProjectedGradientRetry ? "true"
                                                              : "false")
      << ",\"dense_residual_polish_enabled\":"
      << (contract.solverOptions.enableDenseResidualPolish ? "true" : "false")
      << "},\"ground\":{\"shape\":";
  writeJsonString(out, contract.groundShape);
  out << ",\"friction\":" << contract.groundFriction
      << ",\"mobile\":" << (contract.groundMobile ? "true" : "false")
      << "},\"cards\":";
  writeJsonBodies(out, contract.cards);
  out << ",\"cubes\":";
  writeJsonBodies(out, contract.cubes);
  out << "},\"adapter_boundaries\":{"
      << "\"source_contact_gap_recorded_m\":" << kContactGap
      << ",\"source_contact_gap_semantics_implemented\":false"
      << ",\"source_solver_backend_semantics_implemented\":false}"
      << ",\"claim_boundary\":{\"construction_only\":true"
      << ",\"configuration_port_valid\":true"
      << ",\"source_geometry_and_initial_pose_port\":true"
      << ",\"dart_cards_mobile\":true"
      << ",\"dart_cubes_initially_immobile\":true"
      << ",\"dynamics_executed\":false"
      << ",\"trajectory_valid\":false"
      << ",\"solver_valid\":false"
      << ",\"physical_outcome_valid\":false"
      << ",\"trajectory_equivalence\":"
      << (kTrajectoryEquivalence ? "true" : "false")
      << ",\"solver_equivalence\":" << (kSolverEquivalence ? "true" : "false")
      << ",\"physical_outcome_equivalence\":"
      << (kPhysicalOutcomeEquivalence ? "true" : "false")
      << ",\"fig06_parity\":" << (kFig06Parity ? "true" : "false")
      << ",\"video06_parity\":" << (kVideo06Parity ? "true" : "false")
      << ",\"timing_comparability\":"
      << (kTimingComparability ? "true" : "false")
      << ",\"paper_timing_valid\":false"
      << ",\"renderer_colors_source_parity\":false}"
      << ",\"visual_style\":{\"renderer_only\":true"
      << ",\"description\":\"restrained alternating paper colors for layer "
         "legibility\"}}";
  return out.str();
}

} // namespace fbf_author_card_house

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORCARDHOUSESPEC_HPP_
