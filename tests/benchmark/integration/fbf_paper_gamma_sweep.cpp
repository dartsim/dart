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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Fixed-gamma CSV sweep helper for the DART-side FBF paper fixtures. This is a
// step-size/report artifact producer, not an installed API and not the paper's
// full Figure 10 house/arch sweep.
//
// Usage:
//   fbf_paper_gamma_sweep [scenario=backspin]
//                         [gamma_csv=nan,0.5,0.1,0.05,0.01,0.005]
//                         [steps=paper_duration]
//                         [max_contacts=scenario_default]
//                         [max_contacts_per_pair=scenario_default]
//                         [max_outer_iterations=scenario_default]
//                         [inner_sweeps=120]
//                         [inner_local_iterations=32]
//                         [projected_gradient_iterations=scenario_default]
//                         [step_size_scale=scenario_default]
//                         [outer_relaxation=scenario_default]
//
// Scenarios include the small fixtures plus one-step contact-rich scaffolds:
//   card_house_26_reduced_contact (full natural manifold, 96 initial contacts
//   in the repaired reconstruction),
//   masonry_arch_25_reduced_contact, masonry_arch_101_reduced_contact.

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

namespace {

constexpr double kDt = 1.0 / 60.0;
constexpr double kGravity = 9.81;
constexpr double kInclineTan = 0.5;
constexpr double kBackspinRadius = 0.25;
constexpr double kBackspinLinearVelocity = 4.0;
constexpr double kBackspinAngularVelocity = -200.0;
constexpr double kBackspinFriction = 0.5;
constexpr double kTurntableInitialRadius = 1.0;
constexpr double kTurntableRampDuration = 1.0;
constexpr double kPainleveInitialPitch = 0.08;
constexpr double kPainleveInitialVelocity = 3.6;
constexpr double kCardHouseFriction = 0.8;
constexpr double kCardHouseAngle = 0.23;
constexpr double kCardHouseHeight = 1.0;
constexpr double kCardHouseWidth = 0.45;
constexpr double kCardHouseThickness = 0.03;
// Source-informed reconstruction from Newton 1.3's 1000 kg/m^3 rigid-shape
// default; not confirmed as an author scene override.
constexpr double kCardHouseDensity = 1000.0;
constexpr double kCardHouseInitialPenetration = 0.003;
// Reconstruction choice: adjacent horizontal one-meter cards meet with the
// same nominal 3 mm overlap used at the other interfaces. The paper does not
// publish this spacing.
constexpr double kCardHouseFrameSpacing
    = kCardHouseHeight - kCardHouseInitialPenetration;
constexpr double kCardHouseStepSizeScale = 10.0;
constexpr double kCardHouseOuterRelaxation = 1.5;
constexpr double kSmallFixtureStepSizeScale = 2.0;
constexpr std::size_t kCardHouseFourLevelCount = 4u;
// Full natural manifold for the repaired 26-card reconstruction: 512/4 caps
// observe 96 contacts in the initial configuration. This is a measured DART
// reconstruction count, not the paper timing row's 214-contact contract.
constexpr std::size_t kCardHouseReducedMaxContacts = 512u;
constexpr std::size_t kCardHouseReducedMaxContactsPerPair = 4u;
// The paper's contact-rich arch experiments explicitly use mu=0.8. The
// credited Rigid-IPC geometry scene uses 0.5, but that source value is not the
// paper experiment's contact contract.
constexpr double kArchFriction = 0.8;
// Rigid-IPC's default body density ("plastic", src/io/read_rb_scene.cpp:68).
constexpr double kArchDensity = 1000.0;
constexpr std::size_t kArchStoneCount = 25u;
constexpr std::size_t kArch101StoneCount = 101u;
constexpr std::size_t kArchReducedMaxContacts = 48u;
constexpr std::size_t kArchReducedMaxContactsPerPair = 2u;
constexpr std::size_t kArch101ReducedMaxContacts = 38u;
constexpr std::size_t kArch101ReducedMaxContactsPerPair = 2u;
constexpr int kArchMaxOuterIterations = 120000;
constexpr double kArchOuterRelaxation = 1.5;
constexpr double kArchStepSizeScale = 10.0;
constexpr double kPi = 3.141592653589793238462643383279502884;

enum class Scenario
{
  InclineStick,
  InclineSlide,
  Backspin,
  TurntableLowSlow,
  TurntableLowFast,
  TurntableHighSlow,
  TurntableHighFast,
  PainleveSlide,
  PainleveTumble,
  CardHouseFourLevelReduced,
  MasonryArch25Reduced,
  MasonryArch101Reduced,
};

struct SweepMetrics
{
  std::size_t contacts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t warmStarts = 0u;
  std::size_t fallbacks = 0u;
  std::size_t residualSamples = 0u;
  std::size_t residualFailures = 0u;
  std::size_t nonFiniteResiduals = 0u;
  double maxResidual = std::numeric_limits<double>::quiet_NaN();
  double finalResidual = std::numeric_limits<double>::quiet_NaN();
  double finalStepSize = std::numeric_limits<double>::quiet_NaN();
  double finalSafeStepSize = std::numeric_limits<double>::quiet_NaN();
  double finalCouplingVariationRatio = std::numeric_limits<double>::quiet_NaN();
  int finalShrinkIterations = 0;
  const char* finalStatus = "not_run";
  const char* physicalOutcome = "unknown";
  double finalX = std::numeric_limits<double>::quiet_NaN();
  double finalPositionY = std::numeric_limits<double>::quiet_NaN();
  double finalZ = std::numeric_limits<double>::quiet_NaN();
  double finalVx = std::numeric_limits<double>::quiet_NaN();
  double finalVy = std::numeric_limits<double>::quiet_NaN();
  double finalVz = std::numeric_limits<double>::quiet_NaN();
  double finalUpZ = std::numeric_limits<double>::quiet_NaN();
  double finalRadius = std::numeric_limits<double>::quiet_NaN();
  double finalAngularY = std::numeric_limits<double>::quiet_NaN();
};

struct SweepConfig
{
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  int maxOuterIterations = 500;
  int innerMaxSweeps = 120;
  int innerLocalIterations = 32;
  int projectedGradientMaxIterations = 400;
  double stepSizeScale = 1.0;
  double outerRelaxation = 1.0;
};

const char* scenarioName(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
      return "incline_mu_0_5";
    case Scenario::InclineSlide:
      return "incline_mu_0_4";
    case Scenario::Backspin:
      return "backspin";
    case Scenario::TurntableLowSlow:
      return "turntable_mu_0_2_omega_2";
    case Scenario::TurntableLowFast:
      return "turntable_mu_0_2_omega_5";
    case Scenario::TurntableHighSlow:
      return "turntable_mu_0_5_omega_2";
    case Scenario::TurntableHighFast:
      return "turntable_mu_0_5_omega_5";
    case Scenario::PainleveSlide:
      return "painleve_mu_0_5";
    case Scenario::PainleveTumble:
      return "painleve_mu_0_55";
    case Scenario::CardHouseFourLevelReduced:
      return "card_house_26_reduced_contact";
    case Scenario::MasonryArch25Reduced:
      return "masonry_arch_25_reduced_contact";
    case Scenario::MasonryArch101Reduced:
      return "masonry_arch_101_reduced_contact";
  }

  return "unknown";
}

const char* bodyNameForScenario(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
      return "incline_cube";
    case Scenario::Backspin:
      return "backspin_sphere";
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
      return "turntable_rider";
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      return "painleve_box";
    case Scenario::CardHouseFourLevelReduced:
      return "card_house_l3_f0_left";
    case Scenario::MasonryArch25Reduced:
      return "masonry_arch_stone_12";
    case Scenario::MasonryArch101Reduced:
      return "masonry_arch_stone_50";
  }

  return "";
}

const char* exactStatusName(
    dart::constraint::ExactCoulombFbfConstraintSolverStatus status)
{
  switch (status) {
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::NotRun:
      return "not_run";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success:
      return "success";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::
        MaxIterationsAccepted:
      return "max_iterations_accepted";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::
        InvalidOptions:
      return "invalid_options";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::
        UnsupportedProblem:
      return "unsupported_problem";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::FbfFailed:
      return "fbf_failed";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::
        BoxedLcpFallback:
      return "boxed_lcp_fallback";
  }

  return "unknown";
}

bool parseScenario(const char* value, Scenario& scenario)
{
  if (value == nullptr || std::string(value) == "backspin") {
    scenario = Scenario::Backspin;
    return true;
  }

  const std::string name(value);
  if (name == "incline_mu_0_5") {
    scenario = Scenario::InclineStick;
  } else if (name == "incline_mu_0_4") {
    scenario = Scenario::InclineSlide;
  } else if (name == "turntable_mu_0_2_omega_2") {
    scenario = Scenario::TurntableLowSlow;
  } else if (name == "turntable_mu_0_2_omega_5") {
    scenario = Scenario::TurntableLowFast;
  } else if (name == "turntable_mu_0_5_omega_2") {
    scenario = Scenario::TurntableHighSlow;
  } else if (name == "turntable_mu_0_5_omega_5") {
    scenario = Scenario::TurntableHighFast;
  } else if (name == "painleve_mu_0_5") {
    scenario = Scenario::PainleveSlide;
  } else if (name == "painleve_mu_0_55") {
    scenario = Scenario::PainleveTumble;
  } else if (name == "card_house_26_reduced_contact") {
    scenario = Scenario::CardHouseFourLevelReduced;
  } else if (name == "masonry_arch_25_reduced_contact") {
    scenario = Scenario::MasonryArch25Reduced;
  } else if (name == "masonry_arch_101_reduced_contact") {
    scenario = Scenario::MasonryArch101Reduced;
  } else {
    return false;
  }

  return true;
}

bool parseSizeArg(const char* value, std::size_t fallback, std::size_t& output)
{
  if (value == nullptr) {
    output = fallback;
    return true;
  }

  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value, &end, 10);
  if (value[0] == '\0' || end == value || *end != '\0' || errno == ERANGE)
    return false;

  output = static_cast<std::size_t>(parsed);
  return true;
}

bool parseIntArg(const char* value, int fallback, int& output)
{
  if (value == nullptr) {
    output = fallback;
    return true;
  }

  errno = 0;
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (value[0] == '\0' || end == value || *end != '\0' || errno == ERANGE)
    return false;

  output = static_cast<int>(parsed);
  return true;
}

bool parseDoubleArg(const char* value, double fallback, double& output)
{
  if (value == nullptr) {
    output = fallback;
    return true;
  }

  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (value[0] == '\0' || end == value || *end != '\0' || errno == ERANGE)
    return false;

  output = parsed;
  return true;
}

bool parseGammaToken(const std::string& token, double& gamma)
{
  if (token == "nan") {
    gamma = std::numeric_limits<double>::quiet_NaN();
    return true;
  }

  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(token.c_str(), &end);
  if (token.empty() || end == token.c_str() || *end != '\0' || errno == ERANGE)
    return false;

  if (!std::isfinite(parsed) || parsed <= 0.0)
    return false;

  gamma = parsed;
  return true;
}

bool parseGammaList(const char* value, std::vector<double>& gammas)
{
  const std::string input
      = value == nullptr ? "nan,0.5,0.1,0.05,0.01,0.005" : value;

  std::stringstream stream(input);
  std::string token;
  while (std::getline(stream, token, ',')) {
    double gamma = std::numeric_limits<double>::quiet_NaN();
    if (!parseGammaToken(token, gamma))
      return false;

    gammas.push_back(gamma);
  }

  return !gammas.empty();
}

double scenarioFriction(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
      return 0.5;
    case Scenario::InclineSlide:
      return 0.4;
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
      return 0.2;
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
    case Scenario::Backspin:
    case Scenario::PainleveSlide:
      return 0.5;
    case Scenario::PainleveTumble:
      return 0.55;
    case Scenario::CardHouseFourLevelReduced:
      return kCardHouseFriction;
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch101Reduced:
      return kArchFriction;
  }

  return 0.5;
}

double turntableAngularVelocity(Scenario scenario)
{
  switch (scenario) {
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableHighSlow:
      return 2.0;
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighFast:
      return 5.0;
    default:
      return 0.0;
  }
}

std::size_t defaultScenarioSteps(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
      return static_cast<std::size_t>(std::round(2.0 / kDt));
    case Scenario::Backspin:
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
      return static_cast<std::size_t>(std::round(4.0 / kDt));
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      return static_cast<std::size_t>(std::round(2.5 / kDt));
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch101Reduced:
      return 1u;
  }

  return 0u;
}

SweepConfig defaultSweepConfig(Scenario scenario)
{
  SweepConfig config;

  switch (scenario) {
    case Scenario::Backspin:
      config.maxContacts = 1u;
      config.maxContactsPerPair = 1u;
      return config;
    case Scenario::CardHouseFourLevelReduced:
      config.maxContacts = kCardHouseReducedMaxContacts;
      config.maxContactsPerPair = kCardHouseReducedMaxContactsPerPair;
      config.maxOuterIterations = 30000;
      config.projectedGradientMaxIterations = 200;
      config.stepSizeScale = kCardHouseStepSizeScale;
      config.outerRelaxation = kCardHouseOuterRelaxation;
      return config;
    case Scenario::MasonryArch25Reduced:
      config.maxContacts = kArchReducedMaxContacts;
      config.maxContactsPerPair = kArchReducedMaxContactsPerPair;
      config.maxOuterIterations = kArchMaxOuterIterations;
      config.stepSizeScale = kArchStepSizeScale;
      config.outerRelaxation = kArchOuterRelaxation;
      return config;
    case Scenario::MasonryArch101Reduced:
      config.maxContacts = kArch101ReducedMaxContacts;
      config.maxContactsPerPair = kArch101ReducedMaxContactsPerPair;
      config.maxOuterIterations = kArchMaxOuterIterations;
      config.stepSizeScale = kArchStepSizeScale;
      config.outerRelaxation = kArchOuterRelaxation;
      return config;
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      config.maxContacts = 4u;
      config.maxContactsPerPair = 4u;
      config.stepSizeScale = kSmallFixtureStepSizeScale;
      return config;
  }

  config.maxContacts = 4u;
  config.maxContactsPerPair = 4u;
  return config;
}

bool usesCardHouseBudget(Scenario scenario)
{
  return scenario == Scenario::CardHouseFourLevelReduced;
}

bool usesArchBudget(Scenario scenario)
{
  return scenario == Scenario::MasonryArch25Reduced
         || scenario == Scenario::MasonryArch101Reduced;
}

bool isTurntableScenario(Scenario scenario)
{
  return scenario == Scenario::TurntableLowSlow
         || scenario == Scenario::TurntableLowFast
         || scenario == Scenario::TurntableHighSlow
         || scenario == Scenario::TurntableHighFast;
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeFbfOptions(
    double initialStepSize,
    const SweepConfig& config,
    bool cardBudget = false,
    bool archBudget = false)
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = config.maxOuterIterations;
  options.tolerance = 1e-6;
  options.innerMaxSweeps = config.innerMaxSweeps;
  options.innerLocalIterations = config.innerLocalIterations;
  options.projectedGradientMaxIterations
      = config.projectedGradientMaxIterations;
  options.initialStepSize = initialStepSize;
  const bool usesFixedGamma = std::isfinite(initialStepSize);
  options.capInitialStepSizeAtSafeBound = !usesFixedGamma;
  options.enableAdaptiveStepSize = !usesFixedGamma;
  options.enableWarmStart = !(cardBudget || archBudget);
  options.stepSizeScale = config.stepSizeScale;
  options.outerRelaxation = config.outerRelaxation;
  return options;
}

void configureWorldSolver(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    double initialStepSize,
    const SweepConfig& config,
    bool cardBudget = false,
    bool archBudget = false)
{
  world->setTimeStep(kDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto solver
      = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
          makeFbfOptions(initialStepSize, config, cardBudget, archBudget));
  solver->setCollisionDetector(
      dart::collision::DARTCollisionDetector::create());
  solver->setNumSimulationThreads(1u);
  world->setConstraintSolver(std::move(solver));

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = maxContacts;
  collisionOption.maxNumContactsPerPair = maxContactsPerPair;
}

dart::dynamics::SkeletonPtr createHorizontalPlane(double frictionCoeff)
{
  auto skeleton = dart::dynamics::Skeleton::create("ground_plane");
  auto body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                  .second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createInclinePlane(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d normal(-std::sin(theta), 0.0, std::cos(theta));

  auto skeleton = dart::dynamics::Skeleton::create("incline_plane");
  auto body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                  .second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(normal, 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createInclineCube(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d normal(-std::sin(theta), 0.0, std::cos(theta));
  const Eigen::Vector3d size = Eigen::Vector3d::Ones();
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.01;

  auto skeleton = dart::dynamics::Skeleton::create("incline_cube");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("incline_cube_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("incline_cube_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
  transform.translation() = normal * (0.5 * size.z() - kInitialPenetration);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createBackspinSphere()
{
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("backspin_sphere");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("backspin_sphere_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("backspin_sphere_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::SphereShape::computeInertia(kBackspinRadius, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::SphereShape>(kBackspinRadius));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kBackspinFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = kBackspinRadius - kInitialPenetration;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kBackspinLinearVelocity, 0.0, 0.0));
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, kBackspinAngularVelocity, 0.0));
  return skeleton;
}

dart::dynamics::SkeletonPtr createTurntableSupport(double frictionCoeff)
{
  constexpr double kThickness = 0.1;

  auto skeleton = dart::dynamics::Skeleton::create("turntable");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("turntable_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("turntable_body"));
  bodyProperties.mInertia.setMass(1.0);

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(4.0, 4.0, kThickness)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.5 * kThickness;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createTurntableRider(double frictionCoeff)
{
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(0.25);
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("turntable_rider");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("turntable_rider_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("turntable_rider_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      kTurntableInitialRadius, 0.0, 0.5 * size.z() - kInitialPenetration);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createPainleveBox(double frictionCoeff)
{
  const Eigen::Vector3d size(0.6, 0.6, 1.0);
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("painleve_box");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("painleve_box_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("painleve_box_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(kPainleveInitialPitch, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double verticalHalfExtent
      = 0.5
        * (std::abs(transform.linear()(2, 0)) * size.x()
           + std::abs(transform.linear()(2, 1)) * size.y()
           + std::abs(transform.linear()(2, 2)) * size.z());
  transform.translation().z() = verticalHalfExtent - kInitialPenetration;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kPainleveInitialVelocity, 0.0, 0.0));
  return skeleton;
}

double computeCardHouseVerticalHalfExtent(const Eigen::Matrix3d& rotation)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  return 0.5
         * (std::abs(rotation(2, 0)) * size.x()
            + std::abs(rotation(2, 1)) * size.y()
            + std::abs(rotation(2, 2)) * size.z());
}

Eigen::Isometry3d createCardHouseAFrameTransform(
    double centerX, double baseZ, bool leftCard)
{
  const double angle = leftCard ? kCardHouseAngle : -kCardHouseAngle;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const double horizontalHalfExtent
      = 0.5
        * (kCardHouseHeight * std::sin(kCardHouseAngle)
           + kCardHouseThickness * std::cos(kCardHouseAngle));
  const double centerOffset
      = horizontalHalfExtent - 0.5 * kCardHouseInitialPenetration;
  transform.translation().x()
      = centerX + (leftCard ? -centerOffset : centerOffset);
  transform.translation().y() = 0.0;
  transform.translation().z()
      = baseZ + computeCardHouseVerticalHalfExtent(transform.linear())
        - kCardHouseInitialPenetration;
  return transform;
}

Eigen::Isometry3d createCardHouseHorizontalSupportTransform(
    double centerX, double baseZ)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY())
                           .toRotationMatrix();
  transform.translation().x() = centerX;
  transform.translation().y() = 0.0;
  transform.translation().z()
      = baseZ + 0.5 * kCardHouseThickness - kCardHouseInitialPenetration;
  return transform;
}

dart::dynamics::SkeletonPtr createCardHousePlate(
    const std::string& name, const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  const double mass = kCardHouseDensity * size.x() * size.y() * size.z();

  auto skeleton = dart::dynamics::Skeleton::create(name);
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(name + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(name + "_body"));
  bodyProperties.mInertia.setMass(mass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, mass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kCardHouseFriction);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

void addReducedCardHouse(const std::shared_ptr<dart::simulation::World>& world)
{
  const Eigen::Matrix3d aFrameRotation
      = Eigen::AngleAxisd(kCardHouseAngle, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double aFrameHeight
      = 2.0 * computeCardHouseVerticalHalfExtent(aFrameRotation);
  double baseZ = 0.0;

  for (std::size_t level = 0u; level < kCardHouseFourLevelCount; ++level) {
    const std::size_t frameCount = kCardHouseFourLevelCount - level;
    for (std::size_t frame = 0u; frame < frameCount; ++frame) {
      const double centerX
          = (static_cast<double>(frame) - 0.5 * (frameCount - 1))
            * kCardHouseFrameSpacing;
      world->addSkeleton(createCardHousePlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_left",
          createCardHouseAFrameTransform(centerX, baseZ, true)));
      world->addSkeleton(createCardHousePlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_right",
          createCardHouseAFrameTransform(centerX, baseZ, false)));
    }

    if (level + 1u == kCardHouseFourLevelCount) {
      break;
    }

    const double supportBaseZ
        = baseZ + aFrameHeight - kCardHouseInitialPenetration;
    for (std::size_t support = 0u; support + 1u < frameCount; ++support) {
      const double centerX
          = (static_cast<double>(support) + 0.5 - 0.5 * (frameCount - 1))
            * kCardHouseFrameSpacing;
      world->addSkeleton(createCardHousePlate(
          "card_house_l" + std::to_string(level) + "_support"
              + std::to_string(support),
          createCardHouseHorizontalSupportTransform(centerX, supportBaseZ)));
    }

    baseZ = supportBaseZ + kCardHouseThickness - kCardHouseInitialPenetration;
  }
}

dart::dynamics::SkeletonPtr createMasonryArchStone(
    std::size_t index,
    const dart::math::detail::MasonryArchStoneBoxGeometry& geometry)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "masonry_arch_stone_" + std::to_string(index));
  auto* joint
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr)
            .first;
  auto* body = skeleton->getBodyNode(0);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(geometry.size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);
  joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(geometry.transform));

  // Mobility is assigned by the paper-scene builder below.
  const double mass = kArchDensity * geometry.size.x() * geometry.size.y()
                      * geometry.size.z();
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(geometry.size, mass));
  body->setInertia(inertia);
  return skeleton;
}

void addReducedMasonryArch(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t stoneCount)
{
  // Paper Fig. 7 pins the 25-stone springers. Fig. 8 calls the 101-stone case
  // the same setup, so both endpoints are pinned at either scale.
  world->addSkeleton(createHorizontalPlane(kArchFriction));

  const auto stoneGeometry
      = dart::math::detail::generateMasonryArchStoneBoxes(stoneCount);
  for (std::size_t i = 0u; i < stoneCount; ++i) {
    auto stone = createMasonryArchStone(i, stoneGeometry[i]);
    if (i == 0u || i + 1u == stoneCount)
      stone->setMobile(false);
    world->addSkeleton(stone);
  }
}

std::shared_ptr<dart::simulation::World> createSweepWorld(
    Scenario scenario, double initialStepSize, const SweepConfig& config)
{
  const double frictionCoeff = scenarioFriction(scenario);
  const bool cardBudget = usesCardHouseBudget(scenario);
  const bool archBudget = usesArchBudget(scenario);

  auto world = dart::simulation::World::create(
      std::string("exact_coulomb_gamma_sweep_") + scenarioName(scenario));
  configureWorldSolver(
      world,
      config.maxContacts,
      config.maxContactsPerPair,
      initialStepSize,
      config,
      cardBudget,
      archBudget);

  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
      world->addSkeleton(createInclinePlane(frictionCoeff));
      world->addSkeleton(createInclineCube(frictionCoeff));
      break;
    case Scenario::Backspin:
      world->addSkeleton(createHorizontalPlane(kBackspinFriction));
      world->addSkeleton(createBackspinSphere());
      break;
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
      world->addSkeleton(createTurntableSupport(frictionCoeff));
      world->addSkeleton(createTurntableRider(frictionCoeff));
      break;
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      world->addSkeleton(createHorizontalPlane(frictionCoeff));
      world->addSkeleton(createPainleveBox(frictionCoeff));
      break;
    case Scenario::CardHouseFourLevelReduced:
      world->addSkeleton(createHorizontalPlane(frictionCoeff));
      addReducedCardHouse(world);
      break;
    case Scenario::MasonryArch25Reduced:
      addReducedMasonryArch(world, kArchStoneCount);
      break;
    case Scenario::MasonryArch101Reduced:
      addReducedMasonryArch(world, kArch101StoneCount);
      break;
  }

  return world;
}

dart::constraint::ExactCoulombFbfConstraintSolver* getExactSolver(
    const std::shared_ptr<dart::simulation::World>& world)
{
  return dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
      world->getConstraintSolver());
}

void applyScenarioControl(
    const std::shared_ptr<dart::simulation::World>& world,
    Scenario scenario,
    std::size_t step)
{
  if (!isTurntableScenario(scenario))
    return;

  auto turntable = world->getSkeleton("turntable");
  if (!turntable)
    return;

  auto* joint = static_cast<dart::dynamics::FreeJoint*>(turntable->getJoint(0));
  const double time = static_cast<double>(step) * kDt;
  const double ramp = std::min(time / kTurntableRampDuration, 1.0);
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, 0.0, ramp * turntableAngularVelocity(scenario)));
}

void sampleResidual(
    const dart::constraint::ExactCoulombFbfConstraintSolver* solver,
    std::size_t& previousExactSolves,
    SweepMetrics& metrics)
{
  const std::size_t exactSolves = solver->getNumExactCoulombSolves();
  if (exactSolves == previousExactSolves)
    return;

  previousExactSolves = exactSolves;
  ++metrics.residualSamples;

  if (solver->getLastExactCoulombStatus()
          != dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success
      || solver->getLastExactCoulombFbfStatus()
             != dart::math::detail::ExactCoulombFbfStatus::Success) {
    ++metrics.residualFailures;
  }

  const double residual = solver->getLastExactCoulombResidual();
  if (std::isfinite(residual)) {
    metrics.maxResidual = std::isfinite(metrics.maxResidual)
                              ? std::max(metrics.maxResidual, residual)
                              : residual;
  } else {
    ++metrics.nonFiniteResiduals;
  }
}

const char* classifyPhysicalOutcome(
    Scenario scenario, const dart::dynamics::BodyNode* body)
{
  const Eigen::Isometry3d transform = body->getWorldTransform();
  const Eigen::Vector3d position = transform.translation();
  const Eigen::Vector3d linearVelocity = body->getLinearVelocity();
  const double radius = std::hypot(position.x(), position.y());
  const double upZ
      = transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ());

  switch (scenario) {
    case Scenario::InclineStick: {
      const double theta = std::atan(kInclineTan);
      const Eigen::Vector3d downhill(-std::cos(theta), 0.0, -std::sin(theta));
      return std::abs(linearVelocity.dot(downhill)) < 0.2 ? "stick_like"
                                                          : "slide_like";
    }
    case Scenario::InclineSlide: {
      const double theta = std::atan(kInclineTan);
      const Eigen::Vector3d downhill(-std::cos(theta), 0.0, -std::sin(theta));
      return linearVelocity.dot(downhill) > 0.2 ? "slide_like" : "stick_like";
    }
    case Scenario::Backspin:
      return linearVelocity.x() < -1.0 ? "reversed" : "not_reversed";
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighFast:
      return radius > 1.5 ? "ejected" : "captured";
    case Scenario::TurntableHighSlow:
      return radius < 1.25 ? "captured" : "ejected";
    case Scenario::PainleveSlide:
      return upZ > 0.8 ? "upright" : "tumbled";
    case Scenario::PainleveTumble:
      return upZ < 0.7 ? "tumbled" : "upright";
    case Scenario::CardHouseFourLevelReduced:
      return position.allFinite() && linearVelocity.allFinite()
                 ? "reduced_one_step_finite"
                 : "nonfinite";
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch101Reduced:
      return position.allFinite() && linearVelocity.allFinite()
                 ? "reduced_one_step_supported"
                 : "nonfinite";
  }

  return "unknown";
}

double safeValue(double value)
{
  return std::isfinite(value) ? value
                              : std::numeric_limits<double>::quiet_NaN();
}

SweepMetrics runSweepRow(
    Scenario scenario,
    double gamma,
    std::size_t steps,
    const SweepConfig& config)
{
  auto world = createSweepWorld(scenario, gamma, config);
  auto* solver = getExactSolver(world);
  auto skeleton = world->getSkeleton(bodyNameForScenario(scenario));
  if (solver == nullptr || !skeleton || skeleton->getNumBodyNodes() == 0u)
    return SweepMetrics();

  std::size_t previousExactSolves = 0u;
  SweepMetrics metrics;
  for (std::size_t step = 0; step < steps; ++step) {
    applyScenarioControl(world, scenario, step);
    world->step();
    sampleResidual(solver, previousExactSolves, metrics);
  }

  const auto* body = skeleton->getBodyNode(0);
  const Eigen::Isometry3d transform = body->getWorldTransform();
  const Eigen::Vector3d position = transform.translation();
  const Eigen::Vector3d linearVelocity = body->getLinearVelocity();
  const Eigen::Vector3d angularVelocity = body->getAngularVelocity();
  metrics.contacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  metrics.exactSolves = solver->getNumExactCoulombSolves();
  metrics.warmStarts = solver->getNumExactCoulombWarmStarts();
  metrics.fallbacks = solver->getNumBoxedLcpFallbacks();
  metrics.finalResidual = solver->getLastExactCoulombResidual();
  metrics.finalStepSize = solver->getLastExactCoulombStepSize();
  metrics.finalSafeStepSize = solver->getLastExactCoulombSafeStepSize();
  metrics.finalCouplingVariationRatio
      = solver->getLastExactCoulombCouplingVariationRatio();
  metrics.finalShrinkIterations = solver->getLastExactCoulombShrinkIterations();
  metrics.finalStatus = exactStatusName(solver->getLastExactCoulombStatus());
  metrics.physicalOutcome = classifyPhysicalOutcome(scenario, body);
  metrics.finalX = position.x();
  metrics.finalPositionY = position.y();
  metrics.finalZ = position.z();
  metrics.finalVx = linearVelocity.x();
  metrics.finalVy = linearVelocity.y();
  metrics.finalVz = linearVelocity.z();
  metrics.finalUpZ
      = transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ());
  metrics.finalRadius = std::hypot(position.x(), position.y());
  metrics.finalAngularY = angularVelocity.y();
  return metrics;
}

bool isCleanExactRun(const SweepMetrics& metrics)
{
  return metrics.exactSolves > 0u && metrics.fallbacks == 0u
         && metrics.residualSamples > 0u && metrics.residualFailures == 0u
         && metrics.nonFiniteResiduals == 0u
         && std::isfinite(metrics.finalResidual)
         && metrics.finalResidual <= 1e-6;
}

const char* gammaLabel(double gamma)
{
  return std::isnan(gamma) ? "safe" : "fixed";
}

void printHeader()
{
  std::printf(
      "scenario,gamma_mode,gamma,steps,max_contacts,max_contacts_per_pair,"
      "adaptive_step_size,cap_at_safe_bound,final_gamma,safe_gamma,"
      "shrink_iterations,coupling_variation_ratio,"
      "max_outer_iterations,inner_sweeps,inner_local_iterations,"
      "projected_gradient_iterations,step_size_scale,outer_relaxation,"
      "clean_exact,physical_outcome,"
      "contacts,exact_solves,warm_starts,fallbacks,residual_samples,"
      "residual_failures,nonfinite_residuals,max_residual,final_residual,"
      "final_status,x,y,z,vx,vy,vz,up_z,radius,omega_y\n");
}

void printRow(
    Scenario scenario,
    double gamma,
    std::size_t steps,
    const SweepConfig& config)
{
  const SweepMetrics metrics = runSweepRow(scenario, gamma, steps, config);
  const bool usesFixedGamma = std::isfinite(gamma);
  std::printf(
      "%s,%s,%.17g,%zu,%zu,%zu,%d,%d,%.17g,%.17g,%d,%.17g,"
      "%d,%d,%d,%d,%.17g,%.17g,%d,%s,%zu,%zu,%zu,"
      "%zu,%zu,%zu,%zu,%.17g,"
      "%.17g,%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
      scenarioName(scenario),
      gammaLabel(gamma),
      safeValue(gamma),
      steps,
      config.maxContacts,
      config.maxContactsPerPair,
      usesFixedGamma ? 0 : 1,
      usesFixedGamma ? 0 : 1,
      safeValue(metrics.finalStepSize),
      safeValue(metrics.finalSafeStepSize),
      metrics.finalShrinkIterations,
      safeValue(metrics.finalCouplingVariationRatio),
      config.maxOuterIterations,
      config.innerMaxSweeps,
      config.innerLocalIterations,
      config.projectedGradientMaxIterations,
      safeValue(config.stepSizeScale),
      safeValue(config.outerRelaxation),
      isCleanExactRun(metrics) ? 1 : 0,
      metrics.physicalOutcome,
      metrics.contacts,
      metrics.exactSolves,
      metrics.warmStarts,
      metrics.fallbacks,
      metrics.residualSamples,
      metrics.residualFailures,
      metrics.nonFiniteResiduals,
      safeValue(metrics.maxResidual),
      safeValue(metrics.finalResidual),
      metrics.finalStatus,
      safeValue(metrics.finalX),
      safeValue(metrics.finalPositionY),
      safeValue(metrics.finalZ),
      safeValue(metrics.finalVx),
      safeValue(metrics.finalVy),
      safeValue(metrics.finalVz),
      safeValue(metrics.finalUpZ),
      safeValue(metrics.finalRadius),
      safeValue(metrics.finalAngularY));
}

void printUsage()
{
  std::fprintf(
      stderr,
      "Usage: fbf_paper_gamma_sweep [scenario=backspin] "
      "[gamma_csv=nan,0.5,0.1,0.05,0.01,0.005] "
      "[steps=paper_duration] "
      "[max_contacts=scenario_default] "
      "[max_contacts_per_pair=scenario_default] "
      "[max_outer_iterations=scenario_default] "
      "[inner_sweeps=120] "
      "[inner_local_iterations=32] "
      "[projected_gradient_iterations=scenario_default] "
      "[step_size_scale=scenario_default] "
      "[outer_relaxation=scenario_default]\n"
      "Scenarios include card_house_26_reduced_contact (full natural "
      "manifold, 96 initial contacts in the repaired reconstruction), "
      "masonry_arch_25_reduced_contact, and "
      "masonry_arch_101_reduced_contact as one-step scaffolds. "
      "The cap arguments are for reproducing reduced or boundary sweep rows; "
      "they do not make the reduced scaffolds full paper parity.\n");
}

} // namespace

int main(int argc, char** argv)
{
  Scenario scenario = Scenario::Backspin;
  if (!parseScenario(argc > 1 ? argv[1] : nullptr, scenario)) {
    printUsage();
    return 2;
  }

  std::vector<double> gammas;
  if (!parseGammaList(argc > 2 ? argv[2] : nullptr, gammas)) {
    printUsage();
    return 2;
  }

  std::size_t steps = defaultScenarioSteps(scenario);
  if (!parseSizeArg(argc > 3 ? argv[3] : nullptr, steps, steps)) {
    printUsage();
    return 2;
  }

  SweepConfig config = defaultSweepConfig(scenario);
  if (!parseSizeArg(
          argc > 4 ? argv[4] : nullptr, config.maxContacts, config.maxContacts)
      || !parseSizeArg(
          argc > 5 ? argv[5] : nullptr,
          config.maxContactsPerPair,
          config.maxContactsPerPair)
      || !parseIntArg(
          argc > 6 ? argv[6] : nullptr,
          config.maxOuterIterations,
          config.maxOuterIterations)
      || !parseIntArg(
          argc > 7 ? argv[7] : nullptr,
          config.innerMaxSweeps,
          config.innerMaxSweeps)
      || !parseIntArg(
          argc > 8 ? argv[8] : nullptr,
          config.innerLocalIterations,
          config.innerLocalIterations)
      || !parseIntArg(
          argc > 9 ? argv[9] : nullptr,
          config.projectedGradientMaxIterations,
          config.projectedGradientMaxIterations)
      || !parseDoubleArg(
          argc > 10 ? argv[10] : nullptr,
          config.stepSizeScale,
          config.stepSizeScale)
      || !parseDoubleArg(
          argc > 11 ? argv[11] : nullptr,
          config.outerRelaxation,
          config.outerRelaxation)
      || config.maxContacts == 0u || config.maxContactsPerPair == 0u
      || config.maxOuterIterations <= 0 || config.innerMaxSweeps < 0
      || config.innerLocalIterations < 0
      || config.projectedGradientMaxIterations < 0
      || !std::isfinite(config.stepSizeScale) || config.stepSizeScale <= 0.0
      || !std::isfinite(config.outerRelaxation)
      || config.outerRelaxation <= 0.0) {
    printUsage();
    return 2;
  }

  printHeader();
  for (const double gamma : gammas) {
    printRow(scenario, gamma, steps, config);
  }

  return 0;
}
