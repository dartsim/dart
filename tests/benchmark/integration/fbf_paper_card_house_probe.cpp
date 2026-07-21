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
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 *   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// CSV diagnostic helper for the reduced-contact four-level card-house rung from
// the exact-Coulomb FBF paper task. This is a benchmark/report artifact
// producer, not an installed API and not a core DART dependency.
//
// Usage:
//   fbf_paper_card_house_probe [max_contacts_csv=38,64]
//                              [max_contacts_per_pair=1]
//                              [max_outer_iterations=30000]
//                              [inner_sweeps=120]
//                              [inner_local_iterations=32]
//                              [projected_gradient_iterations=200]
//                              [initial_gamma=nan]
//                              [include_constraint_regularization=0]
//                              [inner_diagonal_regularization=0]
//                              [coupling_variation_tolerance=0.9]
//                              [step_size_scale=10]
//                              [outer_relaxation=1.5]
//                              [max_residual_history_samples=0]
//                              [initial_penetration=0.003]
//                              [frame_spacing=0.997]
//                              [use_matrix_free_delassus_operator=0]
//                              [use_matrix_free_delassus_seed=0]

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
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
constexpr double kPi = 3.141592653589793238462643383279502884;

struct ProbeOptions
{
  std::size_t maxContactsPerPair = 1u;
  int maxOuterIterations = 30000;
  int innerSweeps = 120;
  int innerLocalIterations = 32;
  int projectedGradientIterations = 200;
  double initialGamma = std::numeric_limits<double>::quiet_NaN();
  bool includeConstraintRegularization = false;
  double innerDiagonalRegularization = 0.0;
  double couplingVariationTolerance = 0.9;
  double stepSizeScale = kCardHouseStepSizeScale;
  double outerRelaxation = kCardHouseOuterRelaxation;
  int maxResidualHistorySamples = 0;
  double initialPenetration = kCardHouseInitialPenetration;
  double frameSpacing = kCardHouseFrameSpacing;
  bool useMatrixFreeDelassusOperator = false;
  bool useMatrixFreeDelassusSeed = false;
};

struct ProbeResult
{
  std::size_t cardCount = 0u;
  std::size_t contacts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t exactFailures = 0u;
  std::size_t fallbacks = 0u;
  std::size_t warmStarts = 0u;
  std::size_t projectedGradientRetries = 0u;
  std::size_t denseResidualPolishes = 0u;
  double elapsedMs = std::numeric_limits<double>::quiet_NaN();
  double residual = std::numeric_limits<double>::quiet_NaN();
  double primalResidual = std::numeric_limits<double>::quiet_NaN();
  double dualResidual = std::numeric_limits<double>::quiet_NaN();
  double complementarityResidual = std::numeric_limits<double>::quiet_NaN();
  int worstPrimalContact = -1;
  int worstDualContact = -1;
  int worstComplementarityContact = -1;
  double failedResidual = std::numeric_limits<double>::quiet_NaN();
  double failedPrimalResidual = std::numeric_limits<double>::quiet_NaN();
  double failedDualResidual = std::numeric_limits<double>::quiet_NaN();
  double failedComplementarityResidual
      = std::numeric_limits<double>::quiet_NaN();
  int failedWorstPrimalContact = -1;
  int failedWorstDualContact = -1;
  int failedWorstComplementarityContact = -1;
  std::string failedWorstPrimalPair = "none";
  std::string failedWorstDualPair = "none";
  std::string failedWorstComplementarityPair = "none";
  double minCardAxisUp = std::numeric_limits<double>::infinity();
  double minCenterHeight = std::numeric_limits<double>::infinity();
  double maxHorizontalTravel = 0.0;
  const char* exactStatus = "not_run";
  const char* fbfStatus = "invalid_input";
  const char* failedFbfStatus = "invalid_input";
  int fbfIterations = 0;
  int bestIteration = 0;
  int failedFbfIterations = 0;
  int failedBestIteration = 0;
  int maxFbfIterations = 0;
  std::size_t totalFbfIterations = 0u;
  int shrinkIterations = 0;
  int failedShrinkIterations = 0;
  double bestResidual = std::numeric_limits<double>::quiet_NaN();
  double stepSize = std::numeric_limits<double>::quiet_NaN();
  double safeStepSize = std::numeric_limits<double>::quiet_NaN();
  double couplingVariationRatio = std::numeric_limits<double>::quiet_NaN();
  double failedBestResidual = std::numeric_limits<double>::quiet_NaN();
  double failedStepSize = std::numeric_limits<double>::quiet_NaN();
  double failedSafeStepSize = std::numeric_limits<double>::quiet_NaN();
  double failedCouplingVariationRatio
      = std::numeric_limits<double>::quiet_NaN();
  std::size_t residualHistoryRecords = 0u;
  std::size_t residualHistoryRows = 0u;
  std::size_t failedResidualHistoryRows = 0u;
  int failedResidualHistoryLastIteration = 0;
  double failedResidualHistoryFirst = std::numeric_limits<double>::quiet_NaN();
  double failedResidualHistoryLast = std::numeric_limits<double>::quiet_NaN();
  double failedResidualHistoryTailRatio
      = std::numeric_limits<double>::quiet_NaN();
  bool matrixFreeDelassusOperatorUsed = false;
  bool matrixFreeDelassusSeedUsed = false;
  bool exactSuccess = false;
  bool finiteState = true;
};

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
        PlateauAccepted:
      return "plateau_accepted";
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

const char* fbfStatusName(dart::math::detail::ExactCoulombFbfStatus status)
{
  switch (status) {
    case dart::math::detail::ExactCoulombFbfStatus::Success:
      return "success";
    case dart::math::detail::ExactCoulombFbfStatus::MaxIterations:
      return "max_iterations";
    case dart::math::detail::ExactCoulombFbfStatus::InvalidInput:
      return "invalid_input";
    case dart::math::detail::ExactCoulombFbfStatus::InnerSolverFailed:
      return "inner_solver_failed";
    case dart::math::detail::ExactCoulombFbfStatus::StepSizeUnderflow:
      return "step_size_underflow";
    case dart::math::detail::ExactCoulombFbfStatus::Plateau:
      return "plateau";
    case dart::math::detail::ExactCoulombFbfStatus::NonFiniteValue:
      return "non_finite_value";
  }

  return "unknown";
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

bool parseGammaArg(const char* value, double fallback, double& output)
{
  if (value == nullptr) {
    output = fallback;
    return true;
  }

  if (std::string(value) == "nan") {
    output = std::numeric_limits<double>::quiet_NaN();
    return true;
  }

  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (value[0] == '\0' || end == value || *end != '\0' || errno == ERANGE)
    return false;

  if (!std::isfinite(parsed) || parsed <= 0.0)
    return false;

  output = parsed;
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
  if (value[0] == '\0' || end == value || *end != '\0' || errno == ERANGE
      || !std::isfinite(parsed)) {
    return false;
  }

  output = parsed;
  return true;
}

bool parseBoolArg(const char* value, bool fallback, bool& output)
{
  if (value == nullptr) {
    output = fallback;
    return true;
  }

  const std::string parsed(value);
  if (parsed == "1" || parsed == "true") {
    output = true;
    return true;
  }
  if (parsed == "0" || parsed == "false") {
    output = false;
    return true;
  }

  return false;
}

bool parseSizeToken(const std::string& token, std::size_t& output)
{
  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(token.c_str(), &end, 10);
  if (token.empty() || end == token.c_str() || *end != '\0'
      || errno == ERANGE) {
    return false;
  }

  output = static_cast<std::size_t>(parsed);
  return true;
}

bool parseSizeList(const char* value, std::vector<std::size_t>& values)
{
  const std::string input = value == nullptr ? "38,64" : value;
  std::stringstream stream(input);
  std::string token;
  while (std::getline(stream, token, ',')) {
    std::size_t parsed = 0u;
    if (!parseSizeToken(token, parsed))
      return false;
    values.push_back(parsed);
  }

  return !values.empty();
}

double safeValue(double value)
{
  return std::isfinite(value) ? value
                              : std::numeric_limits<double>::quiet_NaN();
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeFbfOptions(
    const ProbeOptions& probeOptions)
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = probeOptions.maxOuterIterations;
  options.tolerance = 1e-6;
  options.innerMaxSweeps = probeOptions.innerSweeps;
  options.innerLocalIterations = probeOptions.innerLocalIterations;
  options.projectedGradientMaxIterations
      = probeOptions.projectedGradientIterations;
  options.initialStepSize = probeOptions.initialGamma;
  options.includeConstraintRegularization
      = probeOptions.includeConstraintRegularization;
  options.innerDiagonalRegularization
      = probeOptions.innerDiagonalRegularization;
  options.couplingVariationTolerance = probeOptions.couplingVariationTolerance;
  options.stepSizeScale = probeOptions.stepSizeScale;
  options.outerRelaxation = probeOptions.outerRelaxation;
  options.useMatrixFreeDelassusOperator
      = probeOptions.useMatrixFreeDelassusOperator;
  // Requesting the legacy impulse-test product route must actually exercise
  // it, so the scratch-backed contact-row operator is disabled for that row.
  options.useContactRowDelassusOperator
      = !probeOptions.useMatrixFreeDelassusOperator;
  options.useMatrixFreeDelassusSeed = probeOptions.useMatrixFreeDelassusSeed;
  options.enableWarmStart = false;
  options.maxResidualHistorySamples = probeOptions.maxResidualHistorySamples;
  options.maxResidualHistoryRecords
      = probeOptions.maxResidualHistorySamples > 0 ? 16 : 0;
  return options;
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

dart::dynamics::SkeletonPtr createCardPlate(
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
    double centerX, double baseZ, bool leftCard, double initialPenetration)
{
  const double angle = leftCard ? kCardHouseAngle : -kCardHouseAngle;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();

  // Position the two oriented-box A-frame cards by their horizontal half
  // extents so their bounding faces overlap by the requested amount. The old
  // 0.96 * halfSpan heuristic produced about 37 mm of penetration for 30 mm
  // cards while reporting a 3 mm target.
  const double horizontalHalfExtent
      = 0.5
        * (kCardHouseHeight * std::sin(kCardHouseAngle)
           + kCardHouseThickness * std::cos(kCardHouseAngle));
  const double centerOffset = horizontalHalfExtent - 0.5 * initialPenetration;
  transform.translation().x()
      = centerX + (leftCard ? -centerOffset : centerOffset);
  transform.translation().y() = 0.0;
  const double verticalHalfExtent
      = computeCardHouseVerticalHalfExtent(transform.linear());
  transform.translation().z() = baseZ + verticalHalfExtent - initialPenetration;
  return transform;
}

Eigen::Isometry3d createCardHouseHorizontalSupportTransform(
    double centerX, double baseZ, double initialPenetration)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY())
                           .toRotationMatrix();
  transform.translation().x() = centerX;
  transform.translation().y() = 0.0;
  transform.translation().z()
      = baseZ + 0.5 * kCardHouseThickness - initialPenetration;
  return transform;
}

bool isCardHouseSkeletonName(const std::string& name)
{
  return name.rfind("card_house_", 0u) == 0u;
}

std::shared_ptr<dart::simulation::World> createCardHouseWorld(
    std::size_t maxContacts, const ProbeOptions& probeOptions)
{
  auto world
      = dart::simulation::World::create("exact_coulomb_card_house_probe");
  world->setTimeStep(kDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto solver
      = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
          makeFbfOptions(probeOptions));
  solver->setCollisionDetector(
      dart::collision::DARTCollisionDetector::create());
  solver->setNumSimulationThreads(1u);
  world->setConstraintSolver(std::move(solver));

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = maxContacts;
  collisionOption.maxNumContactsPerPair = probeOptions.maxContactsPerPair;

  world->addSkeleton(createHorizontalPlane(kCardHouseFriction));

  const Eigen::Matrix3d aFrameRotation
      = Eigen::AngleAxisd(kCardHouseAngle, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double aFrameHeight
      = 2.0 * computeCardHouseVerticalHalfExtent(aFrameRotation);
  double baseZ = 0.0;

  for (int level = 0; level < 4; ++level) {
    const int frameCount = 4 - level;
    for (int frame = 0; frame < frameCount; ++frame) {
      const double centerX
          = (static_cast<double>(frame) - 0.5 * (frameCount - 1))
            * probeOptions.frameSpacing;
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_left",
          createCardHouseAFrameTransform(
              centerX, baseZ, true, probeOptions.initialPenetration)));
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_right",
          createCardHouseAFrameTransform(
              centerX, baseZ, false, probeOptions.initialPenetration)));
    }

    if (level == 3)
      break;

    const double supportBaseZ
        = baseZ + aFrameHeight - probeOptions.initialPenetration;
    for (int support = 0; support < frameCount - 1; ++support) {
      const double centerX
          = (static_cast<double>(support) + 0.5 - 0.5 * (frameCount - 1))
            * probeOptions.frameSpacing;
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_support"
              + std::to_string(support),
          createCardHouseHorizontalSupportTransform(
              centerX, supportBaseZ, probeOptions.initialPenetration)));
    }

    baseZ
        = supportBaseZ + kCardHouseThickness - probeOptions.initialPenetration;
  }

  return world;
}

dart::constraint::ExactCoulombFbfConstraintSolver* getExactSolver(
    const std::shared_ptr<dart::simulation::World>& world)
{
  return dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
      world->getConstraintSolver());
}

std::string bodyNodeName(const dart::dynamics::BodyNode* bodyNode)
{
  return bodyNode != nullptr ? bodyNode->getName() : "null";
}

std::string contactPairLabel(
    const dart::collision::CollisionResult& collisionResult, int contactIndex)
{
  if (contactIndex < 0
      || static_cast<std::size_t>(contactIndex)
             >= collisionResult.getNumContacts()) {
    return "none";
  }

  const auto& contact
      = collisionResult.getContact(static_cast<std::size_t>(contactIndex));
  return bodyNodeName(contact.getBodyNodePtr1()) + "|"
         + bodyNodeName(contact.getBodyNodePtr2());
}

ProbeResult runProbe(std::size_t maxContacts, const ProbeOptions& options)
{
  auto world = createCardHouseWorld(maxContacts, options);
  auto* solver = getExactSolver(world);

  ProbeResult result;
  if (solver == nullptr)
    return result;

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton != nullptr && isCardHouseSkeletonName(skeleton->getName()))
      ++result.cardCount;
  }

  const auto start = std::chrono::steady_clock::now();
  world->step();
  const auto stop = std::chrono::steady_clock::now();
  result.elapsedMs
      = std::chrono::duration<double, std::milli>(stop - start).count();

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr || !isCardHouseSkeletonName(skeleton->getName()))
      continue;

    const auto* body = skeleton->getBodyNode(0);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    result.finiteState = result.finiteState && position.allFinite()
                         && body->getLinearVelocity().allFinite()
                         && body->getAngularVelocity().allFinite();
    result.minCardAxisUp = std::min(
        result.minCardAxisUp,
        transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ()));
    result.minCenterHeight = std::min(result.minCenterHeight, position.z());
    result.maxHorizontalTravel
        = std::max(result.maxHorizontalTravel, position.head<2>().norm());
  }

  result.contacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  result.exactSolves = solver->getNumExactCoulombSolves();
  result.exactFailures = solver->getNumExactCoulombFailures();
  result.fallbacks = solver->getNumBoxedLcpFallbacks();
  result.warmStarts = solver->getNumExactCoulombWarmStarts();
  result.projectedGradientRetries
      = solver->getNumExactCoulombProjectedGradientRetries();
  result.denseResidualPolishes
      = solver->getNumExactCoulombDenseResidualPolishes();
  result.matrixFreeDelassusOperatorUsed
      = solver->getLastExactCoulombMatrixFreeDelassusOperatorUsed();
  result.matrixFreeDelassusSeedUsed
      = solver->getLastExactCoulombMatrixFreeDelassusSeedUsed();
  result.exactStatus = exactStatusName(solver->getLastExactCoulombStatus());
  result.fbfStatus = fbfStatusName(solver->getLastExactCoulombFbfStatus());
  result.fbfIterations = solver->getLastExactCoulombIterations();
  result.bestIteration = solver->getLastExactCoulombBestIteration();
  result.maxFbfIterations = solver->getMaxExactCoulombIterations();
  result.totalFbfIterations = solver->getTotalExactCoulombIterations();
  result.bestResidual = solver->getLastExactCoulombBestResidual();
  result.stepSize = solver->getLastExactCoulombStepSize();
  result.safeStepSize = solver->getLastExactCoulombSafeStepSize();
  result.couplingVariationRatio
      = solver->getLastExactCoulombCouplingVariationRatio();
  result.shrinkIterations = solver->getLastExactCoulombShrinkIterations();
  result.residual = solver->getLastExactCoulombResidual();
  const auto& residual = solver->getLastExactCoulombResidualDetails();
  result.primalResidual = residual.primalFeasibility;
  result.dualResidual = residual.dualFeasibility;
  result.complementarityResidual = residual.complementarity;
  result.worstPrimalContact = static_cast<int>(residual.worstPrimalContact);
  result.worstDualContact = static_cast<int>(residual.worstDualContact);
  result.worstComplementarityContact
      = static_cast<int>(residual.worstComplementarityContact);
  result.failedFbfStatus
      = fbfStatusName(solver->getLastFailedExactCoulombFbfStatus());
  result.failedFbfIterations = solver->getLastFailedExactCoulombIterations();
  result.failedBestIteration = solver->getLastFailedExactCoulombBestIteration();
  result.failedBestResidual = solver->getLastFailedExactCoulombBestResidual();
  result.failedStepSize = solver->getLastFailedExactCoulombStepSize();
  result.failedSafeStepSize = solver->getLastFailedExactCoulombSafeStepSize();
  result.failedCouplingVariationRatio
      = solver->getLastFailedExactCoulombCouplingVariationRatio();
  result.failedShrinkIterations
      = solver->getLastFailedExactCoulombShrinkIterations();
  result.failedResidual = solver->getLastFailedExactCoulombResidual();
  const auto& failedResidual
      = solver->getLastFailedExactCoulombResidualDetails();
  result.failedPrimalResidual = failedResidual.primalFeasibility;
  result.failedDualResidual = failedResidual.dualFeasibility;
  result.failedComplementarityResidual = failedResidual.complementarity;
  result.failedWorstPrimalContact
      = static_cast<int>(failedResidual.worstPrimalContact);
  result.failedWorstDualContact
      = static_cast<int>(failedResidual.worstDualContact);
  result.failedWorstComplementarityContact
      = static_cast<int>(failedResidual.worstComplementarityContact);
  const auto& collisionResult
      = world->getConstraintSolver()->getLastCollisionResult();
  result.failedWorstPrimalPair
      = contactPairLabel(collisionResult, result.failedWorstPrimalContact);
  result.failedWorstDualPair
      = contactPairLabel(collisionResult, result.failedWorstDualContact);
  result.failedWorstComplementarityPair = contactPairLabel(
      collisionResult, result.failedWorstComplementarityContact);

  const auto& residualHistoryRecords
      = solver->getExactCoulombResidualHistoryRecords();
  result.residualHistoryRecords = residualHistoryRecords.size();
  for (const auto& record : residualHistoryRecords) {
    result.residualHistoryRows += record.samples.size();
    if (record.status
            != dart::constraint::ExactCoulombFbfConstraintSolverStatus::
                FbfFailed
        || record.samples.empty()) {
      continue;
    }

    result.failedResidualHistoryRows += record.samples.size();
    result.failedResidualHistoryFirst = record.samples.front().residual.value;
    result.failedResidualHistoryLast = record.samples.back().residual.value;
    result.failedResidualHistoryLastIteration = record.samples.back().iteration;

    if (record.samples.size() >= 2u) {
      const double previous
          = record.samples[record.samples.size() - 2u].residual.value;
      if (std::isfinite(previous) && previous > 0.0) {
        result.failedResidualHistoryTailRatio
            = result.failedResidualHistoryLast / previous;
      }
    }
  }
  result.exactSuccess
      = solver->getLastExactCoulombStatus()
            == dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success
        && solver->getLastExactCoulombFbfStatus()
               == dart::math::detail::ExactCoulombFbfStatus::Success
        && result.exactFailures == 0u && result.fallbacks == 0u
        && std::isfinite(result.residual) && result.residual <= 1e-6;
  return result;
}

const char* gammaLabel(double gamma)
{
  return std::isnan(gamma) ? "safe" : "fixed";
}

void printHeader()
{
  std::printf(
      "scenario,max_contacts,max_contacts_per_pair,max_outer_iterations,"
      "inner_sweeps,inner_local_iterations,projected_gradient_iterations,"
      "gamma_mode,gamma,include_constraint_regularization,"
      "inner_diagonal_regularization,coupling_variation_tolerance,"
      "step_size_scale,outer_relaxation,initial_penetration,frame_spacing,"
      "use_matrix_free_delassus_operator,use_matrix_free_delassus_seed,"
      "contacts,elapsed_ms,exact_success,"
      "card_count,finite_state,exact_status,"
      "fbf_status,fbf_iterations,best_iteration,max_fbf_iterations,"
      "total_fbf_iterations,best_residual,step_size,safe_step_size,"
      "coupling_variation_ratio,shrink_iterations,"
      "residual,primal,dual,complementarity,worst_primal_contact,"
      "worst_dual_contact,worst_complementarity_contact,"
      "exact_solves,exact_failures,"
      "fallbacks,warm_starts,projected_gradient_retries,"
      "dense_residual_polishes,matrix_free_delassus_operator_used,"
      "matrix_free_delassus_seed_used,"
      "failed_fbf_status,"
      "failed_fbf_iterations,failed_best_iteration,failed_best_residual,"
      "failed_step_size,failed_safe_step_size,failed_coupling_variation_ratio,"
      "failed_shrink_iterations,"
      "failed_residual,failed_primal,failed_dual,failed_complementarity,"
      "failed_worst_primal_contact,failed_worst_dual_contact,"
      "failed_worst_complementarity_contact,failed_worst_primal_pair,"
      "failed_worst_dual_pair,failed_worst_complementarity_pair,"
      "residual_history_records,residual_history_rows,"
      "failed_residual_history_rows,failed_residual_history_last_iteration,"
      "failed_residual_history_first,failed_residual_history_last,"
      "failed_residual_history_tail_ratio,"
      "min_card_axis_up,min_center_height,max_horizontal_travel\n");
}

void printRow(std::size_t maxContacts, const ProbeOptions& options)
{
  const ProbeResult result = runProbe(maxContacts, options);
  std::printf(
      "card_house_26,%zu,%zu,%d,%d,%d,%d,%s,%.17g,%d,%.17g,%.17g,%.17g,"
      "%.17g,%.17g,%.17g,%d,%d,%zu,%.17g,%d,%zu,%d,%s,%s,%d,%d,%d,%zu,"
      "%.17g,%.17g,%.17g,"
      "%.17g,%d,%.17g,%.17g,%.17g,%.17g,%d,%d,%d,%zu,%zu,%zu,%zu,%zu,%zu,%d,"
      "%d,%s,%d,%d,%.17g,%.17g,%.17g,%.17g,%d,%.17g,%.17g,%.17g,%.17g,"
      "%d,%d,%d,%s,%s,%s,%zu,%zu,%zu,%d,"
      "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
      maxContacts,
      options.maxContactsPerPair,
      options.maxOuterIterations,
      options.innerSweeps,
      options.innerLocalIterations,
      options.projectedGradientIterations,
      gammaLabel(options.initialGamma),
      safeValue(options.initialGamma),
      options.includeConstraintRegularization ? 1 : 0,
      safeValue(options.innerDiagonalRegularization),
      safeValue(options.couplingVariationTolerance),
      safeValue(options.stepSizeScale),
      safeValue(options.outerRelaxation),
      safeValue(options.initialPenetration),
      safeValue(options.frameSpacing),
      options.useMatrixFreeDelassusOperator ? 1 : 0,
      options.useMatrixFreeDelassusSeed ? 1 : 0,
      result.contacts,
      safeValue(result.elapsedMs),
      result.exactSuccess ? 1 : 0,
      result.cardCount,
      result.finiteState ? 1 : 0,
      result.exactStatus,
      result.fbfStatus,
      result.fbfIterations,
      result.bestIteration,
      result.maxFbfIterations,
      result.totalFbfIterations,
      safeValue(result.bestResidual),
      safeValue(result.stepSize),
      safeValue(result.safeStepSize),
      safeValue(result.couplingVariationRatio),
      result.shrinkIterations,
      safeValue(result.residual),
      safeValue(result.primalResidual),
      safeValue(result.dualResidual),
      safeValue(result.complementarityResidual),
      result.worstPrimalContact,
      result.worstDualContact,
      result.worstComplementarityContact,
      result.exactSolves,
      result.exactFailures,
      result.fallbacks,
      result.warmStarts,
      result.projectedGradientRetries,
      result.denseResidualPolishes,
      result.matrixFreeDelassusOperatorUsed ? 1 : 0,
      result.matrixFreeDelassusSeedUsed ? 1 : 0,
      result.failedFbfStatus,
      result.failedFbfIterations,
      result.failedBestIteration,
      safeValue(result.failedBestResidual),
      safeValue(result.failedStepSize),
      safeValue(result.failedSafeStepSize),
      safeValue(result.failedCouplingVariationRatio),
      result.failedShrinkIterations,
      safeValue(result.failedResidual),
      safeValue(result.failedPrimalResidual),
      safeValue(result.failedDualResidual),
      safeValue(result.failedComplementarityResidual),
      result.failedWorstPrimalContact,
      result.failedWorstDualContact,
      result.failedWorstComplementarityContact,
      result.failedWorstPrimalPair.c_str(),
      result.failedWorstDualPair.c_str(),
      result.failedWorstComplementarityPair.c_str(),
      result.residualHistoryRecords,
      result.residualHistoryRows,
      result.failedResidualHistoryRows,
      result.failedResidualHistoryLastIteration,
      safeValue(result.failedResidualHistoryFirst),
      safeValue(result.failedResidualHistoryLast),
      safeValue(result.failedResidualHistoryTailRatio),
      safeValue(result.minCardAxisUp),
      safeValue(result.minCenterHeight),
      safeValue(result.maxHorizontalTravel));
}

void printUsage()
{
  std::fprintf(
      stderr,
      "Usage: fbf_paper_card_house_probe [max_contacts_csv=38,64] "
      "[max_contacts_per_pair=1] [max_outer_iterations=30000] "
      "[inner_sweeps=120] [inner_local_iterations=32] "
      "[projected_gradient_iterations=200] [initial_gamma=nan] "
      "[include_constraint_regularization=0] "
      "[inner_diagonal_regularization=0] "
      "[coupling_variation_tolerance=0.9] [step_size_scale=10] "
      "[outer_relaxation=1.5] [max_residual_history_samples=0] "
      "[initial_penetration=0.003] [frame_spacing=0.997] "
      "[use_matrix_free_delassus_operator=0] "
      "[use_matrix_free_delassus_seed=0]\n");
}

} // namespace

int main(int argc, char** argv)
{
  std::vector<std::size_t> maxContactValues;
  if (!parseSizeList(argc > 1 ? argv[1] : nullptr, maxContactValues)) {
    printUsage();
    return 2;
  }

  ProbeOptions options;
  if (!parseSizeArg(
          argc > 2 ? argv[2] : nullptr,
          options.maxContactsPerPair,
          options.maxContactsPerPair)
      || !parseIntArg(
          argc > 3 ? argv[3] : nullptr,
          options.maxOuterIterations,
          options.maxOuterIterations)
      || !parseIntArg(
          argc > 4 ? argv[4] : nullptr,
          options.innerSweeps,
          options.innerSweeps)
      || !parseIntArg(
          argc > 5 ? argv[5] : nullptr,
          options.innerLocalIterations,
          options.innerLocalIterations)
      || !parseIntArg(
          argc > 6 ? argv[6] : nullptr,
          options.projectedGradientIterations,
          options.projectedGradientIterations)
      || !parseGammaArg(
          argc > 7 ? argv[7] : nullptr,
          options.initialGamma,
          options.initialGamma)
      || !parseBoolArg(
          argc > 8 ? argv[8] : nullptr,
          options.includeConstraintRegularization,
          options.includeConstraintRegularization)
      || !parseDoubleArg(
          argc > 9 ? argv[9] : nullptr,
          options.innerDiagonalRegularization,
          options.innerDiagonalRegularization)
      || !parseDoubleArg(
          argc > 10 ? argv[10] : nullptr,
          options.couplingVariationTolerance,
          options.couplingVariationTolerance)
      || !parseDoubleArg(
          argc > 11 ? argv[11] : nullptr,
          options.stepSizeScale,
          options.stepSizeScale)
      || options.stepSizeScale <= 0.0
      || !parseDoubleArg(
          argc > 12 ? argv[12] : nullptr,
          options.outerRelaxation,
          options.outerRelaxation)
      || options.outerRelaxation <= 0.0
      || !parseIntArg(
          argc > 13 ? argv[13] : nullptr,
          options.maxResidualHistorySamples,
          options.maxResidualHistorySamples)
      || options.maxResidualHistorySamples < 0
      || !parseDoubleArg(
          argc > 14 ? argv[14] : nullptr,
          options.initialPenetration,
          options.initialPenetration)
      || options.initialPenetration < 0.0
      || !parseDoubleArg(
          argc > 15 ? argv[15] : nullptr,
          options.frameSpacing,
          options.frameSpacing)
      || options.frameSpacing <= 0.0
      || !parseBoolArg(
          argc > 16 ? argv[16] : nullptr,
          options.useMatrixFreeDelassusOperator,
          options.useMatrixFreeDelassusOperator)
      || !parseBoolArg(
          argc > 17 ? argv[17] : nullptr,
          options.useMatrixFreeDelassusSeed,
          options.useMatrixFreeDelassusSeed)) {
    printUsage();
    return 2;
  }

  printHeader();
  for (const auto maxContacts : maxContactValues) {
    printRow(maxContacts, options);
  }

  return 0;
}
