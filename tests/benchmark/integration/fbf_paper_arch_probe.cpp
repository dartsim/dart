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

// CSV diagnostic helper for the reduced-contact masonry-arch rungs from the
// exact-Coulomb FBF paper task. This is a benchmark/report artifact producer,
// not an installed API and not a core DART dependency.
//
// Usage:
//   fbf_paper_arch_probe [scenario=both]
//                        [max_contacts_csv=default]
//                        [max_contacts_per_pair=2]
//                        [max_outer_iterations=20000]
//                        [inner_sweeps=120]
//                        [inner_local_iterations=32]
//                        [projected_gradient_iterations=400]
//                        [initial_gamma=nan]
//                        [include_constraint_regularization=0]
//                        [inner_diagonal_regularization=0]
//                        [coupling_variation_tolerance=0.9]
//                        [step_size_scale=10]
//                        [outer_relaxation=1]
//                        [max_residual_history_samples=0]
//                        [initial_overlap_scale=1.0]
//                        [use_matrix_free_delassus_operator=0]
//                        [use_matrix_free_delassus_seed=0]

#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/math/detail/MasonryArchGeometry.hpp>
#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 1.0 / 60.0;
constexpr double kGravity = 9.81;
// Author-faithful masonry-arch friction coefficient: Rigid-IPC's
// arch-{25,101}-stones.json both specify coefficient_friction = 0.5
// uniformly (see docs/dev_tasks/fbf_exact_coulomb_friction/ PROVENANCE.txt
// section 7). DART's other paper fixtures use 0.8; the ported arch keeps
// the source's own coefficient for scientific fidelity.
constexpr double kArchFriction = 0.5;
// Rigid-IPC's default body density ("plastic", src/io/read_rb_scene.cpp:68).
constexpr double kArchDensity = 1000.0;
constexpr std::size_t kArch25StoneCount = 25u;
constexpr std::size_t kArch101StoneCount = 101u;
// Along-arch (chord-length) scale applied to every stone's box, on top of
// the author-faithful weighted-catenary geometry; a research probing knob
// (formerly needed to force overlap under the old made-up circular-arc
// model -- the ported author geometry needs no artificial overlap, so the
// default is now 1.0, i.e. the unscaled author geometry).
constexpr double kArchOverlapScaleDefault = 1.0;
constexpr std::size_t kArch25ReducedMaxContacts = 48u;
constexpr std::size_t kArch101ReducedMaxContacts = 38u;
constexpr std::size_t kArchReducedMaxContactsPerPair = 2u;
constexpr int kArchMaxOuterIterations = 20000;
constexpr double kArchStepSizeScale = 10.0;
constexpr double kPi = 3.141592653589793238462643383279502884;

enum class Scenario
{
  Arch25,
  Arch101,
};

struct ProbeOptions
{
  std::size_t maxContactsPerPair = kArchReducedMaxContactsPerPair;
  int maxOuterIterations = kArchMaxOuterIterations;
  int innerSweeps = 120;
  int innerLocalIterations = 32;
  int projectedGradientIterations = 400;
  double initialGamma = std::numeric_limits<double>::quiet_NaN();
  bool includeConstraintRegularization = false;
  double innerDiagonalRegularization = 0.0;
  double couplingVariationTolerance = 0.9;
  double stepSizeScale = kArchStepSizeScale;
  double outerRelaxation = 1.0;
  int maxResidualHistorySamples = 0;
  double initialOverlapScale = kArchOverlapScaleDefault;
  bool useMatrixFreeDelassusOperator = false;
  bool useMatrixFreeDelassusSeed = false;
};

struct ProbeResult
{
  std::size_t stoneCount = 0u;
  std::size_t dynamicStoneCount = 0u;
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
  double crownHeight = std::numeric_limits<double>::quiet_NaN();
  double minCenterHeight = std::numeric_limits<double>::infinity();
  double maxHorizontalTravel = 0.0;
  double maxStoneSpeed = 0.0;
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

const char* scenarioName(Scenario scenario)
{
  switch (scenario) {
    case Scenario::Arch25:
      return "masonry_arch_25";
    case Scenario::Arch101:
      return "masonry_arch_101";
  }

  return "unknown";
}

std::size_t scenarioStoneCount(Scenario scenario)
{
  return scenario == Scenario::Arch25 ? kArch25StoneCount
                                      : kArch101StoneCount;
}

std::size_t scenarioDefaultMaxContacts(Scenario scenario)
{
  return scenario == Scenario::Arch25 ? kArch25ReducedMaxContacts
                                      : kArch101ReducedMaxContacts;
}

const char* exactStatusName(
    dart::constraint::ExactCoulombFbfConstraintSolverStatus status)
{
  switch (status) {
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::NotRun:
      return "not_run";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success:
      return "success";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::InvalidOptions:
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
  if (value == nullptr || std::string(value) == "default")
    return true;

  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    std::size_t parsed = 0u;
    if (!parseSizeToken(token, parsed))
      return false;
    values.push_back(parsed);
  }

  return !values.empty();
}

bool parseScenarioToken(
    const std::string& token, std::vector<Scenario>& scenarios)
{
  if (token == "both" || token == "all") {
    scenarios.push_back(Scenario::Arch25);
    scenarios.push_back(Scenario::Arch101);
    return true;
  }
  if (token == "arch25" || token == "25"
      || token == "masonry_arch_25") {
    scenarios.push_back(Scenario::Arch25);
    return true;
  }
  if (token == "arch101" || token == "101"
      || token == "masonry_arch_101") {
    scenarios.push_back(Scenario::Arch101);
    return true;
  }

  return false;
}

bool parseScenarios(const char* value, std::vector<Scenario>& scenarios)
{
  const std::string input = value == nullptr ? "both" : value;
  std::stringstream stream(input);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!parseScenarioToken(token, scenarios))
      return false;
  }

  return !scenarios.empty();
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

dart::dynamics::SkeletonPtr createMasonryArchGroundPlane()
{
  auto skeleton = dart::dynamics::Skeleton::create("ground_plane");
  auto body
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createMasonryArchStone(
    std::size_t index,
    const dart::math::detail::MasonryArchStoneBoxGeometry& geometry,
    const ProbeOptions& probeOptions)
{
  // `initialOverlapScale` is a research probing knob applied on top of the
  // author-faithful geometry: it scales only the along-arch (chord-length)
  // extent, letting a sweep force overlap/gap without distorting the
  // cross-section tapering.
  Eigen::Vector3d size = geometry.size;
  size.x() *= probeOptions.initialOverlapScale;
  const double mass = kArchDensity * size.x() * size.y() * size.z();

  auto skeleton = dart::dynamics::Skeleton::create(
      "masonry_arch_stone_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(
          skeleton->getName() + "_body"));
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
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);
  joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(geometry.transform));
  // All stones are fully dynamic; the source scene fixes only the ground
  // plane (see PROVENANCE.txt section 7 in the geometry-port task notes).
  return skeleton;
}

std::shared_ptr<dart::simulation::World> createMasonryArchWorld(
    Scenario scenario,
    std::size_t maxContacts,
    const ProbeOptions& probeOptions)
{
  const std::size_t stoneCount = scenarioStoneCount(scenario);
  auto world = dart::simulation::World::create(scenarioName(scenario));
  world->setTimeStep(kDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto solver
      = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
          makeFbfOptions(probeOptions));
  solver->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
  solver->setNumSimulationThreads(1u);
  world->setConstraintSolver(std::move(solver));

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = maxContacts;
  collisionOption.maxNumContactsPerPair = probeOptions.maxContactsPerPair;

  // Author boundary condition from the Rigid-IPC arch scenes: only the
  // ground plane is fixed; every stone (including both springers) is
  // dynamic.
  world->addSkeleton(createMasonryArchGroundPlane());
  const auto stoneGeometry
      = dart::math::detail::generateMasonryArchStoneBoxes(stoneCount);
  for (std::size_t i = 0u; i < stoneCount; ++i) {
    world->addSkeleton(createMasonryArchStone(i, stoneGeometry[i], probeOptions));
  }

  return world;
}

dart::constraint::ExactCoulombFbfConstraintSolver* getExactSolver(
    const std::shared_ptr<dart::simulation::World>& world)
{
  return dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
      world->getConstraintSolver());
}

bool isMasonryArchSkeletonName(const std::string& name)
{
  return name.rfind("masonry_arch_stone_", 0u) == 0u;
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

ProbeResult runProbe(
    Scenario scenario, std::size_t maxContacts, const ProbeOptions& options)
{
  auto world = createMasonryArchWorld(scenario, maxContacts, options);
  auto* solver = getExactSolver(world);

  ProbeResult result;
  if (solver == nullptr)
    return result;

  const std::size_t crownIndex = scenarioStoneCount(scenario) / 2u;
  const std::string crownName
      = "masonry_arch_stone_" + std::to_string(crownIndex);

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton != nullptr && isMasonryArchSkeletonName(skeleton->getName())) {
      ++result.stoneCount;
      if (skeleton->isMobile())
        ++result.dynamicStoneCount;
    }
  }

  const auto start = std::chrono::steady_clock::now();
  world->step();
  const auto stop = std::chrono::steady_clock::now();
  result.elapsedMs
      = std::chrono::duration<double, std::milli>(stop - start).count();

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr || !isMasonryArchSkeletonName(skeleton->getName()))
      continue;

    const auto* body = skeleton->getBodyNode(0);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    result.finiteState = result.finiteState && position.allFinite()
                         && body->getLinearVelocity().allFinite()
                         && body->getAngularVelocity().allFinite();
    result.minCenterHeight = std::min(result.minCenterHeight, position.z());
    result.maxHorizontalTravel
        = std::max(result.maxHorizontalTravel, position.head<2>().norm());
    result.maxStoneSpeed
        = std::max(result.maxStoneSpeed, body->getLinearVelocity().norm());
    if (skeleton->getName() == crownName)
      result.crownHeight = position.z();
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
  result.failedBestIteration
      = solver->getLastFailedExactCoulombBestIteration();
  result.failedBestResidual = solver->getLastFailedExactCoulombBestResidual();
  result.failedStepSize = solver->getLastFailedExactCoulombStepSize();
  result.failedSafeStepSize
      = solver->getLastFailedExactCoulombSafeStepSize();
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
      const double previous = record.samples[record.samples.size() - 2u]
                                  .residual.value;
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
      "scenario,stone_count,dynamic_stone_count,max_contacts,"
      "max_contacts_per_pair,max_outer_iterations,inner_sweeps,"
      "inner_local_iterations,projected_gradient_iterations,gamma_mode,gamma,"
      "include_constraint_regularization,inner_diagonal_regularization,"
      "coupling_variation_tolerance,step_size_scale,outer_relaxation,"
      "initial_overlap_scale,use_matrix_free_delassus_operator,"
      "use_matrix_free_delassus_seed,contacts,elapsed_ms,exact_success,"
      "finite_state,exact_status,fbf_status,fbf_iterations,best_iteration,"
      "max_fbf_iterations,total_fbf_iterations,best_residual,step_size,"
      "safe_step_size,coupling_variation_ratio,shrink_iterations,residual,"
      "primal,dual,complementarity,worst_primal_contact,worst_dual_contact,"
      "worst_complementarity_contact,exact_solves,exact_failures,fallbacks,"
      "warm_starts,projected_gradient_retries,dense_residual_polishes,"
      "matrix_free_delassus_operator_used,matrix_free_delassus_seed_used,"
      "failed_fbf_status,failed_fbf_iterations,failed_best_iteration,"
      "failed_best_residual,failed_step_size,failed_safe_step_size,"
      "failed_coupling_variation_ratio,failed_shrink_iterations,"
      "failed_residual,failed_primal,failed_dual,failed_complementarity,"
      "failed_worst_primal_contact,failed_worst_dual_contact,"
      "failed_worst_complementarity_contact,failed_worst_primal_pair,"
      "failed_worst_dual_pair,failed_worst_complementarity_pair,"
      "residual_history_records,residual_history_rows,"
      "failed_residual_history_rows,failed_residual_history_last_iteration,"
      "failed_residual_history_first,failed_residual_history_last,"
      "failed_residual_history_tail_ratio,crown_height,min_center_height,"
      "max_horizontal_travel,max_stone_speed\n");
}

void printRow(
    Scenario scenario,
    std::size_t maxContacts,
    const ProbeOptions& options)
{
  const ProbeResult result = runProbe(scenario, maxContacts, options);
  std::cout << std::setprecision(17) << scenarioName(scenario) << ','
            << result.stoneCount << ',' << result.dynamicStoneCount << ','
            << maxContacts << ',' << options.maxContactsPerPair << ','
            << options.maxOuterIterations << ',' << options.innerSweeps << ','
            << options.innerLocalIterations << ','
            << options.projectedGradientIterations << ','
            << gammaLabel(options.initialGamma) << ','
            << safeValue(options.initialGamma) << ','
            << (options.includeConstraintRegularization ? 1 : 0) << ','
            << safeValue(options.innerDiagonalRegularization) << ','
            << safeValue(options.couplingVariationTolerance) << ','
            << safeValue(options.stepSizeScale) << ','
            << safeValue(options.outerRelaxation) << ','
            << safeValue(options.initialOverlapScale) << ','
            << (options.useMatrixFreeDelassusOperator ? 1 : 0) << ','
            << (options.useMatrixFreeDelassusSeed ? 1 : 0) << ','
            << result.contacts << ',' << safeValue(result.elapsedMs) << ','
            << (result.exactSuccess ? 1 : 0) << ','
            << (result.finiteState ? 1 : 0) << ',' << result.exactStatus << ','
            << result.fbfStatus << ',' << result.fbfIterations << ','
            << result.bestIteration << ',' << result.maxFbfIterations << ','
            << result.totalFbfIterations << ','
            << safeValue(result.bestResidual) << ','
            << safeValue(result.stepSize) << ','
            << safeValue(result.safeStepSize) << ','
            << safeValue(result.couplingVariationRatio) << ','
            << result.shrinkIterations << ',' << safeValue(result.residual)
            << ',' << safeValue(result.primalResidual) << ','
            << safeValue(result.dualResidual) << ','
            << safeValue(result.complementarityResidual) << ','
            << result.worstPrimalContact << ',' << result.worstDualContact
            << ',' << result.worstComplementarityContact << ','
            << result.exactSolves << ',' << result.exactFailures << ','
            << result.fallbacks << ',' << result.warmStarts << ','
            << result.projectedGradientRetries << ','
            << result.denseResidualPolishes << ','
            << (result.matrixFreeDelassusOperatorUsed ? 1 : 0) << ','
            << (result.matrixFreeDelassusSeedUsed ? 1 : 0) << ','
            << result.failedFbfStatus << ',' << result.failedFbfIterations
            << ',' << result.failedBestIteration << ','
            << safeValue(result.failedBestResidual) << ','
            << safeValue(result.failedStepSize) << ','
            << safeValue(result.failedSafeStepSize) << ','
            << safeValue(result.failedCouplingVariationRatio) << ','
            << result.failedShrinkIterations << ','
            << safeValue(result.failedResidual) << ','
            << safeValue(result.failedPrimalResidual) << ','
            << safeValue(result.failedDualResidual) << ','
            << safeValue(result.failedComplementarityResidual) << ','
            << result.failedWorstPrimalContact << ','
            << result.failedWorstDualContact << ','
            << result.failedWorstComplementarityContact << ','
            << result.failedWorstPrimalPair << ','
            << result.failedWorstDualPair << ','
            << result.failedWorstComplementarityPair << ','
            << result.residualHistoryRecords << ','
            << result.residualHistoryRows << ','
            << result.failedResidualHistoryRows << ','
            << result.failedResidualHistoryLastIteration << ','
            << safeValue(result.failedResidualHistoryFirst) << ','
            << safeValue(result.failedResidualHistoryLast) << ','
            << safeValue(result.failedResidualHistoryTailRatio) << ','
            << safeValue(result.crownHeight) << ','
            << safeValue(result.minCenterHeight) << ','
            << safeValue(result.maxHorizontalTravel) << ','
            << safeValue(result.maxStoneSpeed) << '\n'
            << std::flush;
}

void printUsage()
{
  std::fprintf(
      stderr,
      "Usage: fbf_paper_arch_probe [scenario=both] "
      "[max_contacts_csv=default] [max_contacts_per_pair=2] "
      "[max_outer_iterations=20000] [inner_sweeps=120] "
      "[inner_local_iterations=32] [projected_gradient_iterations=400] "
      "[initial_gamma=nan] [include_constraint_regularization=0] "
      "[inner_diagonal_regularization=0] "
      "[coupling_variation_tolerance=0.9] [step_size_scale=10] "
      "[outer_relaxation=1] [max_residual_history_samples=0] "
      "[initial_overlap_scale=1.0] "
      "[use_matrix_free_delassus_operator=0] "
      "[use_matrix_free_delassus_seed=0]\n"
      "Scenarios: arch25, arch101, both, or a comma-separated list.\n");
}

} // namespace

int main(int argc, char** argv)
{
  std::vector<Scenario> scenarios;
  if (!parseScenarios(argc > 1 ? argv[1] : nullptr, scenarios)) {
    printUsage();
    return 2;
  }

  std::vector<std::size_t> maxContactValues;
  if (!parseSizeList(argc > 2 ? argv[2] : nullptr, maxContactValues)) {
    printUsage();
    return 2;
  }

  ProbeOptions options;
  if (!parseSizeArg(
          argc > 3 ? argv[3] : nullptr,
          options.maxContactsPerPair,
          options.maxContactsPerPair)
      || !parseIntArg(
          argc > 4 ? argv[4] : nullptr,
          options.maxOuterIterations,
          options.maxOuterIterations)
      || !parseIntArg(
          argc > 5 ? argv[5] : nullptr,
          options.innerSweeps,
          options.innerSweeps)
      || !parseIntArg(
          argc > 6 ? argv[6] : nullptr,
          options.innerLocalIterations,
          options.innerLocalIterations)
      || !parseIntArg(
          argc > 7 ? argv[7] : nullptr,
          options.projectedGradientIterations,
          options.projectedGradientIterations)
      || !parseGammaArg(
          argc > 8 ? argv[8] : nullptr,
          options.initialGamma,
          options.initialGamma)
      || !parseBoolArg(
          argc > 9 ? argv[9] : nullptr,
          options.includeConstraintRegularization,
          options.includeConstraintRegularization)
      || !parseDoubleArg(
          argc > 10 ? argv[10] : nullptr,
          options.innerDiagonalRegularization,
          options.innerDiagonalRegularization)
      || !parseDoubleArg(
          argc > 11 ? argv[11] : nullptr,
          options.couplingVariationTolerance,
          options.couplingVariationTolerance)
      || !parseDoubleArg(
          argc > 12 ? argv[12] : nullptr,
          options.stepSizeScale,
          options.stepSizeScale)
      || options.stepSizeScale <= 0.0
      || !parseDoubleArg(
          argc > 13 ? argv[13] : nullptr,
          options.outerRelaxation,
          options.outerRelaxation)
      || options.outerRelaxation <= 0.0
      || !parseIntArg(
          argc > 14 ? argv[14] : nullptr,
          options.maxResidualHistorySamples,
          options.maxResidualHistorySamples)
      || options.maxResidualHistorySamples < 0
      || !parseDoubleArg(
          argc > 15 ? argv[15] : nullptr,
          options.initialOverlapScale,
          options.initialOverlapScale)
      || options.initialOverlapScale <= 0.0
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
  std::fflush(stdout);
  for (const auto scenario : scenarios) {
    if (maxContactValues.empty()) {
      printRow(scenario, scenarioDefaultMaxContacts(scenario), options);
      continue;
    }

    for (const auto maxContacts : maxContactValues) {
      printRow(scenario, maxContacts, options);
    }
  }

  return 0;
}
