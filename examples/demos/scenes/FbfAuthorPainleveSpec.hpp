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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORPAINLEVESPEC_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORPAINLEVESPEC_HPP_

#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

#ifndef DART_FBF_AUTHOR_PAINLEVE_SPEC_SHA256
  #error "FBF author-Painleve consumers must bind the shared spec source hash"
#endif
#ifndef DART_FBF_EXACT_SOLVER_OPTIONS_SHA256
  #error "FBF author-Painleve consumers must bind the exact-solver options hash"
#endif

namespace fbf_author_painleve {

inline constexpr const char* kContractSchema
    = "dart.fbf_author_painleve_dynamics_adapter/v1";
inline constexpr const char* kContractKind
    = "source_configuration_dynamics_adapter";
inline constexpr const char* kAuthorRepository
    = "https://github.com/matthcsong/fbf-sca-2026";
inline constexpr const char* kAuthorCommit
    = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0";
inline constexpr const char* kAuthorTree
    = "ffcdafb61adeda2239c8366d054b548b50d26685e";
inline constexpr const char* kAuthorRunBlob
    = "afaa03613b0ad0a30290168d2fd64221fc3523b7";
inline constexpr const char* kAuthorRunSha256
    = "818fa8f75c2c73e2dd08f0e0e9f9f5d58f63d8073dce38f874e2da24b2aa46e3";
inline constexpr const char* kSpecSourceSha256
    = DART_FBF_AUTHOR_PAINLEVE_SPEC_SHA256;
inline constexpr const char* kExactSolverOptionsSha256
    = DART_FBF_EXACT_SOLVER_OPTIONS_SHA256;

inline constexpr double kGravity = 9.81;
inline constexpr double kBoxWidth = 0.3;
inline constexpr double kBoxHeight = 0.6;
inline constexpr double kBoxDepth = 1.2;
inline constexpr double kDensity = 200.0;
inline constexpr double kInitialVelocity = 4.0;
inline constexpr double kCriticalFriction = kBoxWidth / kBoxHeight;
inline constexpr double kTimeStep = 1.0 / 60.0;
inline constexpr double kDuration = 2.0;
inline constexpr std::size_t kTotalSteps = 120u;
inline constexpr double kGroundHalfExtentX = 5.0;
inline constexpr double kGroundHalfExtentY = 1.5;
inline constexpr double kGroundHalfExtentZ = 0.05;
inline constexpr double kSourceContactGap = 0.005;
inline constexpr double kSourceShapeStiffness = 1.0e4;
inline constexpr double kSourceShapeDamping = 1.0e3;
inline constexpr std::size_t kMaxContacts = 4u;
inline constexpr std::size_t kMaxContactsPerPair = 4u;
inline constexpr int kDartMaxOuterIterations = 1000;
inline constexpr double kDartTolerance = 1.0e-6;
inline constexpr int kDartInnerMaxSweeps = 120;
inline constexpr int kDartInnerLocalIterations = 32;
// DART's conservative safe base uses c_gamma=.5, so a scale of ten maps the
// public Painleve CLI's gamma_c=5 request onto DART's adaptive line search.
inline constexpr double kDartStepSizeScale = 10.0;
inline constexpr double kDartPersistentStepSizeSafeBoundScale = 10.0;
inline constexpr const char* kDartSolverPolicy
    = "source_gamma_c_5_strict_dart_adapter";

//==============================================================================
inline dart::constraint::ExactCoulombFbfConstraintSolverOptions
makeExactSolverOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = kDartMaxOuterIterations;
  options.tolerance = kDartTolerance;
  options.innerMaxSweeps = kDartInnerMaxSweeps;
  options.innerLocalIterations = kDartInnerLocalIterations;
  options.stepSizeScale = kDartStepSizeScale;
  return options;
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfCrossStepPolicyOptions
makeExactCrossStepPolicyOptions()
{
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions options;
  options.persistentStepSizeSafeBoundScale
      = kDartPersistentStepSizeSafeBoundScale;
  return options;
}

inline Eigen::Vector3d boxSize()
{
  return Eigen::Vector3d(kBoxWidth, kBoxDepth, kBoxHeight);
}

inline Eigen::Vector3d groundSize()
{
  return Eigen::Vector3d(
      2.0 * kGroundHalfExtentX,
      2.0 * kGroundHalfExtentY,
      2.0 * kGroundHalfExtentZ);
}

inline constexpr double boxMass()
{
  return kDensity * kBoxWidth * kBoxDepth * kBoxHeight;
}

inline Eigen::Matrix3d boxMoment()
{
  const Eigen::Vector3d size = boxSize();
  const double mass = boxMass();
  Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();
  moment(0, 0) = mass * (size.y() * size.y() + size.z() * size.z()) / 12.0;
  moment(1, 1) = mass * (size.x() * size.x() + size.z() * size.z()) / 12.0;
  moment(2, 2) = mass * (size.x() * size.x() + size.y() * size.y()) / 12.0;
  return moment;
}

struct ScenarioSpec
{
  const char* traceScenario = nullptr;
  const char* demoScene = nullptr;
  double friction = 0.0;
};

inline constexpr std::array<ScenarioSpec, 2> kScenarios{{
    {"painleve_author_mu_0_5", "fbf_author_painleve_mu_0_5", 0.5},
    {"painleve_author_mu_0_55", "fbf_author_painleve_mu_0_55", 0.55},
}};

//==============================================================================
inline const ScenarioSpec* findByTraceScenario(const std::string& id)
{
  for (const auto& scenario : kScenarios) {
    if (id == scenario.traceScenario)
      return &scenario;
  }
  return nullptr;
}

//==============================================================================
inline const ScenarioSpec* findByDemoScene(const std::string& id)
{
  for (const auto& scenario : kScenarios) {
    if (id == scenario.demoScene)
      return &scenario;
  }
  return nullptr;
}

struct PhysicsContract
{
  ScenarioSpec scenario;
  std::string implementationSourceSha256;
  std::string solverLane;
  double timeStep = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::size_t simulationThreads = 0u;
  bool deactivationEnabled = true;
  std::string collisionDetector;
  std::string contactManifold;
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  bool splitImpulseEnabled = false;
  bool exactOptionsAvailable = false;
  int maxOuterIterations = 0;
  double tolerance = 0.0;
  int innerMaxSweeps = 0;
  int innerLocalIterations = 0;
  double stepSizeScale = 0.0;
  bool stepSizePersistenceEnabled = false;
  double persistentStepSizeSafeBoundScale = 0.0;
  bool postCorrectionProjectionEnabled = false;
  bool fallbackToBoxedLcpEnabled = false;
  bool groundMobile = true;
  Eigen::Vector3d groundSize = Eigen::Vector3d::Zero();
  Eigen::Isometry3d groundPose = Eigen::Isometry3d::Identity();
  double groundFriction = 0.0;
  bool boxMobile = false;
  Eigen::Vector3d observedBoxSize = Eigen::Vector3d::Zero();
  Eigen::Isometry3d boxPose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d boxLinearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d boxAngularVelocity = Eigen::Vector3d::Zero();
  double boxFriction = 0.0;
  double observedBoxMass = 0.0;
  Eigen::Matrix3d observedBoxMoment = Eigen::Matrix3d::Zero();
};

//==============================================================================
inline void requireNear(double actual, double expected, const std::string& what)
{
  const double scale = std::max({1.0, std::abs(actual), std::abs(expected)});
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-11 * scale) {
    std::ostringstream message;
    message << std::setprecision(std::numeric_limits<double>::max_digits10)
            << "author Painleve mismatch: " << what << " actual=" << actual
            << " expected=" << expected;
    throw std::runtime_error(message.str());
  }
}

//==============================================================================
inline PhysicsContract inspectPhysicsContract(
    const std::shared_ptr<dart::simulation::World>& world,
    const ScenarioSpec& scenario,
    const std::string& implementationSourceSha256)
{
  if (!world || !world->getConstraintSolver())
    throw std::runtime_error("author Painleve adapter has no solver");

  PhysicsContract contract;
  contract.scenario = scenario;
  contract.implementationSourceSha256 = implementationSourceSha256;
  contract.timeStep = world->getTimeStep();
  contract.gravity = world->getGravity();
  contract.simulationThreads = world->getNumSimulationThreads();
  contract.deactivationEnabled = world->getDeactivationOptions().mEnabled;

  auto* exact
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  auto* boxed = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  if (exact) {
    contract.solverLane = "exact_fbf";
    contract.exactOptionsAvailable = true;
    const auto& options = exact->getExactCoulombOptions();
    contract.maxOuterIterations = options.maxOuterIterations;
    contract.tolerance = options.tolerance;
    contract.innerMaxSweeps = options.innerMaxSweeps;
    contract.innerLocalIterations = options.innerLocalIterations;
    contract.stepSizeScale = options.stepSizeScale;
    contract.stepSizePersistenceEnabled = options.enableStepSizePersistence;
    contract.persistentStepSizeSafeBoundScale
        = exact->getExactCoulombCrossStepPolicyOptions()
              .persistentStepSizeSafeBoundScale;
    contract.postCorrectionProjectionEnabled
        = exact->getExactCoulombPostCorrectionProjectionEnabled();
    contract.fallbackToBoxedLcpEnabled = options.fallbackToBoxedLcp;
    if (contract.maxOuterIterations != kDartMaxOuterIterations
        || contract.innerMaxSweeps != kDartInnerMaxSweeps
        || contract.innerLocalIterations != kDartInnerLocalIterations
        || !contract.stepSizePersistenceEnabled
        || !contract.postCorrectionProjectionEnabled
        || !contract.fallbackToBoxedLcpEnabled) {
      throw std::runtime_error("author Painleve mismatch: exact solver policy");
    }
    requireNear(contract.tolerance, kDartTolerance, "exact tolerance");
    requireNear(
        contract.stepSizeScale, kDartStepSizeScale, "exact gamma scale");
    requireNear(
        contract.persistentStepSizeSafeBoundScale,
        kDartPersistentStepSizeSafeBoundScale,
        "persistent gamma safe-bound scale");
  } else if (boxed) {
    contract.solverLane = "boxed_lcp";
  } else {
    throw std::runtime_error("author Painleve adapter solver is unsupported");
  }

  const auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          world->getConstraintSolver()->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author Painleve adapter requires Native collision");
  contract.collisionDetector = detector->getType();
  contract.contactManifold
      = detector->getContactManifoldMode()
                == dart::collision::NativeCollisionDetector::
                    ContactManifoldMode::FourPointPlanar
            ? "four_point_planar"
            : "compact";
  const auto& collisionOption
      = world->getConstraintSolver()->getCollisionOption();
  contract.maxContacts = collisionOption.maxNumContacts;
  contract.maxContactsPerPair = collisionOption.maxNumContactsPerPair;
  contract.splitImpulseEnabled
      = world->getConstraintSolver()->isSplitImpulseEnabled();

  const auto ground = world->getSkeleton("painleve_author_ground");
  const auto box = world->getSkeleton("painleve_author_box");
  if (!ground || !box || ground->getNumBodyNodes() != 1u
      || box->getNumBodyNodes() != 1u) {
    throw std::runtime_error("author Painleve adapter bodies are missing");
  }
  const auto* groundBody = ground->getBodyNode(0u);
  const auto* boxBody = box->getBodyNode(0u);
  const auto* groundNode
      = groundBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  const auto* boxNode
      = boxBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (!groundNode || !boxNode || !groundNode->getDynamicsAspect()
      || !boxNode->getDynamicsAspect()) {
    throw std::runtime_error(
        "author Painleve adapter collision shapes are missing");
  }
  const auto groundBox
      = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
          groundNode->getShape());
  const auto dynamicBox
      = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
          boxNode->getShape());
  if (!groundBox || !dynamicBox)
    throw std::runtime_error("author Painleve adapter shapes are not boxes");

  contract.groundMobile = ground->isMobile();
  contract.groundSize = groundBox->getSize();
  contract.groundPose = groundBody->getWorldTransform();
  contract.groundFriction = groundNode->getDynamicsAspect()->getFrictionCoeff();
  contract.boxMobile = box->isMobile();
  contract.observedBoxSize = dynamicBox->getSize();
  contract.boxPose = boxBody->getWorldTransform();
  contract.boxLinearVelocity = boxBody->getLinearVelocity();
  contract.boxAngularVelocity = boxBody->getAngularVelocity();
  contract.boxFriction = boxNode->getDynamicsAspect()->getFrictionCoeff();
  contract.observedBoxMass = boxBody->getInertia().getMass();
  contract.observedBoxMoment = boxBody->getInertia().getMoment();

  requireNear(contract.timeStep, kTimeStep, "time step");
  if (!contract.gravity.isApprox(Eigen::Vector3d(0.0, 0.0, -kGravity), 1e-12))
    throw std::runtime_error("author Painleve mismatch: gravity");
  if (contract.deactivationEnabled)
    throw std::runtime_error("author Painleve mismatch: deactivation enabled");
  if (contract.contactManifold != "four_point_planar")
    throw std::runtime_error("author Painleve mismatch: contact manifold");
  if (contract.maxContacts != kMaxContacts
      || contract.maxContactsPerPair != kMaxContactsPerPair) {
    throw std::runtime_error("author Painleve mismatch: contact caps");
  }
  if (contract.groundMobile
      || !contract.groundSize.isApprox(groundSize(), 1e-12))
    throw std::runtime_error("author Painleve mismatch: ground geometry");
  if (!contract.groundPose.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12)
      || !contract.groundPose.translation().isApprox(
          Eigen::Vector3d(0.0, 0.0, -kGroundHalfExtentZ), 1e-12)) {
    throw std::runtime_error("author Painleve mismatch: ground initial pose");
  }
  requireNear(contract.groundFriction, scenario.friction, "ground friction");
  if (!contract.boxMobile
      || !contract.observedBoxSize.isApprox(boxSize(), 1e-12)) {
    throw std::runtime_error("author Painleve mismatch: box geometry");
  }
  if (!contract.boxPose.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12)
      || !contract.boxPose.translation().isApprox(
          Eigen::Vector3d(0.0, 0.0, 0.5 * kBoxHeight), 1e-12)) {
    throw std::runtime_error("author Painleve mismatch: box initial pose");
  }
  if (!contract.boxLinearVelocity.isApprox(
          Eigen::Vector3d(kInitialVelocity, 0.0, 0.0), 1e-12)
      || !contract.boxAngularVelocity.isZero(1e-12)) {
    throw std::runtime_error("author Painleve mismatch: box initial velocity");
  }
  requireNear(contract.boxFriction, scenario.friction, "box friction");
  requireNear(contract.observedBoxMass, boxMass(), "box mass");
  if (!contract.observedBoxMoment.isApprox(boxMoment(), 1e-12))
    throw std::runtime_error("author Painleve mismatch: box moment");
  return contract;
}

//==============================================================================
inline void writeJsonString(std::ostream& out, const std::string& value)
{
  out << '"';
  for (const char c : value) {
    if (c == '"' || c == '\\')
      out << '\\';
    out << c;
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
inline std::string physicsContractJson(const PhysicsContract& contract)
{
  if (!contract.scenario.traceScenario || !contract.scenario.demoScene)
    throw std::runtime_error("author Painleve contract has no scenario");

  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema_version\":";
  writeJsonString(out, kContractSchema);
  out << ",\"kind\":";
  writeJsonString(out, kContractKind);
  out << ",\"source_binding\":{\"repository\":";
  writeJsonString(out, kAuthorRepository);
  out << ",\"commit\":";
  writeJsonString(out, kAuthorCommit);
  out << ",\"tree\":";
  writeJsonString(out, kAuthorTree);
  out << ",\"painleve_run_blob\":";
  writeJsonString(out, kAuthorRunBlob);
  out << ",\"painleve_run_py_sha256\":";
  writeJsonString(out, kAuthorRunSha256);
  out << ",\"configuration_spec_sha256\":";
  writeJsonString(out, kSpecSourceSha256);
  out << ",\"exact_solver_options_sha256\":";
  writeJsonString(out, kExactSolverOptionsSha256);
  out << ",\"demo_implementation_sha256\":";
  writeJsonString(out, contract.implementationSourceSha256);
  out << "},\"source_configuration\":{\"gravity_m_s2\":" << kGravity
      << ",\"box_width_m\":" << kBoxWidth << ",\"box_height_m\":" << kBoxHeight
      << ",\"box_depth_m\":" << kBoxDepth << ",\"density_kg_m3\":" << kDensity
      << ",\"mass_kg\":" << boxMass()
      << ",\"initial_velocity_m_s\":" << kInitialVelocity
      << ",\"critical_friction\":" << kCriticalFriction
      << ",\"time_step_seconds\":" << kTimeStep
      << ",\"duration_seconds\":" << kDuration
      << ",\"total_steps\":" << kTotalSteps
      << ",\"source_default_mu_values\":[0.55]"
         ",\"selected_source_supported_mu_sweep\":[0.5,0.55]"
         ",\"ground_half_extents_m\":[5,1.5,0.05]"
         ",\"box_initial_pose\":{\"translation_m\":[0,0,0.3],"
         "\"rotation\":[[1,0,0],[0,1,0],[0,0,1]]}"
         ",\"contact\":{\"friction\":"
      << contract.scenario.friction << ",\"gap_m\":" << kSourceContactGap
      << ",\"shape_stiffness\":" << kSourceShapeStiffness
      << ",\"shape_damping\":" << kSourceShapeDamping
      << "}},\"dart_adapter\":{\"scene_id\":";
  writeJsonString(out, contract.scenario.demoScene);
  out << ",\"world\":{\"time_step_seconds\":" << contract.timeStep
      << ",\"gravity_m_s2\":";
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
      << "},\"solver\":{\"lane\":";
  writeJsonString(out, contract.solverLane);
  out << ",\"configuration_policy\":";
  writeJsonString(out, kDartSolverPolicy);
  out << ",\"split_impulse_enabled\":"
      << (contract.splitImpulseEnabled ? "true" : "false")
      << ",\"exact_options\":";
  if (!contract.exactOptionsAvailable) {
    out << "null";
  } else {
    out << "{\"max_outer_iterations\":" << contract.maxOuterIterations
        << ",\"tolerance\":" << contract.tolerance
        << ",\"inner_max_sweeps\":" << contract.innerMaxSweeps
        << ",\"inner_local_iterations\":" << contract.innerLocalIterations
        << ",\"step_size_scale\":" << contract.stepSizeScale
        << ",\"step_size_persistence_enabled\":"
        << (contract.stepSizePersistenceEnabled ? "true" : "false")
        << ",\"persistent_step_size_safe_bound_scale\":"
        << contract.persistentStepSizeSafeBoundScale
        << ",\"post_correction_projection_enabled\":"
        << (contract.postCorrectionProjectionEnabled ? "true" : "false")
        << ",\"fallback_to_boxed_lcp_enabled\":"
        << (contract.fallbackToBoxedLcpEnabled ? "true" : "false") << '}';
  }
  out << "},\"ground\":{\"mobile\":"
      << (contract.groundMobile ? "true" : "false") << ",\"size_m\":";
  writeJsonVector(out, contract.groundSize);
  out << ",\"initial_pose\":";
  writeJsonPose(out, contract.groundPose);
  out << ",\"friction\":" << contract.groundFriction
      << "},\"box\":{\"mobile\":" << (contract.boxMobile ? "true" : "false")
      << ",\"size_m\":";
  writeJsonVector(out, contract.observedBoxSize);
  out << ",\"initial_pose\":";
  writeJsonPose(out, contract.boxPose);
  out << ",\"initial_linear_velocity_m_s\":";
  writeJsonVector(out, contract.boxLinearVelocity);
  out << ",\"initial_angular_velocity_rad_s\":";
  writeJsonVector(out, contract.boxAngularVelocity);
  out << ",\"friction\":" << contract.boxFriction
      << ",\"mass_kg\":" << contract.observedBoxMass << ",\"moment_kg_m2\":";
  writeJsonMatrix(out, contract.observedBoxMoment);
  out << "}},\"adapter_boundaries\":{"
         "\"source_contact_gap_recorded_m\":0.005,"
         "\"source_contact_gap_semantics_implemented\":false,"
         "\"source_shape_stiffness_semantics_implemented\":false,"
         "\"source_shape_damping_semantics_implemented\":false,"
         "\"source_collision_backend_implemented\":false,"
         "\"source_solver_backend_semantics_implemented\":false,"
         "\"source_float32_semantics_implemented\":false,"
         "\"dart_native_four_point_planar_is_adapter_choice\":true},"
         "\"claim_boundary\":{"
         "\"current_source_parameterized_configuration_port\":true,"
         "\"historical_paper_invocation_known\":false,"
         "\"trajectory_valid\":false,"
         "\"physical_outcome_valid\":false,"
         "\"trajectory_equivalence\":false,"
         "\"solver_equivalence\":false,"
         "\"physical_outcome_equivalence\":false,"
         "\"fig05_parity\":false,"
         "\"video05_parity\":false,"
         "\"timing_comparability\":false,"
         "\"paper_parity\":false}}";
  return out.str();
}

//==============================================================================
inline std::vector<std::pair<std::string, double>> sceneStateFields(
    const std::shared_ptr<dart::simulation::World>& world,
    const ScenarioSpec& scenario)
{
  if (!world)
    throw std::runtime_error("author Painleve scene state has no world");
  if (world->getName() != scenario.demoScene)
    throw std::runtime_error("author Painleve scene state has the wrong world");
  const auto box = world->getSkeleton("painleve_author_box");
  if (!box || box->getNumBodyNodes() != 1u)
    throw std::runtime_error("author Painleve scene state has no box");

  const auto* body = box->getBodyNode(0u);
  const Eigen::Isometry3d transform = body->getWorldTransform();
  const Eigen::Vector3d linearVelocity = body->getLinearVelocity();
  const Eigen::Vector3d angularVelocity = body->getAngularVelocity();
  const Eigen::Vector3d bodyUp = transform.linear().col(2);
  const double uprightness = bodyUp.dot(Eigen::Vector3d::UnitZ());
  const double pitch
      = std::atan2(transform.linear()(0, 2), transform.linear()(2, 2));
  const double worldTime = world->getTime();
  if (!transform.matrix().allFinite() || !linearVelocity.allFinite()
      || !angularVelocity.allFinite() || !bodyUp.allFinite()
      || !std::isfinite(uprightness) || !std::isfinite(pitch)
      || !std::isfinite(worldTime)) {
    throw std::runtime_error("author Painleve scene state is non-finite");
  }

  return {
      {"world_time_seconds", worldTime},
      {"position_x_m", transform.translation().x()},
      {"position_y_m", transform.translation().y()},
      {"position_z_m", transform.translation().z()},
      {"rotation_00", transform.linear()(0, 0)},
      {"rotation_01", transform.linear()(0, 1)},
      {"rotation_02", transform.linear()(0, 2)},
      {"rotation_10", transform.linear()(1, 0)},
      {"rotation_11", transform.linear()(1, 1)},
      {"rotation_12", transform.linear()(1, 2)},
      {"rotation_20", transform.linear()(2, 0)},
      {"rotation_21", transform.linear()(2, 1)},
      {"rotation_22", transform.linear()(2, 2)},
      {"body_up_x", bodyUp.x()},
      {"body_up_y", bodyUp.y()},
      {"body_up_z", bodyUp.z()},
      {"uprightness_cosine", uprightness},
      {"pitch_rad", pitch},
      {"linear_velocity_x_m_s", linearVelocity.x()},
      {"linear_velocity_y_m_s", linearVelocity.y()},
      {"linear_velocity_z_m_s", linearVelocity.z()},
      {"angular_velocity_x_rad_s", angularVelocity.x()},
      {"angular_velocity_y_rad_s", angularVelocity.y()},
      {"angular_velocity_z_rad_s", angularVelocity.z()},
  };
}

} // namespace fbf_author_painleve

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORPAINLEVESPEC_HPP_
