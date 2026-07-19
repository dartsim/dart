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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORTURNTABLESPEC_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORTURNTABLESPEC_HPP_

#include <dart/simulation/World.hpp>

#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <cmath>
#include <cstddef>

#ifndef DART_FBF_AUTHOR_TURNTABLE_SPEC_SHA256
  #error "FBF author-turntable consumers must bind the shared spec source hash"
#endif

namespace fbf_author_turntable {

inline constexpr const char* kContractSchema
    = "dart.fbf_author_turntable_physics_contract/v1";
inline constexpr const char* kContractKind = "physics_control";
inline constexpr const char* kAuthorCommit
    = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0";
inline constexpr const char* kAuthorSceneSourceSha256
    = "5dd330d2e430585fc3e61eb9c11d2d9aa00b518170f7b64b809c69587cf7db53";
inline constexpr const char* kSpecSourceSha256
    = DART_FBF_AUTHOR_TURNTABLE_SPEC_SHA256;

inline constexpr double kTimeStep = 1.0 / 60.0;
inline constexpr double kGravity = 9.81;
inline constexpr double kSupportRadius = 2.0;
inline constexpr double kSupportHalfHeight = 0.05;
inline constexpr double kRiderHalfSize = 0.15;
inline constexpr double kRiderDensity = 500.0;
inline constexpr double kInitialRadius = 1.0;
inline constexpr double kGeometricGap = 0.005;
inline constexpr double kDropHeight = 0.2;
inline constexpr double kSettleDuration = 0.5;
inline constexpr double kRampDuration = 0.5;
inline constexpr double kDuration = 6.0;
inline constexpr std::size_t kMaxContacts = 4u;
inline constexpr std::size_t kMaxContactsPerPair = 4u;
inline constexpr int kMaxOuterIterations = 500;
inline constexpr double kTolerance = 1e-6;
inline constexpr int kInnerMaxSweeps = 120;
inline constexpr int kInnerLocalIterations = 32;
inline constexpr double kStepSizeScale = 2.0;

struct ScenarioSpec
{
  const char* traceScenario;
  const char* demoScene;
  double friction;
  double angularVelocity;
  const char* expectedOutcome;
};

inline constexpr std::array<ScenarioSpec, 4> kScenarios{{
    {"turntable_author_mu_0_2_omega_2",
     "fbf_author_turntable_mu_0_2_omega_2",
     0.2,
     2.0,
     "ejected"},
    {"turntable_author_mu_0_2_omega_5",
     "fbf_author_turntable_mu_0_2_omega_5",
     0.2,
     5.0,
     "ejected"},
    {"turntable_author_mu_0_5_omega_2",
     "fbf_author_turntable_mu_0_5_omega_2",
     0.5,
     2.0,
     "retained_through_6s"},
    {"turntable_author_mu_0_5_omega_5",
     "fbf_author_turntable_mu_0_5_omega_5",
     0.5,
     5.0,
     "ejected"},
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

//==============================================================================
inline double angularVelocityScale(double time)
{
  if (time < kSettleDuration)
    return 0.0;
  if (time >= kSettleDuration + kRampDuration)
    return 1.0;

  const double s = (time - kSettleDuration) / kRampDuration;
  return s * s * (3.0 - 2.0 * s);
}

//==============================================================================
inline double angularVelocity(double time, double targetAngularVelocity)
{
  return targetAngularVelocity * angularVelocityScale(time);
}

//==============================================================================
inline double integratedYaw(double time, double targetAngularVelocity)
{
  if (time <= kSettleDuration)
    return 0.0;

  const double rampTime = time - kSettleDuration;
  if (rampTime < kRampDuration) {
    const double s = rampTime / kRampDuration;
    // Integral of 3*s^2 - 2*s^3 over the normalized ramp interval.
    return targetAngularVelocity * kRampDuration
           * (s * s * s - 0.5 * s * s * s * s);
  }

  return targetAngularVelocity
         * (0.5 * kRampDuration + rampTime - kRampDuration);
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfConstraintSolverOptions
dartBestSolverOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = kMaxOuterIterations;
  options.tolerance = kTolerance;
  options.innerMaxSweeps = kInnerMaxSweeps;
  options.innerLocalIterations = kInnerLocalIterations;
  options.stepSizeScale = kStepSizeScale;
  return options;
}

struct PhysicsContract
{
  const ScenarioSpec* scenario = nullptr;
  std::string solverContract;
  std::string binaryRole;
  std::string binarySourceSha256;
  double timeStep = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::size_t simulationThreads = 0u;
  bool deactivationEnabled = true;
  std::string collisionDetector;
  std::string contactManifold;
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  bool splitImpulseEnabled = false;
  int maxOuterIterations = 0;
  double tolerance = 0.0;
  int innerMaxSweeps = 0;
  int innerLocalIterations = 0;
  double stepSizeScale = 0.0;
  bool warmStartEnabled = false;
  bool fallbackToBoxedLcpEnabled = false;
  bool projectedGradientRetryEnabled = false;
  bool denseResidualPolishEnabled = false;
  bool contactRowOperatorEnabled = false;
  bool denseContactRowSnapshotEnabled = false;
  std::string supportShape;
  bool supportMobile = true;
  double supportRadius = 0.0;
  double supportHeight = 0.0;
  double supportFriction = 0.0;
  Eigen::Isometry3d supportPose = Eigen::Isometry3d::Identity();
  std::string riderShape;
  Eigen::Vector3d riderSize = Eigen::Vector3d::Zero();
  double riderFriction = 0.0;
  double riderMass = 0.0;
  Eigen::Matrix3d riderMoment = Eigen::Matrix3d::Zero();
  Eigen::Isometry3d riderPose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d riderLinearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d riderAngularVelocity = Eigen::Vector3d::Zero();
};

//==============================================================================
inline PhysicsContract inspectPhysicsContract(
    const std::shared_ptr<dart::simulation::World>& world,
    const ScenarioSpec& scenario,
    const std::string& solverContract,
    const std::string& binaryRole,
    const std::string& binarySourceSha256)
{
  if (!world)
    throw std::runtime_error("author turntable contract has no world");

  auto* solver
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  if (solver == nullptr)
    throw std::runtime_error("author turntable contract requires exact FBF");

  const auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          solver->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author turntable contract requires Native collision");

  const auto support = world->getSkeleton("turntable");
  const auto rider = world->getSkeleton("turntable_rider");
  if (!support || !rider || support->getNumBodyNodes() != 1u
      || rider->getNumBodyNodes() != 1u) {
    throw std::runtime_error("author turntable contract bodies are missing");
  }

  const auto* supportBody = support->getBodyNode(0);
  const auto* riderBody = rider->getBodyNode(0);
  const auto* supportNode
      = supportBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  const auto* riderNode
      = riderBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (supportNode == nullptr || riderNode == nullptr
      || supportNode->getDynamicsAspect() == nullptr
      || riderNode->getDynamicsAspect() == nullptr) {
    throw std::runtime_error(
        "author turntable contract collision shapes are missing");
  }

  const auto supportCylinder
      = std::dynamic_pointer_cast<const dart::dynamics::CylinderShape>(
          supportNode->getShape());
  const auto riderBox
      = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
          riderNode->getShape());
  if (!supportCylinder || !riderBox) {
    throw std::runtime_error(
        "author turntable contract shape types are not cylinder/box");
  }

  PhysicsContract contract;
  contract.scenario = &scenario;
  contract.solverContract = solverContract;
  contract.binaryRole = binaryRole;
  contract.binarySourceSha256 = binarySourceSha256;
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
  const auto& options = solver->getExactCoulombOptions();
  contract.maxOuterIterations = options.maxOuterIterations;
  contract.tolerance = options.tolerance;
  contract.innerMaxSweeps = options.innerMaxSweeps;
  contract.innerLocalIterations = options.innerLocalIterations;
  contract.stepSizeScale = options.stepSizeScale;
  contract.warmStartEnabled = options.enableWarmStart;
  contract.fallbackToBoxedLcpEnabled = options.fallbackToBoxedLcp;
  contract.projectedGradientRetryEnabled = options.enableProjectedGradientRetry;
  contract.denseResidualPolishEnabled = options.enableDenseResidualPolish;
  contract.contactRowOperatorEnabled = options.useContactRowDelassusOperator;
  contract.denseContactRowSnapshotEnabled
      = options.assembleDenseContactRowSnapshot;
  contract.supportShape = "cylinder";
  contract.supportMobile = support->isMobile();
  contract.supportRadius = supportCylinder->getRadius();
  contract.supportHeight = supportCylinder->getHeight();
  contract.supportFriction
      = supportNode->getDynamicsAspect()->getFrictionCoeff();
  contract.supportPose = supportBody->getWorldTransform();
  contract.riderShape = "box";
  contract.riderSize = riderBox->getSize();
  contract.riderFriction = riderNode->getDynamicsAspect()->getFrictionCoeff();
  contract.riderMass = riderBody->getInertia().getMass();
  contract.riderMoment = riderBody->getInertia().getMoment();
  contract.riderPose = riderBody->getWorldTransform();
  contract.riderLinearVelocity = riderBody->getLinearVelocity();
  contract.riderAngularVelocity = riderBody->getAngularVelocity();
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
inline std::string physicsContractJson(const PhysicsContract& contract)
{
  if (contract.scenario == nullptr)
    throw std::runtime_error("author turntable contract has no scenario");

  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema_version\":";
  writeJsonString(out, kContractSchema);
  out << ",\"kind\":";
  writeJsonString(out, kContractKind);
  out << ",\"author_source\":{\"commit\":";
  writeJsonString(out, kAuthorCommit);
  out << ",\"turntable_run_py_sha256\":";
  writeJsonString(out, kAuthorSceneSourceSha256);
  out << "},\"physics_spec_source_sha256\":";
  writeJsonString(out, kSpecSourceSha256);
  out << ",\"binary_binding\":{\"role\":";
  writeJsonString(out, contract.binaryRole);
  out << ",\"implementation_source_sha256\":";
  writeJsonString(out, contract.binarySourceSha256);
  out << "},\"scenario\":{\"trace_id\":";
  writeJsonString(out, contract.scenario->traceScenario);
  out << ",\"demo_scene_id\":";
  writeJsonString(out, contract.scenario->demoScene);
  out << ",\"friction\":" << contract.scenario->friction
      << ",\"angular_velocity_rad_s\":" << contract.scenario->angularVelocity
      << ",\"expected_outcome\":";
  writeJsonString(out, contract.scenario->expectedOutcome);
  out << "},\"world\":{\"time_step_seconds\":" << contract.timeStep
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
      << "},\"solver\":{\"type\":\"exact_fbf\",\"contract\":";
  writeJsonString(out, contract.solverContract);
  out << ",\"split_impulse_enabled\":"
      << (contract.splitImpulseEnabled ? "true" : "false")
      << ",\"max_outer_iterations\":" << contract.maxOuterIterations
      << ",\"tolerance\":" << contract.tolerance
      << ",\"inner_max_sweeps\":" << contract.innerMaxSweeps
      << ",\"inner_local_iterations\":" << contract.innerLocalIterations
      << ",\"step_size_scale\":" << contract.stepSizeScale
      << ",\"warm_start_enabled\":"
      << (contract.warmStartEnabled ? "true" : "false")
      << ",\"fallback_to_boxed_lcp_enabled\":"
      << (contract.fallbackToBoxedLcpEnabled ? "true" : "false")
      << ",\"projected_gradient_retry_enabled\":"
      << (contract.projectedGradientRetryEnabled ? "true" : "false")
      << ",\"dense_residual_polish_enabled\":"
      << (contract.denseResidualPolishEnabled ? "true" : "false")
      << ",\"contact_row_operator_enabled\":"
      << (contract.contactRowOperatorEnabled ? "true" : "false")
      << ",\"dense_contact_row_snapshot_enabled\":"
      << (contract.denseContactRowSnapshotEnabled ? "true" : "false")
      << "},\"support\":{\"shape\":";
  writeJsonString(out, contract.supportShape);
  out << ",\"mobile\":" << (contract.supportMobile ? "true" : "false")
      << ",\"radius_m\":" << contract.supportRadius
      << ",\"height_m\":" << contract.supportHeight
      << ",\"friction\":" << contract.supportFriction << ",\"initial_pose\":";
  writeJsonPose(out, contract.supportPose);
  out << "},\"rider\":{\"shape\":";
  writeJsonString(out, contract.riderShape);
  out << ",\"size_m\":";
  writeJsonVector(out, contract.riderSize);
  out << ",\"friction\":" << contract.riderFriction
      << ",\"mass_kg\":" << contract.riderMass << ",\"moment_kg_m2\":";
  writeJsonMatrix(out, contract.riderMoment);
  out << ",\"initial_pose\":";
  writeJsonPose(out, contract.riderPose);
  out << ",\"initial_linear_velocity_m_s\":";
  writeJsonVector(out, contract.riderLinearVelocity);
  out << ",\"initial_angular_velocity_rad_s\":";
  writeJsonVector(out, contract.riderAngularVelocity);
  out << "},\"control\":{\"settle_duration_seconds\":" << kSettleDuration
      << ",\"ramp_duration_seconds\":" << kRampDuration
      << ",\"duration_seconds\":" << kDuration
      << ",\"ramp\":\"cubic_smoothstep\"}"
      << ",\"visual_asset_identity\":null}";
  return out.str();
}

} // namespace fbf_author_turntable

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORTURNTABLESPEC_HPP_
