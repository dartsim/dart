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

#include "DemoHost.hpp"

#include "HeadlessExactFbfFailFast.hpp"
#include "Theme.hpp"

#include <dart/config.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/CollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/Viewport>
#include <osgGA/EventQueue>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>
#include <unordered_set>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

namespace dart_demos {

namespace {

constexpr int kDefaultWindowWidth = 1600;
constexpr int kDefaultWindowHeight = 1000;

//==============================================================================
std::string toLower(const std::string& value)
{
  std::string result = value;
  std::transform(
      result.begin(), result.end(), result.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
  return result;
}

//==============================================================================
bool matchesFilter(const DemoScene& scene, const std::string& filterLower)
{
  if (filterLower.empty())
    return true;

  return toLower(scene.id).find(filterLower) != std::string::npos
         || toLower(scene.title).find(filterLower) != std::string::npos
         || toLower(scene.summary).find(filterLower) != std::string::npos
         || toLower(scene.category).find(filterLower) != std::string::npos;
}

//==============================================================================
bool startsWith(const std::string& value, const char* prefix)
{
  const std::string prefixString(prefix);
  return value.size() >= prefixString.size()
         && value.compare(0, prefixString.size(), prefixString) == 0;
}

//==============================================================================
/// Renders "f - Shoot sphere" when the key is a plain printable character, or
/// just the label otherwise (e.g. arrow keys).
std::string formatKeyActionLabel(const KeyAction& action)
{
  if (action.key >= 32 && action.key < 127)
    return std::string(1, static_cast<char>(action.key)) + " - " + action.label;
  return action.label;
}

//==============================================================================
std::string formatKeyForMessage(int key)
{
  if (key >= 32 && key < 127)
    return std::string("'") + static_cast<char>(key) + "'";
  return std::to_string(key);
}

//==============================================================================
const char* exactSolverStatusLabel(
    dart::constraint::ExactCoulombFbfConstraintSolverStatus status)
{
  using dart::constraint::ExactCoulombFbfConstraintSolverStatus;
  switch (status) {
    case ExactCoulombFbfConstraintSolverStatus::NotRun:
      return "not_run";
    case ExactCoulombFbfConstraintSolverStatus::Success:
      return "success";
    case ExactCoulombFbfConstraintSolverStatus::MaxIterationsAccepted:
      return "max_iterations_accepted";
    case ExactCoulombFbfConstraintSolverStatus::PlateauAccepted:
      return "plateau_accepted";
    case ExactCoulombFbfConstraintSolverStatus::InvalidOptions:
      return "invalid_options";
    case ExactCoulombFbfConstraintSolverStatus::UnsupportedProblem:
      return "unsupported_problem";
    case ExactCoulombFbfConstraintSolverStatus::FbfFailed:
      return "fbf_failed";
    case ExactCoulombFbfConstraintSolverStatus::BoxedLcpFallback:
      return "boxed_lcp_fallback";
  }
  return "unknown";
}

//==============================================================================
const char* exactFbfStatusLabel(
    dart::math::detail::ExactCoulombFbfStatus status)
{
  using dart::math::detail::ExactCoulombFbfStatus;
  switch (status) {
    case ExactCoulombFbfStatus::Success:
      return "success";
    case ExactCoulombFbfStatus::MaxIterations:
      return "max_iterations";
    case ExactCoulombFbfStatus::InvalidInput:
      return "invalid_input";
    case ExactCoulombFbfStatus::InnerSolverFailed:
      return "inner_solver_failed";
    case ExactCoulombFbfStatus::StepSizeUnderflow:
      return "step_size_underflow";
    case ExactCoulombFbfStatus::Plateau:
      return "plateau";
    case ExactCoulombFbfStatus::NonFiniteValue:
      return "non_finite_value";
  }
  return "unknown";
}

//==============================================================================
const char* exactBuildStatusLabel(
    dart::constraint::detail::ExactCoulombConstraintBuildStatus status)
{
  using dart::constraint::detail::ExactCoulombConstraintBuildStatus;
  switch (status) {
    case ExactCoulombConstraintBuildStatus::Success:
      return "success";
    case ExactCoulombConstraintBuildStatus::EmptyInput:
      return "empty_input";
    case ExactCoulombConstraintBuildStatus::NullConstraint:
      return "null_constraint";
    case ExactCoulombConstraintBuildStatus::InvalidOptions:
      return "invalid_options";
    case ExactCoulombConstraintBuildStatus::UnsupportedDimension:
      return "unsupported_dimension";
    case ExactCoulombConstraintBuildStatus::UnsupportedBounds:
      return "unsupported_bounds";
    case ExactCoulombConstraintBuildStatus::UnsupportedFrictionCoupling:
      return "unsupported_friction_coupling";
    case ExactCoulombConstraintBuildStatus::UnsupportedAnisotropicFriction:
      return "unsupported_anisotropic_friction";
    case ExactCoulombConstraintBuildStatus::NonFiniteData:
      return "non_finite_data";
  }
  return "unknown";
}

//==============================================================================
void writeJsonString(std::ostream& out, const std::string& value)
{
  static constexpr char kHex[] = "0123456789abcdef";
  out << '"';
  for (const unsigned char c : value) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
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
        if (c < 0x20u) {
          out << "\\u00" << kHex[(c >> 4u) & 0x0fu] << kHex[c & 0x0fu];
        } else {
          out << static_cast<char>(c);
        }
        break;
    }
  }
  out << '"';
}

//==============================================================================
void writeJsonNumber(std::ostream& out, double value)
{
  if (std::isfinite(value))
    out << std::setprecision(17) << value;
  else
    out << "null";
}

//==============================================================================
void writeJsonIntArray(std::ostream& out, const std::vector<int>& values)
{
  out << '[';
  for (std::size_t index = 0u; index < values.size(); ++index) {
    if (index > 0u)
      out << ',';
    out << values[index];
  }
  out << ']';
}

//==============================================================================
struct SolverDiagnosticsSnapshot
{
  struct LastFailure
  {
    struct ColoredBlockGaussSeidel
    {
      bool enabled = false;
      bool participantAffinityEnabled = false;
      bool used = false;
      std::size_t solves = 0u;
      std::size_t dispatches = 0u;
      std::size_t participants = 0u;
      std::size_t manifolds = 0u;
      std::size_t colors = 0u;
      std::size_t maxManifoldsPerColor = 0u;
      std::vector<int> logicalCpuIds;
      std::vector<int> maxPhaseLogicalCpuIds;
    };

    bool available = false;
    std::size_t contacts = 0u;
    std::string status;
    std::string buildStatus;
    std::string fbfStatus;
    double residual = std::numeric_limits<double>::quiet_NaN();
    double primalFeasibility = std::numeric_limits<double>::quiet_NaN();
    double dualFeasibility = std::numeric_limits<double>::quiet_NaN();
    double complementarity = std::numeric_limits<double>::quiet_NaN();
    std::ptrdiff_t worstPrimalContact = -1;
    std::ptrdiff_t worstDualContact = -1;
    std::ptrdiff_t worstComplementarityContact = -1;
    double bestResidual = std::numeric_limits<double>::quiet_NaN();
    int bestIteration = 0;
    int iterations = 0;
    double stepSize = std::numeric_limits<double>::quiet_NaN();
    double safeStepSize = std::numeric_limits<double>::quiet_NaN();
    double couplingVariationRatio = std::numeric_limits<double>::quiet_NaN();
    int shrinkIterations = 0;
    ColoredBlockGaussSeidel coloredBlockGaussSeidel;
  };

  struct GroupOutcome
  {
    std::size_t solveIndex = 0u;
    std::size_t contacts = 0u;
    std::string status;
    std::string fbfStatus;
    int iterations = 0;
    int shrinkIterations = 0;
    double finalResidual = std::numeric_limits<double>::quiet_NaN();
    double finalNaturalMapResidual = std::numeric_limits<double>::quiet_NaN();
    double plateauReferenceNaturalMapResidual
        = std::numeric_limits<double>::quiet_NaN();
    double plateauRelativeImprovement
        = std::numeric_limits<double>::quiet_NaN();
    int lineSearchShrinkCapCount = 0;
    double correctionStepSize = std::numeric_limits<double>::quiet_NaN();
    double lastInnerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
    bool sourceContinuationActive = false;
  };

  std::string solver = "UnknownConstraintSolver";
  bool available = false;
  std::string gap;
  std::string status;
  std::string fbfStatus;
  double residual = std::numeric_limits<double>::quiet_NaN();
  double bestResidual = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  std::size_t totalIterations = 0u;
  std::size_t exactSolves = 0u;
  std::size_t exactAttempts = 0u;
  std::size_t maxIterationsAccepted = 0u;
  double worstResidual = std::numeric_limits<double>::quiet_NaN();
  std::size_t exactFailures = 0u;
  std::size_t boxedLcpFallbacks = 0u;
  std::size_t warmStarts = 0u;
  std::size_t coloredBlockGaussSeidelSolves = 0u;
  std::size_t contacts = 0u;
  bool sourceContinuationRequested = false;
  bool sourceContinuationLastActive = false;
  bool worldStateFinite = false;
  bool groupHistoryTruncated = false;
  bool lastLineSearchShrinkCapReached = false;
  int lastLineSearchShrinkCapCount = 0;
  double lastCorrectionStepSize = std::numeric_limits<double>::quiet_NaN();
  double lastInnerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
  std::size_t plateausAccepted = 0u;
  std::size_t lineSearchShrinkCaps = 0u;
  std::size_t stepExactAttempts = 0u;
  std::size_t stepExactSolves = 0u;
  std::size_t stepPlateausAccepted = 0u;
  std::size_t stepMaxIterationsAccepted = 0u;
  std::size_t stepLineSearchShrinks = 0u;
  std::size_t stepLineSearchShrinkCaps = 0u;
  std::vector<GroupOutcome> groupOutcomes;
  LastFailure lastFailure;
};

//==============================================================================
struct SolverDiagnosticsCursor
{
  std::size_t nextGroupSolveIndex = 0u;
  std::size_t exactAttempts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t plateausAccepted = 0u;
  std::size_t maxIterationsAccepted = 0u;
  std::size_t lineSearchShrinkCaps = 0u;
};

//==============================================================================
bool isWorldStateFinite(const dart::simulation::WorldPtr& world)
{
  if (!world || !std::isfinite(world->getTime())
      || !std::isfinite(world->getTimeStep())
      || !world->getGravity().allFinite()) {
    return false;
  }
  for (std::size_t index = 0u; index < world->getNumSkeletons(); ++index) {
    const auto skeleton = world->getSkeleton(index);
    if (!skeleton || !skeleton->getPositions().allFinite()
        || !skeleton->getVelocities().allFinite()) {
      return false;
    }
  }
  return true;
}

//==============================================================================
SolverDiagnosticsSnapshot captureSolverDiagnostics(
    const dart::simulation::WorldPtr& world,
    SolverDiagnosticsCursor* cursor = nullptr)
{
  SolverDiagnosticsSnapshot snapshot;
  if (!world || !world->getConstraintSolver()) {
    snapshot.solver = "UnavailableConstraintSolver";
    snapshot.gap = "active world has no constraint solver";
    return snapshot;
  }

  const auto* solver
      = dynamic_cast<const dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  if (!solver) {
    if (dynamic_cast<const dart::constraint::BoxedLcpConstraintSolver*>(
            world->getConstraintSolver())) {
      snapshot.solver = "BoxedLcpConstraintSolver";
    }
    snapshot.gap
        = "active solver does not expose exact-Coulomb FBF diagnostics";
    return snapshot;
  }

  snapshot.solver = "ExactCoulombFbfConstraintSolver";
  snapshot.available = true;
  snapshot.contacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  const auto solverStatus = solver->getLastExactCoulombStatus();
  snapshot.status = exactSolverStatusLabel(solverStatus);
  snapshot.fbfStatus
      = solverStatus
                == dart::constraint::ExactCoulombFbfConstraintSolverStatus::
                    NotRun
            ? "not_run"
            : exactFbfStatusLabel(solver->getLastExactCoulombFbfStatus());
  snapshot.residual = solver->getLastExactCoulombResidual();
  snapshot.bestResidual = solver->getLastExactCoulombBestResidual();
  snapshot.iterations = solver->getLastExactCoulombIterations();
  snapshot.totalIterations = solver->getTotalExactCoulombIterations();
  snapshot.exactSolves = solver->getNumExactCoulombSolves();
  snapshot.exactAttempts = solver->getNumExactCoulombAttempts();
  snapshot.maxIterationsAccepted
      = solver->getNumExactCoulombMaxIterationsAccepted();
  snapshot.worstResidual = solver->getWorstExactCoulombResidual();
  snapshot.exactFailures = solver->getNumExactCoulombFailures();
  snapshot.boxedLcpFallbacks = solver->getNumBoxedLcpFallbacks();
  snapshot.warmStarts = solver->getNumExactCoulombWarmStarts();
  snapshot.coloredBlockGaussSeidelSolves
      = solver->getNumExactCoulombColoredBlockGaussSeidelSolves();
  const auto continuationOptions
      = solver->getExactCoulombSourceContinuationOptions();
  snapshot.sourceContinuationRequested = continuationOptions.enabled;
  snapshot.sourceContinuationLastActive
      = solver->getLastExactCoulombSourceContinuationActive();
  snapshot.worldStateFinite = isWorldStateFinite(world);
  snapshot.lastLineSearchShrinkCapReached
      = solver->getLastExactCoulombLineSearchShrinkCapReached();
  snapshot.lastLineSearchShrinkCapCount
      = solver->getLastExactCoulombLineSearchShrinkCapCount();
  snapshot.lastCorrectionStepSize = solver->getLastExactCoulombStepSize();
  snapshot.lastInnerSolveStepSize
      = solver->getLastExactCoulombInnerSolveStepSize();
  snapshot.plateausAccepted = solver->getNumExactCoulombPlateausAccepted();
  snapshot.lineSearchShrinkCaps
      = solver->getNumExactCoulombLineSearchShrinkCaps();

  if (cursor != nullptr) {
    const auto delta =
        [&snapshot](std::size_t current, std::size_t previous) -> std::size_t {
      if (current < previous) {
        snapshot.groupHistoryTruncated = true;
        return 0u;
      }
      return current - previous;
    };
    snapshot.stepExactAttempts
        = delta(snapshot.exactAttempts, cursor->exactAttempts);
    snapshot.stepExactSolves = delta(snapshot.exactSolves, cursor->exactSolves);
    snapshot.stepPlateausAccepted
        = delta(snapshot.plateausAccepted, cursor->plateausAccepted);
    snapshot.stepMaxIterationsAccepted
        = delta(snapshot.maxIterationsAccepted, cursor->maxIterationsAccepted);
    snapshot.stepLineSearchShrinkCaps
        = delta(snapshot.lineSearchShrinkCaps, cursor->lineSearchShrinkCaps);

    const std::size_t expectedFirstSolveIndex = cursor->nextGroupSolveIndex;
    const auto& records = solver->getExactCoulombResidualHistoryRecords();
    if (!records.empty()
        && records.front().solveIndex > cursor->nextGroupSolveIndex) {
      snapshot.groupHistoryTruncated = true;
    }
    for (const auto& record : records) {
      if (record.solveIndex < cursor->nextGroupSolveIndex)
        continue;
      SolverDiagnosticsSnapshot::GroupOutcome outcome;
      outcome.solveIndex = record.solveIndex;
      outcome.contacts = record.contactCount;
      outcome.status = exactSolverStatusLabel(record.status);
      outcome.fbfStatus = exactFbfStatusLabel(record.fbfStatus);
      outcome.iterations = record.iterations;
      outcome.shrinkIterations = record.shrinkIterations;
      outcome.finalResidual = record.finalResidual;
      outcome.finalNaturalMapResidual = record.finalNaturalMapResidual;
      outcome.plateauReferenceNaturalMapResidual
          = record.plateauReferenceNaturalMapResidual;
      outcome.plateauRelativeImprovement = record.plateauRelativeImprovement;
      outcome.lineSearchShrinkCapCount = record.lineSearchShrinkCapCount;
      outcome.lastInnerSolveStepSize = record.lastInnerSolveStepSize;
      if (!record.samples.empty())
        outcome.correctionStepSize = record.samples.back().stepSize;
      outcome.sourceContinuationActive = record.sourceContinuationActive;
      snapshot.groupOutcomes.push_back(std::move(outcome));
    }
    for (const auto& outcome : snapshot.groupOutcomes) {
      if (outcome.shrinkIterations >= 0) {
        snapshot.stepLineSearchShrinks
            += static_cast<std::size_t>(outcome.shrinkIterations);
      }
    }
    if (!snapshot.groupOutcomes.empty()
        && snapshot.groupOutcomes.front().solveIndex
               != expectedFirstSolveIndex) {
      snapshot.groupHistoryTruncated = true;
    }
    if (!records.empty())
      cursor->nextGroupSolveIndex = records.back().solveIndex + 1u;
    cursor->exactAttempts = snapshot.exactAttempts;
    cursor->exactSolves = snapshot.exactSolves;
    cursor->plateausAccepted = snapshot.plateausAccepted;
    cursor->maxIterationsAccepted = snapshot.maxIterationsAccepted;
    cursor->lineSearchShrinkCaps = snapshot.lineSearchShrinkCaps;
  }
  if (snapshot.exactFailures > 0u) {
    const auto& residual = solver->getLastFailedExactCoulombResidualDetails();
    snapshot.lastFailure.available = true;
    snapshot.lastFailure.contacts
        = solver->getLastFailedExactCoulombContactCount();
    snapshot.lastFailure.status
        = exactSolverStatusLabel(solver->getLastFailedExactCoulombStatus());
    snapshot.lastFailure.buildStatus
        = exactBuildStatusLabel(solver->getLastFailedExactCoulombBuildStatus());
    snapshot.lastFailure.fbfStatus
        = exactFbfStatusLabel(solver->getLastFailedExactCoulombFbfStatus());
    snapshot.lastFailure.residual = solver->getLastFailedExactCoulombResidual();
    snapshot.lastFailure.primalFeasibility = residual.primalFeasibility;
    snapshot.lastFailure.dualFeasibility = residual.dualFeasibility;
    snapshot.lastFailure.complementarity = residual.complementarity;
    snapshot.lastFailure.worstPrimalContact = residual.worstPrimalContact;
    snapshot.lastFailure.worstDualContact = residual.worstDualContact;
    snapshot.lastFailure.worstComplementarityContact
        = residual.worstComplementarityContact;
    snapshot.lastFailure.bestResidual
        = solver->getLastFailedExactCoulombBestResidual();
    snapshot.lastFailure.bestIteration
        = solver->getLastFailedExactCoulombBestIteration();
    snapshot.lastFailure.iterations
        = solver->getLastFailedExactCoulombIterations();
    snapshot.lastFailure.stepSize = solver->getLastFailedExactCoulombStepSize();
    snapshot.lastFailure.safeStepSize
        = solver->getLastFailedExactCoulombSafeStepSize();
    snapshot.lastFailure.couplingVariationRatio
        = solver->getLastFailedExactCoulombCouplingVariationRatio();
    snapshot.lastFailure.shrinkIterations
        = solver->getLastFailedExactCoulombShrinkIterations();
    auto& colored = snapshot.lastFailure.coloredBlockGaussSeidel;
    colored.enabled
        = solver->getLastFailedExactCoulombColoredBlockGaussSeidelEnabled();
    colored.participantAffinityEnabled
        = solver
              ->getLastFailedExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled();
    colored.used
        = solver->getLastFailedExactCoulombColoredBlockGaussSeidelUsed();
    colored.solves
        = solver->getLastFailedExactCoulombColoredBlockGaussSeidelSolves();
    colored.dispatches
        = solver->getLastFailedExactCoulombColoredBlockGaussSeidelDispatches();
    colored.participants
        = solver
              ->getLastFailedExactCoulombColoredBlockGaussSeidelParticipants();
    colored.manifolds
        = solver->getLastFailedExactCoulombColoredBlockGaussSeidelManifolds();
    colored.colors
        = solver->getLastFailedExactCoulombColoredBlockGaussSeidelColors();
    colored.maxManifoldsPerColor
        = solver
              ->getLastFailedExactCoulombColoredBlockGaussSeidelMaxManifoldsPerColor();
    colored.logicalCpuIds
        = solver
              ->getLastFailedExactCoulombColoredBlockGaussSeidelLogicalCpuIds();
    colored.maxPhaseLogicalCpuIds
        = solver
              ->getLastFailedExactCoulombColoredBlockGaussSeidelMaxPhaseLogicalCpuIds();
  }
  return snapshot;
}

//==============================================================================
void writeSolverDiagnosticsJson(
    std::ostream& out, const SolverDiagnosticsSnapshot& snapshot)
{
  out << "{\n      \"solver\": ";
  writeJsonString(out, snapshot.solver);
  out << ",\n      \"available\": " << (snapshot.available ? "true" : "false");
  if (!snapshot.available) {
    out << ",\n      \"gap\": ";
    writeJsonString(out, snapshot.gap);
    out << "\n    }";
    return;
  }

  out << ",\n      \"status\": ";
  writeJsonString(out, snapshot.status);
  out << ",\n      \"fbf_status\": ";
  writeJsonString(out, snapshot.fbfStatus);
  out << ",\n      \"residual\": ";
  writeJsonNumber(out, snapshot.residual);
  out << ",\n      \"best_residual\": ";
  writeJsonNumber(out, snapshot.bestResidual);
  out << ",\n      \"iterations\": " << snapshot.iterations
      << ",\n      \"total_iterations\": " << snapshot.totalIterations
      << ",\n      \"exact_solves\": " << snapshot.exactSolves
      << ",\n      \"exact_attempts\": " << snapshot.exactAttempts
      << ",\n      \"accepted_at_cap\": " << snapshot.maxIterationsAccepted
      << ",\n      \"worst_residual\": ";
  writeJsonNumber(out, snapshot.worstResidual);
  out << ",\n      \"exact_failures\": " << snapshot.exactFailures
      << ",\n      \"boxed_lcp_fallbacks\": " << snapshot.boxedLcpFallbacks
      << ",\n      \"warm_starts\": " << snapshot.warmStarts
      << ",\n      \"colored_block_gauss_seidel\": {\"solves\": "
      << snapshot.coloredBlockGaussSeidelSolves << "}"
      << ",\n      \"contacts\": " << snapshot.contacts;
  if (snapshot.sourceContinuationRequested) {
    out << ",\n      \"source_continuation\": {"
        << "\n        \"requested\": true"
        << ",\n        \"last_active\": "
        << (snapshot.sourceContinuationLastActive ? "true" : "false")
        << ",\n        \"world_state_finite\": "
        << (snapshot.worldStateFinite ? "true" : "false")
        << ",\n        \"group_history_truncated\": "
        << (snapshot.groupHistoryTruncated ? "true" : "false")
        << ",\n        \"cumulative\": {\"plateaus_accepted\": "
        << snapshot.plateausAccepted
        << ", \"max_iterations_accepted\": " << snapshot.maxIterationsAccepted
        << ", \"line_search_shrink_caps\": " << snapshot.lineSearchShrinkCaps
        << "}"
        << ",\n        \"step\": {\"exact_attempts\": "
        << snapshot.stepExactAttempts
        << ", \"exact_solves\": " << snapshot.stepExactSolves
        << ", \"plateaus_accepted\": " << snapshot.stepPlateausAccepted
        << ", \"max_iterations_accepted\": "
        << snapshot.stepMaxIterationsAccepted
        << ", \"line_search_shrinks\": " << snapshot.stepLineSearchShrinks
        << ", \"line_search_shrink_caps\": "
        << snapshot.stepLineSearchShrinkCaps << "}"
        << ",\n        \"last_attempt\": {"
           "\"line_search_shrink_cap_reached\": "
        << (snapshot.lastLineSearchShrinkCapReached ? "true" : "false")
        << ", \"line_search_shrink_cap_count\": "
        << snapshot.lastLineSearchShrinkCapCount
        << ", \"correction_step_size\": ";
    writeJsonNumber(out, snapshot.lastCorrectionStepSize);
    out << ", \"last_inner_solve_step_size\": ";
    writeJsonNumber(out, snapshot.lastInnerSolveStepSize);
    out << "}"
        << ",\n        \"group_outcomes\": [";
    for (std::size_t index = 0u; index < snapshot.groupOutcomes.size();
         ++index) {
      const auto& group = snapshot.groupOutcomes[index];
      out << (index == 0u ? "\n" : ",\n")
          << "          {\"solve_index\": " << group.solveIndex
          << ", \"contact_count\": " << group.contacts << ", \"status\": ";
      writeJsonString(out, group.status);
      out << ", \"fbf_status\": ";
      writeJsonString(out, group.fbfStatus);
      out << ", \"iterations\": " << group.iterations
          << ", \"line_search_shrinks\": " << group.shrinkIterations
          << ", \"final_residual\": ";
      writeJsonNumber(out, group.finalResidual);
      out << ", \"final_natural_map_residual\": ";
      writeJsonNumber(out, group.finalNaturalMapResidual);
      out << ", \"plateau_reference_natural_map_residual\": ";
      writeJsonNumber(out, group.plateauReferenceNaturalMapResidual);
      out << ", \"plateau_relative_improvement\": ";
      writeJsonNumber(out, group.plateauRelativeImprovement);
      out << ", \"line_search_shrink_cap_count\": "
          << group.lineSearchShrinkCapCount << ", \"correction_step_size\": ";
      writeJsonNumber(out, group.correctionStepSize);
      out << ", \"last_inner_solve_step_size\": ";
      writeJsonNumber(out, group.lastInnerSolveStepSize);
      out << ", \"source_continuation_active\": "
          << (group.sourceContinuationActive ? "true" : "false") << '}';
    }
    if (!snapshot.groupOutcomes.empty())
      out << '\n';
    out << "        ]\n      }";
  }
  out << ",\n      \"last_failure\": ";
  if (!snapshot.lastFailure.available) {
    out << "null\n    }";
    return;
  }

  const auto& failure = snapshot.lastFailure;
  out << "{\n        \"contact_count\": " << failure.contacts
      << ",\n        \"status\": ";
  writeJsonString(out, failure.status);
  out << ",\n        \"build_status\": ";
  writeJsonString(out, failure.buildStatus);
  out << ",\n        \"fbf_status\": ";
  writeJsonString(out, failure.fbfStatus);
  out << ",\n        \"residual\": ";
  writeJsonNumber(out, failure.residual);
  out << ",\n        \"primal_feasibility\": ";
  writeJsonNumber(out, failure.primalFeasibility);
  out << ",\n        \"dual_feasibility\": ";
  writeJsonNumber(out, failure.dualFeasibility);
  out << ",\n        \"complementarity\": ";
  writeJsonNumber(out, failure.complementarity);
  out << ",\n        \"worst_primal_contact\": " << failure.worstPrimalContact
      << ",\n        \"worst_dual_contact\": " << failure.worstDualContact
      << ",\n        \"worst_complementarity_contact\": "
      << failure.worstComplementarityContact
      << ",\n        \"best_residual\": ";
  writeJsonNumber(out, failure.bestResidual);
  out << ",\n        \"best_iteration\": " << failure.bestIteration
      << ",\n        \"iterations\": " << failure.iterations
      << ",\n        \"step_size\": ";
  writeJsonNumber(out, failure.stepSize);
  out << ",\n        \"safe_step_size\": ";
  writeJsonNumber(out, failure.safeStepSize);
  out << ",\n        \"coupling_variation_ratio\": ";
  writeJsonNumber(out, failure.couplingVariationRatio);
  out << ",\n        \"shrink_iterations\": " << failure.shrinkIterations;
  const auto& colored = failure.coloredBlockGaussSeidel;
  out << ",\n        \"colored_block_gauss_seidel\": {"
      << "\n          \"enabled\": " << (colored.enabled ? "true" : "false")
      << ",\n          \"participant_affinity_enabled\": "
      << (colored.participantAffinityEnabled ? "true" : "false")
      << ",\n          \"used\": " << (colored.used ? "true" : "false")
      << ",\n          \"solves\": " << colored.solves
      << ",\n          \"dispatches\": " << colored.dispatches
      << ",\n          \"participants\": " << colored.participants
      << ",\n          \"manifolds\": " << colored.manifolds
      << ",\n          \"colors\": " << colored.colors
      << ",\n          \"max_manifolds_per_color\": "
      << colored.maxManifoldsPerColor << ",\n          \"logical_cpu_ids\": ";
  writeJsonIntArray(out, colored.logicalCpuIds);
  out << ",\n          \"max_phase_logical_cpu_ids\": ";
  writeJsonIntArray(out, colored.maxPhaseLogicalCpuIds);
  out << "\n        }\n      }\n    }";
}

//==============================================================================
struct HeadlessShotResult
{
  std::size_t sequence = 0u;
  std::size_t step = 0u;
  std::string path;
  double simTime = 0.0;
  bool success = false;
  SolverDiagnosticsSnapshot diagnostics;
};

//==============================================================================
struct HeadlessStepResult
{
  std::size_t step = 0u;
  double simTime = 0.0;
  std::optional<SceneStateFields> sceneState;
  SolverDiagnosticsSnapshot diagnostics;
};

//==============================================================================
struct HeadlessActionResult
{
  std::size_t sequence = 0u;
  std::size_t step = 0u;
  int key = 0;
  bool success = false;
};

//==============================================================================
struct HeadlessExactFbfFailFastState
{
  bool triggered = false;
  std::size_t step = 0u;
  const char* reason = nullptr;
};

//==============================================================================
detail::HeadlessExactFbfSourceContinuationDiagnostics
makeSourceContinuationGateDiagnostics(const SolverDiagnosticsSnapshot& snapshot)
{
  detail::HeadlessExactFbfSourceContinuationDiagnostics diagnostics;
  diagnostics.requested = snapshot.sourceContinuationRequested;
  diagnostics.lastActive = snapshot.sourceContinuationLastActive;
  diagnostics.worldStateFinite = snapshot.worldStateFinite;
  diagnostics.groupHistoryTruncated = snapshot.groupHistoryTruncated;
  diagnostics.lastLineSearchShrinkCapReached
      = snapshot.lastLineSearchShrinkCapReached;
  diagnostics.lastLineSearchShrinkCapCount
      = snapshot.lastLineSearchShrinkCapCount;
  diagnostics.lastCorrectionStepSize = snapshot.lastCorrectionStepSize;
  diagnostics.lastInnerSolveStepSize = snapshot.lastInnerSolveStepSize;
  diagnostics.exactAttempts = snapshot.exactAttempts;
  diagnostics.exactSolves = snapshot.exactSolves;
  diagnostics.exactFailures = snapshot.exactFailures;
  diagnostics.boxedFallbacks = snapshot.boxedLcpFallbacks;
  diagnostics.stepExactAttempts = snapshot.stepExactAttempts;
  diagnostics.stepExactSolves = snapshot.stepExactSolves;
  diagnostics.stepPlateausAccepted = snapshot.stepPlateausAccepted;
  diagnostics.stepMaxIterationsAccepted = snapshot.stepMaxIterationsAccepted;
  diagnostics.stepLineSearchShrinks = snapshot.stepLineSearchShrinks;
  diagnostics.stepLineSearchShrinkCaps = snapshot.stepLineSearchShrinkCaps;
  diagnostics.groups.reserve(snapshot.groupOutcomes.size());
  for (const auto& group : snapshot.groupOutcomes) {
    detail::HeadlessExactFbfSourceContinuationGroupDiagnostics item;
    item.solveIndex = group.solveIndex;
    item.contactCount = group.contacts;
    item.sourceContinuationActive = group.sourceContinuationActive;
    item.iterations = group.iterations;
    item.shrinkIterations = group.shrinkIterations;
    item.lineSearchShrinkCapCount = group.lineSearchShrinkCapCount;
    item.finalResidual = group.finalResidual;
    item.finalNaturalMapResidual = group.finalNaturalMapResidual;
    item.plateauReferenceNaturalMapResidual
        = group.plateauReferenceNaturalMapResidual;
    item.plateauRelativeImprovement = group.plateauRelativeImprovement;
    item.correctionStepSize = group.correctionStepSize;
    item.lastInnerSolveStepSize = group.lastInnerSolveStepSize;
    if (group.status == "success" && group.fbfStatus == "success") {
      item.outcome = detail::HeadlessExactFbfSourceContinuationOutcome::Success;
    } else if (
        group.status == "plateau_accepted" && group.fbfStatus == "plateau") {
      item.outcome
          = detail::HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted;
    } else if (
        group.status == "max_iterations_accepted"
        && group.fbfStatus == "max_iterations") {
      item.outcome = detail::HeadlessExactFbfSourceContinuationOutcome::
          MaxIterationsAccepted;
    }
    diagnostics.groups.push_back(item);
  }
  return diagnostics;
}

//==============================================================================
const char* buildModeLabel()
{
#if DART_BUILD_MODE_DEBUG
  return "Debug";
#elif DART_BUILD_MODE_RELEASE
  return "Release";
#else
  return "Unknown";
#endif
}

//==============================================================================
bool writeHeadlessSidecar(
    const std::string& path,
    const std::string& scene,
    const std::string& activeScene,
    const std::string& runtimeCommand,
    std::size_t totalSteps,
    std::size_t completedSteps,
    int width,
    int height,
    const std::string& collisionDetector,
    const std::string& physicsContractJson,
    const std::vector<HeadlessStepResult>& steps,
    const std::vector<HeadlessShotResult>& shots,
    const std::vector<HeadlessActionResult>& actions,
    const SolverDiagnosticsSnapshot& finalDiagnostics,
    const HeadlessExactFbfFailFastState* failFast,
    const HeadlessExactFbfFailFastState* sourceContinuation)
{
  std::ofstream out(path);
  if (!out) {
    std::cerr << "[headless] failed to open sidecar '" << path
              << "' for writing.\n";
    return false;
  }

  out << "{\n  \"schema_version\": ";
  writeJsonString(out, "dart.demos_headless_timeline/v1");
  out << ",\n  \"scene\": ";
  writeJsonString(out, scene);
  out << ",\n  \"active_scene\": ";
  writeJsonString(out, activeScene);
  out << ",\n  \"runtime_command\": ";
  writeJsonString(out, runtimeCommand);
  out << ",\n  \"build\": {\n    \"dart_version\": ";
  writeJsonString(out, DART_VERSION);
  out << ",\n    \"mode\": ";
  writeJsonString(out, buildModeLabel());
  out << "\n  },\n  \"total_steps\": " << totalSteps
      << ",\n  \"completed_steps\": " << completedSteps
      << ",\n  \"width\": " << width << ",\n  \"height\": " << height
      << ",\n  \"collision_detector\": ";
  writeJsonString(out, collisionDetector);
  out << ",\n  \"physics_contract\": ";
  if (physicsContractJson.empty())
    out << "null";
  else
    out << physicsContractJson;
  out << ",\n  \"event_order\": ";
  writeJsonString(out, "captures_before_actions_at_each_completed_step");
  if (failFast) {
    out << ",\n  \"headless_exact_fbf_fail_fast\": {\n"
        << "    \"enabled\": true,\n    \"residual_tolerance\": ";
    writeJsonNumber(out, detail::kHeadlessExactFbfResidualTolerance);
    out << ",\n    \"triggered\": " << (failFast->triggered ? "true" : "false")
        << ",\n    \"step\": ";
    if (failFast->triggered)
      out << failFast->step;
    else
      out << "null";
    out << ",\n    \"reason\": ";
    if (failFast->triggered)
      writeJsonString(out, failFast->reason);
    else
      out << "null";
    out << "\n  }";
  }
  if (sourceContinuation) {
    out << ",\n  \"headless_exact_fbf_source_continuation\": {\n"
        << "    \"enabled\": true,\n    \"requested\": ";
    writeJsonString(out, "source_continuation");
    out << ",\n    \"allowed_outcomes\": [\"success\", "
           "\"plateau_accepted\", \"max_iterations_accepted\"]"
        << ",\n    \"line_search_cap_action\": \"shrink_cap\""
        << ",\n    \"triggered\": "
        << (sourceContinuation->triggered ? "true" : "false")
        << ",\n    \"step\": ";
    if (sourceContinuation->triggered)
      out << sourceContinuation->step;
    else
      out << "null";
    out << ",\n    \"reason\": ";
    if (sourceContinuation->triggered)
      writeJsonString(out, sourceContinuation->reason);
    else
      out << "null";
    out << "\n  }";
  }

  out << ",\n  \"steps\": [";
  for (std::size_t i = 0u; i < steps.size(); ++i) {
    const auto& step = steps[i];
    out << (i == 0u ? "\n" : ",\n") << "    {\"step\": " << step.step
        << ", \"sim_time\": ";
    writeJsonNumber(out, step.simTime);
    if (step.sceneState.has_value()) {
      out << ", \"scene_state\": {";
      for (std::size_t fieldIndex = 0u; fieldIndex < step.sceneState->size();
           ++fieldIndex) {
        const auto& field = (*step.sceneState)[fieldIndex];
        if (fieldIndex != 0u)
          out << ',';
        writeJsonString(out, field.first);
        out << ':';
        writeJsonNumber(out, field.second);
      }
      out << '}';
    }
    out << ", \"solver_diagnostics\": ";
    writeSolverDiagnosticsJson(out, step.diagnostics);
    out << '}';
  }
  if (!steps.empty())
    out << '\n';
  out << "  ]";

  out << ",\n  \"shots\": [";
  for (std::size_t i = 0u; i < shots.size(); ++i) {
    const auto& shot = shots[i];
    out << (i == 0u ? "\n" : ",\n") << "    {\"sequence\": " << shot.sequence
        << ", \"step\": " << shot.step << ", \"path\": ";
    writeJsonString(out, shot.path);
    out << ", \"sim_time\": ";
    writeJsonNumber(out, shot.simTime);
    out << ", \"success\": " << (shot.success ? "true" : "false")
        << ", \"solver_diagnostics\": ";
    writeSolverDiagnosticsJson(out, shot.diagnostics);
    out << '}';
  }
  if (!shots.empty())
    out << '\n';
  out << "  ],\n  \"actions\": [";
  for (std::size_t i = 0u; i < actions.size(); ++i) {
    const auto& action = actions[i];
    out << (i == 0u ? "\n" : ",\n") << "    {\"sequence\": " << action.sequence
        << ", \"step\": " << action.step << ", \"key\": ";
    writeJsonString(out, std::string(1, static_cast<char>(action.key)));
    out << ", \"key_code\": " << action.key
        << ", \"success\": " << (action.success ? "true" : "false") << '}';
  }
  if (!actions.empty())
    out << '\n';
  out << "  ],\n  \"events\": [";

  std::size_t shotIndex = 0u;
  std::size_t actionIndex = 0u;
  bool firstEvent = true;
  while (shotIndex < shots.size() || actionIndex < actions.size()) {
    const bool takeShot
        = actionIndex >= actions.size()
          || (shotIndex < shots.size()
              && shots[shotIndex].sequence < actions[actionIndex].sequence);
    out << (firstEvent ? "\n" : ",\n") << "    {";
    firstEvent = false;
    if (takeShot) {
      const auto& shot = shots[shotIndex++];
      out << "\"sequence\": " << shot.sequence
          << ", \"type\": \"shot\", \"step\": " << shot.step << ", \"path\": ";
      writeJsonString(out, shot.path);
      out << ", \"success\": " << (shot.success ? "true" : "false");
    } else {
      const auto& action = actions[actionIndex++];
      out << "\"sequence\": " << action.sequence
          << ", \"type\": \"action\", \"step\": " << action.step
          << ", \"key\": ";
      writeJsonString(out, std::string(1, static_cast<char>(action.key)));
      out << ", \"success\": " << (action.success ? "true" : "false");
    }
    out << '}';
  }
  if (!firstEvent)
    out << '\n';
  out << "  ],\n  \"solver_diagnostics\": ";
  writeSolverDiagnosticsJson(out, finalDiagnostics);
  out << "\n}\n";

  if (!out.good()) {
    std::cerr << "[headless] failed while writing sidecar '" << path << "'.\n";
    return false;
  }
  return true;
}

//==============================================================================
bool renderDocumentationSection(
    const char* label, const std::string& text, bool hasPrevious)
{
  if (text.empty())
    return false;

  if (hasPrevious)
    ImGui::Spacing();
  ImGui::TextDisabled("%s", label);
  ImGui::TextWrapped("%s", text.c_str());
  return true;
}

//==============================================================================
bool renderScenePanelDocumentation(const ScenePanelDocumentation& documentation)
{
  bool rendered = false;
  rendered |= renderDocumentationSection(
      "Overview", documentation.overview, rendered);
  rendered |= renderDocumentationSection(
      "Expected Result", documentation.expectedResult, rendered);
  rendered |= renderDocumentationSection(
      "Coverage", documentation.coverage, rendered);

  if (rendered)
    ImGui::Separator();
  return rendered;
}

//==============================================================================
/// True if `worldNode`'s RTF stats reflect real real-time-driven refreshes
/// (i.e. RealTimeWorldNode::refresh() has actually run with simulate() on) and
/// are therefore meaningful to display. Deterministic stepping paths --
/// headless capture, --cycle-scenes, the paused/Step-button path -- all drive
/// the world directly via DemoWorldNode::stepOnce(), bypassing refresh()'s
/// wall-clock budget entirely, so mLowestRealTimeFactor is left at its
/// never-updated +infinity default (and the smoothed/highest fields would
/// otherwise misleadingly read 0.00 instead of "not measured").
bool hasLiveRtfStats(
    const dart::gui::osg::ImGuiViewer* viewer, const DemoWorldNode* worldNode)
{
  return worldNode && viewer->isSimulating()
         && std::isfinite(worldNode->getLowestRealTimeFactor());
}

//==============================================================================
float calcButtonWidth(const char* label)
{
  const ImGuiStyle& style = ImGui::GetStyle();
  return ImGui::CalcTextSize(label).x + 2.0f * style.FramePadding.x;
}

//==============================================================================
void sameLineIfEnoughRoom(float nextItemWidth)
{
  const ImGuiStyle& style = ImGui::GetStyle();
  const float windowRight
      = ImGui::GetWindowPos().x + ImGui::GetContentRegionMax().x;
  const float nextRight
      = ImGui::GetItemRectMax().x + style.ItemSpacing.x + nextItemWidth;
  if (nextRight <= windowRight)
    ImGui::SameLine();
}

//==============================================================================
float clampedItemWidth(float preferred, float minimum)
{
  const float avail = std::max(1.0f, ImGui::GetContentRegionAvail().x);
  return std::clamp(preferred, std::min(minimum, avail), avail);
}

//==============================================================================
std::vector<std::string> getAvailableCollisionDetectorNames()
{
  std::vector<std::string> names;
  auto* factory = dart::collision::CollisionDetector::getFactory();
  if (!factory)
    return names;

  const auto keys = factory->getKeys();
  names.reserve(keys.size());
  for (const auto& key : keys)
    names.push_back(key);

  std::sort(names.begin(), names.end());
  return names;
}

//==============================================================================
std::string getWorldCollisionDetectorName(
    const dart::simulation::WorldPtr& world)
{
  if (!world)
    return "none";

  const auto detector = world->getCollisionDetector();
  return detector ? detector->getType() : "none";
}

//==============================================================================
struct SceneCounts
{
  std::size_t skeletons = 0;
  std::size_t bodyNodes = 0;
  std::size_t softBodies = 0;
  std::size_t pointMasses = 0;
};

//==============================================================================
SceneCounts collectSceneCounts(const dart::simulation::WorldPtr& world)
{
  SceneCounts counts;
  if (!world)
    return counts;

  counts.skeletons = world->getNumSkeletons();
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto& skeleton = world->getSkeleton(i);
    counts.bodyNodes += skeleton->getNumBodyNodes();
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      const auto* softBodyNode = skeleton->getBodyNode(j)->asSoftBodyNode();
      if (!softBodyNode)
        continue;

      ++counts.softBodies;
      counts.pointMasses += softBodyNode->getNumPointMasses();
    }
  }

  return counts;
}

//==============================================================================
/// Forwards ImGui rendering to the host; kept alive by ImGuiHandler's own
/// widget list once registered.
class HostPanelWidget : public dart::gui::osg::ImGuiWidget
{
public:
  explicit HostPanelWidget(DemoHost* host) : mHost(host) {}

  void render() override
  {
    mHost->renderPanels();
  }

private:
  DemoHost* mHost;
};

//==============================================================================
/// Persistent (never torn down) key handler that dispatches to the active
/// scene's key actions. Installed once at startup.
class DemoKeyHandler : public ::osgGA::GUIEventHandler
{
public:
  explicit DemoKeyHandler(DemoHost* host) : mHost(host) {}

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getHandled())
      return false;

    if (ea.getEventType() != ::osgGA::GUIEventAdapter::KEYDOWN)
      return false;

    // Never let typing in an ImGui field (e.g. the always-visible "Search
    // demos..." box) trigger a scene key action -- osgGA delivers key events
    // to every handler regardless of ImGui focus.
    if (ImGui::GetIO().WantCaptureKeyboard)
      return false;

    return mHost->handleKey(ea.getKey());
  }

private:
  DemoHost* mHost;
};

} // namespace

//==============================================================================
DemoHostContext::DemoHostContext(DemoHost& host) : mHost(host) {}

//==============================================================================
dart::gui::osg::ImGuiViewer* DemoHostContext::viewer() const
{
  return mHost.getViewer();
}

//==============================================================================
dart::gui::osg::WorldNode* DemoHostContext::worldNode() const
{
  return mHost.getWorldNode();
}

//==============================================================================
void DemoHostContext::addTeardown(std::function<void()> teardown)
{
  mHost.addTeardown(std::move(teardown));
}

//==============================================================================
void DemoHostContext::addAttachment(
    dart::gui::osg::ViewerAttachment* attachment)
{
  auto* viewer = mHost.getViewer();
  viewer->addAttachment(attachment);
  addTeardown([viewer, attachment] { viewer->removeAttachment(attachment); });
}

//==============================================================================
void DemoHostContext::addEventHandler(::osgGA::GUIEventHandler* handler)
{
  auto* viewer = mHost.getViewer();
  viewer->addEventHandler(handler);
  addTeardown([viewer, handler] { viewer->removeEventHandler(handler); });
}

//==============================================================================
void DemoHostContext::log(const std::string& message) const
{
  mHost.log(LogEntry::Level::Info, message);
}

//==============================================================================
DemoWorldNode::DemoWorldNode(const dart::simulation::WorldPtr& world)
  : dart::gui::osg::RealTimeWorldNode(world), mStepCount(0)
{
}

//==============================================================================
void DemoWorldNode::setHooks(
    std::function<void()> preStep,
    std::function<void()> postStep,
    std::function<void()> preRefresh)
{
  mPreStep = std::move(preStep);
  mPostStep = std::move(postStep);
  mPreRefresh = std::move(preRefresh);
}

//==============================================================================
void DemoWorldNode::setHookErrorSink(
    std::function<void(const std::string&, const std::string&)> sink)
{
  mHookErrorSink = std::move(sink);
}

//==============================================================================
void DemoWorldNode::stepOnce()
{
  customPreStep();
  if (getWorld())
    getWorld()->step();
  customPostStep();
}

//==============================================================================
std::size_t DemoWorldNode::getStepCount() const
{
  return mStepCount;
}

//==============================================================================
void DemoWorldNode::customPreStep()
{
  invokeHook(mPreStep, "preStep");
  beginStepTiming();
}

//==============================================================================
void DemoWorldNode::customPostStep()
{
  endStepTiming();
  invokeHook(mPostStep, "postStep");
  ++mStepCount;
}

//==============================================================================
void DemoWorldNode::customPreRefresh()
{
  mStepCountAtRefreshStart = mStepCount;
  invokeHook(mPreRefresh, "preRefresh");
}

//==============================================================================
void DemoWorldNode::customPostRefresh()
{
  mLastRefreshStepCount = mStepCount - mStepCountAtRefreshStart;
}

//==============================================================================
std::size_t DemoWorldNode::getLastRefreshStepCount() const
{
  return mLastRefreshStepCount;
}

//==============================================================================
double DemoWorldNode::getLastStepMs() const
{
  return mLastStepMs;
}

//==============================================================================
double DemoWorldNode::getMovingAverageStepMs() const
{
  return mMovingAverageStepMs;
}

//==============================================================================
double DemoWorldNode::getMinStepMs() const
{
  return mMinStepMs;
}

//==============================================================================
double DemoWorldNode::getMaxStepMs() const
{
  return mMaxStepMs;
}

//==============================================================================
std::size_t DemoWorldNode::getStepTimingSamples() const
{
  return mStepTimingSamples;
}

//==============================================================================
void DemoWorldNode::invokeHook(std::function<void()>& hook, const char* name)
{
  if (!hook)
    return;

  try {
    hook();
  } catch (const std::exception& e) {
    if (mHookErrorSink)
      mHookErrorSink(name, e.what());
    hook = nullptr;
  } catch (...) {
    if (mHookErrorSink)
      mHookErrorSink(name, "unknown error");
    hook = nullptr;
  }
}

//==============================================================================
void DemoWorldNode::beginStepTiming()
{
  mStepStart = std::chrono::steady_clock::now();
  mStepTimingActive = true;
}

//==============================================================================
void DemoWorldNode::endStepTiming()
{
  if (!mStepTimingActive)
    return;

  mStepTimingActive = false;
  mLastStepMs = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - mStepStart)
                    .count();

  if (mStepTimingSamples == 0u) {
    mMovingAverageStepMs = mLastStepMs;
    mMinStepMs = mLastStepMs;
    mMaxStepMs = mLastStepMs;
  } else {
    constexpr double kAlpha = 0.08;
    mMovingAverageStepMs += kAlpha * (mLastStepMs - mMovingAverageStepMs);
    mMinStepMs = std::min(mMinStepMs, mLastStepMs);
    mMaxStepMs = std::max(mMaxStepMs, mLastStepMs);
  }

  ++mStepTimingSamples;
}

//==============================================================================
DemoHost::DemoHost(
    std::vector<DemoScene> scenes,
    double guiScale,
    std::string collisionDetectorName,
    std::size_t simulationThreads)
  : mScenes(std::move(scenes)),
    mPerformanceStatsPanel(240u),
    mRequestedCollisionDetectorName(toLower(collisionDetectorName)),
    mSimulationThreads(static_cast<int>(std::min<std::size_t>(
        simulationThreads,
        static_cast<std::size_t>(std::numeric_limits<int>::max())))),
    mGuiScale(dart::gui::osg::sanitizeGuiScale(guiScale))
{
  buildCategories();
  mAvailableCollisionDetectors = getAvailableCollisionDetectorNames();
  mViewer = new dart::gui::osg::ImGuiViewer();

  // Captures dart::common's dtmsg/dtwarn/dterr output (and any other
  // std::cout/std::cerr writes) into the same log the host already uses for
  // its own app-event messages, so the Diagnostics log console shows both
  // through one severity-filtered, autoscrolling view. Restored (stream
  // buffers un-redirected) when mLogCapture is destroyed alongside this host.
  mLogCapture = std::make_unique<LogCapture>(
      [this](CapturedLogLevel level, const std::string& message) {
        LogEntry::Level mapped = LogEntry::Level::Info;
        if (level == CapturedLogLevel::Warning)
          mapped = LogEntry::Level::Warning;
        else if (level == CapturedLogLevel::Error)
          mapped = LogEntry::Level::Error;
        log(mapped, message);
      });
}

//==============================================================================
DemoHost::~DemoHost()
{
  // See the header comment: without this, whichever scene is still active at
  // process exit leaks its DnD/attachment/event-handler teardowns into the
  // viewer's own destruction instead of running them first.
  teardownCurrentScene();
}

//==============================================================================
void DemoHost::buildCategories()
{
  mCategories.clear();
  mCategoryIndexByName.clear();

  for (std::size_t i = 0; i < mScenes.size(); ++i) {
    const std::string& category = mScenes[i].category;
    const auto found = mCategoryIndexByName.find(category);
    std::size_t categoryIndex;
    if (found == mCategoryIndexByName.end()) {
      categoryIndex = mCategories.size();
      mCategoryIndexByName.emplace(category, categoryIndex);
      mCategories.push_back(CategoryGroup{category, {}});
    } else {
      categoryIndex = found->second;
    }
    mCategories[categoryIndex].sceneIndices.push_back(i);
  }
}

//==============================================================================
const DemoScene* DemoHost::findScene(const std::string& id) const
{
  for (const auto& scene : mScenes) {
    if (scene.id == id)
      return &scene;
  }
  return nullptr;
}

//==============================================================================
void DemoHost::setInitialScene(const std::string& id)
{
  mInitialSceneId = id;
}

//==============================================================================
void DemoHost::setDebugSelectBodyName(const std::string& name)
{
  mDebugSelectBodyName = name;
}

//==============================================================================
void DemoHost::setDebugRecordProfile(bool on)
{
  mDebugRecordProfile = on;
}

//==============================================================================
void DemoHost::addHeadlessActionKey(int key)
{
  mHeadlessActionKeys.push_back(key);
}

//==============================================================================
void DemoHost::requestScenePanelTab(ScenePanelTab tab)
{
  mRequestedScenePanelTab = tab;
  mHasRequestedScenePanelTab = true;
}

//==============================================================================
std::size_t DemoHost::getActiveWorldNodeCount() const
{
  return mActiveWorldNodeCount;
}

//==============================================================================
dart::gui::osg::ImGuiViewer* DemoHost::getViewer() const
{
  return mViewer.get();
}

//==============================================================================
DemoWorldNode* DemoHost::getWorldNode() const
{
  return mWorldNode.get();
}

//==============================================================================
void DemoHost::addTeardown(std::function<void()> teardown)
{
  mExtraTeardowns.push_back(std::move(teardown));
}

//==============================================================================
void DemoHost::log(LogEntry::Level level, const std::string& message)
{
  mLog.push_back(LogEntry{level, message});

  // A ring buffer of ~2000 entries: generous enough to survive a captured
  // dtmsg/dtwarn/dterr burst (e.g. a noisy solver) without losing the app's
  // own recent status messages, while staying bounded.
  constexpr std::size_t kMaxLogEntries = 2000;
  while (mLog.size() > kMaxLogEntries)
    mLog.pop_front();
}

//==============================================================================
void DemoHost::syncCollisionDetectorSelectionFromWorld()
{
  mCurrentCollisionDetectorName = getWorldCollisionDetectorName(mCurrentWorld);
  mCollisionDetectorIndex = -1;
  for (std::size_t i = 0; i < mAvailableCollisionDetectors.size(); ++i) {
    if (mAvailableCollisionDetectors[i] == mCurrentCollisionDetectorName) {
      mCollisionDetectorIndex = static_cast<int>(i);
      break;
    }
  }
}

//==============================================================================
bool DemoHost::setCollisionDetectorByName(const std::string& name)
{
  if (!mCurrentWorld || name.empty())
    return true;

  auto* factory = dart::collision::CollisionDetector::getFactory();
  if (!factory || !factory->canCreate(name)) {
    log(LogEntry::Level::Warning,
        "Collision detector '" + name + "' is not available; keeping '"
            + getWorldCollisionDetectorName(mCurrentWorld) + "'.");
    syncCollisionDetectorSelectionFromWorld();
    return false;
  }

  auto detector = factory->create(name);
  if (!detector) {
    log(LogEntry::Level::Warning,
        "Collision detector '" + name + "' failed to create; keeping '"
            + getWorldCollisionDetectorName(mCurrentWorld) + "'.");
    syncCollisionDetectorSelectionFromWorld();
    return false;
  }

  mCurrentWorld->setCollisionDetector(detector);
  mRequestedCollisionDetectorName = detector->getType();
  syncCollisionDetectorSelectionFromWorld();
  mPerformanceStatsPanel.reset();
  log(LogEntry::Level::Info,
      "Collision detector set to '" + mCurrentCollisionDetectorName + "'.");
  return true;
}

//==============================================================================
void DemoHost::applyRuntimeOptionsToWorld()
{
  if (!mCurrentWorld)
    return;

  if (mSimulationThreads < 0)
    mSimulationThreads = 1;
  mCurrentWorld->setNumSimulationThreads(
      static_cast<std::size_t>(mSimulationThreads));
  mSimulationThreads
      = static_cast<int>(mCurrentWorld->getNumSimulationThreads());

  if (!mRequestedCollisionDetectorName.empty())
    setCollisionDetectorByName(mRequestedCollisionDetectorName);
  else
    syncCollisionDetectorSelectionFromWorld();
}

//==============================================================================
void DemoHost::ensureViewerConfigured()
{
  if (mViewerConfigured)
    return;
  mViewerConfigured = true;

  mViewer->setThreadingModel(::osgViewer::ViewerBase::SingleThreaded);

  applyModernDarkColors();
  applyModernDarkMetrics();
  mViewer->getImGuiHandler()->setGuiScale(mGuiScale);

  mViewer->getCamera()->setClearColor(::osg::Vec4(0.58f, 0.62f, 0.65f, 1.0f));

  mViewer->getImGuiHandler()->addWidget(
      std::make_shared<HostPanelWidget>(this));
  mViewer->addEventHandler(new DemoKeyHandler(this));

  // Host-wide picking: plain left-click selects a body for the Inspector;
  // Ctrl+left-drag starts a spring-force drag (DragForce owns both, since it
  // needs the same persistent mouse handler -- see DragForce.hpp).
  mDragForce.installMouseHandler(
      mViewer.get(), [this](dart::dynamics::BodyNode* body) {
        mInspector.setSelection(body);
      });

  // View-utilities grid attachment: host chrome, persists across scene
  // switches (unlike a scene's own attachments, which are torn down on
  // switch via ctx.addTeardown). Off by default -- most scenes already
  // render their own ground plane/floor.
  mGridVisual = new dart::gui::osg::GridVisual();
  mGridVisual->display(false);
  mViewer->addAttachment(mGridVisual.get());
}

//==============================================================================
void DemoHost::applyShadowState()
{
  if (!mWorldNode)
    return;

  if (mShadowsEnabled && mCurrentSceneWantsShadows) {
    mWorldNode->setShadowTechnique(
        dart::gui::osg::WorldNode::createDefaultShadowTechnique(mViewer.get()));
  } else {
    mWorldNode->setShadowTechnique(nullptr);
  }
}

//==============================================================================
void DemoHost::requestSceneSwitch(const std::string& id)
{
  mPendingSceneId = id;
}

//==============================================================================
void DemoHost::processPendingSwitch()
{
  if (!mPendingSceneId.has_value())
    return;

  const std::string requestedId = *mPendingSceneId;
  mPendingSceneId.reset();

  const DemoScene* scene = findScene(requestedId);
  if (!scene) {
    mLastSwitchFailed = true;
    log(LogEntry::Level::Error, "Unknown demo id '" + requestedId + "'");
    return;
  }

  mStatusLine = "Starting demo '" + scene->title + "'...";
  log(LogEntry::Level::Info, mStatusLine);

  DemoSceneSetup setup;
  bool ok = true;
  std::string failureReason;
  try {
    setup = scene->factory();
    if (!setup.world)
      throw std::runtime_error("factory returned a null world");
  } catch (const std::exception& e) {
    ok = false;
    failureReason = e.what();
  } catch (...) {
    ok = false;
    failureReason = "unknown error";
  }

  if (!ok) {
    mLastSwitchFailed = true;
    if (!mCurrentSceneId.empty()) {
      mStatusLine = "Restored previous demo '" + mCurrentSceneTitle
                    + "' after '" + scene->title + "' failed: " + failureReason;
    } else {
      mStatusLine
          = "Failed to start demo '" + scene->title + "': " + failureReason;
      teardownCurrentScene();
      installEmptyFallback();
    }
    log(LogEntry::Level::Error, mStatusLine);
    return;
  }

  mLastSwitchFailed = false;
  teardownCurrentScene();
  installScene(*scene, std::move(setup));
  mCurrentSceneId = scene->id;
  mCurrentSceneTitle = scene->title;
  mStatusLine = "Running demo '" + scene->title + "'";
  log(LogEntry::Level::Info, mStatusLine);
}

//==============================================================================
void DemoHost::teardownCurrentScene()
{
  // Host-level facilities first: they hold raw BodyNode/DegreeOfFreedom
  // pointers and SimpleFrame/InteractiveFrame objects that belong to the
  // world about to be destroyed below.
  mInspector.reset();
  mDragForce.reset(mViewer.get());
  mContactVisualizer.reset();

  // Tear down drag-and-drop / attachments / event handlers before the world
  // node and world itself, since these extras may hold raw pointers into
  // objects the world owns (e.g. a SimpleFrame's DnD registration).
  for (auto it = mExtraTeardowns.rbegin(); it != mExtraTeardowns.rend(); ++it) {
    try {
      (*it)();
    } catch (...) {
      // Teardown must never crash the host.
    }
  }
  mExtraTeardowns.clear();

  if (mWorldNode) {
    mViewer->removeWorldNode(mWorldNode.get());
    mWorldNode = nullptr;
    if (mActiveWorldNodeCount > 0)
      --mActiveWorldNodeCount;
  }

  mCurrentWorld.reset();
  mCurrentRenderPanel = nullptr;
  mCurrentKeyActions.clear();
  mCurrentCameraHome.reset();
  mCurrentSceneDocumentation = ScenePanelDocumentation{};
  mCurrentPhysicsContractProvider = nullptr;
  mCurrentSceneStateProvider = nullptr;
  mCurrentSceneId.clear();
  mCurrentSceneTitle.clear();
}

//==============================================================================
void DemoHost::installScene(const DemoScene& scene, DemoSceneSetup setup)
{
  mCurrentWorld = setup.world;
  applyRuntimeOptionsToWorld();
  mPerformanceStatsPanel.reset();
  mDragForce.onSceneInstalled(mCurrentWorld);
  mContactVisualizer.onSceneInstalled(mCurrentWorld);
  mCurrentSceneWantsShadows = setup.enableShadows;

  mWorldNode = new DemoWorldNode(mCurrentWorld);
  applyShadowState();

  // Compose the host-level facilities (joint-slider edits, drag-force/
  // gizmo spring, contact-visualizer refresh) around the scene's own hooks.
  // Each facility call is internally exception-safe (never throws), so a
  // fault in one never disables the scene's own preStep/postStep/preRefresh
  // the way DemoWorldNode::invokeHook's generic guard would if it were
  // tripped by host code.
  mWorldNode->setHooks(
      [this, scenePreStep = std::move(setup.preStep)] {
        mInspector.applyQueuedEdits();
        mDragForce.applyPreStep();
        if (scenePreStep)
          scenePreStep();
      },
      [this, scenePostStep = std::move(setup.postStep)] {
        if (scenePostStep)
          scenePostStep();
        mContactVisualizer.applyPostStep();
      },
      [this, scenePreRefresh = std::move(setup.preRefresh)] {
        mInspector.applyQueuedEdits();
        mInspector.updateHighlight(ImGui::GetTime());
        mDragForce.applyPreRefresh(!mViewer->isSimulating());
        if (scenePreRefresh)
          scenePreRefresh();
      });
  mWorldNode->setHookErrorSink(
      [this](const std::string& hookName, const std::string& reason) {
        log(LogEntry::Level::Error,
            "Demo '" + mCurrentSceneTitle + "' " + hookName
                + " failed: " + reason + " (disabled)");
      });

  mTargetRtf = 1.0f;
  mWorldNode->setTargetRealTimeFactor(mTargetRtf);

  // Re-read the new world's timestep so the toolbar's Timestep control reflects
  // the active scene (each scene factory sets its own; e.g. the chain scenes
  // use 1/2000 s while most others use 1/1000 s).
  mTimeStep = static_cast<float>(mCurrentWorld->getTimeStep());

  mGravityEnabled = mCurrentWorld->getGravity().norm() > 1e-8;
  mSavedGravity = mGravityEnabled ? mCurrentWorld->getGravity()
                                  : Eigen::Vector3d(0.0, 0.0, -9.81);

  mViewer->addWorldNode(mWorldNode.get());
  ++mActiveWorldNodeCount;

  mCurrentRenderPanel = setup.renderPanel;
  mCurrentKeyActions = std::move(setup.keyActions);
  mCurrentCameraHome = setup.cameraHome;
  mCurrentSceneDocumentation = scene.scenePanelDocumentation;
  mCurrentPhysicsContractProvider = std::move(setup.physicsContractProvider);
  mCurrentSceneStateProvider = std::move(setup.sceneStateProvider);

  for (const auto& frame : setup.dragFrames) {
    if (!frame)
      continue;
    auto* dnd = mViewer->enableDragAndDrop(frame.get());
    if (dnd) {
      auto* viewer = mViewer.get();
      mExtraTeardowns.push_back(
          [viewer, dnd] { viewer->disableDragAndDrop(dnd); });
    }
  }

  if (setup.onActivate) {
    DemoHostContext ctx(*this);
    try {
      setup.onActivate(ctx);
    } catch (const std::exception& e) {
      log(LogEntry::Level::Error,
          "Demo '" + scene.title + "' onActivate failed: " + e.what());
    } catch (...) {
      log(LogEntry::Level::Error,
          "Demo '" + scene.title + "' onActivate failed: unknown error");
    }
  }

  const CameraHome defaultHome{
      ::osg::Vec3d(6.0, 8.0, 4.0),
      ::osg::Vec3d(0.0, 0.0, 1.0),
      ::osg::Vec3d(0.0, 0.0, 1.0)};
  applyCameraHome(
      mCurrentCameraHome.value_or(defaultHome),
      mViewer->getCameraManipulator() != nullptr);
}

//==============================================================================
void DemoHost::installEmptyFallback()
{
  DemoScene fallback;
  fallback.id.clear();
  fallback.title = "(no demo loaded)";

  DemoSceneSetup setup;
  setup.world = dart::simulation::World::create();
  setup.enableShadows = false;

  installScene(fallback, std::move(setup));
  mCurrentSceneId.clear();
  mCurrentSceneTitle = fallback.title;
}

//==============================================================================
void DemoHost::applyCameraHome(const CameraHome& home, bool viaManipulator)
{
  if (viaManipulator && mViewer->getCameraManipulator()) {
    mViewer->getCameraManipulator()->setHomePosition(
        home.eye, home.center, home.up);
    mViewer->setCameraManipulator(mViewer->getCameraManipulator());
  } else {
    mViewer->getCamera()->setViewMatrixAsLookAt(home.eye, home.center, home.up);
  }
}

//==============================================================================
void DemoHost::fitCameraToWorld()
{
  if (!mCurrentWorld)
    return;
  dart::gui::osg::applyDefaultCameraPose(*mViewer, mCurrentWorld);
}

//==============================================================================
int DemoHost::listScenes() const
{
  for (const auto& category : mCategories) {
    std::cout << category.name << "\n";
    for (const auto index : category.sceneIndices) {
      const DemoScene& scene = mScenes[index];
      std::cout << "  " << scene.id << "  -  " << scene.title;
      if (!scene.summary.empty())
        std::cout << ": " << scene.summary;
      std::cout << "\n";
    }
  }
  return 0;
}

//==============================================================================
int DemoHost::verifyFbfSceneDocs() const
{
  bool ok = true;
  std::size_t checked = 0u;
  for (const auto& scene : mScenes) {
    if (!startsWith(scene.id, "fbf_"))
      continue;

    ++checked;
    if (scene.category != "Research") {
      std::cerr << "[verify-fbf-scene-docs] " << scene.id
                << " is not in the Research category.\n";
      ok = false;
    }
    if (scene.summary.empty()) {
      std::cerr << "[verify-fbf-scene-docs] " << scene.id
                << " has an empty catalog summary.\n";
      ok = false;
    }
    if (!scene.scenePanelDocumentation.isComplete()) {
      std::cerr << "[verify-fbf-scene-docs] " << scene.id
                << " is missing Scene-tab overview, expected result, or "
                   "coverage text.\n";
      ok = false;
    }
    if (!scene.factory) {
      std::cerr << "[verify-fbf-scene-docs] " << scene.id
                << " has no scene factory.\n";
      ok = false;
    }
  }

  if (checked == 0u) {
    std::cerr << "[verify-fbf-scene-docs] no fbf_* scenes found.\n";
    return 1;
  }

  std::cout << "[verify-fbf-scene-docs] checked " << checked
            << " FBF research scene(s): " << (ok ? "OK" : "FAILED") << "\n";
  return ok ? 0 : 1;
}

//==============================================================================
int DemoHost::printScenePhysicsContract(const std::string& sceneId)
{
  if (sceneId.empty()) {
    std::cerr << "[scene-physics-contract] scene id must not be empty.\n";
    return 1;
  }

  if (!findScene(sceneId)) {
    std::cerr << "[scene-physics-contract] unknown demo id '" << sceneId
              << "'.\n";
    return 1;
  }

  // This follows the normal scene-installation lifecycle so runtime options
  // and activation-scoped physics settings are reflected in the contract. It
  // deliberately does not call ensureViewerConfigured(), realize(), frame(),
  // or any capture path.
  requestSceneSwitch(sceneId);
  processPendingSwitch();
  if (mLastSwitchFailed || mCurrentSceneId != sceneId) {
    std::cerr << "[scene-physics-contract] demo '" << sceneId
              << "' failed to start.\n";
    return 1;
  }

  for (const int key : mHeadlessActionKeys) {
    if (!invokeHeadlessActionKey(key)) {
      std::cerr << "[scene-physics-contract] action key "
                << formatKeyForMessage(key) << " failed for demo '" << sceneId
                << "'.\n";
      return 1;
    }
  }

  if (!mCurrentPhysicsContractProvider) {
    std::cerr << "[scene-physics-contract] demo '" << sceneId
              << "' has no physics contract provider.\n";
    return 1;
  }

  try {
    const std::string contract = mCurrentPhysicsContractProvider();
    if (contract.empty()) {
      std::cerr
          << "[scene-physics-contract] physics contract provider for demo '"
          << sceneId << "' returned empty output.\n";
      return 1;
    }
    std::cout << contract << '\n';
  } catch (const std::exception& error) {
    std::cerr << "[scene-physics-contract] physics contract failed for demo '"
              << sceneId << "': " << error.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "[scene-physics-contract] physics contract failed for demo '"
              << sceneId << "': unknown error\n";
    return 1;
  }

  return 0;
}

//==============================================================================
int DemoHost::cycleScenes(int framesPerScene)
{
  bool anyFailure = false;

  // Loop the whole catalog twice in-process: this rapid re-switch robustness
  // gate must not leak world nodes or corrupt state across repeated switches.
  for (int repeat = 0; repeat < 2; ++repeat) {
    for (const auto& scene : mScenes) {
      requestSceneSwitch(scene.id);
      processPendingSwitch();

      if (mLastSwitchFailed) {
        std::cerr << "[cycle-scenes] demo '" << scene.id
                  << "' failed to start.\n";
        anyFailure = true;
      }

      for (int frame = 0; frame < framesPerScene && mWorldNode; ++frame)
        mWorldNode->stepOnce();

      if (mActiveWorldNodeCount != 1) {
        std::cerr << "[cycle-scenes] world-node leak detected after '"
                  << scene.id << "' (active=" << mActiveWorldNodeCount << ")\n";
        anyFailure = true;
      }
    }
  }

  std::cout << "[cycle-scenes] cycled " << mScenes.size() << " demo(s) x2, "
            << framesPerScene << " frame(s) each. "
            << (anyFailure ? "FAILED" : "OK") << "\n";
  return anyFailure ? 1 : 0;
}

//==============================================================================
bool DemoHost::prepareOffscreenContext(
    int width, int height, const CameraHome& home)
{
  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->readDISPLAY();
  traits->setUndefinedScreenDetailsToDefaultScreen();
  traits->x = 0;
  traits->y = 0;
  traits->width = width;
  traits->height = height;
  traits->red = traits->green = traits->blue = 8;
  traits->alpha = 8;
  traits->depth = 24;
  traits->windowDecoration = false;
  traits->pbuffer = true;
  traits->doubleBuffer = true;

  ::osg::ref_ptr<::osg::GraphicsContext> gc
      = ::osg::GraphicsContext::createGraphicsContext(traits.get());
  if (!gc) {
    std::cerr << "[headless] Failed to create an off-screen GL context (no "
                 "usable DISPLAY?).\n";
    return false;
  }

  auto* camera = mViewer->getCamera();
  camera->setGraphicsContext(gc.get());
  camera->setViewport(new ::osg::Viewport(0, 0, width, height));
  camera->setProjectionMatrixAsPerspective(
      30.0, static_cast<double>(width) / height, 0.1, 1000.0);
  const GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);

  // SingleThreaded makes Viewer::captureScreen's final-draw callback complete
  // inside frame(). Other OSG threading models may run draw callbacks on a
  // separate thread, which would make a single extra frame an unreliable
  // screenshot completion point for the headless CLI path.
  mViewer->setThreadingModel(::osgViewer::ViewerBase::SingleThreaded);
  mViewer->setCameraManipulator(nullptr);
  mViewer->simulate(false);
  camera->setViewMatrixAsLookAt(home.eye, home.center, home.up);

  mViewer->realize();
  if (!mViewer->isRealized()) {
    std::cerr << "[headless] Viewer failed to realize off-screen.\n";
    return false;
  }

  if (auto* queue = mViewer->getEventQueue()) {
    queue->windowResize(0, 0, width, height);
    queue->setMouseInputRange(0.0f, 0.0f, width, height);
  }

  return true;
}

//==============================================================================
bool DemoHost::prepareHeadlessRun(
    const std::string& sceneId,
    int width,
    int height,
    CameraHome& home,
    bool& sceneFailed)
{
  ensureViewerConfigured();

  const std::string initial = !sceneId.empty()           ? sceneId
                              : !mInitialSceneId.empty() ? mInitialSceneId
                              : !mScenes.empty()         ? mScenes.front().id
                                                         : std::string();
  if (initial.empty()) {
    std::cerr << "[headless] No demo scenes are registered.\n";
    return false;
  }

  requestSceneSwitch(initial);
  processPendingSwitch();
  sceneFailed = mLastSwitchFailed;
  if (sceneFailed) {
    std::cerr << "[headless] demo '" << initial
              << "' failed to start; capturing the fallback state.\n";
  }

  // Headless output is evidence/presentation media, so omit the interactive
  // contact-force debug overlay. Interactive runs retain the default-on
  // visualizer and its Diagnostics toggle.
  mContactVisualizer.setEnabled(false);

  // Hidden debug/test hooks (main.cpp's --debug-select-body/
  // --debug-record-profile): let a headless capture exercise UI state that
  // normally requires interactive input: scene tree + inspector with a
  // selection, and profiler recording in the Tools tab.
  if (!mDebugSelectBodyName.empty() && mCurrentWorld) {
    dart::dynamics::BodyNode* found = nullptr;
    for (std::size_t s = 0; s < mCurrentWorld->getNumSkeletons() && !found;
         ++s) {
      found = mCurrentWorld->getSkeleton(s)->getBodyNode(mDebugSelectBodyName);
    }
    if (found) {
      mInspector.setSelection(found);
      requestScenePanelTab(ScenePanelTab::Inspector);
    } else {
      std::cerr << "[headless] --debug-select-body '" << mDebugSelectBodyName
                << "' not found in demo '" << initial << "'.\n";
    }
  }
  if (mDebugRecordProfile) {
    mProfiler.setRecordingForTest(true);
    requestScenePanelTab(ScenePanelTab::Tools);
  }

  for (const int key : mHeadlessActionKeys) {
    if (!handleKey(key)) {
      std::cerr << "[headless] action key " << formatKeyForMessage(key)
                << " is not registered for demo '" << initial << "'.\n";
      return false;
    }
  }

  const CameraHome defaultHome{
      ::osg::Vec3d(6.0, 8.0, 4.0),
      ::osg::Vec3d(0.0, 0.0, 1.0),
      ::osg::Vec3d(0.0, 0.0, 1.0)};
  home = mCurrentCameraHome.value_or(defaultHome);

  return prepareOffscreenContext(width, height, home);
}

//==============================================================================
bool DemoHost::captureHeadlessShot(
    const std::string& shotPath, const CameraHome& home)
{
  errno = 0;
  if (std::remove(shotPath.c_str()) != 0 && errno != ENOENT) {
    std::cerr << "[headless] failed to remove stale screenshot '" << shotPath
              << "': " << std::strerror(errno) << "\n";
    return false;
  }

  // Re-pin the view (realize() may have reset it) and draw, then capture.
  mViewer->getCamera()->setViewMatrixAsLookAt(home.eye, home.center, home.up);
  mViewer->frame();
  mViewer->frame();
  mViewer->captureScreen(shotPath);
  mViewer->frame(); // SaveScreen writes the PNG during this frame.

  std::ifstream captured(shotPath, std::ios::binary);
  if (!captured.good()) {
    std::cerr << "[headless] failed to write screenshot '" << shotPath
              << "'.\n";
    return false;
  }
  return true;
}

//==============================================================================
int DemoHost::runHeadlessShot(
    const std::string& shotPath,
    int steps,
    const std::string& sceneId,
    int width,
    int height)
{
  CameraHome home;
  bool sceneFailed = false;
  if (!prepareHeadlessRun(sceneId, width, height, home, sceneFailed))
    return 1;

  for (int i = 0; i < steps && mWorldNode; ++i)
    mWorldNode->stepOnce();

  if (!captureHeadlessShot(shotPath, home))
    return 1;

  std::cout << "[headless] sim time "
            << (mCurrentWorld ? mCurrentWorld->getTime() : 0.0) << " s; wrote "
            << shotPath << "\n";
  return sceneFailed ? 1 : 0;
}

//==============================================================================
int DemoHost::runHeadlessTimeline(
    const HeadlessTimeline& timeline,
    int totalSteps,
    const std::string& sceneId,
    int width,
    int height)
{
  if (totalSteps < 0) {
    std::cerr << "[headless] timeline total steps must be nonnegative.\n";
    return 1;
  }

  const std::size_t requestedSteps = static_cast<std::size_t>(totalSteps);
  for (const auto& shot : timeline.shots) {
    if (shot.path.empty() || shot.step > requestedSteps) {
      std::cerr << "[headless] invalid shot scheduled for step " << shot.step
                << ".\n";
      return 1;
    }
  }
  for (const auto& action : timeline.actions) {
    if (action.step > requestedSteps) {
      std::cerr << "[headless] invalid action scheduled for step "
                << action.step << ".\n";
      return 1;
    }
  }

  const std::string requestedScene = !sceneId.empty() ? sceneId
                                     : !mInitialSceneId.empty()
                                         ? mInitialSceneId
                                     : !mScenes.empty() ? mScenes.front().id
                                                        : std::string();

  CameraHome home;
  bool sceneFailed = false;
  if (!prepareHeadlessRun(sceneId, width, height, home, sceneFailed))
    return 1;

  std::string physicsContractJson;
  if (mCurrentPhysicsContractProvider) {
    try {
      physicsContractJson = mCurrentPhysicsContractProvider();
    } catch (const std::exception& e) {
      std::cerr << "[headless] physics contract failed for demo '"
                << requestedScene << "': " << e.what() << "\n";
      return 1;
    } catch (...) {
      std::cerr << "[headless] physics contract failed for demo '"
                << requestedScene << "': unknown error\n";
      return 1;
    }
  }

  bool eventFailed = false;
  std::size_t completedSteps = 0u;
  std::size_t sequence = 0u;
  std::vector<HeadlessShotResult> shotResults;
  std::vector<HeadlessStepResult> stepResults;
  std::vector<HeadlessActionResult> actionResults;
  SolverDiagnosticsCursor diagnosticsCursor;
  HeadlessExactFbfFailFastState failFast;
  HeadlessExactFbfFailFastState sourceContinuation;
  shotResults.reserve(timeline.shots.size());
  stepResults.reserve(requestedSteps + 1u);
  actionResults.reserve(timeline.actions.size());

  for (std::size_t step = 0u; step <= requestedSteps; ++step) {
    completedSteps = step;

    HeadlessStepResult stepResult;
    stepResult.step = step;
    stepResult.simTime = mCurrentWorld ? mCurrentWorld->getTime() : 0.0;
    stepResult.diagnostics
        = captureSolverDiagnostics(mCurrentWorld, &diagnosticsCursor);
    if (mCurrentSceneStateProvider) {
      try {
        stepResult.sceneState = mCurrentSceneStateProvider();
      } catch (const std::exception& e) {
        std::cerr << "[headless] scene-state provider failed for demo '"
                  << requestedScene << "' at completed step " << step << ": "
                  << e.what() << "\n";
        return 1;
      } catch (...) {
        std::cerr << "[headless] scene-state provider failed for demo '"
                  << requestedScene << "' at completed step " << step
                  << ": unknown error\n";
        return 1;
      }
      if (stepResult.sceneState->empty()) {
        std::cerr << "[headless] scene-state provider for demo '"
                  << requestedScene << "' returned no fields at completed step "
                  << step << ".\n";
        return 1;
      }
      std::unordered_set<std::string> keys;
      for (const auto& field : *stepResult.sceneState) {
        if (field.first.empty() || !keys.insert(field.first).second
            || !std::isfinite(field.second)) {
          std::cerr << "[headless] scene-state provider for demo '"
                    << requestedScene
                    << "' returned an empty/duplicate key or non-finite value "
                    << "at completed step " << step << ".\n";
          return 1;
        }
      }
    }
    stepResults.push_back(stepResult);

    if (timeline.exactFbfFailFast) {
      detail::HeadlessExactFbfFailFastDiagnostics diagnostics;
      diagnostics.exactAttempts = stepResult.diagnostics.exactAttempts;
      diagnostics.acceptedAtCap = stepResult.diagnostics.maxIterationsAccepted;
      diagnostics.exactFailures = stepResult.diagnostics.exactFailures;
      diagnostics.boxedFallbacks = stepResult.diagnostics.boxedLcpFallbacks;
      diagnostics.residual = stepResult.diagnostics.residual;
      diagnostics.worstResidual = stepResult.diagnostics.worstResidual;

      const auto decision
          = detail::evaluateHeadlessExactFbfFailFast(diagnostics);
      if (decision.triggered) {
        failFast.triggered = true;
        failFast.step = step;
        failFast.reason = decision.reason;
        std::cerr << "[headless] exact-FBF fail-fast triggered at completed "
                     "step "
                  << step << ": " << decision.reason << ".\n";
        break;
      }
    }
    if (timeline.exactFbfSourceContinuation) {
      const auto decision = detail::evaluateHeadlessExactFbfSourceContinuation(
          makeSourceContinuationGateDiagnostics(stepResult.diagnostics));
      if (decision.triggered) {
        sourceContinuation.triggered = true;
        sourceContinuation.step = step;
        sourceContinuation.reason = decision.reason;
        std::cerr
            << "[headless] exact-FBF source-continuation gate triggered at "
               "completed step "
            << step << ": " << decision.reason << ".\n";
        break;
      }
    }

    // A completed-step state belongs to the simulation before any action at
    // that step. Capture every same-step request first, in CLI order.
    for (const auto& shot : timeline.shots) {
      if (shot.step != step)
        continue;

      HeadlessShotResult result;
      result.sequence = sequence++;
      result.step = step;
      result.path = shot.path;
      result.simTime = mCurrentWorld ? mCurrentWorld->getTime() : 0.0;
      result.success = captureHeadlessShot(shot.path, home);
      result.diagnostics = stepResult.diagnostics;
      eventFailed |= !result.success;
      if (result.success) {
        std::cout << "[headless] step " << step << "; sim time "
                  << result.simTime << " s; wrote " << shot.path << "\n";
      }
      shotResults.push_back(std::move(result));
    }

    // Actions run after every same-step capture and before the next physics
    // step, so a step-N action first affects the N+1 completed state.
    for (const auto& action : timeline.actions) {
      if (action.step != step)
        continue;

      HeadlessActionResult result;
      result.sequence = sequence++;
      result.step = step;
      result.key = action.key;
      result.success = invokeHeadlessActionKey(action.key);
      eventFailed |= !result.success;
      if (!result.success) {
        std::cerr << "[headless] scheduled action key "
                  << formatKeyForMessage(action.key) << " failed at step "
                  << step << " for demo '" << requestedScene << "'.\n";
      }
      actionResults.push_back(result);
    }

    if (step == requestedSteps)
      break;
    if (!mWorldNode) {
      std::cerr << "[headless] world node disappeared before timeline step "
                << (step + 1u) << ".\n";
      eventFailed = true;
      break;
    }
    mWorldNode->stepOnce();
  }

  const SolverDiagnosticsSnapshot finalDiagnostics
      = stepResults.empty() ? captureSolverDiagnostics(mCurrentWorld)
                            : stepResults.back().diagnostics;
  if (!timeline.sidecarPath.empty()) {
    const bool sidecarWritten = writeHeadlessSidecar(
        timeline.sidecarPath,
        requestedScene,
        mCurrentSceneId,
        timeline.runtimeCommand,
        requestedSteps,
        completedSteps,
        width,
        height,
        getWorldCollisionDetectorName(mCurrentWorld),
        physicsContractJson,
        stepResults,
        shotResults,
        actionResults,
        finalDiagnostics,
        timeline.exactFbfFailFast ? &failFast : nullptr,
        timeline.exactFbfSourceContinuation ? &sourceContinuation : nullptr);
    eventFailed |= !sidecarWritten;
    if (sidecarWritten)
      std::cout << "[headless] wrote timeline sidecar " << timeline.sidecarPath
                << "\n";
  }

  return sceneFailed || eventFailed || failFast.triggered
                 || sourceContinuation.triggered
             ? 1
             : 0;
}

//==============================================================================
int DemoHost::run()
{
  ensureViewerConfigured();

  mViewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(kDefaultWindowWidth, mGuiScale),
      dart::gui::osg::scaleWindowExtent(kDefaultWindowHeight, mGuiScale));

  const std::string initial = !mInitialSceneId.empty() ? mInitialSceneId
                              : !mScenes.empty()       ? mScenes.front().id
                                                       : std::string();
  if (!initial.empty()) {
    requestSceneSwitch(initial);
    processPendingSwitch();
  }

  if (!mViewer->isRealized())
    mViewer->realize();

  mViewer->simulate(true);

  while (!mViewer->done()) {
    // Scene switches are queued (requestSceneSwitch) and only ever executed
    // here, between frames -- never from inside ImGui rendering.
    processPendingSwitch();
    mViewer->frame();
  }

  return 0;
}

//==============================================================================
KeyAction* DemoHost::findKeyAction(int key)
{
  for (auto& action : mCurrentKeyActions) {
    if (action.key == key)
      return &action;
  }

  // Case-insensitive fallback: try the opposite-case alphabetic key so a scene
  // that binds 'q' still responds to a Shift-held 'Q' (and vice versa). Only
  // ASCII letters are folded; special keys (arrows, Tab, ...) carry large
  // osgGA KeySymbol codes and are left untouched.
  if (key >= 0 && key <= 255 && std::isalpha(static_cast<unsigned char>(key))) {
    const int lower = std::tolower(static_cast<unsigned char>(key));
    const int alt = (key == lower)
                        ? std::toupper(static_cast<unsigned char>(key))
                        : lower;
    for (auto& action : mCurrentKeyActions) {
      if (action.key == alt)
        return &action;
    }
  }
  return nullptr;
}

//==============================================================================
bool DemoHost::invokeKeyAction(KeyAction& action)
{
  if (!action.callback)
    return false;

  try {
    action.callback();
    return true;
  } catch (const std::exception& e) {
    log(LogEntry::Level::Error,
        "Action '" + action.label + "' failed: " + e.what() + " (disabled)");
    action.callback = nullptr;
  } catch (...) {
    log(LogEntry::Level::Error,
        "Action '" + action.label + "' failed: unknown error (disabled)");
    action.callback = nullptr;
  }
  return false;
}

//==============================================================================
bool DemoHost::invokeHeadlessActionKey(int key)
{
  auto* action = findKeyAction(key);
  return action && invokeKeyAction(*action);
}

//==============================================================================
bool DemoHost::handleKey(int key)
{
  auto* action = findKeyAction(key);
  if (!action)
    return false;
  invokeKeyAction(*action);
  return true;
}

//==============================================================================
void DemoHost::renderPanels()
{
  const ImGuiIO& io = ImGui::GetIO();
  const float scale = static_cast<float>(mGuiScale);
  const float screenW = std::max(1.0f, io.DisplaySize.x);
  const float screenH = std::max(1.0f, io.DisplaySize.y);

  const float toolbarH = std::min(58.0f * scale, screenH * 0.18f);
  const float bottomH = std::min(156.0f * scale, screenH * 0.26f);
  const float leftW = std::min(260.0f * scale, screenW * 0.30f);
  const float rightW = std::min(360.0f * scale, screenW * 0.34f);
  const float middleH = std::max(1.0f, screenH - toolbarH - bottomH);

  constexpr ImGuiWindowFlags kChromeFlags
      = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
        | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;

  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(screenW, toolbarH), ImGuiCond_Always);
  if (ImGui::Begin("Simulation", nullptr, kChromeFlags))
    renderToolbar();
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(0.0f, toolbarH), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(leftW, middleH), ImGuiCond_Always);
  if (ImGui::Begin("Demos", nullptr, kChromeFlags))
    renderNavigator();
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(screenW - rightW, toolbarH), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(rightW, middleH), ImGuiCond_Always);
  const std::string panelTitle
      = mCurrentSceneTitle.empty() ? "Scene" : mCurrentSceneTitle;
  if (ImGui::Begin(panelTitle.c_str(), nullptr, kChromeFlags))
    renderScenePanel();
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(0.0f, screenH - bottomH), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(screenW, bottomH), ImGuiCond_Always);
  if (ImGui::Begin("Diagnostics", nullptr, kChromeFlags))
    renderDiagnostics();
  ImGui::End();
}

//==============================================================================
void DemoHost::renderToolbar()
{
  const bool hasScene = mWorldNode != nullptr;
  const bool simulating = mViewer->isSimulating();
  const float scale = static_cast<float>(mGuiScale);

  ImGui::TextColored(ImVec4(0.42f, 0.70f, 1.0f, 1.0f), "DART");
  ImGui::SameLine();
  ImGui::TextDisabled("demos");
  if (!mCurrentSceneTitle.empty()) {
    sameLineIfEnoughRoom(ImGui::CalcTextSize(mCurrentSceneTitle.c_str()).x);
    ImGui::TextUnformatted(mCurrentSceneTitle.c_str());
  }

  sameLineIfEnoughRoom(calcButtonWidth(simulating ? "Pause" : "Play"));
  ImGui::BeginDisabled(!hasScene);
  if (ImGui::Button(simulating ? "Pause" : "Play"))
    mViewer->simulate(!simulating);
  sameLineIfEnoughRoom(calcButtonWidth("Step"));
  if (ImGui::Button("Step") && mWorldNode)
    mWorldNode->stepOnce();
  sameLineIfEnoughRoom(calcButtonWidth("Rebuild"));
  if (ImGui::Button("Rebuild") && !mCurrentSceneId.empty())
    requestSceneSwitch(mCurrentSceneId);
  sameLineIfEnoughRoom(calcButtonWidth("Reset"));
  // Reset restores the scene's initial state by re-running its factory (a
  // fresh World), the same as Rebuild -- see the Rebuild/Reset note in
  // README or the phase-1 report for why world->reset() alone is not enough.
  if (ImGui::Button("Reset") && !mCurrentSceneId.empty())
    requestSceneSwitch(mCurrentSceneId);
  ImGui::EndDisabled();

  sameLineIfEnoughRoom(90.0f * scale);
  ImGui::Text("Time %.2f s", mCurrentWorld ? mCurrentWorld->getTime() : 0.0);
  sameLineIfEnoughRoom(72.0f * scale);
  ImGui::Text("FPS %.0f", static_cast<double>(ImGui::GetIO().Framerate));
  sameLineIfEnoughRoom(96.0f * scale);
  if (hasLiveRtfStats(mViewer.get(), mWorldNode.get()))
    ImGui::Text("Sim %.2fx", mWorldNode->getSmoothedRealTimeFactor());
  else
    ImGui::TextUnformatted("Sim --");
  sameLineIfEnoughRoom(100.0f * scale);
  ImGui::Text(
      "Steps/frame %zu",
      mWorldNode ? mWorldNode->getLastRefreshStepCount() : std::size_t{0});

  sameLineIfEnoughRoom(190.0f * scale);
  ImGui::TextUnformatted("Target");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(clampedItemWidth(116.0f * scale, 76.0f * scale));
  if (ImGui::SliderFloat(
          "##target_rtf",
          &mTargetRtf,
          0.1f,
          4.0f,
          "%.2fx",
          ImGuiSliderFlags_AlwaysClamp)
      && std::isfinite(mTargetRtf)) {
    mTargetRtf = std::clamp(mTargetRtf, 0.1f, 4.0f);
    if (mWorldNode)
      mWorldNode->setTargetRealTimeFactor(mTargetRtf);
  }
  sameLineIfEnoughRoom(90.0f * scale);
  if (ImGui::Checkbox("Gravity", &mGravityEnabled) && mCurrentWorld) {
    if (mGravityEnabled) {
      mCurrentWorld->setGravity(mSavedGravity);
    } else {
      mSavedGravity = mCurrentWorld->getGravity();
      mCurrentWorld->setGravity(Eigen::Vector3d::Zero());
    }
  }

  // Timestep control. Logarithmic so the 1e-5..1e-2 s range is usable; applied
  // directly to the active world (the same frame-loop thread ImGui renders on,
  // matching the Gravity/Target RTF writes above). NaN input is rejected by
  // keeping the previous value.
  sameLineIfEnoughRoom(170.0f * scale);
  ImGui::TextUnformatted("dt");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(clampedItemWidth(120.0f * scale, 86.0f * scale));
  if (ImGui::SliderFloat(
          "##timestep",
          &mTimeStep,
          1e-5f,
          1e-2f,
          "%.5f s",
          ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic)
      && std::isfinite(mTimeStep)) {
    mTimeStep = std::clamp(mTimeStep, 1e-5f, 1e-2f);
    if (mCurrentWorld)
      mCurrentWorld->setTimeStep(mTimeStep);
  }

  renderRuntimeControls();

  sameLineIfEnoughRoom(calcButtonWidth("View"));
  if (ImGui::Button("View"))
    ImGui::OpenPopup("##view_menu_popup");
  if (ImGui::BeginPopup("##view_menu_popup")) {
    renderViewMenu();
    ImGui::EndPopup();
  }

  sameLineIfEnoughRoom(120.0f * scale);
  mDragForce.renderToolbarStatus(mGuiScale);
}

//==============================================================================
void DemoHost::renderRuntimeControls()
{
  const float scale = static_cast<float>(mGuiScale);

  sameLineIfEnoughRoom(250.0f * scale);
  ImGui::SetNextItemWidth(clampedItemWidth(170.0f * scale, 120.0f * scale));
  ImGui::BeginDisabled(!mCurrentWorld || mAvailableCollisionDetectors.empty());
  const char* currentDetector
      = mCollisionDetectorIndex >= 0
                && mCollisionDetectorIndex
                       < static_cast<int>(mAvailableCollisionDetectors.size())
            ? mAvailableCollisionDetectors[mCollisionDetectorIndex].c_str()
            : mCurrentCollisionDetectorName.c_str();
  if (ImGui::BeginCombo("Collision", currentDetector)) {
    for (std::size_t i = 0; i < mAvailableCollisionDetectors.size(); ++i) {
      const bool selected = static_cast<int>(i) == mCollisionDetectorIndex;
      if (ImGui::Selectable(mAvailableCollisionDetectors[i].c_str(), selected))
        setCollisionDetectorByName(mAvailableCollisionDetectors[i]);
      if (selected)
        ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  ImGui::EndDisabled();

  sameLineIfEnoughRoom(128.0f * scale);
  ImGui::SetNextItemWidth(clampedItemWidth(90.0f * scale, 64.0f * scale));
  ImGui::BeginDisabled(!mCurrentWorld);
  int requestedThreads = mSimulationThreads;
  if (ImGui::InputInt("Threads", &requestedThreads, 1, 4)) {
    requestedThreads = std::clamp(requestedThreads, 0, 256);
    if (mCurrentWorld) {
      mCurrentWorld->setNumSimulationThreads(
          static_cast<std::size_t>(requestedThreads));
      mSimulationThreads
          = static_cast<int>(mCurrentWorld->getNumSimulationThreads());
      mPerformanceStatsPanel.reset();
    }
  }
  ImGui::EndDisabled();
}

//==============================================================================
void DemoHost::renderViewMenu()
{
  ImGui::BeginDisabled(!mCurrentWorld);
  if (ImGui::Button("Fit scene"))
    fitCameraToWorld();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Reset camera"))
    mViewer->home();

  ImGui::Separator();
  if (ImGui::Checkbox("Shadows", &mShadowsEnabled))
    applyShadowState();

  bool gridDisplayed = mGridVisual && mGridVisual->isDisplayed();
  if (ImGui::Checkbox("Grid", &gridDisplayed) && mGridVisual)
    mGridVisual->display(gridDisplayed);

  if (mGridVisual) {
    static const char* const kPlaneNames[] = {"XY", "YZ", "ZX"};
    int planeIndex = static_cast<int>(mGridVisual->getPlaneType());
    ImGui::SetNextItemWidth(120.0f * static_cast<float>(mGuiScale));
    if (ImGui::Combo(
            "Plane", &planeIndex, kPlaneNames, IM_ARRAYSIZE(kPlaneNames))) {
      mGridVisual->setPlaneType(
          static_cast<dart::gui::osg::GridVisual::PlaneType>(planeIndex));
    }

    float cellSize = static_cast<float>(mGridVisual->getMinorLineStepSize());
    ImGui::SetNextItemWidth(120.0f * static_cast<float>(mGuiScale));
    if (ImGui::SliderFloat(
            "Cell size",
            &cellSize,
            0.05f,
            5.0f,
            "%.2f m",
            ImGuiSliderFlags_AlwaysClamp)
        && std::isfinite(cellSize)) {
      mGridVisual->setMinorLineStepSize(std::clamp(cellSize, 0.05f, 5.0f));
    }

    int numCells = static_cast<int>(mGridVisual->getNumCells());
    ImGui::SetNextItemWidth(120.0f * static_cast<float>(mGuiScale));
    if (ImGui::SliderInt(
            "Num cells", &numCells, 2, 100, "%d", ImGuiSliderFlags_AlwaysClamp)
        && numCells > 0) {
      mGridVisual->setNumCells(static_cast<std::size_t>(numCells));
    }
  }

  ImGui::Separator();
  bool headlights = mViewer->checkHeadlights();
  if (ImGui::Checkbox("Headlights", &headlights))
    mViewer->switchHeadlights(headlights);

  ImGui::Separator();
  float guiScale = static_cast<float>(mGuiScale);
  ImGui::SetNextItemWidth(160.0f * static_cast<float>(mGuiScale));
  if (ImGui::SliderFloat(
          "GUI scale",
          &guiScale,
          0.75f,
          2.0f,
          "%.2fx",
          ImGuiSliderFlags_AlwaysClamp)
      && std::isfinite(guiScale)) {
    mGuiScale
        = dart::gui::osg::sanitizeGuiScale(std::clamp(guiScale, 0.75f, 2.0f));
    mViewer->getImGuiHandler()->setGuiScale(mGuiScale);
  }
}

//==============================================================================
void DemoHost::renderNavigator()
{
  ImGui::TextWrapped("%s", mStatusLine.c_str());
  ImGui::Separator();

  const float clearW = calcButtonWidth("Clear");
  const float searchW = std::max(
      ImGui::GetFontSize() * 5.0f,
      ImGui::GetContentRegionAvail().x - clearW
          - ImGui::GetStyle().ItemSpacing.x);
  ImGui::SetNextItemWidth(searchW);
  ImGui::InputTextWithHint(
      "##search", "Search demos...", mSearchBuf, sizeof(mSearchBuf));
  sameLineIfEnoughRoom(clearW);
  if (ImGui::Button("Clear"))
    mSearchBuf[0] = '\0';

  const std::string filterLower = toLower(mSearchBuf);

  std::size_t shown = 0;
  for (const auto& scene : mScenes) {
    if (matchesFilter(scene, filterLower))
      ++shown;
  }
  ImGui::Text("Showing %zu/%zu", shown, mScenes.size());
  ImGui::Separator();

  ImGui::BeginChild("##navigator_scroll", ImVec2(0.0f, 0.0f), false);
  for (const auto& category : mCategories) {
    bool anyVisible = false;
    for (const auto index : category.sceneIndices) {
      if (matchesFilter(mScenes[index], filterLower)) {
        anyVisible = true;
        break;
      }
    }
    if (!anyVisible)
      continue;

    if (ImGui::CollapsingHeader(
            category.name.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      for (const auto index : category.sceneIndices) {
        const DemoScene& scene = mScenes[index];
        if (!matchesFilter(scene, filterLower))
          continue;

        std::string label = scene.title;
        if (mPendingSceneId && *mPendingSceneId == scene.id)
          label += " (starting)";

        const bool selected = (scene.id == mCurrentSceneId);
        if (ImGui::Selectable(label.c_str(), selected))
          requestSceneSwitch(scene.id);
        if (ImGui::IsItemHovered() && !scene.summary.empty())
          ImGui::SetTooltip("%s", scene.summary.c_str());
      }
    }
  }
  ImGui::EndChild();
}

//==============================================================================
void DemoHost::renderInspectorSection()
{
  if (ImGui::CollapsingHeader("Scene Tree", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::BeginChild(
        "##scene_tree_scroll",
        ImVec2(
            0.0f,
            std::min(
                180.0f * static_cast<float>(mGuiScale),
                ImGui::GetContentRegionAvail().y * 0.45f)),
        true,
        ImGuiWindowFlags_HorizontalScrollbar);
    mInspector.renderTree(mCurrentWorld);
    ImGui::EndChild();
  }

  if (ImGui::CollapsingHeader("Inspector", ImGuiTreeNodeFlags_DefaultOpen)) {
    const bool paused = !mViewer->isSimulating();
    mInspector.renderDetail(paused, getWorldNode());

    if (auto* selected = mInspector.getSelection()) {
      if (!mDragForce.isGizmoAttached()) {
        if (ImGui::Button("Attach gizmo"))
          mDragForce.attachGizmo(selected, mCurrentWorld, mViewer.get());
      } else if (ImGui::Button("Detach gizmo")) {
        mDragForce.detachGizmo(mViewer.get());
      }
    }
  }
}

//==============================================================================
void DemoHost::renderSceneControlsSection()
{
  renderScenePanelDocumentation(mCurrentSceneDocumentation);

  if (mCurrentRenderPanel) {
    try {
      mCurrentRenderPanel();
    } catch (const std::exception& e) {
      log(LogEntry::Level::Error,
          "Scene panel for '" + mCurrentSceneTitle + "' failed: " + e.what()
              + " (panel disabled)");
      mCurrentRenderPanel = nullptr;
    } catch (...) {
      log(LogEntry::Level::Error,
          "Scene panel for '" + mCurrentSceneTitle
              + "' failed: unknown error (panel disabled)");
      mCurrentRenderPanel = nullptr;
    }
  } else {
    ImGui::TextDisabled("This demo has no custom controls.");
  }

  if (!mCurrentKeyActions.empty()) {
    ImGui::Separator();
    ImGui::TextUnformatted("Actions");
    for (auto& action : mCurrentKeyActions) {
      const std::string label = formatKeyActionLabel(action) + "##key_action_"
                                + std::to_string(action.key);
      ImGui::BeginDisabled(!action.callback);
      if (ImGui::Button(label.c_str()))
        invokeKeyAction(action);
      ImGui::EndDisabled();
    }
  }
}

//==============================================================================
void DemoHost::renderToolsSection()
{
  if (ImGui::CollapsingHeader(
          "Contact Visualizer", ImGuiTreeNodeFlags_DefaultOpen)) {
    mContactVisualizer.renderToggle();
  }

  if (ImGui::CollapsingHeader("Drag Force", ImGuiTreeNodeFlags_DefaultOpen)) {
    mDragForce.renderTunables(mGuiScale);
  }

  if (ImGui::CollapsingHeader("Profiler", ImGuiTreeNodeFlags_DefaultOpen)) {
    mProfiler.render();
  }
}

//==============================================================================
void DemoHost::renderScenePanel()
{
  const auto tabSelectionFlags = [this](ScenePanelTab tab) {
    return mHasRequestedScenePanelTab && mRequestedScenePanelTab == tab
               ? ImGuiTabItemFlags_SetSelected
               : static_cast<ImGuiTabItemFlags>(0);
  };

  if (ImGui::BeginTabBar("##scene_panel_tabs")) {
    if (ImGui::BeginTabItem(
            "Scene", nullptr, tabSelectionFlags(ScenePanelTab::Scene))) {
      ImGui::BeginChild(
          "##scene_panel_scroll",
          ImVec2(0.0f, 0.0f),
          false,
          ImGuiWindowFlags_AlwaysVerticalScrollbar);
      renderSceneControlsSection();
      ImGui::EndChild();
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem(
            "Inspector",
            nullptr,
            tabSelectionFlags(ScenePanelTab::Inspector))) {
      renderInspectorSection();
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem(
            "Tools", nullptr, tabSelectionFlags(ScenePanelTab::Tools))) {
      renderToolsSection();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  mHasRequestedScenePanelTab = false;
}

//==============================================================================
void DemoHost::renderDiagnostics()
{
  std::size_t numDofs = 0;
  const SceneCounts counts = collectSceneCounts(mCurrentWorld);
  if (mCurrentWorld) {
    for (std::size_t i = 0; i < mCurrentWorld->getNumSkeletons(); ++i) {
      const auto& skel = mCurrentWorld->getSkeleton(i);
      numDofs += skel->getNumDofs();
    }
  }

  const std::size_t numContacts
      = mCurrentWorld ? mCurrentWorld->getLastCollisionResult().getNumContacts()
                      : std::size_t{0};
  const std::size_t numConstraints
      = (mCurrentWorld && mCurrentWorld->getConstraintSolver())
            ? mCurrentWorld->getConstraintSolver()->getNumConstraints()
            : std::size_t{0};

  dart::gui::osg::PerformanceStatsData stats;
  stats.frame = static_cast<int>(mWorldNode ? mWorldNode->getStepCount() : 0u);
  stats.simTime = mCurrentWorld ? mCurrentWorld->getTime() : 0.0;
  stats.sceneName = mCurrentSceneTitle;
  stats.collisionDetectorName = getWorldCollisionDetectorName(mCurrentWorld);
  stats.simulationThreads
      = mCurrentWorld ? mCurrentWorld->getNumSimulationThreads() : 1u;
  stats.renderFps = static_cast<double>(ImGui::GetIO().Framerate);
  stats.targetRealTimeFactor = mTargetRtf;
  stats.timeStep = mCurrentWorld ? mCurrentWorld->getTimeStep() : 0.0;
  stats.lastStepMs = mWorldNode ? mWorldNode->getLastStepMs() : 0.0;
  stats.movingAverageStepMs
      = mWorldNode ? mWorldNode->getMovingAverageStepMs() : 0.0;
  stats.minStepMs = mWorldNode ? mWorldNode->getMinStepMs() : 0.0;
  stats.maxStepMs = mWorldNode ? mWorldNode->getMaxStepMs() : 0.0;
  stats.measuredSteps = mWorldNode ? mWorldNode->getStepTimingSamples() : 0u;
  stats.contacts = numContacts;
  stats.softBodies = counts.softBodies;
  stats.pointMasses = counts.pointMasses;
  stats.skeletons = counts.skeletons;
  stats.bodyNodes = counts.bodyNodes;
  mPerformanceStatsPanel.render(stats);

  ImGui::Text(
      "Steps %zu   Sim %.2f s   Steps/frame %zu   dt %.5f s   Skeletons %zu   "
      "Bodies %zu   Soft %zu   Points %zu   DOFs %zu   Contacts %zu   "
      "Constraints %zu",
      mWorldNode ? mWorldNode->getStepCount() : std::size_t{0},
      mCurrentWorld ? mCurrentWorld->getTime() : 0.0,
      mWorldNode ? mWorldNode->getLastRefreshStepCount() : std::size_t{0},
      mCurrentWorld ? mCurrentWorld->getTimeStep() : 0.0,
      counts.skeletons,
      counts.bodyNodes,
      counts.softBodies,
      counts.pointMasses,
      numDofs,
      numContacts,
      numConstraints);
  if (hasLiveRtfStats(mViewer.get(), mWorldNode.get())) {
    ImGui::Text(
        "RTF min/avg/max %.2f / %.2f / %.2f",
        mWorldNode->getLowestRealTimeFactor(),
        mWorldNode->getSmoothedRealTimeFactor(),
        mWorldNode->getHighestRealTimeFactor());
  }

  ImGui::Separator();
  renderLogSection(0.0f);
}

//==============================================================================
void DemoHost::renderLogSection(float height)
{
  ImGui::Checkbox("Info", &mLogShowInfo);
  sameLineIfEnoughRoom(95.0f * static_cast<float>(mGuiScale));
  ImGui::Checkbox("Warning", &mLogShowWarning);
  sameLineIfEnoughRoom(72.0f * static_cast<float>(mGuiScale));
  ImGui::Checkbox("Error", &mLogShowError);
  sameLineIfEnoughRoom(100.0f * static_cast<float>(mGuiScale));
  ImGui::Checkbox("Autoscroll", &mLogAutoscroll);
  sameLineIfEnoughRoom(calcButtonWidth("Copy"));
  if (ImGui::Button("Copy")) {
    std::string joined;
    for (const auto& entry : mLog) {
      if ((entry.level == LogEntry::Level::Info && !mLogShowInfo)
          || (entry.level == LogEntry::Level::Warning && !mLogShowWarning)
          || (entry.level == LogEntry::Level::Error && !mLogShowError))
        continue;
      joined += entry.message;
      joined += '\n';
    }
    ImGui::SetClipboardText(joined.c_str());
  }

  ImGui::BeginChild("##log_scroll", ImVec2(0.0f, height), true);
  for (const auto& entry : mLog) {
    if ((entry.level == LogEntry::Level::Info && !mLogShowInfo)
        || (entry.level == LogEntry::Level::Warning && !mLogShowWarning)
        || (entry.level == LogEntry::Level::Error && !mLogShowError))
      continue;

    ImVec4 color = ImGui::GetStyle().Colors[ImGuiCol_Text];
    if (entry.level == LogEntry::Level::Error)
      color = ImVec4(0.95f, 0.35f, 0.35f, 1.0f);
    else if (entry.level == LogEntry::Level::Warning)
      color = ImVec4(0.95f, 0.75f, 0.25f, 1.0f);

    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextWrapped("%s", entry.message.c_str());
    ImGui::PopStyleColor();
  }
  if (mLogAutoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
    ImGui::SetScrollHereY(1.0f);
  ImGui::EndChild();
}

} // namespace dart_demos
