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

#ifndef DART_EXAMPLES_DEMOS_HEADLESSEXACTFBFFAILFAST_HPP_
#define DART_EXAMPLES_DEMOS_HEADLESSEXACTFBFFAILFAST_HPP_

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart_demos {
namespace detail {

constexpr double kHeadlessExactFbfResidualTolerance = 1e-6;

//==============================================================================
/// Minimal exact-FBF diagnostics needed by the opt-in headless fail-fast gate.
struct HeadlessExactFbfFailFastDiagnostics
{
  std::size_t exactAttempts = 0u;
  std::size_t acceptedAtCap = 0u;
  std::size_t exactFailures = 0u;
  std::size_t boxedFallbacks = 0u;
  double residual = std::numeric_limits<double>::quiet_NaN();
  double worstResidual = std::numeric_limits<double>::quiet_NaN();
};

//==============================================================================
/// Result of evaluating one completed-step diagnostics record.
struct HeadlessExactFbfFailFastDecision
{
  bool triggered = false;
  const char* reason = nullptr;
};

//==============================================================================
/// Evaluates one completed-step diagnostics record in stable priority order.
///
/// Non-finite residuals are expected before the first exact solve, so they are
/// accepted while `exactAttempts` is zero. Once an attempt has occurred, both
/// the current and worst-to-date residual must be finite and within tolerance.
inline HeadlessExactFbfFailFastDecision evaluateHeadlessExactFbfFailFast(
    const HeadlessExactFbfFailFastDiagnostics& diagnostics,
    double residualTolerance = kHeadlessExactFbfResidualTolerance)
{
  if (diagnostics.boxedFallbacks > 0u)
    return {true, "boxed_fallback"};
  if (diagnostics.exactFailures > 0u)
    return {true, "exact_failure"};
  if (diagnostics.acceptedAtCap > 0u)
    return {true, "iteration_cap"};
  if (diagnostics.exactAttempts == 0u)
    return {};
  if (!std::isfinite(diagnostics.residual)
      || !std::isfinite(diagnostics.worstResidual)) {
    return {true, "nonfinite_residual"};
  }
  if (diagnostics.residual > residualTolerance
      || diagnostics.worstResidual > residualTolerance) {
    return {true, "residual_tolerance_exceeded"};
  }
  return {};
}

//==============================================================================
enum class HeadlessExactFbfSourceContinuationOutcome
{
  Success,
  PlateauAccepted,
  MaxIterationsAccepted,
  Invalid,
};

//==============================================================================
struct HeadlessExactFbfSourceContinuationGroupDiagnostics
{
  std::size_t solveIndex = 0u;
  std::size_t contactCount = 0u;
  HeadlessExactFbfSourceContinuationOutcome outcome
      = HeadlessExactFbfSourceContinuationOutcome::Invalid;
  bool sourceContinuationActive = false;
  int iterations = 0;
  int shrinkIterations = -1;
  int lineSearchShrinkCapCount = -1;
  double finalResidual = std::numeric_limits<double>::quiet_NaN();
  double finalNaturalMapResidual = std::numeric_limits<double>::quiet_NaN();
  double plateauReferenceNaturalMapResidual
      = std::numeric_limits<double>::quiet_NaN();
  double plateauRelativeImprovement = std::numeric_limits<double>::quiet_NaN();
  double correctionStepSize = std::numeric_limits<double>::quiet_NaN();
  double lastInnerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
};

//==============================================================================
/// Completed-step accounting for the separate source-continuation capture gate.
struct HeadlessExactFbfSourceContinuationDiagnostics
{
  bool requested = false;
  bool lastActive = false;
  bool worldStateFinite = false;
  bool groupHistoryTruncated = false;
  bool lastLineSearchShrinkCapReached = false;
  int lastLineSearchShrinkCapCount = 0;
  double lastCorrectionStepSize = std::numeric_limits<double>::quiet_NaN();
  double lastInnerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
  std::size_t exactAttempts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t exactFailures = 0u;
  std::size_t boxedFallbacks = 0u;
  std::size_t stepExactAttempts = 0u;
  std::size_t stepExactSolves = 0u;
  std::size_t stepPlateausAccepted = 0u;
  std::size_t stepMaxIterationsAccepted = 0u;
  std::size_t stepLineSearchShrinks = 0u;
  std::size_t stepLineSearchShrinkCaps = 0u;
  std::vector<HeadlessExactFbfSourceContinuationGroupDiagnostics> groups;
};

//==============================================================================
/// Allows only complete, finite, solver-accepted source-continuation outcomes.
inline HeadlessExactFbfFailFastDecision
evaluateHeadlessExactFbfSourceContinuation(
    const HeadlessExactFbfSourceContinuationDiagnostics& diagnostics)
{
  if (!diagnostics.requested)
    return {true, "source_continuation_not_requested"};
  if (diagnostics.boxedFallbacks > 0u)
    return {true, "boxed_fallback"};
  if (diagnostics.exactFailures > 0u)
    return {true, "exact_failure"};
  if (diagnostics.exactAttempts
      != diagnostics.exactSolves + diagnostics.exactFailures) {
    return {true, "cumulative_accounting_mismatch"};
  }
  if (!diagnostics.worldStateFinite)
    return {true, "nonfinite_world_state"};
  if (diagnostics.groupHistoryTruncated)
    return {true, "group_history_truncated"};
  if (diagnostics.stepExactAttempts != diagnostics.stepExactSolves
      || diagnostics.groups.size() != diagnostics.stepExactAttempts) {
    return {true, "group_accounting_mismatch"};
  }

  std::size_t plateaus = 0u;
  std::size_t maxIterations = 0u;
  std::size_t shrinks = 0u;
  std::size_t shrinkCaps = 0u;
  for (std::size_t index = 0u; index < diagnostics.groups.size(); ++index) {
    const auto& group = diagnostics.groups[index];
    if (group.contactCount == 0u
        || (index > 0u
            && group.solveIndex
                   != diagnostics.groups[index - 1u].solveIndex + 1u)) {
      return {true, "invalid_group_telemetry"};
    }
    if (!group.sourceContinuationActive)
      return {true, "source_continuation_inactive"};
    if (group.iterations < 0 || group.shrinkIterations < 0
        || group.lineSearchShrinkCapCount < 0)
      return {true, "invalid_group_telemetry"};
    if (group.lineSearchShrinkCapCount > group.shrinkIterations
        || (group.shrinkIterations > 0 && group.iterations == 0)
        || static_cast<long long>(group.shrinkIterations)
               > 8LL * static_cast<long long>(group.iterations)
        || group.lineSearchShrinkCapCount > group.iterations) {
      return {true, "invalid_group_telemetry"};
    }
    if (!std::isfinite(group.finalResidual) || group.finalResidual < 0.0
        || !std::isfinite(group.finalNaturalMapResidual)
        || group.finalNaturalMapResidual < 0.0) {
      return {true, "nonfinite_group_residual"};
    }
    if (group.iterations > 0
        && (!std::isfinite(group.correctionStepSize)
            || group.correctionStepSize <= 0.0
            || !std::isfinite(group.lastInnerSolveStepSize)
            || group.lastInnerSolveStepSize <= 0.0)) {
      return {true, "nonfinite_group_step_size"};
    }
    if (group.iterations > 0
        && ((group.lineSearchShrinkCapCount == 0
             && group.correctionStepSize != group.lastInnerSolveStepSize)
            || (group.lineSearchShrinkCapCount > 0
                && group.correctionStepSize > group.lastInnerSolveStepSize))) {
      return {true, "group_step_size_relation_mismatch"};
    }

    switch (group.outcome) {
      case HeadlessExactFbfSourceContinuationOutcome::Success:
        if (group.iterations > 200
            || (group.iterations > 0 && group.iterations % 5 != 0)) {
          return {true, "termination_timing_mismatch"};
        }
        if ((group.iterations == 0 && group.finalNaturalMapResidual >= 1e-6)
            || (group.iterations > 0 && group.finalResidual >= 1e-6)) {
          return {true, "success_tolerance_not_strict"};
        }
        break;
      case HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted:
        if (group.iterations < 30 || group.iterations > 200
            || group.iterations % 5 != 0) {
          return {true, "termination_timing_mismatch"};
        }
        if (!std::isfinite(group.plateauReferenceNaturalMapResidual)
            || group.plateauReferenceNaturalMapResidual <= 0.0
            || !std::isfinite(group.plateauRelativeImprovement)) {
          return {true, "plateau_telemetry_mismatch"};
        }
        {
          const double expectedImprovement
              = (group.plateauReferenceNaturalMapResidual
                 - group.finalNaturalMapResidual)
                / group.plateauReferenceNaturalMapResidual;
          const double scale = std::max(1.0, std::abs(expectedImprovement));
          if (std::abs(group.plateauRelativeImprovement - expectedImprovement)
                  > 8.0 * std::numeric_limits<double>::epsilon() * scale
              || !(group.plateauRelativeImprovement < 0.01)) {
            return {true, "plateau_telemetry_mismatch"};
          }
        }
        ++plateaus;
        break;
      case HeadlessExactFbfSourceContinuationOutcome::MaxIterationsAccepted:
        if (group.iterations != 200)
          return {true, "termination_timing_mismatch"};
        ++maxIterations;
        break;
      case HeadlessExactFbfSourceContinuationOutcome::Invalid:
        return {true, "unaccepted_group_outcome"};
    }
    if (group.outcome
            != HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted
        && (std::isfinite(group.plateauReferenceNaturalMapResidual)
            || std::isfinite(group.plateauRelativeImprovement))) {
      return {true, "plateau_telemetry_mismatch"};
    }
    shrinks += static_cast<std::size_t>(group.shrinkIterations);
    shrinkCaps += static_cast<std::size_t>(group.lineSearchShrinkCapCount);
  }

  if (plateaus != diagnostics.stepPlateausAccepted
      || maxIterations != diagnostics.stepMaxIterationsAccepted
      || shrinks != diagnostics.stepLineSearchShrinks
      || shrinkCaps != diagnostics.stepLineSearchShrinkCaps) {
    return {true, "group_counter_mismatch"};
  }
  if (!diagnostics.groups.empty()) {
    const auto& last = diagnostics.groups.back();
    const auto sameNumber = [](double lhs, double rhs) {
      return lhs == rhs || (std::isnan(lhs) && std::isnan(rhs));
    };
    if (diagnostics.lastActive != last.sourceContinuationActive
        || diagnostics.lastLineSearchShrinkCapCount
               != last.lineSearchShrinkCapCount
        || diagnostics.lastLineSearchShrinkCapReached
               != (last.lineSearchShrinkCapCount > 0)
        || !sameNumber(
            diagnostics.lastCorrectionStepSize, last.correctionStepSize)
        || !sameNumber(
            diagnostics.lastInnerSolveStepSize, last.lastInnerSolveStepSize)) {
      return {true, "last_group_telemetry_mismatch"};
    }
  }
  return {};
}

} // namespace detail
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_HEADLESSEXACTFBFFAILFAST_HPP_
