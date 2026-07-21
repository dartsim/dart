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

#include "../../../examples/demos/HeadlessExactFbfFailFast.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using dart_demos::detail::evaluateHeadlessExactFbfFailFast;
using dart_demos::detail::evaluateHeadlessExactFbfSourceContinuation;
using dart_demos::detail::HeadlessExactFbfFailFastDiagnostics;
using dart_demos::detail::HeadlessExactFbfSourceContinuationDiagnostics;
using dart_demos::detail::HeadlessExactFbfSourceContinuationGroupDiagnostics;
using dart_demos::detail::HeadlessExactFbfSourceContinuationOutcome;
using dart_demos::detail::kHeadlessExactFbfResidualTolerance;

//==============================================================================
HeadlessExactFbfSourceContinuationGroupDiagnostics makeContinuationGroup(
    HeadlessExactFbfSourceContinuationOutcome outcome,
    std::size_t solveIndex,
    int iterations,
    int shrinks = 0,
    int shrinkCaps = 0)
{
  HeadlessExactFbfSourceContinuationGroupDiagnostics group;
  group.solveIndex = solveIndex;
  group.contactCount = 1u;
  group.outcome = outcome;
  group.sourceContinuationActive = true;
  group.iterations = iterations;
  group.shrinkIterations = shrinks;
  group.lineSearchShrinkCapCount = shrinkCaps;
  group.finalResidual
      = outcome == HeadlessExactFbfSourceContinuationOutcome::Success ? 5e-7
                                                                      : 5e-4;
  group.finalNaturalMapResidual = 5e-7;
  if (outcome == HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted) {
    group.plateauReferenceNaturalMapResidual = 5.04e-7;
    group.plateauRelativeImprovement
        = (group.plateauReferenceNaturalMapResidual
           - group.finalNaturalMapResidual)
          / group.plateauReferenceNaturalMapResidual;
  }
  group.lastInnerSolveStepSize = 1.0;
  group.correctionStepSize = shrinkCaps > 0 ? 0.7 : 1.0;
  return group;
}

//==============================================================================
HeadlessExactFbfSourceContinuationDiagnostics makeContinuationDiagnostics(
    std::vector<HeadlessExactFbfSourceContinuationGroupDiagnostics> groups)
{
  HeadlessExactFbfSourceContinuationDiagnostics diagnostics;
  diagnostics.requested = true;
  diagnostics.worldStateFinite = true;
  diagnostics.exactAttempts = groups.size();
  diagnostics.exactSolves = groups.size();
  diagnostics.stepExactAttempts = groups.size();
  diagnostics.stepExactSolves = groups.size();
  diagnostics.groups = std::move(groups);
  for (const auto& group : diagnostics.groups) {
    if (group.outcome
        == HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted) {
      ++diagnostics.stepPlateausAccepted;
    }
    if (group.outcome
        == HeadlessExactFbfSourceContinuationOutcome::MaxIterationsAccepted) {
      ++diagnostics.stepMaxIterationsAccepted;
    }
    diagnostics.stepLineSearchShrinks
        += static_cast<std::size_t>(group.shrinkIterations);
    diagnostics.stepLineSearchShrinkCaps
        += static_cast<std::size_t>(group.lineSearchShrinkCapCount);
  }
  if (!diagnostics.groups.empty()) {
    const auto& last = diagnostics.groups.back();
    diagnostics.lastActive = last.sourceContinuationActive;
    diagnostics.lastLineSearchShrinkCapCount = last.lineSearchShrinkCapCount;
    diagnostics.lastLineSearchShrinkCapReached
        = last.lineSearchShrinkCapCount > 0;
    diagnostics.lastCorrectionStepSize = last.correctionStepSize;
    diagnostics.lastInnerSolveStepSize = last.lastInnerSolveStepSize;
  }
  return diagnostics;
}

//==============================================================================
TEST(HeadlessExactFbfFailFast, AllowsNoAttemptNonfiniteResiduals)
{
  HeadlessExactFbfFailFastDiagnostics diagnostics;

  const auto decision = evaluateHeadlessExactFbfFailFast(diagnostics);

  EXPECT_FALSE(decision.triggered);
  EXPECT_EQ(decision.reason, nullptr);
}

//==============================================================================
TEST(HeadlessExactFbfFailFast, UsesStableTriggerPriority)
{
  HeadlessExactFbfFailFastDiagnostics diagnostics;
  diagnostics.exactAttempts = 1u;
  diagnostics.acceptedAtCap = 1u;
  diagnostics.exactFailures = 1u;
  diagnostics.boxedFallbacks = 1u;

  auto decision = evaluateHeadlessExactFbfFailFast(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "boxed_fallback");

  diagnostics.boxedFallbacks = 0u;
  decision = evaluateHeadlessExactFbfFailFast(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "exact_failure");

  diagnostics.exactFailures = 0u;
  decision = evaluateHeadlessExactFbfFailFast(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "iteration_cap");
}

//==============================================================================
TEST(HeadlessExactFbfFailFast, RejectsNonfiniteResidualAfterAttempt)
{
  HeadlessExactFbfFailFastDiagnostics diagnostics;
  diagnostics.exactAttempts = 1u;
  diagnostics.residual = kHeadlessExactFbfResidualTolerance;
  diagnostics.worstResidual = std::numeric_limits<double>::infinity();

  auto decision = evaluateHeadlessExactFbfFailFast(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "nonfinite_residual");

  diagnostics.residual = std::numeric_limits<double>::quiet_NaN();
  diagnostics.worstResidual = kHeadlessExactFbfResidualTolerance;
  decision = evaluateHeadlessExactFbfFailFast(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "nonfinite_residual");
}

//==============================================================================
TEST(HeadlessExactFbfFailFast, EnforcesInclusiveResidualTolerance)
{
  HeadlessExactFbfFailFastDiagnostics diagnostics;
  diagnostics.exactAttempts = 1u;
  diagnostics.residual = kHeadlessExactFbfResidualTolerance;
  diagnostics.worstResidual = kHeadlessExactFbfResidualTolerance;

  auto decision = evaluateHeadlessExactFbfFailFast(diagnostics);
  EXPECT_FALSE(decision.triggered);

  diagnostics.worstResidual = std::nextafter(
      kHeadlessExactFbfResidualTolerance,
      std::numeric_limits<double>::infinity());
  decision = evaluateHeadlessExactFbfFailFast(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "residual_tolerance_exceeded");

  diagnostics.worstResidual = kHeadlessExactFbfResidualTolerance;
  diagnostics.residual = std::nextafter(
      kHeadlessExactFbfResidualTolerance,
      std::numeric_limits<double>::infinity());
  decision = evaluateHeadlessExactFbfFailFast(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "residual_tolerance_exceeded");
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, AllowsAcceptedMixedGroupOutcomes)
{
  auto diagnostics = makeContinuationDiagnostics(
      {makeContinuationGroup(
           HeadlessExactFbfSourceContinuationOutcome::Success, 7u, 5),
       makeContinuationGroup(
           HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted,
           8u,
           30,
           8,
           1),
       makeContinuationGroup(
           HeadlessExactFbfSourceContinuationOutcome::MaxIterationsAccepted,
           9u,
           200)});

  const auto decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);

  EXPECT_FALSE(decision.triggered);
  EXPECT_EQ(decision.reason, nullptr);
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, CoversEveryDeclaredGateReason)
{
  const auto valid = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5)});
  std::vector<std::string> observed;
  const auto expectReason
      = [&observed](
            const HeadlessExactFbfSourceContinuationDiagnostics& diagnostics,
            const char* expected) {
          const auto decision
              = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
          ASSERT_TRUE(decision.triggered);
          ASSERT_NE(decision.reason, nullptr);
          EXPECT_EQ(std::string(decision.reason), expected);
          observed.emplace_back(decision.reason);
        };

  HeadlessExactFbfSourceContinuationDiagnostics diagnostics;
  expectReason(diagnostics, "source_continuation_not_requested");

  diagnostics = valid;
  diagnostics.boxedFallbacks = 1u;
  expectReason(diagnostics, "boxed_fallback");

  diagnostics = valid;
  diagnostics.exactFailures = 1u;
  expectReason(diagnostics, "exact_failure");

  diagnostics = valid;
  ++diagnostics.exactAttempts;
  expectReason(diagnostics, "cumulative_accounting_mismatch");

  diagnostics = valid;
  diagnostics.worldStateFinite = false;
  expectReason(diagnostics, "nonfinite_world_state");

  diagnostics = valid;
  diagnostics.groupHistoryTruncated = true;
  expectReason(diagnostics, "group_history_truncated");

  diagnostics = valid;
  ++diagnostics.stepExactAttempts;
  expectReason(diagnostics, "group_accounting_mismatch");

  diagnostics = valid;
  diagnostics.groups[0].contactCount = 0u;
  expectReason(diagnostics, "invalid_group_telemetry");

  diagnostics = valid;
  diagnostics.groups[0].sourceContinuationActive = false;
  diagnostics.lastActive = false;
  expectReason(diagnostics, "source_continuation_inactive");

  diagnostics = valid;
  diagnostics.groups[0].finalResidual
      = std::numeric_limits<double>::quiet_NaN();
  expectReason(diagnostics, "nonfinite_group_residual");

  diagnostics = valid;
  diagnostics.groups[0].correctionStepSize
      = std::numeric_limits<double>::quiet_NaN();
  diagnostics.lastCorrectionStepSize = std::numeric_limits<double>::quiet_NaN();
  expectReason(diagnostics, "nonfinite_group_step_size");

  diagnostics = valid;
  diagnostics.groups[0].correctionStepSize = 0.5;
  diagnostics.lastCorrectionStepSize = 0.5;
  expectReason(diagnostics, "group_step_size_relation_mismatch");

  diagnostics = valid;
  diagnostics.groups[0].iterations = 6;
  expectReason(diagnostics, "termination_timing_mismatch");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted, 0u, 30)});
  diagnostics.groups[0].plateauRelativeImprovement = 0.01;
  expectReason(diagnostics, "plateau_telemetry_mismatch");

  diagnostics = valid;
  diagnostics.groups[0].finalResidual = kHeadlessExactFbfResidualTolerance;
  expectReason(diagnostics, "success_tolerance_not_strict");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Invalid, 0u, 5)});
  expectReason(diagnostics, "unaccepted_group_outcome");

  diagnostics = valid;
  ++diagnostics.stepLineSearchShrinks;
  expectReason(diagnostics, "group_counter_mismatch");

  diagnostics = valid;
  diagnostics.lastActive = false;
  expectReason(diagnostics, "last_group_telemetry_mismatch");

  EXPECT_EQ(observed.size(), 18u);
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, UsesFailClosedPriority)
{
  HeadlessExactFbfSourceContinuationDiagnostics diagnostics;
  auto decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "source_continuation_not_requested");

  diagnostics.requested = true;
  diagnostics.worldStateFinite = true;
  diagnostics.boxedFallbacks = 1u;
  diagnostics.exactFailures = 1u;
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "boxed_fallback");

  diagnostics.boxedFallbacks = 0u;
  diagnostics.exactAttempts = 1u;
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "exact_failure");
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, RejectsInactiveAndTruncatedHistory)
{
  auto diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5)});
  diagnostics.groups[0].sourceContinuationActive = false;
  diagnostics.lastActive = false;
  auto decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "source_continuation_inactive");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5)});
  diagnostics.groupHistoryTruncated = true;
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "group_history_truncated");
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, RejectsNonfiniteGroupTelemetry)
{
  auto diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5)});
  diagnostics.groups[0].finalNaturalMapResidual
      = std::numeric_limits<double>::quiet_NaN();
  auto decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "nonfinite_group_residual");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5)});
  diagnostics.groups[0].lastInnerSolveStepSize
      = std::numeric_limits<double>::infinity();
  diagnostics.lastInnerSolveStepSize = std::numeric_limits<double>::infinity();
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "nonfinite_group_step_size");
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, RejectsAccountingAndCounterMismatch)
{
  auto diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5)});
  diagnostics.stepExactAttempts = 2u;
  auto decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "group_accounting_mismatch");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5, 2)});
  ++diagnostics.stepLineSearchShrinks;
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "group_counter_mismatch");
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, RejectsInvalidStatusAndToleranceEdge)
{
  auto diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Invalid, 0u, 5)});
  auto decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "unaccepted_group_outcome");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5)});
  diagnostics.groups[0].finalResidual = kHeadlessExactFbfResidualTolerance;
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "success_tolerance_not_strict");
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, RejectsShrinkCapAndGammaMismatch)
{
  auto diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted,
      0u,
      30,
      0,
      1)});
  auto decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "invalid_group_telemetry");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 0, 1)});
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "invalid_group_telemetry");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 1, 9)});
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "invalid_group_telemetry");

  diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::Success, 0u, 5)});
  diagnostics.groups[0].correctionStepSize = 0.7;
  diagnostics.lastCorrectionStepSize = 0.7;
  decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);
  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "group_step_size_relation_mismatch");
}

//==============================================================================
TEST(HeadlessExactFbfSourceContinuation, RejectsPlateauEvidenceMismatch)
{
  auto diagnostics = makeContinuationDiagnostics({makeContinuationGroup(
      HeadlessExactFbfSourceContinuationOutcome::PlateauAccepted, 0u, 30)});
  diagnostics.groups[0].plateauRelativeImprovement = 0.01;

  const auto decision = evaluateHeadlessExactFbfSourceContinuation(diagnostics);

  ASSERT_TRUE(decision.triggered);
  EXPECT_EQ(std::string(decision.reason), "plateau_telemetry_mismatch");
}

} // namespace
