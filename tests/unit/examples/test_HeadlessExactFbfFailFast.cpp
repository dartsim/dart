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

namespace {

using dart_demos::detail::evaluateHeadlessExactFbfFailFast;
using dart_demos::detail::HeadlessExactFbfFailFastDiagnostics;
using dart_demos::detail::kHeadlessExactFbfResidualTolerance;

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

} // namespace
