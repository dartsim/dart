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

// PLAN-110 Dojo-style de-risking spike. This intentionally exposes no public
// API: it is a test-only scalar contact solve that mirrors the core Dojo-style
// idea DART needs to evaluate before promising a second differentiable rigid
// solver family: solve a relaxed contact complementarity equation on a central
// path, and obtain gradients by implicit differentiation of that solve.

#include <gtest/gtest.h>

#include <algorithm>

#include <cmath>

namespace {

struct ToyContactInput
{
  double position{0.001};
  double velocity{-1.0};
  double mass{2.0};
  double force{3.0};
  double timeStep{0.01};
  double gravity{-9.81};
  double centralPath{1e-5};
};

struct ToyContactResult
{
  double lambda{0.0};
  double gap{0.0};
  double nextPosition{0.0};
  double nextVelocity{0.0};
  double dLambdaDForce{0.0};
  double dLambdaDMass{0.0};
  double dNextPositionDForce{0.0};
  double dNextPositionDMass{0.0};
  double residual{0.0};
  int iterations{0};
};

//==============================================================================
ToyContactResult solveScalarCentralPathContact(const ToyContactInput& input)
{
  const double h = input.timeStep;
  const double invMass = 1.0 / input.mass;
  const double acceleration = input.force * invMass + input.gravity;
  const double freeVelocity = input.velocity + h * acceleration;
  const double freeGap = input.position + h * freeVelocity;

  // Positive root warm-start for lambda * (free_gap + h * lambda / m) = mu.
  double lambda = std::max(0.0, -freeGap * input.mass / h) + 1e-3;
  double gap = freeGap + h * lambda * invMass;
  double residual = lambda * gap - input.centralPath;
  int iterations = 0;
  for (; iterations < 32 && std::abs(residual) > 1e-14; ++iterations) {
    const double dResidualDLambda = gap + lambda * h * invMass;
    lambda = std::max(1e-12, lambda - residual / dResidualDLambda);
    gap = freeGap + h * lambda * invMass;
    residual = lambda * gap - input.centralPath;
  }

  const double dResidualDLambda = gap + lambda * h * invMass;
  const double dFreeGapDForce = h * h * invMass;
  const double dFreeGapDMass = -h * h * input.force * invMass * invMass;
  const double dGapDMassAtFixedLambda
      = dFreeGapDMass - h * lambda * invMass * invMass;
  const double dResidualDForce = lambda * dFreeGapDForce;
  const double dResidualDMass = lambda * dGapDMassAtFixedLambda;

  ToyContactResult result;
  result.lambda = lambda;
  result.gap = gap;
  result.nextVelocity = freeVelocity + lambda * invMass;
  result.nextPosition = input.position + h * result.nextVelocity;
  result.dLambdaDForce = -dResidualDForce / dResidualDLambda;
  result.dLambdaDMass = -dResidualDMass / dResidualDLambda;
  result.dNextPositionDForce
      = dFreeGapDForce + h * invMass * result.dLambdaDForce;
  result.dNextPositionDMass = dFreeGapDMass + h * invMass * result.dLambdaDMass
                              - h * lambda * invMass * invMass;
  result.residual = residual;
  result.iterations = iterations;
  return result;
}

//==============================================================================
double centralDifferenceNextPositionWrtForce(
    ToyContactInput input, double epsilon)
{
  input.force += epsilon;
  const double plus = solveScalarCentralPathContact(input).nextPosition;
  input.force -= 2.0 * epsilon;
  const double minus = solveScalarCentralPathContact(input).nextPosition;
  return (plus - minus) / (2.0 * epsilon);
}

//==============================================================================
double centralDifferenceNextPositionWrtMass(
    ToyContactInput input, double epsilon)
{
  input.mass += epsilon;
  const double plus = solveScalarCentralPathContact(input).nextPosition;
  input.mass -= 2.0 * epsilon;
  const double minus = solveScalarCentralPathContact(input).nextPosition;
  return (plus - minus) / (2.0 * epsilon);
}

} // namespace

//==============================================================================
TEST(DojoStyleIpmSpike, ScalarCentralPathContactGradientMatchesFiniteDifference)
{
  const ToyContactInput input;
  const ToyContactResult result = solveScalarCentralPathContact(input);

  ASSERT_TRUE(std::isfinite(result.lambda));
  ASSERT_TRUE(std::isfinite(result.gap));
  EXPECT_GT(result.lambda, 0.0);
  EXPECT_GT(result.gap, 0.0);
  EXPECT_NEAR(result.lambda * result.gap, input.centralPath, 1e-12);
  EXPECT_LT(std::abs(result.residual), 1e-12);
  EXPECT_LT(result.iterations, 10);

  const double forceFd = centralDifferenceNextPositionWrtForce(input, 1e-5);
  const double massFd = centralDifferenceNextPositionWrtMass(input, 1e-5);

  EXPECT_NEAR(result.dNextPositionDForce, forceFd, 1e-9);
  EXPECT_NEAR(result.dNextPositionDMass, massFd, 1e-9);
}
