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

#include <dart/math/detail/ExactCoulombFbfSolver.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include <cmath>

namespace {

auto makeDenseDelassusOperator(Eigen::MatrixXd delassus)
{
  return [delassus](
             const Eigen::Ref<const Eigen::VectorXd>& input,
             Eigen::Ref<Eigen::VectorXd> output) {
    output.noalias() = delassus * input;
  };
}

void expectVectorNear(
    const Eigen::VectorXd& actual,
    const Eigen::VectorXd& expected,
    double tolerance)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (Eigen::Index i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], tolerance);
  }
}

void expectVectorNear(
    const Eigen::VectorXd& actual, const Eigen::VectorXd& expected)
{
  expectVectorNear(actual, expected, 1e-12);
}

dart::math::detail::ExactCoulombContactProblem makeOneContactProblem(
    const Eigen::Vector3d& freeVelocity, double coefficient)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity = freeVelocity;
  problem.coefficients = Eigen::VectorXd::Constant(1, coefficient);
  return problem;
}

Eigen::Matrix3d makeCrossCoupledLocalHessian()
{
  Eigen::Matrix3d hessian;
  hessian << 4.0, 1.0, 0.5, 1.0, 3.0, 0.2, 0.5, 0.2, 2.0;
  return hessian;
}

double computeLocalQuadraticObjective(
    const Eigen::Matrix3d& hessian,
    const Eigen::Vector3d& linearTerm,
    const Eigen::Vector3d& reaction)
{
  return 0.5 * reaction.dot(hessian * reaction) + linearTerm.dot(reaction);
}

void expectLocalConeKkt(
    const Eigen::Matrix3d& hessian,
    const Eigen::Vector3d& linearTerm,
    double coefficient,
    const Eigen::Vector3d& reaction,
    double tolerance = 1e-10)
{
  const Eigen::Vector3d gradient = hessian * reaction + linearTerm;
  const double tangentNorm = std::hypot(reaction[1], reaction[2]);
  const double gradientTangentNorm = std::hypot(gradient[1], gradient[2]);
  EXPECT_GE(reaction[0], -tolerance);
  EXPECT_LE(tangentNorm, coefficient * reaction[0] + tolerance);
  EXPECT_GE(gradient[0], coefficient * gradientTangentNorm - tolerance);
  EXPECT_NEAR(reaction.dot(gradient), 0.0, tolerance);
}

} // namespace

TEST(ExactCoulombFbfSolver, ComputesSafeStepFromSpectralBound)
{
  const auto problem = makeOneContactProblem(Eigen::Vector3d::Zero(), 0.5);
  const Eigen::Matrix3d delassus = 4.0 * Eigen::Matrix3d::Identity();

  const double safeStep
      = dart::math::detail::computeExactCoulombFbfSafeStepSize(
          problem, makeDenseDelassusOperator(delassus), 4);

  EXPECT_NEAR(safeStep, 0.25, 1e-12);
}

TEST(ExactCoulombFbfSolver, AutomaticGammaMayUseScaledSafeStep)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = 4.0 * Eigen::Matrix3d::Identity();
  const auto unusedInnerSolver = [](const auto&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    double,
                                    Eigen::Ref<Eigen::VectorXd>) {
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.stepSizeScale = 2.0;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      Eigen::Vector3d::Zero(),
      makeDenseDelassusOperator(delassus),
      unusedInnerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_NEAR(result.safeStepSize, 0.25, 1e-12);
  EXPECT_NEAR(result.stepSize, 0.5, 1e-12);
}

TEST(ExactCoulombFbfSolver, ExplicitGammaCapsAtUnscaledSafeStep)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = 4.0 * Eigen::Matrix3d::Identity();
  const auto unusedInnerSolver = [](const auto&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    double,
                                    Eigen::Ref<Eigen::VectorXd>) {
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 10.0;
  options.stepSizeScale = 2.0;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      Eigen::Vector3d::Zero(),
      makeDenseDelassusOperator(delassus),
      unusedInnerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_NEAR(result.safeStepSize, 0.25, 1e-12);
  EXPECT_NEAR(result.stepSize, result.safeStepSize, 1e-12);
}

TEST(ExactCoulombFbfSolver, CapsGammaForGoodInitialWarmStart)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = 4.0 * Eigen::Matrix3d::Identity();
  const auto unusedInnerSolver = [](const auto&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    double,
                                    Eigen::Ref<Eigen::VectorXd>) {
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.stepSizeScale = 8.0;
  options.initialResidualStepSizeCapThreshold = 2.0;
  options.initialResidualStepSizeCap = 0.1;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      Eigen::Vector3d::Zero(),
      makeDenseDelassusOperator(delassus),
      unusedInnerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_TRUE(result.initialStepSizeCapApplied);
  EXPECT_NEAR(result.safeStepSize, 0.25, 1e-12);
  EXPECT_NEAR(result.uncappedInitialStepSize, 2.0, 1e-12);
  EXPECT_NEAR(result.stepSize, options.initialResidualStepSizeCap, 1e-12);
  EXPECT_TRUE(
      dart::math::detail::isFiniteCoulombConeResidual(result.initialResidual));
}

TEST(ExactCoulombFbfSolver, CanUseNaturalMapResidualForWarmStartGammaCap)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = 4.0 * Eigen::Matrix3d::Identity();
  const auto unusedInnerSolver = [](const auto&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    double,
                                    Eigen::Ref<Eigen::VectorXd>) {
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.stepSizeScale = 8.0;
  options.initialResidualStepSizeCapThreshold = 2.0;
  options.initialResidualStepSizeCap = 0.1;
  options.useNaturalMapResidualForInitialStepSizeCap = true;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      Eigen::Vector3d::Zero(),
      makeDenseDelassusOperator(delassus),
      unusedInnerSolver,
      options);

  EXPECT_TRUE(result.initialStepSizeCapApplied);
  EXPECT_NEAR(result.initialNaturalMapResidual, 1.0, 1e-12);
  EXPECT_NEAR(
      result.naturalMapResidual, result.initialNaturalMapResidual, 1e-12);
  EXPECT_NEAR(result.stepSize, options.initialResidualStepSizeCap, 1e-12);
}

TEST(ExactCoulombFbfSolver, LeavesGammaUncappedForLargeInitialResidual)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = 4.0 * Eigen::Matrix3d::Identity();
  const auto unusedInnerSolver = [](const auto&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    double,
                                    Eigen::Ref<Eigen::VectorXd>) {
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.stepSizeScale = 8.0;
  options.initialResidualStepSizeCapThreshold = 0.0;
  options.initialResidualStepSizeCap = 0.1;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      Eigen::Vector3d::Zero(),
      makeDenseDelassusOperator(delassus),
      unusedInnerSolver,
      options);

  EXPECT_FALSE(result.initialStepSizeCapApplied);
  EXPECT_NEAR(result.uncappedInitialStepSize, 2.0, 1e-12);
  EXPECT_NEAR(result.stepSize, result.uncappedInitialStepSize, 1e-12);
}

TEST(ExactCoulombFbfSolver, ScalesExplicitSafeBoundAndClampsGammaRange)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = 4.0 * Eigen::Matrix3d::Identity();
  const auto unusedInnerSolver = [](const auto&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    double,
                                    Eigen::Ref<Eigen::VectorXd>) {
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 100.0;
  options.explicitStepSizeSafeBoundScale = 10.0;
  options.minimumStepSize = 1e-6;
  options.maximumStepSize = 1.0;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      Eigen::Vector3d::Zero(),
      makeDenseDelassusOperator(delassus),
      unusedInnerSolver,
      options);

  EXPECT_NEAR(result.safeStepSize, 0.25, 1e-12);
  EXPECT_NEAR(result.uncappedInitialStepSize, options.maximumStepSize, 1e-12);
  EXPECT_NEAR(result.stepSize, options.maximumStepSize, 1e-12);
}

TEST(ExactCoulombFbfSolver, ComputesPaperResidualScales)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(0.0, -3.0, 4.0), 0.5);
  const Eigen::Matrix3d delassus = 2.0 * Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction(2.0, 0.0, 0.0);

  const auto scales = dart::math::detail::computeExactCoulombFbfResidualScales(
      problem, initialReaction, makeDenseDelassusOperator(delassus), 8);

  EXPECT_NEAR(scales.reactionScale, 2.5, 1e-12);
  EXPECT_NEAR(scales.velocityScale, std::sqrt(67.25), 1e-12);
}

TEST(ExactCoulombFbfSolver, ReusesSpectralEstimateForResidualScales)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  int operatorCalls = 0;
  const auto applyDelassus = [&delassus, &operatorCalls](
                                 const Eigen::Ref<const Eigen::VectorXd>& input,
                                 Eigen::Ref<Eigen::VectorXd> output) {
    ++operatorCalls;
    output.noalias() = delassus * input;
  };
  const auto unusedInnerSolver = [](const auto&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                    double,
                                    Eigen::Ref<Eigen::VectorXd>) {
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.maxOuterIterations = 0;
  options.spectralIterations = 4;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      Eigen::Vector3d::Zero(),
      applyDelassus,
      unusedInnerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  // Four power iterations, one Rayleigh product, one initial-scale velocity,
  // and one initial residual product. A duplicate spectral estimate would add
  // five more products.
  EXPECT_EQ(operatorCalls, 7);
}

TEST(ExactCoulombLocalConeQuadratic, ReturnsZeroForDualConeLinearTerm)
{
  const Eigen::Matrix3d hessian = makeCrossCoupledLocalHessian();
  const Eigen::Vector3d linearTerm(1.0, 1.0, 0.0);
  constexpr double kCoefficient = 0.6;

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, kCoefficient, factorization));

  Eigen::Vector3d reaction;
  ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
      factorization, linearTerm, reaction));
  expectVectorNear(reaction, Eigen::Vector3d::Zero());
  expectLocalConeKkt(hessian, linearTerm, kCoefficient, reaction);
}

TEST(ExactCoulombLocalConeQuadratic, ReturnsUnconstrainedInteriorMinimizer)
{
  const Eigen::Matrix3d hessian = makeCrossCoupledLocalHessian();
  const Eigen::Vector3d expected(2.0, 0.3, -0.4);
  const Eigen::Vector3d linearTerm(-8.1, -2.82, -0.26);
  constexpr double kCoefficient = 0.6;

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, kCoefficient, factorization));

  Eigen::Vector3d reaction;
  ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
      factorization, linearTerm, reaction));
  expectVectorNear(reaction, expected, 1e-12);
  expectLocalConeKkt(hessian, linearTerm, kCoefficient, reaction);
}

TEST(
    ExactCoulombLocalConeQuadratic,
    AcceptsCapturedInteriorMinimizerWithSmallGradient)
{
  Eigen::Matrix3d hessian;
  hessian << 6.2508524116994826, 0.08447251325082239, -0.071655259667449872,
      0.08447251325082239, 6.5776044639887665, 0.02118378221216205,
      -0.071655259667449872, 0.02118378221216205, 6.3474515650102292;
  const Eigen::Vector3d linearTerm(
      -259.58179375663889, 35.903906297983241, -67.104239447047505);
  const Eigen::Vector3d capturedReaction(
      41.735732449040448, -6.0301265110008266, 11.063111606382517);
  constexpr double kCoefficient = 0.8;

  const Eigen::Vector3d capturedGradient
      = hessian * capturedReaction + linearTerm;
  const double coneTolerance = 4096.0 * std::numeric_limits<double>::epsilon();
  const double reactionTolerance
      = coneTolerance * (std::max)(1.0, capturedReaction.norm());
  const double gradientTolerance
      = coneTolerance * (std::max)(1.0, capturedGradient.norm());

  EXPECT_LT(
      std::hypot(capturedReaction[1], capturedReaction[2]) + reactionTolerance,
      kCoefficient * capturedReaction[0]);
  EXPECT_LE(capturedGradient.norm(), gradientTolerance);

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, kCoefficient, factorization));
  EXPECT_TRUE(dart::math::detail::isExactCoulombLocalConeQuadraticKktPoint(
      factorization, linearTerm, capturedReaction));

  Eigen::Vector3d reaction;
  ASSERT_TRUE(
      dart::math::detail::trySolveExactCoulombLocalConeQuadraticAnalytically(
          factorization, linearTerm, reaction));
  expectVectorNear(reaction, capturedReaction, 1e-12);
}

TEST(
    ExactCoulombLocalConeQuadratic,
    InteriorStationarityDoesNotRequireRedundantScaledDotProduct)
{
  const Eigen::Matrix3d hessian = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d expectedReaction(4.0, 0.0, 0.0);
  constexpr double kCoefficient = 0.8;
  const double coneTolerance = 4096.0 * std::numeric_limits<double>::epsilon();
  const Eigen::Vector3d acceptedGradient(0.75 * coneTolerance, 0.0, 0.0);
  const Eigen::Vector3d linearTerm = -expectedReaction + acceptedGradient;
  const Eigen::Vector3d actualGradient
      = hessian * expectedReaction + linearTerm;
  const double gradientTolerance
      = coneTolerance * (std::max)(1.0, actualGradient.norm());
  const double gapTolerance
      = 8192.0 * std::numeric_limits<double>::epsilon()
        * (1.0 + expectedReaction.norm() * actualGradient.norm());

  ASSERT_LE(actualGradient.norm(), gradientTolerance);
  ASSERT_GT(std::abs(expectedReaction.dot(actualGradient)), gapTolerance);

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, kCoefficient, factorization));
  EXPECT_TRUE(dart::math::detail::isExactCoulombLocalConeQuadraticKktPoint(
      factorization, linearTerm, expectedReaction));

  Eigen::Vector3d reaction;
  ASSERT_TRUE(
      dart::math::detail::trySolveExactCoulombLocalConeQuadraticAnalytically(
          factorization, linearTerm, reaction));
  expectVectorNear(reaction, expectedReaction, 1e-12);
}

TEST(ExactCoulombLocalConeQuadratic, SolvesCrossCoupledBoundaryMinimizer)
{
  const Eigen::Matrix3d hessian = makeCrossCoupledLocalHessian();
  const Eigen::Vector3d linearTerm(-6.57, -4.328, -2.394);
  const Eigen::Vector3d expected(1.5, 0.72, 0.54);
  constexpr double kCoefficient = 0.6;

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, kCoefficient, factorization));

  Eigen::Vector3d reaction;
  ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
      factorization, linearTerm, reaction));
  expectVectorNear(reaction, expected, 1e-11);
  expectLocalConeKkt(hessian, linearTerm, kCoefficient, reaction);
}

TEST(
    ExactCoulombLocalConeQuadratic,
    InverseEuclideanProjectionDiffersFromExactMetricProjection)
{
  const Eigen::Matrix3d hessian = makeCrossCoupledLocalHessian();
  const Eigen::Vector3d linearTerm(-6.57, -4.328, -2.394);
  constexpr double kCoefficient = 0.6;

  Eigen::Matrix3d symmetricHessian;
  Eigen::Matrix3d inverseHessian;
  ASSERT_TRUE(dart::math::detail::prepareExactCoulombLocalSpdInverse(
      hessian, symmetricHessian, inverseHessian));
  Eigen::Vector3d inverseProjected;
  ASSERT_TRUE(
      dart::math::detail::solveExactCoulombLocalConeInverseEuclideanProjection(
          inverseHessian, linearTerm, kCoefficient, inverseProjected));
  const Eigen::Vector3d expectedInverseProjected(
      1.5035607320294018, 0.7007049576384029, 0.5682100996156688);
  expectVectorNear(inverseProjected, expectedInverseProjected, 1e-12);

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, kCoefficient, factorization));
  Eigen::Vector3d exact;
  ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
      factorization, linearTerm, exact));
  EXPECT_FALSE(dart::math::detail::isExactCoulombLocalConeQuadraticKktPoint(
      factorization, linearTerm, inverseProjected));
  EXPECT_TRUE(dart::math::detail::isExactCoulombLocalConeQuadraticKktPoint(
      factorization, linearTerm, exact));
  EXPECT_GT(
      std::abs(inverseProjected.dot(hessian * inverseProjected + linearTerm)),
      1e-3);
}

TEST(ExactCoulombLocalConeQuadratic, SolvesBoundaryMinimizerAtPositivePole)
{
  const Eigen::Matrix3d hessian = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d linearTerm(0.0, -2.0, 0.0);
  const Eigen::Vector3d expected(0.8, 0.4, 0.0);
  constexpr double kCoefficient = 0.5;

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, kCoefficient, factorization));

  Eigen::Vector3d reaction;
  ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
      factorization, linearTerm, reaction));
  expectVectorNear(reaction, expected, 1e-12);
  expectLocalConeKkt(hessian, linearTerm, kCoefficient, reaction);
}

TEST(ExactCoulombLocalConeQuadratic, SolvesFrictionlessNormalRay)
{
  const Eigen::Matrix3d hessian = makeCrossCoupledLocalHessian();
  const Eigen::Vector3d linearTerm(-6.0, 5.0, -7.0);
  const Eigen::Vector3d expected(1.5, 0.0, 0.0);

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, 0.0, factorization));

  Eigen::Vector3d reaction;
  ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
      factorization, linearTerm, reaction));
  expectVectorNear(reaction, expected, 1e-12);
  expectLocalConeKkt(hessian, linearTerm, 0.0, reaction);
}

TEST(ExactCoulombLocalConeQuadratic, MatchesGeneratedBoundaryKktOracles)
{
  constexpr int kCases = 32;
  for (int index = 0; index < kCases; ++index) {
    Eigen::Matrix3d generator;
    const double phase = 0.37 * index;
    generator << 1.0 + 0.03 * index, 0.2 * std::sin(phase),
        -0.1 * std::cos(phase), 0.15 * std::cos(0.7 * phase),
        0.8 + 0.01 * index, 0.12 * std::sin(1.3 * phase),
        -0.08 * std::sin(phase), 0.09 * std::cos(1.1 * phase),
        0.6 + 0.02 * index;
    const Eigen::Matrix3d hessian
        = generator.transpose() * generator
          + (0.2 + 0.01 * index) * Eigen::Matrix3d::Identity();
    const double coefficient = 0.15 + 0.025 * (index % 20);
    const double angle = 0.31 + 0.47 * index;
    const Eigen::Vector2d tangentDirection(std::cos(angle), std::sin(angle));
    const double normal = 0.25 + 0.08 * index;
    const double dualMagnitude = 0.4 + 0.05 * (index % 9);

    Eigen::Vector3d expected;
    expected[0] = normal;
    expected.tail<2>() = coefficient * normal * tangentDirection;
    Eigen::Vector3d gradient;
    gradient[0] = coefficient * dualMagnitude;
    gradient.tail<2>() = -dualMagnitude * tangentDirection;
    const Eigen::Vector3d linearTerm = gradient - hessian * expected;

    dart::math::detail::ExactCoulombLocalConeQuadraticFactorization
        factorization;
    ASSERT_TRUE(
        dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
            hessian, coefficient, factorization))
        << "case " << index;
    Eigen::Vector3d reaction;
    ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
        factorization, linearTerm, reaction))
        << "case " << index;
    expectVectorNear(reaction, expected, 2e-10);
    expectLocalConeKkt(hessian, linearTerm, coefficient, reaction, 2e-10);
  }
}

TEST(
    ExactCoulombLocalConeQuadratic,
    FallbackSolvesObservedNearGeneralizedPoleSystems)
{
  struct RegressionCase
  {
    Eigen::Matrix3d hessian;
    Eigen::Vector3d linearTerm;
  };

  const std::vector<RegressionCase> cases{
      {(Eigen::Matrix3d() << 8.8694614805585577,
        0.15660946717152119,
        0.034864384169893702,
        0.15660946717152119,
        8.2393607911363222,
        -0.1485933754924732,
        0.034864384169893702,
        -0.1485933754924732,
        8.4414104883828571)
           .finished(),
       Eigen::Vector3d(
           -0.00054331100851662956,
           -0.080070927918681561,
           0.019111755192014482)},
      {(Eigen::Matrix3d() << 9.8482144407794969,
        -0.0043204662416178024,
        -0.00051226759631371049,
        -0.0043204662416178024,
        10.654270300381615,
        -0.027442870623253166,
        -0.00051226759631371049,
        -0.027442870623253166,
        10.142705037544077)
           .finished(),
       Eigen::Vector3d(
           3.191498553400951e-05, -1.543690630784238, 0.010816871687194661)},
      {(Eigen::Matrix3d() << 11.562412321554103,
        -0.095747155820775143,
        -0.052244072712456027,
        -0.095747155820775143,
        11.606083447600202,
        0.070321708383667886,
        -0.052244072712456027,
        0.070321708383667886,
        11.473863068530736)
           .finished(),
       Eigen::Vector3d(
           0.0020493450982079775, -0.46587625070486771, -0.15949090030121615)},
  };

  constexpr double kCoefficient = 0.8;
  for (std::size_t index = 0; index < cases.size(); ++index) {
    dart::math::detail::ExactCoulombLocalConeQuadraticFactorization
        factorization;
    ASSERT_TRUE(
        dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
            cases[index].hessian, kCoefficient, factorization))
        << "case " << index;

    Eigen::Vector3d reaction;
    ASSERT_TRUE(
        dart::math::detail::
            solveExactCoulombLocalConeQuadraticProjectedGradientFallback(
                factorization, cases[index].linearTerm, reaction))
        << "case " << index;
    expectLocalConeKkt(
        cases[index].hessian,
        cases[index].linearTerm,
        kCoefficient,
        reaction,
        1e-10);
    EXPECT_LE(
        computeLocalQuadraticObjective(
            cases[index].hessian, cases[index].linearTerm, reaction),
        0.0)
        << "case " << index;

    Eigen::Vector3d dispatchedReaction;
    ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
        factorization, cases[index].linearTerm, dispatchedReaction))
        << "case " << index;
    expectLocalConeKkt(
        cases[index].hessian,
        cases[index].linearTerm,
        kCoefficient,
        dispatchedReaction,
        1e-10);
    EXPECT_LE(
        computeLocalQuadraticObjective(
            cases[index].hessian, cases[index].linearTerm, dispatchedReaction),
        0.0)
        << "case " << index;
  }
}

TEST(
    ExactCoulombLocalConeQuadratic,
    CertifiesCapturedLargeReactionBoundarySystems)
{
  struct RegressionCase
  {
    Eigen::Matrix3d hessian;
    Eigen::Vector3d linearTerm;
    Eigen::Vector3d rejectedReaction;
  };

  const std::vector<RegressionCase> cases{
      {(Eigen::Matrix3d() << 7.1907589132276225e-6,
        1.2169993349876767e-6,
        1.2282536613576839e-6,
        1.2169993349876767e-6,
        5.0677346390504761e-6,
        -1.4164574374109603e-6,
        1.2282536613576839e-6,
        -1.4164574374109603e-6,
        5.073802380017959e-6)
           .finished(),
       Eigen::Vector3d(
           -1.3271099209248689, -0.85711660281582613, -0.015438848287437671),
       Eigen::Vector3d(
           162567.92604690389, 130054.34078155806, -3.8153627644154415)},
      {(Eigen::Matrix3d() << 7.1952997739510286e-6,
        -1.216999334996704e-6,
        -1.2282536613511058e-6,
        -1.216999334996704e-6,
        5.0722754997582079e-6,
        -1.4164574374124178e-6,
        -1.2282536613511058e-6,
        -1.4164574374124178e-6,
        5.0783432407557604e-6)
           .finished(),
       Eigen::Vector3d(
           -1.3268461317823594, 0.85699742654855504, 0.015452913817274896),
       Eigen::Vector3d(
           162441.17684002011, -129952.94146521356, -1.3296683748809879)},
      {(Eigen::Matrix3d() << 7.1907589132283509e-6,
        1.2169993349899628e-6,
        -1.2282536613556267e-6,
        1.2169993349899628e-6,
        5.0677346390464019e-6,
        1.4164574374113103e-6,
        -1.2282536613556267e-6,
        1.4164574374113103e-6,
        5.073802380022754e-6)
           .finished(),
       Eigen::Vector3d(
           -1.3245892794059606, -0.85536439019449562, 0.01540428071767159),
       Eigen::Vector3d(
           162251.13171999122, 129800.9052912031, 4.6916527293924446)},
      {(Eigen::Matrix3d() << 7.1952997739450671e-6,
        -1.2169993349824522e-6,
        1.2282536613635233e-6,
        -1.2169993349824522e-6,
        5.0722754997812116e-6,
        1.4164574374095139e-6,
        1.2282536613635233e-6,
        1.4164574374095139e-6,
        5.078343240726824e-6)
           .finished(),
       Eigen::Vector3d(
           -1.3249758616679168, 0.85570616727935145, -0.015415797451800538),
       Eigen::Vector3d(
           162206.85145511033, -129765.48115433479, -1.5910145466114045)},
  };

  constexpr double kCoefficient = 0.8;
  for (std::size_t index = 0; index < cases.size(); ++index) {
    dart::math::detail::ExactCoulombLocalConeQuadraticFactorization
        factorization;
    ASSERT_TRUE(
        dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
            cases[index].hessian, kCoefficient, factorization))
        << "case " << index;
    EXPECT_TRUE(dart::math::detail::
                    isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
                        factorization,
                        cases[index].linearTerm,
                        cases[index].rejectedReaction))
        << "case " << index;

    Eigen::Vector3d analyticReaction;
    ASSERT_TRUE(
        dart::math::detail::trySolveExactCoulombLocalConeQuadraticAnalytically(
            factorization, cases[index].linearTerm, analyticReaction))
        << "case " << index;
    EXPECT_TRUE(
        dart::math::detail::
            isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
                factorization, cases[index].linearTerm, analyticReaction))
        << "case " << index;

    Eigen::Vector3d fallbackReaction;
    ASSERT_TRUE(
        dart::math::detail::
            solveExactCoulombLocalConeQuadraticProjectedGradientFallback(
                factorization, cases[index].linearTerm, fallbackReaction))
        << "case " << index;
    EXPECT_TRUE(
        dart::math::detail::
            isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
                factorization, cases[index].linearTerm, fallbackReaction))
        << "case " << index;

    Eigen::Vector3d dispatchedReaction;
    ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
        factorization, cases[index].linearTerm, dispatchedReaction))
        << "case " << index;
    EXPECT_TRUE(
        dart::math::detail::
            isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
                factorization, cases[index].linearTerm, dispatchedReaction))
        << "case " << index;
  }
}

TEST(ExactCoulombLocalConeQuadratic, ChecksBoundaryNormalConeStationarity)
{
  constexpr double kCoefficient = 0.8;
  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;

  const Eigen::Matrix3d smallHessian = 1e-6 * Eigen::Matrix3d::Identity();
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          smallHessian, kCoefficient, factorization));
  const Eigen::Vector3d largeBoundaryReaction(1e6, 8e5, 0.0);
  const Eigen::Vector3d signature(-0.64, 1.0, 1.0);
  const Eigen::Vector3d nearlyNormalGradient
      = -1e-7 * signature.cwiseProduct(largeBoundaryReaction)
        + Eigen::Vector3d(8e-13, 0.0, 0.0);
  const Eigen::Vector3d nearlyNormalLinearTerm
      = nearlyNormalGradient - smallHessian * largeBoundaryReaction;
  EXPECT_FALSE(dart::math::detail::isExactCoulombLocalConeQuadraticKktPoint(
      factorization, nearlyNormalLinearTerm, largeBoundaryReaction));
  EXPECT_TRUE(
      dart::math::detail::
          isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
              factorization, nearlyNormalLinearTerm, largeBoundaryReaction));

  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          Eigen::Matrix3d::Identity(), kCoefficient, factorization));
  const Eigen::Vector3d boundaryReaction(1.0, 0.8, 0.0);
  const Eigen::Vector3d nonNormalLinearTerm(-0.1, -1.8, 0.1);
  EXPECT_FALSE(dart::math::detail::
                   isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
                       factorization, nonNormalLinearTerm, boundaryReaction));

  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          Eigen::Matrix3d::Identity(), 0.0, factorization));
  EXPECT_FALSE(dart::math::detail::
                   isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
                       factorization,
                       Eigen::Vector3d(-1.0, 0.0, 0.0),
                       Eigen::Vector3d(1.0, 0.0, 0.0)));
}

TEST(ExactCoulombLocalConeQuadratic, ExactlyMinimizesOverAFeasibleCoordinate)
{
  const Eigen::Matrix3d hessian = makeCrossCoupledLocalHessian();
  const Eigen::Vector3d linearTerm(-6.57, -4.328, -2.394);
  constexpr double kCoefficient = 0.6;
  const Eigen::Vector3d previous(2.0, 0.4, -0.3);

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          hessian, kCoefficient, factorization));
  Eigen::Vector3d reaction;
  ASSERT_TRUE(dart::math::detail::solveExactCoulombLocalConeQuadratic(
      factorization, linearTerm, reaction));

  EXPECT_LT(
      computeLocalQuadraticObjective(hessian, linearTerm, reaction),
      computeLocalQuadraticObjective(hessian, linearTerm, previous));
  expectLocalConeKkt(hessian, linearTerm, kCoefficient, reaction);
}

TEST(ExactCoulombLocalConeQuadratic, RejectsNonSpdOrAsymmetricHessian)
{
  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  Eigen::Matrix3d indefinite = Eigen::Matrix3d::Identity();
  indefinite(2, 2) = -1.0;
  EXPECT_FALSE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          indefinite, 0.5, factorization));

  Eigen::Matrix3d asymmetric = Eigen::Matrix3d::Identity();
  asymmetric(0, 1) = 1e-6;
  EXPECT_FALSE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          asymmetric, 0.5, factorization));
}

TEST(ExactCoulombFrozenConeSolver, SolvesSingleContactNormalAnalyticalCase)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d referenceReaction = Eigen::Vector3d::Zero();
  const Eigen::Vector3d frozenCoupling = Eigen::Vector3d::Zero();

  dart::math::detail::ExactCoulombFrozenConeOptions options;
  options.maxIterations = 100;
  options.tolerance = 1e-12;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeProjectedGradient(
          problem,
          referenceReaction,
          frozenCoupling,
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  EXPECT_NEAR(result.fixedPointError, 0.0, 1e-12);
  expectVectorNear(result.reaction, Eigen::Vector3d(0.5, 0.0, 0.0));
}

TEST(ExactCoulombFrozenConeSolver, ProjectsTangentialImpulseOntoCone)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(0.0, -2.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Zero();
  const Eigen::Vector3d referenceReaction = Eigen::Vector3d::Zero();
  const Eigen::Vector3d frozenCoupling = Eigen::Vector3d::Zero();

  dart::math::detail::ExactCoulombFrozenConeOptions options;
  options.maxIterations = 10;
  options.tolerance = 1e-12;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeProjectedGradient(
          problem,
          referenceReaction,
          frozenCoupling,
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(result.reaction, Eigen::Vector3d(0.8, 0.4, 0.0));
}

TEST(ExactCoulombFrozenConeSolver, SolvesSmallProductConeDiagonalCase)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity.resize(6);
  problem.freeVelocity << -1.0, 0.0, 0.0, -4.0, 0.0, 0.0;
  problem.coefficients = Eigen::Vector2d(0.5, 0.8);

  Eigen::MatrixXd delassus = Eigen::MatrixXd::Zero(6, 6);
  delassus.diagonal() << 1.0, 1.0, 1.0, 2.0, 2.0, 2.0;

  const Eigen::VectorXd referenceReaction = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd frozenCoupling = Eigen::VectorXd::Zero(6);

  dart::math::detail::ExactCoulombFrozenConeOptions options;
  options.maxIterations = 200;
  options.tolerance = 1e-12;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeProjectedGradient(
          problem,
          referenceReaction,
          frozenCoupling,
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  Eigen::VectorXd expected(6);
  expected << 0.5, 0.0, 0.0, 4.0 / 3.0, 0.0, 0.0;

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(result.reaction, expected);
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    SolvesSingleContactNormalAnalyticalCase)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d referenceReaction = Eigen::Vector3d::Zero();
  const Eigen::Vector3d frozenCoupling = Eigen::Vector3d::Zero();

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = 10;
  options.localIterations = 4;
  options.tolerance = 1e-12;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  EXPECT_NEAR(result.fixedPointError, 0.0, 1e-12);
  expectVectorNear(result.reaction, Eigen::Vector3d(0.5, 0.0, 0.0));
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    DefaultsToExactMetricAndKeepsInverseProjectionExplicit)
{
  const Eigen::Matrix3d localHessian = makeCrossCoupledLocalHessian();
  const Eigen::Matrix3d delassus = localHessian - Eigen::Matrix3d::Identity();
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-6.57, -4.328, -2.394), 0.6);

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = 1;
  options.runFixedSweeps = true;
  ASSERT_EQ(
      options.localSolver,
      dart::math::detail::ExactCoulombFrozenConeLocalSolver::
          ExactMetricProjection);
  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  auto inverseOptions = options;
  inverseOptions.localSolver = dart::math::detail::
      ExactCoulombFrozenConeLocalSolver::InverseEuclideanProjection;
  const auto inverseResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          1.0,
          makeDenseDelassusOperator(delassus),
          inverseOptions);

  ASSERT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::MaxIterations);
  ASSERT_EQ(
      inverseResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::MaxIterations);
  expectVectorNear(
      inverseResult.reaction,
      Eigen::Vector3d(
          1.5035607320294018, 0.7007049576384029, 0.5682100996156688),
      1e-11);
  expectVectorNear(result.reaction, Eigen::Vector3d(1.5, 0.72, 0.54), 1e-11);

  dart::math::detail::ExactCoulombLocalConeQuadraticFactorization factorization;
  ASSERT_TRUE(
      dart::math::detail::prepareExactCoulombLocalConeQuadraticFactorization(
          localHessian, 0.6, factorization));
  const Eigen::Vector3d linearTerm = problem.freeVelocity;
  EXPECT_TRUE(dart::math::detail::isExactCoulombLocalConeQuadraticKktPoint(
      factorization, linearTerm, result.reaction));
  EXPECT_FALSE(dart::math::detail::isExactCoulombLocalConeQuadraticKktPoint(
      factorization, linearTerm, inverseResult.reaction));
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver, CanRunTheFullFixedSweepBudget)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = 3;
  options.localIterations = 4;
  options.tolerance = 1.0;
  options.runFixedSweeps = true;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  EXPECT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::MaxIterations);
  EXPECT_EQ(result.iterations, options.maxSweeps);
  expectVectorNear(result.reaction, Eigen::Vector3d(0.5, 0.0, 0.0));
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    UsesExplicitInitialIterateWithoutChangingProximalCenter)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d referenceReaction = Eigen::Vector3d::Zero();
  Eigen::VectorXd initialReaction(3);
  initialReaction << 2.0, 0.5, 0.0;
  const Eigen::Vector3d frozenCoupling = Eigen::Vector3d::Zero();

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = 0;
  options.initialReaction = &initialReaction;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  EXPECT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::MaxIterations);
  EXPECT_EQ(result.iterations, 0);
  expectVectorNear(result.reaction, initialReaction);
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    ProjectsTangentialImpulseOntoCone)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(0.0, -2.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Zero();
  const Eigen::Vector3d referenceReaction = Eigen::Vector3d::Zero();
  const Eigen::Vector3d frozenCoupling = Eigen::Vector3d::Zero();

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = 10;
  options.localIterations = 4;
  options.tolerance = 1e-12;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(result.reaction, Eigen::Vector3d(0.8, 0.4, 0.0));
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    SolvesSmallProductConeDiagonalCase)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity.resize(6);
  problem.freeVelocity << -1.0, 0.0, 0.0, -4.0, 0.0, 0.0;
  problem.coefficients = Eigen::Vector2d(0.5, 0.8);

  Eigen::MatrixXd delassus = Eigen::MatrixXd::Zero(6, 6);
  delassus.diagonal() << 1.0, 1.0, 1.0, 2.0, 2.0, 2.0;

  const Eigen::VectorXd referenceReaction = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd frozenCoupling = Eigen::VectorXd::Zero(6);

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = 10;
  options.localIterations = 4;
  options.tolerance = 1e-12;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  Eigen::VectorXd expected(6);
  expected << 0.5, 0.0, 0.0, 4.0 / 3.0, 0.0, 0.0;

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(result.reaction, expected);
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    MatchesProjectedGradientOnCoupledProblem)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity.resize(6);
  problem.freeVelocity << -1.0, -0.2, 0.1, -0.6, 0.15, -0.05;
  problem.coefficients = Eigen::Vector2d(0.5, 0.8);

  Eigen::MatrixXd delassus = 2.0 * Eigen::MatrixXd::Identity(6, 6);
  delassus(0, 3) = 0.2;
  delassus(3, 0) = 0.2;
  delassus(1, 4) = -0.1;
  delassus(4, 1) = -0.1;
  delassus(2, 5) = 0.05;
  delassus(5, 2) = 0.05;

  Eigen::VectorXd referenceReaction(6);
  referenceReaction << 0.1, 0.0, 0.0, 0.2, 0.1, 0.0;
  Eigen::VectorXd frozenCoupling(6);
  frozenCoupling << 0.04, 0.0, 0.0, 0.03, 0.0, 0.0;
  const double stepSizeGamma = 0.4;

  dart::math::detail::ExactCoulombFrozenConeOptions projectedOptions;
  projectedOptions.maxIterations = 1000;
  projectedOptions.tolerance = 1e-13;

  const auto projectedResult
      = dart::math::detail::solveExactCoulombFrozenConeProjectedGradient(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          projectedOptions);

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions bgsOptions;
  bgsOptions.maxSweeps = 100;
  bgsOptions.localIterations = 20;
  bgsOptions.tolerance = 1e-12;

  const auto bgsResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          bgsOptions);
  auto exactOptions = bgsOptions;
  exactOptions.localSolver = dart::math::detail::
      ExactCoulombFrozenConeLocalSolver::ExactMetricProjection;
  const auto exactResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          exactOptions);
  auto localProjectedOptions = bgsOptions;
  localProjectedOptions.localSolver = dart::math::detail::
      ExactCoulombFrozenConeLocalSolver::ProjectedGradient;
  const auto localProjectedResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          localProjectedOptions);

  EXPECT_EQ(
      projectedResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  EXPECT_EQ(
      bgsResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  EXPECT_EQ(
      exactResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  EXPECT_EQ(
      localProjectedResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(bgsResult.reaction, projectedResult.reaction, 1e-8);
  expectVectorNear(exactResult.reaction, projectedResult.reaction, 1e-8);
  expectVectorNear(
      localProjectedResult.reaction, projectedResult.reaction, 1e-8);
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    ExactCoordinateSweepDoesNotIncreaseFrozenObjective)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity.resize(6);
  problem.freeVelocity << -1.2, -0.7, 0.3, -0.8, 0.4, -0.25;
  problem.coefficients = Eigen::Vector2d(0.6, 0.4);

  Eigen::MatrixXd generator = Eigen::MatrixXd::Identity(6, 6);
  generator(0, 1) = 0.3;
  generator(1, 2) = -0.2;
  generator(2, 3) = 0.15;
  generator(3, 4) = -0.25;
  generator(4, 5) = 0.1;
  generator(0, 5) = 0.12;
  const Eigen::MatrixXd delassus = generator.transpose() * generator
                                   + 0.2 * Eigen::MatrixXd::Identity(6, 6);
  const Eigen::VectorXd referenceReaction = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd frozenCoupling = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd initialReaction(6);
  initialReaction << 1.0, 0.2, -0.1, 0.8, -0.1, 0.1;
  constexpr double kStepSizeGamma = 0.7;

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = 1;
  options.runFixedSweeps = true;
  options.localSolver = dart::math::detail::ExactCoulombFrozenConeLocalSolver::
      ExactMetricProjection;
  options.initialReaction = &initialReaction;
  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          kStepSizeGamma,
          makeDenseDelassusOperator(delassus),
          options);

  ASSERT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::MaxIterations);
  const Eigen::MatrixXd hessian
      = delassus + (1.0 / kStepSizeGamma) * Eigen::MatrixXd::Identity(6, 6);
  const Eigen::VectorXd linearTerm = problem.freeVelocity + frozenCoupling
                                     - referenceReaction / kStepSizeGamma;
  const auto objective = [&](const Eigen::VectorXd& reaction) {
    return 0.5 * reaction.dot(hessian * reaction) + linearTerm.dot(reaction);
  };
  EXPECT_LE(objective(result.reaction), objective(initialReaction) + 1e-12);
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    BlockColumnOverloadWithCachedBlocksMatchesReference)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity.resize(6);
  problem.freeVelocity << -1.0, -0.2, 0.1, -0.6, 0.15, -0.05;
  problem.coefficients = Eigen::Vector2d(0.5, 0.8);

  Eigen::MatrixXd delassus = 2.0 * Eigen::MatrixXd::Identity(6, 6);
  delassus(0, 3) = 0.2;
  delassus(3, 0) = 0.2;
  delassus(1, 4) = -0.1;
  delassus(4, 1) = -0.1;
  delassus(2, 5) = 0.05;
  delassus(5, 2) = 0.05;

  Eigen::VectorXd referenceReaction(6);
  referenceReaction << 0.1, 0.0, 0.0, 0.2, 0.1, 0.0;
  Eigen::VectorXd frozenCoupling(6);
  frozenCoupling << 0.04, 0.0, 0.0, 0.03, 0.0, 0.0;
  const double stepSizeGamma = 0.4;

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = 100;
  options.localIterations = 20;
  options.tolerance = 1e-12;

  const auto referenceResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          options);

  std::vector<Eigen::Matrix3d> cachedBlocks{
      delassus.block<3, 3>(0, 0), delassus.block<3, 3>(3, 3)};
  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelWorkspace workspace;
  auto cachedOptions = options;
  cachedOptions.cachedDiagonalBlocks = &cachedBlocks;
  cachedOptions.workspace = &workspace;

  const auto accumulateBlockColumns
      = [&delassus](
            Eigen::Index contact,
            const Eigen::Vector3d& delta,
            Eigen::Ref<Eigen::VectorXd> accumulator) {
          accumulator.noalias() += delassus.middleCols<3>(3 * contact) * delta;
        };

  auto uncachedIncrementalOptions = options;
  uncachedIncrementalOptions.cachedDiagonalBlocks = &cachedBlocks;
  const auto uncachedIncrementalResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          accumulateBlockColumns,
          uncachedIncrementalOptions);
  const auto incrementalResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          accumulateBlockColumns,
          cachedOptions);
  const auto repeatedResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          accumulateBlockColumns,
          cachedOptions);

  EXPECT_EQ(
      referenceResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  EXPECT_EQ(
      incrementalResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  EXPECT_EQ(
      uncachedIncrementalResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(
      incrementalResult.reaction, uncachedIncrementalResult.reaction, 1e-12);
  expectVectorNear(incrementalResult.reaction, referenceResult.reaction, 1e-10);
  EXPECT_EQ(
      repeatedResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(repeatedResult.reaction, referenceResult.reaction, 1e-10);
  EXPECT_EQ(workspace.getLocalSystemBuildCount(), 1u);
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    WorkspaceInvalidatesWhenLocalSystemKeyChanges)
{
  using Workspace
      = dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelWorkspace;

  Workspace workspace;
  std::vector<Eigen::Matrix3d> diagonalBlocks{Eigen::Matrix3d::Identity()};
  Eigen::VectorXd coefficients = Eigen::VectorXd::Constant(1, 0.5);
  constexpr auto kLocalSolver = dart::math::detail::
      ExactCoulombFrozenConeLocalSolver::ExactMetricProjection;

  ASSERT_TRUE(workspace.prepareLocalSystems(
      diagonalBlocks, coefficients, 0.4, kLocalSolver, 0.0));
  EXPECT_EQ(workspace.getLocalSystemBuildCount(), 1u);

  ASSERT_TRUE(workspace.prepareLocalSystems(
      diagonalBlocks, coefficients, 0.4, kLocalSolver, 0.0));
  EXPECT_EQ(workspace.getLocalSystemBuildCount(), 1u);

  ASSERT_TRUE(workspace.prepareLocalSystems(
      diagonalBlocks, coefficients, 0.2, kLocalSolver, 0.0));
  EXPECT_EQ(workspace.getLocalSystemBuildCount(), 2u);

  diagonalBlocks[0](0, 0) = 1.5;
  ASSERT_TRUE(workspace.prepareLocalSystems(
      diagonalBlocks, coefficients, 0.2, kLocalSolver, 0.0));
  EXPECT_EQ(workspace.getLocalSystemBuildCount(), 3u);

  coefficients[0] = 0.8;
  ASSERT_TRUE(workspace.prepareLocalSystems(
      diagonalBlocks, coefficients, 0.2, kLocalSolver, 0.0));
  EXPECT_EQ(workspace.getLocalSystemBuildCount(), 4u);

  diagonalBlocks.push_back(2.0 * Eigen::Matrix3d::Identity());
  coefficients.conservativeResize(2);
  coefficients[1] = 0.3;
  ASSERT_TRUE(workspace.prepareLocalSystems(
      diagonalBlocks, coefficients, 0.2, kLocalSolver, 0.0));
  EXPECT_EQ(workspace.getLocalSystemBuildCount(), 5u);
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    SlowChainCrossesDelassusRefreshBoundary)
{
  // Regression guard for the incremental Delassus tracking: a weakly
  // diagonally dominant chain-coupled product cone converges slowly enough
  // to cross the internal 16-sweep full-refresh boundary several times, so
  // drift between the tracked response and the true product would surface
  // as a mismatch against the projected-gradient reference.
  constexpr int kContacts = 24;
  const Eigen::Index dimension = 3 * kContacts;

  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity = Eigen::VectorXd::Zero(dimension);
  problem.coefficients = Eigen::VectorXd::Constant(kContacts, 0.6);
  for (int contact = 0; contact < kContacts; ++contact) {
    problem.freeVelocity[3 * contact] = -1.0 - 0.05 * contact;
    problem.freeVelocity[3 * contact + 1] = 0.15;
  }

  Eigen::MatrixXd delassus = Eigen::MatrixXd::Identity(dimension, dimension);
  for (int contact = 0; contact + 1 < kContacts; ++contact) {
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Index row = 3 * contact + axis;
      const Eigen::Index next = 3 * (contact + 1) + axis;
      delassus(row, next) = 0.48;
      delassus(next, row) = 0.48;
    }
  }

  const Eigen::VectorXd referenceReaction = Eigen::VectorXd::Zero(dimension);
  const Eigen::VectorXd frozenCoupling = Eigen::VectorXd::Zero(dimension);
  const double stepSizeGamma = 2.0;

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions bgsOptions;
  bgsOptions.maxSweeps = 400;
  bgsOptions.localIterations = 24;
  bgsOptions.tolerance = 1e-13;
  bgsOptions.localTolerance = 1e-14;

  const auto bgsResult
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          bgsOptions);

  ASSERT_EQ(
      bgsResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  // The whole point of this fixture: crossing the internal 16-sweep refresh
  // interval (measured 36 sweeps, i.e. two refreshes). The assertion keeps
  // headroom for harmless numeric refactors that shave a few sweeps.
  EXPECT_GT(bgsResult.iterations, 16);

  dart::math::detail::ExactCoulombFrozenConeOptions projectedOptions;
  projectedOptions.maxIterations = 200000;
  projectedOptions.tolerance = 1e-13;

  const auto projectedResult
      = dart::math::detail::solveExactCoulombFrozenConeProjectedGradient(
          problem,
          referenceReaction,
          frozenCoupling,
          stepSizeGamma,
          makeDenseDelassusOperator(delassus),
          projectedOptions);

  ASSERT_EQ(
      projectedResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(bgsResult.reaction, projectedResult.reaction, 1e-8);
}

TEST(
    ExactCoulombFrozenConeBlockGaussSeidelSolver,
    RejectsCachedDiagonalBlockCountMismatch)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity.resize(6);
  problem.freeVelocity << -1.0, 0.0, 0.0, -4.0, 0.0, 0.0;
  problem.coefficients = Eigen::Vector2d(0.5, 0.8);

  Eigen::MatrixXd delassus = Eigen::MatrixXd::Identity(6, 6);
  const Eigen::VectorXd referenceReaction = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd frozenCoupling = Eigen::VectorXd::Zero(6);

  std::vector<Eigen::Matrix3d> wrongSizeBlocks{Eigen::Matrix3d::Identity()};
  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.cachedDiagonalBlocks = &wrongSizeBlocks;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          referenceReaction,
          frozenCoupling,
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  EXPECT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::InvalidInput);
}

TEST(ExactCoulombFrozenConeBlockGaussSeidelSolver, RejectsInvalidOptions)
{
  const auto problem = makeOneContactProblem(Eigen::Vector3d::Zero(), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions options;
  options.maxSweeps = -1;

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
          problem,
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          1.0,
          makeDenseDelassusOperator(delassus),
          options);

  EXPECT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::InvalidInput);
}

TEST(ExactCoulombFrozenConeSolver, RejectsInvalidStepSizeGamma)
{
  const auto problem = makeOneContactProblem(Eigen::Vector3d::Zero(), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();

  const auto result
      = dart::math::detail::solveExactCoulombFrozenConeProjectedGradient(
          problem,
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          0.0,
          makeDenseDelassusOperator(delassus));

  EXPECT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::InvalidInput);
}

TEST(ExactCoulombFbfSolver, RejectsInvalidOptions)
{
  const auto problem = makeOneContactProblem(Eigen::Vector3d::Zero(), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();

  int innerCalls = 0;
  const auto innerSolver = [&innerCalls](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double,
                               Eigen::Ref<Eigen::VectorXd>) {
    ++innerCalls;
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.maxOuterIterations = -1;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      Eigen::Vector3d::Zero(),
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::InvalidInput);
  EXPECT_EQ(innerCalls, 0);
}

TEST(ExactCoulombFbfSolver, StopsBeforeInnerSolveWhenInitialResidualIsSmall)
{
  const auto problem = makeOneContactProblem(Eigen::Vector3d::Zero(), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  int innerCalls = 0;
  const auto innerSolver = [&innerCalls](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double,
                               Eigen::Ref<Eigen::VectorXd> output) {
    ++innerCalls;
    output.setZero();
    return true;
  };

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver);

  EXPECT_EQ(result.status, dart::math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_EQ(innerCalls, 0);
  EXPECT_TRUE(result.residualHistory.empty());
  expectVectorNear(result.reaction, Eigen::Vector3d::Zero());
}

TEST(ExactCoulombFbfSolver, ConvergesAfterOneAcceptedOuterIteration)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  int innerCalls = 0;
  const auto innerSolver = [&innerCalls](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double,
                               Eigen::Ref<Eigen::VectorXd> output) {
    ++innerCalls;
    output[0] = 1.0;
    output[1] = 0.0;
    output[2] = 0.0;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.tolerance = 1e-12;
  options.maxResidualHistorySamples = 8;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(result.status, dart::math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_EQ(result.iterations, 1);
  EXPECT_EQ(innerCalls, 1);
  ASSERT_EQ(result.residualHistory.size(), 2u);
  EXPECT_EQ(result.residualHistory[0].iteration, 0);
  EXPECT_EQ(result.residualHistory[1].iteration, 1);
  EXPECT_GT(result.residualHistory[0].residual.value, options.tolerance);
  EXPECT_NEAR(result.residualHistory[1].residual.value, 0.0, 1e-12);
  EXPECT_NEAR(result.residualHistory[1].stepSize, 1.0, 1e-12);
  EXPECT_NEAR(
      result.residualHistory[1].safeStepSize, result.safeStepSize, 1e-12);
  EXPECT_NEAR(result.stepSize, 1.0, 1e-12);
  EXPECT_NEAR(result.residual.value, 0.0, 1e-12);
  expectVectorNear(result.reaction, Eigen::Vector3d(1.0, 0.0, 0.0));
}

TEST(ExactCoulombFbfSolver, AppliesFbfCorrectionAndProjectsBackToCone)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  const auto innerSolver = [](const auto&,
                              const Eigen::Ref<const Eigen::VectorXd>&,
                              const Eigen::Ref<const Eigen::VectorXd>&,
                              double,
                              Eigen::Ref<Eigen::VectorXd> output) {
    output[0] = 2.0;
    output[1] = 1.0;
    output[2] = 0.0;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.tolerance = 0.0;
  options.maxOuterIterations = 1;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(result.iterations, 1);
  EXPECT_NEAR(result.couplingVariationRatio, 0.5 / std::sqrt(5.0), 1e-12);
  expectVectorNear(result.reaction, Eigen::Vector3d(1.6, 0.8, 0.0));
}

TEST(ExactCoulombFbfSolver, CanSkipProjectionAfterFbfCorrection)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  const auto innerSolver = [](const auto&,
                              const Eigen::Ref<const Eigen::VectorXd>&,
                              const Eigen::Ref<const Eigen::VectorXd>&,
                              double,
                              Eigen::Ref<Eigen::VectorXd> output) {
    output[0] = 2.0;
    output[1] = 1.0;
    output[2] = 0.0;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.projectAfterCorrection = false;
  options.tolerance = 0.0;
  options.maxOuterIterations = 1;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(result.iterations, 1);
  EXPECT_GT(result.reaction.tail<2>().norm(), 0.5 * result.reaction[0]);
  expectVectorNear(result.reaction, Eigen::Vector3d(1.5, 1.0, 0.0));
}

TEST(ExactCoulombFbfSolver, AppliesOuterRelaxationToAcceptedCorrection)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  const auto innerSolver = [](const auto&,
                              const Eigen::Ref<const Eigen::VectorXd>&,
                              const Eigen::Ref<const Eigen::VectorXd>&,
                              double,
                              Eigen::Ref<Eigen::VectorXd> output) {
    output[0] = 2.0;
    output[1] = 1.0;
    output[2] = 0.0;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.outerRelaxation = 0.5;
  options.tolerance = 0.0;
  options.maxOuterIterations = 1;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(result.iterations, 1);
  expectVectorNear(result.reaction, Eigen::Vector3d(0.8, 0.4, 0.0));
}

TEST(ExactCoulombFbfSolver, RetainsBestIterateOnMaxIterations)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.0);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  int innerCalls = 0;
  std::vector<Eigen::VectorXd> innerInitialReactions;
  const auto innerSolver = [&innerCalls, &innerInitialReactions](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double,
                               const Eigen::VectorXd& innerInitialReaction,
                               Eigen::Ref<Eigen::VectorXd> output) {
    innerInitialReactions.push_back(innerInitialReaction);
    output.setZero();
    output[0] = innerCalls == 0 ? 0.25 : 2.0;
    ++innerCalls;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.tolerance = 0.0;
  options.maxOuterIterations = 2;
  options.maxResidualHistorySamples = 3;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(result.iterations, 2);
  ASSERT_EQ(innerInitialReactions.size(), 2u);
  expectVectorNear(innerInitialReactions[0], Eigen::Vector3d::Zero());
  expectVectorNear(innerInitialReactions[1], Eigen::Vector3d(0.25, 0.0, 0.0));
  EXPECT_EQ(result.bestIteration, 1);
  EXPECT_LT(
      result.bestResidual.value, result.residualHistory.back().residual.value);
  expectVectorNear(result.reaction, Eigen::Vector3d(2.0, 0.0, 0.0));
  expectVectorNear(result.bestReaction, Eigen::Vector3d(0.25, 0.0, 0.0));
}

TEST(ExactCoulombFbfSolver, ShrinksStepUntilCouplingVariationIsAccepted)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  int innerCalls = 0;
  const auto innerSolver = [&innerCalls](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double,
                               Eigen::Ref<Eigen::VectorXd> output) {
    ++innerCalls;
    output[0] = 2.0;
    output[1] = 1.0;
    output[2] = 0.0;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.couplingVariationTolerance = 0.1;
  options.maxOuterIterations = 1;
  options.maxStepShrinkIterations = 5;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(result.shrinkIterations, 3);
  EXPECT_EQ(innerCalls, 4);
  EXPECT_NEAR(result.stepSize, std::pow(0.7, 3), 1e-12);
  EXPECT_LE(result.couplingVariationRatio, options.couplingVariationTolerance);
}

TEST(ExactCoulombFbfSolver, CanRunAnUncappedFixedGammaExperiment)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  std::vector<double> attemptedStepSizes;
  const auto innerSolver = [&attemptedStepSizes](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double stepSizeGamma,
                               Eigen::Ref<Eigen::VectorXd> output) {
    attemptedStepSizes.push_back(stepSizeGamma);
    output << 2.0, 1.0, 0.0;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 10.0;
  options.capInitialStepSizeAtSafeBound = false;
  options.enableAdaptiveStepSize = false;
  options.couplingVariationTolerance = 1e-12;
  options.maxOuterIterations = 2;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  ASSERT_EQ(attemptedStepSizes.size(), 2u);
  EXPECT_EQ(result.shrinkIterations, 0);
  EXPECT_GT(result.couplingVariationRatio, options.couplingVariationTolerance);
  EXPECT_NEAR(result.stepSize, options.initialStepSize, 1e-12);
  for (const double stepSize : attemptedStepSizes)
    EXPECT_NEAR(stepSize, options.initialStepSize, 1e-12);
}

TEST(ExactCoulombFbfSolver, DoesNotShrinkPastFinalPermittedTrial)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  std::vector<double> attemptedStepSizes;
  const auto innerSolver = [&attemptedStepSizes](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double stepSizeGamma,
                               Eigen::Ref<Eigen::VectorXd> output) {
    attemptedStepSizes.push_back(stepSizeGamma);
    output << 2.0, 1.0, 0.0;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.couplingVariationTolerance = std::numeric_limits<double>::min();
  options.maxOuterIterations = 1;
  options.maxStepShrinkIterations = 2;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFbfStatus::StepSizeUnderflow);
  ASSERT_EQ(attemptedStepSizes.size(), 3u);
  EXPECT_EQ(result.shrinkIterations, 2);
  EXPECT_NEAR(attemptedStepSizes[0], 1.0, 1e-12);
  EXPECT_NEAR(attemptedStepSizes[1], 0.7, 1e-12);
  EXPECT_NEAR(attemptedStepSizes[2], 0.49, 1e-12);
  EXPECT_NEAR(result.stepSize, attemptedStepSizes.back(), 1e-12);
}

TEST(ExactCoulombFbfSolver, KeepsShrunkStepForRemainderOfSolve)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  std::vector<double> attemptedStepSizes;
  const auto innerSolver = [&attemptedStepSizes](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double stepSizeGamma,
                               Eigen::Ref<Eigen::VectorXd> output) {
    attemptedStepSizes.push_back(stepSizeGamma);
    output[0] = 2.0;
    output[1] = 1.0;
    output[2] = 0.0;
    return true;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.couplingVariationTolerance = 0.1;
  options.maxOuterIterations = 2;
  options.maxStepShrinkIterations = 5;
  options.tolerance = 0.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status, dart::math::detail::ExactCoulombFbfStatus::MaxIterations);

  int baseStepAttempts = 0;
  for (const double stepSize : attemptedStepSizes) {
    if (std::abs(stepSize - options.initialStepSize) <= 1e-12)
      ++baseStepAttempts;
  }

  EXPECT_EQ(baseStepAttempts, 1);
  ASSERT_GE(attemptedStepSizes.size(), 5u);
  EXPECT_NEAR(attemptedStepSizes.back(), std::pow(0.7, 3), 1e-12);
}

TEST(ExactCoulombFbfSolver, RunsWithProjectedGradientInnerSolver)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto applyDelassus = makeDenseDelassusOperator(delassus);
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  dart::math::detail::ExactCoulombFrozenConeOptions innerOptions;
  innerOptions.maxIterations = 100;
  innerOptions.tolerance = 1e-12;

  const auto innerSolver =
      [&applyDelassus, &innerOptions](
          const auto& innerProblem,
          const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
          const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
          double stepSizeGamma,
          Eigen::Ref<Eigen::VectorXd> output) {
        const auto innerResult
            = dart::math::detail::solveExactCoulombFrozenConeProjectedGradient(
                innerProblem,
                referenceReaction,
                frozenCoupling,
                stepSizeGamma,
                applyDelassus,
                innerOptions);
        output = innerResult.reaction;
        return innerResult.status
               == dart::math::detail::ExactCoulombFrozenConeStatus::Success;
      };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem, initialReaction, applyDelassus, innerSolver, options);

  EXPECT_EQ(result.status, dart::math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_LT(result.residual.value, options.tolerance);
  EXPECT_NEAR(result.reaction[0], 1.0, 1e-9);
  EXPECT_NEAR(result.reaction[1], 0.0, 1e-12);
  EXPECT_NEAR(result.reaction[2], 0.0, 1e-12);
}

TEST(ExactCoulombFbfSolver, RunsWithBlockGaussSeidelInnerSolver)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto applyDelassus = makeDenseDelassusOperator(delassus);
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions
      innerOptions;
  innerOptions.maxSweeps = 20;
  innerOptions.localIterations = 4;
  innerOptions.tolerance = 1e-12;

  const auto innerSolver
      = [&applyDelassus, &innerOptions](
            const auto& innerProblem,
            const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
            const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
            double stepSizeGamma,
            Eigen::Ref<Eigen::VectorXd> output) {
          const auto innerResult
              = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
                  innerProblem,
                  referenceReaction,
                  frozenCoupling,
                  stepSizeGamma,
                  applyDelassus,
                  innerOptions);
          output = innerResult.reaction;
          return innerResult.status
                 == dart::math::detail::ExactCoulombFrozenConeStatus::Success;
        };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem, initialReaction, applyDelassus, innerSolver, options);

  EXPECT_EQ(result.status, dart::math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_LT(result.residual.value, options.tolerance);
  EXPECT_NEAR(result.reaction[0], 1.0, 1e-9);
  EXPECT_NEAR(result.reaction[1], 0.0, 1e-12);
  EXPECT_NEAR(result.reaction[2], 0.0, 1e-12);
}

TEST(ExactCoulombFbfSolver, ReportsInnerSolverFailure)
{
  const auto problem
      = makeOneContactProblem(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.5);
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d initialReaction = Eigen::Vector3d::Zero();

  const auto innerSolver = [](const auto&,
                              const Eigen::Ref<const Eigen::VectorXd>&,
                              const Eigen::Ref<const Eigen::VectorXd>&,
                              double,
                              Eigen::Ref<Eigen::VectorXd>) {
    return false;
  };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;

  const auto result = dart::math::detail::solveExactCoulombFbf(
      problem,
      initialReaction,
      makeDenseDelassusOperator(delassus),
      innerSolver,
      options);

  EXPECT_EQ(
      result.status,
      dart::math::detail::ExactCoulombFbfStatus::InnerSolverFailed);
  EXPECT_EQ(result.iterations, 0);
}
