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

  EXPECT_EQ(
      projectedResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  EXPECT_EQ(
      bgsResult.status,
      dart::math::detail::ExactCoulombFrozenConeStatus::Success);
  expectVectorNear(bgsResult.reaction, projectedResult.reaction, 1e-8);
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
  auto cachedOptions = options;
  cachedOptions.cachedDiagonalBlocks = &cachedBlocks;

  const auto accumulateBlockColumns
      = [&delassus](
            Eigen::Index contact,
            const Eigen::Vector3d& delta,
            Eigen::Ref<Eigen::VectorXd> accumulator) {
          accumulator.noalias() += delassus.middleCols<3>(3 * contact) * delta;
        };

  const auto incrementalResult
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
  expectVectorNear(incrementalResult.reaction, referenceResult.reaction, 1e-10);
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
  const auto innerSolver = [&innerCalls](
                               const auto&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               const Eigen::Ref<const Eigen::VectorXd>&,
                               double,
                               Eigen::Ref<Eigen::VectorXd> output) {
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

TEST(ExactCoulombFbfSolver, RecoversBaseStepAfterAcceptedShrink)
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

  EXPECT_GE(baseStepAttempts, 2);
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
