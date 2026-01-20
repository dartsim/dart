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

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart::math;

//==============================================================================
TEST(PgsSolver, SolvesStandardPositiveDefiniteLcp)
{
  Eigen::Matrix2d A;
  A << 4.0, 1.0, 1.0, 3.0;

  const Eigen::Vector2d target(0.5, 0.25);
  const Eigen::Vector2d b = A * target;

  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 1000;
  options.warmStart = false;
  options.complementarityTolerance = 1e-3;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpProblem problem(
      A,
      b,
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector2i::Constant(-1));
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(result.succeeded());
  EXPECT_FALSE(x.hasNaN());
  EXPECT_NEAR(x[0], target[0], 1e-4);
  EXPECT_NEAR(x[1], target[1], 1e-4);
}

//==============================================================================
TEST(PgsSolver, SolvesBoxedProblemWithFrictionIndex)
{
  Eigen::Matrix3d A;
  A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;

  const Eigen::Vector3d target(1.0, 0.2, -0.1);
  const Eigen::Vector3d b = A * target;

  Eigen::Vector3d lo = Eigen::Vector3d::Zero();
  Eigen::Vector3d hi;
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5;
  Eigen::Vector3i findex;
  findex << -1, 0, 0;

  PgsSolver pgs;
  LcpOptions pgsOptions = pgs.getDefaultOptions();
  pgsOptions.maxIterations = 5000;
  pgsOptions.warmStart = false;
  pgsOptions.complementarityTolerance = 1e-3;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpProblem problem(A, b, lo, hi, findex);
  const auto pgsResult = pgs.solve(problem, x, pgsOptions);
  EXPECT_TRUE(pgsResult.succeeded());
  EXPECT_FALSE(x.hasNaN());

  // Use the pivoting Dantzig solver as a reference for the boxed problem.
  DantzigSolver reference;
  Eigen::VectorXd referenceX = Eigen::VectorXd::Zero(3);
  LcpOptions refOptions;
  refOptions.warmStart = false;
  const auto refResult = reference.solve(problem, referenceX, refOptions);
  ASSERT_TRUE(refResult.succeeded());

  EXPECT_NEAR(x[0], referenceX[0], 1e-3);
  EXPECT_NEAR(x[1], referenceX[1], 1e-3);
  EXPECT_NEAR(x[2], referenceX[2], 1e-3);

  const double mu = hi[1];
  EXPECT_LE(std::abs(x[1]), mu * x[0] + 1e-8);
  EXPECT_LE(std::abs(x[2]), mu * x[0] + 1e-8);
}

//==============================================================================
TEST(PgsSolver, ReportsInvalidProblemForDimensionMismatch)
{
  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 5;
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpProblem problem(
      Eigen::MatrixXd::Identity(2, 2),
      Eigen::VectorXd::Zero(3),
      Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity()),
      Eigen::VectorXi::Constant(2, -1));

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(PgsSolver, ReportsInvalidProblemForOutOfRangeFindex)
{
  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 5;
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpProblem problem(
      Eigen::MatrixXd::Identity(2, 2),
      Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity()),
      (Eigen::VectorXi(2) << 2, -1).finished());

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(PgsSolver, RelaxationAffectsSinglePassUpdate)
{
  Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  Eigen::Vector2d b;
  b << 1.0, 2.0;

  LcpProblem problem(
      A,
      b,
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector2i::Constant(-1));

  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 1;
  options.warmStart = false;
  options.validateSolution = false;
  options.complementarityTolerance = 1e-6;

  Eigen::VectorXd x1 = Eigen::VectorXd::Zero(2);
  options.relaxation = 1.0;
  solver.solve(problem, x1, options);

  Eigen::VectorXd xHalf = Eigen::VectorXd::Zero(2);
  options.relaxation = 0.5;
  solver.solve(problem, xHalf, options);

  EXPECT_NEAR(x1[0], 1.0, 1e-12);
  EXPECT_NEAR(x1[1], 2.0, 1e-12);
  EXPECT_NEAR(xHalf[0], 0.5, 1e-12);
  EXPECT_NEAR(xHalf[1], 1.0, 1e-12);
}
