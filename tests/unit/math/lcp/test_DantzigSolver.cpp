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

#include <dart/math/lcp/lcp_validation.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <gtest/gtest.h>

#include <limits>

using namespace dart::math;

namespace {

void ExpectValidSolution(
    const LcpProblem& problem, const Eigen::VectorXd& x, double tol)
{
  const Eigen::VectorXd w = problem.A * x - problem.b;
  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  ASSERT_TRUE(
      detail::computeEffectiveBounds(
          problem.lo, problem.hi, problem.findex, x, loEff, hiEff, &message))
      << message;
  EXPECT_TRUE(detail::validateSolution(x, w, loEff, hiEff, tol, &message))
      << message;
}

} // namespace

//==============================================================================
TEST(DantzigSolver, SolvesStandardPositiveDefiniteLcp)
{
  Eigen::Matrix2d A;
  A << 4.0, 1.0, 1.0, 3.0;

  const Eigen::Vector2d target(0.5, 0.25);
  const Eigen::Vector2d b = A * target;

  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpProblem problem(
      A,
      b,
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector2i::Constant(-1));
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(result.succeeded());
  EXPECT_NEAR(x[0], target[0], 1e-8);
  EXPECT_NEAR(x[1], target[1], 1e-8);
  ExpectValidSolution(problem, x, 1e-6);
}

//==============================================================================
TEST(DantzigSolver, SolvesBoxedLcpWithActiveUpperBound)
{
  Eigen::Matrix2d A;
  A << 2.0, 0.5, 0.5, 1.5;

  Eigen::Vector2d lo;
  lo << -1.0, 0.0;
  Eigen::Vector2d hi;
  hi << 1.0, 0.2;

  Eigen::Vector2d target;
  target << 0.1, 0.2;
  Eigen::Vector2d w;
  w << 0.0, -0.3;
  const Eigen::Vector2d b = A * target - w;

  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpProblem problem(A, b, lo, hi, Eigen::Vector2i::Constant(-1));
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(result.succeeded());
  EXPECT_NEAR(x[0], target[0], 1e-6);
  EXPECT_NEAR(x[1], target[1], 1e-6);
  ExpectValidSolution(problem, x, 1e-6);
}

//==============================================================================
TEST(DantzigSolver, SolvesBoxedLcpWithFrictionIndex)
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

  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpProblem problem(A, b, lo, hi, findex);
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(result.succeeded());
  EXPECT_NEAR(x[0], target[0], 1e-6);
  EXPECT_NEAR(x[1], target[1], 1e-6);
  EXPECT_NEAR(x[2], target[2], 1e-6);
  ExpectValidSolution(problem, x, 1e-6);
}

//==============================================================================
TEST(DantzigSolver, ReportsInvalidProblemForOutOfRangeFindex)
{
  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
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
