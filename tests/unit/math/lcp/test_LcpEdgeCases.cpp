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
#include <dart/math/lcp/pivoting/lemke_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <gtest/gtest.h>

#include <limits>

using namespace dart::math;

namespace {

LcpProblem MakeStandard2dProblem()
{
  Eigen::Matrix2d A;
  A << 4.0, 1.0, 1.0, 3.0;

  const Eigen::Vector2d target(0.5, 0.25);
  const Eigen::Vector2d b = A * target;

  return LcpProblem(
      A,
      b,
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector2i::Constant(-1));
}

LcpProblem MakeEmptyProblem()
{
  return LcpProblem(
      Eigen::MatrixXd(0, 0),
      Eigen::VectorXd(0),
      Eigen::VectorXd(0),
      Eigen::VectorXd(0),
      Eigen::VectorXi(0));
}

} // namespace

//==============================================================================
TEST(LcpEdgeCases, EmptyProblemSucceedsAcrossSolvers)
{
  const auto problem = MakeEmptyProblem();

  Eigen::VectorXd x = Eigen::VectorXd::Ones(3);

  DantzigSolver dantzig;
  auto result = dantzig.solve(problem, x, dantzig.getDefaultOptions());
  EXPECT_TRUE(result.succeeded());
  EXPECT_EQ(x.size(), 0);

  PgsSolver pgs;
  x = Eigen::VectorXd::Ones(5);
  result = pgs.solve(problem, x, pgs.getDefaultOptions());
  EXPECT_TRUE(result.succeeded());
  EXPECT_EQ(x.size(), 0);

  LemkeSolver lemke;
  x = Eigen::VectorXd::Ones(7);
  result = lemke.solve(problem, x, lemke.getDefaultOptions());
  EXPECT_TRUE(result.succeeded());
  EXPECT_EQ(x.size(), 0);
}

//==============================================================================
TEST(LcpEdgeCases, SolversResizeXWhenWarmStartingWrongSize)
{
  const auto problem = MakeStandard2dProblem();

  DantzigSolver dantzig;
  LcpOptions dantzigOptions = dantzig.getDefaultOptions();
  dantzigOptions.warmStart = true;

  Eigen::VectorXd x;
  auto result = dantzig.solve(problem, x, dantzigOptions);
  EXPECT_TRUE(result.succeeded()) << result.message;
  EXPECT_EQ(x.size(), 2);

  PgsSolver pgs;
  LcpOptions pgsOptions = pgs.getDefaultOptions();
  pgsOptions.maxIterations = 1000;
  pgsOptions.warmStart = true;
  pgsOptions.complementarityTolerance = 1e-3;

  x = Eigen::VectorXd(); // wrong size on purpose
  result = pgs.solve(problem, x, pgsOptions);
  EXPECT_TRUE(result.succeeded()) << result.message;
  EXPECT_EQ(x.size(), 2);
  EXPECT_FALSE(x.hasNaN());
}

//==============================================================================
TEST(LcpEdgeCases, WarmStartIgnoresNaN)
{
  const auto problem = MakeStandard2dProblem();

  Eigen::VectorXd x
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::quiet_NaN());

  DantzigSolver dantzig;
  LcpOptions dantzigOptions = dantzig.getDefaultOptions();
  dantzigOptions.warmStart = true;
  auto result = dantzig.solve(problem, x, dantzigOptions);
  EXPECT_TRUE(result.succeeded()) << result.message;
  EXPECT_TRUE(x.allFinite());

  PgsSolver pgs;
  LcpOptions pgsOptions = pgs.getDefaultOptions();
  pgsOptions.maxIterations = 1000;
  pgsOptions.warmStart = true;
  pgsOptions.complementarityTolerance = 1e-3;
  result = pgs.solve(problem, x, pgsOptions);
  EXPECT_TRUE(result.succeeded()) << result.message;
  EXPECT_TRUE(x.allFinite());
}

//==============================================================================
TEST(LcpEdgeCases, InvalidBoundsAreRejected)
{
  Eigen::Matrix2d A;
  A << 4.0, 1.0, 1.0, 3.0;
  const Eigen::Vector2d b = Eigen::Vector2d::Zero();

  Eigen::Vector2d lo;
  lo << 0.0, 1.0;
  Eigen::Vector2d hi;
  hi << std::numeric_limits<double>::infinity(), 0.0;

  LcpProblem problem(A, b, lo, hi, Eigen::Vector2i::Constant(-1));

  Eigen::VectorXd x;

  DantzigSolver dantzig;
  auto result = dantzig.solve(problem, x, dantzig.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);

  PgsSolver pgs;
  result = pgs.solve(problem, x, pgs.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(LcpEdgeCases, FindexRequiresFiniteMu)
{
  Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  Eigen::Vector2d b = Eigen::Vector2d::Zero();

  Eigen::Vector2d lo;
  lo << 0.0, -1.0;
  Eigen::Vector2d hi;
  hi << std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity();
  Eigen::Vector2i findex;
  findex << -1, 0;

  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x;

  DantzigSolver dantzig;
  auto result = dantzig.solve(problem, x, dantzig.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);

  PgsSolver pgs;
  result = pgs.solve(problem, x, pgs.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(LcpEdgeCases, FindexSelfReferenceIsRejected)
{
  Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  Eigen::Vector2d b = Eigen::Vector2d::Zero();

  Eigen::Vector2d lo;
  lo << 0.0, -1.0;
  Eigen::Vector2d hi;
  hi << std::numeric_limits<double>::infinity(), 0.5;
  Eigen::Vector2i findex;
  findex << -1, 1;

  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x;

  DantzigSolver dantzig;
  auto result = dantzig.solve(problem, x, dantzig.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);

  PgsSolver pgs;
  result = pgs.solve(problem, x, pgs.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}
