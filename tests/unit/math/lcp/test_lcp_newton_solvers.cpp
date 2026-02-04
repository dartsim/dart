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

#include <dart/math/lcp/lcp_types.hpp>
#include <dart/math/lcp/newton/fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/newton/minimum_map_newton_solver.hpp>
#include <dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/other/interior_point_solver.hpp>
#include <dart/math/lcp/pivoting/lemke_solver.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart::math;

namespace {

LcpProblem makeDiagonalProblem(int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 2.0;
  Eigen::VectorXd target = Eigen::VectorXd::Constant(n, 0.5);
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeSpdProblem()
{
  Eigen::MatrixXd A(3, 3);
  A << 4.0, 0.8, 0.2, 0.8, 3.5, 0.4, 0.2, 0.4, 2.5;
  Eigen::VectorXd target(3);
  target << 0.3, 0.2, 0.4;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(3, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeBoxedProblem()
{
  const int n = 3;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 2.0;
  Eigen::VectorXd target = Eigen::VectorXd::Constant(n, 0.5);
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(n, 0.6);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeFrictionProblem()
{
  Eigen::MatrixXd A(4, 4);
  A << 3.2, 0.4, 0.1, 0.0, 0.4, 2.8, 0.3, 0.1, 0.1, 0.3, 2.5, 0.2, 0.0, 0.1,
      0.2, 2.2;

  Eigen::VectorXd target(4);
  target << 1.0, 0.2, -0.1, 0.05;
  Eigen::VectorXd b = A * target;

  Eigen::VectorXd lo(4);
  lo << 0.0, -0.4, -0.4, -0.4;
  Eigen::VectorXd hi(4);
  hi << std::numeric_limits<double>::infinity(), 0.4, 0.4, 0.4;
  Eigen::VectorXi findex(4);
  findex << -1, 0, 0, 0;

  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeSizeOneProblem()
{
  Eigen::MatrixXd A(1, 1);
  A << 3.0;
  Eigen::VectorXd target(1);
  target << 0.5;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(1, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeIllConditionedProblem()
{
  Eigen::MatrixXd A(3, 3);
  A << 1.0, 0.999, 0.998, 0.999, 0.998, 0.997, 0.998, 0.997, 0.996;
  Eigen::VectorXd target(3);
  target << 0.2, 0.15, 0.1;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(3, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeInfeasibleProblem()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

void expectDiagonalSolution(const Eigen::VectorXd& x, double expected)
{
  EXPECT_TRUE(x.array().isFinite().all());
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(x[i], expected, 1e-6);
  }
}

} // namespace

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, NameAndCategory)
{
  FischerBurmeisterNewtonSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, SolveDiagonalProblem)
{
  FischerBurmeisterNewtonSolver solver;
  auto problem = makeDiagonalProblem(4);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 100;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, SolveSpdProblem)
{
  FischerBurmeisterNewtonSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, SolveBoxedProblem)
{
  FischerBurmeisterNewtonSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < x.size(); ++i) {
    EXPECT_GE(x[i], -1e-6);
    EXPECT_LE(x[i], 0.6 + 1e-6);
  }
}

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, SolveFrictionProblem)
{
  FischerBurmeisterNewtonSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  const double mu = 0.4;
  EXPECT_GE(x[0], -1e-6);
  for (int i = 1; i < 4; ++i) {
    EXPECT_LE(std::abs(x[i]), mu * std::abs(x[0]) + 1e-6);
  }
}

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, SolveSizeOneProblem)
{
  FischerBurmeisterNewtonSolver solver;
  auto problem = makeSizeOneProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, SolveIllConditionedProblem)
{
  FischerBurmeisterNewtonSolver solver;
  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, ConvergenceFailureWithLowIterations)
{
  FischerBurmeisterNewtonSolver solver;
  auto problem = makeInfeasibleProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 1;
  options.absoluteTolerance = 1e-14;
  options.relativeTolerance = 1e-14;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::Success);
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, NameAndCategory)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, SolveDiagonalProblem)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  auto problem = makeDiagonalProblem(4);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 100;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, SolveSpdProblem)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, SolveBoxedProblem)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < x.size(); ++i) {
    EXPECT_GE(x[i], -1e-6);
    EXPECT_LE(x[i], 0.6 + 1e-6);
  }
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, SolveFrictionProblem)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  const double mu = 0.4;
  EXPECT_GE(x[0], -1e-6);
  for (int i = 1; i < 4; ++i) {
    EXPECT_LE(std::abs(x[i]), mu * std::abs(x[0]) + 1e-6);
  }
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, SolveSizeOneProblem)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  auto problem = makeSizeOneProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, SolveIllConditionedProblem)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(
    PenalizedFischerBurmeisterNewtonSolver, ConvergenceFailureWithLowIterations)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  auto problem = makeInfeasibleProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 1;
  options.absoluteTolerance = 1e-14;
  options.relativeTolerance = 1e-14;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::Success);
}

//==============================================================================
TEST(MinimumMapNewtonSolver, NameAndCategory)
{
  MinimumMapNewtonSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(MinimumMapNewtonSolver, SolveDiagonalProblem)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeDiagonalProblem(4);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 100;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  expectDiagonalSolution(x, 0.5);
}

//==============================================================================
TEST(MinimumMapNewtonSolver, SolveSpdProblem)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  const Eigen::VectorXd expected = problem.A.ldlt().solve(problem.b);
  for (int i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(x[i], expected[i], 1e-5);
  }
}

//==============================================================================
TEST(MinimumMapNewtonSolver, SolveBoxedProblem)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < x.size(); ++i) {
    EXPECT_GE(x[i], -1e-6);
    EXPECT_LE(x[i], 0.6 + 1e-6);
  }
}

//==============================================================================
TEST(MinimumMapNewtonSolver, SolveFrictionProblem)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  const double mu = 0.4;
  EXPECT_GE(x[0], -1e-6);
  for (int i = 1; i < 4; ++i) {
    EXPECT_LE(std::abs(x[i]), mu * std::abs(x[0]) + 1e-6);
  }
}

//==============================================================================
TEST(MinimumMapNewtonSolver, SolveSizeOneProblem)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeSizeOneProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_NEAR(x[0], 0.5, 1e-6);
}

//==============================================================================
TEST(MinimumMapNewtonSolver, SolveIllConditionedProblem)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(MinimumMapNewtonSolver, ConvergenceFailureWithLowIterations)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeInfeasibleProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 1;
  options.absoluteTolerance = 1e-14;
  options.relativeTolerance = 1e-14;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::Success);
}

//==============================================================================
TEST(MinimumMapNewtonSolver, LineSearchFailureWithZeroSteps)
{
  MinimumMapNewtonSolver solver;
  MinimumMapNewtonSolver::Parameters params;
  params.maxLineSearchSteps = 0;
  solver.setParameters(params);

  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 2;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Failed);
  EXPECT_FALSE(result.message.empty());
}

//==============================================================================
TEST(FischerBurmeisterNewtonSolver, LineSearchFailureWithZeroSteps)
{
  FischerBurmeisterNewtonSolver solver;
  FischerBurmeisterNewtonSolver::Parameters params;
  params.maxLineSearchSteps = 0;
  solver.setParameters(params);

  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 2;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Failed);
  EXPECT_FALSE(result.message.empty());
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, LineSearchFailureWithZeroSteps)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  PenalizedFischerBurmeisterNewtonSolver::Parameters params;
  params.maxLineSearchSteps = 0;
  solver.setParameters(params);

  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 2;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Failed);
  EXPECT_FALSE(result.message.empty());
}

//==============================================================================
TEST(PenalizedFischerBurmeisterNewtonSolver, RejectsInvalidLambda)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  PenalizedFischerBurmeisterNewtonSolver::Parameters params;
  params.lambda = 0.0;
  solver.setParameters(params);

  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 10;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//==============================================================================
TEST(InteriorPointSolver, NameAndCategory)
{
  InteriorPointSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(InteriorPointSolver, SolveDiagonalProblem)
{
  InteriorPointSolver solver;
  auto problem = makeDiagonalProblem(4);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 100;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(x[i], 0.5, 1e-4);
  }
}

//==============================================================================
TEST(InteriorPointSolver, SolveSpdProblem)
{
  InteriorPointSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  const Eigen::VectorXd expected = problem.A.ldlt().solve(problem.b);
  for (int i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(x[i], expected[i], 1e-4);
  }
}

//==============================================================================
TEST(InteriorPointSolver, SolveBoxedProblem)
{
  InteriorPointSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < x.size(); ++i) {
    EXPECT_GE(x[i], -1e-6);
    EXPECT_LE(x[i], 0.6 + 1e-6);
  }
}

//==============================================================================
TEST(InteriorPointSolver, SolveFrictionProblem)
{
  InteriorPointSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  const double mu = 0.4;
  EXPECT_GE(x[0], -1e-6);
  for (int i = 1; i < 4; ++i) {
    EXPECT_LE(std::abs(x[i]), mu * std::abs(x[0]) + 1e-6);
  }
}

//==============================================================================
TEST(InteriorPointSolver, SolveSizeOneProblem)
{
  InteriorPointSolver solver;
  auto problem = makeSizeOneProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_NEAR(x[0], 0.5, 1e-4);
}

//==============================================================================
TEST(InteriorPointSolver, SolveIllConditionedProblem)
{
  InteriorPointSolver solver;
  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(InteriorPointSolver, ConvergenceFailureWithLowIterations)
{
  InteriorPointSolver solver;
  auto problem = makeInfeasibleProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 1;
  options.absoluteTolerance = 1e-14;
  options.relativeTolerance = 1e-14;
  options.complementarityTolerance = 1e-14;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::Success);
}

//==============================================================================
TEST(LemkeSolver, NameAndCategory)
{
  LemkeSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(LemkeSolver, SolveDiagonalProblem)
{
  LemkeSolver solver;
  auto problem = makeDiagonalProblem(4);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  expectDiagonalSolution(x, 0.5);
}

//==============================================================================
TEST(LemkeSolver, SolveSpdProblem)
{
  LemkeSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  const Eigen::VectorXd expected = problem.A.ldlt().solve(problem.b);
  for (int i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(x[i], expected[i], 1e-5);
  }
}

//==============================================================================
TEST(LemkeSolver, SolveBoxedProblem)
{
  LemkeSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < x.size(); ++i) {
    EXPECT_GE(x[i], -1e-6);
    EXPECT_LE(x[i], 0.6 + 1e-6);
  }
}

//==============================================================================
TEST(LemkeSolver, SolveFrictionProblem)
{
  LemkeSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
  const double mu = 0.4;
  EXPECT_GE(x[0], -1e-6);
  for (int i = 1; i < 4; ++i) {
    EXPECT_LE(std::abs(x[i]), mu * std::abs(x[0]) + 1e-6);
  }
}

//==============================================================================
TEST(LemkeSolver, SolveSizeOneProblem)
{
  LemkeSolver solver;
  auto problem = makeSizeOneProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_NEAR(x[0], 0.5, 1e-6);
}

//==============================================================================
TEST(LemkeSolver, SolveIllConditionedProblem)
{
  LemkeSolver solver;
  auto problem = makeIllConditionedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(LemkeSolver, ConvergenceFailureWithLowIterations)
{
  LemkeSolver solver;
  auto problem = makeInfeasibleProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 1;
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::Success);
}
