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
#include <dart/math/lcp/other/shock_propagation_solver.hpp>
#include <dart/math/lcp/pivoting/baraff_solver.hpp>
#include <dart/math/lcp/projection/bgs_solver.hpp>
#include <dart/math/lcp/projection/nncg_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>

using namespace dart::math;

namespace {

LcpProblem makeSimpleDiagonalProblem(int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 2.0;
  Eigen::VectorXd b = -Eigen::VectorXd::Ones(n);
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
  A << 4.0, 1.0, 0.0, 1.0, 3.0, 0.5, 0.0, 0.5, 2.0;
  Eigen::VectorXd b(3);
  b << -1.0, -2.0, -1.0;
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
  Eigen::MatrixXd A(2, 2);
  A << 3.0, 1.0, 1.0, 3.0;
  Eigen::VectorXd b(2);
  b << -2.0, -2.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(2, 5.0);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

} // namespace

//==============================================================================
TEST(BaraffSolver, NameAndCategory)
{
  BaraffSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(BaraffSolver, SolveDiagonalProblem)
{
  BaraffSolver solver;
  auto problem = makeSimpleDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(x[i], -1e-6);
  }
}

//==============================================================================
TEST(BaraffSolver, SolveSpdProblem)
{
  BaraffSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(x[i], -1e-6);
  }
}

//==============================================================================
TEST(BaraffSolver, SolveSize1DoesNotCrash)
{
  BaraffSolver solver;
  auto problem = makeSimpleDiagonalProblem(1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);

  LcpOptions options;
  auto result = solver.solve(problem, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
}

//==============================================================================
TEST(NncgSolver, NameAndCategory)
{
  NncgSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(NncgSolver, DefaultParameters)
{
  NncgSolver solver;
  const auto& params = solver.getParameters();
  EXPECT_EQ(params.pgsIterations, 1);
  EXPECT_EQ(params.restartInterval, 10);
  EXPECT_DOUBLE_EQ(params.restartThreshold, 1.0);
}

//==============================================================================
TEST(NncgSolver, SetParameters)
{
  NncgSolver solver;
  NncgSolver::Parameters params;
  params.pgsIterations = 5;
  params.restartInterval = 20;
  params.restartThreshold = 0.5;
  solver.setParameters(params);

  const auto& retrieved = solver.getParameters();
  EXPECT_EQ(retrieved.pgsIterations, 5);
  EXPECT_EQ(retrieved.restartInterval, 20);
  EXPECT_DOUBLE_EQ(retrieved.restartThreshold, 0.5);
}

//==============================================================================
TEST(NncgSolver, SolveDiagonalProblem)
{
  NncgSolver solver;
  auto problem = makeSimpleDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(x[i], -1e-6);
  }
}

//==============================================================================
TEST(NncgSolver, SolveBoxedProblem)
{
  NncgSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < 2; ++i) {
    EXPECT_GE(x[i], -1e-6);
    EXPECT_LE(x[i], 5.0 + 1e-6);
  }
}

//==============================================================================
TEST(BgsSolver, NameAndCategory)
{
  BgsSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(BgsSolver, DefaultParameters)
{
  BgsSolver solver;
  const auto& params = solver.getParameters();
  EXPECT_TRUE(params.blockSizes.empty());
}

//==============================================================================
TEST(BgsSolver, SetParameters)
{
  BgsSolver solver;
  BgsSolver::Parameters params;
  params.blockSizes = {1, 2};
  solver.setParameters(params);

  const auto& retrieved = solver.getParameters();
  ASSERT_EQ(retrieved.blockSizes.size(), 2u);
  EXPECT_EQ(retrieved.blockSizes[0], 1);
  EXPECT_EQ(retrieved.blockSizes[1], 2);
}

//==============================================================================
TEST(BgsSolver, SolveDiagonalProblem)
{
  BgsSolver solver;
  auto problem = makeSimpleDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(x[i], -1e-6);
  }
}

//==============================================================================
TEST(BgsSolver, SolveWithBlockSizes)
{
  BgsSolver solver;
  BgsSolver::Parameters params;
  params.blockSizes = {1, 1, 1};
  solver.setParameters(params);

  auto problem = makeSimpleDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
}

//==============================================================================
TEST(BgsSolver, SolveBoxedProblem)
{
  BgsSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
}

//==============================================================================
TEST(ShockPropagationSolver, NameAndCategory)
{
  ShockPropagationSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(ShockPropagationSolver, DefaultParameters)
{
  ShockPropagationSolver solver;
  const auto& params = solver.getParameters();
  EXPECT_TRUE(params.blockSizes.empty());
  EXPECT_TRUE(params.layers.empty());
}

//==============================================================================
TEST(ShockPropagationSolver, SetParameters)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1, 1};
  params.layers = {{0}, {1}, {2}};
  solver.setParameters(params);

  const auto& retrieved = solver.getParameters();
  ASSERT_EQ(retrieved.blockSizes.size(), 3u);
  ASSERT_EQ(retrieved.layers.size(), 3u);
  EXPECT_EQ(retrieved.layers[0][0], 0);
  EXPECT_EQ(retrieved.layers[1][0], 1);
  EXPECT_EQ(retrieved.layers[2][0], 2);
}

//==============================================================================
TEST(ShockPropagationSolver, SolveDiagonalProblem)
{
  ShockPropagationSolver solver;
  auto problem = makeSimpleDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
}

//==============================================================================
TEST(ShockPropagationSolver, SolveWithLayers)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1, 1};
  params.layers = {{0, 1, 2}};
  solver.setParameters(params);

  auto problem = makeSimpleDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
}

//==============================================================================
TEST(ShockPropagationSolver, SolveWithMultipleLayers)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1, 1};
  params.layers = {{0}, {1}, {2}};
  solver.setParameters(params);

  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
}

//==============================================================================
TEST(LcpTypes, LcpResultDefaults)
{
  LcpResult result;
  EXPECT_EQ(result.status, LcpSolverStatus::NotSolved);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_FALSE(result.validated);
  EXPECT_FALSE(result.succeeded());
}

//==============================================================================
TEST(LcpTypes, LcpResultWithStatus)
{
  LcpResult result(LcpSolverStatus::Success);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(result.succeeded());

  LcpResult failed(LcpSolverStatus::Failed);
  EXPECT_FALSE(failed.succeeded());

  LcpResult maxIter(LcpSolverStatus::MaxIterations);
  EXPECT_FALSE(maxIter.succeeded());
}

//==============================================================================
TEST(LcpTypes, LcpOptionsDefaults)
{
  LcpOptions options;
  EXPECT_EQ(options.maxIterations, 0);
  EXPECT_GT(options.absoluteTolerance, 0.0);
  EXPECT_GT(options.relativeTolerance, 0.0);
  EXPECT_TRUE(options.validateSolution);
  EXPECT_DOUBLE_EQ(options.relaxation, 1.0);
  EXPECT_FALSE(options.warmStart);
}

//==============================================================================
TEST(LcpTypes, LcpOptionsFactories)
{
  auto relaxed = LcpOptions::withRelaxation(1.5, 50);
  EXPECT_DOUBLE_EQ(relaxed.relaxation, 1.5);
  EXPECT_EQ(relaxed.maxIterations, 50);

  auto accurate = LcpOptions::highAccuracy();
  EXPECT_LT(accurate.absoluteTolerance, 1e-8);
  EXPECT_GT(accurate.maxIterations, 100);

  auto fast = LcpOptions::realTime();
  EXPECT_LE(fast.maxIterations, 100);
  EXPECT_FALSE(fast.validateSolution);
}

//==============================================================================
TEST(LcpTypes, StatusToString)
{
  EXPECT_FALSE(toString(LcpSolverStatus::Success).empty());
  EXPECT_FALSE(toString(LcpSolverStatus::Failed).empty());
  EXPECT_FALSE(toString(LcpSolverStatus::MaxIterations).empty());
  EXPECT_FALSE(toString(LcpSolverStatus::NumericalError).empty());
  EXPECT_FALSE(toString(LcpSolverStatus::InvalidProblem).empty());
  EXPECT_FALSE(toString(LcpSolverStatus::Degenerate).empty());
  EXPECT_FALSE(toString(LcpSolverStatus::NotSolved).empty());
}

//==============================================================================
TEST(LcpTypes, LcpProblemConstruction)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);

  LcpProblem problem(A, b, lo, hi, findex);
  EXPECT_EQ(problem.A.rows(), 2);
  EXPECT_EQ(problem.A.cols(), 2);
  EXPECT_EQ(problem.b.size(), 2);
  EXPECT_EQ(problem.lo.size(), 2);
  EXPECT_EQ(problem.hi.size(), 2);
  EXPECT_EQ(problem.findex.size(), 2);
}

//=============================================================================
TEST(BaraffSolver, EmptyProblem)
{
  BaraffSolver solver;
  Eigen::MatrixXd A(0, 0);
  Eigen::VectorXd b(0);
  Eigen::VectorXd lo(0);
  Eigen::VectorXd hi(0);
  Eigen::VectorXi findex(0);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x;

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(x.size(), 0);
}

//=============================================================================
TEST(BaraffSolver, NonStandardBoundsFallback)
{
  BaraffSolver solver;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd target(2);
  target << 0.5, 0.25;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(2, 1.0);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_NEAR(x[0], target[0], 1e-6);
  EXPECT_NEAR(x[1], target[1], 1e-6);
}

//=============================================================================
TEST(BaraffSolver, WarmStartClassificationEarlyExit)
{
  BaraffSolver solver;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b(3);
  b << 0.5, -1.0, 0.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(3, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x(3);
  x << 0.5, 0.0, 0.5;

  LcpOptions options;
  options.warmStart = true;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Failed);
  EXPECT_EQ(result.iterations, 0);
}

//=============================================================================
TEST(BaraffSolver, SingularActiveSetMatrixNumericalError)
{
  BaraffSolver solver;
  Eigen::MatrixXd A(3, 3);
  A << 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::VectorXd b(3);
  b << 2.0, 2.0, 1.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(3, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x(3);
  x << 1.0, 1.0, 0.0;

  LcpOptions options;
  options.warmStart = true;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::NumericalError);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(BaraffSolver, InvalidSearchDirection)
{
  BaraffSolver solver;
  Eigen::MatrixXd A(1, 1);
  A(0, 0) = 0.0;
  Eigen::VectorXd b(1);
  b << 1.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(1, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::NumericalError);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(BaraffSolver, BlockingFromActiveSetHitsMaxIterations)
{
  BaraffSolver solver;
  Eigen::MatrixXd A(2, 2);
  A << 1.0, 1.0, 1.0, 2.0;
  Eigen::VectorXd b(2);
  b << 0.5, 2.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x(2);
  x << 0.5, 0.0;

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 1;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::MaxIterations);
  EXPECT_EQ(result.iterations, 1);
}

//=============================================================================
TEST(BaraffSolver, BlockingFromFreeSetHitsMaxIterations)
{
  BaraffSolver solver;
  Eigen::MatrixXd A(2, 2);
  A << 1.0, -1.0, -1.0, 2.0;
  Eigen::VectorXd b(2);
  b << -0.1, 1.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 1;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::MaxIterations);
}

//=============================================================================
TEST(BaraffSolver, EnteringFreeSetWhenStepSmall)
{
  BaraffSolver solver;
  Eigen::MatrixXd A(1, 1);
  A(0, 0) = 1e6;
  Eigen::VectorXd b(1);
  b << 1.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(1, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);

  LcpOptions options;
  options.absoluteTolerance = 1e-5;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Failed);
}

//=============================================================================
TEST(BaraffSolver, ConvergedValidatedSolution)
{
  BaraffSolver solver;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd target(2);
  target << 0.25, 0.75;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.validateSolution = true;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(result.validated);
  EXPECT_NEAR(x[0], target[0], 1e-6);
  EXPECT_NEAR(x[1], target[1], 1e-6);
}

//=============================================================================
TEST(ShockPropagationSolver, EmptyProblem)
{
  ShockPropagationSolver solver;
  Eigen::MatrixXd A(0, 0);
  Eigen::VectorXd b(0);
  Eigen::VectorXd lo(0);
  Eigen::VectorXd hi(0);
  Eigen::VectorXi findex(0);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x;

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(x.size(), 0);
}

//=============================================================================
TEST(ShockPropagationSolver, InvalidProblemDimensions)
{
  ShockPropagationSolver solver;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x;

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(ShockPropagationSolver, BlockSizesZero)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {0, 1};
  solver.setParameters(params);

  auto problem = makeSimpleDiagonalProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(ShockPropagationSolver, BlockSizesSumMismatch)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 2};
  solver.setParameters(params);

  auto problem = makeSimpleDiagonalProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(ShockPropagationSolver, FrictionIndexOutsideBlock)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  solver.setParameters(params);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex(2);
  findex << 1, -1;
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(ShockPropagationSolver, EmptyLayerError)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  params.layers = {{}};
  solver.setParameters(params);

  auto problem = makeSimpleDiagonalProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(ShockPropagationSolver, LayerIndexOutOfRange)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  params.layers = {{0, 2}};
  solver.setParameters(params);

  auto problem = makeSimpleDiagonalProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(ShockPropagationSolver, BlockIndexMultipleLayers)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  params.layers = {{0}, {0}};
  solver.setParameters(params);

  auto problem = makeSimpleDiagonalProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(ShockPropagationSolver, LayersNotCoveringAllBlocks)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  params.layers = {{0}};
  solver.setParameters(params);

  auto problem = makeSimpleDiagonalProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//=============================================================================
TEST(ShockPropagationSolver, CustomOptionsOverride)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {0};
  solver.setParameters(params);

  ShockPropagationSolver::Parameters customParams;
  customParams.blockSizes = {1, 1};
  customParams.layers = {{0, 1}};

  auto problem = makeSimpleDiagonalProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.customOptions = &customParams;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
}

//=============================================================================
TEST(ShockPropagationSolver, DirectSolverSmallStandardBlock)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {2};
  solver.setParameters(params);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2) * 2.0;
  Eigen::VectorXd target(2);
  target << 0.5, 1.0;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 5;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_NEAR(x[0], target[0], 1e-6);
  EXPECT_NEAR(x[1], target[1], 1e-6);
}

//=============================================================================
TEST(ShockPropagationSolver, NonStandardBlockUsesDantzig)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {2};
  solver.setParameters(params);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2) * 2.0;
  Eigen::VectorXd target(2);
  target << 0.25, 0.75;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(2, 1.0);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 5;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_LE(x[0], 1.0 + 1e-6);
  EXPECT_LE(x[1], 1.0 + 1e-6);
}

//=============================================================================
TEST(ShockPropagationSolver, MaxIterationsStatus)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  solver.setParameters(params);

  Eigen::MatrixXd A(2, 2);
  A << 2.0, 1.0, 1.0, 2.0;
  Eigen::VectorXd b(2);
  b << 3.0, 3.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 1;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::MaxIterations);
  EXPECT_EQ(result.iterations, 1);
}

//=============================================================================
TEST(ShockPropagationSolver, ValidationOnSuccess)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  solver.setParameters(params);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd target(2);
  target << 0.4, 0.6;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 5;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(result.validated);
}

//=============================================================================
TEST(ShockPropagationSolver, WarmStartConvergesImmediately)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {2};
  solver.setParameters(params);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd target(2);
  target << 0.2, 0.8;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = target;

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 5;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
}
