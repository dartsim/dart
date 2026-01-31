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
#include <dart/math/lcp/other/mprgp_solver.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/blocked_jacobi_solver.hpp>
#include <dart/math/lcp/projection/subspace_minimization_solver.hpp>
#include <dart/math/lcp/projection/symmetric_psor_solver.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart::math;

namespace {

constexpr double kTol = 1e-6;

LcpProblem makeDiagonalProblem(int n)
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

LcpProblem makeFrictionProblem()
{
  Eigen::MatrixXd A(3, 3);
  A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;
  const Eigen::Vector3d target(1.0, 0.2, -0.1);
  const Eigen::VectorXd b = -(A * target);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd hi(3);
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5;
  Eigen::VectorXi findex(3);
  findex << -1, 0, 0;
  return LcpProblem(
      std::move(A), b, std::move(lo), std::move(hi), std::move(findex));
}

LcpProblem makeZeroRhsProblem(int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 2.0;
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
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

LcpProblem makeIdentityProblem(int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(n);
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

void ExpectNonnegative(const Eigen::VectorXd& x, double tol)
{
  for (int i = 0; i < x.size(); ++i) {
    EXPECT_GE(x[i], -tol);
  }
}

void ExpectBoxed(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    double tol)
{
  for (int i = 0; i < x.size(); ++i) {
    if (std::isfinite(lo[i])) {
      EXPECT_GE(x[i], lo[i] - tol);
    }
    if (std::isfinite(hi[i])) {
      EXPECT_LE(x[i], hi[i] + tol);
    }
  }
}

void ExpectFrictionBounds(
    const Eigen::Ref<const Eigen::VectorXd>& x, double mu, double tol)
{
  const double normal = std::abs(x[0]);
  const double bound = mu * normal;
  EXPECT_GE(x[0], -tol);
  EXPECT_LE(x[1], bound + tol);
  EXPECT_GE(x[1], -bound - tol);
  EXPECT_LE(x[2], bound + tol);
  EXPECT_GE(x[2], -bound - tol);
}

LcpProblem makeDantzigUnboundedProblem()
{
  Eigen::MatrixXd A(3, 3);
  A << 4.0, 1.0, 0.25, 1.0, 3.0, 0.5, 0.25, 0.5, 2.5;
  Eigen::VectorXd target(3);
  target << 0.5, -0.2, 0.1;
  const Eigen::VectorXd b = -(A * target);
  Eigen::VectorXd lo(3);
  lo << 0.0, -std::numeric_limits<double>::infinity(), 0.0;
  Eigen::VectorXd hi(3);
  hi << std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), 1.0;
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  return LcpProblem(
      std::move(A), b, std::move(lo), std::move(hi), std::move(findex));
}

LcpProblem makeDantzigBoxedProblem()
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
  const Eigen::Vector2d b = w - A * target;
  return LcpProblem(A, b, lo, hi, Eigen::Vector2i::Constant(-1));
}

LcpProblem makeDantzigLargeProblem()
{
  constexpr int n = 8;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 4.0;
  A(0, 1) = A(1, 0) = 0.2;
  A(0, 2) = A(2, 0) = 0.1;
  A(1, 2) = A(2, 1) = 0.05;
  A(3, 4) = A(4, 3) = 0.15;
  A(3, 5) = A(5, 3) = 0.1;
  A(4, 5) = A(5, 4) = 0.05;
  A(6, 7) = A(7, 6) = 0.1;

  Eigen::VectorXd target(n);
  target << 1.0, 0.2, -0.1, 0.8, -0.05, 0.15, 0.2, 0.25;

  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi(n);
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5,
      std::numeric_limits<double>::infinity(), 0.4, 0.4, 0.2, 0.25;

  Eigen::VectorXi findex(n);
  findex << -1, 0, 0, -1, 3, 3, -1, -1;

  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  w[6] = -0.1;
  w[7] = -0.05;
  const Eigen::VectorXd b = w - A * target;

  return LcpProblem(
      std::move(A), b, std::move(lo), std::move(hi), std::move(findex));
}

} // namespace

//==============================================================================
TEST(MprgpSolver, NameAndCategory)
{
  MprgpSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(MprgpSolver, SolveDiagonalProblem)
{
  MprgpSolver solver;
  auto problem = makeDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(MprgpSolver, SolveSpdProblem)
{
  MprgpSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(MprgpSolver, SolveBoxedProblem)
{
  MprgpSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectBoxed(x, problem.lo, problem.hi, kTol);
}

//==============================================================================
TEST(MprgpSolver, SolveFrictionProblem)
{
  MprgpSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectFrictionBounds(x, 0.5, kTol);
}

//==============================================================================
TEST(MprgpSolver, SolveWarmStart)
{
  MprgpSolver solver;
  auto problem = makeDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.25);

  LcpOptions options;
  options.maxIterations = 200;
  options.warmStart = true;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(MprgpSolver, SolveEdgeCases)
{
  MprgpSolver solver;
  LcpOptions options;
  options.maxIterations = 200;

  auto size1 = makeDiagonalProblem(1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  auto result = solver.solve(size1, x, options);
  EXPECT_TRUE(x.array().isFinite().all());

  auto zeroRhs = makeZeroRhsProblem(3);
  x = Eigen::VectorXd::Zero(3);
  result = solver.solve(zeroRhs, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NEAR(x.norm(), 0.0, 1e-10);

  auto identity = makeIdentityProblem(2);
  x = Eigen::VectorXd::Zero(2);
  result = solver.solve(identity, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(BlockedJacobiSolver, NameAndCategory)
{
  BlockedJacobiSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(BlockedJacobiSolver, SolveDiagonalProblem)
{
  BlockedJacobiSolver solver;
  auto problem = makeDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(BlockedJacobiSolver, SolveSpdProblem)
{
  BlockedJacobiSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(BlockedJacobiSolver, SolveBoxedProblem)
{
  BlockedJacobiSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectBoxed(x, problem.lo, problem.hi, kTol);
}

//==============================================================================
TEST(BlockedJacobiSolver, SolveFrictionProblem)
{
  BlockedJacobiSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectFrictionBounds(x, 0.5, kTol);
}

//==============================================================================
TEST(BlockedJacobiSolver, SolveWarmStart)
{
  BlockedJacobiSolver solver;
  auto problem = makeDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.25);

  LcpOptions options;
  options.maxIterations = 100;
  options.warmStart = true;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(BlockedJacobiSolver, SolveEdgeCases)
{
  BlockedJacobiSolver solver;
  LcpOptions options;
  options.maxIterations = 100;

  auto size1 = makeDiagonalProblem(1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  auto result = solver.solve(size1, x, options);
  EXPECT_TRUE(x.array().isFinite().all());

  auto zeroRhs = makeZeroRhsProblem(3);
  x = Eigen::VectorXd::Zero(3);
  result = solver.solve(zeroRhs, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NEAR(x.norm(), 0.0, 1e-10);

  auto identity = makeIdentityProblem(2);
  x = Eigen::VectorXd::Zero(2);
  result = solver.solve(identity, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(SymmetricPsorSolver, NameAndCategory)
{
  SymmetricPsorSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(SymmetricPsorSolver, SolveDiagonalProblem)
{
  SymmetricPsorSolver solver;
  auto problem = makeDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(SymmetricPsorSolver, SolveSpdProblem)
{
  SymmetricPsorSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(SymmetricPsorSolver, SolveBoxedProblem)
{
  SymmetricPsorSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectBoxed(x, problem.lo, problem.hi, kTol);
}

//==============================================================================
TEST(SymmetricPsorSolver, SolveFrictionProblem)
{
  SymmetricPsorSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 100;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectFrictionBounds(x, 0.5, kTol);
}

//==============================================================================
TEST(SymmetricPsorSolver, SolveWarmStart)
{
  SymmetricPsorSolver solver;
  auto problem = makeDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.25);

  LcpOptions options;
  options.maxIterations = 100;
  options.warmStart = true;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(SymmetricPsorSolver, SolveEdgeCases)
{
  SymmetricPsorSolver solver;
  LcpOptions options;
  options.maxIterations = 100;

  auto size1 = makeDiagonalProblem(1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  auto result = solver.solve(size1, x, options);
  EXPECT_TRUE(x.array().isFinite().all());

  auto zeroRhs = makeZeroRhsProblem(3);
  x = Eigen::VectorXd::Zero(3);
  result = solver.solve(zeroRhs, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NEAR(x.norm(), 0.0, 1e-10);

  auto identity = makeIdentityProblem(2);
  x = Eigen::VectorXd::Zero(2);
  result = solver.solve(identity, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(SubspaceMinimizationSolver, NameAndCategory)
{
  SubspaceMinimizationSolver solver;
  EXPECT_FALSE(solver.getName().empty());
  EXPECT_FALSE(solver.getCategory().empty());
}

//==============================================================================
TEST(SubspaceMinimizationSolver, SolveDiagonalProblem)
{
  SubspaceMinimizationSolver solver;
  auto problem = makeDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(SubspaceMinimizationSolver, SolveSpdProblem)
{
  SubspaceMinimizationSolver solver;
  auto problem = makeSpdProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(SubspaceMinimizationSolver, SolveBoxedProblem)
{
  SubspaceMinimizationSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectBoxed(x, problem.lo, problem.hi, kTol);
}

//==============================================================================
TEST(SubspaceMinimizationSolver, SolveFrictionProblem)
{
  SubspaceMinimizationSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 200;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectFrictionBounds(x, 0.5, kTol);
}

//==============================================================================
TEST(SubspaceMinimizationSolver, SolveWarmStart)
{
  SubspaceMinimizationSolver solver;
  auto problem = makeDiagonalProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.25);

  LcpOptions options;
  options.maxIterations = 200;
  options.warmStart = true;
  auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(SubspaceMinimizationSolver, SolveEdgeCases)
{
  SubspaceMinimizationSolver solver;
  LcpOptions options;
  options.maxIterations = 200;

  auto size1 = makeDiagonalProblem(1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  auto result = solver.solve(size1, x, options);
  EXPECT_TRUE(x.array().isFinite().all());

  auto zeroRhs = makeZeroRhsProblem(3);
  x = Eigen::VectorXd::Zero(3);
  result = solver.solve(zeroRhs, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NEAR(x.norm(), 0.0, 1e-10);

  auto identity = makeIdentityProblem(2);
  x = Eigen::VectorXd::Zero(2);
  result = solver.solve(identity, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  ExpectNonnegative(x, kTol);
}

//==============================================================================
TEST(DantzigSolver, MatrixOperationsAcrossSizes)
{
  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 100;
  options.warmStart = false;

  Eigen::MatrixXd A1(1, 1);
  A1(0, 0) = 2.0;
  Eigen::VectorXd b1(1);
  b1 << 1.0;
  Eigen::VectorXd lo1(1);
  lo1 << -std::numeric_limits<double>::infinity();
  Eigen::VectorXd hi1(1);
  hi1 << std::numeric_limits<double>::infinity();
  Eigen::VectorXi findex1(1);
  findex1 << -1;

  LcpProblem size1(
      std::move(A1),
      std::move(b1),
      std::move(lo1),
      std::move(hi1),
      std::move(findex1));
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  auto result = solver.solve(size1, x, options);
  EXPECT_TRUE(x.array().isFinite().all());

  auto unbounded = makeDantzigUnboundedProblem();
  x = Eigen::VectorXd::Zero(3);
  result = solver.solve(unbounded, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  ExpectBoxed(x, unbounded.lo, unbounded.hi, kTol);

  auto boxed = makeDantzigBoxedProblem();
  x = Eigen::VectorXd::Zero(2);
  result = solver.solve(boxed, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  ExpectBoxed(x, boxed.lo, boxed.hi, kTol);

  auto large = makeDantzigLargeProblem();
  x = Eigen::VectorXd::Zero(8);
  result = solver.solve(large, x, options);
  EXPECT_TRUE(x.array().isFinite().all());
  ExpectFrictionBounds(x.head<3>(), 0.5, kTol);
  ExpectBoxed(x, large.lo, large.hi, kTol);
}
