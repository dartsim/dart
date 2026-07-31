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
#include <vector>

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

//==============================================================================
TEST(PgsSolver, RejectsInvalidRelaxation)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(1, 1);
  Eigen::VectorXd b(1);
  b << 1.0;

  LcpProblem problem(
      A,
      b,
      Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity()),
      Eigen::VectorXi::Constant(1, -1));

  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 5;
  options.warmStart = false;
  options.relaxation = 2.5;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

//==============================================================================
TEST(PgsSolver, HandlesZeroDiagonalEntries)
{
  Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
  A(1, 1) = 2.0;
  Eigen::Vector2d b;
  b << 1.0, 0.5;

  LcpProblem problem(
      A,
      b,
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector2i::Constant(-1));

  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 3;
  options.warmStart = false;
  options.validateSolution = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);

  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_NEAR(x[0], 0.0, 1e-12);
}

//==============================================================================
namespace {

// The boxed friction-index problem from SolvesBoxedProblemWithFrictionIndex,
// shared by the scratch-overload tests below.
struct BoxedFrictionProblem
{
  Eigen::Matrix3d A;
  Eigen::Vector3d b;
  Eigen::Vector3d lo = Eigen::Vector3d::Zero();
  Eigen::Vector3d hi;
  Eigen::Vector3i findex;

  BoxedFrictionProblem()
  {
    A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;
    b = A * Eigen::Vector3d(1.0, 0.2, -0.1);
    hi << std::numeric_limits<double>::infinity(), 0.5, 0.5;
    findex << -1, 0, 0;
  }
};

} // namespace

//==============================================================================
TEST(PgsSolver, ScratchOverloadMatchesOwningSolve)
{
  const BoxedFrictionProblem fixture;

  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 5000;
  options.warmStart = false;
  options.complementarityTolerance = 1e-3;

  Eigen::VectorXd owningX = Eigen::VectorXd::Zero(3);
  const LcpProblem problem(
      fixture.A, fixture.b, fixture.lo, fixture.hi, fixture.findex);
  const auto owningResult = solver.solve(problem, owningX, options);
  ASSERT_TRUE(owningResult.succeeded());

  PgsSolver::Scratch scratch;
  Eigen::VectorXd scratchX = Eigen::VectorXd::Zero(3);
  const auto scratchResult = solver.solve(
      fixture.A,
      fixture.b,
      fixture.lo,
      fixture.hi,
      fixture.findex,
      scratchX,
      scratch,
      options);

  EXPECT_EQ(scratchResult.status, owningResult.status);
  EXPECT_EQ(scratchResult.iterations, owningResult.iterations);
  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(scratchX[i], owningX[i]);
  }
}

//==============================================================================
TEST(PgsSolver, ScratchOverloadSolvesInPlaceThroughMaps)
{
  const BoxedFrictionProblem fixture;

  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 5000;
  options.warmStart = false;
  options.complementarityTolerance = 1e-3;

  // Mirror the boxed-LCP contact fallback: every operand, including the
  // solution, is a map over caller-owned storage.
  std::vector<double> aData(9), bData(3), loData(3), hiData(3), xData(3, 0.0);
  std::vector<int> findexData(3);
  Eigen::Map<Eigen::MatrixXd>(aData.data(), 3, 3) = fixture.A;
  Eigen::Map<Eigen::Vector3d>(bData.data()) = fixture.b;
  Eigen::Map<Eigen::Vector3d>(loData.data()) = fixture.lo;
  Eigen::Map<Eigen::Vector3d>(hiData.data()) = fixture.hi;
  Eigen::Map<Eigen::Vector3i>(findexData.data()) = fixture.findex;

  PgsSolver::Scratch scratch;
  Eigen::Map<Eigen::VectorXd> x(xData.data(), 3);
  const auto result = solver.solve(
      Eigen::Map<const Eigen::MatrixXd>(aData.data(), 3, 3),
      Eigen::Map<const Eigen::VectorXd>(bData.data(), 3),
      Eigen::Map<const Eigen::VectorXd>(loData.data(), 3),
      Eigen::Map<const Eigen::VectorXd>(hiData.data(), 3),
      Eigen::Map<const Eigen::VectorXi>(findexData.data(), 3),
      x,
      scratch,
      options);

  ASSERT_TRUE(result.succeeded());
  Eigen::VectorXd owningX = Eigen::VectorXd::Zero(3);
  const LcpProblem problem(
      fixture.A, fixture.b, fixture.lo, fixture.hi, fixture.findex);
  ASSERT_TRUE(solver.solve(problem, owningX, options).succeeded());
  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(x[i], owningX[i]);
  }
}

//==============================================================================
TEST(PgsSolver, ScratchOverloadZeroesNonFiniteWarmStart)
{
  const BoxedFrictionProblem fixture;

  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 5000;
  options.complementarityTolerance = 1e-3;

  // The contact fallback engages exactly when the pivoting solve leaves
  // non-finite impulses behind; those entries must restart from zero.
  PgsSolver::Scratch scratch;
  Eigen::VectorXd x(3);
  x << std::numeric_limits<double>::quiet_NaN(), 0.1,
      -std::numeric_limits<double>::infinity();
  options.warmStart = true;
  const auto result = solver.solve(
      fixture.A,
      fixture.b,
      fixture.lo,
      fixture.hi,
      fixture.findex,
      x,
      scratch,
      options);

  ASSERT_TRUE(result.succeeded());
  EXPECT_TRUE(x.array().isFinite().all());
}

//==============================================================================
TEST(PgsSolver, ScratchOverloadRejectsMismatchedSolutionSize)
{
  const BoxedFrictionProblem fixture;

  PgsSolver solver;
  PgsSolver::Scratch scratch;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(
      fixture.A,
      fixture.b,
      fixture.lo,
      fixture.hi,
      fixture.findex,
      x,
      scratch,
      solver.getDefaultOptions());

  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

//==============================================================================
TEST(PgsSolver, ScratchCapacityIsStableAcrossRepeatedSolves)
{
  const BoxedFrictionProblem fixture;

  PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 5000;
  options.warmStart = false;
  options.complementarityTolerance = 1e-3;

  PgsSolver::Scratch scratch;
  scratch.reserve(3);

  const auto solveOnce = [&](Eigen::VectorXd& x) {
    return solver.solve(
        fixture.A,
        fixture.b,
        fixture.lo,
        fixture.hi,
        fixture.findex,
        x,
        scratch,
        options);
  };

  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  ASSERT_TRUE(solveOnce(x).succeeded());

  const auto aCapacity = scratch.Adata.capacity();
  const auto xCapacity = scratch.xdata.capacity();
  const auto wCapacity = scratch.w.capacity();
  const auto orderCapacity = scratch.order.capacity();

  // Repeated same-shape solves must reuse the warmed buffers: capacity growth
  // here is exactly the global-heap allocation the contact fallback must not
  // perform after the bake boundary.
  x.setZero();
  ASSERT_TRUE(solveOnce(x).succeeded());
  EXPECT_EQ(scratch.Adata.capacity(), aCapacity);
  EXPECT_EQ(scratch.xdata.capacity(), xCapacity);
  EXPECT_EQ(scratch.w.capacity(), wCapacity);
  EXPECT_EQ(scratch.order.capacity(), orderCapacity);

  // A smaller same-scratch solve shrinks sizes, never capacities.
  Eigen::Matrix2d smallA;
  smallA << 4.0, 1.0, 1.0, 3.0;
  const Eigen::Vector2d smallB = smallA * Eigen::Vector2d(0.5, 0.25);
  Eigen::VectorXd smallX = Eigen::VectorXd::Zero(2);
  const auto smallResult = solver.solve(
      smallA,
      smallB,
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector2i::Constant(-1),
      smallX,
      scratch,
      options);
  ASSERT_TRUE(smallResult.succeeded());
  EXPECT_EQ(scratch.Adata.capacity(), aCapacity);
  EXPECT_EQ(scratch.xdata.capacity(), xCapacity);
}
