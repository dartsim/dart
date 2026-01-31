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
#include <dart/math/lcp/lcp_validation.hpp>
#include <dart/math/lcp/newton/fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/newton/minimum_map_newton_solver.hpp>
#include <dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/other/mprgp_solver.hpp>
#include <dart/math/lcp/other/shock_propagation_solver.hpp>
#include <dart/math/lcp/other/staggering_solver.hpp>
#include <dart/math/lcp/pivoting/dantzig/matrix.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/pivoting/direct_solver.hpp>
#include <dart/math/lcp/pivoting/lemke_solver.hpp>
#include <dart/math/lcp/projection/bgs_solver.hpp>
#include <dart/math/lcp/projection/blocked_jacobi_solver.hpp>
#include <dart/math/lcp/projection/jacobi_solver.hpp>
#include <dart/math/lcp/projection/nncg_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>
#include <dart/math/lcp/projection/red_black_gauss_seidel_solver.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include <cmath>

using namespace dart::math;

namespace {

LcpProblem makeStandardProblem(int n, double diag = 2.0, double target = 0.25)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * diag;
  const Eigen::VectorXd xTarget = Eigen::VectorXd::Constant(n, target);
  Eigen::VectorXd b = A * xTarget;
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

LcpProblem makeBoxedProblem()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2) * 3.0;
  Eigen::VectorXd b(2);
  b << 0.6, 0.9;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(2, 0.8);
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
  A << 3.0, 0.1, 0.0, 0.1, 2.5, 0.05, 0.0, 0.05, 2.0;
  Eigen::VectorXd target(3);
  target << 0.4, 0.1, -0.05;
  Eigen::VectorXd b = A * target;

  Eigen::VectorXd lo(3);
  lo << 0.0, -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity();
  Eigen::VectorXd hi(3);
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5;
  Eigen::VectorXi findex(3);
  findex << -1, 0, 0;
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeZeroProblem()
{
  Eigen::MatrixXd A(0, 0);
  Eigen::VectorXd b(0);
  Eigen::VectorXd lo(0);
  Eigen::VectorXd hi(0);
  Eigen::VectorXi findex(0);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

} // namespace

TEST(LcpValidationCoverage, DetectsDimensionMismatch)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, DetectsFindexOutOfRange)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex(2);
  findex << 2, -1;
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, DetectsSelfReferenceFindex)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex(2);
  findex << -1, 1;
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, DetectsNaNBounds)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(1, 1);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi(1);
  hi[0] = std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(1, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, DetectsFrictionWithInfiniteHi)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex(2);
  findex << -1, 0;
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, ComputesEffectiveBoundsForFriction)
{
  Eigen::VectorXd lo(3);
  lo << 0.0, -1.0, -1.0;
  Eigen::VectorXd hi(3);
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.25;
  Eigen::VectorXi findex(3);
  findex << -1, 0, 0;
  Eigen::VectorXd x(3);
  x << 0.4, 0.0, 0.0;

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_TRUE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_NEAR(loEff[1], -0.2, 1e-12);
  EXPECT_NEAR(hiEff[1], 0.2, 1e-12);
  EXPECT_NEAR(loEff[2], -0.1, 1e-12);
  EXPECT_NEAR(hiEff[2], 0.1, 1e-12);
}

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsMismatchedSizes)
{
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_FALSE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, ValidatesSolutionBounds)
{
  Eigen::VectorXd x(2);
  x << 0.5, 0.5;
  Eigen::VectorXd w(2);
  w << 0.0, 0.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);

  std::string message;
  EXPECT_TRUE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

  x[1] = 1.5;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, ComputesResidualAndComplementarityNorms)
{
  Eigen::VectorXd x(2);
  x << 0.2, 0.8;
  Eigen::VectorXd w(2);
  w << 0.1, -0.3;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);

  const double residual = detail::naturalResidualInfinityNorm(x, w, lo, hi);
  const double comp = detail::complementarityInfinityNorm(x, w, lo, hi, 1e-6);
  EXPECT_GT(residual, 0.0);
  EXPECT_GT(comp, 0.0);
}

TEST(LcpTypesCoverage, LcpResultSucceeded)
{
  LcpResult ok(LcpSolverStatus::Success);
  EXPECT_TRUE(ok.succeeded());

  LcpResult fail(LcpSolverStatus::Failed);
  EXPECT_FALSE(fail.succeeded());
}

TEST(LcpTypesCoverage, LcpOptionsWarmStartDefaults)
{
  LcpOptions options;
  EXPECT_FALSE(options.warmStart);
  const auto tuned = LcpOptions::highAccuracy();
  EXPECT_GT(tuned.maxIterations, 0);
}

TEST(DantzigMatrixCoverage, DotUsesBothPaths)
{
  std::vector<double> aSmall = {1.0, 2.0, 3.0, 4.0};
  std::vector<double> bSmall = {0.5, 0.5, 0.5, 0.5};
  const double expectedSmall = 5.0;
  EXPECT_NEAR(Dot(aSmall.data(), bSmall.data(), 4), expectedSmall, 1e-12);

  const int nLarge = 32;
  std::vector<double> aLarge(nLarge);
  std::vector<double> bLarge(nLarge);
  double expectedLarge = 0.0;
  for (int i = 0; i < nLarge; ++i) {
    aLarge[i] = static_cast<double>(i + 1);
    bLarge[i] = 0.25;
    expectedLarge += aLarge[i] * bLarge[i];
  }
  EXPECT_NEAR(Dot(aLarge.data(), bLarge.data(), nLarge), expectedLarge, 1e-12);
}

TEST(DantzigMatrixCoverage, FactorAndSolveCholesky)
{
  const int n = 2;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  A[0] = 4.0;
  A[1] = 1.0;
  A[nskip] = 1.0;
  A[nskip + 1] = 3.0;

  std::vector<double> b = {1.0, 2.0};
  EXPECT_EQ(dFactorCholesky(A.data(), n), 1);
  dSolveCholesky(A.data(), b.data(), n);

  Eigen::Matrix2d Ae;
  Ae << 4.0, 1.0, 1.0, 3.0;
  Eigen::Vector2d be;
  be << 1.0, 2.0;
  const Eigen::Vector2d expected = Ae.ldlt().solve(be);
  EXPECT_NEAR(b[0], expected[0], 1e-8);
  EXPECT_NEAR(b[1], expected[1], 1e-8);
}

TEST(DantzigMatrixCoverage, CholeskyFailureOnNonPositiveDefinite)
{
  const int n = 2;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  A[0] = 0.0;
  A[nskip + 1] = -1.0;
  EXPECT_EQ(dFactorCholesky(A.data(), n), 0);
}

TEST(DantzigMatrixCoverage, InvertPositiveDefiniteMatrix)
{
  const int n = 2;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  A[0] = 2.0;
  A[1] = 0.5;
  A[nskip] = 0.5;
  A[nskip + 1] = 1.5;

  std::vector<double> Ainv(n * nskip, 0.0);
  EXPECT_EQ(dInvertPDMatrix(A.data(), Ainv.data(), n), 1);

  Eigen::Matrix2d Ae;
  Ae << 2.0, 0.5, 0.5, 1.5;
  const Eigen::Matrix2d expected = Ae.inverse();
  EXPECT_NEAR(Ainv[0], expected(0, 0), 1e-8);
  EXPECT_NEAR(Ainv[1], expected(0, 1), 1e-8);
  EXPECT_NEAR(Ainv[nskip], expected(1, 0), 1e-8);
  EXPECT_NEAR(Ainv[nskip + 1], expected(1, 1), 1e-8);
}

TEST(DantzigMatrixCoverage, PositiveDefiniteCheck)
{
  const int n = 2;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  A[0] = 1.0;
  A[1] = 0.0;
  A[nskip] = 0.0;
  A[nskip + 1] = 1.0;
  EXPECT_EQ(dIsPositiveDefinite(A.data(), n), 1);

  std::vector<double> B(n * nskip, 0.0);
  B[0] = 1.0;
  B[1] = 2.0;
  B[nskip] = 2.0;
  B[nskip + 1] = 1.0;
  EXPECT_EQ(dIsPositiveDefinite(B.data(), n), 0);
}

TEST(DantzigMatrixCoverage, VectorScaleAndSolveLdlt)
{
  const int n = 3;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  Eigen::Matrix3d Ae;
  Ae << 4.0, 0.2, 0.1, 0.2, 3.5, 0.3, 0.1, 0.3, 2.5;
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      A[r * nskip + c] = Ae(r, c);
    }
  }

  std::vector<double> d(n, 0.0);
  dFactorLDLT(A.data(), d.data(), n, nskip);

  std::vector<double> b = {1.0, 2.0, 3.0};
  dSolveLDLT(A.data(), d.data(), b.data(), n, nskip);

  const Eigen::Vector3d expected
      = Ae.ldlt().solve(Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_NEAR(b[0], expected[0], 1e-6);
  EXPECT_NEAR(b[1], expected[1], 1e-6);
  EXPECT_NEAR(b[2], expected[2], 1e-6);

  std::vector<double> scale = {2.0, 1.0, 0.5};
  dVectorScale(b.data(), scale.data(), n);
  EXPECT_NEAR(b[0], expected[0] * 2.0, 1e-6);
}

TEST(DantzigMatrixCoverage, LDLTUpdatesAndRemoveRowCol)
{
  const int n = 3;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  Eigen::Matrix3d Ae;
  Ae << 2.0, 0.1, 0.0, 0.1, 1.8, 0.2, 0.0, 0.2, 1.5;
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      A[r * nskip + c] = Ae(r, c);
    }
  }

  std::vector<double> d(n, 0.0);
  dFactorLDLT(A.data(), d.data(), n, nskip);

  std::vector<double> a = {1.0, -0.2, 0.3};
  dLDLTAddTL(A.data(), d.data(), a.data(), n, nskip);
  EXPECT_TRUE(std::isfinite(d[0]));

  std::vector<double*> rows(n);
  for (int r = 0; r < n; ++r) {
    rows[r] = A.data() + r * nskip;
  }
  std::vector<int> perm = {0, 1, 2};
  dLDLTRemove(rows.data(), perm.data(), A.data(), d.data(), n, n, 1, nskip);
  EXPECT_TRUE(std::isfinite(d[0]));

  std::vector<double> dense = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  dRemoveRowCol(dense.data(), n, n, 1);
  EXPECT_NEAR(dense[0], 1.0, 1e-12);
  EXPECT_NEAR(dense[1], 3.0, 1e-12);
}

TEST(NncgSolverCoverage, RejectsInvalidParameters)
{
  NncgSolver solver;
  NncgSolver::Parameters params;
  params.pgsIterations = 0;
  params.restartInterval = 5;
  params.restartThreshold = 0.5;

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

TEST(NncgSolverCoverage, SolvesFrictionWarmStart)
{
  NncgSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.05);

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(MprgpSolverCoverage, SolvesStandardProblem)
{
  MprgpSolver solver;
  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 100;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(MprgpSolverCoverage, RejectsInvalidDivisionEpsilon)
{
  MprgpSolver solver;
  MprgpSolver::Parameters params;
  params.epsilonForDivision = 0.0;

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(LemkeSolverCoverage, SolvesStandardProblem)
{
  LemkeSolver solver;
  auto problem = makeStandardProblem(3, 2.5, 0.4);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(LemkeSolverCoverage, FallsBackForBoxedProblem)
{
  LemkeSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(BlockedJacobiSolverCoverage, RejectsInvalidBlockSizes)
{
  BlockedJacobiSolver solver;
  BlockedJacobiSolver::Parameters params;
  params.blockSizes = {1, 1};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(BlockedJacobiSolverCoverage, SolvesFrictionBlock)
{
  BlockedJacobiSolver solver;
  BlockedJacobiSolver::Parameters params;
  params.blockSizes = {3};

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 50;

  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(BgsSolverCoverage, RejectsZeroBlockSize)
{
  BgsSolver solver;
  BgsSolver::Parameters params;
  params.blockSizes = {2, 0};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(BgsSolverCoverage, SolvesStandardProblem)
{
  BgsSolver solver;
  BgsSolver::Parameters params;
  params.blockSizes = {2};

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 50;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(RedBlackGaussSeidelCoverage, RejectsInvalidRelaxation)
{
  RedBlackGaussSeidelSolver solver;
  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.relaxation = 3.0;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(RedBlackGaussSeidelCoverage, SolvesFrictionWarmStart)
{
  RedBlackGaussSeidelSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.05);

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 80;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(JacobiSolverCoverage, RejectsInvalidDivisionEpsilon)
{
  JacobiSolver solver;
  JacobiSolver::Parameters params;
  params.epsilonForDivision = 0.0;

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(JacobiSolverCoverage, SolvesFrictionProblem)
{
  JacobiSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 150;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(PgsSolverCoverage, RejectsInvalidRelaxation)
{
  PgsSolver solver;
  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.relaxation = 3.0;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(PgsSolverCoverage, SolvesWarmStartWithRandomOrder)
{
  PgsSolver solver;
  PgsSolver::Parameters params;
  params.randomizeConstraintOrder = true;

  LcpOptions options;
  options.customOptions = &params;
  options.warmStart = true;
  options.maxIterations = 16;

  auto problem = makeStandardProblem(4);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(4, 0.1);
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(DantzigSolverCoverage, SolvesFrictionWarmStart)
{
  DantzigSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.05);

  LcpOptions options;
  options.warmStart = true;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(DantzigSolverCoverage, SolvesZeroSizeProblem)
{
  DantzigSolver solver;
  auto problem = makeZeroProblem();
  Eigen::VectorXd x;

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(x.size(), 0);
}

TEST(DirectSolverCoverage, FallsBackForLargeProblem)
{
  DirectSolver solver;
  auto problem = makeStandardProblem(4);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(DirectSolverCoverage, SolvesSmallStandardProblem)
{
  DirectSolver solver;
  auto problem = makeStandardProblem(2, 2.0, 0.3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(StaggeringSolverCoverage, FallsBackWithoutFriction)
{
  StaggeringSolver solver;
  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(StaggeringSolverCoverage, SolvesFrictionProblem)
{
  StaggeringSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 60;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(ShockPropagationSolverCoverage, RejectsDuplicateLayerIndices)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  params.layers = {{0}, {0}};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(ShockPropagationSolverCoverage, SolvesWithCustomLayers)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1, 1};
  params.layers = {{0, 1, 2}};

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 2;

  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(FischerBurmeisterNewtonCoverage, SolvesStandardProblem)
{
  FischerBurmeisterNewtonSolver solver;
  auto problem = makeStandardProblem(3, 2.0, 0.2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 25;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(PenalizedFischerBurmeisterNewtonCoverage, RejectsInvalidLambda)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  PenalizedFischerBurmeisterNewtonSolver::Parameters params;
  params.lambda = 1.5;

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(MinimumMapNewtonCoverage, FallsBackForBoxedProblem)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}
