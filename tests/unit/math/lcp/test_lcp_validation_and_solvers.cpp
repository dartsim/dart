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
#include <dart/math/lcp/other/interior_point_solver.hpp>
#include <dart/math/lcp/other/mprgp_solver.hpp>
#include <dart/math/lcp/other/shock_propagation_solver.hpp>
#include <dart/math/lcp/other/staggering_solver.hpp>
#include <dart/math/lcp/pivoting/baraff_solver.hpp>
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
#include <dart/math/lcp/projection/subspace_minimization_solver.hpp>
#include <dart/math/lcp/projection/symmetric_psor_solver.hpp>

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

LcpProblem makeShiftedStandardProblem()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3) * 1.5;
  Eigen::VectorXd b(3);
  b << 0.6, 0.4, 0.2;
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

LcpProblem makeNearSingularProblem(int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    const double diag = (i == 0) ? 1e-4 : 1.0 + 0.05 * static_cast<double>(i);
    A(i, i) = diag;
    if (i + 1 < n) {
      A(i, i + 1) = 0.02;
      A(i + 1, i) = 0.02;
    }
  }
  const Eigen::VectorXd target = Eigen::VectorXd::Constant(n, 0.35);
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

LcpProblem makeNonSymmetricProblem()
{
  Eigen::MatrixXd A(2, 2);
  A << 2.0, 1.0, 0.0, 1.5;
  Eigen::VectorXd target(2);
  target << 0.3, 0.2;
  Eigen::VectorXd b = A * target;
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

LcpProblem makeIndefiniteProblem()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
  A(0, 0) = 1.0;
  A(1, 1) = -0.5;
  A(2, 2) = 1.5;
  Eigen::VectorXd target(3);
  target << 0.2, 0.1, 0.3;
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

LcpProblem makeZeroDiagonalProblem()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3) * 2.0;
  A(1, 1) = 0.0;
  A(0, 1) = 0.1;
  A(1, 0) = 0.1;
  Eigen::VectorXd target(3);
  target << 0.2, 0.1, 0.3;
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

LcpProblem makeTwoBlockStandardProblem()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(4, 4) * 2.0;
  A(0, 1) = 0.1;
  A(1, 0) = 0.1;
  A(2, 3) = 0.1;
  A(3, 2) = 0.1;
  Eigen::VectorXd target = Eigen::VectorXd::Constant(4, 0.25);
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(4, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(4, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeFrictionBlockProblem()
{
  const int contacts = 3;
  const int blockSize = 3;
  const int n = contacts * blockSize;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 2.0;

  for (int c = 0; c < contacts; ++c) {
    const int base = c * blockSize;
    A(base, base + 1) = 0.1;
    A(base + 1, base) = 0.1;
    A(base, base + 2) = 0.05;
    A(base + 2, base) = 0.05;
    A(base + 1, base + 2) = 0.08;
    A(base + 2, base + 1) = 0.08;
  }

  A(0, 3) = 0.02;
  A(3, 0) = 0.02;
  A(3, 6) = 0.02;
  A(6, 3) = 0.02;

  Eigen::VectorXd target(n);
  for (int c = 0; c < contacts; ++c) {
    const int base = c * blockSize;
    target[base] = 0.4;
    target[base + 1] = 0.1;
    target[base + 2] = -0.05;
  }
  Eigen::VectorXd b = A * target;

  Eigen::VectorXd lo(n);
  Eigen::VectorXd hi(n);
  Eigen::VectorXi findex(n);
  for (int c = 0; c < contacts; ++c) {
    const int base = c * blockSize;
    lo[base] = 0.0;
    hi[base] = std::numeric_limits<double>::infinity();
    findex[base] = -1;

    for (int f = 1; f < blockSize; ++f) {
      lo[base + f] = -std::numeric_limits<double>::infinity();
      hi[base + f] = 0.5;
      findex[base + f] = base;
    }
  }

  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeFrictionCoupledProblem()
{
  const int contacts = 2;
  const int blockSize = 3;
  const int n = contacts * blockSize;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 2.2;

  for (int c = 0; c < contacts; ++c) {
    const int base = c * blockSize;
    A(base, base + 1) = 0.12;
    A(base + 1, base) = 0.12;
    A(base, base + 2) = 0.07;
    A(base + 2, base) = 0.07;
    A(base + 1, base + 2) = 0.09;
    A(base + 2, base + 1) = 0.09;
  }

  A(0, 3) = 0.05;
  A(3, 0) = 0.05;
  A(1, 4) = 0.02;
  A(4, 1) = 0.02;
  A(2, 5) = 0.03;
  A(5, 2) = 0.03;

  Eigen::VectorXd target(n);
  for (int c = 0; c < contacts; ++c) {
    const int base = c * blockSize;
    target[base] = 0.35;
    target[base + 1] = 0.12;
    target[base + 2] = -0.08;
  }
  Eigen::VectorXd b = A * target;

  Eigen::VectorXd lo(n);
  Eigen::VectorXd hi(n);
  Eigen::VectorXi findex(n);
  for (int c = 0; c < contacts; ++c) {
    const int base = c * blockSize;
    lo[base] = 0.0;
    hi[base] = std::numeric_limits<double>::infinity();
    findex[base] = -1;

    for (int f = 1; f < blockSize; ++f) {
      lo[base + f] = -std::numeric_limits<double>::infinity();
      hi[base + f] = 0.6;
      findex[base + f] = base;
    }
  }

  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeLemkePivotProblem()
{
  const int n = 5;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 2.0;
  for (int i = 0; i < n - 1; ++i) {
    A(i, i + 1) = 0.18;
    A(i + 1, i) = 0.11;
  }
  A(0, 2) = 0.06;
  A(2, 0) = 0.06;
  A(1, 3) = 0.05;
  A(3, 1) = 0.05;
  A(2, 4) = 0.04;
  A(4, 2) = 0.04;

  Eigen::VectorXd target(n);
  target << 0.35, 0.22, 0.28, 0.15, 0.3;
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

LcpProblem makeDirectSolverProblem()
{
  Eigen::MatrixXd A(3, 3);
  A << 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 3.0;
  Eigen::VectorXd target(3);
  target << 0.2, 0.1, 0.3;
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

TEST(LcpValidationCoverage, DetectsNonSquareMatrix)
{
  Eigen::MatrixXd A(3, 2);
  A << 1.0, 0.1, 0.0, 2.0, 0.0, 0.3;
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(3);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, DetectsBoundsDimensionMismatch)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(3);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, DetectsFindexDimensionMismatch)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(3);
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

TEST(LcpValidationCoverage, DetectsLowerBoundAboveUpperBound)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(1, 1);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd lo(1);
  Eigen::VectorXd hi(1);
  lo[0] = 1.0;
  hi[0] = 0.5;
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(1, -1);
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

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsInvalidFrictionIndex)
{
  Eigen::VectorXd lo(2);
  lo << 0.0, -1.0;
  Eigen::VectorXd hi(2);
  hi << std::numeric_limits<double>::infinity(), 0.5;
  Eigen::VectorXd x(2);
  x << 0.2, 0.0;
  Eigen::VectorXi findex(2);
  findex << -1, 5;

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_FALSE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_FALSE(message.empty());

  findex << -1, 1;
  EXPECT_FALSE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsInvalidReferenceValue)
{
  Eigen::VectorXd lo(2);
  lo << 0.0, -1.0;
  Eigen::VectorXd hi(2);
  hi << std::numeric_limits<double>::infinity(), 0.5;
  Eigen::VectorXd x(2);
  x << std::numeric_limits<double>::infinity(), 0.0;
  Eigen::VectorXi findex(2);
  findex << -1, 0;

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_FALSE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsInvalidFrictionCoeff)
{
  Eigen::VectorXd lo(2);
  lo << 0.0, -1.0;
  Eigen::VectorXd hi(2);
  hi << std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity();
  Eigen::VectorXd x(2);
  x << 0.2, 0.0;
  Eigen::VectorXi findex(2);
  findex << -1, 0;

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

TEST(LcpValidationCoverage, ValidateSolutionRejectsInvalidDimensions)
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);

  std::string message;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, ValidateSolutionRejectsNonFiniteValues)
{
  Eigen::VectorXd x(2);
  x << 0.1, std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXd w = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);

  std::string message;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_FALSE(message.empty());
}

TEST(LcpValidationCoverage, ValidateSolutionDetectsComplementarityViolations)
{
  Eigen::VectorXd x(2);
  Eigen::VectorXd w(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  std::string message;

  x << -0.1, 0.5;
  w << 0.0, 0.0;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

  x << 0.5, 1.2;
  w << 0.0, 0.0;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

  lo << 0.3, 0.5;
  hi << 0.3, 0.5;
  x << 0.3, 0.5;
  w << 1.0, 0.0;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

  lo << 0.0, 0.0;
  hi << 1.0, 1.0;
  x << 0.0, 1.0;
  w << -0.1, 0.2;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

  x << 0.5, 0.5;
  w << 0.1, 0.1;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
}

TEST(LcpValidationCoverage, ValidateProblemRejectsNanBoundsWithMessage)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  hi[1] = std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_NE(message.find("NaN"), std::string::npos);
}

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsDimensionMismatch)
{
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_FALSE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_NE(message.find("dimensions"), std::string::npos);
}

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsFindexOutOfRange)
{
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(2, 0.5);
  Eigen::VectorXi findex(2);
  findex << -1, 3;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_FALSE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_NE(message.find("Invalid friction index entry"), std::string::npos);
}

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsSelfReference)
{
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(2, 0.5);
  Eigen::VectorXi findex(2);
  findex << -1, 1;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_FALSE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_NE(message.find("self reference"), std::string::npos);
}

TEST(LcpValidationCoverage, ValidateProblemRejectsNanLowerBound)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  lo[0] = std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_NE(message.find("NaN"), std::string::npos);
}

TEST(LcpValidationCoverage, ValidateSolutionFixedAndComplementarityCases)
{
  Eigen::VectorXd x(1);
  Eigen::VectorXd w(1);
  Eigen::VectorXd lo(1);
  Eigen::VectorXd hi(1);
  std::string message;

  lo << 0.2;
  hi << 0.2;
  x << 0.2;
  w << 0.1;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

  lo << 0.0;
  hi << 1.0;
  x << 0.0;
  w << -0.2;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

  x << 1.0;
  w << 0.2;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

  x << 0.5;
  w << 0.2;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
}

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsNonFiniteMu)
{
  Eigen::VectorXd lo(2);
  lo << 0.0, -1.0;
  Eigen::VectorXd hi(2);
  hi << std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity();
  Eigen::VectorXi findex(2);
  findex << -1, 0;
  Eigen::VectorXd x(2);
  x << 0.2, 0.0;

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_FALSE(detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_NE(message.find("Invalid friction coefficient"), std::string::npos);
}

TEST(LcpValidationCoverage, ValidateSolutionRejectsDimensionMismatchWithMessage)
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);

  std::string message;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_NE(message.find("dimension mismatch"), std::string::npos);
}

TEST(LcpValidationCoverage, ValidateSolutionRejectsNonFiniteValuesWithMessage)
{
  Eigen::VectorXd x(2);
  x << 0.1, std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXd w = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);

  std::string message;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_NE(message.find("non-finite"), std::string::npos);
}

TEST(LcpValidationCoverage, ValidateSolutionRejectsLowerBoundViolation)
{
  Eigen::VectorXd x(1);
  x << -0.1;
  Eigen::VectorXd w = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(1);

  std::string message;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_NE(message.find("lower bound"), std::string::npos);
}

TEST(LcpValidationCoverage, ValidateSolutionRejectsUpperBoundViolation)
{
  Eigen::VectorXd x(1);
  x << 1.2;
  Eigen::VectorXd w = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(1);

  std::string message;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_NE(message.find("upper bound"), std::string::npos);
}

TEST(LcpValidationCoverage, ValidateSolutionRejectsPositiveWAtUpperBound)
{
  Eigen::VectorXd x(1);
  x << 1.0;
  Eigen::VectorXd w(1);
  w << 0.1;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(1);

  std::string message;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_NE(message.find("non-positive"), std::string::npos);
}

TEST(LcpValidationCoverage, ValidateSolutionRejectsInteriorResidual)
{
  Eigen::VectorXd x(1);
  x << 0.5;
  Eigen::VectorXd w(1);
  w << 0.2;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(1);

  std::string message;
  EXPECT_FALSE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));
  EXPECT_NE(message.find("interior"), std::string::npos);
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

TEST(DantzigMatrixCoverage, LdltAddTlHandlesSmallSize)
{
  const int n = 1;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  A[0] = 1.0;
  std::vector<double> d(n, 1.0);
  std::vector<double> a(n, 0.2);

  dLDLTAddTL(A.data(), d.data(), a.data(), n, nskip);
  EXPECT_NEAR(d[0], 1.0, 1e-12);
}

TEST(DantzigMatrixCoverage, LdltRemoveHandlesEdgeCases)
{
  const int n = 3;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  Eigen::Matrix3d Ae;
  Ae << 2.0, 0.1, 0.0, 0.1, 2.2, 0.2, 0.0, 0.2, 1.8;
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      A[r * nskip + c] = Ae(r, c);
    }
  }

  std::vector<double> d(n, 0.0);
  dFactorLDLT(A.data(), d.data(), n, nskip);

  std::vector<double*> rows(n);
  for (int r = 0; r < n; ++r) {
    rows[r] = A.data() + r * nskip;
  }
  std::vector<int> perm = {0, 1, 2};
  const size_t tmpSize = dEstimateLDLTRemoveTmpbufSize<double>(n, nskip);
  const size_t tmpCount = (tmpSize + sizeof(double) - 1) / sizeof(double);
  std::vector<double> tmp(tmpCount);
  dLDLTRemove(
      rows.data(), perm.data(), A.data(), d.data(), n, n, 0, nskip, tmp.data());
  EXPECT_TRUE(std::isfinite(d[0]));

  std::vector<double> A2(n * nskip, 0.0);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      A2[r * nskip + c] = Ae(r, c);
    }
  }
  std::vector<double> d2(n, 0.0);
  dFactorLDLT(A2.data(), d2.data(), n, nskip);
  std::vector<double*> rows2(n);
  for (int r = 0; r < n; ++r) {
    rows2[r] = A2.data() + r * nskip;
  }
  dLDLTRemove(
      rows2.data(), perm.data(), A2.data(), d2.data(), n, n, n - 1, nskip);
  EXPECT_TRUE(std::isfinite(d2[0]));
}

TEST(DantzigMatrixCoverage, RemoveRowColHandlesFirstAndLast)
{
  const int n = 3;
  std::vector<double> dense = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  dRemoveRowCol(dense.data(), n, n, n - 1);
  EXPECT_NEAR(dense[0], 1.0, 1e-12);

  std::vector<double> dense2 = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  dRemoveRowCol(dense2.data(), n, n, 0);
  EXPECT_NEAR(dense2[0], 5.0, 1e-12);
}

TEST(DantzigMatrixCoverage, SolveL1AndSolveL1T)
{
  const int n = 5;
  const int nskip = padding(n);
  std::vector<double> L(n * nskip, 0.0);
  Eigen::MatrixXd Le = Eigen::MatrixXd::Zero(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c <= r; ++c) {
      const double value = (r == c) ? 1.0 : 0.1 * static_cast<double>(r + 1);
      L[r * nskip + c] = value;
      Le(r, c) = value;
    }
  }

  std::vector<double> b = {1.0, 2.0, 3.0, 4.0, 5.0};
  const Eigen::VectorXd be = Eigen::VectorXd::Map(b.data(), n);
  const Eigen::VectorXd expected
      = Le.triangularView<Eigen::UnitLower>().solve(be);
  dSolveL1(L.data(), b.data(), n, nskip);
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(b[static_cast<std::size_t>(i)], expected[i], 1e-8);
  }

  std::vector<double> b2 = {1.0, -1.0, 2.0, -2.0, 0.5};
  const Eigen::VectorXd be2 = Eigen::VectorXd::Map(b2.data(), n);
  const Eigen::VectorXd expected2
      = Le.transpose().triangularView<Eigen::UnitUpper>().solve(be2);
  dSolveL1T(L.data(), b2.data(), n, nskip);
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(b2[static_cast<std::size_t>(i)], expected2[i], 1e-8);
  }
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

TEST(MprgpSolverCoverage, SolvesSimpleThreeByThreeProblem)
{
  MprgpSolver solver;
  auto problem = makeStandardProblem(3, 1.8, 0.3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 80;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(MprgpSolverCoverage, SolvesShiftedProblem)
{
  MprgpSolver solver;
  auto problem = makeShiftedStandardProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.2);

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 80;
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

TEST(MprgpSolverCoverage, RejectsNegativeSymmetryTolerance)
{
  MprgpSolver solver;
  MprgpSolver::Parameters params;
  params.symmetryTolerance = -0.5;

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(MprgpSolverCoverage, WarmStartResetsNonFiniteGuess)
{
  MprgpSolver solver;
  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(2, 0.2);
  x[0] = std::numeric_limits<double>::quiet_NaN();

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 50;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_TRUE((x.array() >= 0.0).all());
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

TEST(LemkeSolverCoverage, SolvesWarmStartProblem)
{
  LemkeSolver solver;
  auto problem = makeShiftedStandardProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.05);

  LcpOptions options;
  options.warmStart = true;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(LemkeSolverCoverage, SolvesNonPositiveBWithZeroSolution)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b(2);
  b << -0.5, -0.2;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  LemkeSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_NEAR(x.norm(), 0.0, 1e-12);
}

TEST(LemkeSolverCoverage, SolvesZeroSizeProblem)
{
  LemkeSolver solver;
  auto problem = makeZeroProblem();
  Eigen::VectorXd x;

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(x.size(), 0);
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

TEST(BlockedJacobiSolverCoverage, SolvesAutoBlockPartition)
{
  BlockedJacobiSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 60;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(BlockedJacobiSolverCoverage, SolvesSimpleThreeByThreeProblem)
{
  BlockedJacobiSolver solver;
  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 40;
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

TEST(BgsSolverCoverage, SolvesAutoBlockPartition)
{
  BgsSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 60;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(BgsSolverCoverage, SolvesSimpleThreeByThreeProblem)
{
  BgsSolver solver;
  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 40;
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

TEST(DantzigSolverCoverage, SolvesLargeFrictionCoupledProblem)
{
  DantzigSolver solver;
  auto problem = makeFrictionCoupledProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(problem.b.size(), 0.05);

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 200;
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

TEST(StaggeringSolverCoverage, SolvesSimpleThreeByThreeProblem)
{
  StaggeringSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.1);

  LcpOptions options;
  options.warmStart = true;
  options.relaxation = 1.1;
  options.maxIterations = 12;
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

TEST(ShockPropagationSolverCoverage, SolvesSimpleThreeByThreeProblem)
{
  ShockPropagationSolver solver;
  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 2;
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

TEST(MinimumMapNewtonCoverage, SolvesShiftedStandardProblem)
{
  MinimumMapNewtonSolver solver;
  auto problem = makeShiftedStandardProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.1);

  LcpOptions options;
  options.warmStart = true;
  options.maxIterations = 40;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(LemkeSolverCoverage, SolvesIllConditionedProblem)
{
  LemkeSolver solver;
  auto problem = makeNearSingularProblem(6);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(6);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(LemkeSolverCoverage, SolvesNearSingularFiveByFiveProblem)
{
  LemkeSolver solver;
  auto problem = makeNearSingularProblem(5);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(5);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(LemkeSolverCoverage, SolvesLargerPivotingProblem)
{
  LemkeSolver solver;
  auto problem = makeLemkePivotProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(MprgpSolverCoverage, FallsBackForNonSymmetricProblem)
{
  MprgpSolver solver;
  auto problem = makeNonSymmetricProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 20;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(MprgpSolverCoverage, FallsBackForIndefiniteProblem)
{
  MprgpSolver solver;
  auto problem = makeIndefiniteProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 20;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(MprgpSolverCoverage, ReportsProjectedGradientFailure)
{
  MprgpSolver solver;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Constant(2, 0.1);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  MprgpSolver::Parameters params;
  params.epsilonForDivision = 1.0;

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 5;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Failed);
  EXPECT_FALSE(result.message.empty());
}

TEST(InteriorPointSolverCoverage, SolvesWithClampedParameters)
{
  InteriorPointSolver solver;
  auto problem = makeStandardProblem(6);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(6, 0.2);

  InteriorPointSolver::Parameters params;
  params.sigma = 1.5;
  params.stepScale = 1.5;

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 3;
  options.warmStart = true;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(InteriorPointSolverCoverage, SolvesSimpleThreeByThreeProblem)
{
  InteriorPointSolver solver;
  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 10;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(DirectSolverCoverage, SolvesProblemWithSingularSubsets)
{
  DirectSolver solver;
  auto problem = makeDirectSolverProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(BlockedJacobiSolverCoverage, SolvesMultiBlockFrictionProblem)
{
  BlockedJacobiSolver solver;
  BlockedJacobiSolver::Parameters params;
  params.blockSizes = {3, 3, 3};

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 8;

  auto problem = makeFrictionBlockProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(BgsSolverCoverage, SolvesMultiBlockFrictionProblem)
{
  BgsSolver solver;
  BgsSolver::Parameters params;
  params.blockSizes = {3, 3, 3};

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 8;

  auto problem = makeFrictionBlockProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(JacobiSolverCoverage, HandlesZeroDiagonalEntries)
{
  JacobiSolver solver;
  auto problem = makeZeroDiagonalProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.maxIterations = 12;
  options.relaxation = 1.2;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(RedBlackGaussSeidelCoverage, SolvesFrictionProblemWithRelaxation)
{
  RedBlackGaussSeidelSolver solver;
  auto problem = makeFrictionBlockProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(problem.b.size(), 0.05);

  LcpOptions options;
  options.warmStart = true;
  options.relaxation = 1.3;
  options.maxIterations = 10;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(SymmetricPsorSolverCoverage, SolvesFrictionProblem)
{
  SymmetricPsorSolver solver;
  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(3, 0.05);

  LcpOptions options;
  options.warmStart = true;
  options.relaxation = 1.4;
  options.maxIterations = 10;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(SubspaceMinimizationSolverCoverage, RejectsInvalidPgsIterations)
{
  SubspaceMinimizationSolver solver;
  SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = 0;

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(SubspaceMinimizationSolverCoverage, SolvesBoxedProblem)
{
  SubspaceMinimizationSolver solver;
  SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = 5;
  params.activeSetTolerance = 1e-4;

  LcpOptions options;
  options.customOptions = &params;
  options.relaxation = 1.1;
  options.maxIterations = 10;
  options.warmStart = true;

  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(2, 0.2);
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(FischerBurmeisterNewtonCoverage, SolvesWarmStartProblem)
{
  FischerBurmeisterNewtonSolver solver;
  FischerBurmeisterNewtonSolver::Parameters params;
  params.smoothingEpsilon = 1e-3;

  auto problem = makeStandardProblem(4, 2.0, 0.25);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(4, 0.2);

  LcpOptions options;
  options.customOptions = &params;
  options.warmStart = true;
  options.maxIterations = 8;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(PenalizedFischerBurmeisterNewtonCoverage, SolvesWarmStartProblem)
{
  PenalizedFischerBurmeisterNewtonSolver solver;
  PenalizedFischerBurmeisterNewtonSolver::Parameters params;
  params.lambda = 0.5;
  params.smoothingEpsilon = 1e-3;

  auto problem = makeStandardProblem(4, 2.0, 0.2);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(4, 0.8);

  LcpOptions options;
  options.customOptions = &params;
  options.warmStart = true;
  options.maxIterations = 8;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(ShockPropagationSolverCoverage, RejectsInvalidBlockSizes)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {2, 1};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(4);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(ShockPropagationSolverCoverage, RejectsFrictionIndexOutsideBlock)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 2};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

TEST(ShockPropagationSolverCoverage, RejectsLayerOutOfRange)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {2, 2};
  params.layers = {{0, 2}};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeTwoBlockStandardProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(ShockPropagationSolverCoverage, RejectsMissingLayerCoverage)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {2, 2};
  params.layers = {{0}};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeTwoBlockStandardProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(ShockPropagationSolverCoverage, SolvesLayeredStandardBlocks)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {2, 2};
  params.layers = {{0}, {1}};

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 2;

  auto problem = makeTwoBlockStandardProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(ShockPropagationSolverCoverage, SolvesFrictionBlocks)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {3, 3, 3};

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 2;

  auto problem = makeFrictionBlockProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(StaggeringSolverCoverage, SolvesMultiContactFrictionProblem)
{
  StaggeringSolver solver;
  auto problem = makeFrictionBlockProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());

  LcpOptions options;
  options.maxIterations = 12;
  options.relaxation = 1.2;
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

namespace {

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

LcpProblem makeInfeasibleSingularProblem(int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
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

void expectZeroSolution(const Eigen::VectorXd& x)
{
  EXPECT_LT(x.norm(), 1e-4);
}

} // namespace

TEST(LcpValidationCoverage, ValidateSolutionAcceptsFixedAndInteriorCases)
{
  Eigen::VectorXd x(3);
  Eigen::VectorXd w(3);
  Eigen::VectorXd lo(3);
  Eigen::VectorXd hi(3);

  lo << 0.0, 0.2, -0.5;
  hi << 0.0, 0.8, 0.5;
  x << 0.0, 0.6, 0.1;
  w << 0.0, 0.0, 0.0;

  std::string message;
  EXPECT_TRUE(detail::validateSolution(x, w, lo, hi, 1e-8, &message));

  const double comp = detail::complementarityInfinityNorm(x, w, lo, hi, 1e-8);
  EXPECT_NEAR(comp, 0.0, 1e-12);
}

TEST(DantzigMatrixCoverage, SolveL1AndL1TWithLargerMatrix)
{
  const int n = 8;
  const int nskip = padding(n);
  std::vector<double> L(n * nskip, 0.0);
  Eigen::MatrixXd Le = Eigen::MatrixXd::Identity(n, n);
  for (int r = 1; r < n; ++r) {
    for (int c = 0; c < r; ++c) {
      const double value = 0.02 * static_cast<double>(r + 1);
      L[r * nskip + c] = value;
      Le(r, c) = value;
    }
  }

  std::vector<double> b(n, 0.0);
  for (int i = 0; i < n; ++i) {
    b[static_cast<std::size_t>(i)] = static_cast<double>(i + 1);
  }

  const Eigen::VectorXd be = Eigen::VectorXd::Map(b.data(), n);
  const Eigen::VectorXd expected
      = Le.triangularView<Eigen::UnitLower>().solve(be);
  dSolveL1(L.data(), b.data(), n, nskip);
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(b[static_cast<std::size_t>(i)], expected[i], 1e-8);
  }

  std::vector<double> b2(n, 0.0);
  for (int i = 0; i < n; ++i) {
    b2[static_cast<std::size_t>(i)] = static_cast<double>(i % 3) - 1.0;
  }
  const Eigen::VectorXd be2 = Eigen::VectorXd::Map(b2.data(), n);
  const Eigen::VectorXd expected2
      = Le.transpose().triangularView<Eigen::UnitUpper>().solve(be2);
  dSolveL1T(L.data(), b2.data(), n, nskip);
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR(b2[static_cast<std::size_t>(i)], expected2[i], 1e-8);
  }
}

TEST(DantzigMatrixCoverage, FactorCholeskyWithTmpBuffer)
{
  const int n = 3;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  Eigen::Matrix3d Ae;
  Ae << 4.0, 0.2, 0.1, 0.2, 3.0, 0.3, 0.1, 0.3, 2.5;
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      A[r * nskip + c] = Ae(r, c);
    }
  }

  std::vector<double> tmp(n, 0.0);
  EXPECT_EQ(dFactorCholesky(A.data(), n, tmp.data()), 1);

  std::vector<double> b = {1.0, -2.0, 0.5};
  dSolveCholesky(A.data(), b.data(), n, tmp.data());

  const Eigen::Vector3d expected
      = Ae.ldlt().solve(Eigen::Vector3d(1.0, -2.0, 0.5));
  EXPECT_NEAR(b[0], expected[0], 1e-8);
  EXPECT_NEAR(b[1], expected[1], 1e-8);
  EXPECT_NEAR(b[2], expected[2], 1e-8);
}

TEST(LemkeSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  LemkeSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_EQ(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(2, 2.2, 0.25);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());

  LemkeSolver customSolver;
  LcpOptions options = customSolver.getDefaultOptions();
  options.absoluteTolerance = 1e-8;
  options.validateSolution = true;

  auto zeroProblem = makeZeroRhsProblem(2);
  Eigen::VectorXd xZero = Eigen::VectorXd::Zero(2);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_EQ(zeroResult.status, LcpSolverStatus::Success);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(2);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(2);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(BaraffSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  BaraffSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3, 2.0, 0.3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());

  BaraffSolver customSolver;
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 20;
  options.warmStart = true;
  options.absoluteTolerance = 1e-7;

  auto zeroProblem = makeZeroRhsProblem(3);
  LcpOptions zeroOptions = options;
  zeroOptions.warmStart = false;
  zeroOptions.maxIterations = 50;
  Eigen::VectorXd xZero = Eigen::VectorXd::Zero(3);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, zeroOptions);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  BaraffSolver emptySolver;
  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = emptySolver.solve(
      emptyProblem, xEmpty, emptySolver.getDefaultOptions());
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(DirectSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  DirectSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GE(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(2, 2.0, 0.35);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);

  DirectSolver customSolver;
  LcpOptions options = customSolver.getDefaultOptions();
  options.absoluteTolerance = 1e-8;

  auto zeroProblem = makeZeroRhsProblem(2);
  Eigen::VectorXd xZero = Eigen::VectorXd::Zero(2);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_EQ(zeroResult.status, LcpSolverStatus::Success);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(2);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(2);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_EQ(singularResult.status, LcpSolverStatus::Failed);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(DantzigSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  DantzigSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GE(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3, 2.5, 0.2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());

  DantzigSolver customSolver;
  LcpOptions options = customSolver.getDefaultOptions();
  options.absoluteTolerance = 1e-8;
  options.validateSolution = true;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Zero(3);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_EQ(zeroResult.status, LcpSolverStatus::Success);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(PgsSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  PgsSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(2, 2.0, 0.25);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  PgsSolver customSolver;
  PgsSolver::Parameters params;
  params.randomizeConstraintOrder = true;
  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 25;
  options.relaxation = 1.1;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(2);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(2, 0.2);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(2);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(2);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(BlockedJacobiSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  BlockedJacobiSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  BlockedJacobiSolver customSolver;
  BlockedJacobiSolver::Parameters params;
  params.blockSizes = {2};
  customSolver.setParameters(params);
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 30;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(2);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(2, 0.1);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(2);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(2);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  BlockedJacobiSolver emptySolver;
  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = emptySolver.solve(
      emptyProblem, xEmpty, emptySolver.getDefaultOptions());
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(BgsSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  BgsSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  BgsSolver customSolver;
  BgsSolver::Parameters params;
  params.blockSizes = {2};
  customSolver.setParameters(params);
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 30;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(2);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(2, 0.1);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(2);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(2);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  BgsSolver emptySolver;
  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = emptySolver.solve(
      emptyProblem, xEmpty, emptySolver.getDefaultOptions());
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(JacobiSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  JacobiSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  JacobiSolver customSolver;
  JacobiSolver::Parameters params;
  params.epsilonForDivision = 1e-8;
  customSolver.setParameters(params);
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 40;
  options.relaxation = 1.1;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(3, 0.2);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(RedBlackGaussSeidelCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  RedBlackGaussSeidelSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  RedBlackGaussSeidelSolver customSolver;
  RedBlackGaussSeidelSolver::Parameters params;
  params.epsilonForDivision = 1e-8;
  customSolver.setParameters(params);
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 40;
  options.relaxation = 1.2;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(3, 0.2);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(SymmetricPsorSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  SymmetricPsorSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  SymmetricPsorSolver customSolver;
  SymmetricPsorSolver::Parameters params;
  params.epsilonForDivision = 1e-8;
  customSolver.setParameters(params);
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 40;
  options.relaxation = 1.25;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(3, 0.2);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(NncgSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  NncgSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  NncgSolver customSolver;
  NncgSolver::Parameters params;
  params.pgsIterations = 2;
  params.restartInterval = 2;
  params.restartThreshold = 0.5;
  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 20;
  options.relaxation = 1.0;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(3, 0.05);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(SubspaceMinimizationSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  SubspaceMinimizationSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  SubspaceMinimizationSolver customSolver;
  SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = 3;
  params.activeSetTolerance = 1e-5;
  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 20;
  options.relaxation = 1.0;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(3, 0.05);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(FischerBurmeisterNewtonCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  FischerBurmeisterNewtonSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3, 2.0, 0.2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  FischerBurmeisterNewtonSolver customSolver;
  FischerBurmeisterNewtonSolver::Parameters params;
  params.smoothingEpsilon = 1e-4;
  params.maxLineSearchSteps = 6;
  params.sufficientDecrease = 0.1;
  params.stepReduction = 0.5;
  params.minStep = 1e-4;
  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 10;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(3);
  LcpOptions zeroOptions = options;
  zeroOptions.maxIterations = 25;
  zeroOptions.warmStart = false;
  Eigen::VectorXd xZero = Eigen::VectorXd::Zero(3);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, zeroOptions);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(MinimumMapNewtonCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  MinimumMapNewtonSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3, 2.0, 0.2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  MinimumMapNewtonSolver customSolver;
  MinimumMapNewtonSolver::Parameters params;
  params.maxLineSearchSteps = 6;
  params.sufficientDecrease = 0.1;
  params.stepReduction = 0.5;
  params.minStep = 1e-4;
  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 10;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(3);
  LcpOptions zeroOptions = options;
  zeroOptions.maxIterations = 25;
  zeroOptions.warmStart = false;
  Eigen::VectorXd xZero = Eigen::VectorXd::Zero(3);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, zeroOptions);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(PenalizedFischerBurmeisterNewtonCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  PenalizedFischerBurmeisterNewtonSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3, 2.0, 0.25);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  PenalizedFischerBurmeisterNewtonSolver customSolver;
  PenalizedFischerBurmeisterNewtonSolver::Parameters params;
  params.lambda = 0.7;
  params.smoothingEpsilon = 1e-4;
  params.maxLineSearchSteps = 6;
  params.sufficientDecrease = 0.1;
  params.stepReduction = 0.5;
  params.minStep = 1e-4;
  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 10;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(3, 0.05);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(MprgpSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  MprgpSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  MprgpSolver customSolver;
  MprgpSolver::Parameters params;
  params.epsilonForDivision = 1e-8;
  params.symmetryTolerance = 1e-8;
  params.checkPositiveDefinite = false;
  customSolver.setParameters(params);
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 20;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(3, 0.05);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(InteriorPointSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  InteriorPointSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  InteriorPointSolver customSolver;
  InteriorPointSolver::Parameters params;
  params.sigma = 0.9;
  params.stepScale = 0.8;
  customSolver.setParameters(params);
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 8;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(3);
  LcpOptions zeroOptions = options;
  zeroOptions.maxIterations = 50;
  zeroOptions.warmStart = false;
  Eigen::VectorXd xZero = Eigen::VectorXd::Zero(3);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, zeroOptions);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(StaggeringSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  StaggeringSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto friction = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(friction, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  StaggeringSolver customSolver;
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 6;
  options.relaxation = 1.1;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(3);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(3, 0.05);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(3);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(ShockPropagationSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  ShockPropagationSolver defaultSolver;
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);

  ShockPropagationSolver customSolver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  params.layers = {{0}, {1}};
  customSolver.setParameters(params);
  LcpOptions options = customSolver.getDefaultOptions();
  options.maxIterations = 2;
  options.warmStart = true;

  auto zeroProblem = makeZeroRhsProblem(2);
  Eigen::VectorXd xZero = Eigen::VectorXd::Constant(2, 0.05);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, options);
  EXPECT_NE(zeroResult.status, LcpSolverStatus::InvalidProblem);
  expectZeroSolution(xZero);

  auto singularProblem = makeInfeasibleSingularProblem(2);
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(2);
  auto singularResult = customSolver.solve(singularProblem, xSingular, options);
  EXPECT_NE(singularResult.status, LcpSolverStatus::Success);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}
