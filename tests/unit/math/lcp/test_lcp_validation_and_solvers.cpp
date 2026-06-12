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

#include <dart/all.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
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

LcpProblem makeBoxedActiveProblem(Eigen::VectorXd* expected = nullptr)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
  A.diagonal() << 4.0, 3.0, 2.0;
  Eigen::VectorXd lo(3);
  lo << -1.0, -0.5, -0.25;
  Eigen::VectorXd hi(3);
  hi << 0.75, 1.0, 0.75;
  Eigen::VectorXd target(3);
  target << lo[0], hi[1], 0.2;
  Eigen::VectorXd w(3);
  w << 0.5, -0.5, 0.0;
  Eigen::VectorXd b = A * target - w;
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  if (expected) {
    *expected = target;
  }
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeFrictionProblem(Eigen::VectorXd* expected = nullptr)
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
  if (expected) {
    *expected = target;
  }
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

LcpProblem makeFrictionBlockProblem(
    int contacts = 3, Eigen::VectorXd* expected = nullptr)
{
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

  if (contacts > 1) {
    A(0, 3) = 0.02;
    A(3, 0) = 0.02;
  }
  if (contacts > 2) {
    A(3, 6) = 0.02;
    A(6, 3) = 0.02;
  }

  Eigen::VectorXd target(n);
  for (int c = 0; c < contacts; ++c) {
    const int base = c * blockSize;
    target[base] = 0.4;
    target[base + 1] = 0.1;
    target[base + 2] = -0.05;
  }
  Eigen::VectorXd b = A * target;
  if (expected) {
    *expected = target;
  }

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

Eigen::MatrixXd makeHilbertMatrix(int n, double scale)
{
  Eigen::MatrixXd A(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      A(r, c) = scale / static_cast<double>(r + c + 1);
    }
  }
  return A;
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

using RowMajorMatrixXd
    = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

RowMajorMatrixXd makeRowMajorMatrix(int rows, int cols, double offset)
{
  RowMajorMatrixXd matrix(rows, cols);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const double row = static_cast<double>(r + 1);
      const double col = static_cast<double>(c + 2);
      matrix(r, c) = offset + 0.1 * row - 0.03 * col + 0.005 * row * col;
    }
  }
  return matrix;
}

void expectRowMajorDataNear(
    const std::vector<double>& actual,
    const RowMajorMatrixXd& expected,
    double tolerance)
{
  ASSERT_EQ(actual.size(), static_cast<std::size_t>(expected.size()));
  for (Eigen::Index i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(
        actual[static_cast<std::size_t>(i)], expected.data()[i], tolerance);
  }
}

void expectInvalidParameter(const LcpResult& result, const char* token)
{
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_NE(result.message.find(token), std::string::npos) << result.message;
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

TEST(LcpValidationCoverage, DetectsNonFiniteMatrixEntries)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  A(0, 1) = std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_NE(message.find("Matrix"), std::string::npos);
}

TEST(LcpValidationCoverage, DetectsNonFiniteRhsEntries)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  b[1] = std::numeric_limits<double>::infinity();
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_NE(message.find("Vector b"), std::string::npos);
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

TEST(LcpValidationCoverage, DetectsFrictionWithNegativeHi)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lo(2);
  lo << 0.0, -1.0;
  Eigen::VectorXd hi(2);
  hi << std::numeric_limits<double>::infinity(), -0.5;
  Eigen::VectorXi findex(2);
  findex << -1, 0;
  LcpProblem problem(A, b, lo, hi, findex);

  std::string message;
  EXPECT_FALSE(detail::validateProblem(problem, &message));
  EXPECT_NE(message.find("non-negative"), std::string::npos);
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
  EXPECT_TRUE(
      detail::computeEffectiveBounds(
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
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
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
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
          lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_FALSE(message.empty());

  findex << -1, 1;
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
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
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
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
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
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
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
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
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
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
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
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
  EXPECT_TRUE(detail::validateSolution(x, w, lo, hi, 1e-6, &message));

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
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
          lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_NE(message.find("Invalid friction coefficient"), std::string::npos);
}

TEST(LcpValidationCoverage, ComputeEffectiveBoundsRejectsNegativeMu)
{
  Eigen::VectorXd lo(2);
  lo << 0.0, -1.0;
  Eigen::VectorXd hi(2);
  hi << std::numeric_limits<double>::infinity(), -0.5;
  Eigen::VectorXi findex(2);
  findex << -1, 0;
  Eigen::VectorXd x(2);
  x << 0.2, 0.0;

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  EXPECT_FALSE(
      detail::computeEffectiveBounds(
          lo, hi, findex, x, loEff, hiEff, &message));
  EXPECT_NE(message.find("negative"), std::string::npos);
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

TEST(LcpValidationCoverage, NaturalResidualProjectsToBounds)
{
  Eigen::VectorXd x(1);
  Eigen::VectorXd w(1);
  Eigen::VectorXd lo(1);
  Eigen::VectorXd hi(1);
  x << 2.0;
  w << -1.0;
  lo << 0.0;
  hi << 1.0;

  const double residual = detail::naturalResidualInfinityNorm(x, w, lo, hi);
  EXPECT_NEAR(residual, 1.0, 1e-12);
}

TEST(LcpValidationCoverage, ComplementarityInfinityNormTracksBoundCases)
{
  Eigen::VectorXd x(3);
  Eigen::VectorXd w(3);
  Eigen::VectorXd lo(3);
  Eigen::VectorXd hi(3);
  lo << 0.0, 0.5, 0.0;
  hi << 1.0, 0.5, std::numeric_limits<double>::infinity();
  x << 0.0, 0.5, 0.25;
  w << -0.2, 0.3, 0.4;

  const double comp = detail::complementarityInfinityNorm(x, w, lo, hi, 1e-6);
  EXPECT_NEAR(comp, 0.4, 1e-12);
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

TEST(DantzigMatrixCoverage, ScalarTraitsExposePrecisionSpecificMath)
{
  EXPECT_FLOAT_EQ(ScalarTraits<float>::epsilon(), 1e-7f);
  EXPECT_TRUE(std::isinf(ScalarTraits<float>::inf()));
  EXPECT_FLOAT_EQ(ScalarTraits<float>::sqrt(9.0f), 3.0f);
  EXPECT_FLOAT_EQ(ScalarTraits<float>::abs(-2.0f), 2.0f);
  EXPECT_FLOAT_EQ(ScalarTraits<float>::reciprocal(4.0f), 0.25f);
  EXPECT_FLOAT_EQ(ScalarTraits<float>::reciprocalSqrt(4.0f), 0.5f);
  EXPECT_NEAR(ScalarTraits<float>::sin(0.25f), std::sin(0.25f), 1e-7f);
  EXPECT_NEAR(ScalarTraits<float>::cos(0.25f), std::cos(0.25f), 1e-7f);

  EXPECT_DOUBLE_EQ(ScalarTraits<double>::epsilon(), 1e-14);
  EXPECT_TRUE(std::isinf(ScalarTraits<double>::inf()));
  EXPECT_DOUBLE_EQ(ScalarTraits<double>::sqrt(16.0), 4.0);
  EXPECT_DOUBLE_EQ(ScalarTraits<double>::abs(-3.0), 3.0);
  EXPECT_DOUBLE_EQ(ScalarTraits<double>::reciprocal(8.0), 0.125);
  EXPECT_DOUBLE_EQ(ScalarTraits<double>::reciprocalSqrt(16.0), 0.25);
  EXPECT_NEAR(ScalarTraits<double>::sin(0.5), std::sin(0.5), 1e-15);
  EXPECT_NEAR(ScalarTraits<double>::cos(0.5), std::cos(0.5), 1e-15);
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

TEST(DantzigMatrixCoverage, InvertRejectsNonPositiveDefiniteMatrix)
{
  const int n = 2;
  const int nskip = padding(n);
  std::vector<double> A(n * nskip, 0.0);
  A[0] = 0.0;
  A[1] = 1.0;
  A[nskip] = 1.0;
  A[nskip + 1] = 0.0;

  std::vector<double> Ainv(n * nskip, 0.0);
  EXPECT_EQ(dInvertPDMatrix(A.data(), Ainv.data(), n), 0);
}

TEST(DantzigMatrixCoverage, MultiplyVariantsMatchEigen)
{
  const int p = 2;
  const int q = 3;
  const int r = 2;
  std::vector<double> B = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<double> C = {0.5, -0.2, 1.1, 0.0, -0.4, 0.8};
  std::vector<double> A0(p * r, 0.0);

  dMultiply0(A0.data(), B.data(), C.data(), p, q, r);

  Eigen::Matrix<double, p, q, Eigen::RowMajor> Be;
  Eigen::Matrix<double, q, r, Eigen::RowMajor> Ce;
  std::ranges::copy(B, Be.data());
  std::ranges::copy(C, Ce.data());
  Eigen::Matrix<double, p, r, Eigen::RowMajor> expected0 = Be * Ce;
  for (int i = 0; i < p * r; ++i) {
    EXPECT_NEAR(A0[static_cast<std::size_t>(i)], expected0.data()[i], 1e-12);
  }

  std::vector<double> A1(p * r, 0.0);
  std::vector<double> B1 = {1.0, -0.5, 2.0, 0.25, -1.0, 0.75};
  std::vector<double> C1 = {0.4, -0.2, 0.3, 1.2, -0.1, 0.9};
  dMultiply1(A1.data(), B1.data(), C1.data(), p, q, r);

  Eigen::Matrix<double, q, p, Eigen::RowMajor> Be1;
  Eigen::Matrix<double, q, r, Eigen::RowMajor> Ce1;
  std::ranges::copy(B1, Be1.data());
  std::ranges::copy(C1, Ce1.data());
  Eigen::Matrix<double, p, r, Eigen::RowMajor> expected1
      = Be1.transpose() * Ce1;
  for (int i = 0; i < p * r; ++i) {
    EXPECT_NEAR(A1[static_cast<std::size_t>(i)], expected1.data()[i], 1e-12);
  }

  std::vector<double> A2(p * r, 0.0);
  std::vector<double> B2 = {0.2, 0.4, -0.1, 0.3, 0.5, -0.2};
  std::vector<double> C2 = {1.1, -0.4, 0.6, 0.2, -0.3, 0.8};
  dMultiply2(A2.data(), B2.data(), C2.data(), p, q, r);

  Eigen::Matrix<double, p, q, Eigen::RowMajor> Be2;
  Eigen::Matrix<double, r, q, Eigen::RowMajor> Ce2;
  std::ranges::copy(B2, Be2.data());
  std::ranges::copy(C2, Ce2.data());
  Eigen::Matrix<double, p, r, Eigen::RowMajor> expected2
      = Be2 * Ce2.transpose();
  for (int i = 0; i < p * r; ++i) {
    EXPECT_NEAR(A2[static_cast<std::size_t>(i)], expected2.data()[i], 1e-12);
  }
}

TEST(DantzigMatrixCoverage, MultiplyVariantsUseEigenPathForLargerMatrices)
{
  const int p = 3;
  const int q = MATRIX_MULTIPLY_SIMD_THRESHOLD;
  const int r = MATRIX_MULTIPLY_SIMD_THRESHOLD;

  const RowMajorMatrixXd B0 = makeRowMajorMatrix(p, q, 0.2);
  const RowMajorMatrixXd C0 = makeRowMajorMatrix(q, r, -0.4);
  std::vector<double> A0(static_cast<std::size_t>(p * r), 0.0);
  dMultiply0(A0.data(), B0.data(), C0.data(), p, q, r);
  expectRowMajorDataNear(A0, B0 * C0, 1e-12);

  const RowMajorMatrixXd B1 = makeRowMajorMatrix(q, p, 0.15);
  const RowMajorMatrixXd C1 = makeRowMajorMatrix(q, r, 0.35);
  std::vector<double> A1(static_cast<std::size_t>(p * r), 0.0);
  dMultiply1(A1.data(), B1.data(), C1.data(), p, q, r);
  expectRowMajorDataNear(A1, B1.transpose() * C1, 1e-12);

  const RowMajorMatrixXd B2 = makeRowMajorMatrix(p, q, -0.1);
  const RowMajorMatrixXd C2 = makeRowMajorMatrix(r, q, 0.45);
  std::vector<double> A2(static_cast<std::size_t>(p * r), 0.0);
  dMultiply2(A2.data(), B2.data(), C2.data(), p, q, r);
  expectRowMajorDataNear(A2, B2 * C2.transpose(), 1e-12);
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

TEST(MprgpSolverCoverage, StalledDirectionReportsFiniteResult)
{
  constexpr int n = 12;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n) * 2.0;
  for (int i = 0; i + 1 < n; ++i) {
    A(i, i + 1) = 0.08;
    A(i + 1, i) = 0.08;
  }
  for (int i = 0; i + 2 < n; ++i) {
    A(i, i + 2) = 0.025;
    A(i + 2, i) = 0.025;
  }

  Eigen::VectorXd target(n);
  for (int i = 0; i < n; ++i) {
    target[i]
        = 0.15
          + (0.50 - 0.15) * static_cast<double>(i) / static_cast<double>(n - 1);
  }

  LcpProblem problem(A, A * target);
  MprgpSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  LcpOptions options = LcpOptions::highAccuracy();
  options.maxIterations = 1000;
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(
      result.status == LcpSolverStatus::Success
      || result.status == LcpSolverStatus::MaxIterations)
      << result.message;
  EXPECT_TRUE(std::isfinite(result.residual));
  EXPECT_TRUE(std::isfinite(result.complementarity));
  EXPECT_TRUE(x.array().isFinite().all());
  EXPECT_LT((x - target).lpNorm<Eigen::Infinity>(), 1e-6);
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

TEST(LemkeSolverCoverage, SolvesMultipleSizes)
{
  LemkeSolver solver;
  const std::vector<int> sizes = {1, 3, 5, 10};
  for (const int size : sizes) {
    auto problem = makeStandardProblem(size, 2.0, 0.25);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(size);

    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_TRUE(x.array().isFinite().all());

    const Eigen::VectorXd expected = Eigen::VectorXd::Constant(size, 0.25);
    EXPECT_TRUE(x.isApprox(expected, 1e-6));
  }
}

TEST(StandardStrictInteriorFastPath, PivotAndBarrierSolversUseLinearSolve)
{
  auto problem = makeStandardProblem(4, 3.0, 0.35);
  const Eigen::VectorXd expected = Eigen::VectorXd::Constant(4, 0.35);

  {
    DantzigSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    auto options = solver.getDefaultOptions();
    options.warmStart = false;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    LemkeSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    BaraffSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    InteriorPointSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }
}

TEST(StandardStrictInteriorFastPath, NewtonSolversUseLinearSolve)
{
  auto problem = makeStandardProblem(4, 3.0, 0.35);
  const Eigen::VectorXd expected = Eigen::VectorXd::Constant(4, 0.35);

  {
    BoxedSemiSmoothNewtonSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    auto options = solver.getDefaultOptions();
    options.warmStart = false;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    FischerBurmeisterNewtonSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    PenalizedFischerBurmeisterNewtonSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    MinimumMapNewtonSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }
}

TEST(StandardStrictInteriorFastPath, ProjectionAndBlockSolversUseLinearSolve)
{
  auto problem = makeStandardProblem(4, 3.0, 0.35);
  const Eigen::VectorXd expected = Eigen::VectorXd::Constant(4, 0.35);

  {
    ApgdSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    LcpOptions options = solver.getDefaultOptions();
    options.warmStart = false;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    BgsSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    BlockedJacobiSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    NncgSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    JacobiSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    RedBlackGaussSeidelSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    ShockPropagationSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    SymmetricPsorSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    SubspaceMinimizationSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }
}

TEST(StandardStrictInteriorFastPath, HighOverheadSolversUseLargeLinearSolve)
{
  auto problem = makeStandardProblem(96, 3.0, 0.35);
  const Eigen::VectorXd expected = Eigen::VectorXd::Constant(96, 0.35);

  {
    BgsSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(96);
    LcpOptions options = solver.getDefaultOptions();
    options.maxIterations = 1;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    NncgSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(96);
    LcpOptions options = solver.getDefaultOptions();
    options.maxIterations = 1;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    SymmetricPsorSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(96);
    LcpOptions options = solver.getDefaultOptions();
    options.maxIterations = 1;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    SubspaceMinimizationSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(96);
    LcpOptions options = solver.getDefaultOptions();
    options.maxIterations = 1;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }
}

TEST(StandardStrictInteriorFastPath, OtherSolversUseLinearSolve)
{
  auto problem = makeStandardProblem(4, 3.0, 0.35);
  const Eigen::VectorXd expected = Eigen::VectorXd::Constant(4, 0.35);

  {
    AdmmSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    LcpOptions options = solver.getDefaultOptions();
    options.warmStart = false;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    MprgpSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }

  {
    SapSolver solver;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    LcpOptions options = solver.getDefaultOptions();
    options.warmStart = false;
    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  }
}

TEST(BoxedProjectedActiveSetFastPath, AdmmUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  AdmmSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, DantzigUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  DantzigSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, ApgdUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  ApgdSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, ShockPropagationUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  ShockPropagationSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, NncgUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  NncgSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, BlockedJacobiUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  BlockedJacobiSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, BgsUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  BgsSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, BoxedSemiSmoothNewtonUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  BoxedSemiSmoothNewtonSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, SapUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  SapSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(BoxedProjectedActiveSetFastPath, SubspaceMinimizationUsesLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeBoxedActiveProblem(&expected);
  SubspaceMinimizationSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(FrictionIndexInteriorFastPath, HighOverheadSolversUseLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionProblem(&expected);

  const auto expectFastPath = [&](auto& solver) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
    LcpOptions options = solver.getDefaultOptions();
    options.warmStart = false;
    options.maxIterations = 1;

    const auto result = solver.solve(problem, x, options);
    EXPECT_EQ(result.status, LcpSolverStatus::Success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_TRUE(x.isApprox(expected, 1e-8));
  };

  {
    DantzigSolver solver;
    expectFastPath(solver);
  }
  {
    JacobiSolver solver;
    expectFastPath(solver);
  }
  {
    RedBlackGaussSeidelSolver solver;
    expectFastPath(solver);
  }
  {
    BlockedJacobiSolver solver;
    expectFastPath(solver);
  }
  {
    BgsSolver solver;
    expectFastPath(solver);
  }
  {
    NncgSolver solver;
    expectFastPath(solver);
  }
  {
    SubspaceMinimizationSolver solver;
    expectFastPath(solver);
  }
  {
    ShockPropagationSolver solver;
    expectFastPath(solver);
  }
  {
    SymmetricPsorSolver solver;
    expectFastPath(solver);
  }
  {
    StaggeringSolver solver;
    expectFastPath(solver);
  }
  {
    AdmmSolver solver;
    expectFastPath(solver);
  }
  {
    ApgdSolver solver;
    expectFastPath(solver);
  }
  {
    BoxedSemiSmoothNewtonSolver solver;
    expectFastPath(solver);
  }
}

TEST(FrictionIndexInteriorFastPath, ApgdUsesMediumLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(16, &expected);
  ApgdSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(FrictionIndexInteriorFastPath, SymmetricPsorUsesMediumLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(16, &expected);
  SymmetricPsorSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(FrictionIndexInteriorFastPath, ShockPropagationUsesLargeLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(64, &expected);
  ShockPropagationSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(FrictionIndexInteriorFastPath, JacobiUsesLargeLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(64, &expected);
  JacobiSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(FrictionIndexInteriorFastPath, RedBlackGaussSeidelUsesLargeLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(64, &expected);
  RedBlackGaussSeidelSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(FrictionIndexInteriorFastPath, BgsUsesLargeLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(64, &expected);
  BgsSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(FrictionIndexInteriorFastPath, AdmmUsesMediumLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(16, &expected);
  AdmmSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(FrictionIndexInteriorFastPath, BoxedSemiSmoothNewtonUsesMediumLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(16, &expected);
  BoxedSemiSmoothNewtonSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(
    FrictionIndexInteriorFastPath,
    BoxedSemiSmoothNewtonUsesConfiguredLargeLinearSolve)
{
  Eigen::VectorXd expected;
  auto problem = makeFrictionBlockProblem(64, &expected);
  BoxedSemiSmoothNewtonSolver solver;
  BoxedSemiSmoothNewtonSolver::Parameters params;
  params.maxFrictionIndexExactSolveDimension = static_cast<int>(problem.size());

  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.customOptions = &params;
  options.warmStart = false;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(expected, 1e-8));
}

TEST(
    BoxedSemiSmoothNewtonSolverCoverage,
    WarmStartAcceptsConvergedLineSearchStep)
{
  auto problem = makeStandardProblem(3);
  BoxedSemiSmoothNewtonSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.size());
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = true;
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 1);
  EXPECT_TRUE(
      x.isApprox(Eigen::VectorXd::Constant(problem.size(), 0.25), 1e-8));
}

TEST(LemkeSolverCoverage, MaxIterationsExceeded)
{
  LemkeSolver solver;
  auto problem = makeLemkePivotProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());

  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 1;

  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(LemkeSolverCoverage, ValidationMode)
{
  LemkeSolver solver;
  auto problem = makeNearSingularProblem(5);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());

  LcpOptions options = solver.getDefaultOptions();
  options.validateSolution = true;
  options.absoluteTolerance = 1e-12;
  options.complementarityTolerance = 1e-12;

  const auto result = solver.solve(problem, x, options);
  EXPECT_TRUE(result.validated);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
}

TEST(LemkeSolverCoverage, RejectsInvalidProblem)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  LemkeSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, solver.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
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

TEST(LemkeSolverCoverage, ReportsFailureOnDegeneratePivot)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, 1);
  Eigen::VectorXd b(1);
  b << 1.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(1, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  LemkeSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  LcpOptions options;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Failed);
  EXPECT_FALSE(result.message.empty());
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

TEST(BlockedJacobiSolverCoverage, RejectsFrictionIndexOutsideBlock)
{
  BlockedJacobiSolver solver;
  BlockedJacobiSolver::Parameters params;
  params.blockSizes = {1, 2};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeFrictionProblem();
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

TEST(BlockedJacobiSolverCoverage, ReportsMaxIterationsWhenNotConverged)
{
  BlockedJacobiSolver solver;
  Eigen::MatrixXd A(2, 2);
  A << 2.0, 1.5, 1.0, 2.5;
  Eigen::VectorXd target(2);
  target << 0.0, 0.25;
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpOptions options;
  options.maxIterations = 1;
  options.absoluteTolerance = 1e-12;
  options.relativeTolerance = 1e-12;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::MaxIterations);
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

TEST(BgsSolverCoverage, RejectsFrictionIndexOutsideCustomBlock)
{
  BgsSolver solver;
  BgsSolver::Parameters params;
  params.blockSizes = {1, 2};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeFrictionProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
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

TEST(BgsSolverCoverage, SolvesSingletonBoxedBlocksWithProjection)
{
  BgsSolver solver;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
  A(0, 0) = 2.0;
  A(1, 1) = 4.0;
  Eigen::Vector2d b(3.0, -8.0);
  Eigen::Vector2d lo(0.0, -1.0);
  Eigen::Vector2d hi(1.0, 2.0);
  LcpProblem problem(A, b, lo, hi);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 1;
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(result.succeeded()) << result.message;
  ASSERT_EQ(x.size(), 2);
  EXPECT_NEAR(x[0], 1.0, 1e-12);
  EXPECT_NEAR(x[1], -1.0, 1e-12);
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

TEST(PgsSolverCoverage, RejectsInvalidDivisionEpsilon)
{
  PgsSolver solver;
  PgsSolver::Parameters params;
  params.epsilonForDivision = 0.0;

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  expectInvalidParameter(result, "epsilon_for_division");
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

TEST(DantzigSolverCoverage, EarlyTerminationOnTrivialProblem)
{
  DantzigSolver solver;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(2) * -1.0;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.earlyTermination = true;
  options.validateSolution = false;
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
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

TEST(ShockPropagationSolverCoverage, RejectsEmptyLayer)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {1, 1};
  params.layers = {{}, {1}};

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
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(x.isApprox(Eigen::VectorXd::Constant(3, 0.25), 1e-8));
}

TEST(ShockPropagationSolverCoverage, SolvesFeasibleBoxedBlockDirectly)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {3};
  params.layers = {{0}};

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3) * 2.0;
  Eigen::Vector3d target(0.25, 0.5, 0.75);
  Eigen::VectorXd b = A * target;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd hi = Eigen::VectorXd::Ones(3);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
  LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  LcpOptions options;
  options.customOptions = &params;
  options.maxIterations = 1;
  const auto result = solver.solve(problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Success) << result.message;
  ASSERT_EQ(x.size(), target.size());
  EXPECT_TRUE(x.isApprox(target, 1e-12));
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

TEST(LemkeSolverCoverage, ExercisesIllConditionedProblem)
{
  LemkeSolver solver;
  const int n = 6;
  const std::vector<double> scales = {1e2, 1e4, 1e6, 1e8};

  Eigen::VectorXd target(n);
  for (int i = 0; i < n; ++i) {
    target[i] = 0.2 + 0.03 * static_cast<double>(i);
  }

  for (const double scale : scales) {
    Eigen::MatrixXd A = makeHilbertMatrix(n, scale);
    Eigen::VectorXd b = A * target;
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hi
        = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
    LcpProblem problem(A, b, lo, hi, findex);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    const auto result = solver.solve(problem, x, solver.getDefaultOptions());
    // Ill-conditioned problems may or may not succeed; just exercise the path
    EXPECT_TRUE(x.array().isFinite().all());
  }
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

TEST(MprgpSolverCoverage, ReportsProjectedGradientStallWithFiniteMetrics)
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
  EXPECT_EQ(result.status, LcpSolverStatus::MaxIterations);
  EXPECT_TRUE(std::isfinite(result.residual));
  EXPECT_TRUE(std::isfinite(result.complementarity));
  EXPECT_FALSE(result.message.empty());
}

TEST(InteriorPointSolverCoverage, RejectsInvalidParameters)
{
  auto solve = [](InteriorPointSolver::Parameters params) {
    InteriorPointSolver solver;
    LcpOptions options;
    options.customOptions = &params;

    auto problem = makeStandardProblem(2);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    return solver.solve(problem, x, options);
  };

  InteriorPointSolver::Parameters params;
  params.sigma = 1.5;
  expectInvalidParameter(solve(params), "sigma");

  params = {};
  params.stepScale = 1.5;
  expectInvalidParameter(solve(params), "step_scale");
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

TEST(InteriorPointSolverCoverage, FallsBackForBoxedProblem)
{
  InteriorPointSolver solver;
  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);

  LcpOptions options;
  options.maxIterations = 5;
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

TEST(
    ShockPropagationSolverCoverage, RejectsInvalidBoxedBlockSizesBeforeFastPath)
{
  ShockPropagationSolver solver;
  ShockPropagationSolver::Parameters params;
  params.blockSizes = {2, 1};

  LcpOptions options;
  options.customOptions = &params;

  auto problem = makeBoxedProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
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

TEST(ShockPropagationSolverCoverage, GroupsFrictionIndicesWithDefaults)
{
  ShockPropagationSolver solver;
  auto problem = makeFrictionBlockProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());

  LcpOptions options;
  options.maxIterations = 2;
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

LcpProblem makeFixedZeroBoundProblem()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(1, 1);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(1);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(1);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(1, -1);
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

  w << 1.0, 0.0, 0.0;
  EXPECT_TRUE(detail::validateSolution(x, w, lo, hi, 1e-8, &message));

  const double comp = detail::complementarityInfinityNorm(x, w, lo, hi, 1e-8);
  EXPECT_NEAR(comp, 0.0, 1e-12);
}

TEST(LcpValidationCoverage, ComplementarityNormTracksBoundCases)
{
  Eigen::VectorXd x(3);
  Eigen::VectorXd w(3);
  Eigen::VectorXd lo(3);
  Eigen::VectorXd hi(3);

  lo << 0.0, 0.0, -1.0;
  hi << std::numeric_limits<double>::infinity(), 1.0, 1.0;
  x << 0.0, 1.0, 0.2;
  w << -0.3, 0.4, 0.3;

  const double comp = detail::complementarityInfinityNorm(x, w, lo, hi, 1e-8);
  EXPECT_GT(comp, 0.0);
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
  EXPECT_EQ(defaultSolver.getName(), "Direct");
  EXPECT_EQ(defaultSolver.getCategory(), "Pivoting");
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
  EXPECT_EQ(defaultSolver.getName(), "Dantzig");
  EXPECT_EQ(defaultSolver.getCategory(), "Pivoting");
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
  EXPECT_EQ(defaultSolver.getName(), "Pgs");
  EXPECT_EQ(defaultSolver.getCategory(), "Projection");
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
  EXPECT_EQ(defaultSolver.getName(), "Jacobi");
  EXPECT_EQ(defaultSolver.getCategory(), "Projection");
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
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().epsilonForDivision,
      params.epsilonForDivision);
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
  EXPECT_EQ(defaultSolver.getName(), "RedBlackGaussSeidel");
  EXPECT_EQ(defaultSolver.getCategory(), "Projection");
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
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().epsilonForDivision,
      params.epsilonForDivision);
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
  EXPECT_EQ(defaultSolver.getName(), "NNCG");
  EXPECT_EQ(defaultSolver.getCategory(), "Projection");
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

TEST(ApgdSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  ApgdSolver defaultSolver;
  EXPECT_EQ(defaultSolver.getName(), "Apgd");
  EXPECT_EQ(defaultSolver.getCategory(), "Projection");
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());

  ApgdSolver customSolver;
  ApgdSolver::Parameters params;
  params.epsilonForDivision = 1e-8;
  params.adaptiveRestart = false;
  params.restartCheckInterval = 0;
  customSolver.setParameters(params);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().epsilonForDivision,
      params.epsilonForDivision);
  EXPECT_EQ(
      customSolver.getParameters().adaptiveRestart, params.adaptiveRestart);
  EXPECT_EQ(
      customSolver.getParameters().restartCheckInterval,
      params.restartCheckInterval);

  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 25;
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

TEST(ApgdSolverCoverage, RejectsInvalidParameters)
{
  auto solve = [](ApgdSolver::Parameters params) {
    ApgdSolver solver;
    LcpOptions options = solver.getDefaultOptions();
    options.customOptions = &params;
    auto problem = makeStandardProblem(2);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    return solver.solve(problem, x, options);
  };

  ApgdSolver::Parameters params;
  params.epsilonForDivision = 0.0;
  expectInvalidParameter(solve(params), "epsilon_for_division");

  params = {};
  params.restartCheckInterval = -1;
  expectInvalidParameter(solve(params), "restart_check_interval");
}

TEST(TgsSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  TgsSolver defaultSolver;
  EXPECT_EQ(defaultSolver.getName(), "Tgs");
  EXPECT_EQ(defaultSolver.getCategory(), "Projection");
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());

  TgsSolver customSolver;
  TgsSolver::Parameters params;
  params.epsilonForDivision = 1e-8;
  customSolver.setParameters(params);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().epsilonForDivision,
      params.epsilonForDivision);

  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 25;
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

TEST(TgsSolverCoverage, RejectsInvalidDivisionEpsilon)
{
  TgsSolver solver;
  TgsSolver::Parameters params;
  params.epsilonForDivision = 0.0;

  LcpOptions options = solver.getDefaultOptions();
  options.customOptions = &params;

  auto problem = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto result = solver.solve(problem, x, options);
  expectInvalidParameter(result, "epsilon_for_division");
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
  customSolver.setParameters(params);
  EXPECT_EQ(customSolver.getParameters().pgsIterations, params.pgsIterations);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().activeSetTolerance,
      params.activeSetTolerance);
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

TEST(MinimumMapNewtonCoverage, RejectsInvalidParameters)
{
  auto solve = [](MinimumMapNewtonSolver::Parameters params) {
    MinimumMapNewtonSolver solver;
    LcpOptions options = solver.getDefaultOptions();
    options.customOptions = &params;
    auto problem = makeStandardProblem(2);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    return solver.solve(problem, x, options);
  };

  MinimumMapNewtonSolver::Parameters params;
  params.maxLineSearchSteps = 0;
  expectInvalidParameter(solve(params), "max_line_search_steps");
  params = {};
  params.stepReduction = 1.0;
  expectInvalidParameter(solve(params), "step_reduction");
  params = {};
  params.sufficientDecrease = 1.0;
  expectInvalidParameter(solve(params), "sufficient_decrease");
  params = {};
  params.minStep = 0.0;
  expectInvalidParameter(solve(params), "min_step");
  params = {};
  params.maxGradientDescentWarmStartSteps = -1;
  expectInvalidParameter(
      solve(params), "max_gradient_descent_warm_start_steps");
  params = {};
  params.maxGradientDescentLineSearchSteps = 0;
  expectInvalidParameter(
      solve(params), "max_gradient_descent_line_search_steps");
  params = {};
  params.gradientDescentStepReduction = 1.0;
  expectInvalidParameter(solve(params), "gradient_descent_step_reduction");
  params = {};
  params.gradientDescentSufficientDecrease = 1.0;
  expectInvalidParameter(solve(params), "gradient_descent_sufficient_decrease");
  params = {};
  params.gradientDescentMinStep = 0.0;
  expectInvalidParameter(solve(params), "gradient_descent_min_step");
  params = {};
  params.maxPgsWarmStartIterations = -1;
  expectInvalidParameter(solve(params), "max_pgs_warm_start_iterations");
  params = {};
  params.pgsWarmStartRelaxation = 0.0;
  expectInvalidParameter(solve(params), "pgs_warm_start_relaxation");
}

TEST(FischerBurmeisterNewtonCoverage, RejectsInvalidParameters)
{
  auto solve = [](FischerBurmeisterNewtonSolver::Parameters params) {
    FischerBurmeisterNewtonSolver solver;
    LcpOptions options = solver.getDefaultOptions();
    options.customOptions = &params;
    auto problem = makeStandardProblem(2);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    return solver.solve(problem, x, options);
  };

  FischerBurmeisterNewtonSolver::Parameters params;
  params.smoothingEpsilon = 0.0;
  expectInvalidParameter(solve(params), "smoothing_epsilon");
  params = {};
  params.maxLineSearchSteps = 0;
  expectInvalidParameter(solve(params), "max_line_search_steps");
  params = {};
  params.stepReduction = 1.0;
  expectInvalidParameter(solve(params), "step_reduction");
  params = {};
  params.sufficientDecrease = 1.0;
  expectInvalidParameter(solve(params), "sufficient_decrease");
  params = {};
  params.minStep = 0.0;
  expectInvalidParameter(solve(params), "min_step");
  params = {};
  params.maxGradientDescentWarmStartSteps = -1;
  expectInvalidParameter(
      solve(params), "max_gradient_descent_warm_start_steps");
  params = {};
  params.maxGradientDescentLineSearchSteps = 0;
  expectInvalidParameter(
      solve(params), "max_gradient_descent_line_search_steps");
  params = {};
  params.gradientDescentStepReduction = 1.0;
  expectInvalidParameter(solve(params), "gradient_descent_step_reduction");
  params = {};
  params.gradientDescentSufficientDecrease = 1.0;
  expectInvalidParameter(solve(params), "gradient_descent_sufficient_decrease");
  params = {};
  params.gradientDescentMinStep = 0.0;
  expectInvalidParameter(solve(params), "gradient_descent_min_step");
  params = {};
  params.maxPgsWarmStartIterations = -1;
  expectInvalidParameter(solve(params), "max_pgs_warm_start_iterations");
  params = {};
  params.pgsWarmStartRelaxation = 0.0;
  expectInvalidParameter(solve(params), "pgs_warm_start_relaxation");
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
  customSolver.setParameters(params);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().smoothingEpsilon, params.smoothingEpsilon);
  EXPECT_EQ(
      customSolver.getParameters().maxLineSearchSteps,
      params.maxLineSearchSteps);
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
  customSolver.setParameters(params);
  EXPECT_EQ(
      customSolver.getParameters().maxLineSearchSteps,
      params.maxLineSearchSteps);
  EXPECT_DOUBLE_EQ(customSolver.getParameters().minStep, params.minStep);
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

TEST(PenalizedFischerBurmeisterNewtonCoverage, RejectsInvalidParameters)
{
  auto solve = [](PenalizedFischerBurmeisterNewtonSolver::Parameters params) {
    PenalizedFischerBurmeisterNewtonSolver solver;
    LcpOptions options = solver.getDefaultOptions();
    options.customOptions = &params;
    auto problem = makeStandardProblem(2);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    return solver.solve(problem, x, options);
  };

  PenalizedFischerBurmeisterNewtonSolver::Parameters params;
  params.smoothingEpsilon = 0.0;
  expectInvalidParameter(solve(params), "smoothing_epsilon");
  params = {};
  params.lambda = 1.5;
  expectInvalidParameter(solve(params), "lambda");
  params = {};
  params.maxLineSearchSteps = 0;
  expectInvalidParameter(solve(params), "max_line_search_steps");
  params = {};
  params.stepReduction = 1.0;
  expectInvalidParameter(solve(params), "step_reduction");
  params = {};
  params.sufficientDecrease = 1.0;
  expectInvalidParameter(solve(params), "sufficient_decrease");
  params = {};
  params.minStep = 0.0;
  expectInvalidParameter(solve(params), "min_step");
  params = {};
  params.maxGradientDescentWarmStartSteps = -1;
  expectInvalidParameter(
      solve(params), "max_gradient_descent_warm_start_steps");
  params = {};
  params.maxGradientDescentLineSearchSteps = 0;
  expectInvalidParameter(
      solve(params), "max_gradient_descent_line_search_steps");
  params = {};
  params.gradientDescentStepReduction = 1.0;
  expectInvalidParameter(solve(params), "gradient_descent_step_reduction");
  params = {};
  params.gradientDescentSufficientDecrease = 1.0;
  expectInvalidParameter(solve(params), "gradient_descent_sufficient_decrease");
  params = {};
  params.gradientDescentMinStep = 0.0;
  expectInvalidParameter(solve(params), "gradient_descent_min_step");
  params = {};
  params.maxPgsWarmStartIterations = -1;
  expectInvalidParameter(solve(params), "max_pgs_warm_start_iterations");
  params = {};
  params.pgsWarmStartRelaxation = 0.0;
  expectInvalidParameter(solve(params), "pgs_warm_start_relaxation");
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
  customSolver.setParameters(params);
  EXPECT_DOUBLE_EQ(customSolver.getParameters().lambda, params.lambda);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().smoothingEpsilon, params.smoothingEpsilon);
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

TEST(BoxedSemiSmoothNewtonSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  BoxedSemiSmoothNewtonSolver defaultSolver;
  EXPECT_EQ(defaultSolver.getName(), "BoxedSemiSmoothNewton");
  EXPECT_EQ(defaultSolver.getCategory(), "Newton");
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_GT(defaults.maxIterations, 0);
  EXPECT_TRUE(defaults.validateSolution);
  EXPECT_TRUE(defaults.warmStart);

  auto standard = makeStandardProblem(3, 2.0, 0.25);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());

  BoxedSemiSmoothNewtonSolver customSolver;
  BoxedSemiSmoothNewtonSolver::Parameters params;
  params.maxLineSearchSteps = 4;
  params.stepReduction = 0.4;
  params.sufficientDecrease = 1e-3;
  params.minStep = 1e-6;
  params.jacobianRegularization = 1e-7;
  params.maxPgsWarmStartIterations = 3;
  params.pgsWarmStartRelaxation = 0.9;
  params.maxFrictionIndexExactSolveDimension = 96;
  customSolver.setParameters(params);
  EXPECT_EQ(
      customSolver.getParameters().maxLineSearchSteps,
      params.maxLineSearchSteps);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().stepReduction, params.stepReduction);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().sufficientDecrease,
      params.sufficientDecrease);
  EXPECT_DOUBLE_EQ(customSolver.getParameters().minStep, params.minStep);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().jacobianRegularization,
      params.jacobianRegularization);
  EXPECT_EQ(
      customSolver.getParameters().maxPgsWarmStartIterations,
      params.maxPgsWarmStartIterations);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().pgsWarmStartRelaxation,
      params.pgsWarmStartRelaxation);
  EXPECT_EQ(
      customSolver.getParameters().maxFrictionIndexExactSolveDimension,
      params.maxFrictionIndexExactSolveDimension);

  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 5;
  options.validateSolution = false;
  options.warmStart = true;

  auto boxedProblem = makeBoxedProblem();
  Eigen::VectorXd xBoxed(2);
  xBoxed << -1.0, 2.0;
  auto boxedResult = customSolver.solve(boxedProblem, xBoxed, options);
  EXPECT_NE(boxedResult.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(xBoxed.array().isFinite().all());
  EXPECT_GE(xBoxed[0], boxedProblem.lo[0] - 1e-8);
  EXPECT_LE(xBoxed[1], boxedProblem.hi[1] + 1e-8);

  options.warmStart = false;
  auto frictionProblem = makeFrictionProblem();
  Eigen::VectorXd xFriction = Eigen::VectorXd::Constant(3, 0.25);
  auto frictionResult = customSolver.solve(frictionProblem, xFriction, options);
  EXPECT_NE(frictionResult.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(xFriction.array().isFinite().all());

  auto invalidProblem = makeStandardProblem(2);
  invalidProblem.lo[0] = 1.0;
  invalidProblem.hi[0] = 0.0;
  Eigen::VectorXd xInvalid = Eigen::VectorXd::Zero(2);
  auto invalidResult = customSolver.solve(invalidProblem, xInvalid, options);
  EXPECT_EQ(invalidResult.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(invalidResult.message.empty());

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(BoxedSemiSmoothNewtonSolverCoverage, ParameterAndValidationBranches)
{
  BoxedSemiSmoothNewtonSolver solver;

  BoxedSemiSmoothNewtonSolver::Parameters invalidParams;
  invalidParams.maxLineSearchSteps = 0;
  LcpOptions invalidOptions = solver.getDefaultOptions();
  invalidOptions.customOptions = &invalidParams;
  invalidOptions.maxIterations = 1;
  invalidOptions.validateSolution = false;

  auto standard = makeStandardProblem(2);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto invalidResult = solver.solve(standard, x, invalidOptions);
  EXPECT_EQ(invalidResult.status, LcpSolverStatus::InvalidProblem);
  EXPECT_NE(
      invalidResult.message.find("max_line_search_steps"), std::string::npos);

  invalidParams = {};
  invalidParams.maxPgsWarmStartIterations = -1;
  invalidOptions.customOptions = &invalidParams;
  auto invalidWarmStartIterations = solver.solve(standard, x, invalidOptions);
  EXPECT_EQ(invalidWarmStartIterations.status, LcpSolverStatus::InvalidProblem);
  EXPECT_NE(
      invalidWarmStartIterations.message.find("max_pgs_warm_start_iterations"),
      std::string::npos);

  invalidParams = {};
  invalidParams.pgsWarmStartRelaxation = 0.0;
  invalidOptions.customOptions = &invalidParams;
  auto invalidWarmStartRelaxation = solver.solve(standard, x, invalidOptions);
  EXPECT_EQ(invalidWarmStartRelaxation.status, LcpSolverStatus::InvalidProblem);
  EXPECT_NE(
      invalidWarmStartRelaxation.message.find("pgs_warm_start_relaxation"),
      std::string::npos);

  invalidParams = {};
  invalidParams.maxFrictionIndexExactSolveDimension = -1;
  invalidOptions.customOptions = &invalidParams;
  auto invalidExactSolveDimension = solver.solve(standard, x, invalidOptions);
  EXPECT_EQ(invalidExactSolveDimension.status, LcpSolverStatus::InvalidProblem);
  EXPECT_NE(
      invalidExactSolveDimension.message.find(
          "max_friction_index_exact_solve_dimension"),
      std::string::npos);

  LcpOptions validationOptions = solver.getDefaultOptions();
  validationOptions.validateSolution = true;
  auto fixedProblem = makeFixedZeroBoundProblem();
  Eigen::VectorXd xFixed = Eigen::VectorXd::Zero(1);
  const auto validationResult
      = solver.solve(fixedProblem, xFixed, validationOptions);
  EXPECT_EQ(validationResult.status, LcpSolverStatus::Success);
  EXPECT_TRUE(validationResult.validated);
  EXPECT_TRUE(validationResult.message.empty());
  EXPECT_NEAR(xFixed[0], 0.0, 1e-12);
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
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().epsilonForDivision,
      params.epsilonForDivision);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().symmetryTolerance, params.symmetryTolerance);
  EXPECT_EQ(
      customSolver.getParameters().checkPositiveDefinite,
      params.checkPositiveDefinite);
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
  EXPECT_DOUBLE_EQ(customSolver.getParameters().sigma, params.sigma);
  EXPECT_DOUBLE_EQ(customSolver.getParameters().stepScale, params.stepScale);
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

TEST(AdmmSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  AdmmSolver defaultSolver;
  EXPECT_EQ(defaultSolver.getName(), "Admm");
  EXPECT_EQ(defaultSolver.getCategory(), "Other");
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_EQ(defaults.maxIterations, 200);
  EXPECT_TRUE(defaults.validateSolution);
  EXPECT_TRUE(defaults.warmStart);
  EXPECT_DOUBLE_EQ(defaultSolver.getParameters().rhoInit, 4.0);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());

  AdmmSolver customSolver;
  AdmmSolver::Parameters params;
  params.rhoInit = 0.6;
  params.muProx = 0.2;
  params.adaptiveRhoTolerance = 1.2;
  params.adaptiveRho = false;
  customSolver.setParameters(params);
  EXPECT_DOUBLE_EQ(customSolver.getParameters().rhoInit, params.rhoInit);
  EXPECT_DOUBLE_EQ(customSolver.getParameters().muProx, params.muProx);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().adaptiveRhoTolerance,
      params.adaptiveRhoTolerance);
  EXPECT_EQ(customSolver.getParameters().adaptiveRho, params.adaptiveRho);

  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 1;
  options.absoluteTolerance = 1e-12;
  options.validateSolution = false;
  options.warmStart = false;

  auto frictionProblem = makeFrictionProblem();
  Eigen::VectorXd xWarm = Eigen::VectorXd::Constant(3, 0.25);
  auto maxIterResult = customSolver.solve(frictionProblem, xWarm, options);
  EXPECT_EQ(maxIterResult.status, LcpSolverStatus::MaxIterations);
  EXPECT_EQ(maxIterResult.iterations, 1);
  EXPECT_TRUE(xWarm.array().isFinite().all());

  auto invalidProblem = makeStandardProblem(2);
  invalidProblem.lo[0] = 1.0;
  invalidProblem.hi[0] = 0.0;
  Eigen::VectorXd xInvalid = Eigen::VectorXd::Zero(2);
  auto invalidResult = customSolver.solve(invalidProblem, xInvalid, options);
  EXPECT_EQ(invalidResult.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(invalidResult.message.empty());

  AdmmSolver::Parameters singularParams;
  singularParams.rhoInit = 1e-9;
  singularParams.muProx = 0.0;
  singularParams.adaptiveRho = false;
  LcpOptions singularOptions = customSolver.getDefaultOptions();
  singularOptions.customOptions = &singularParams;
  singularOptions.validateSolution = false;
  auto singularProblem = makeIndefiniteProblem();
  Eigen::VectorXd xSingular = Eigen::VectorXd::Zero(3);
  auto singularResult
      = customSolver.solve(singularProblem, xSingular, singularOptions);
  EXPECT_EQ(singularResult.status, LcpSolverStatus::NumericalError);
  EXPECT_FALSE(singularResult.message.empty());

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(AdmmSolverCoverage, ValidatesFixedBoundSolutionAfterConvergence)
{
  AdmmSolver solver;
  AdmmSolver::Parameters params;
  params.adaptiveRhoTolerance = 1.2;

  LcpOptions options = solver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 80;
  options.absoluteTolerance = 1e-8;
  options.validateSolution = true;

  auto problem = makeFixedZeroBoundProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::Success);
  EXPECT_TRUE(result.validated);
  EXPECT_TRUE(result.message.empty());
  EXPECT_NEAR(x[0], 0.0, 1e-12);
}

TEST(SapSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  SapSolver defaultSolver;
  EXPECT_EQ(defaultSolver.getName(), "Sap");
  EXPECT_EQ(defaultSolver.getCategory(), "Other");
  const auto defaults = defaultSolver.getDefaultOptions();
  EXPECT_EQ(defaults.maxIterations, 50);
  EXPECT_FALSE(defaults.validateSolution);
  EXPECT_TRUE(defaults.warmStart);

  auto standard = makeStandardProblem(3);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  auto result = defaultSolver.solve(standard, x, defaults);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());

  SapSolver customSolver;
  SapSolver::Parameters params;
  params.regularization = 1e-3;
  params.armijosParameter = 1e-3;
  params.backtrackingFactor = 0.25;
  params.maxLineSearchIterations = 3;
  customSolver.setParameters(params);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().regularization, params.regularization);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().armijosParameter, params.armijosParameter);
  EXPECT_DOUBLE_EQ(
      customSolver.getParameters().backtrackingFactor,
      params.backtrackingFactor);
  EXPECT_EQ(
      customSolver.getParameters().maxLineSearchIterations,
      params.maxLineSearchIterations);

  LcpOptions options = customSolver.getDefaultOptions();
  options.customOptions = &params;
  options.maxIterations = 3;
  options.validateSolution = false;
  options.warmStart = false;

  auto frictionProblem = makeFrictionProblem();
  Eigen::VectorXd xWarm = Eigen::VectorXd::Constant(3, 0.25);
  auto customResult = customSolver.solve(frictionProblem, xWarm, options);
  EXPECT_NE(customResult.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(xWarm.array().isFinite().all());

  auto invalidProblem = makeStandardProblem(2);
  invalidProblem.lo[0] = 1.0;
  invalidProblem.hi[0] = 0.0;
  Eigen::VectorXd xInvalid = Eigen::VectorXd::Zero(2);
  auto invalidResult = customSolver.solve(invalidProblem, xInvalid, options);
  EXPECT_EQ(invalidResult.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(invalidResult.message.empty());

  LcpOptions validationOptions = customSolver.getDefaultOptions();
  validationOptions.validateSolution = true;
  validationOptions.maxIterations = 5;
  auto zeroProblem = makeZeroRhsProblem(2);
  Eigen::VectorXd xZero = Eigen::VectorXd::Zero(2);
  auto zeroResult = customSolver.solve(zeroProblem, xZero, validationOptions);
  EXPECT_EQ(zeroResult.status, LcpSolverStatus::Success);
  EXPECT_TRUE(zeroResult.validated);
  expectZeroSolution(xZero);

  auto emptyProblem = makeZeroProblem();
  Eigen::VectorXd xEmpty;
  auto emptyResult = customSolver.solve(emptyProblem, xEmpty, options);
  EXPECT_EQ(emptyResult.status, LcpSolverStatus::Success);
  EXPECT_EQ(xEmpty.size(), 0);
}

TEST(SapSolverCoverage, RegularizesIndefiniteHessian)
{
  SapSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 1;
  options.validateSolution = false;

  auto problem = makeIndefiniteProblem();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  const auto result = solver.solve(problem, x, options);
  EXPECT_NE(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_TRUE(x.array().isFinite().all());
}

TEST(StaggeringSolverCoverage, DefaultCustomZeroRhsAndEdgeCases)
{
  StaggeringSolver defaultSolver;
  EXPECT_EQ(defaultSolver.getName(), "Staggering");
  EXPECT_EQ(defaultSolver.getCategory(), "Other");
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
