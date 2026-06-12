/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include "dart/math/NonNegativeLeastSquares.hpp"
#include "dart/math/Random.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>
#include <vector>

using dart::math::solveNonNegativeLeastSquares;

namespace {

//==============================================================================
// Brute-force reference: enumerate every active set, solve the unconstrained
// least squares problem on the passive columns, and keep the best feasible
// solution. Only usable for small column counts.
Eigen::VectorXd bruteForceNnls(
    const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  const int n = static_cast<int>(A.cols());
  Eigen::VectorXd best = Eigen::VectorXd::Zero(n);
  double bestResidual = b.squaredNorm();

  for (int mask = 1; mask < (1 << n); ++mask) {
    std::vector<int> cols;
    for (int j = 0; j < n; ++j) {
      if (mask & (1 << j)) {
        cols.push_back(j);
      }
    }

    Eigen::MatrixXd subA(A.rows(), cols.size());
    for (std::size_t k = 0; k < cols.size(); ++k) {
      subA.col(k) = A.col(cols[k]);
    }

    const Eigen::VectorXd subX = subA.colPivHouseholderQr().solve(b);
    if ((subX.array() < 0.0).any()) {
      continue;
    }

    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    for (std::size_t k = 0; k < cols.size(); ++k) {
      x[cols[k]] = subX[k];
    }

    const double residual = (A * x - b).squaredNorm();
    if (residual < bestResidual - 1e-12) {
      bestResidual = residual;
      best = x;
    }
  }

  return best;
}

} // namespace

//==============================================================================
TEST(NonNegativeLeastSquares, SolvesIdentitySystem)
{
  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  const Eigen::Vector3d b(1.0, 2.0, 3.0);

  Eigen::VectorXd x;
  EXPECT_TRUE(solveNonNegativeLeastSquares(A, b, x));

  ASSERT_EQ(x.size(), 3);
  EXPECT_TRUE(x.isApprox(b, 1e-12));
}

//==============================================================================
TEST(NonNegativeLeastSquares, MatchesUnconstrainedWhenInterior)
{
  dart::math::Random::setSeed(7);

  for (int trial = 0; trial < 20; ++trial) {
    const Eigen::MatrixXd A
        = Eigen::MatrixXd::Random(6, 4) + 2.0 * Eigen::MatrixXd::Identity(6, 4);
    const Eigen::VectorXd xTrue
        = Eigen::VectorXd::Random(4).cwiseAbs().array() + 0.1;
    const Eigen::VectorXd b = A * xTrue;

    Eigen::VectorXd x;
    ASSERT_TRUE(solveNonNegativeLeastSquares(A, b, x));
    EXPECT_TRUE(x.isApprox(xTrue, 1e-8))
        << "trial " << trial << "\nexpected: " << xTrue.transpose()
        << "\nactual:   " << x.transpose();
  }
}

//==============================================================================
TEST(NonNegativeLeastSquares, ClampsNegativeUnconstrainedSolution)
{
  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::Vector2d b(1.0, -1.0);

  Eigen::VectorXd x;
  EXPECT_TRUE(solveNonNegativeLeastSquares(A, b, x));

  ASSERT_EQ(x.size(), 2);
  EXPECT_NEAR(x[0], 1.0, 1e-12);
  EXPECT_NEAR(x[1], 0.0, 1e-12);
}

//==============================================================================
TEST(NonNegativeLeastSquares, MatchesBruteForceOnRandomSmallProblems)
{
  std::srand(42);

  for (int trial = 0; trial < 50; ++trial) {
    const Eigen::MatrixXd A = Eigen::MatrixXd::Random(4, 3);
    const Eigen::VectorXd b = Eigen::VectorXd::Random(4);

    Eigen::VectorXd x;
    ASSERT_TRUE(solveNonNegativeLeastSquares(A, b, x)) << "trial " << trial;

    const Eigen::VectorXd expected = bruteForceNnls(A, b);
    const double residual = (A * x - b).squaredNorm();
    const double expectedResidual = (A * expected - b).squaredNorm();

    EXPECT_GE(x.minCoeff(), -1e-12) << "trial " << trial;
    EXPECT_NEAR(residual, expectedResidual, 1e-8) << "trial " << trial;
  }
}

//==============================================================================
TEST(NonNegativeLeastSquares, HandlesDuplicateColumns)
{
  Eigen::MatrixXd A(3, 4);
  A.col(0) = Eigen::Vector3d(1.0, 0.0, 0.0);
  A.col(1) = Eigen::Vector3d(1.0, 0.0, 0.0); // duplicate of col 0
  A.col(2) = Eigen::Vector3d(0.0, 1.0, 0.0);
  A.col(3) = Eigen::Vector3d(0.0, 0.0, 1.0);
  const Eigen::Vector3d b(2.0, 3.0, 4.0);

  Eigen::VectorXd x;
  EXPECT_TRUE(solveNonNegativeLeastSquares(A, b, x));

  EXPECT_GE(x.minCoeff(), -1e-12);
  EXPECT_NEAR((A * x - b).norm(), 0.0, 1e-10);
}

//==============================================================================
TEST(NonNegativeLeastSquares, ZeroMatrixYieldsZeroSolution)
{
  const Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 2);
  const Eigen::Vector3d b(1.0, 2.0, 3.0);

  Eigen::VectorXd x;
  EXPECT_TRUE(solveNonNegativeLeastSquares(A, b, x));

  ASSERT_EQ(x.size(), 2);
  EXPECT_NEAR(x.norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(NonNegativeLeastSquares, HandlesEmptyColumns)
{
  const Eigen::MatrixXd A(3, 0);
  const Eigen::Vector3d b(1.0, 2.0, 3.0);

  Eigen::VectorXd x;
  EXPECT_TRUE(solveNonNegativeLeastSquares(A, b, x));
  EXPECT_EQ(x.size(), 0);
}

//==============================================================================
TEST(NonNegativeLeastSquares, IsDeterministic)
{
  std::srand(123);
  const Eigen::MatrixXd A = Eigen::MatrixXd::Random(5, 4);
  const Eigen::VectorXd b = Eigen::VectorXd::Random(5);

  Eigen::VectorXd x1;
  Eigen::VectorXd x2;
  ASSERT_TRUE(solveNonNegativeLeastSquares(A, b, x1));
  ASSERT_TRUE(solveNonNegativeLeastSquares(A, b, x2));

  EXPECT_TRUE(x1 == x2);
}

//==============================================================================
TEST(NonNegativeLeastSquares, RejectsNonFiniteInputs)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
  A(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector2d b(1.0, 1.0);

  Eigen::VectorXd x;
  EXPECT_FALSE(solveNonNegativeLeastSquares(A, b, x));
}

//==============================================================================
TEST(NonNegativeLeastSquares, RejectsDimensionMismatch)
{
  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  const Eigen::Vector2d b(1.0, 1.0);

  Eigen::VectorXd x;
  EXPECT_FALSE(solveNonNegativeLeastSquares(A, b, x));
}
