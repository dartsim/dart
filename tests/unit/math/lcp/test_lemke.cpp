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

#include "../../../helpers/gtest_utils.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/pivoting/lemke_solver.hpp"

#include <gtest/gtest.h>

using dart::math::LcpOptions;
using dart::math::LcpProblem;
using dart::math::LcpSolverStatus;

namespace {

void ExpectValidLCP(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& x)
{
  const Eigen::VectorXd w = A * x - b;
  EXPECT_GE(w.minCoeff(), -2e-6); // allow tiny numerical noise
  EXPECT_GE(x.minCoeff(), -1e-6);
  EXPECT_LT((w.array() * x.array()).abs().maxCoeff(), 1e-4);
}

} // namespace

//==============================================================================
TEST(Lemke, Lemke_1D)
{
  dart::math::LemkeSolver solver;
  Eigen::MatrixXd A(1, 1);
  Eigen::VectorXd b(1);
  Eigen::VectorXd f(1);

  A << 1;
  b << -1.5;

  LcpProblem problem(
      A,
      b,
      Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity()),
      Eigen::VectorXi::Constant(1, -1));
  LcpOptions options = solver.getDefaultOptions();
  const auto result = solver.solve(problem, f, options);
  EXPECT_EQ(result.status, dart::math::LcpSolverStatus::Success);
  ExpectValidLCP(A, b, f);
}

//==============================================================================
TEST(Lemke, Lemke_2D)
{
  dart::math::LemkeSolver solver;
  Eigen::MatrixXd A(2, 2);
  Eigen::VectorXd b(2);
  Eigen::VectorXd f(2);

  A << 3.12, 0.1877, 0.1877, 3.254;
  b << -0.00662, -0.006711;

  LcpProblem problem(
      A,
      b,
      Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity()),
      Eigen::VectorXi::Constant(2, -1));
  LcpOptions options = solver.getDefaultOptions();
  const auto result = solver.solve(problem, f, options);
  EXPECT_EQ(result.status, dart::math::LcpSolverStatus::Success);
  ExpectValidLCP(A, b, f);
}

//==============================================================================
TEST(Lemke, Lemke_4D)
{
  dart::math::LemkeSolver solver;
  Eigen::MatrixXd A(4, 4);
  Eigen::VectorXd b(4);
  Eigen::VectorXd f(4);

  A << 3.999, 0.9985, 1.001, -2, 0.9985, 3.998, -2, 0.9995, 1.001, -2, 4.002,
      1.001, -2, 0.9995, 1.001, 4.001;

  b << -0.01008, -0.009494, -0.07234, -0.07177;

  LcpProblem problem(
      A,
      b,
      Eigen::VectorXd::Zero(4),
      Eigen::VectorXd::Constant(4, std::numeric_limits<double>::infinity()),
      Eigen::VectorXi::Constant(4, -1));
  LcpOptions options = solver.getDefaultOptions();
  const auto result = solver.solve(problem, f, options);
  EXPECT_EQ(result.status, dart::math::LcpSolverStatus::Success);
  ExpectValidLCP(A, b, f);
}

//==============================================================================
TEST(Lemke, Lemke_6D)
{
  dart::math::LemkeSolver solver;
  Eigen::MatrixXd A(6, 6);
  Eigen::VectorXd b(6);
  Eigen::VectorXd f(6);

  A << 3.1360, -2.0370, 0.9723, 0.1096, -2.0370, 0.9723, -2.0370, 3.7820,
      0.8302, -0.0257, 2.4730, 0.0105, 0.9723, 0.8302, 5.1250, -2.2390, -1.9120,
      3.4080, 0.1096, -0.0257, -2.2390, 3.1010, -0.0257, -2.2390, -2.0370,
      2.4730, -1.9120, -0.0257, 5.4870, -0.0242, 0.9723, 0.0105, 3.4080,
      -2.2390, -0.0242, 3.3860;

  b << 0.1649, -0.0025, -0.0904, -0.0093, -0.0000, -0.0889;

  LcpProblem problem(
      A,
      b,
      Eigen::VectorXd::Zero(6),
      Eigen::VectorXd::Constant(6, std::numeric_limits<double>::infinity()),
      Eigen::VectorXi::Constant(6, -1));
  LcpOptions options = solver.getDefaultOptions();
  const auto result = solver.solve(problem, f, options);
  EXPECT_EQ(result.status, dart::math::LcpSolverStatus::Success);
  ExpectValidLCP(A, b, f);
}

//==============================================================================
TEST(Lemke, Lemke_12D)
{
  dart::math::LemkeSolver solver;
  Eigen::MatrixXd A(12, 12);
  Eigen::VectorXd b(12);
  Eigen::VectorXd f(12);

  A << 4.03, -1.014, -1.898, 1.03, -1.014, -1.898, 1, -1.014, -1.898, -2,
      -1.014, -1.898, -1.014, 4.885, -1.259, 1.888, 3.81, 2.345, -1.879, 1.281,
      -2.334, 1.022, 0.206, 1.27, -1.898, -1.259, 3.2, -1.032, -0.6849, 1.275,
      1.003, 0.6657, 3.774, 1.869, 1.24, 1.85, 1.03, 1.888, -1.032, 4.03, 1.888,
      -1.032, -2, 1.888, -1.032, 1, 1.888, -1.032, -1.014, 3.81, -0.6849, 1.888,
      3.225, 1.275, -1.879, 1.85, -1.27, 1.022, 1.265, 0.6907, -1.898, 2.345,
      1.275, -1.032, 1.275, 4.86, 1.003, -1.24, 0.2059, 1.869, -2.309, 3.791, 1,
      -1.879, 1.003, -2, -1.879, 1.003, 3.97, -1.879, 1.003, 0.9703, -1.879,
      1.003, -1.014, 1.281, 0.6657, 1.888, 1.85, -1.24, -1.879, 3.187, 1.234,
      1.022, 3.755, -0.6714, -1.898, -2.334, 3.774, -1.032, -1.27, 0.2059,
      1.003, 1.234, 4.839, 1.869, 2.299, 1.27, -2, 1.022, 1.869, 1, 1.022,
      1.869, 0.9703, 1.022, 1.869, 3.97, 1.022, 1.869, -1.014, 0.206, 1.24,
      1.888, 1.265, -2.309, -1.879, 3.755, 2.299, 1.022, 4.814, -1.25, -1.898,
      1.27, 1.85, -1.032, 0.6907, 3.791, 1.003, -0.6714, 1.27, 1.869, -1.25,
      3.212;

  b << -0.00981, -1.458e-10, 5.357e-10, -0.0098, -1.44e-10, 5.298e-10,
      -0.009807, -1.399e-10, 5.375e-10, -0.009807, -1.381e-10, 5.316e-10;

  LcpProblem problem(
      A,
      b,
      Eigen::VectorXd::Zero(12),
      Eigen::VectorXd::Constant(12, std::numeric_limits<double>::infinity()),
      Eigen::VectorXi::Constant(12, -1));
  LcpOptions options = solver.getDefaultOptions();
  const auto result = solver.solve(problem, f, options);
  EXPECT_EQ(result.status, dart::math::LcpSolverStatus::Success);
  ExpectValidLCP(A, b, f);
}

//==============================================================================
TEST(Lemke, SolvesBoxedBoundsViaFallback)
{
  dart::math::LemkeSolver solver;

  Eigen::MatrixXd A(1, 1);
  Eigen::VectorXd b(1);
  Eigen::VectorXd f(1);
  A << 1.0;
  b << 0.0;

  LcpProblem problem(
      A,
      b,
      Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Constant(1, 1.0),
      Eigen::VectorXi::Constant(1, -1));
  const auto result = solver.solve(problem, f, solver.getDefaultOptions());
  EXPECT_EQ(result.status, dart::math::LcpSolverStatus::Success);
  EXPECT_GE(f[0], 0.0);
  EXPECT_LE(f[0], 1.0);
}

//==============================================================================
TEST(Lemke, SolvesFrictionIndexViaFallback)
{
  dart::math::LemkeSolver solver;

  Eigen::Matrix3d A;
  A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;

  const Eigen::Vector3d target(1.0, 0.2, -0.1);
  const Eigen::Vector3d b = A * target;

  Eigen::Vector3d lo;
  lo << 0.0, -0.5, -0.5;
  Eigen::Vector3d hi;
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5;
  Eigen::Vector3i findex;
  findex << -1, 0, 0;

  LcpProblem problem(A, b, lo, hi, findex);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  const auto result = solver.solve(problem, x, solver.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::Success);

  dart::math::DantzigSolver reference;
  Eigen::VectorXd referenceX = Eigen::VectorXd::Zero(3);
  LcpOptions refOptions = reference.getDefaultOptions();
  refOptions.warmStart = false;
  const auto refResult = reference.solve(problem, referenceX, refOptions);
  ASSERT_TRUE(refResult.succeeded());

  EXPECT_NEAR(x[0], referenceX[0], 1e-6);
  EXPECT_NEAR(x[1], referenceX[1], 1e-6);
  EXPECT_NEAR(x[2], referenceX[2], 1e-6);

  const double mu = hi[1];
  EXPECT_LE(std::abs(x[1]), mu * x[0] + 1e-8);
  EXPECT_LE(std::abs(x[2]), mu * x[0] + 1e-8);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
