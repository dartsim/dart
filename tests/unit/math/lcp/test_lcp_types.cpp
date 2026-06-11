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

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart::math;

namespace {

bool vectorExactWithInfinities(
    const Eigen::VectorXd& actual, const Eigen::VectorXd& expected)
{
  if (actual.size() != expected.size()) {
    return false;
  }

  for (Eigen::Index i = 0; i < actual.size(); ++i) {
    if (std::isinf(actual[i]) || std::isinf(expected[i])) {
      if (actual[i] != expected[i]) {
        return false;
      }
    } else if (std::abs(actual[i] - expected[i]) > 1e-12) {
      return false;
    }
  }

  return true;
}

} // namespace

TEST(LcpTypesTest, StatusToString)
{
  EXPECT_EQ(toString(LcpSolverStatus::Success), "Success");
  EXPECT_EQ(toString(LcpSolverStatus::Failed), "Failed");
  EXPECT_EQ(toString(LcpSolverStatus::MaxIterations), "MaxIterations");
  EXPECT_EQ(toString(LcpSolverStatus::NumericalError), "NumericalError");
  EXPECT_EQ(toString(LcpSolverStatus::InvalidProblem), "InvalidProblem");
  EXPECT_EQ(toString(LcpSolverStatus::Degenerate), "Degenerate");
  EXPECT_EQ(toString(LcpSolverStatus::NotSolved), "NotSolved");

  EXPECT_EQ(toString(static_cast<LcpSolverStatus>(-1)), "Unknown");
  EXPECT_EQ(toString(static_cast<LcpSolverStatus>(99)), "Unknown");
}

TEST(LcpTypesTest, StandardProblemConstructorSetsCanonicalBounds)
{
  Eigen::Matrix2d A;
  A << 2.0, 0.1, 0.1, 3.0;
  Eigen::Vector2d b(1.0, -0.5);

  const LcpProblem problem(A, b);

  EXPECT_EQ(problem.size(), 2);
  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(problem.isStandardLcp());
  EXPECT_TRUE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_TRUE(problem.A.isApprox(A));
  EXPECT_TRUE(problem.b.isApprox(b));
  EXPECT_TRUE(problem.lo.isZero());
  EXPECT_TRUE(problem.hi.array().isInf().all());
  EXPECT_TRUE((problem.findex.array() == -1).all());
}

TEST(LcpTypesTest, EmptyStandardProblemIsCanonical)
{
  const LcpProblem problem(Eigen::MatrixXd(0, 0), Eigen::VectorXd(0));

  EXPECT_EQ(problem.size(), 0);
  EXPECT_TRUE(problem.empty());
  EXPECT_TRUE(problem.isStandardLcp());
  EXPECT_TRUE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_EQ(problem.lo.size(), 0);
  EXPECT_EQ(problem.hi.size(), 0);
  EXPECT_EQ(problem.findex.size(), 0);
}

TEST(LcpTypesTest, BoxedProblemConstructorSetsNoFrictionIndex)
{
  Eigen::Matrix2d A;
  A << 4.0, 0.5, 0.5, 2.0;
  Eigen::Vector2d b(0.25, 0.75);
  Eigen::Vector2d lo(-1.0, 0.0);
  Eigen::Vector2d hi(1.0, std::numeric_limits<double>::infinity());

  const LcpProblem problem(A, b, lo, hi);

  EXPECT_EQ(problem.size(), 2);
  EXPECT_FALSE(problem.isStandardLcp());
  EXPECT_TRUE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_TRUE(problem.A.isApprox(A));
  EXPECT_TRUE(problem.b.isApprox(b));
  EXPECT_TRUE(problem.lo.isApprox(lo));
  EXPECT_TRUE(vectorExactWithInfinities(problem.hi, hi));
  EXPECT_TRUE((problem.findex.array() == -1).all());
}

TEST(LcpTypesTest, FrictionIndexProblemConstructorPreservesFindex)
{
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  Eigen::Vector3d b(1.0, 0.25, -0.25);
  Eigen::Vector3d lo(0.0, -0.5, -0.5);
  Eigen::Vector3d hi(std::numeric_limits<double>::infinity(), 0.5, 0.5);
  Eigen::Vector3i findex(-1, 0, 0);

  const LcpProblem problem(A, b, lo, hi, findex);

  EXPECT_EQ(problem.size(), 3);
  EXPECT_FALSE(problem.isStandardLcp());
  EXPECT_FALSE(problem.isBoxedLcp());
  EXPECT_TRUE(problem.hasFrictionIndex());
  EXPECT_TRUE(problem.A.isApprox(A));
  EXPECT_TRUE(problem.b.isApprox(b));
  EXPECT_TRUE(problem.lo.isApprox(lo));
  EXPECT_TRUE(vectorExactWithInfinities(problem.hi, hi));
  EXPECT_TRUE((problem.findex.array() == findex.array()).all());
}

TEST(LcpTypesTest, StandardClassificationHonorsTolerance)
{
  Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  Eigen::Vector2d b(0.5, 0.25);
  Eigen::Vector2d lo(1e-9, -1e-9);
  Eigen::Vector2d hi
      = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());

  const LcpProblem problem(A, b, lo, hi);

  EXPECT_FALSE(problem.isStandardLcp());
  EXPECT_TRUE(problem.isBoxedLcp());
  EXPECT_TRUE(problem.isStandardLcp(1e-8));
}

TEST(LcpTypesTest, StandardClassificationRejectsInvalidVectorDimensions)
{
  const LcpProblem problem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      Eigen::VectorXd::Zero(1),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector2i::Constant(-1));

  EXPECT_FALSE(problem.isStandardLcp());
  EXPECT_FALSE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
}
