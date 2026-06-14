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

TEST(LcpTypesTest, ProblemTypeToString)
{
  EXPECT_EQ(toString(LcpProblemType::Invalid), "Invalid");
  EXPECT_EQ(toString(LcpProblemType::Standard), "Standard");
  EXPECT_EQ(toString(LcpProblemType::Boxed), "Boxed");
  EXPECT_EQ(toString(LcpProblemType::FrictionIndex), "FrictionIndex");

  EXPECT_EQ(toString(static_cast<LcpProblemType>(-1)), "Unknown");
  EXPECT_EQ(toString(static_cast<LcpProblemType>(99)), "Unknown");
}

TEST(LcpTypesTest, StandardProblemConstructorSetsCanonicalBounds)
{
  Eigen::Matrix2d A;
  A << 2.0, 0.1, 0.1, 3.0;
  Eigen::Vector2d b(1.0, -0.5);

  const LcpProblem problem(A, b);

  EXPECT_EQ(problem.size(), 2);
  EXPECT_FALSE(problem.empty());
  EXPECT_TRUE(problem.isValid());
  EXPECT_TRUE(problem.getValidationMessage().empty());
  EXPECT_TRUE(problem.isStandardLcp());
  EXPECT_TRUE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_EQ(problem.getFrictionIndexRowCount(), 0);
  EXPECT_EQ(problem.getFrictionIndexContactCount(), 0);
  EXPECT_EQ(problem.getType(), LcpProblemType::Standard);
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
  EXPECT_TRUE(problem.isValid());
  EXPECT_TRUE(problem.getValidationMessage().empty());
  EXPECT_TRUE(problem.isStandardLcp());
  EXPECT_TRUE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_EQ(problem.getFrictionIndexRowCount(), 0);
  EXPECT_EQ(problem.getFrictionIndexContactCount(), 0);
  EXPECT_EQ(problem.getType(), LcpProblemType::Standard);
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
  EXPECT_TRUE(problem.isValid());
  EXPECT_TRUE(problem.getValidationMessage().empty());
  EXPECT_FALSE(problem.isStandardLcp());
  EXPECT_TRUE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_EQ(problem.getFrictionIndexRowCount(), 0);
  EXPECT_EQ(problem.getFrictionIndexContactCount(), 0);
  EXPECT_EQ(problem.getType(), LcpProblemType::Boxed);
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
  EXPECT_TRUE(problem.isValid());
  EXPECT_TRUE(problem.getValidationMessage().empty());
  EXPECT_FALSE(problem.isStandardLcp());
  EXPECT_FALSE(problem.isBoxedLcp());
  EXPECT_TRUE(problem.hasFrictionIndex());
  EXPECT_EQ(problem.getFrictionIndexRowCount(), 2);
  EXPECT_EQ(problem.getFrictionIndexContactCount(), 1);
  EXPECT_EQ(problem.getType(), LcpProblemType::FrictionIndex);
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
  EXPECT_EQ(problem.getType(), LcpProblemType::Boxed);
  EXPECT_EQ(problem.getType(1e-8), LcpProblemType::Standard);
}

TEST(LcpTypesTest, FrictionIndexMetadataCountsUniqueReferencedNormalRows)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd lo(6);
  lo << 0.0, -0.5, -0.5, 0.0, -0.3, -0.3;
  Eigen::VectorXd hi(6);
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5,
      std::numeric_limits<double>::infinity(), 0.3, 0.3;
  Eigen::VectorXi findex(6);
  findex << -1, 0, 0, -1, 3, 3;

  const LcpProblem problem(A, b, lo, hi, findex);

  EXPECT_TRUE(problem.hasFrictionIndex());
  EXPECT_EQ(problem.getType(), LcpProblemType::FrictionIndex);
  EXPECT_EQ(problem.getFrictionIndexRowCount(), 4);
  EXPECT_EQ(problem.getFrictionIndexContactCount(), 2);
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
  EXPECT_EQ(problem.getFrictionIndexRowCount(), 0);
  EXPECT_EQ(problem.getFrictionIndexContactCount(), 0);
  EXPECT_FALSE(problem.isValid());
  EXPECT_EQ(
      problem.getValidationMessage(), "Matrix/vector dimensions inconsistent");
  EXPECT_EQ(problem.getType(), LcpProblemType::Invalid);
}

TEST(LcpTypesTest, ClassificationRejectsInvalidMatrixDimensions)
{
  const LcpProblem problem(
      Eigen::MatrixXd::Identity(1, 1), Eigen::Vector2d(0.5, 0.25));

  EXPECT_FALSE(problem.isStandardLcp());
  EXPECT_FALSE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_FALSE(problem.isValid());
  EXPECT_EQ(
      problem.getValidationMessage(), "Matrix/vector dimensions inconsistent");
  EXPECT_EQ(problem.getType(), LcpProblemType::Invalid);
}

TEST(LcpTypesTest, ClassificationRejectsNonFiniteMatrixAndVectorData)
{
  Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  A(0, 1) = std::numeric_limits<double>::quiet_NaN();

  const LcpProblem nanMatrixProblem(A, Eigen::Vector2d(0.5, 0.25));

  EXPECT_FALSE(nanMatrixProblem.isStandardLcp());
  EXPECT_FALSE(nanMatrixProblem.isBoxedLcp());
  EXPECT_FALSE(nanMatrixProblem.hasFrictionIndex());
  EXPECT_FALSE(nanMatrixProblem.isValid());
  EXPECT_EQ(
      nanMatrixProblem.getValidationMessage(),
      "Matrix contains non-finite values");
  EXPECT_EQ(nanMatrixProblem.getType(), LcpProblemType::Invalid);

  Eigen::Vector2d b(0.5, std::numeric_limits<double>::infinity());
  const LcpProblem infVectorProblem(Eigen::Matrix2d::Identity(), b);

  EXPECT_FALSE(infVectorProblem.isStandardLcp());
  EXPECT_FALSE(infVectorProblem.isBoxedLcp());
  EXPECT_FALSE(infVectorProblem.hasFrictionIndex());
  EXPECT_FALSE(infVectorProblem.isValid());
  EXPECT_EQ(
      infVectorProblem.getValidationMessage(),
      "Vector b contains non-finite values");
  EXPECT_EQ(infVectorProblem.getType(), LcpProblemType::Invalid);
}

TEST(LcpTypesTest, BoxedClassificationRejectsInvalidBounds)
{
  const LcpProblem reversedBoundsProblem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      Eigen::Vector2d(1.0, 0.0),
      Eigen::Vector2d(0.0, 1.0));

  EXPECT_FALSE(reversedBoundsProblem.isBoxedLcp());
  EXPECT_FALSE(reversedBoundsProblem.isValid());
  EXPECT_EQ(
      reversedBoundsProblem.getValidationMessage(),
      "Lower bound exceeds upper bound");
  EXPECT_EQ(reversedBoundsProblem.getType(), LcpProblemType::Invalid);

  Eigen::Vector2d hi(1.0, std::numeric_limits<double>::quiet_NaN());
  const LcpProblem nanBoundsProblem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      Eigen::Vector2d::Zero(),
      hi);

  EXPECT_FALSE(nanBoundsProblem.isBoxedLcp());
  EXPECT_FALSE(nanBoundsProblem.isValid());
  EXPECT_EQ(nanBoundsProblem.getValidationMessage(), "Bounds contain NaN");
  EXPECT_EQ(nanBoundsProblem.getType(), LcpProblemType::Invalid);

  Eigen::Vector2d loWithPositiveInfinity;
  loWithPositiveInfinity << 0.0, std::numeric_limits<double>::infinity();
  const LcpProblem positiveInfiniteLowerProblem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      loWithPositiveInfinity,
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()));

  EXPECT_FALSE(positiveInfiniteLowerProblem.isBoxedLcp());
  EXPECT_FALSE(positiveInfiniteLowerProblem.isValid());
  EXPECT_EQ(
      positiveInfiniteLowerProblem.getValidationMessage(),
      "Bounds have invalid infinity direction");
  EXPECT_EQ(positiveInfiniteLowerProblem.getType(), LcpProblemType::Invalid);

  Eigen::Vector2d hiWithNegativeInfinity;
  hiWithNegativeInfinity << 1.0, -std::numeric_limits<double>::infinity();
  const LcpProblem negativeInfiniteUpperProblem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      Eigen::Vector2d::Zero(),
      hiWithNegativeInfinity);

  EXPECT_FALSE(negativeInfiniteUpperProblem.isBoxedLcp());
  EXPECT_FALSE(negativeInfiniteUpperProblem.isValid());
  EXPECT_EQ(
      negativeInfiniteUpperProblem.getValidationMessage(),
      "Bounds have invalid infinity direction");
  EXPECT_EQ(negativeInfiniteUpperProblem.getType(), LcpProblemType::Invalid);
}

TEST(LcpTypesTest, FrictionIndexClassificationRejectsInvalidBoundsDimensions)
{
  const LcpProblem problem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      Eigen::VectorXd::Zero(1),
      Eigen::Vector2d::Ones(),
      Eigen::Vector2i(-1, 0));

  EXPECT_FALSE(problem.isStandardLcp());
  EXPECT_FALSE(problem.isBoxedLcp());
  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_FALSE(problem.isValid());
  EXPECT_EQ(
      problem.getValidationMessage(), "Matrix/vector dimensions inconsistent");
  EXPECT_EQ(problem.getType(), LcpProblemType::Invalid);
}

TEST(LcpTypesTest, FrictionIndexClassificationRejectsInvalidReferences)
{
  Eigen::Vector2d lo;
  lo << 0.0, -1.0;
  Eigen::Vector2d hi;
  hi << std::numeric_limits<double>::infinity(), 1.0;

  const LcpProblem outOfRangeProblem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      lo,
      hi,
      Eigen::Vector2i(-1, 2));

  EXPECT_FALSE(outOfRangeProblem.hasFrictionIndex());
  EXPECT_EQ(outOfRangeProblem.getFrictionIndexRowCount(), 0);
  EXPECT_EQ(outOfRangeProblem.getFrictionIndexContactCount(), 0);
  EXPECT_FALSE(outOfRangeProblem.isValid());
  EXPECT_EQ(
      outOfRangeProblem.getValidationMessage(),
      "Friction index entry out of range");
  EXPECT_EQ(outOfRangeProblem.getType(), LcpProblemType::Invalid);

  const LcpProblem selfReferenceProblem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      lo,
      hi,
      Eigen::Vector2i(-1, 1));

  EXPECT_FALSE(selfReferenceProblem.hasFrictionIndex());
  EXPECT_EQ(selfReferenceProblem.getFrictionIndexRowCount(), 0);
  EXPECT_EQ(selfReferenceProblem.getFrictionIndexContactCount(), 0);
  EXPECT_FALSE(selfReferenceProblem.isValid());
  EXPECT_EQ(
      selfReferenceProblem.getValidationMessage(),
      "Friction index entry cannot reference itself");
  EXPECT_EQ(selfReferenceProblem.getType(), LcpProblemType::Invalid);
}

TEST(LcpTypesTest, FrictionIndexClassificationRejectsNegativeCoefficient)
{
  Eigen::Vector2d lo;
  lo << 0.0, -1.0;
  Eigen::Vector2d hi;
  hi << std::numeric_limits<double>::infinity(), -0.5;

  const LcpProblem problem(
      Eigen::Matrix2d::Identity(),
      Eigen::Vector2d(0.5, 0.25),
      lo,
      hi,
      Eigen::Vector2i(-1, 0));

  EXPECT_FALSE(problem.hasFrictionIndex());
  EXPECT_EQ(problem.getFrictionIndexRowCount(), 0);
  EXPECT_EQ(problem.getFrictionIndexContactCount(), 0);
  EXPECT_FALSE(problem.isValid());
  EXPECT_EQ(
      problem.getValidationMessage(),
      "Friction coefficient (hi) must be non-negative");
  EXPECT_EQ(problem.getType(), LcpProblemType::Invalid);
}
