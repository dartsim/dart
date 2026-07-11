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

#include <dart/math/detail/ExactCoulombContactProblem.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace {

auto makeDenseDelassusOperator(Eigen::MatrixXd delassus)
{
  return [delassus](
             const Eigen::Ref<const Eigen::VectorXd>& input,
             Eigen::Ref<Eigen::VectorXd> output) {
    output.noalias() = delassus * input;
  };
}

void expectVectorNear(
    const Eigen::VectorXd& actual, const Eigen::VectorXd& expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (Eigen::Index i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], 1e-12);
  }
}

void expectResidualNear(
    const dart::math::detail::CoulombConeResidual& actual,
    double primalFeasibility,
    double dualFeasibility,
    double complementarity,
    double value)
{
  EXPECT_NEAR(actual.primalFeasibility, primalFeasibility, 1e-12);
  EXPECT_NEAR(actual.dualFeasibility, dualFeasibility, 1e-12);
  EXPECT_NEAR(actual.complementarity, complementarity, 1e-12);
  EXPECT_NEAR(actual.value, value, 1e-12);
}

} // namespace

TEST(ExactCoulombContactProblem, ValidatesNormalFirstProblemShape)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity = Eigen::VectorXd::Zero(6);
  problem.coefficients = Eigen::Vector2d(0.5, 0.8);

  EXPECT_TRUE(dart::math::detail::isValidExactCoulombContactProblem(problem));
  EXPECT_EQ(problem.getContactCount(), 2);
  EXPECT_EQ(problem.getDimension(), 6);

  problem.freeVelocity.resize(5);
  EXPECT_FALSE(dart::math::detail::isValidExactCoulombContactProblem(problem));

  problem.freeVelocity = Eigen::VectorXd::Zero(6);
  problem.coefficients[1] = -0.1;
  EXPECT_FALSE(dart::math::detail::isValidExactCoulombContactProblem(problem));
}

TEST(ExactCoulombContactProblem, ComputesMatrixFreeContactVelocity)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity = Eigen::Vector3d(1.0, -1.0, 0.5);
  problem.coefficients = Eigen::VectorXd::Constant(1, 0.5);

  const Eigen::Matrix3d delassus = Eigen::Vector3d(2.0, 3.0, 4.0).asDiagonal();
  const Eigen::Vector3d reaction(0.5, 1.0, -2.0);

  const Eigen::VectorXd velocity
      = dart::math::detail::computeExactCoulombContactVelocityNormalFirst(
          problem, reaction, makeDenseDelassusOperator(delassus));

  expectVectorNear(velocity, Eigen::Vector3d(2.0, 2.0, -7.5));
}

TEST(ExactCoulombContactProblem, ComputesDeSaxceFengAugmentedVelocity)
{
  Eigen::VectorXd velocity(6);
  velocity << 1.0, 3.0, 4.0, -2.0, 6.0, 8.0;

  Eigen::VectorXd coefficients(2);
  coefficients << 0.5, 2.0;

  const Eigen::VectorXd augmentedVelocity
      = dart::math::detail::computeExactCoulombAugmentedVelocityNormalFirst(
          velocity, coefficients);

  Eigen::VectorXd expected(6);
  expected << 3.5, 3.0, 4.0, 18.0, 6.0, 8.0;
  expectVectorNear(augmentedVelocity, expected);
}

TEST(ExactCoulombContactProblem, ComputesZeroResidualForBalancedContact)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity = Eigen::Vector3d(-2.0, -0.3, -0.4);
  problem.coefficients = Eigen::VectorXd::Constant(1, 0.5);

  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d reaction(2.0, 0.3, 0.4);

  const auto residual
      = dart::math::detail::computeExactCoulombContactResidualNormalFirst(
          problem, reaction, makeDenseDelassusOperator(delassus));

  expectResidualNear(residual, 0.0, 0.0, 0.0, 0.0);
}

TEST(ExactCoulombContactProblem, ComputesResidualAfterVelocityAssembly)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity = Eigen::Vector3d(-2.0, 4.0, 0.0);
  problem.coefficients = Eigen::VectorXd::Constant(1, 0.5);

  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Zero();
  const Eigen::Vector3d reaction = Eigen::Vector3d::Zero();

  const auto residual
      = dart::math::detail::computeExactCoulombContactResidualNormalFirst(
          problem, reaction, makeDenseDelassusOperator(delassus));

  expectResidualNear(residual, 0.0, std::sqrt(3.2), 0.0, std::sqrt(3.2));
}

TEST(ExactCoulombContactProblem, EstimatesLargestDelassusEigenvalue)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity = Eigen::VectorXd::Zero(6);
  problem.coefficients = Eigen::Vector2d(0.5, 0.8);

  Eigen::MatrixXd delassus = Eigen::MatrixXd::Zero(6, 6);
  delassus.diagonal() << 2.0, 1.0, 0.5, 0.25, 0.125, 0.0625;

  const double estimate
      = dart::math::detail::estimateLargestExactCoulombDelassusEigenvalue(
          problem, makeDenseDelassusOperator(delassus), 15);

  EXPECT_NEAR(estimate, 2.0, 1e-8);
}

TEST(ExactCoulombContactProblem, EstimatesZeroForEmptyProblem)
{
  dart::math::detail::ExactCoulombContactProblem problem;
  problem.freeVelocity.resize(0);
  problem.coefficients.resize(0);

  const Eigen::MatrixXd delassus(0, 0);
  const double estimate
      = dart::math::detail::estimateLargestExactCoulombDelassusEigenvalue(
          problem, makeDenseDelassusOperator(delassus), 10);

  EXPECT_EQ(estimate, 0.0);
}
