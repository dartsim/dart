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

#include <dart/math/detail/CoulombCone.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace {

void expectVectorNear(
    const Eigen::Vector3d& actual, const Eigen::Vector3d& expected)
{
  EXPECT_NEAR(actual[0], expected[0], 1e-12);
  EXPECT_NEAR(actual[1], expected[1], 1e-12);
  EXPECT_NEAR(actual[2], expected[2], 1e-12);
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

TEST(CoulombCone, ProjectsInteriorPointToItself)
{
  const Eigen::Vector3d value{2.0, 0.3, 0.4};

  const Eigen::Vector3d projected
      = dart::math::detail::projectCoulombConeNormalFirst(value, 0.5);

  expectVectorNear(projected, value);
}

TEST(CoulombCone, ProjectsPolarRegionToOrigin)
{
  const Eigen::Vector3d value{-2.0, 0.3, 0.4};

  const Eigen::Vector3d projected
      = dart::math::detail::projectCoulombConeNormalFirst(value, 0.5);

  expectVectorNear(projected, Eigen::Vector3d::Zero());
}

TEST(CoulombCone, ProjectsExteriorPointToBoundary)
{
  const Eigen::Vector3d value{0.0, 2.0, 0.0};

  const Eigen::Vector3d projected
      = dart::math::detail::projectCoulombConeNormalFirst(value, 0.5);

  expectVectorNear(projected, Eigen::Vector3d(0.8, 0.4, 0.0));
  EXPECT_NEAR(projected.tail<2>().norm(), 0.5 * projected[0], 1e-12);
}

TEST(CoulombCone, HandlesZeroFrictionAsNormalRay)
{
  const Eigen::Vector3d positiveNormal{2.0, 3.0, 4.0};
  const Eigen::Vector3d negativeNormal{-2.0, 3.0, 4.0};

  expectVectorNear(
      dart::math::detail::projectCoulombConeNormalFirst(positiveNormal, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0));
  expectVectorNear(
      dart::math::detail::projectCoulombConeNormalFirst(negativeNormal, 0.0),
      Eigen::Vector3d::Zero());
}

TEST(CoulombCone, ProjectsDualConeWithReciprocalCoefficient)
{
  const Eigen::Vector3d value{0.0, 4.0, 0.0};

  const Eigen::Vector3d projected
      = dart::math::detail::projectCoulombDualConeNormalFirst(value, 0.5);

  expectVectorNear(projected, Eigen::Vector3d(1.6, 3.2, 0.0));
  EXPECT_NEAR(projected.tail<2>().norm(), 2.0 * projected[0], 1e-12);
}

TEST(CoulombCone, HandlesZeroFrictionDualAsNormalHalfspace)
{
  const Eigen::Vector3d positiveNormal{2.0, 3.0, 4.0};
  const Eigen::Vector3d negativeNormal{-2.0, 3.0, 4.0};

  expectVectorNear(
      dart::math::detail::projectCoulombDualConeNormalFirst(
          positiveNormal, 0.0),
      positiveNormal);
  expectVectorNear(
      dart::math::detail::projectCoulombDualConeNormalFirst(
          negativeNormal, 0.0),
      Eigen::Vector3d(0.0, 3.0, 4.0));
}

TEST(CoulombConeResidual, ReportsZeroForExactContact)
{
  const Eigen::Vector3d reaction{2.0, 0.3, 0.4};
  const Eigen::Vector3d augmentedVelocity = Eigen::Vector3d::Zero();

  const auto residual
      = dart::math::detail::computeCoulombConeResidualNormalFirst(
          reaction, augmentedVelocity, 0.5);

  expectResidualNear(residual, 0.0, 0.0, 0.0, 0.0);
}

TEST(CoulombConeResidual, MeasuresScaledPrimalConeDistance)
{
  const Eigen::Vector3d reaction{0.0, 2.0, 0.0};
  const Eigen::Vector3d augmentedVelocity = Eigen::Vector3d::Zero();
  const dart::math::detail::CoulombConeResidualScales scales{2.0, 4.0};
  const double expectedPrimal = std::sqrt(3.2) / scales.reactionScale;

  const auto residual
      = dart::math::detail::computeCoulombConeResidualNormalFirst(
          reaction, augmentedVelocity, 0.5, scales);

  expectResidualNear(residual, expectedPrimal, 0.0, 0.0, expectedPrimal);
}

TEST(CoulombConeResidual, MeasuresScaledDualConeDistance)
{
  const Eigen::Vector3d reaction = Eigen::Vector3d::Zero();
  const Eigen::Vector3d augmentedVelocity{0.0, 4.0, 0.0};
  const dart::math::detail::CoulombConeResidualScales scales{2.0, 4.0};
  const double expectedDual = std::sqrt(3.2) / scales.velocityScale;

  const auto residual
      = dart::math::detail::computeCoulombConeResidualNormalFirst(
          reaction, augmentedVelocity, 0.5, scales);

  expectResidualNear(residual, 0.0, expectedDual, 0.0, expectedDual);
}

TEST(CoulombConeResidual, MeasuresScaledComplementarityGap)
{
  const Eigen::Vector3d reaction{2.0, 0.0, 0.0};
  const Eigen::Vector3d augmentedVelocity{3.0, 0.0, 0.0};
  const dart::math::detail::CoulombConeResidualScales scales{2.0, 3.0};

  const auto residual
      = dart::math::detail::computeCoulombConeResidualNormalFirst(
          reaction, augmentedVelocity, 0.5, scales);

  expectResidualNear(residual, 0.0, 0.0, 1.0, 1.0);
}

TEST(CoulombConeResidual, MeasuresProductConeDistanceAndGlobalGap)
{
  Eigen::VectorXd reactions(6);
  reactions << 0.0, 2.0, 0.0, 2.0, 0.0, 0.0;

  Eigen::VectorXd augmentedVelocities(6);
  augmentedVelocities << 0.0, 0.0, 0.0, 3.0, 0.0, 0.0;

  Eigen::VectorXd coefficients(2);
  coefficients << 0.5, 0.5;

  const dart::math::detail::CoulombConeResidualScales scales{2.0, 3.0};
  const double expectedPrimal = std::sqrt(3.2) / scales.reactionScale;

  const auto residual
      = dart::math::detail::computeCoulombConeResidualNormalFirst(
          reactions, augmentedVelocities, coefficients, scales);

  expectResidualNear(residual, expectedPrimal, 0.0, 1.0, 1.0);
  EXPECT_EQ(residual.worstPrimalContact, 0);
  EXPECT_EQ(residual.worstDualContact, 0);
  EXPECT_EQ(residual.worstComplementarityContact, 1);
}
