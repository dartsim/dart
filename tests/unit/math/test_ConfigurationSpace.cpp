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

#include "../../helpers/GTestUtils.hpp"
#include "dart/math/configuration_space.hpp"
#include "dart/math/geometry.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::math;

//==============================================================================
TEST(ConfigurationSpaceTests, RealVectorSpaceInverseUsesLdltFallback)
{
  Eigen::Matrix<double, 5, 5> random
      = Eigen::Matrix<double, 5, 5>::Random().eval();
  // Make the matrix symmetric positive definite so LDLT is well conditioned.
  Eigen::Matrix<double, 5, 5> spd
      = random.transpose() * random + Eigen::Matrix<double, 5, 5>::Identity();
  const Eigen::Matrix<double, 5, 5> identity
      = Eigen::Matrix<double, 5, 5>::Identity();

  const auto inv = inverse<RealVectorSpace<5>>(spd);

  EXPECT_MATRIX_NEAR(spd * inv, identity, 1e-12);
}

//==============================================================================
TEST(ConfigurationSpaceTests, SO3RoundTripThroughEuclideanSpace)
{
  const Eigen::Vector3d axisAngle(0.2, -0.4, 0.1);
  const auto rot = math::expMapRot(axisAngle);

  const auto euclidean = toEuclideanPoint<SO3Space>(rot);
  EXPECT_VECTOR_NEAR(euclidean, axisAngle, 1e-12);

  const auto manifold = toManifoldPoint<SO3Space>(euclidean);
  EXPECT_MATRIX_NEAR(manifold, rot, 1e-12);
}

//==============================================================================
TEST(ConfigurationSpaceTests, SE3RoundTripThroughEuclideanSpace)
{
  Eigen::Vector6d twist;
  twist << 0.1, -0.3, 0.2, 0.5, -0.25, 1.0;

  const auto pose = toManifoldPoint<SE3Space>(twist);
  const auto recovered = toEuclideanPoint<SE3Space>(pose);

  EXPECT_VECTOR_NEAR(recovered, twist, 1e-12);
}

//==============================================================================
TEST(ConfigurationSpaceTests, IntegratePositionSO3MatchesClosedForm)
{
  const Eigen::Matrix3d pose
      = math::expMapRot(Eigen::Vector3d(0.1, -0.2, 0.05));
  const Eigen::Vector3d velocity(0.4, 0.0, -0.25);
  const double dt = 0.02;

  const Eigen::Matrix3d integrated
      = integratePosition<SO3Space>(pose, velocity, dt);
  const Eigen::Matrix3d expected
      = (pose * math::expMapRot(velocity * dt)).eval();

  EXPECT_MATRIX_NEAR(integrated, expected, 1e-12);
}

//==============================================================================
TEST(ConfigurationSpaceTests, IntegratePositionSE3AppliesTwistIncrement)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = math::expMapRot(Eigen::Vector3d(0.0, 0.3, -0.1));
  pose.translation() = Eigen::Vector3d(1.0, -2.0, 0.5);

  Eigen::Vector6d velocity;
  velocity << 0.2, -0.1, 0.05, 0.25, -0.3, 0.4;
  const double dt = 0.1;

  const Eigen::Isometry3d integrated
      = integratePosition<SE3Space>(pose, velocity, dt);
  const Eigen::Isometry3d expected
      = pose * toManifoldPoint<SE3Space>(velocity * dt);

  EXPECT_TRUE(integrated.isApprox(expected, 1e-12));
}

//==============================================================================
TEST(ConfigurationSpaceTests, IntegrateVelocityAddsAcceleration)
{
  const Eigen::Vector2d velocity(0.5, -0.2);
  const Eigen::Vector2d acceleration(1.0, 0.25);
  const double dt = 0.4;
  const Eigen::Vector2d expected = velocity + acceleration * dt;

  const auto integrated
      = integrateVelocity<R2Space>(velocity, acceleration, dt);
  EXPECT_VECTOR_NEAR(integrated, expected, 1e-12);
}
