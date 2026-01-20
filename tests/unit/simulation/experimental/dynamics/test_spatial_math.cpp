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

#include <dart/simulation/experimental/dynamics/spatial_math.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace dse = dart::simulation::experimental;
namespace dynamics = dse::dynamics;

//==============================================================================
// Spatial Inertia Tests
//==============================================================================

TEST(SpatialMath, MakeSpatialInertiaPointMassAtOrigin)
{
  double mass = 2.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();

  dynamics::SpatialInertia I = dynamics::makeSpatialInertia(mass, com, inertia);

  EXPECT_NEAR(I(3, 3), mass, 1e-10);
  EXPECT_NEAR(I(4, 4), mass, 1e-10);
  EXPECT_NEAR(I(5, 5), mass, 1e-10);

  EXPECT_TRUE((I.topLeftCorner<3, 3>().isApprox(Eigen::Matrix3d::Zero())));
}

TEST(SpatialMath, MakeSpatialInertiaSymmetric)
{
  double mass = 1.5;
  Eigen::Vector3d com(0.1, 0.2, 0.3);
  Eigen::Matrix3d inertia;
  inertia << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;

  dynamics::SpatialInertia I = dynamics::makeSpatialInertia(mass, com, inertia);

  EXPECT_TRUE(I.isApprox(I.transpose(), 1e-10));
}

TEST(SpatialMath, MakeSpatialInertiaPositiveDefinite)
{
  double mass = 2.0;
  Eigen::Vector3d com(0.1, 0.0, 0.0);
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();

  dynamics::SpatialInertia I = dynamics::makeSpatialInertia(mass, com, inertia);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix6d> solver(I);
  EXPECT_TRUE((solver.eigenvalues().array() > -1e-10).all());
}

TEST(SpatialMath, PointMassInertiaMatchesMakeSpatialInertia)
{
  double mass = 3.0;
  Eigen::Vector3d position(1.0, 2.0, 3.0);

  dynamics::SpatialInertia I1 = dynamics::makePointMassInertia(mass, position);
  dynamics::SpatialInertia I2
      = dynamics::makeSpatialInertia(mass, position, Eigen::Matrix3d::Zero());

  EXPECT_TRUE(I1.isApprox(I2, 1e-10));
}

//==============================================================================
// Spatial Vector Construction Tests
//==============================================================================

TEST(SpatialMath, MakeSpatialVelocity)
{
  Eigen::Vector3d angular(1.0, 2.0, 3.0);
  Eigen::Vector3d linear(4.0, 5.0, 6.0);

  dynamics::SpatialVelocity V = dynamics::makeSpatialVelocity(angular, linear);

  EXPECT_TRUE(dynamics::angular(V).isApprox(angular));
  EXPECT_TRUE(dynamics::linear(V).isApprox(linear));
}

TEST(SpatialMath, MakeSpatialForce)
{
  Eigen::Vector3d torque(1.0, 2.0, 3.0);
  Eigen::Vector3d force(4.0, 5.0, 6.0);

  dynamics::SpatialForce F = dynamics::makeSpatialForce(torque, force);

  EXPECT_TRUE(dynamics::angular(F).isApprox(torque));
  EXPECT_TRUE(dynamics::linear(F).isApprox(force));
}

//==============================================================================
// Spatial Cross Product Tests
//==============================================================================

TEST(SpatialMath, SpatialCrossZeroVelocity)
{
  dynamics::SpatialVelocity V1 = dynamics::SpatialVelocity::Zero();
  dynamics::SpatialVelocity V2;
  V2 << 1, 2, 3, 4, 5, 6;

  dynamics::SpatialVelocity result = dynamics::spatialCross(V1, V2);

  EXPECT_TRUE(result.isApprox(dynamics::SpatialVelocity::Zero()));
}

TEST(SpatialMath, SpatialCrossAntiSymmetric)
{
  dynamics::SpatialVelocity V1;
  V1 << 1, 2, 3, 4, 5, 6;
  dynamics::SpatialVelocity V2;
  V2 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  dynamics::SpatialVelocity cross1 = dynamics::spatialCross(V1, V2);
  dynamics::SpatialVelocity cross2 = dynamics::spatialCross(V2, V1);

  EXPECT_TRUE(cross1.isApprox(-cross2, 1e-10));
}

TEST(SpatialMath, SpatialCrossStarBilinear)
{
  dynamics::SpatialVelocity V;
  V << 1, 2, 3, 4, 5, 6;
  dynamics::SpatialForce F1;
  F1 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  dynamics::SpatialForce F2;
  F2 << 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;

  dynamics::SpatialForce result1 = dynamics::spatialCrossStar(V, F1 + F2);
  dynamics::SpatialForce result2
      = dynamics::spatialCrossStar(V, F1) + dynamics::spatialCrossStar(V, F2);

  EXPECT_TRUE(result1.isApprox(result2, 1e-10));
}

//==============================================================================
// Spatial Transform Tests
//==============================================================================

TEST(SpatialMath, TransformSpatialVelocityIdentity)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  dynamics::SpatialVelocity V;
  V << 1, 2, 3, 4, 5, 6;

  dynamics::SpatialVelocity result = dynamics::transformSpatialVelocity(T, V);

  EXPECT_TRUE(result.isApprox(V, 1e-10));
}

TEST(SpatialMath, TransformSpatialVelocityInverseUndoes)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  T.translate(Eigen::Vector3d(1, 2, 3));

  dynamics::SpatialVelocity V;
  V << 1, 2, 3, 4, 5, 6;

  dynamics::SpatialVelocity transformed
      = dynamics::transformSpatialVelocity(T, V);
  dynamics::SpatialVelocity restored
      = dynamics::transformSpatialVelocityInverse(T, transformed);

  EXPECT_TRUE(restored.isApprox(V, 1e-10));
}

TEST(SpatialMath, TransformSpatialForceIdentity)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  dynamics::SpatialForce F;
  F << 1, 2, 3, 4, 5, 6;

  dynamics::SpatialForce result = dynamics::transformSpatialForce(T, F);

  EXPECT_TRUE(result.isApprox(F, 1e-10));
}

TEST(SpatialMath, TransformSpatialForceInverseUndoes)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()));
  T.translate(Eigen::Vector3d(-1, 0.5, 2));

  dynamics::SpatialForce F;
  F << 1, 2, 3, 4, 5, 6;

  dynamics::SpatialForce transformed = dynamics::transformSpatialForce(T, F);
  dynamics::SpatialForce restored
      = dynamics::transformSpatialForceInverse(T, transformed);

  EXPECT_TRUE(restored.isApprox(F, 1e-10));
}

TEST(SpatialMath, TransformSpatialInertiaIdentity)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  dynamics::SpatialInertia I = dynamics::makeSpatialInertia(
      2.0, Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Matrix3d::Identity());

  dynamics::SpatialInertia result = dynamics::transformSpatialInertia(T, I);

  EXPECT_TRUE(result.isApprox(I, 1e-10));
}

TEST(SpatialMath, TransformSpatialInertiaPreservesSymmetry)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d(1, 1, 1).normalized()));
  T.translate(Eigen::Vector3d(1, 2, 3));

  dynamics::SpatialInertia I = dynamics::makeSpatialInertia(
      2.0, Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Matrix3d::Identity());

  dynamics::SpatialInertia result = dynamics::transformSpatialInertia(T, I);

  EXPECT_TRUE(result.isApprox(result.transpose(), 1e-10));
}
