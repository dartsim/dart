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

#include <dart/simulation/experimental/kinematics/joint_transform.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace dse = dart::simulation::experimental;
namespace kinematics = dse::kinematics;

//==============================================================================
// Fixed Joint Transform Tests
//==============================================================================

TEST(JointTransform, FixedReturnsIdentity)
{
  Eigen::Isometry3d T = kinematics::computeFixedTransform();

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, FixedHasNoRotation)
{
  Eigen::Isometry3d T = kinematics::computeFixedTransform();

  EXPECT_TRUE(T.rotation().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(JointTransform, FixedHasNoTranslation)
{
  Eigen::Isometry3d T = kinematics::computeFixedTransform();

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Revolute Joint Transform Tests
//==============================================================================

TEST(JointTransform, RevoluteZeroAngleIsIdentity)
{
  Eigen::Isometry3d T
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), 0.0);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, RevoluteHasNoTranslation)
{
  Eigen::Isometry3d T
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), M_PI_2);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST(JointTransform, Revolute90DegreesAroundZ)
{
  Eigen::Isometry3d T
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), M_PI_2);

  Eigen::Vector3d xAxis = T.rotation() * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(xAxis.isApprox(Eigen::Vector3d::UnitY(), 1e-10));

  Eigen::Vector3d yAxis = T.rotation() * Eigen::Vector3d::UnitY();
  EXPECT_TRUE(yAxis.isApprox(-Eigen::Vector3d::UnitX(), 1e-10));
}

TEST(JointTransform, Revolute180DegreesAroundZ)
{
  Eigen::Isometry3d T
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), M_PI);

  Eigen::Vector3d xAxis = T.rotation() * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(xAxis.isApprox(-Eigen::Vector3d::UnitX(), 1e-10));

  Eigen::Vector3d yAxis = T.rotation() * Eigen::Vector3d::UnitY();
  EXPECT_TRUE(yAxis.isApprox(-Eigen::Vector3d::UnitY(), 1e-10));
}

TEST(JointTransform, Revolute90DegreesAroundX)
{
  Eigen::Isometry3d T
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitX(), M_PI_2);

  Eigen::Vector3d yAxis = T.rotation() * Eigen::Vector3d::UnitY();
  EXPECT_TRUE(yAxis.isApprox(Eigen::Vector3d::UnitZ(), 1e-10));

  Eigen::Vector3d zAxis = T.rotation() * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(zAxis.isApprox(-Eigen::Vector3d::UnitY(), 1e-10));
}

TEST(JointTransform, Revolute90DegreesAroundY)
{
  Eigen::Isometry3d T
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitY(), M_PI_2);

  Eigen::Vector3d xAxis = T.rotation() * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(xAxis.isApprox(-Eigen::Vector3d::UnitZ(), 1e-10));

  Eigen::Vector3d zAxis = T.rotation() * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(zAxis.isApprox(Eigen::Vector3d::UnitX(), 1e-10));
}

TEST(JointTransform, RevoluteNonUnitAxisNormalized)
{
  Eigen::Vector3d axis(0.0, 0.0, 2.0);
  Eigen::Isometry3d T = kinematics::computeRevoluteTransform(axis, M_PI_2);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), M_PI_2);

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, RevoluteArbitraryAxis)
{
  Eigen::Vector3d axis(1.0, 1.0, 1.0);
  axis.normalize();
  double angle = M_PI * 2.0 / 3.0;

  Eigen::Isometry3d T = kinematics::computeRevoluteTransform(axis, angle);

  Eigen::Vector3d xAxis = T.rotation() * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(xAxis.isApprox(Eigen::Vector3d::UnitY(), 1e-10));

  Eigen::Vector3d yAxis = T.rotation() * Eigen::Vector3d::UnitY();
  EXPECT_TRUE(yAxis.isApprox(Eigen::Vector3d::UnitZ(), 1e-10));

  Eigen::Vector3d zAxis = T.rotation() * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(zAxis.isApprox(Eigen::Vector3d::UnitX(), 1e-10));
}

TEST(JointTransform, RevoluteFullRotation)
{
  Eigen::Isometry3d T = kinematics::computeRevoluteTransform(
      Eigen::Vector3d::UnitZ(), 2.0 * M_PI);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity(), 1e-10));
}

//==============================================================================
// Prismatic Joint Transform Tests
//==============================================================================

TEST(JointTransform, PrismaticZeroDisplacementIsIdentity)
{
  Eigen::Isometry3d T
      = kinematics::computePrismaticTransform(Eigen::Vector3d::UnitZ(), 0.0);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, PrismaticHasNoRotation)
{
  Eigen::Isometry3d T
      = kinematics::computePrismaticTransform(Eigen::Vector3d::UnitZ(), 1.5);

  EXPECT_TRUE(T.rotation().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(JointTransform, PrismaticTranslationAlongZ)
{
  double displacement = 2.5;
  Eigen::Isometry3d T = kinematics::computePrismaticTransform(
      Eigen::Vector3d::UnitZ(), displacement);

  EXPECT_TRUE(
      T.translation().isApprox(Eigen::Vector3d(0.0, 0.0, displacement), 1e-10));
}

TEST(JointTransform, PrismaticTranslationAlongX)
{
  double displacement = 3.0;
  Eigen::Isometry3d T = kinematics::computePrismaticTransform(
      Eigen::Vector3d::UnitX(), displacement);

  EXPECT_TRUE(
      T.translation().isApprox(Eigen::Vector3d(displacement, 0.0, 0.0), 1e-10));
}

TEST(JointTransform, PrismaticTranslationAlongY)
{
  double displacement = -1.5;
  Eigen::Isometry3d T = kinematics::computePrismaticTransform(
      Eigen::Vector3d::UnitY(), displacement);

  EXPECT_TRUE(
      T.translation().isApprox(Eigen::Vector3d(0.0, displacement, 0.0), 1e-10));
}

TEST(JointTransform, PrismaticNegativeDisplacement)
{
  double displacement = -4.0;
  Eigen::Isometry3d T = kinematics::computePrismaticTransform(
      Eigen::Vector3d::UnitZ(), displacement);

  EXPECT_TRUE(
      T.translation().isApprox(Eigen::Vector3d(0.0, 0.0, displacement), 1e-10));
}

TEST(JointTransform, PrismaticNonUnitAxisNormalized)
{
  Eigen::Vector3d axis(0.0, 3.0, 0.0);
  double displacement = 2.0;
  Eigen::Isometry3d T
      = kinematics::computePrismaticTransform(axis, displacement);

  EXPECT_TRUE(
      T.translation().isApprox(Eigen::Vector3d(0.0, displacement, 0.0), 1e-10));
}

TEST(JointTransform, PrismaticArbitraryAxis)
{
  Eigen::Vector3d axis(1.0, 1.0, 0.0);
  axis.normalize();
  double displacement = std::sqrt(2.0);

  Eigen::Isometry3d T
      = kinematics::computePrismaticTransform(axis, displacement);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d(1.0, 1.0, 0.0), 1e-10));
}

//==============================================================================
// Screw Joint Transform Tests
//==============================================================================

TEST(JointTransform, ScrewZeroAngleIsIdentity)
{
  Eigen::Isometry3d T
      = kinematics::computeScrewTransform(Eigen::Vector3d::UnitZ(), 0.1, 0.0);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, ScrewZeroPitchIsRevolute)
{
  double angle = M_PI_2;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();

  Eigen::Isometry3d screwT
      = kinematics::computeScrewTransform(axis, 0.0, angle);
  Eigen::Isometry3d revoluteT
      = kinematics::computeRevoluteTransform(axis, angle);

  EXPECT_TRUE(screwT.isApprox(revoluteT, 1e-10));
}

TEST(JointTransform, ScrewCouplesRotationAndTranslation)
{
  double pitch = 0.5;
  double angle = M_PI;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();

  Eigen::Isometry3d T = kinematics::computeScrewTransform(axis, pitch, angle);

  double expectedTranslation = pitch * angle;
  EXPECT_TRUE(T.translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, expectedTranslation), 1e-10));

  Eigen::Vector3d xAxis = T.rotation() * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(xAxis.isApprox(-Eigen::Vector3d::UnitX(), 1e-10));
}

TEST(JointTransform, ScrewAroundX)
{
  double pitch = 1.0;
  double angle = M_PI_2;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();

  Eigen::Isometry3d T = kinematics::computeScrewTransform(axis, pitch, angle);

  double expectedTranslation = pitch * angle;
  EXPECT_TRUE(T.translation().isApprox(
      Eigen::Vector3d(expectedTranslation, 0.0, 0.0), 1e-10));

  Eigen::Vector3d yAxis = T.rotation() * Eigen::Vector3d::UnitY();
  EXPECT_TRUE(yAxis.isApprox(Eigen::Vector3d::UnitZ(), 1e-10));
}

TEST(JointTransform, ScrewNegativePitch)
{
  double pitch = -0.25;
  double angle = 2.0 * M_PI;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();

  Eigen::Isometry3d T = kinematics::computeScrewTransform(axis, pitch, angle);

  double expectedTranslation = pitch * angle;
  EXPECT_NEAR(T.translation().z(), expectedTranslation, 1e-10);
}

TEST(JointTransform, ScrewFullRotation)
{
  double pitch = 0.1;
  double angle = 2.0 * M_PI;

  Eigen::Isometry3d T = kinematics::computeScrewTransform(
      Eigen::Vector3d::UnitZ(), pitch, angle);

  double expectedTranslation = pitch * angle;
  EXPECT_TRUE(T.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-10));
  EXPECT_NEAR(T.translation().z(), expectedTranslation, 1e-10);
}

//==============================================================================
// Universal Joint Transform Tests
//==============================================================================

TEST(JointTransform, UniversalZeroAnglesIsIdentity)
{
  Eigen::Isometry3d T = kinematics::computeUniversalTransform(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), 0.0, 0.0);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, UniversalHasNoTranslation)
{
  Eigen::Isometry3d T = kinematics::computeUniversalTransform(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), M_PI_4, M_PI_4);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST(JointTransform, UniversalFirstAxisOnly)
{
  Eigen::Isometry3d T = kinematics::computeUniversalTransform(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), M_PI_2, 0.0);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitX(), M_PI_2);

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, UniversalSecondAxisOnly)
{
  Eigen::Isometry3d T = kinematics::computeUniversalTransform(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), 0.0, M_PI_2);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitY(), M_PI_2);

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, UniversalBothAxes90Degrees)
{
  Eigen::Isometry3d T = kinematics::computeUniversalTransform(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), M_PI_2, M_PI_2);

  Eigen::Isometry3d R1
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitX(), M_PI_2);
  Eigen::Vector3d rotatedAxis2 = R1.rotation() * Eigen::Vector3d::UnitY();
  Eigen::Isometry3d R2
      = kinematics::computeRevoluteTransform(rotatedAxis2, M_PI_2);

  Eigen::Isometry3d expected = R2 * R1;

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, UniversalXYAxes)
{
  Eigen::Isometry3d T = kinematics::computeUniversalTransform(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), M_PI_4, M_PI_4);

  EXPECT_TRUE(T.rotation().isUnitary());

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST(JointTransform, UniversalZXAxes)
{
  Eigen::Isometry3d T = kinematics::computeUniversalTransform(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), M_PI_2, M_PI_2);

  EXPECT_TRUE(T.rotation().isUnitary());
}

//==============================================================================
// Ball Joint Transform Tests (Euler Angles)
//==============================================================================

TEST(JointTransform, BallZeroAnglesIsIdentity)
{
  Eigen::Isometry3d T
      = kinematics::computeBallTransform(Eigen::Vector3d::Zero());

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, BallHasNoTranslation)
{
  Eigen::Vector3d angles(M_PI_4, M_PI_4, M_PI_4);
  Eigen::Isometry3d T = kinematics::computeBallTransform(angles);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST(JointTransform, BallRollOnly)
{
  Eigen::Vector3d angles(M_PI_2, 0.0, 0.0);
  Eigen::Isometry3d T = kinematics::computeBallTransform(angles);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitX(), M_PI_2);

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, BallPitchOnly)
{
  Eigen::Vector3d angles(0.0, M_PI_2, 0.0);
  Eigen::Isometry3d T = kinematics::computeBallTransform(angles);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitY(), M_PI_2);

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, BallYawOnly)
{
  Eigen::Vector3d angles(0.0, 0.0, M_PI_2);
  Eigen::Isometry3d T = kinematics::computeBallTransform(angles);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), M_PI_2);

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, BallZYXOrder)
{
  Eigen::Vector3d angles(M_PI_4, M_PI_4, M_PI_4);
  Eigen::Isometry3d T = kinematics::computeBallTransform(angles);

  Eigen::Matrix3d expected
      = (Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitX()))
            .toRotationMatrix();

  EXPECT_TRUE(T.rotation().isApprox(expected, 1e-10));
}

TEST(JointTransform, BallRotationIsOrthonormal)
{
  Eigen::Vector3d angles(0.3, 0.5, 0.7);
  Eigen::Isometry3d T = kinematics::computeBallTransform(angles);

  EXPECT_TRUE(T.rotation().isUnitary());

  EXPECT_NEAR(T.rotation().determinant(), 1.0, 1e-10);
}

//==============================================================================
// Ball Joint Transform Tests (Quaternion)
//==============================================================================

TEST(JointTransform, BallQuatIdentity)
{
  Eigen::Isometry3d T
      = kinematics::computeBallTransformQuat(Eigen::Quaterniond::Identity());

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, BallQuatHasNoTranslation)
{
  Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX());

  Eigen::Isometry3d T = kinematics::computeBallTransformQuat(q);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST(JointTransform, BallQuat90AroundZ)
{
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d T = kinematics::computeBallTransformQuat(q);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), M_PI_2);

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, BallQuatNormalized)
{
  Eigen::Quaterniond q(2.0, 0.0, 0.0, 0.0);

  Eigen::Isometry3d T = kinematics::computeBallTransformQuat(q);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity(), 1e-10));
}

TEST(JointTransform, BallQuatArbitrary)
{
  Eigen::Quaterniond q(
      Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()));

  Eigen::Isometry3d T = kinematics::computeBallTransformQuat(q);

  EXPECT_TRUE(T.rotation().isApprox(q.normalized().toRotationMatrix(), 1e-10));
}

//==============================================================================
// Planar Joint Transform Tests
//==============================================================================

TEST(JointTransform, PlanarZeroIsIdentity)
{
  Eigen::Isometry3d T = kinematics::computePlanarTransform(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 0.0, 0.0, 0.0);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, PlanarTranslationX)
{
  double x = 2.5;
  Eigen::Isometry3d T = kinematics::computePlanarTransform(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), x, 0.0, 0.0);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d(x, 0.0, 0.0), 1e-10));
  EXPECT_TRUE(T.rotation().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(JointTransform, PlanarTranslationY)
{
  double y = -1.5;
  Eigen::Isometry3d T = kinematics::computePlanarTransform(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 0.0, y, 0.0);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d(0.0, y, 0.0), 1e-10));
}

TEST(JointTransform, PlanarRotation)
{
  double theta = M_PI_2;
  Eigen::Isometry3d T = kinematics::computePlanarTransform(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 0.0, 0.0, theta);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), theta);

  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, PlanarCombined)
{
  double x = 1.0;
  double y = 2.0;
  double theta = M_PI_4;

  Eigen::Isometry3d T = kinematics::computePlanarTransform(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), x, y, theta);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d(x, y, 0.0), 1e-10));

  Eigen::Matrix3d expectedRotation
      = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(T.rotation().isApprox(expectedRotation, 1e-10));
}

TEST(JointTransform, PlanarXYPlane)
{
  double x = 3.0;
  double y = 4.0;
  double theta = M_PI;

  Eigen::Isometry3d T = kinematics::computePlanarTransform(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), x, y, theta);

  EXPECT_NEAR(T.translation().z(), 0.0, 1e-10);
}

TEST(JointTransform, PlanarXZPlane)
{
  double x = 1.0;
  double y = 2.0;
  double theta = M_PI_2;

  Eigen::Isometry3d T = kinematics::computePlanarTransform(
      Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX(), x, y, theta);

  EXPECT_TRUE(T.translation().isApprox(
      x * Eigen::Vector3d::UnitX() + y * (-Eigen::Vector3d::UnitZ()), 1e-10));
}

TEST(JointTransform, PlanarYZPlane)
{
  double x = 1.5;
  double y = 2.5;
  double theta = M_PI_4;

  Eigen::Isometry3d T = kinematics::computePlanarTransform(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), x, y, theta);

  EXPECT_NEAR(T.translation().x(), 0.0, 1e-10);
}

//==============================================================================
// Free Joint Transform Tests (6-DOF Vector)
//==============================================================================

TEST(JointTransform, FreeZeroIsIdentity)
{
  Eigen::Vector<double, 6> pos = Eigen::Vector<double, 6>::Zero();
  Eigen::Isometry3d T = kinematics::computeFreeTransform(pos);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, FreeTranslationOnly)
{
  Eigen::Vector<double, 6> pos;
  pos << 1.0, 2.0, 3.0, 0.0, 0.0, 0.0;

  Eigen::Isometry3d T = kinematics::computeFreeTransform(pos);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 1e-10));
  EXPECT_TRUE(T.rotation().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(JointTransform, FreeRotationOnly)
{
  Eigen::Vector<double, 6> pos;
  pos << 0.0, 0.0, 0.0, M_PI_2, 0.0, 0.0;

  Eigen::Isometry3d T = kinematics::computeFreeTransform(pos);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d::Zero()));

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitX(), M_PI_2);
  EXPECT_TRUE(T.rotation().isApprox(expected.rotation(), 1e-10));
}

TEST(JointTransform, FreeCombined)
{
  Eigen::Vector<double, 6> pos;
  pos << 1.0, 2.0, 3.0, M_PI_4, M_PI_4, M_PI_4;

  Eigen::Isometry3d T = kinematics::computeFreeTransform(pos);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 1e-10));

  Eigen::Vector3d euler(M_PI_4, M_PI_4, M_PI_4);
  Eigen::Isometry3d ballT = kinematics::computeBallTransform(euler);
  EXPECT_TRUE(T.rotation().isApprox(ballT.rotation(), 1e-10));
}

TEST(JointTransform, FreeNegativeValues)
{
  Eigen::Vector<double, 6> pos;
  pos << -1.0, -2.0, -3.0, -M_PI_4, -M_PI_4, -M_PI_4;

  Eigen::Isometry3d T = kinematics::computeFreeTransform(pos);

  EXPECT_TRUE(
      T.translation().isApprox(Eigen::Vector3d(-1.0, -2.0, -3.0), 1e-10));
}

//==============================================================================
// Free Joint Transform Tests (Separate Components)
//==============================================================================

TEST(JointTransform, FreeSeparateZeroIsIdentity)
{
  Eigen::Isometry3d T = kinematics::computeFreeTransform(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(JointTransform, FreeSeparateTranslationOnly)
{
  Eigen::Vector3d trans(5.0, 6.0, 7.0);
  Eigen::Vector3d euler = Eigen::Vector3d::Zero();

  Eigen::Isometry3d T = kinematics::computeFreeTransform(trans, euler);

  EXPECT_TRUE(T.translation().isApprox(trans, 1e-10));
  EXPECT_TRUE(T.rotation().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(JointTransform, FreeSeparateRotationOnly)
{
  Eigen::Vector3d trans = Eigen::Vector3d::Zero();
  Eigen::Vector3d euler(0.1, 0.2, 0.3);

  Eigen::Isometry3d T = kinematics::computeFreeTransform(trans, euler);

  EXPECT_TRUE(T.translation().isApprox(Eigen::Vector3d::Zero()));

  Eigen::Isometry3d ballT = kinematics::computeBallTransform(euler);
  EXPECT_TRUE(T.rotation().isApprox(ballT.rotation(), 1e-10));
}

TEST(JointTransform, FreeSeparateMatchesVector)
{
  Eigen::Vector3d trans(1.0, 2.0, 3.0);
  Eigen::Vector3d euler(0.1, 0.2, 0.3);

  Eigen::Isometry3d T1 = kinematics::computeFreeTransform(trans, euler);

  Eigen::Vector<double, 6> pos;
  pos << trans, euler;
  Eigen::Isometry3d T2 = kinematics::computeFreeTransform(pos);

  EXPECT_TRUE(T1.isApprox(T2, 1e-10));
}

//==============================================================================
// computeJointTransform Generic Dispatch Tests
//==============================================================================

TEST(JointTransform, GenericFixedJoint)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Fixed;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  EXPECT_TRUE(T.isApprox(kinematics::computeFixedTransform()));
}

TEST(JointTransform, GenericRevoluteJoint)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Revolute;
  joint.axis = Eigen::Vector3d::UnitZ();
  joint.position = Eigen::VectorXd(1);
  joint.position << M_PI_2;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  Eigen::Isometry3d expected
      = kinematics::computeRevoluteTransform(joint.axis, M_PI_2);
  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, GenericPrismaticJoint)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Prismatic;
  joint.axis = Eigen::Vector3d::UnitY();
  joint.position = Eigen::VectorXd(1);
  joint.position << 2.5;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  Eigen::Isometry3d expected
      = kinematics::computePrismaticTransform(joint.axis, 2.5);
  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, GenericScrewJoint)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Screw;
  joint.axis = Eigen::Vector3d::UnitZ();
  joint.pitch = 0.5;
  joint.position = Eigen::VectorXd(1);
  joint.position << M_PI;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  Eigen::Isometry3d expected
      = kinematics::computeScrewTransform(joint.axis, joint.pitch, M_PI);
  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, GenericUniversalJoint)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Universal;
  joint.axis = Eigen::Vector3d::UnitX();
  joint.axis2 = Eigen::Vector3d::UnitY();
  joint.position = Eigen::VectorXd(2);
  joint.position << M_PI_4, M_PI_4;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  Eigen::Isometry3d expected = kinematics::computeUniversalTransform(
      joint.axis, joint.axis2, M_PI_4, M_PI_4);
  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, GenericBallJoint)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Ball;
  joint.position = Eigen::VectorXd(3);
  joint.position << 0.1, 0.2, 0.3;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  Eigen::Isometry3d expected
      = kinematics::computeBallTransform(joint.position.head<3>());
  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, GenericPlanarJoint)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Planar;
  joint.axis = Eigen::Vector3d::UnitZ();
  joint.axis2 = Eigen::Vector3d::UnitX();
  joint.position = Eigen::VectorXd(3);
  joint.position << 1.0, 2.0, M_PI_4;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  Eigen::Isometry3d expected = kinematics::computePlanarTransform(
      joint.axis, joint.axis2, 1.0, 2.0, M_PI_4);
  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, GenericFreeJoint)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Free;
  joint.position = Eigen::VectorXd(6);
  joint.position << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  Eigen::Vector<double, 6> pos6(joint.position.head<6>());
  Eigen::Isometry3d expected = kinematics::computeFreeTransform(pos6);
  EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

TEST(JointTransform, GenericCustomJointReturnsIdentity)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Custom;

  Eigen::Isometry3d T = kinematics::computeJointTransform(joint);

  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));
}

//==============================================================================
// Transform Property Tests
//==============================================================================

TEST(JointTransform, TransformIsRigid)
{
  Eigen::Vector<double, 6> pos;
  pos << 1.0, 2.0, 3.0, 0.3, 0.5, 0.7;

  Eigen::Isometry3d T = kinematics::computeFreeTransform(pos);

  EXPECT_TRUE(T.rotation().isUnitary());
  EXPECT_NEAR(T.rotation().determinant(), 1.0, 1e-10);
}

TEST(JointTransform, TransformComposition)
{
  Eigen::Isometry3d T1
      = kinematics::computeRevoluteTransform(Eigen::Vector3d::UnitZ(), M_PI_4);
  Eigen::Isometry3d T2
      = kinematics::computePrismaticTransform(Eigen::Vector3d::UnitX(), 1.0);

  Eigen::Isometry3d T_composed = T1 * T2;

  EXPECT_TRUE(T_composed.rotation().isUnitary());
}

TEST(JointTransform, InverseTransform)
{
  Eigen::Vector<double, 6> pos;
  pos << 1.0, 2.0, 3.0, 0.3, 0.5, 0.7;

  Eigen::Isometry3d T = kinematics::computeFreeTransform(pos);
  Eigen::Isometry3d T_inv = T.inverse();

  Eigen::Isometry3d identity = T * T_inv;

  EXPECT_TRUE(identity.isApprox(Eigen::Isometry3d::Identity(), 1e-10));
}
