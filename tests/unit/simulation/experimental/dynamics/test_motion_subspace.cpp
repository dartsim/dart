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

#include <dart/simulation/experimental/dynamics/motion_subspace.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace dse = dart::simulation::experimental;
namespace dynamics = dse::dynamics;

//==============================================================================
// Fixed Joint Motion Subspace Tests
//==============================================================================

TEST(MotionSubspace, FixedHasZeroColumns)
{
  auto S = dynamics::computeFixedMotionSubspace();

  EXPECT_EQ(S.rows(), 6);
  EXPECT_EQ(S.cols(), 0);
}

//==============================================================================
// Revolute Joint Motion Subspace Tests
//==============================================================================

TEST(MotionSubspace, RevoluteZAxisHasCorrectStructure)
{
  auto S = dynamics::computeRevoluteMotionSubspace(Eigen::Vector3d::UnitZ());

  EXPECT_EQ(S.rows(), 6);
  EXPECT_EQ(S.cols(), 1);

  EXPECT_TRUE(S.head<3>().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(S.tail<3>().isApprox(Eigen::Vector3d::Zero()));
}

TEST(MotionSubspace, RevoluteXAxisHasCorrectStructure)
{
  auto S = dynamics::computeRevoluteMotionSubspace(Eigen::Vector3d::UnitX());

  EXPECT_TRUE(S.head<3>().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(S.tail<3>().isApprox(Eigen::Vector3d::Zero()));
}

TEST(MotionSubspace, RevoluteArbitraryAxis)
{
  Eigen::Vector3d axis(1.0, 2.0, 3.0);
  axis.normalize();

  auto S = dynamics::computeRevoluteMotionSubspace(axis);

  EXPECT_TRUE(S.head<3>().isApprox(axis));
  EXPECT_TRUE(S.tail<3>().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Prismatic Joint Motion Subspace Tests
//==============================================================================

TEST(MotionSubspace, PrismaticZAxisHasCorrectStructure)
{
  auto S = dynamics::computePrismaticMotionSubspace(Eigen::Vector3d::UnitZ());

  EXPECT_EQ(S.rows(), 6);
  EXPECT_EQ(S.cols(), 1);

  EXPECT_TRUE(S.head<3>().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(S.tail<3>().isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(MotionSubspace, PrismaticYAxisHasCorrectStructure)
{
  auto S = dynamics::computePrismaticMotionSubspace(Eigen::Vector3d::UnitY());

  EXPECT_TRUE(S.head<3>().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(S.tail<3>().isApprox(Eigen::Vector3d::UnitY()));
}

//==============================================================================
// Screw Joint Motion Subspace Tests
//==============================================================================

TEST(MotionSubspace, ScrewZeroPitchEqualsRevolute)
{
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();

  auto S_screw = dynamics::computeScrewMotionSubspace(axis, 0.0);
  auto S_revolute = dynamics::computeRevoluteMotionSubspace(axis);

  EXPECT_TRUE(S_screw.isApprox(S_revolute));
}

TEST(MotionSubspace, ScrewHasCoupledRotationTranslation)
{
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  double pitch = 0.5;

  auto S = dynamics::computeScrewMotionSubspace(axis, pitch);

  EXPECT_TRUE(S.head<3>().isApprox(axis));
  EXPECT_TRUE(S.tail<3>().isApprox(pitch * axis));
}

//==============================================================================
// Universal Joint Motion Subspace Tests
//==============================================================================

TEST(MotionSubspace, UniversalHasTwoColumns)
{
  auto S = dynamics::computeUniversalMotionSubspace(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), 0.0);

  EXPECT_EQ(S.rows(), 6);
  EXPECT_EQ(S.cols(), 2);
}

TEST(MotionSubspace, UniversalFirstColumnIsAxis1)
{
  Eigen::Vector3d axis1 = Eigen::Vector3d::UnitX();
  auto S = dynamics::computeUniversalMotionSubspace(
      axis1, Eigen::Vector3d::UnitY(), 0.0);

  EXPECT_TRUE(S.col(0).head<3>().isApprox(axis1));
  EXPECT_TRUE(S.col(0).tail<3>().isApprox(Eigen::Vector3d::Zero()));
}

TEST(MotionSubspace, UniversalSecondColumnRotatesWithAngle1)
{
  Eigen::Vector3d axis1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitY();
  double angle1 = M_PI_2;

  auto S = dynamics::computeUniversalMotionSubspace(axis1, axis2, angle1);

  Eigen::AngleAxisd rotation(angle1, axis1);
  Eigen::Vector3d expectedAxis2 = rotation * axis2;

  EXPECT_TRUE(S.col(1).head<3>().isApprox(expectedAxis2, 1e-10));
  EXPECT_TRUE(S.col(1).tail<3>().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Ball Joint Motion Subspace Tests
//==============================================================================

TEST(MotionSubspace, BallHasThreeColumns)
{
  auto S = dynamics::computeBallMotionSubspace();

  EXPECT_EQ(S.rows(), 6);
  EXPECT_EQ(S.cols(), 3);
}

TEST(MotionSubspace, BallTopBlockIsIdentity)
{
  auto S = dynamics::computeBallMotionSubspace();

  EXPECT_TRUE(S.topRows<3>().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(MotionSubspace, BallBottomBlockIsZero)
{
  auto S = dynamics::computeBallMotionSubspace();

  EXPECT_TRUE(S.bottomRows<3>().isApprox(Eigen::Matrix3d::Zero()));
}

//==============================================================================
// Planar Joint Motion Subspace Tests
//==============================================================================

TEST(MotionSubspace, PlanarHasThreeColumns)
{
  auto S = dynamics::computePlanarMotionSubspace(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());

  EXPECT_EQ(S.rows(), 6);
  EXPECT_EQ(S.cols(), 3);
}

TEST(MotionSubspace, PlanarFirstColumnIsRotation)
{
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  auto S
      = dynamics::computePlanarMotionSubspace(normal, Eigen::Vector3d::UnitX());

  EXPECT_TRUE(S.col(0).head<3>().isApprox(normal));
  EXPECT_TRUE(S.col(0).tail<3>().isApprox(Eigen::Vector3d::Zero()));
}

TEST(MotionSubspace, PlanarSecondThirdColumnsAreTranslations)
{
  auto S = dynamics::computePlanarMotionSubspace(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());

  EXPECT_TRUE(S.col(1).head<3>().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(S.col(2).head<3>().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(S.col(1).tail<3>().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(S.col(2).tail<3>().isApprox(Eigen::Vector3d::UnitY()));
}

//==============================================================================
// Free Joint Motion Subspace Tests
//==============================================================================

TEST(MotionSubspace, FreeHasSixColumns)
{
  auto S = dynamics::computeFreeMotionSubspace();

  EXPECT_EQ(S.rows(), 6);
  EXPECT_EQ(S.cols(), 6);
}

TEST(MotionSubspace, FreeIsIdentity)
{
  auto S = dynamics::computeFreeMotionSubspace();

  EXPECT_TRUE(S.isApprox(Eigen::Matrix6d::Identity()));
}

//==============================================================================
// Generic Dispatch Tests
//==============================================================================

TEST(MotionSubspace, DispatchFixed)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Fixed;

  auto S = dynamics::computeMotionSubspace(joint);

  EXPECT_EQ(S.cols(), 0);
}

TEST(MotionSubspace, DispatchRevolute)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Revolute;
  joint.axis = Eigen::Vector3d::UnitZ();

  auto S = dynamics::computeMotionSubspace(joint);

  EXPECT_EQ(S.cols(), 1);
  EXPECT_TRUE(S.isApprox(dynamics::computeRevoluteMotionSubspace(joint.axis)));
}

TEST(MotionSubspace, DispatchPrismatic)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Prismatic;
  joint.axis = Eigen::Vector3d::UnitX();

  auto S = dynamics::computeMotionSubspace(joint);

  EXPECT_EQ(S.cols(), 1);
  EXPECT_TRUE(S.isApprox(dynamics::computePrismaticMotionSubspace(joint.axis)));
}

TEST(MotionSubspace, DispatchBall)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Ball;

  auto S = dynamics::computeMotionSubspace(joint);

  EXPECT_EQ(S.cols(), 3);
  EXPECT_TRUE(S.isApprox(dynamics::computeBallMotionSubspace()));
}

TEST(MotionSubspace, DispatchFree)
{
  dse::comps::Joint joint;
  joint.type = dse::comps::JointType::Free;

  auto S = dynamics::computeMotionSubspace(joint);

  EXPECT_EQ(S.cols(), 6);
  EXPECT_TRUE(S.isApprox(dynamics::computeFreeMotionSubspace()));
}
