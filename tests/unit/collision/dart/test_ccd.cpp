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

#include <dart/collision/dart/Types.hpp>
#include <dart/collision/dart/narrow_phase/Ccd.hpp>
#include <dart/collision/dart/narrow_phase/NarrowPhase.hpp>
#include <dart/collision/dart/shapes/Shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

using namespace dart::collision::native;

//==============================================================================
// Sphere-cast Sphere tests
//==============================================================================

TEST(SphereCastSphere, Miss)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 0);

  Eigen::Vector3d start(10, 0, 0);
  Eigen::Vector3d end(10, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastSphere(start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
  EXPECT_FALSE(result.isHit());
}

TEST(SphereCastSphere, DegenerateQuadraticBranches)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  constexpr double radius = 0.5;

  CcdOption option;

  {
    CcdResult result;
    EXPECT_FALSE(sphereCastSphere(
        Eigen::Vector3d(5.0, 0.0, 0.0),
        Eigen::Vector3d(5.0, 0.0, 0.0),
        radius,
        target,
        transform,
        option,
        result));
  }

  {
    CcdResult result;
    EXPECT_FALSE(sphereCastSphere(
        Eigen::Vector3d(5.0, 0.0, 0.0),
        Eigen::Vector3d(5.0 - 1e-6, 0.0, 0.0),
        radius,
        target,
        transform,
        option,
        result));
  }

  {
    CcdResult result;
    EXPECT_FALSE(sphereCastSphere(
        Eigen::Vector3d(5.0, 0.0, 0.0),
        Eigen::Vector3d(6.0, 0.0, 0.0),
        radius,
        target,
        transform,
        option,
        result));
  }
}

TEST(SphereCastSphere, DirectHit)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastSphere(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_NEAR(result.timeOfImpact, 0.35, 1e-6);
  EXPECT_NEAR(result.point.z(), 4.0, 1e-6);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-6);
}

TEST(SphereCastSphere, StartingInside)
{
  SphereShape target(2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastSphere(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-6);
}

TEST(SphereCastSphere, GrazingHit)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.4, 0, 5);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastSphere(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(SphereCastSphere, JustMissing)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.6, 0, 5);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastSphere(start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(SphereCastSphere, StationarySweep)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0.5, 0, 0);
  Eigen::Vector3d end(0.5, 0, 0);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastSphere(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-6);
}

TEST(SphereCastSphere, StationaryTouching)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(1.5, 0, 0);
  Eigen::Vector3d end(1.5, 0, 0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastSphere(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-6);
  EXPECT_NEAR(result.normal.x(), 1.0, 1e-6);
}

//==============================================================================
// Sphere-cast Box tests
//==============================================================================

TEST(SphereCastBox, Miss)
{
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(10, 0, 0);
  Eigen::Vector3d end(10, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastBox(start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(SphereCastBox, SlabMissAndInitialOverlapNormal)
{
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  CcdOption option;

  {
    CcdResult result;
    EXPECT_FALSE(sphereCastBox(
        Eigen::Vector3d(-5.0, 2.0, 0.0),
        Eigen::Vector3d(5.0, 1.5, 0.0),
        0.1,
        target,
        transform,
        option,
        result));
  }

  {
    CcdResult result;
    ASSERT_TRUE(sphereCastBox(
        Eigen::Vector3d(1.2, 0.0, 0.0),
        Eigen::Vector3d(1.2, 0.0, 0.0),
        0.5,
        target,
        transform,
        option,
        result));
    EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
    EXPECT_NEAR(result.normal.x(), 1.0, 1e-12);
  }
}

TEST(SphereCastBox, InflatedCornerStationaryMiss)
{
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d center(1.4, 1.4, 0.0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  EXPECT_FALSE(
      sphereCastBox(center, center, radius, target, transform, option, result));
}

TEST(SphereCastBox, InflatedCornerSweepMiss)
{
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(1.4, 1.4, -3.0);
  Eigen::Vector3d end(1.4, 1.4, 3.0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  EXPECT_FALSE(
      sphereCastBox(start, end, radius, target, transform, option, result));
}

TEST(SphereCastBox, HitFrontFace)
{
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, -5);
  Eigen::Vector3d end(0, 0, 5);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastBox(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-6);
}

TEST(SphereCastBox, HitSideFace)
{
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(-5, 0, 0);
  Eigen::Vector3d end(5, 0, 0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastBox(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-6);
}

TEST(SphereCastBox, StartingInside)
{
  BoxShape target(Eigen::Vector3d(2, 2, 2));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastBox(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-6);
}

TEST(SphereCastBox, RotatedBox)
{
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(
      3.141592653589793238462643383279502884 / 4, Eigen::Vector3d::UnitY()));

  Eigen::Vector3d start(0, 0, -5);
  Eigen::Vector3d end(0, 0, 5);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastBox(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(SphereCastBox, TranslatedBox)
{
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 7);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastBox(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.55, 1e-6);
}

//==============================================================================
// Sphere-cast Capsule tests
//==============================================================================

TEST(SphereCastCapsule, Miss)
{
  CapsuleShape target(0.5, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(10, 0, 0);
  Eigen::Vector3d end(10, 0, 10);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCapsule(
      start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(SphereCastCapsule, HitCylindricalPart)
{
  CapsuleShape target(1.0, 4.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(-5, 0, 0);
  Eigen::Vector3d end(5, 0, 0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCapsule(
      start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-6);
}

TEST(SphereCastCapsule, StartingInsideCylindricalPart)
{
  CapsuleShape target(1.0, 4.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  CcdOption option;

  {
    CcdResult result;
    ASSERT_TRUE(sphereCastCapsule(
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
        0.5,
        target,
        transform,
        option,
        result));
    EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
  }

  {
    CcdResult result;
    ASSERT_TRUE(sphereCastCapsule(
        Eigen::Vector3d(0.5, 0.0, -1.0),
        Eigen::Vector3d(0.5, 0.0, 1.0),
        0.5,
        target,
        transform,
        option,
        result));
    EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
  }

  {
    CcdResult result;
    ASSERT_TRUE(sphereCastCapsule(
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(3.0, 0.0, 0.0),
        0.5,
        target,
        transform,
        option,
        result));
    EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
  }
}

TEST(SphereCastCapsule, HitSphericalCap)
{
  CapsuleShape target(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, -5);
  Eigen::Vector3d end(0, 0, 5);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCapsule(
      start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-6);
}

TEST(SphereCastCapsule, TranslatedCapsule)
{
  CapsuleShape target(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCapsule(
      start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
}

TEST(SphereCastCapsule, RotatedCapsule)
{
  CapsuleShape target(0.5, 4.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(
      3.141592653589793238462643383279502884 / 2, Eigen::Vector3d::UnitX()));

  Eigen::Vector3d start(0, -5, 0);
  Eigen::Vector3d end(0, 5, 0);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCapsule(
      start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

//==============================================================================
// Sphere-cast Plane tests
//==============================================================================

TEST(SphereCastPlane, Miss_ParallelMotion)
{
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, 5);
  Eigen::Vector3d end(10, 0, 5);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastPlane(start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(SphereCastPlane, Miss_MovingAway)
{
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, 5);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastPlane(start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(SphereCastPlane, HitFromAbove)
{
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, 5);
  Eigen::Vector3d end(0, 0, -5);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastPlane(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.45, 1e-6);
  EXPECT_NEAR(result.point.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-6);
}

TEST(SphereCastPlane, StartingBelow)
{
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, -1);
  Eigen::Vector3d end(0, 0, 5);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastPlane(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-6);
}

TEST(SphereCastPlane, PlaneWithOffset)
{
  PlaneShape target(Eigen::Vector3d::UnitZ(), 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, 10);
  Eigen::Vector3d end(0, 0, 0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastPlane(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.75, 1e-6);
  EXPECT_NEAR(result.point.z(), 2.0, 1e-6);
}

TEST(SphereCastPlane, TransformedPlane)
{
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 3);

  Eigen::Vector3d start(0, 0, 10);
  Eigen::Vector3d end(0, 0, 0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastPlane(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.65, 1e-6);
  EXPECT_NEAR(result.point.z(), 3.0, 1e-6);
}

TEST(SphereCastPlane, RotatedPlane)
{
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(
      3.141592653589793238462643383279502884 / 2, Eigen::Vector3d::UnitX()));

  Eigen::Vector3d start(0, -5, 0);
  Eigen::Vector3d end(0, 5, 0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastPlane(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

//==============================================================================
// Sphere-cast Cylinder tests
//==============================================================================

TEST(SphereCastCylinder, Miss)
{
  CylinderShape target(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(10, 0, 0);
  Eigen::Vector3d end(10, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCylinder(
      start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(SphereCastCylinder, HitCurvedSurface)
{
  CylinderShape target(1.0, 4.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(-5, 0, 0);
  Eigen::Vector3d end(5, 0, 0);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCylinder(
      start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-6);
}

TEST(SphereCastCylinder, HitTopCap)
{
  CylinderShape target(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, 5);
  Eigen::Vector3d end(0, 0, -5);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCylinder(
      start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-6);
}

TEST(SphereCastCylinder, HitBottomCap)
{
  CylinderShape target(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(0, 0, -5);
  Eigen::Vector3d end(0, 0, 5);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit = sphereCastCylinder(
      start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-6);
}

TEST(SphereCastCylinder, StartingInside)
{
  CylinderShape target(1.0, 4.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  CcdOption option;

  {
    CcdResult result;
    ASSERT_TRUE(sphereCastCylinder(
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
        0.5,
        target,
        transform,
        option,
        result));
    EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
  }

  {
    CcdResult result;
    ASSERT_TRUE(sphereCastCylinder(
        Eigen::Vector3d(0.5, 0.0, -1.0),
        Eigen::Vector3d(0.5, 0.0, 1.0),
        0.5,
        target,
        transform,
        option,
        result));
    EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
  }
}

TEST(SphereCastCylinder, HitCapRim)
{
  CylinderShape target(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;
  ASSERT_TRUE(sphereCastCylinder(
      Eigen::Vector3d(1.3, 0.0, 5.0),
      Eigen::Vector3d(1.3, 0.0, -5.0),
      0.5,
      target,
      transform,
      option,
      result));

  EXPECT_NEAR(result.timeOfImpact, 0.36, 0.05);
  EXPECT_TRUE(result.point.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
}

//==============================================================================
// Sphere-cast Convex tests
//==============================================================================

TEST(SphereCastConvex, Miss)
{
  std::vector<Eigen::Vector3d> vertices
      = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  ConvexShape target(vertices);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(10, 0, 0);
  Eigen::Vector3d end(10, 0, 10);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastConvex(start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(SphereCastConvex, DirectHit)
{
  std::vector<Eigen::Vector3d> vertices
      = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  ConvexShape target(vertices);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(-5, 0, 0);
  Eigen::Vector3d end(5, 0, 0);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastConvex(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.3);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-6);
}

TEST(SphereCastConvex, TranslatedTarget)
{
  std::vector<Eigen::Vector3d> vertices
      = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  ConvexShape target(vertices);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastConvex(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.3);
  EXPECT_LT(result.timeOfImpact, 0.5);
}

TEST(SphereCastConvex, StationaryOverlapUsesFallbackDirections)
{
  ConvexShape target(
      {{-1, -1, -1},
       {1, -1, -1},
       {1, 1, -1},
       {-1, 1, -1},
       {-1, -1, 1},
       {1, -1, 1},
       {1, 1, 1},
       {-1, 1, 1}});
  const Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;
  ASSERT_TRUE(sphereCastConvex(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      0.25,
      target,
      transform,
      option,
      result));
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
  EXPECT_TRUE(result.point.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
}

//==============================================================================
// Sphere-cast Mesh tests
//==============================================================================

TEST(SphereCastMesh, Miss)
{
  std::vector<Eigen::Vector3d> vertices
      = {{0, 0, 0}, {2, 0, 0}, {1, 2, 0}, {1, 1, 2}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  MeshShape target(vertices, triangles);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(10, 0, 0);
  Eigen::Vector3d end(10, 0, 10);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastMesh(start, end, radius, target, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(SphereCastMesh, DirectHit)
{
  std::vector<Eigen::Vector3d> vertices
      = {{-1, -1, -1}, {1, -1, -1}, {0, 1, -1}, {0, 0, 1}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  MeshShape target(vertices, triangles);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d start(-5, 0, 0);
  Eigen::Vector3d end(5, 0, 0);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastMesh(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(SphereCastMesh, TranslatedTarget)
{
  std::vector<Eigen::Vector3d> vertices
      = {{0, 0, 0}, {2, 0, 0}, {1, 2, 0}, {1, 1, 2}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  MeshShape target(vertices, triangles);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  Eigen::Vector3d start(1, 1, 0);
  Eigen::Vector3d end(1, 1, 10);
  double radius = 0.25;

  CcdOption option;
  CcdResult result;

  bool hit
      = sphereCastMesh(start, end, radius, target, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.3);
  EXPECT_LT(result.timeOfImpact, 0.6);
}

//==============================================================================
// Capsule-cast Sphere tests
//==============================================================================

TEST(CapsuleCastSphere, Miss)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape target(1.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(10, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(10, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastSphere(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(CapsuleCastSphere, DirectHit)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape target(1.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(0, 0, 5);

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(0, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastSphere(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(CapsuleCastSphere, RotatingCapsule)
{
  CapsuleShape capsule(0.5, 4.0);
  SphereShape target(1.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(3, 0, 0);

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.rotate(Eigen::AngleAxisd(
      3.141592653589793238462643383279502884 / 2, Eigen::Vector3d::UnitY()));

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastSphere(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(CapsuleCastSphere, SideOfCapsuleBodyHit)
{
  // A long thin capsule (axis along Z) slides sideways in -X past a sphere
  // aligned with its mid-height. The sphere strikes the cylindrical body far
  // from both spherical caps -- a contact a caps-only endpoint sweep misses.
  CapsuleShape capsule(0.2, 4.0); // radius 0.2, half-height 2.0
  SphereShape target(0.3);
  const Eigen::Isometry3d targetTransform
      = Eigen::Isometry3d::Identity(); // sphere at origin

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(3, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(-3, 0, 0);

  CcdOption option;
  CcdResult result;

  const bool hit = capsuleCastSphere(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  // Contact when |x| reaches capsuleRadius + sphereRadius = 0.5; with
  // x(t) = 3 - 6t that is t ~ 0.417.
  EXPECT_NEAR(result.timeOfImpact, 0.417, 0.05);
}

//==============================================================================
// Capsule-cast Box tests
//==============================================================================

TEST(CapsuleCastBox, Miss)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(10, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(10, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastBox(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(CapsuleCastBox, DirectHit)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape target(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(0, 0, -5);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 5);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastBox(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
}

//==============================================================================
// Capsule-cast Capsule tests
//==============================================================================

TEST(CapsuleCastCapsule, Miss)
{
  CapsuleShape capsule(0.5, 2.0);
  CapsuleShape target(1.0, 2.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(10, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(10, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastCapsule(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(CapsuleCastCapsule, DirectHit)
{
  CapsuleShape capsule(0.5, 2.0);
  CapsuleShape target(1.0, 2.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(5, 0, 0);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastCapsule(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-6);
}

//==============================================================================
// Capsule-cast Plane tests
//==============================================================================

TEST(CapsuleCastPlane, Miss)
{
  CapsuleShape capsule(0.5, 2.0);
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(0, 0, 5);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastPlane(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(CapsuleCastPlane, DirectHit)
{
  CapsuleShape capsule(0.5, 2.0);
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(0, 0, 5);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, -5);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastPlane(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(CapsuleCastPlane, TiltedCapsule)
{
  CapsuleShape capsule(0.5, 4.0);
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(0, 0, 5);
  capsuleStart.rotate(Eigen::AngleAxisd(
      3.141592653589793238462643383279502884 / 4, Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, -5);
  capsuleEnd.rotate(Eigen::AngleAxisd(
      3.141592653589793238462643383279502884 / 4, Eigen::Vector3d::UnitX()));

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastPlane(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(CapsuleCastPlane, RotatingEndpointArcHits)
{
  CapsuleShape capsule(0.2, 4.0);
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(0, 0, 1.5);
  capsuleStart.rotate(Eigen::AngleAxisd(
      -3.141592653589793238462643383279502884 / 3, Eigen::Vector3d::UnitY()));

  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 1.5);
  capsuleEnd.rotate(Eigen::AngleAxisd(
      3.141592653589793238462643383279502884 / 3, Eigen::Vector3d::UnitY()));

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastPlane(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-6);
}

//==============================================================================
// Capsule-cast Cylinder tests
//==============================================================================

TEST(CapsuleCastCylinder, Miss)
{
  CapsuleShape capsule(0.5, 2.0);
  CylinderShape target(1.0, 2.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(10, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(10, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastCylinder(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(CapsuleCastCylinder, DirectHit)
{
  CapsuleShape capsule(0.5, 2.0);
  CylinderShape target(1.0, 2.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(5, 0, 0);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastCylinder(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
}

//==============================================================================
// Capsule-cast Convex tests
//==============================================================================

TEST(CapsuleCastConvex, Miss)
{
  CapsuleShape capsule(0.5, 2.0);
  std::vector<Eigen::Vector3d> vertices
      = {{-1, -1, -1},
         {1, -1, -1},
         {1, 1, -1},
         {-1, 1, -1},
         {-1, -1, 1},
         {1, -1, 1},
         {1, 1, 1},
         {-1, 1, 1}};
  ConvexShape target(vertices);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(10, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(10, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastConvex(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(CapsuleCastConvex, DirectHit)
{
  CapsuleShape capsule(0.5, 2.0);
  std::vector<Eigen::Vector3d> vertices
      = {{-1, -1, -1},
         {1, -1, -1},
         {1, 1, -1},
         {-1, 1, -1},
         {-1, -1, 1},
         {1, -1, 1},
         {1, 1, 1},
         {-1, 1, 1}};
  ConvexShape target(vertices);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(5, 0, 0);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastConvex(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-6);
}

TEST(CapsuleCastConvex, StationaryOverlapUsesFallbackDirections)
{
  CapsuleShape capsule(0.25, 0.5);
  ConvexShape target(
      {{-1, -1, -1},
       {1, -1, -1},
       {1, 1, -1},
       {-1, 1, -1},
       {-1, -1, 1},
       {1, -1, 1},
       {1, 1, 1},
       {-1, 1, 1}});
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;
  ASSERT_TRUE(capsuleCastConvex(
      identity, identity, capsule, target, identity, option, result));
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
  EXPECT_TRUE(result.point.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
}

//==============================================================================
// Capsule-cast Mesh tests
//==============================================================================

TEST(CapsuleCastMesh, Miss)
{
  CapsuleShape capsule(0.5, 2.0);
  std::vector<Eigen::Vector3d> vertices
      = {{0, 0, 0}, {2, 0, 0}, {1, 2, 0}, {1, 1, 2}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  MeshShape target(vertices, triangles);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(10, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(10, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastMesh(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(CapsuleCastMesh, DirectHit)
{
  CapsuleShape capsule(0.25, 1.0);
  std::vector<Eigen::Vector3d> vertices
      = {{0, 0, 0}, {2, 0, 0}, {1, 2, 0}, {1, 1, 2}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  MeshShape target(vertices, triangles);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(0, 0, 5);

  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(1, 1, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(1, 1, 10);

  CcdOption option;
  CcdResult result;

  bool hit = capsuleCastMesh(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

//==============================================================================
// Conservative Advancement tests
//==============================================================================

std::vector<Eigen::Vector3d> makeCubeVertices(double halfExtent)
{
  double h = halfExtent;
  return {
      {-h, -h, -h},
      {h, -h, -h},
      {h, h, -h},
      {-h, h, -h},
      {-h, -h, h},
      {h, -h, h},
      {h, h, h},
      {-h, h, h}};
}

std::vector<Eigen::Vector3d> makeBoxVertices(const Eigen::Vector3d& half)
{
  return {
      {-half.x(), -half.y(), -half.z()},
      {half.x(), -half.y(), -half.z()},
      {half.x(), half.y(), -half.z()},
      {-half.x(), half.y(), -half.z()},
      {-half.x(), -half.y(), half.z()},
      {half.x(), -half.y(), half.z()},
      {half.x(), half.y(), half.z()},
      {-half.x(), half.y(), half.z()}};
}

TEST(ConservativeAdvancement, Miss)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d transformAStart = Eigen::Isometry3d::Identity();
  transformAStart.translation() = Eigen::Vector3d(10, 0, 0);
  Eigen::Isometry3d transformAEnd = Eigen::Isometry3d::Identity();
  transformAEnd.translation() = Eigen::Vector3d(10, 10, 0);

  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;

  bool hit = conservativeAdvancement(
      shapeA,
      transformAStart,
      transformAEnd,
      shapeB,
      transformB,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(ConservativeAdvancement, DirectHit)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d transformAStart = Eigen::Isometry3d::Identity();
  transformAStart.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d transformAEnd = Eigen::Isometry3d::Identity();
  transformAEnd.translation() = Eigen::Vector3d(5, 0, 0);

  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;

  bool hit = conservativeAdvancement(
      shapeA,
      transformAStart,
      transformAEnd,
      shapeB,
      transformB,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.3);
  EXPECT_LT(result.timeOfImpact, 0.6);
}

TEST(ConservativeAdvancement, StartingInContact)
{
  ConvexShape shapeA(makeCubeVertices(1.0));
  ConvexShape shapeB(makeCubeVertices(1.0));

  Eigen::Isometry3d transformAStart = Eigen::Isometry3d::Identity();
  transformAStart.translation() = Eigen::Vector3d(1.5, 0, 0);
  Eigen::Isometry3d transformAEnd = Eigen::Isometry3d::Identity();
  transformAEnd.translation() = Eigen::Vector3d(5, 0, 0);

  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;

  bool hit = conservativeAdvancement(
      shapeA,
      transformAStart,
      transformAEnd,
      shapeB,
      transformB,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 0.1);
}

TEST(ConservativeAdvancement, StationaryOverlapUsesFallbackDirections)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;
  const bool hit = conservativeAdvancement(
      shapeA, identity, identity, shapeB, identity, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
  EXPECT_TRUE(result.point.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
}

TEST(ConservativeAdvancement, RotatingShape)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d transformAStart = Eigen::Isometry3d::Identity();
  transformAStart.translation() = Eigen::Vector3d(3, 0, 0);

  Eigen::Isometry3d transformAEnd = Eigen::Isometry3d::Identity();
  transformAEnd.translation() = Eigen::Vector3d(0.5, 0, 0);
  transformAEnd.rotate(Eigen::AngleAxisd(
      3.141592653589793238462643383279502884 / 4, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;

  bool hit = conservativeAdvancement(
      shapeA,
      transformAStart,
      transformAEnd,
      shapeB,
      transformB,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.5);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

//==============================================================================
// Convex cast (both shapes moving)
//==============================================================================

TEST(ConvexCast, BothMovingTowardEachOther)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d aStart = Eigen::Isometry3d::Identity();
  aStart.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d aEnd = Eigen::Isometry3d::Identity();
  aEnd.translation() = Eigen::Vector3d(0, 0, 0);

  Eigen::Isometry3d bStart = Eigen::Isometry3d::Identity();
  bStart.translation() = Eigen::Vector3d(5, 0, 0);
  Eigen::Isometry3d bEnd = Eigen::Isometry3d::Identity();
  bEnd.translation() = Eigen::Vector3d(0, 0, 0);

  CcdOption option;
  CcdResult result;

  const bool hit
      = convexCast(shapeA, aStart, aEnd, shapeB, bStart, bEnd, option, result);

  EXPECT_TRUE(hit);
  // Centers close from 10 to surface contact at distance 1 -> t = 0.9.
  EXPECT_NEAR(result.timeOfImpact, 0.9, 5e-2);
}

TEST(ConvexCast, BothMovingSameDirectionMiss)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d aStart = Eigen::Isometry3d::Identity();
  aStart.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d aEnd = Eigen::Isometry3d::Identity();
  aEnd.translation() = Eigen::Vector3d(0, 0, 0);

  Eigen::Isometry3d bStart = Eigen::Isometry3d::Identity();
  bStart.translation() = Eigen::Vector3d(-3, 0, 0);
  Eigen::Isometry3d bEnd = Eigen::Isometry3d::Identity();
  bEnd.translation() = Eigen::Vector3d(2, 0, 0);

  CcdOption option;
  CcdResult result;

  const bool hit
      = convexCast(shapeA, aStart, aEnd, shapeB, bStart, bEnd, option, result);

  EXPECT_FALSE(hit);
}

TEST(ConvexCast, MatchesConservativeAdvancementWhenBStatic)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d aStart = Eigen::Isometry3d::Identity();
  aStart.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d aEnd = Eigen::Isometry3d::Identity();
  aEnd.translation() = Eigen::Vector3d(5, 0, 0);

  const Eigen::Isometry3d bStatic = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult castResult;
  CcdResult advResult;

  const bool castHit = convexCast(
      shapeA, aStart, aEnd, shapeB, bStatic, bStatic, option, castResult);
  const bool advHit = conservativeAdvancement(
      shapeA, aStart, aEnd, shapeB, bStatic, option, advResult);

  EXPECT_EQ(castHit, advHit);
  ASSERT_TRUE(castHit);
  EXPECT_NEAR(castResult.timeOfImpact, advResult.timeOfImpact, 1e-9);
}

// A zero (or negative) iteration budget must still detect shapes already
// overlapping at t = 0; the cast loops clamp to at least one iteration so the
// initial configuration is always tested.
TEST(ConvexCast, ZeroIterationBudgetDetectsInitialOverlap)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));
  const Eigen::Isometry3d atOrigin = Eigen::Isometry3d::Identity();

  CcdOption option;
  option.maxIterations = 0;
  CcdResult result;
  const bool hit = convexCast(
      shapeA, atOrigin, atOrigin, shapeB, atOrigin, atOrigin, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-9);
}

TEST(ConvexCast, CoincidentSupportPointsGiveFiniteNormal)
{
  // Degenerate convexes whose support points coincide (overlapping point-like
  // shapes) must not produce a NaN contact normal: the magnitude is checked
  // before normalizing, so the fallback normal is used instead.
  ConvexShape shapeA(std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()});
  ConvexShape shapeB(std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()});
  const Eigen::Isometry3d atOrigin = Eigen::Isometry3d::Identity();

  CcdOption option;
  CcdResult result;
  const bool hit = convexCast(
      shapeA, atOrigin, atOrigin, shapeB, atOrigin, atOrigin, option, result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.normal.allFinite());
  EXPECT_NEAR(result.normal.norm(), 1.0, 1e-9);
}

TEST(SplineCast, ZeroIterationBudgetDetectsInitialOverlap)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));
  const Eigen::Isometry3d atOrigin = Eigen::Isometry3d::Identity();
  const std::array<Eigen::Vector3d, 4> origin
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};

  CcdOption option;
  option.maxIterations = 0;
  CcdResult result;
  const bool hit
      = splineCast(shapeA, origin, origin, shapeB, atOrigin, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-9);
}

//==============================================================================
// Spline (cubic-Bezier) cast tests
//==============================================================================

namespace {

// First time on a fine grid at which shape A (following the spline) is within
// contact tolerance of static shape B -- a discrete reference that upper-bounds
// the true time of impact, so a conservative cast must not exceed it.
double splineSubstepHitTime(
    const ConvexShape& shapeA,
    const std::array<Eigen::Vector3d, 4>& translation,
    const std::array<Eigen::Vector3d, 4>& rotation,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& transformB,
    const CcdOption& option)
{
  constexpr int kSamples = 4000;
  const bool rotates = rotation[0].squaredNorm() + rotation[1].squaredNorm()
                           + rotation[2].squaredNorm()
                           + rotation[3].squaredNorm()
                       > 1e-12;
  for (int i = 0; i <= kSamples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(kSamples);
    const double u = 1.0 - t;
    const double b0 = u * u * u;
    const double b1 = 3.0 * t * u * u;
    const double b2 = 3.0 * t * t * u;
    const double b3 = t * t * t;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = b0 * translation[0] + b1 * translation[1]
                         + b2 * translation[2] + b3 * translation[3];
    if (rotates) {
      const Eigen::Vector3d w = b0 * rotation[0] + b1 * rotation[1]
                                + b2 * rotation[2] + b3 * rotation[3];
      const double angle = w.norm();
      if (angle > 1e-12) {
        pose.linear() = Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
      }
    }
    CcdResult probe;
    if (conservativeAdvancement(
            shapeA, pose, pose, shapeB, transformB, option, probe)) {
      return t;
    }
  }
  return -1.0;
}

} // namespace

// A spline can curve away from the chord between its endpoints, so it collides
// where a straight-line cast over the same endpoints misses entirely. This is
// the capability the linear and screw motion models cannot represent.
TEST(SplineCast, CurvedPathHitsWhereChordMisses)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();
  transformB.translation() = Eigen::Vector3d(0, -1, 0);

  // Downward-bulging cubic Bezier; the chord stays at y = 2 (far from B), but
  // the curve dips to the obstacle at its midpoint.
  const std::array<Eigen::Vector3d, 4> translation
      = {Eigen::Vector3d(-2, 2, 0),
         Eigen::Vector3d(-2, -2, 0),
         Eigen::Vector3d(2, -2, 0),
         Eigen::Vector3d(2, 2, 0)};
  const std::array<Eigen::Vector3d, 4> rotation
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};

  CcdOption option;
  CcdResult result;
  const bool hit = splineCast(
      shapeA, translation, rotation, shapeB, transformB, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 0.5);

  // A straight cast over the same endpoints (the chord) never approaches B.
  Eigen::Isometry3d chordStart = Eigen::Isometry3d::Identity();
  chordStart.translation() = translation[0];
  Eigen::Isometry3d chordEnd = Eigen::Isometry3d::Identity();
  chordEnd.translation() = translation[3];
  CcdResult chordResult;
  const bool chordHit = conservativeAdvancement(
      shapeA, chordStart, chordEnd, shapeB, transformB, option, chordResult);
  EXPECT_FALSE(chordHit);
}

// A conservative cast must never overshoot the true contact, so its time of
// impact stays at or below the first overlap a fine discrete sampling finds.
TEST(SplineCast, ConservativeVersusSubstep)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();
  transformB.translation() = Eigen::Vector3d(0, -1, 0);

  const std::array<Eigen::Vector3d, 4> translation
      = {Eigen::Vector3d(-2, 2, 0),
         Eigen::Vector3d(-2, -2, 0),
         Eigen::Vector3d(2, -2, 0),
         Eigen::Vector3d(2, 2, 0)};
  const std::array<Eigen::Vector3d, 4> rotation
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};

  CcdOption option;
  CcdResult result;
  ASSERT_TRUE(splineCast(
      shapeA, translation, rotation, shapeB, transformB, option, result));

  const double substepHit = splineSubstepHitTime(
      shapeA, translation, rotation, shapeB, transformB, option);
  ASSERT_GT(substepHit, 0.0);
  EXPECT_LE(result.timeOfImpact, substepHit + 1e-3);
}

// Evenly spaced collinear control points make the cubic Bezier a constant-speed
// straight line, so the spline cast must reproduce the linear impact time.
TEST(SplineCast, DegeneratesToLinearMotion)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  const Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d p0(-5, 0, 0);
  const Eigen::Vector3d p3(5, 0, 0);
  const Eigen::Vector3d d = p3 - p0;
  const std::array<Eigen::Vector3d, 4> translation
      = {p0, p0 + d / 3.0, p0 + 2.0 * d / 3.0, p3};
  const std::array<Eigen::Vector3d, 4> rotation
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};

  CcdOption option;
  CcdResult splineResult;
  ASSERT_TRUE(splineCast(
      shapeA, translation, rotation, shapeB, transformB, option, splineResult));

  Eigen::Isometry3d linearStart = Eigen::Isometry3d::Identity();
  linearStart.translation() = p0;
  Eigen::Isometry3d linearEnd = Eigen::Isometry3d::Identity();
  linearEnd.translation() = p3;
  CcdResult linearResult;
  ASSERT_TRUE(conservativeAdvancement(
      shapeA,
      linearStart,
      linearEnd,
      shapeB,
      transformB,
      option,
      linearResult));

  EXPECT_NEAR(splineResult.timeOfImpact, linearResult.timeOfImpact, 1e-3);
}

// A spline carrying rotation exercises the angular term of the motion bound: a
// rod that is clear of the obstacle while horizontal sweeps into it as it
// spins.
TEST(SplineCast, RotatingSplineHitsConservatively)
{
  ConvexShape shapeA(makeBoxVertices(Eigen::Vector3d(1.2, 0.1, 0.1)));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();

  // Stationary center just above B; rotation about Z sweeps the rod down into
  // B.
  const std::array<Eigen::Vector3d, 4> translation
      = {Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 1, 0)};
  const double pi = 3.141592653589793238462643383279502884;
  const std::array<Eigen::Vector3d, 4> rotation
      = {Eigen::Vector3d(0, 0, 0),
         Eigen::Vector3d(0, 0, pi / 3.0),
         Eigen::Vector3d(0, 0, 2.0 * pi / 3.0),
         Eigen::Vector3d(0, 0, pi)};

  CcdOption option;
  CcdResult result;
  const bool hit = splineCast(
      shapeA, translation, rotation, shapeB, transformB, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);

  const double substepHit = splineSubstepHitTime(
      shapeA, translation, rotation, shapeB, transformB, option);
  ASSERT_GT(substepHit, 0.0);
  EXPECT_LE(result.timeOfImpact, substepHit + 1e-3);
}

// The fast advancement mode takes larger displacement-based steps, trading the
// no-tunnelling guarantee for speed. On a sharply curved sweep it strides past
// the true first contact and reports a later one, whereas the conservative mode
// (default) reports the first contact. Both still detect the collision.
TEST(SplineCast, FastModeOvershootsConservativeFirstContact)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();
  transformB.translation() = Eigen::Vector3d(0, -1, 0);

  const std::array<Eigen::Vector3d, 4> translation
      = {Eigen::Vector3d(-2, 2, 0),
         Eigen::Vector3d(-2, -2, 0),
         Eigen::Vector3d(2, -2, 0),
         Eigen::Vector3d(2, 2, 0)};
  const std::array<Eigen::Vector3d, 4> rotation
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};

  CcdOption conservative;
  CcdResult conservativeResult;
  ASSERT_TRUE(splineCast(
      shapeA,
      translation,
      rotation,
      shapeB,
      transformB,
      conservative,
      conservativeResult));

  CcdOption fast;
  fast.advancement = CcdAdvancement::Fast;
  CcdResult fastResult;
  ASSERT_TRUE(splineCast(
      shapeA, translation, rotation, shapeB, transformB, fast, fastResult));

  // The conservative mode finds the genuine first contact; the fast mode
  // strides past it to a later overlap.
  EXPECT_LT(conservativeResult.timeOfImpact, 0.5);
  EXPECT_GT(fastResult.timeOfImpact, conservativeResult.timeOfImpact);
}

//==============================================================================
// Determinism tests
//==============================================================================

TEST(SphereCast, Determinism)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.23456, 2.34567, 5.0);

  Eigen::Vector3d start(0.1, 0.2, 0.0);
  Eigen::Vector3d end(0.1, 0.2, 10.0);
  double radius = 0.3;

  CcdOption option;

  std::vector<CcdResult> results;
  for (int i = 0; i < 100; ++i) {
    CcdResult result;
    [[maybe_unused]] bool hit = sphereCastSphere(
        start, end, radius, target, transform, option, result);
    results.push_back(result);
  }

  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_EQ(results[i].hit, results[0].hit);
    EXPECT_EQ(results[i].timeOfImpact, results[0].timeOfImpact);
    EXPECT_EQ(results[i].point.x(), results[0].point.x());
    EXPECT_EQ(results[i].point.y(), results[0].point.y());
    EXPECT_EQ(results[i].point.z(), results[0].point.z());
    EXPECT_EQ(results[i].normal.x(), results[0].normal.x());
    EXPECT_EQ(results[i].normal.y(), results[0].normal.y());
    EXPECT_EQ(results[i].normal.z(), results[0].normal.z());
  }
}

//==============================================================================
// NarrowPhase dispatcher tests
//==============================================================================

TEST(NarrowPhaseSphereCast, SphereTarget)
{
  SphereShape target(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  CcdOption option;
  CcdResult result;

  const bool hit = NarrowPhase::sphereCast(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 10),
      0.5,
      &target,
      transform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_NEAR(result.timeOfImpact, 0.35, 1e-6);
}

TEST(NarrowPhaseSphereCast, BoxTarget)
{
  BoxShape target(Eigen::Vector3d::Ones());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  CcdOption option;
  CcdResult result;

  const bool hit = NarrowPhase::sphereCast(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 10),
      0.5,
      &target,
      transform,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
}

TEST(NarrowPhaseSphereCast, CompoundUsesEarliestChildHit)
{
  CompoundShape target;

  Eigen::Isometry3d farther = Eigen::Isometry3d::Identity();
  farther.translation() = Eigen::Vector3d(0, 0, 8);
  target.addChild(std::make_unique<SphereShape>(1.0), farther);

  Eigen::Isometry3d nearer = Eigen::Isometry3d::Identity();
  nearer.translation() = Eigen::Vector3d(0, 0, 5);
  target.addChild(std::make_unique<SphereShape>(1.0), nearer);

  CcdOption option;
  CcdResult result;

  const bool hit = NarrowPhase::sphereCast(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 10),
      0.5,
      &target,
      Eigen::Isometry3d::Identity(),
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.35, 1e-6);
}

TEST(NarrowPhaseSphereCast, CompoundKeepsEndpointChildHit)
{
  CompoundShape target;

  Eigen::Isometry3d endpoint = Eigen::Isometry3d::Identity();
  endpoint.translation() = Eigen::Vector3d(0, 0, 11.5);
  target.addChild(std::make_unique<SphereShape>(1.0), endpoint);

  CcdOption option;
  CcdResult result;

  const bool hit = NarrowPhase::sphereCast(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 10),
      0.5,
      &target,
      Eigen::Isometry3d::Identity(),
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_NEAR(result.timeOfImpact, 1.0, 1e-12);
}

TEST(NarrowPhaseSphereCast, IsSphereCastSupported)
{
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Cylinder));
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Compound));
  EXPECT_FALSE(NarrowPhase::isSphereCastSupported(ShapeType::Sdf));
}

TEST(NarrowPhaseCapsuleCast, SphereTarget)
{
  SphereShape target(1.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(0, 0, 5);

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  const bool hit = NarrowPhase::capsuleCast(
      start, end, capsule, &target, targetTransform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
}

TEST(NarrowPhaseCapsuleCast, BoxTarget)
{
  BoxShape target(Eigen::Vector3d::Ones());
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(0, 0, 5);

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  const bool hit = NarrowPhase::capsuleCast(
      start, end, capsule, &target, targetTransform, option, result);

  EXPECT_TRUE(hit);
}

TEST(NarrowPhaseCapsuleCast, CylinderTarget)
{
  CylinderShape target(1.0, 2.0);
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(0, 0, 5);

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  const bool hit = NarrowPhase::capsuleCast(
      start, end, capsule, &target, targetTransform, option, result);

  EXPECT_TRUE(hit);
}

TEST(NarrowPhaseCapsuleCast, CompoundUsesEarliestChildHit)
{
  CompoundShape target;

  Eigen::Isometry3d farther = Eigen::Isometry3d::Identity();
  farther.translation() = Eigen::Vector3d(0, 0, 8);
  target.addChild(std::make_unique<SphereShape>(1.0), farther);

  Eigen::Isometry3d nearer = Eigen::Isometry3d::Identity();
  nearer.translation() = Eigen::Vector3d(0, 0, 5);
  target.addChild(std::make_unique<SphereShape>(1.0), nearer);

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
  end.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  const bool hit = NarrowPhase::capsuleCast(
      start,
      end,
      capsule,
      &target,
      Eigen::Isometry3d::Identity(),
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_LT(result.timeOfImpact, 0.5);
}

TEST(NarrowPhaseCapsuleCast, IsCapsuleCastSupported)
{
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Cylinder));
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Compound));
  EXPECT_FALSE(NarrowPhase::isCapsuleCastSupported(ShapeType::Sdf));
}
