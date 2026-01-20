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

#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/narrow_phase/ccd.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

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
  transform.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()));

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
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

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
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

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
  capsuleEnd.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));

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
  capsuleStart.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, -5);
  capsuleEnd.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()));

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

TEST(ConservativeAdvancement, RotatingShape)
{
  ConvexShape shapeA(makeCubeVertices(0.5));
  ConvexShape shapeB(makeCubeVertices(0.5));

  Eigen::Isometry3d transformAStart = Eigen::Isometry3d::Identity();
  transformAStart.translation() = Eigen::Vector3d(3, 0, 0);

  Eigen::Isometry3d transformAEnd = Eigen::Isometry3d::Identity();
  transformAEnd.translation() = Eigen::Vector3d(0.5, 0, 0);
  transformAEnd.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

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
  CollisionWorld world;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);
  auto target
      = world.createObject(std::make_unique<SphereShape>(1.0), transform);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = NarrowPhase::sphereCast(start, end, radius, target, option, result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(result.object, &target);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(NarrowPhaseSphereCast, BoxTarget)
{
  CollisionWorld world;
  auto target = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)));

  Eigen::Vector3d start(0, 0, -5);
  Eigen::Vector3d end(0, 0, 5);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  bool hit
      = NarrowPhase::sphereCast(start, end, radius, target, option, result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(result.object, &target);
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
}

//==============================================================================
// NarrowPhase capsule-cast dispatcher tests
//==============================================================================

TEST(NarrowPhaseCapsuleCast, SphereTarget)
{
  CollisionWorld world;
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 5);
  auto target = world.createObject(std::make_unique<SphereShape>(1.0), tf);

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  bool hit = NarrowPhase::capsuleCast(
      capsuleStart, capsuleEnd, capsule, target, option, result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(*result.object, target);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(NarrowPhaseCapsuleCast, BoxTarget)
{
  CollisionWorld world;
  auto target = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(2, 2, 2)));

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(0, 0, -5);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 5);

  CcdOption option;
  CcdResult result;

  bool hit = NarrowPhase::capsuleCast(
      capsuleStart, capsuleEnd, capsule, target, option, result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(*result.object, target);
}

TEST(NarrowPhaseCapsuleCast, CylinderTarget)
{
  CollisionWorld world;
  auto target = world.createObject(std::make_unique<CylinderShape>(1.0, 2.0));

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(-5, 0, 0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(5, 0, 0);

  CcdOption option;
  CcdResult result;

  bool hit = NarrowPhase::capsuleCast(
      capsuleStart, capsuleEnd, capsule, target, option, result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(*result.object, target);
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
}
