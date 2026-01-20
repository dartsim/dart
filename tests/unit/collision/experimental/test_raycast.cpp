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

#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/narrow_phase/raycast.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(RaycastSphere, Miss)
{
  SphereShape sphere(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  Ray ray(Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastSphere(ray, sphere, transform, option, result);

  EXPECT_FALSE(hit);
  EXPECT_FALSE(result.isHit());
}

TEST(RaycastSphere, HitFromOutside)
{
  SphereShape sphere(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastSphere(ray, sphere, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 4.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-10);
}

TEST(RaycastSphere, GrazingHit)
{
  SphereShape sphere(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(-5, 1, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastSphere(ray, sphere, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_NEAR(result.distance, 5.0, 1e-10);
  EXPECT_NEAR(result.point.x(), 0.0, 1e-10);
  EXPECT_NEAR(result.point.y(), 1.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 0.0, 1e-10);
  EXPECT_NEAR(result.normal.x(), 0.0, 1e-10);
  EXPECT_NEAR(result.normal.y(), 1.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), 0.0, 1e-10);
}

TEST(RaycastSphere, HitFromInside)
{
  SphereShape sphere(2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastSphere(ray, sphere, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 2.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 2.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-10);
}

TEST(RaycastSphere, MaxDistanceRespected)
{
  SphereShape sphere(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 10);

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option = RaycastOption::withMaxDistance(5.0);
  RaycastResult result;

  bool hit = raycastSphere(ray, sphere, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastSphere, DiagonalRay)
{
  SphereShape sphere(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(5, 5, 5);

  Eigen::Vector3d dir = Eigen::Vector3d(1, 1, 1).normalized();
  Ray ray(Eigen::Vector3d(0, 0, 0), dir);
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastSphere(ray, sphere, transform, option, result);

  EXPECT_TRUE(hit);
  double expectedDist = std::sqrt(75.0) - 1.0;
  EXPECT_NEAR(result.distance, expectedDist, 1e-6);
}

TEST(RaycastBox, Miss)
{
  BoxShape box(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(5, 5, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastBox(ray, box, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastBox, HitFrontFace)
{
  BoxShape box(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, -5), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastBox(ray, box, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.z(), -1.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-10);
}

TEST(RaycastBox, ThinSlabHit)
{
  BoxShape box(Eigen::Vector3d(1, 1, 1e-3));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastBox(ray, box, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 0.999, 1e-10);
  EXPECT_NEAR(result.point.x(), 0.0, 1e-10);
  EXPECT_NEAR(result.point.y(), 0.0, 1e-10);
  EXPECT_NEAR(result.point.z(), -1e-3, 1e-10);
  EXPECT_NEAR(result.normal.x(), 0.0, 1e-10);
  EXPECT_NEAR(result.normal.y(), 0.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-10);
}

TEST(RaycastBox, HitSideFace)
{
  BoxShape box(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(-5, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastBox(ray, box, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.x(), -1.0, 1e-10);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-10);
}

TEST(RaycastBox, RayStartsInside)
{
  BoxShape box(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastBox(ray, box, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 1.0, 1e-10);
  EXPECT_NEAR(result.point.x(), 1.0, 1e-10);
  EXPECT_NEAR(result.point.y(), 0.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 0.0, 1e-10);
  EXPECT_NEAR(result.normal.x(), 1.0, 1e-10);
  EXPECT_NEAR(result.normal.y(), 0.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), 0.0, 1e-10);
}

TEST(RaycastBox, RotatedBox)
{
  BoxShape box(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()));

  Ray ray(Eigen::Vector3d(0, 0, -5), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastBox(ray, box, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_LT(result.distance, 5.0);
}

TEST(RaycastBox, MaxDistanceRespected)
{
  BoxShape box(Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 10);

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option = RaycastOption::withMaxDistance(5.0);
  RaycastResult result;

  bool hit = raycastBox(ray, box, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastCapsule, Miss)
{
  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(5, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCapsule(ray, capsule, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastCapsule, HitCylindricalPart)
{
  CapsuleShape capsule(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(-5, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCapsule(ray, capsule, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.x(), -1.0, 1e-10);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-10);
}

TEST(RaycastCapsule, HitSphericalCap)
{
  CapsuleShape capsule(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, -5), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCapsule(ray, capsule, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 3.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-10);
}

TEST(RaycastCapsule, RayStartsInside)
{
  CapsuleShape capsule(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCapsule(ray, capsule, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 2.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 2.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-10);
}

TEST(RaycastCylinder, Miss)
{
  CylinderShape cylinder(0.5, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(5, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCylinder(ray, cylinder, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastCylinder, HitCurvedSurface)
{
  CylinderShape cylinder(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(-5, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCylinder(ray, cylinder, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.x(), -1.0, 1e-10);
  EXPECT_NEAR(result.normal.x(), -1.0, 1e-10);
}

TEST(RaycastCylinder, HitTopCap)
{
  CylinderShape cylinder(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 5), Eigen::Vector3d(0, 0, -1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCylinder(ray, cylinder, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 1.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-10);
}

TEST(RaycastCylinder, HitBottomCap)
{
  CylinderShape cylinder(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, -5), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCylinder(ray, cylinder, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.z(), -1.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-10);
}

TEST(RaycastCylinder, RayStartsInside)
{
  CylinderShape cylinder(1.0, 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastCylinder(ray, cylinder, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 1.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 1.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-10);
}

TEST(RaycastPlane, Miss_ParallelRay)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 5), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastPlane(ray, plane, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastPlane, NearParallelRay)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d dir = Eigen::Vector3d(1, 0, -1e-6).normalized();
  Ray ray(Eigen::Vector3d(0, 0, 1), dir);
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastPlane(ray, plane, transform, option, result);

  EXPECT_TRUE(hit);
  double expected = 1.0 / std::abs(dir.z());
  EXPECT_NEAR(result.distance, expected, 1e-4);
  EXPECT_NEAR(result.point.z(), 0.0, 1e-8);
}

TEST(RaycastPlane, Miss_BackfaceCulling)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, -5), Eigen::Vector3d(0, 0, -1));
  RaycastOption option;
  option.backfaceCulling = true;
  RaycastResult result;

  bool hit = raycastPlane(ray, plane, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastPlane, HitFromAbove)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 5), Eigen::Vector3d(0, 0, -1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastPlane(ray, plane, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 5.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 0.0, 1e-10);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-10);
}

TEST(RaycastPlane, HitWithOffset)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 2.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 10), Eigen::Vector3d(0, 0, -1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastPlane(ray, plane, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 8.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 2.0, 1e-10);
}

TEST(RaycastPlane, HitWithTransform)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 3);

  Ray ray(Eigen::Vector3d(0, 0, 10), Eigen::Vector3d(0, 0, -1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastPlane(ray, plane, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 7.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 3.0, 1e-10);
}

TEST(RaycastPlane, NoBackfaceCulling)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, -5), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  option.backfaceCulling = false;
  RaycastResult result;

  bool hit = raycastPlane(ray, plane, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 5.0, 1e-10);
}

TEST(Raycast, Determinism)
{
  SphereShape sphere(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.23456, 2.34567, 3.45678);

  Ray ray(
      Eigen::Vector3d(0.1, 0.2, 0.3),
      Eigen::Vector3d(0.5, 0.6, 0.7).normalized());
  RaycastOption option;

  std::vector<RaycastResult> results;
  for (int i = 0; i < 100; ++i) {
    RaycastResult result;
    [[maybe_unused]] bool hit
        = raycastSphere(ray, sphere, transform, option, result);
    results.push_back(result);
  }

  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_EQ(results[i].hit, results[0].hit);
    EXPECT_EQ(results[i].distance, results[0].distance);
    EXPECT_EQ(results[i].point.x(), results[0].point.x());
    EXPECT_EQ(results[i].point.y(), results[0].point.y());
    EXPECT_EQ(results[i].point.z(), results[0].point.z());
    EXPECT_EQ(results[i].normal.x(), results[0].normal.x());
    EXPECT_EQ(results[i].normal.y(), results[0].normal.y());
    EXPECT_EQ(results[i].normal.z(), results[0].normal.z());
  }
}

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

std::vector<MeshShape::Triangle> makeCubeTriangles()
{
  return {
      {0, 2, 1},
      {0, 3, 2},
      {4, 5, 6},
      {4, 6, 7},
      {0, 1, 5},
      {0, 5, 4},
      {2, 3, 7},
      {2, 7, 6},
      {0, 4, 7},
      {0, 7, 3},
      {1, 2, 6},
      {1, 6, 5}};
}

TEST(RaycastMesh, Miss)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 5, 0), Eigen::Vector3d(0, 1, 0));
  RaycastOption option;
  option.backfaceCulling = false;
  RaycastResult result;

  bool hit = raycastMesh(ray, mesh, transform, option, result);

  EXPECT_FALSE(hit) << "distance=" << result.distance << " point=("
                    << result.point.x() << "," << result.point.y() << ","
                    << result.point.z() << ")";
}

TEST(RaycastMesh, HitFrontFace)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(-5, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastMesh(ray, mesh, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.x(), -1.0, 1e-10);
  EXPECT_NEAR(std::abs(result.normal.x()), 1.0, 1e-10);
}

TEST(RaycastMesh, HitTopFace)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 5), Eigen::Vector3d(0, 0, -1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastMesh(ray, mesh, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 1.0, 1e-10);
  EXPECT_NEAR(std::abs(result.normal.z()), 1.0, 1e-10);
}

TEST(RaycastMesh, TransformedMesh)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastMesh(ray, mesh, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.z(), 4.0, 1e-10);
}

TEST(RaycastMesh, MaxDistanceRespected)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(-5, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  option.maxDistance = 2.0;
  RaycastResult result;

  bool hit = raycastMesh(ray, mesh, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastMesh, BackfaceCullingHitsCorrectFace)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(-5, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  option.backfaceCulling = true;
  RaycastResult result;

  bool hit = raycastMesh(ray, mesh, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
  EXPECT_NEAR(result.point.x(), -1.0, 1e-10);
}

TEST(RaycastMesh, BackfaceCullingFromInside)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  option.backfaceCulling = true;
  RaycastResult result;

  bool hit = raycastMesh(ray, mesh, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastMesh, NarrowPhaseSupported)
{
  EXPECT_TRUE(NarrowPhase::isRaycastSupported(ShapeType::Mesh));
}

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale)
{
  return {
      {scale, 0, 0},
      {-scale, 0, 0},
      {0, scale, 0},
      {0, -scale, 0},
      {0, 0, scale},
      {0, 0, -scale}};
}

TEST(RaycastConvex, Miss)
{
  ConvexShape convex(makeOctahedronVertices(1.0));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(5, 5, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastConvex(ray, convex, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastConvex, HitFromOutside)
{
  ConvexShape convex(makeOctahedronVertices(1.0));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(-5, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastConvex(ray, convex, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.distance, 3.9);
  EXPECT_LT(result.distance, 4.1);
  EXPECT_NEAR(result.point.x(), -1.0, 0.1);
}

TEST(RaycastConvex, HitFromOrigin)
{
  ConvexShape convex(makeOctahedronVertices(1.0));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, 0, 5);

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastConvex(ray, convex, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.distance, 3.9);
  EXPECT_LT(result.distance, 4.1);
}

TEST(RaycastConvex, TransformedConvex)
{
  ConvexShape convex(makeCubeVertices(1.0));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(5, 0, 0);

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastConvex(ray, convex, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.distance, 3.9);
  EXPECT_LT(result.distance, 4.1);
}

TEST(RaycastConvex, MaxDistanceRespected)
{
  ConvexShape convex(makeOctahedronVertices(1.0));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(10, 0, 0);

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option = RaycastOption::withMaxDistance(5.0);
  RaycastResult result;

  bool hit = raycastConvex(ray, convex, transform, option, result);

  EXPECT_FALSE(hit);
}

TEST(RaycastConvex, RayStartsInside)
{
  ConvexShape convex(makeCubeVertices(2.0));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;
  RaycastResult result;

  bool hit = raycastConvex(ray, convex, transform, option, result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.distance, 0.0, 1e-6);
}

TEST(RaycastConvex, NarrowPhaseSupported)
{
  EXPECT_TRUE(NarrowPhase::isRaycastSupported(ShapeType::Convex));
}
