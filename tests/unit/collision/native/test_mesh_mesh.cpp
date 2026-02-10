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

#include <dart/collision/native/collision_object.hpp>
#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/narrow_phase/raycast.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

std::vector<Eigen::Vector3d> makeCubeVertices(double halfExtent)
{
  const double h = halfExtent;
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

MeshShape makeGridMesh(int resolution, double scale)
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<MeshShape::Triangle> triangles;

  vertices.reserve(
      static_cast<std::size_t>((resolution + 1) * (resolution + 1)));
  triangles.reserve(static_cast<std::size_t>(resolution * resolution * 2));

  for (int y = 0; y <= resolution; ++y) {
    for (int x = 0; x <= resolution; ++x) {
      const double px = (static_cast<double>(x) / resolution - 0.5) * scale;
      const double py = (static_cast<double>(y) / resolution - 0.5) * scale;
      vertices.emplace_back(px, py, 0.0);
    }
  }

  const auto idx = [resolution](int x, int y) {
    return y * (resolution + 1) + x;
  };

  for (int y = 0; y < resolution; ++y) {
    for (int x = 0; x < resolution; ++x) {
      triangles.emplace_back(idx(x, y), idx(x + 1, y), idx(x + 1, y + 1));
      triangles.emplace_back(idx(x, y), idx(x + 1, y + 1), idx(x, y + 1));
    }
  }

  return MeshShape(std::move(vertices), std::move(triangles));
}

} // namespace

TEST(MeshMesh, BoxMeshesColliding)
{
  CollisionWorld world;
  auto obj1 = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0.0, 0.0);
  auto obj2 = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()),
      tf2);

  CollisionOption option;
  option.maxNumContacts = 8;
  CollisionResult result;

  const bool hit = NarrowPhase::collide(obj1, obj2, option, result);
  ASSERT_TRUE(hit);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_GT(result.getContact(0).normal.norm(), 0.9);
  EXPECT_GE(result.getContact(0).depth, 0.0);
}

TEST(MeshMesh, BoxMeshesSeparated)
{
  CollisionWorld world;
  auto obj1 = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
  auto obj2 = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()),
      tf2);

  CollisionResult result;
  EXPECT_FALSE(NarrowPhase::collide(obj1, obj2, CollisionOption(), result));
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(PrimitiveMesh, SphereVsMesh)
{
  CollisionWorld world;
  auto meshObj = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()));

  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.0, 0.0, 1.3);
  auto sphereObj
      = world.createObject(std::make_unique<SphereShape>(0.5), sphereTf);

  CollisionResult result;
  EXPECT_TRUE(
      NarrowPhase::collide(meshObj, sphereObj, CollisionOption(), result));
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(RaycastMesh, BvhTraversalHit)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Ray ray(Eigen::Vector3d(-5.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0));
  RaycastResult result;
  EXPECT_TRUE(raycastMesh(
      ray,
      mesh,
      Eigen::Isometry3d::Identity(),
      RaycastOption::unlimited(),
      result));
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
}

TEST(MeshMesh, SingleTriangleAndLargeMesh)
{
  std::vector<Eigen::Vector3d> triVertices
      = {{-0.5, -0.5, 0.0}, {0.5, -0.5, 0.0}, {0.0, 0.5, 0.0}};
  std::vector<MeshShape::Triangle> triFaces = {{0, 1, 2}};

  CollisionWorld world;
  auto singleTriangle
      = world.createObject(std::make_unique<MeshShape>(triVertices, triFaces));

  Eigen::Isometry3d gridTf = Eigen::Isometry3d::Identity();
  gridTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto largeGrid = world.createObject(
      std::make_unique<MeshShape>(makeGridMesh(40, 4.0)), gridTf);

  CollisionResult result;
  EXPECT_TRUE(
      NarrowPhase::collide(
          singleTriangle, largeGrid, CollisionOption(), result));
  EXPECT_GE(result.numContacts(), 1u);

  DistanceResult distance;
  const double d = NarrowPhase::distance(
      singleTriangle, largeGrid, DistanceOption(), distance);
  EXPECT_LE(d, 0.0);
}
