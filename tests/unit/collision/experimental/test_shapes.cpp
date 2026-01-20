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

#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(SphereShape, Construction)
{
  SphereShape sphere(2.5);

  EXPECT_EQ(sphere.getType(), ShapeType::Sphere);
  EXPECT_DOUBLE_EQ(sphere.getRadius(), 2.5);
}

TEST(SphereShape, ComputeLocalAabb)
{
  SphereShape sphere(3.0);

  auto aabb = sphere.computeLocalAabb();

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-3, -3, -3));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(3, 3, 3));
}

TEST(SphereShape, SmallRadius)
{
  SphereShape sphere(0.001);

  EXPECT_DOUBLE_EQ(sphere.getRadius(), 0.001);

  auto aabb = sphere.computeLocalAabb();
  EXPECT_NEAR(aabb.volume(), 8e-9, 1e-15);
}

TEST(BoxShape, Construction)
{
  BoxShape box(Eigen::Vector3d(1, 2, 3));

  EXPECT_EQ(box.getType(), ShapeType::Box);
  EXPECT_EQ(box.getHalfExtents(), Eigen::Vector3d(1, 2, 3));
}

TEST(BoxShape, ComputeLocalAabb)
{
  BoxShape box(Eigen::Vector3d(1, 2, 3));

  auto aabb = box.computeLocalAabb();

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-1, -2, -3));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(1, 2, 3));
}

TEST(BoxShape, CubeAabb)
{
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  auto aabb = box.computeLocalAabb();

  EXPECT_EQ(aabb.center(), Eigen::Vector3d::Zero());
  EXPECT_EQ(aabb.halfExtents(), Eigen::Vector3d(0.5, 0.5, 0.5));
  EXPECT_DOUBLE_EQ(aabb.volume(), 1.0);
}

TEST(Shape, Polymorphism)
{
  std::unique_ptr<Shape> sphere = std::make_unique<SphereShape>(1.0);
  std::unique_ptr<Shape> box
      = std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1));

  EXPECT_EQ(sphere->getType(), ShapeType::Sphere);
  EXPECT_EQ(box->getType(), ShapeType::Box);

  auto sphereAabb = sphere->computeLocalAabb();
  auto boxAabb = box->computeLocalAabb();

  EXPECT_DOUBLE_EQ(sphereAabb.volume(), 8.0);
  EXPECT_DOUBLE_EQ(boxAabb.volume(), 8.0);
}

TEST(ShapeType, EnumValues)
{
  EXPECT_NE(ShapeType::Sphere, ShapeType::Box);
  EXPECT_NE(ShapeType::Box, ShapeType::Capsule);
  EXPECT_NE(ShapeType::Capsule, ShapeType::Cylinder);
  EXPECT_NE(ShapeType::Cylinder, ShapeType::Mesh);
}

TEST(CapsuleShape, Construction)
{
  CapsuleShape capsule(0.5, 2.0);

  EXPECT_EQ(capsule.getType(), ShapeType::Capsule);
  EXPECT_DOUBLE_EQ(capsule.getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(capsule.getHeight(), 2.0);
}

TEST(CapsuleShape, ComputeLocalAabb)
{
  CapsuleShape capsule(0.5, 2.0);

  auto aabb = capsule.computeLocalAabb();

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-0.5, -0.5, -1.5));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(0.5, 0.5, 1.5));
}

TEST(CapsuleShape, ZeroHeight)
{
  CapsuleShape capsule(1.0, 0.0);

  auto aabb = capsule.computeLocalAabb();

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-1, -1, -1));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(1, 1, 1));
}

TEST(CapsuleShape, Polymorphism)
{
  std::unique_ptr<Shape> capsule = std::make_unique<CapsuleShape>(0.5, 2.0);

  EXPECT_EQ(capsule->getType(), ShapeType::Capsule);

  auto aabb = capsule->computeLocalAabb();
  EXPECT_DOUBLE_EQ(aabb.halfExtents().z(), 1.5);
}

TEST(CylinderShape, Construction)
{
  CylinderShape cylinder(0.5, 2.0);

  EXPECT_EQ(cylinder.getType(), ShapeType::Cylinder);
  EXPECT_DOUBLE_EQ(cylinder.getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(cylinder.getHeight(), 2.0);
}

TEST(CylinderShape, ComputeLocalAabb)
{
  CylinderShape cylinder(0.5, 2.0);

  auto aabb = cylinder.computeLocalAabb();

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-0.5, -0.5, -1.0));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(0.5, 0.5, 1.0));
}

TEST(CylinderShape, Polymorphism)
{
  std::unique_ptr<Shape> cylinder = std::make_unique<CylinderShape>(1.0, 4.0);

  EXPECT_EQ(cylinder->getType(), ShapeType::Cylinder);

  auto aabb = cylinder->computeLocalAabb();
  EXPECT_DOUBLE_EQ(aabb.halfExtents().z(), 2.0);
}

TEST(ConvexShape, Construction)
{
  std::vector<Eigen::Vector3d> vertices
      = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  ConvexShape convex(vertices);

  EXPECT_EQ(convex.getType(), ShapeType::Convex);
  EXPECT_EQ(convex.getVertices().size(), 6u);
}

TEST(ConvexShape, ComputeLocalAabb)
{
  std::vector<Eigen::Vector3d> vertices
      = {{2, 0, 0}, {-1, 0, 0}, {0, 3, 0}, {0, -1, 0}, {0, 0, 4}, {0, 0, -1}};

  ConvexShape convex(vertices);

  auto aabb = convex.computeLocalAabb();

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-1, -1, -1));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(2, 3, 4));
}

TEST(ConvexShape, SupportFunction)
{
  std::vector<Eigen::Vector3d> vertices
      = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  ConvexShape convex(vertices);

  EXPECT_EQ(convex.support(Eigen::Vector3d(1, 0, 0)), Eigen::Vector3d(1, 0, 0));
  EXPECT_EQ(
      convex.support(Eigen::Vector3d(-1, 0, 0)), Eigen::Vector3d(-1, 0, 0));
  EXPECT_EQ(convex.support(Eigen::Vector3d(0, 1, 0)), Eigen::Vector3d(0, 1, 0));
  EXPECT_EQ(convex.support(Eigen::Vector3d(0, 0, 1)), Eigen::Vector3d(0, 0, 1));
}

TEST(ConvexShape, SupportDiagonalDirection)
{
  std::vector<Eigen::Vector3d> vertices = {{1, 1, 1}, {-1, -1, -1}};

  ConvexShape convex(vertices);

  EXPECT_EQ(
      convex.support(Eigen::Vector3d(1, 1, 1).normalized()),
      Eigen::Vector3d(1, 1, 1));
  EXPECT_EQ(
      convex.support(Eigen::Vector3d(-1, -1, -1).normalized()),
      Eigen::Vector3d(-1, -1, -1));
}

TEST(ConvexShape, EmptyVertices)
{
  ConvexShape convex({});

  auto aabb = convex.computeLocalAabb();
  EXPECT_EQ(aabb.min, Eigen::Vector3d::Zero());
  EXPECT_EQ(aabb.max, Eigen::Vector3d::Zero());

  EXPECT_EQ(convex.support(Eigen::Vector3d(1, 0, 0)), Eigen::Vector3d::Zero());
}

TEST(MeshShape, Construction)
{
  std::vector<Eigen::Vector3d> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  std::vector<MeshShape::Triangle> triangles = {{0, 1, 2}};

  MeshShape mesh(vertices, triangles);

  EXPECT_EQ(mesh.getType(), ShapeType::Mesh);
  EXPECT_EQ(mesh.getVertices().size(), 3u);
  EXPECT_EQ(mesh.getTriangles().size(), 1u);
}

TEST(MeshShape, ComputeLocalAabb)
{
  std::vector<Eigen::Vector3d> vertices
      = {{0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {0, 0, 4}};
  std::vector<MeshShape::Triangle> triangles = {{0, 1, 2}, {0, 1, 3}};

  MeshShape mesh(vertices, triangles);

  auto aabb = mesh.computeLocalAabb();

  EXPECT_EQ(aabb.min, Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(2, 3, 4));
}

TEST(MeshShape, SupportFunction)
{
  std::vector<Eigen::Vector3d> vertices
      = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};

  MeshShape mesh(vertices, triangles);

  EXPECT_EQ(mesh.support(Eigen::Vector3d(1, 0, 0)), Eigen::Vector3d(1, 0, 0));
  EXPECT_EQ(mesh.support(Eigen::Vector3d(-1, 0, 0)), Eigen::Vector3d(-1, 0, 0));
  EXPECT_EQ(mesh.support(Eigen::Vector3d(0, 1, 0)), Eigen::Vector3d(0, 1, 0));
  EXPECT_EQ(mesh.support(Eigen::Vector3d(0, 0, 1)), Eigen::Vector3d(0, 0, 1));
}

TEST(MeshShape, UnitCube)
{
  std::vector<Eigen::Vector3d> vertices
      = {{0, 0, 0},
         {1, 0, 0},
         {1, 1, 0},
         {0, 1, 0},
         {0, 0, 1},
         {1, 0, 1},
         {1, 1, 1},
         {0, 1, 1}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 2},
         {0, 2, 3},
         {4, 6, 5},
         {4, 7, 6},
         {0, 5, 1},
         {0, 4, 5},
         {2, 6, 7},
         {2, 7, 3},
         {0, 7, 4},
         {0, 3, 7},
         {1, 5, 6},
         {1, 6, 2}};

  MeshShape mesh(vertices, triangles);

  auto aabb = mesh.computeLocalAabb();
  EXPECT_EQ(aabb.min, Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(1, 1, 1));
  EXPECT_DOUBLE_EQ(aabb.volume(), 1.0);
}
