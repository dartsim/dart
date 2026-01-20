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

#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/narrow_phase/convex_convex.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale = 1.0)
{
  return {
      {scale, 0, 0},
      {-scale, 0, 0},
      {0, scale, 0},
      {0, -scale, 0},
      {0, 0, scale},
      {0, 0, -scale}};
}

std::vector<Eigen::Vector3d> makeCubeVertices(double halfExtent = 1.0)
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

TEST(ConvexCollision, ConvexConvexIntersecting)
{
  auto octahedron1 = std::make_unique<ConvexShape>(makeOctahedronVertices());
  auto octahedron2 = std::make_unique<ConvexShape>(makeOctahedronVertices());

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides = collideConvexConvex(
      *octahedron1, tf1, *octahedron2, tf2, result, option);

  EXPECT_TRUE(collides);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(ConvexCollision, ConvexConvexSeparated)
{
  auto octahedron1 = std::make_unique<ConvexShape>(makeOctahedronVertices());
  auto octahedron2 = std::make_unique<ConvexShape>(makeOctahedronVertices());

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  CollisionOption option;

  bool collides = collideConvexConvex(
      *octahedron1, tf1, *octahedron2, tf2, result, option);

  EXPECT_FALSE(collides);
}

TEST(ConvexCollision, ConvexSphereIntersecting)
{
  auto octahedron = std::make_unique<ConvexShape>(makeOctahedronVertices());
  auto sphere = std::make_unique<SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides
      = collideConvexConvex(*octahedron, tf1, *sphere, tf2, result, option);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, ConvexBoxIntersecting)
{
  auto octahedron = std::make_unique<ConvexShape>(makeOctahedronVertices());
  auto box = std::make_unique<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.0, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides
      = collideConvexConvex(*octahedron, tf1, *box, tf2, result, option);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, MeshMeshIntersecting)
{
  std::vector<Eigen::Vector3d> vertices1 = makeCubeVertices();
  std::vector<MeshShape::Triangle> triangles1
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

  auto mesh1 = std::make_unique<MeshShape>(vertices1, triangles1);
  auto mesh2 = std::make_unique<MeshShape>(vertices1, triangles1);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides = collideConvexConvex(*mesh1, tf1, *mesh2, tf2, result, option);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, MeshMeshSeparated)
{
  std::vector<Eigen::Vector3d> vertices1 = makeCubeVertices();
  std::vector<MeshShape::Triangle> triangles1
      = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}};

  auto mesh1 = std::make_unique<MeshShape>(vertices1, triangles1);
  auto mesh2 = std::make_unique<MeshShape>(vertices1, triangles1);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  CollisionResult result;
  CollisionOption option;

  bool collides = collideConvexConvex(*mesh1, tf1, *mesh2, tf2, result, option);

  EXPECT_FALSE(collides);
}

TEST(ConvexCollision, NarrowPhaseSupportsConvex)
{
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Convex));
}

TEST(ConvexCollision, CollisionWorldWithConvex)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices()));
  auto obj2 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices()));

  obj1.setTransform(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  obj2.setTransform(tf2);

  CollisionOption option;
  option.maxNumContacts = 10;
  CollisionResult result;

  bool collides = world.collide(option, result);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, RotatedConvex)
{
  auto cube1 = std::make_unique<ConvexShape>(makeCubeVertices(0.5));
  auto cube2 = std::make_unique<ConvexShape>(makeCubeVertices(0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.linear() = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())
                     .toRotationMatrix();
  tf2.translation() = Eigen::Vector3d(1.0, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides = collideConvexConvex(*cube1, tf1, *cube2, tf2, result, option);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, RotatedConvexNoCollision)
{
  auto cube1 = std::make_unique<ConvexShape>(makeCubeVertices(0.5));
  auto cube2 = std::make_unique<ConvexShape>(makeCubeVertices(0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);

  CollisionResult result;
  CollisionOption option;

  bool collides = collideConvexConvex(*cube1, tf1, *cube2, tf2, result, option);

  EXPECT_FALSE(collides);
}
