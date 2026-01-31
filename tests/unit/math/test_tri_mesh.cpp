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

#include "../../helpers/gtest_utils.hpp"
#include "dart/math/tri_mesh.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace math;

//==============================================================================
TEST(TriMeshTests, DefaultConstructor)
{
  auto mesh = TriMeshd();
  EXPECT_FALSE(mesh.hasTriangles());
  EXPECT_FALSE(mesh.hasTriangleNormals());
  EXPECT_TRUE(mesh.isEmpty());
}

//==============================================================================
TEST(TriMeshTests, SetTriangles)
{
  auto mesh = TriMeshd();
  EXPECT_TRUE(mesh.isEmpty());

  auto vertices = TriMeshd::Vertices();
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  auto triangles = TriMeshd::Triangles();
  triangles.emplace_back(0, 1, 2);

  mesh.setTriangles(vertices, triangles);
  EXPECT_TRUE(mesh.hasTriangles());
  EXPECT_FALSE(mesh.hasTriangleNormals());
  EXPECT_FALSE(mesh.isEmpty());
  EXPECT_EQ(mesh.getVertices(), vertices);
  EXPECT_EQ(mesh.getTriangles(), triangles);

  mesh.computeVertexNormals();
  EXPECT_TRUE(mesh.hasTriangles());
  EXPECT_TRUE(mesh.hasTriangleNormals());
  EXPECT_FALSE(mesh.isEmpty());

  mesh.clear();
  EXPECT_FALSE(mesh.hasTriangles());
  EXPECT_FALSE(mesh.hasTriangleNormals());
  EXPECT_TRUE(mesh.isEmpty());
}

//==============================================================================
TEST(TriMeshTests, Operators)
{
  auto mesh1 = TriMeshd();
  auto mesh2 = TriMeshd();

  auto vertices = TriMeshd::Vertices();
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  auto triangles = TriMeshd::Triangles();
  triangles.emplace_back(0, 1, 2);

  mesh1.setTriangles(vertices, triangles);
  EXPECT_EQ(mesh1.getVertices().size(), 3);
  EXPECT_EQ(mesh1.getTriangles().size(), 1);
  mesh2.setTriangles(vertices, triangles);
  EXPECT_EQ(mesh2.getVertices().size(), 3);
  EXPECT_EQ(mesh2.getTriangles().size(), 1);

  auto mesh3 = mesh1 + mesh2;
  EXPECT_EQ(mesh3.getVertices().size(), 6);
  EXPECT_EQ(mesh3.getTriangles().size(), 2);
  EXPECT_FALSE(mesh3.hasTriangleNormals());
  EXPECT_FALSE(mesh3.hasVertexNormals());

  mesh1.computeVertexNormals();
  EXPECT_TRUE(mesh1.hasTriangleNormals());
  EXPECT_TRUE(mesh1.hasVertexNormals());
  EXPECT_FALSE(mesh2.hasTriangleNormals());
  EXPECT_FALSE(mesh2.hasVertexNormals());
  EXPECT_FALSE((mesh1 + mesh2).hasTriangleNormals());
  EXPECT_FALSE((mesh1 + mesh2).hasVertexNormals());

  mesh2.computeVertexNormals();
  EXPECT_TRUE(mesh1.hasTriangleNormals());
  EXPECT_TRUE(mesh1.hasVertexNormals());
  EXPECT_TRUE(mesh2.hasTriangleNormals());
  EXPECT_TRUE(mesh2.hasVertexNormals());
  EXPECT_TRUE((mesh1 + mesh2).hasTriangleNormals());
  EXPECT_TRUE((mesh1 + mesh2).hasVertexNormals());

  mesh1 += mesh2;
  EXPECT_EQ(mesh1.getVertices().size(), 6);
  EXPECT_EQ(mesh1.getTriangles().size(), 2);
}

//==============================================================================
TEST(TriMeshTests, GenerateConvexHull)
{
  auto mesh = TriMeshd();
  EXPECT_TRUE(mesh.isEmpty());

  auto emptyConvexHull = mesh.generateConvexHull();
  ASSERT_NE(emptyConvexHull, nullptr);
  EXPECT_TRUE(emptyConvexHull->isEmpty());

  auto vertices = TriMeshd::Vertices();
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  vertices.emplace_back(0, 0, 1);
  mesh.setTriangles(vertices, {});

  auto convexHull = mesh.generateConvexHull();
  ASSERT_NE(convexHull, nullptr);
  EXPECT_EQ(convexHull->getVertices().size(), vertices.size());
  EXPECT_EQ(convexHull->getTriangles().size(), 4);
}

//==============================================================================
TEST(TriMeshTests, ReserveAndAdd)
{
  auto mesh = TriMeshd();

  mesh.reserveVertices(3);
  mesh.reserveTriangles(1);
  mesh.reserveVertexNormals(3);

  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addTriangle(0, 1, 2);

  EXPECT_EQ(mesh.getVertices().size(), 3u);
  EXPECT_EQ(mesh.getTriangles().size(), 1u);
  EXPECT_FALSE(mesh.hasVertexNormals());

  mesh.addVertexNormal(0.0, 0.0, 1.0);
  mesh.addVertexNormal(0.0, 0.0, 1.0);
  mesh.addVertexNormal(0.0, 0.0, 1.0);
  EXPECT_TRUE(mesh.hasVertexNormals());
  EXPECT_EQ(mesh.getVertexNormals().size(), 3u);

  Eigen::Vector3d vertex(0.5, 0.5, 0.0);
  mesh.addVertex(vertex);
  EXPECT_FALSE(mesh.hasVertexNormals());
  EXPECT_EQ(mesh.getVertices().size(), 4u);

  Eigen::Matrix<std::size_t, 3, 1> triangle;
  triangle << 0, 1, 3;
  mesh.addTriangle(triangle);
  EXPECT_EQ(mesh.getTriangles().size(), 2u);

  Eigen::Vector3d normal(0.0, 1.0, 0.0);
  mesh.addVertexNormal(normal);
  EXPECT_TRUE(mesh.hasVertexNormals());
  EXPECT_EQ(mesh.getVertexNormals().size(), 4u);
}

//==============================================================================
TEST(TriMeshTests, ComputeVertexNormals)
{
  auto mesh = TriMeshd();

  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addTriangle(0, 1, 2);

  mesh.computeVertexNormals();

  EXPECT_TRUE(mesh.hasVertexNormals());
  EXPECT_EQ(mesh.getVertexNormals().size(), mesh.getVertices().size());

  for (const auto& normal : mesh.getVertexNormals()) {
    EXPECT_NEAR(normal.z(), 1.0, 1e-10);
    EXPECT_NEAR(normal.x(), 0.0, 1e-10);
    EXPECT_NEAR(normal.y(), 0.0, 1e-10);
  }
}

//==============================================================================
TEST(TriMeshTests, BasicsAndNormals)
{
  TriMeshd mesh;
  EXPECT_TRUE(mesh.isEmpty());

  mesh.reserveVertices(4);
  mesh.reserveTriangles(2);
  mesh.reserveVertexNormals(4);
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addVertex(0.0, 0.0, 1.0);
  mesh.addTriangle(0, 1, 2);
  mesh.addTriangle(0, 1, 3);

  EXPECT_EQ(mesh.getVertices().size(), 4u);
  EXPECT_EQ(mesh.getTriangles().size(), 2u);
  EXPECT_FALSE(mesh.hasTriangleNormals());
  EXPECT_FALSE(mesh.hasVertexNormals());

  mesh.computeVertexNormals();
  EXPECT_TRUE(mesh.hasTriangleNormals());
  EXPECT_TRUE(mesh.hasVertexNormals());
  EXPECT_EQ(mesh.getTriangleNormals().size(), 2u);
  EXPECT_EQ(mesh.getVertexNormals().size(), 4u);

  mesh.clear();
  EXPECT_TRUE(mesh.isEmpty());
  EXPECT_FALSE(mesh.hasTriangles());
}

//==============================================================================
TEST(TriMeshTests, PlusEqualsEmptyKeepsNormals)
{
  TriMeshd mesh;
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addTriangle(0, 1, 2);
  mesh.computeVertexNormals();

  TriMeshd empty;
  mesh += empty;

  EXPECT_TRUE(mesh.hasTriangles());
  EXPECT_TRUE(mesh.hasVertexNormals());
  EXPECT_TRUE(mesh.hasTriangleNormals());
}

//==============================================================================
TEST(TriMeshTests, PlusEqualsClearsNormalsOnMismatch)
{
  TriMeshd meshWithoutNormals;
  meshWithoutNormals.addVertex(0.0, 0.0, 0.0);
  meshWithoutNormals.addVertex(1.0, 0.0, 0.0);
  meshWithoutNormals.addVertex(0.0, 1.0, 0.0);
  meshWithoutNormals.addTriangle(0, 1, 2);

  TriMeshd meshWithNormals;
  meshWithNormals.addVertex(0.0, 0.0, 0.0);
  meshWithNormals.addVertex(0.0, 1.0, 0.0);
  meshWithNormals.addVertex(0.0, 0.0, 1.0);
  meshWithNormals.addTriangle(0, 1, 2);
  meshWithNormals.computeVertexNormals();

  meshWithoutNormals += meshWithNormals;

  EXPECT_EQ(meshWithoutNormals.getTriangles().size(), 2u);
  EXPECT_FALSE(meshWithoutNormals.hasTriangleNormals());
}

//==============================================================================
TEST(TriMeshTests, GenerateConvexHullWithoutOptimization)
{
  TriMeshd mesh;
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addVertex(0.0, 0.0, 1.0);
  mesh.addTriangle(0, 1, 2);
  mesh.addTriangle(0, 1, 3);
  mesh.addTriangle(0, 2, 3);
  mesh.addTriangle(1, 2, 3);

  auto convexHull = mesh.generateConvexHull(false);
  ASSERT_NE(convexHull, nullptr);
  EXPECT_GE(convexHull->getVertices().size(), 4u);
  EXPECT_GT(convexHull->getTriangles().size(), 0u);
}

//==============================================================================
TEST(TriMeshTests, VertexNormalsNormalized)
{
  TriMeshd mesh;
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addVertex(1.0, 1.0, 0.0);
  mesh.addTriangle(0, 1, 2);
  mesh.addTriangle(1, 3, 2);

  mesh.computeVertexNormals();

  for (const auto& normal : mesh.getVertexNormals()) {
    EXPECT_NEAR(normal.norm(), 1.0, 1e-12);
  }
}

//==============================================================================
TEST(TriMeshTests, SetTrianglesClearsPreviousNormals)
{
  TriMeshd mesh;
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addTriangle(0, 1, 2);
  mesh.computeVertexNormals();

  TriMeshd::Vertices vertices;
  vertices.emplace_back(0.0, 0.0, 0.0);
  vertices.emplace_back(1.0, 0.0, 0.0);
  vertices.emplace_back(0.0, 1.0, 0.0);
  vertices.emplace_back(0.0, 0.0, 1.0);
  TriMeshd::Triangles triangles;
  triangles.emplace_back(0, 1, 3);
  triangles.emplace_back(0, 2, 3);

  mesh.setTriangles(vertices, triangles);
  EXPECT_FALSE(mesh.hasVertexNormals());
  EXPECT_FALSE(mesh.hasTriangleNormals());
  EXPECT_EQ(mesh.getTriangles().size(), 2u);
}
