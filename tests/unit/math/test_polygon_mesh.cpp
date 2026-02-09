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

#include "dart/math/polygon_mesh.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace dart;
using namespace math;

//==============================================================================
TEST(PolygonMeshTests, TriangulateQuad)
{
  PolygonMeshd mesh;
  mesh.reserveVertices(4);
  mesh.reserveFaces(1);

  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(1.0, 1.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addFace({0, 1, 2, 3});

  const auto triMesh = mesh.triangulate();
  EXPECT_EQ(triMesh.getVertices().size(), 4u);
  EXPECT_EQ(triMesh.getTriangles().size(), 2u);

  const auto& vertices = triMesh.getVertices();
  const auto& face = mesh.getFaces()[0];
  double polygonArea = 0.0;
  for (std::size_t i = 0; i < face.size(); ++i) {
    const auto& p = vertices[face[i]];
    const auto& q = vertices[face[(i + 1) % face.size()]];
    polygonArea += p.x() * q.y() - q.x() * p.y();
  }
  polygonArea = 0.5 * std::abs(polygonArea);

  double triangleArea = 0.0;
  for (const auto& triangle : triMesh.getTriangles()) {
    const auto& a = vertices[triangle[0]];
    const auto& b = vertices[triangle[1]];
    const auto& c = vertices[triangle[2]];
    const auto cross = (b - a).cross(c - a);
    triangleArea += 0.5 * std::abs(cross.z());
  }

  EXPECT_NEAR(triangleArea, polygonArea, 1e-12);
}

//==============================================================================
TEST(PolygonMeshTests, TriangulateConcave)
{
  PolygonMeshd mesh;
  mesh.reserveVertices(5);
  mesh.reserveFaces(1);

  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(2.0, 0.0, 0.0);
  mesh.addVertex(2.0, 2.0, 0.0);
  mesh.addVertex(1.0, 1.0, 0.0);
  mesh.addVertex(0.0, 2.0, 0.0);
  mesh.addFace({0, 1, 2, 3, 4});

  const auto triMesh = mesh.triangulate();
  EXPECT_EQ(triMesh.getTriangles().size(), 3u);

  const auto& vertices = triMesh.getVertices();
  const auto& face = mesh.getFaces()[0];
  double polygonArea = 0.0;
  for (std::size_t i = 0; i < face.size(); ++i) {
    const auto& p = vertices[face[i]];
    const auto& q = vertices[face[(i + 1) % face.size()]];
    polygonArea += p.x() * q.y() - q.x() * p.y();
  }
  polygonArea = 0.5 * std::abs(polygonArea);

  double triangleArea = 0.0;
  for (const auto& triangle : triMesh.getTriangles()) {
    const auto& a = vertices[triangle[0]];
    const auto& b = vertices[triangle[1]];
    const auto& c = vertices[triangle[2]];
    const auto cross = (b - a).cross(c - a);
    triangleArea += 0.5 * std::abs(cross.z());
  }

  EXPECT_NEAR(triangleArea, polygonArea, 1e-12);
}

//==============================================================================
TEST(PolygonMeshTests, TriangulateComputesNormals)
{
  PolygonMeshd mesh;
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);
  mesh.addFace({0, 1, 2});

  const auto triMesh = mesh.triangulate();
  EXPECT_TRUE(triMesh.hasVertexNormals());
  EXPECT_EQ(triMesh.getVertexNormals().size(), triMesh.getVertices().size());
}

//==============================================================================
TEST(PolygonMeshTests, TriangulateDegenerateFaceFallsBack)
{
  PolygonMeshd mesh;
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(2.0, 0.0, 0.0);
  mesh.addVertex(3.0, 0.0, 0.0);
  mesh.addFace({0, 1, 2, 3});

  const auto triMesh = mesh.triangulate();
  EXPECT_EQ(triMesh.getTriangles().size(), 2u);
}

//==============================================================================
TEST(PolygonMeshTests, PreservesVertexNormals)
{
  PolygonMeshd mesh;
  mesh.reserveVertices(3);
  mesh.reserveVertexNormals(3);
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addVertex(0.0, 1.0, 0.0);

  mesh.addVertexNormal(0.0, 0.0, 1.0);
  mesh.addVertexNormal(0.0, 0.0, 1.0);
  mesh.addVertexNormal(0.0, 0.0, 1.0);

  mesh.addFace({0, 1, 2});
  const auto triMesh = mesh.triangulate();

  ASSERT_TRUE(triMesh.hasVertexNormals());
  ASSERT_EQ(triMesh.getVertexNormals().size(), triMesh.getVertices().size());
  EXPECT_TRUE(triMesh.getVertexNormals()[0].isApprox(
      Eigen::Vector3d(0.0, 0.0, 1.0), 1e-12));
}

//==============================================================================
TEST(PolygonMeshTests, TriangulateIgnoresSmallFaces)
{
  PolygonMeshd mesh;
  mesh.addVertex(0.0, 0.0, 0.0);
  mesh.addVertex(1.0, 0.0, 0.0);
  mesh.addFace({0, 1});

  const auto triMesh = mesh.triangulate();
  EXPECT_EQ(triMesh.getTriangles().size(), 0u);
}
