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

#include <dart/collision/dart/shapes/Shape.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>

#include <cmath>

using namespace dart::collision::native;

namespace {

Aabb computeTriangleAabb(
    const std::vector<Eigen::Vector3d>& vertices,
    const MeshShape::Triangle& triangle)
{
  const Eigen::Vector3d& v0 = vertices[static_cast<std::size_t>(triangle[0])];
  const Eigen::Vector3d& v1 = vertices[static_cast<std::size_t>(triangle[1])];
  const Eigen::Vector3d& v2 = vertices[static_cast<std::size_t>(triangle[2])];
  return Aabb(v0.cwiseMin(v1).cwiseMin(v2), v0.cwiseMax(v1).cwiseMax(v2));
}

void expectValidMeshBvh(const MeshShape& mesh, bool enforceLeafSizeLimit)
{
  const auto& nodes = mesh.bvhNodes();
  const auto& triOrder = mesh.bvhTriIndices();
  const auto& triangles = mesh.getTriangles();
  ASSERT_FALSE(nodes.empty());
  ASSERT_EQ(triOrder.size(), triangles.size());

  std::vector<int> visitCounts(triangles.size(), 0);
  const auto visitNode = [&](auto&& self, int nodeIndex) -> void {
    ASSERT_GE(nodeIndex, 0);
    ASSERT_LT(static_cast<std::size_t>(nodeIndex), nodes.size());

    const auto& node = nodes[static_cast<std::size_t>(nodeIndex)];
    EXPECT_TRUE(node.box.min.allFinite());
    EXPECT_TRUE(node.box.max.allFinite());
    EXPECT_TRUE((node.box.max.array() >= node.box.min.array()).all());

    if (node.left < 0 && node.right < 0) {
      ASSERT_GE(node.first, 0);
      ASSERT_GT(node.count, 0);
      if (enforceLeafSizeLimit) {
        EXPECT_LE(node.count, 4);
      }
      ASSERT_LE(
          static_cast<std::size_t>(node.first + node.count), triOrder.size());

      for (int i = 0; i < node.count; ++i) {
        const int triangleIndex
            = triOrder[static_cast<std::size_t>(node.first + i)];
        ASSERT_GE(triangleIndex, 0);
        ASSERT_LT(static_cast<std::size_t>(triangleIndex), triangles.size());
        ++visitCounts[static_cast<std::size_t>(triangleIndex)];
        EXPECT_TRUE(node.box.contains(computeTriangleAabb(
            mesh.getVertices(),
            triangles[static_cast<std::size_t>(triangleIndex)])));
      }
      return;
    }

    ASSERT_GE(node.left, 0);
    ASSERT_GE(node.right, 0);
    EXPECT_EQ(node.first, 0);
    EXPECT_EQ(node.count, 0);
    ASSERT_LT(static_cast<std::size_t>(node.left), nodes.size());
    ASSERT_LT(static_cast<std::size_t>(node.right), nodes.size());
    EXPECT_TRUE(
        node.box.contains(nodes[static_cast<std::size_t>(node.left)].box));
    EXPECT_TRUE(
        node.box.contains(nodes[static_cast<std::size_t>(node.right)].box));
    self(self, node.left);
    self(self, node.right);
  };

  visitNode(visitNode, 0);
  for (const int count : visitCounts) {
    EXPECT_EQ(count, 1);
  }
}

std::vector<Eigen::Vector3d> makeCubeVertices(double scale)
{
  const double half = 0.5 * scale;
  return {
      {-half, -half, -half},
      {half, -half, -half},
      {-half, half, -half},
      {half, half, -half},
      {-half, -half, half},
      {half, -half, half},
      {-half, half, half},
      {half, half, half},
  };
}

std::vector<Eigen::Vector3d> makeCenteredTetrahedronVertices(double scale)
{
  const double zBase = -1.0 / (2.0 * std::sqrt(6.0));
  return {
      scale * Eigen::Vector3d(0.5, -0.5 / std::sqrt(3.0), zBase),
      scale * Eigen::Vector3d(-0.5, -0.5 / std::sqrt(3.0), zBase),
      scale * Eigen::Vector3d(0.0, 1.0 / std::sqrt(3.0), zBase),
      scale * Eigen::Vector3d(0.0, 0.0, std::sqrt(3.0 / 8.0)),
  };
}

void expectLocalAabbMatchesVertices(
    const std::vector<Eigen::Vector3d>& vertices, double tolerance)
{
  ASSERT_FALSE(vertices.empty());
  ConvexShape convex(vertices);

  Eigen::Vector3d expectedMin = vertices.front();
  Eigen::Vector3d expectedMax = vertices.front();
  for (const auto& vertex : vertices) {
    expectedMin = expectedMin.cwiseMin(vertex);
    expectedMax = expectedMax.cwiseMax(vertex);
  }

  const auto aabb = convex.computeLocalAabb();
  EXPECT_TRUE(aabb.min.isApprox(expectedMin, tolerance));
  EXPECT_TRUE(aabb.max.isApprox(expectedMax, tolerance));
}

} // namespace

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
  EXPECT_NE(ShapeType::Cylinder, ShapeType::Plane);
  EXPECT_NE(ShapeType::Plane, ShapeType::Mesh);
  EXPECT_NE(ShapeType::Mesh, ShapeType::Convex);
  EXPECT_NE(ShapeType::Convex, ShapeType::Sdf);
  EXPECT_NE(ShapeType::Sdf, ShapeType::Compound);
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

TEST(CapsuleShape, ComputeLocalAabbAcrossRadiusHeightCases)
{
  const std::array<std::pair<double, double>, 4> cases{{
      {2.0, 3.0},
      {20.0, 30.0},
      {3.0, 2.0},
      {30.0, 20.0},
  }};

  for (const auto& [radius, height] : cases) {
    CapsuleShape capsule(radius, height);

    const auto aabb = capsule.computeLocalAabb();

    const Eigen::Vector3d expectedHalfExtents(
        radius, radius, 0.5 * height + radius);
    EXPECT_EQ(aabb.center(), Eigen::Vector3d::Zero());
    EXPECT_EQ(aabb.halfExtents(), expectedHalfExtents);
  }
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

TEST(ConvexShape, ScaledPolytopeLocalBounds)
{
  const std::array<double, 3> scales{{0.001, 1.0, 1000.0}};

  for (const double scale : scales) {
    const double tolerance = 1e-12 * std::max(1.0, scale);
    expectLocalAabbMatchesVertices(makeCubeVertices(scale), tolerance);
    expectLocalAabbMatchesVertices(
        makeCenteredTetrahedronVertices(scale), tolerance);
  }
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

TEST(ConvexShape, SupportMatchesExhaustiveVertexSearch)
{
  const std::vector<Eigen::Vector3d> vertices{
      {-1, -1, -1},
      {1, -1, -1},
      {1, 1, -1},
      {-1, 1, -1},
      {-1, -1, 1},
      {1, -1, 1},
      {1, 1, 1},
      {-1, 1, 1}};
  const ConvexShape convex(vertices);
  const std::array<Eigen::Vector3d, 5> directions{{
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1},
      {1, 1, 0},
      {-1, 1, 1},
  }};

  for (const auto& direction : directions) {
    const Eigen::Vector3d support = convex.support(direction);
    const double supportDot = support.dot(direction);

    double expectedDot = -std::numeric_limits<double>::infinity();
    for (const auto& vertex : vertices) {
      const double vertexDot = vertex.dot(direction);
      if (vertexDot > expectedDot) {
        expectedDot = vertexDot;
      }
    }

    EXPECT_DOUBLE_EQ(supportDot, expectedDot);
  }
}

TEST(ConvexShape, TetrahedronSupportVerticesAcrossScales)
{
  const std::array<double, 3> scales{{0.001, 1.0, 1000.0}};

  for (const double scale : scales) {
    const auto vertices = makeCenteredTetrahedronVertices(scale);
    const ConvexShape convex(vertices);
    const double tolerance = 1e-12 * std::max(1.0, scale);

    for (const auto& vertex : vertices) {
      EXPECT_TRUE(convex.support(vertex).isApprox(vertex, tolerance));
    }
  }
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

TEST(MeshShape, BvhSingleTriangle)
{
  std::vector<Eigen::Vector3d> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  std::vector<MeshShape::Triangle> triangles = {{0, 1, 2}};

  MeshShape mesh(vertices, triangles);

  ASSERT_EQ(mesh.bvhTriIndices().size(), 1u);
  ASSERT_EQ(mesh.bvhNodes().size(), 1u);
  EXPECT_EQ(mesh.bvhTriIndices()[0], 0);

  const auto& root = mesh.bvhNodes()[0];
  EXPECT_EQ(root.left, -1);
  EXPECT_EQ(root.right, -1);
  EXPECT_EQ(root.first, 0);
  EXPECT_EQ(root.count, 1);
}

TEST(MeshShape, BvhLargeMesh)
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<MeshShape::Triangle> triangles;
  constexpr int kResolution = 32;
  vertices.reserve(
      static_cast<std::size_t>((kResolution + 1) * (kResolution + 1)));
  triangles.reserve(static_cast<std::size_t>(kResolution * kResolution * 2));

  for (int y = 0; y <= kResolution; ++y) {
    for (int x = 0; x <= kResolution; ++x) {
      vertices.emplace_back(
          static_cast<double>(x), static_cast<double>(y), 0.0);
    }
  }

  const auto idx = [](int x, int y) {
    return y * (kResolution + 1) + x;
  };

  for (int y = 0; y < kResolution; ++y) {
    for (int x = 0; x < kResolution; ++x) {
      triangles.emplace_back(idx(x, y), idx(x + 1, y), idx(x + 1, y + 1));
      triangles.emplace_back(idx(x, y), idx(x + 1, y + 1), idx(x, y + 1));
    }
  }

  MeshShape mesh(vertices, triangles);

  ASSERT_EQ(mesh.bvhTriIndices().size(), triangles.size());
  ASSERT_FALSE(mesh.bvhNodes().empty());
  expectValidMeshBvh(mesh, true);

  const auto localAabb = mesh.computeLocalAabb();
  const auto& rootBox = mesh.bvhNodes()[0].box;
  EXPECT_EQ(rootBox.min, localAabb.min);
  EXPECT_EQ(rootBox.max, localAabb.max);
  if (triangles.size() > 4) {
    const auto& root = mesh.bvhNodes()[0];
    EXPECT_EQ(root.count, 0);
    EXPECT_GE(root.left, 0);
    EXPECT_GE(root.right, 0);
  }
}

TEST(MeshShape, BvhDegenerateTriangles)
{
  std::vector<Eigen::Vector3d> vertices
      = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {0, 0, 0}, {1, 0, 0}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 0, 0}, {0, 1, 2}, {3, 3, 4}, {1, 1, 1}, {2, 4, 2}};

  MeshShape mesh(vertices, triangles);

  ASSERT_EQ(mesh.bvhTriIndices().size(), triangles.size());
  ASSERT_FALSE(mesh.bvhNodes().empty());
  expectValidMeshBvh(mesh, false);

  const auto localAabb = mesh.computeLocalAabb();
  const auto& rootBox = mesh.bvhNodes()[0].box;
  EXPECT_TRUE(rootBox.contains(localAabb));
  EXPECT_TRUE(rootBox.min.allFinite());
  EXPECT_TRUE(rootBox.max.allFinite());
}

// DART 6 supplementary coverage tests (not ported from DART 7)
namespace {

class FakeSdfField final : public SignedDistanceField
{
public:
  bool distance(
      const Eigen::Vector3d&,
      double* distance,
      const SdfQueryOptions&) const override
  {
    *distance = 0.25;
    return true;
  }

  bool distanceAndGradient(
      const Eigen::Vector3d&,
      double* distance,
      Eigen::Vector3d* gradient,
      const SdfQueryOptions&) const override
  {
    *distance = 0.25;
    *gradient = Eigen::Vector3d::UnitX();
    return true;
  }

  void batchDistanceAndGradient(
      span<const Eigen::Vector3d> points_F,
      span<double> distances,
      span<Eigen::Vector3d> gradients,
      span<std::uint8_t> observed,
      const SdfQueryOptions&) const override
  {
    for (std::size_t i = 0; i < points_F.size(); ++i) {
      distances[i] = 0.25;
      gradients[i] = Eigen::Vector3d::UnitX();
      observed[i] = 1u;
    }
  }

  Aabb localAabb() const override
  {
    return Aabb(Eigen::Vector3d(-1.0, -2.0, -3.0), Eigen::Vector3d::Ones());
  }

  double voxelSize() const override
  {
    return 0.1;
  }

  double maxDistance() const override
  {
    return 4.0;
  }
};

} // namespace

TEST(PlaneShape, NormalOffsetAndUnboundedAabb)
{
  PlaneShape plane(Eigen::Vector3d(0.0, 0.0, 2.0), -0.75);

  EXPECT_EQ(plane.getType(), ShapeType::Plane);
  EXPECT_TRUE(plane.getNormal().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_DOUBLE_EQ(plane.getOffset(), -0.75);

  const Aabb aabb = plane.computeLocalAabb();
  EXPECT_FALSE(aabb.min.allFinite());
  EXPECT_FALSE(aabb.max.allFinite());
  EXPECT_LT(aabb.min.x(), 0.0);
  EXPECT_GT(aabb.max.x(), 0.0);
}

TEST(SdfShape, NullAndFieldAabbs)
{
  SdfShape empty(nullptr);

  EXPECT_EQ(empty.getType(), ShapeType::Sdf);
  EXPECT_EQ(empty.getField(), nullptr);
  EXPECT_EQ(empty.computeLocalAabb().min, Eigen::Vector3d::Zero());
  EXPECT_EQ(empty.computeLocalAabb().max, Eigen::Vector3d::Zero());

  std::shared_ptr<const SignedDistanceField> field(new FakeSdfField);
  SdfShape sdf(field);

  EXPECT_EQ(sdf.getField(), field.get());
  const Aabb aabb = sdf.computeLocalAabb();
  EXPECT_EQ(aabb.min, Eigen::Vector3d(-1.0, -2.0, -3.0));
  EXPECT_EQ(aabb.max, Eigen::Vector3d::Ones());
}

TEST(CompoundShape, SkipsNullChildrenAndMergesTransformedAabbs)
{
  CompoundShape compound;
  EXPECT_EQ(compound.getType(), ShapeType::Compound);
  EXPECT_EQ(compound.computeLocalAabb().min, Eigen::Vector3d::Zero());
  EXPECT_EQ(compound.computeLocalAabb().max, Eigen::Vector3d::Zero());

  Eigen::Isometry3d ignoredTransform = Eigen::Isometry3d::Identity();
  ignoredTransform.translation() = Eigen::Vector3d(100.0, 0.0, 0.0);
  compound.addChild(nullptr, ignoredTransform);
  EXPECT_EQ(compound.numChildren(), 1u);
  EXPECT_EQ(compound.computeLocalAabb().min, Eigen::Vector3d::Zero());
  EXPECT_EQ(compound.computeLocalAabb().max, Eigen::Vector3d::Zero());

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
  boxTransform.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  compound.addChild(
      std::unique_ptr<Shape>(new BoxShape(Eigen::Vector3d::Ones())),
      boxTransform);

  Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();
  sphereTransform.translation() = Eigen::Vector3d(-1.0, 0.0, 0.0);
  compound.addChild(
      std::unique_ptr<Shape>(new SphereShape(0.5)), sphereTransform);

  EXPECT_EQ(compound.numChildren(), 3u);
  EXPECT_EQ(compound.childShape(1).getType(), ShapeType::Box);
  EXPECT_EQ(
      compound.childTransform(2).translation(), sphereTransform.translation());

  const Aabb merged = compound.computeLocalAabb();
  EXPECT_EQ(merged.min, Eigen::Vector3d(-1.5, -1.0, -1.0));
  EXPECT_EQ(merged.max, Eigen::Vector3d(3.0, 1.0, 1.0));

  compound.removeChild(42u);
  EXPECT_EQ(compound.numChildren(), 3u);

  compound.removeChild(0u);
  EXPECT_EQ(compound.numChildren(), 2u);
  EXPECT_EQ(compound.children().size(), 2u);
  EXPECT_EQ(compound.computeLocalAabb().min, merged.min);
  EXPECT_EQ(compound.computeLocalAabb().max, merged.max);
}
