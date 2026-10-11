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

#include <dart/gui/detail/geometry.hpp>
#include <dart/gui/detail/interaction.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/MultiSphereConvexHullShape.hpp>
#include <dart/dynamics/PyramidShape.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace {
using namespace dart;
using namespace dart::gui;
RenderableDescriptor pickFixture(const dynamics::Shape& shape)
{
  RenderableDescriptor d;
  d.id = 1;
  d.geometry = *describeShape(shape);
  return d;
}
RenderableDescriptor triangleFixture()
{
  RenderableDescriptor d;
  d.id = 1;
  d.geometry.kind = ShapeKind::Mesh;
  d.geometry.hasLocalBounds = true;
  d.geometry.localBoundsMin = {-1, -1, 0};
  d.geometry.localBoundsMax = {1, 1, 0};
  d.geometry.triangleVertices = {{-1, -1, 0}, {1, -1, 0}, {0, 1, 0}};
  d.geometry.triangleIndices = {{0, 1, 2}};
  return d;
}
VisualPickOptions viewportFixture()
{
  VisualPickOptions v;
  v.eye = {0, 0, 4};
  v.center = {0, 0, 0};
  v.up = {0, 1, 0};
  v.fovyDegrees = 60;
  v.width = 800;
  v.height = 600;
  return v;
}
PickRay rayFromPixel(const VisualPickOptions& v, double x, double y)
{
  const double tangent = std::tan(v.fovyDegrees * 3.14159265358979323846 / 360);
  const Eigen::Vector3d forward = (v.center - v.eye).normalized();
  const Eigen::Vector3d right = forward.cross(v.up).normalized();
  const Eigen::Vector3d up = right.cross(forward);
  return {
      v.eye,
      (forward + right * ((2 * x / v.width - 1) * tangent * v.width / v.height)
       + up * ((1 - 2 * y / v.height) * tangent))
          .normalized()};
}
} // namespace

TEST(GuiInteractions, MeshHolesAndEmptyMeshesNeverPickBoundingBoxes)
{
  auto mesh = triangleFixture();
  mesh.geometry.localBoundsMin = {-2, -2, 0};
  mesh.geometry.localBoundsMax = {2, 2, 0};
  mesh.geometry.triangleVertices
      = {{-2, -2, 0},
         {-1, -2, 0},
         {-1, 2, 0},
         {-2, 2, 0},
         {1, -2, 0},
         {2, -2, 0},
         {2, 2, 0},
         {1, 2, 0}};
  mesh.geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}, {4, 5, 6}, {4, 6, 7}};
  EXPECT_FALSE(intersectRenderableHit(mesh, {{0, 0, 3}, {0, 0, -1}}));
  const auto edge = intersectRenderableHit(mesh, {{-1.5, 0, 3}, {0, 0, -2}});
  ASSERT_TRUE(edge);
  EXPECT_NEAR(edge->distance, 3, 1e-12);
  EXPECT_TRUE(edge->normal.isApprox(Eigen::Vector3d::UnitZ()));
  mesh.geometry.triangleIndices.clear();
  EXPECT_FALSE(intersectRenderableHit(mesh, {{-1.5, 0, 3}, {0, 0, -1}}));
  mesh.geometry.triangleIndices = {{-1, 0, 1}, {0, 1, 500}};
  EXPECT_FALSE(intersectRenderableHit(mesh, {{-1.5, 0, 3}, {0, 0, -1}}));
}

TEST(GuiInteractions, HiddenFrontGeometryAndDistanceLimits)
{
  auto front = pickFixture(dynamics::BoxShape(Eigen::Vector3d::Ones()));
  auto back = front;
  front.id = 1;
  back.id = 2;
  front.worldTransform.translation() = Eigen::Vector3d(0, 0, 2);
  front.material.visible = false;
  const PickRay ray{{0, 0, 5}, {0, 0, -3}};
  const auto hit = pickNearestRenderable({front, back}, ray);
  ASSERT_TRUE(hit);
  EXPECT_EQ(hit->id, 2u);
  EXPECT_EQ(hit->renderableIndex, 1u);
  EXPECT_NEAR(hit->distance, 4.5, 1e-12);
  EXPECT_FALSE(pickNearestRenderable({front, back}, ray, 4));
  EXPECT_FALSE(pickNearestRenderable({front, back}, ray, -1));
}

TEST(GuiInteractions, AffineMeshDistancesAndNormals)
{
  auto mesh = triangleFixture();
  mesh.geometry.triangleVertices = {{-1, -1, -1}, {1, -1, 1}, {0, 1, 0}};
  const Eigen::Matrix3d rotation
      = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized())
            .toRotationMatrix();
  mesh.worldTransform.linear()
      = rotation * Eigen::Vector3d(2, 0.5, 3).asDiagonal();
  mesh.worldTransform.translation() = Eigen::Vector3d(3, -2, 1);
  const Eigen::Vector3d normal
      = mesh.worldTransform.linear().inverse().transpose()
        * Eigen::Vector3d(-1, 0, 1).normalized();
  const Eigen::Vector3d unit = normal.normalized();
  const Eigen::Vector3d expected
      = mesh.worldTransform * Eigen::Vector3d::Zero();
  const auto hit
      = intersectRenderableHit(mesh, {expected + unit * 3, -unit * 7});
  ASSERT_TRUE(hit);
  EXPECT_NEAR(hit->distance, 3, 1e-10);
  EXPECT_TRUE(hit->point.isApprox(expected, 1e-10));
  EXPECT_TRUE(hit->normal.isApprox(unit, 1e-10));
  mesh.worldTransform.linear().col(0).setZero();
  EXPECT_FALSE(intersectRenderableHit(mesh, {expected + unit * 3, -unit}));
}

TEST(GuiInteractions, PrimitiveSurfaceExactnessAndInsideExits)
{
  const PickRay ray{{0.9, 0.9, 3}, {0, 0, -1}};
  EXPECT_FALSE(
      intersectRenderableHit(pickFixture(dynamics::SphereShape(1)), ray));
  EXPECT_FALSE(
      intersectRenderableHit(pickFixture(dynamics::CylinderShape(1, 2)), ray));
  EXPECT_FALSE(
      intersectRenderableHit(pickFixture(dynamics::CapsuleShape(1, 2)), ray));
  const auto sphere = intersectRenderableHit(
      pickFixture(dynamics::SphereShape(1)), {{0, 0, 3}, {0, 0, -4}});
  ASSERT_TRUE(sphere);
  EXPECT_NEAR(sphere->distance, 2, 1e-12);
  EXPECT_TRUE(sphere->point.isApprox(Eigen::Vector3d::UnitZ()));
  const auto inside = intersectRenderableHit(
      pickFixture(dynamics::BoxShape(Eigen::Vector3d::Constant(2))),
      {{0, 0, 0}, {5, 0, 0}});
  ASSERT_TRUE(inside);
  EXPECT_NEAR(inside->distance, 1, 1e-12);
  EXPECT_TRUE(inside->normal.isApprox(Eigen::Vector3d::UnitX()));
  const auto ellipsoid = intersectRenderableHit(
      pickFixture(dynamics::EllipsoidShape({2, 4, 6})),
      {{0, 0, 5}, {0, 0, -1}});
  ASSERT_TRUE(ellipsoid);
  EXPECT_NEAR(ellipsoid->distance, 2, 1e-12);
  const auto cone = intersectRenderableHit(
      pickFixture(dynamics::ConeShape(1, 2)), {{0.75, 0, 3}, {0, 0, -1}});
  ASSERT_TRUE(cone);
  EXPECT_NEAR(cone->distance, 3.5, 1e-12);
  const auto pyramid = intersectRenderableHit(
      pickFixture(dynamics::PyramidShape(2, 2, 2)), {{0.75, 0, 3}, {0, 0, -1}});
  ASSERT_TRUE(pyramid);
  EXPECT_NEAR(pyramid->distance, 3.5, 1e-12);
}

TEST(GuiInteractions, MultiSphereConvexHullIncludesBridge)
{
  dynamics::MultiSphereConvexHullShape shape({});
  shape.addSphere(0.5, {-1, 0, 0});
  shape.addSphere(0.5, {1, 0, 0});
  const auto descriptor = pickFixture(shape);
  ASSERT_FALSE(descriptor.geometry.triangleIndices.empty());
  const auto hit = intersectRenderableHit(descriptor, {{0, 0, 3}, {0, 0, -1}});
  ASSERT_TRUE(hit);
  EXPECT_NEAR(hit->distance, 2.5, 1e-6);
  EXPECT_TRUE(hit->normal.allFinite());
}

TEST(GuiInteractions, LineConnectionsHavePixelWidthsAndGaps)
{
  auto line = triangleFixture();
  line.geometry.kind = ShapeKind::LineSegments;
  line.geometry.lineVertices
      = {{-1, 0, 0}, {-0.2, 0, 0}, {0.2, 0, 0}, {1, 0, 0}};
  line.geometry.lineConnections = {{0, 1}, {2, 3}, {0, 100}, {-1, 0}};
  line.geometry.lineThickness = 6;
  const auto viewport = viewportFixture();
  const double x = 400 - 0.5 * 300 / std::tan(3.14159265358979323846 / 6) / 4;
  EXPECT_FALSE(
      intersectRenderableHit(line, rayFromPixel(viewport, 400, 300), viewport));
  const auto hit
      = intersectRenderableHit(line, rayFromPixel(viewport, x, 302), viewport);
  ASSERT_TRUE(hit);
  EXPECT_NEAR(hit->point.z(), 0, 1e-10);
  EXPECT_FALSE(
      intersectRenderableHit(line, rayFromPixel(viewport, x, 304), viewport));
  EXPECT_FALSE(intersectRenderableHit(line, rayFromPixel(viewport, x, 300)));
}

TEST(GuiInteractions, PointModesAndPerLeafVoxelSizes)
{
  auto cloud = triangleFixture();
  cloud.geometry.kind = ShapeKind::PointCloud;
  cloud.geometry.pointCloudPoints = {{0, 0, 0}};
  cloud.geometry.pointSize = 1;
  const auto viewport = viewportFixture();
  cloud.geometry.pointShapeType = PointShapeType::Box;
  const auto box = intersectRenderableHit(cloud, {{0, 0, 4}, {0, 0, -1}});
  ASSERT_TRUE(box);
  EXPECT_NEAR(box->distance, 3.5, 1e-12);
  cloud.geometry.pointShapeType = PointShapeType::BillboardSquare;
  const auto square
      = intersectRenderableHit(cloud, {{0.45, 0.45, 4}, {0, 0, -1}}, viewport);
  ASSERT_TRUE(square);
  EXPECT_NEAR(square->distance, 4, 1e-12);
  cloud.geometry.pointShapeType = PointShapeType::BillboardCircle;
  EXPECT_FALSE(
      intersectRenderableHit(cloud, {{0.45, 0.45, 4}, {0, 0, -1}}, viewport));
  EXPECT_TRUE(
      intersectRenderableHit(cloud, {{0.2, 0.2, 4}, {0, 0, -1}}, viewport));
  cloud.geometry.pointShapeType = PointShapeType::Point;
  cloud.geometry.pointSize = 10;
  EXPECT_TRUE(intersectRenderableHit(
      cloud, rayFromPixel(viewport, 404, 300), viewport));
  EXPECT_FALSE(intersectRenderableHit(
      cloud, rayFromPixel(viewport, 406, 300), viewport));
  EXPECT_FALSE(intersectRenderableHit(cloud, rayFromPixel(viewport, 400, 300)));
  auto voxel = cloud;
  voxel.geometry.kind = ShapeKind::VoxelGrid;
  voxel.geometry.voxelCenters = {{-1, 0, 0}, {1, 0, 0}};
  voxel.geometry.voxelSize = 2;
  voxel.geometry.voxelSizes = {0.2, 1};
  EXPECT_FALSE(intersectRenderableHit(voxel, {{-0.5, 0, 4}, {0, 0, -1}}));
  const auto leaf = intersectRenderableHit(voxel, {{1.4, 0, 4}, {0, 0, -1}});
  ASSERT_TRUE(leaf);
  EXPECT_NEAR(leaf->distance, 3.5, 1e-12);
}

TEST(GuiInteractions, ConstrainedDragsRejectParallelAndBehindRays)
{
  const PickRay a{{0, 0, 3}, {0, 0, -1}}, b{{1, 2, 3}, {0, 0, -2}};
  const auto plane = computePlaneDragTranslation(
      a, b, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(plane);
  EXPECT_TRUE(plane->isApprox(Eigen::Vector3d(1, 2, 0)));
  const auto axis = computeAxisDragTranslation(
      a, b, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  ASSERT_TRUE(axis);
  EXPECT_TRUE(axis->isApprox(Eigen::Vector3d(1, 0, 0)));
  EXPECT_FALSE(computePlaneDragTranslation(
      a,
      {{0, 0, 3}, {1, 0, 0}},
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ()));
  EXPECT_FALSE(computeAxisDragTranslation(
      a,
      {{0, 0, 3}, {1, 0, 0}},
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(intersectPlane(
      {{0, 0, 3}, {0, 0, 1}},
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ()));
}

TEST(GuiInteractions, MeshPickNormalsInterpolateAuthoredVertexNormals)
{
  auto mesh = triangleFixture();
  const Eigen::Vector3d tilted = Eigen::Vector3d(0, 1, 1).normalized();
  mesh.geometry.triangleNormals
      = {Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ(), tilted};
  const auto hit = intersectRenderableHit(mesh, {{0, 0, 3}, {0, 0, -1}});
  ASSERT_TRUE(hit);
  const Eigen::Vector3d expected
      = (Eigen::Vector3d::UnitZ() * 0.5 + tilted * 0.5).normalized();
  EXPECT_TRUE(hit->normal.isApprox(expected, 1e-12));
  EXPECT_NEAR(hit->distance, 3, 1e-12);
}
