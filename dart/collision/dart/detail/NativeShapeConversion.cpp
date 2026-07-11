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

#include "dart/collision/dart/detail/NativeShapeConversion.hpp"

#include "dart/collision/dart/shapes/Shape.hpp"
#include "dart/common/Console.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConvexMeshShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PyramidShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SphereShape.hpp"

#if HAVE_OCTOMAP
  #include "dart/dynamics/VoxelGridShape.hpp"
#endif

#include <limits>
#include <set>
#include <string>
#include <vector>

namespace dart {
namespace collision {
namespace detail {

namespace {

std::vector<native::ConvexShape::Face> makeConvexFacesFromTriangles(
    const std::vector<Eigen::Vector3d>& vertices,
    const dynamics::ConvexMeshShape::Triangles& triangles)
{
  std::vector<native::ConvexShape::Face> faces;
  faces.reserve(triangles.size());

  for (const auto& triangle : triangles) {
    if (triangle[0] >= vertices.size() || triangle[1] >= vertices.size()
        || triangle[2] >= vertices.size()) {
      continue;
    }

    const Eigen::Vector3d& v0 = vertices[triangle[0]];
    const Eigen::Vector3d& v1 = vertices[triangle[1]];
    const Eigen::Vector3d& v2 = vertices[triangle[2]];
    faces.push_back({v0, (v1 - v0).cross(v2 - v0)});
  }

  return faces;
}

std::unique_ptr<native::Shape> createConvexOrNull(
    std::vector<Eigen::Vector3d> vertices,
    const std::string& shapeType,
    std::vector<native::ConvexShape::Face> faces = {})
{
  if (vertices.empty()) {
    static std::set<std::string> warnedInvalidShapeTypes;
    if (warnedInvalidShapeTypes.insert(shapeType).second) {
      dtwarn << "[NativeShapeConversion] Shape type [" << shapeType
             << "] did not provide convex vertices. This shape will be "
             << "skipped by the native adapter.\n";
    }
    return nullptr;
  }

  return std::make_unique<native::ConvexShape>(
      std::move(vertices), std::move(faces));
}

std::unique_ptr<native::Shape> createMeshOrNull(
    const dynamics::MeshShape& meshShape, const std::string& shapeType)
{
  auto mesh = meshShape.getTriMesh();
  if (!mesh || mesh->getVertices().empty() || mesh->getTriangles().empty()) {
    static std::set<std::string> warnedInvalidShapeTypes;
    if (warnedInvalidShapeTypes.insert(shapeType).second) {
      dtwarn << "[NativeShapeConversion] Shape type [" << shapeType
             << "] did not provide triangle mesh data. This shape will be "
             << "skipped by the native adapter.\n";
    }
    return nullptr;
  }

  const auto& sourceVertices = mesh->getVertices();
  const auto& sourceTriangles = mesh->getTriangles();

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(sourceVertices.size());
  const Eigen::Vector3d& scale = meshShape.getScale();
  for (const auto& vertex : sourceVertices) {
    vertices.emplace_back(vertex.cwiseProduct(scale));
  }

  std::vector<native::MeshShape::Triangle> triangles;
  triangles.reserve(sourceTriangles.size());
  const auto maxInt = static_cast<std::size_t>(std::numeric_limits<int>::max());
  for (const auto& triangle : sourceTriangles) {
    if (triangle[0] >= sourceVertices.size()
        || triangle[1] >= sourceVertices.size()
        || triangle[2] >= sourceVertices.size() || triangle[0] > maxInt
        || triangle[1] > maxInt || triangle[2] > maxInt) {
      static std::set<std::string> warnedInvalidShapeTypes;
      if (warnedInvalidShapeTypes.insert(shapeType).second) {
        dtwarn << "[NativeShapeConversion] Shape type [" << shapeType
               << "] has triangle indices that cannot be represented by the "
               << "native mesh adapter. This shape will be skipped.\n";
      }
      return nullptr;
    }

    triangles.emplace_back(
        static_cast<int>(triangle[0]),
        static_cast<int>(triangle[1]),
        static_cast<int>(triangle[2]));
  }

  return std::make_unique<native::MeshShape>(
      std::move(vertices), std::move(triangles));
}

std::vector<Eigen::Vector3d> makePyramidVertices(
    double baseWidth, double baseDepth, double height)
{
  const double halfWidth = 0.5 * baseWidth;
  const double halfDepth = 0.5 * baseDepth;
  const double baseZ = -0.5 * height;
  return {
      {-halfWidth, -halfDepth, baseZ},
      {halfWidth, -halfDepth, baseZ},
      {halfWidth, halfDepth, baseZ},
      {-halfWidth, halfDepth, baseZ},
      {0.0, 0.0, 0.5 * height}};
}

#if HAVE_OCTOMAP
std::unique_ptr<native::Shape> createVoxelGrid(
    const dynamics::VoxelGridShape& voxelGrid)
{
  auto octree = voxelGrid.getOctree();
  auto compound = std::make_unique<native::CompoundShape>();
  if (!octree)
    return compound;

  const double occupancyThreshold = octree->getOccupancyThres();
  for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end;
       ++it) {
    if (it->getOccupancy() < occupancyThreshold)
      continue;

    const auto coordinate = it.getCoordinate();
    Eigen::Isometry3d voxelTransform = Eigen::Isometry3d::Identity();
    voxelTransform.translation()
        = Eigen::Vector3d(coordinate.x(), coordinate.y(), coordinate.z());

    compound->addChild(
        std::make_unique<native::BoxShape>(
            Eigen::Vector3d::Constant(0.5 * it.getSize())),
        voxelTransform);
  }

  return compound;
}
#endif

} // namespace

//==============================================================================
std::unique_ptr<native::Shape> NativeShapeConversion::create(
    const dynamics::Shape& shape)
{
  const auto& shapeType = shape.getType();

  if (shapeType == dynamics::SphereShape::getStaticType()) {
    const auto& sphere = static_cast<const dynamics::SphereShape&>(shape);
    return std::make_unique<native::SphereShape>(sphere.getRadius());
  }

  if (shapeType == dynamics::BoxShape::getStaticType()) {
    const auto& box = static_cast<const dynamics::BoxShape&>(shape);
    return std::make_unique<native::BoxShape>(0.5 * box.getSize());
  }

  if (shapeType == dynamics::CapsuleShape::getStaticType()) {
    const auto& capsule = static_cast<const dynamics::CapsuleShape&>(shape);
    return std::make_unique<native::CapsuleShape>(
        capsule.getRadius(), capsule.getHeight());
  }

  if (shapeType == dynamics::CylinderShape::getStaticType()) {
    const auto& cylinder = static_cast<const dynamics::CylinderShape&>(shape);
    return std::make_unique<native::CylinderShape>(
        cylinder.getRadius(), cylinder.getHeight());
  }

  if (shapeType == dynamics::PlaneShape::getStaticType()) {
    const auto& plane = static_cast<const dynamics::PlaneShape&>(shape);
    return std::make_unique<native::PlaneShape>(
        plane.getNormal(), plane.getOffset());
  }

  if (shapeType == dynamics::MeshShape::getStaticType()) {
    const auto& mesh = static_cast<const dynamics::MeshShape&>(shape);
    return createMeshOrNull(mesh, shapeType);
  }

  if (shapeType == dynamics::ConvexMeshShape::getStaticType()) {
    const auto& convex = static_cast<const dynamics::ConvexMeshShape&>(shape);
    const auto& mesh = convex.getMesh();
    if (!mesh) {
      return createConvexOrNull({}, shapeType);
    }
    return createConvexOrNull(
        mesh->getVertices(),
        shapeType,
        makeConvexFacesFromTriangles(
            mesh->getVertices(), mesh->getTriangles()));
  }

  if (shapeType == dynamics::PyramidShape::getStaticType()) {
    const auto& pyramid = static_cast<const dynamics::PyramidShape&>(shape);
    return createConvexOrNull(
        makePyramidVertices(
            pyramid.getBaseWidth(),
            pyramid.getBaseDepth(),
            pyramid.getHeight()),
        shapeType);
  }

#if HAVE_OCTOMAP
  if (shapeType == dynamics::VoxelGridShape::getStaticType()) {
    const auto& voxelGrid = static_cast<const dynamics::VoxelGridShape&>(shape);
    return createVoxelGrid(voxelGrid);
  }
#endif

  static std::set<std::string> warnedShapeTypes;
  if (warnedShapeTypes.insert(shapeType).second) {
    dtwarn << "[NativeShapeConversion] Shape type [" << shapeType
           << "] is not supported by DARTCollisionDetector yet. This "
           << "shape will be skipped by the native adapter.\n";
  }

  return nullptr;
}

} // namespace detail
} // namespace collision
} // namespace dart
