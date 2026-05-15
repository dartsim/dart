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

#include <dart/config.hpp>

#include <dart/gui/experimental/scene.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/mesh_material.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/multi_sphere_convex_hull_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/point_cloud_shape.hpp>
#include <dart/dynamics/point_mass.hpp>
#include <dart/dynamics/pyramid_shape.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/soft_mesh_shape.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#if DART_HAVE_OCTOMAP
  #include <dart/dynamics/voxel_grid_shape.hpp>
#endif

#include <dart/math/geometry.hpp>
#include <dart/math/tri_mesh.hpp>

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <string>
#include <utility>

#include <cmath>

namespace dart::gui::experimental {

RenderableId makeRenderableId(const dynamics::ShapeFrame& shapeFrame)
{
  return reinterpret_cast<RenderableId>(&shapeFrame);
}

RenderableId makeRenderableId(const dynamics::ShapeNode& shapeNode)
{
  return makeRenderableId(static_cast<const dynamics::ShapeFrame&>(shapeNode));
}

namespace {

void setLocalBounds(
    GeometryDescriptor& descriptor,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max)
{
  descriptor.localBoundsMin = min;
  descriptor.localBoundsMax = max;
  descriptor.hasLocalBounds = true;
}

void setShapeBoundingBoxBounds(
    GeometryDescriptor& descriptor, const dynamics::Shape& shape)
{
  const auto& bounds = shape.getBoundingBox();
  descriptor.size = bounds.getMax() - bounds.getMin();
  setLocalBounds(descriptor, bounds.getMin(), bounds.getMax());
}

Eigen::Vector4d toVector4d(const Eigen::Vector4f& value)
{
  return value.cast<double>();
}

MeshMaterialDescriptor makeMeshMaterialDescriptor(
    const dynamics::MeshMaterial& material)
{
  MeshMaterialDescriptor descriptor;
  descriptor.ambient = toVector4d(material.ambient);
  descriptor.diffuse = toVector4d(material.diffuse);
  descriptor.specular = toVector4d(material.specular);
  descriptor.emissive = toVector4d(material.emissive);
  descriptor.shininess = material.shininess;
  descriptor.metallicFactor = material.metallicFactor;
  descriptor.roughnessFactor = material.roughnessFactor;
  descriptor.baseColorTexturePath = material.baseColorTexturePath;
  descriptor.metallicTexturePath = material.metallicTexturePath;
  descriptor.roughnessTexturePath = material.roughnessTexturePath;
  descriptor.metallicRoughnessTexturePath
      = material.metallicRoughnessTexturePath;
  descriptor.normalTexturePath = material.normalTexturePath;
  descriptor.occlusionTexturePath = material.occlusionTexturePath;
  descriptor.emissiveTexturePath = material.emissiveTexturePath;
  descriptor.textureImagePaths = material.textureImagePaths;
  return descriptor;
}

MeshPartDescriptor makeMeshPartDescriptor(
    const dynamics::MeshShape::SubMeshRange& range)
{
  MeshPartDescriptor descriptor;
  descriptor.vertexOffset = range.vertexOffset;
  descriptor.vertexCount = range.vertexCount;
  descriptor.triangleOffset = range.triangleOffset;
  descriptor.triangleCount = range.triangleCount;
  descriptor.materialIndex = range.materialIndex;
  return descriptor;
}

MeshAlphaMode makeMeshAlphaMode(dynamics::MeshShape::AlphaMode mode)
{
  switch (mode) {
    case dynamics::MeshShape::BLEND:
      return MeshAlphaMode::Blend;
    case dynamics::MeshShape::AUTO:
      return MeshAlphaMode::Auto;
    case dynamics::MeshShape::SHAPE_ALPHA:
      return MeshAlphaMode::ShapeAlpha;
  }

  return MeshAlphaMode::Blend;
}

template <typename Derived>
Eigen::Vector3d makeScaledTriangleNormal(
    const Eigen::MatrixBase<Derived>& sourceNormal,
    const Eigen::Vector3d& scale)
{
  const Eigen::Vector3d source = sourceNormal.template cast<double>();
  if (!source.allFinite()) {
    return Eigen::Vector3d::UnitZ();
  }

  Eigen::Vector3d scaledNormal = source;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(scale[axis]) > 1e-12) {
      scaledNormal[axis] /= scale[axis];
    } else {
      scaledNormal[axis] = 0.0;
    }
  }

  if (!scaledNormal.allFinite() || scaledNormal.squaredNorm() <= 1e-12) {
    scaledNormal = source;
  }

  return scaledNormal.squaredNorm() > 1e-12 ? scaledNormal.normalized()
                                            : Eigen::Vector3d::UnitZ();
}

template <typename S>
void setTriangleMeshData(
    GeometryDescriptor& descriptor,
    const math::TriMesh<S>& triMesh,
    const Eigen::Vector3d& scale = Eigen::Vector3d::Ones())
{
  const auto& vertices = triMesh.getVertices();
  const auto& triangles = triMesh.getTriangles();
  if (vertices.empty() || triangles.empty()
      || vertices.size()
             > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }

  descriptor.triangleVertices.clear();
  descriptor.triangleIndices.clear();
  descriptor.triangleNormals.clear();
  descriptor.triangleVertices.reserve(vertices.size());
  descriptor.triangleIndices.reserve(triangles.size());

  for (const auto& vertex : vertices) {
    const Eigen::Vector3d point
        = scale.cwiseProduct(vertex.template cast<double>());
    if (!point.allFinite()) {
      descriptor.triangleVertices.clear();
      descriptor.triangleIndices.clear();
      descriptor.triangleNormals.clear();
      return;
    }
    descriptor.triangleVertices.push_back(point);
  }

  if (triMesh.hasVertexNormals()
      && triMesh.getVertexNormals().size() == vertices.size()) {
    descriptor.triangleNormals.reserve(triMesh.getVertexNormals().size());
    for (const auto& normal : triMesh.getVertexNormals()) {
      descriptor.triangleNormals.push_back(
          makeScaledTriangleNormal(normal, scale));
    }
  }

  for (const auto& triangle : triangles) {
    if (triangle[0] >= vertices.size() || triangle[1] >= vertices.size()
        || triangle[2] >= vertices.size()) {
      continue;
    }

    descriptor.triangleIndices.emplace_back(
        static_cast<int>(triangle[0]),
        static_cast<int>(triangle[1]),
        static_cast<int>(triangle[2]));
  }
}

template <typename S>
void setHeightmapTriangleData(
    GeometryDescriptor& descriptor,
    const dynamics::HeightmapShape<S>& heightmapShape)
{
  const auto& heightmap = heightmapShape.getHeightField();
  const Eigen::Index rows = heightmap.rows();
  const Eigen::Index cols = heightmap.cols();
  if (rows < 2 || cols < 2
      || static_cast<std::size_t>(heightmap.size())
             > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }

  const auto scale = heightmapShape.getScale().template cast<double>();
  const double spanX = static_cast<double>(cols - 1) * scale.x();
  const double spanY = static_cast<double>(rows - 1) * scale.y();
  const double xOffset = -0.5 * spanX;
  const double yOffset = 0.5 * spanY;

  descriptor.triangleVertices.clear();
  descriptor.triangleIndices.clear();
  descriptor.triangleNormals.clear();
  descriptor.triangleVertices.reserve(
      static_cast<std::size_t>(heightmap.size()));
  for (Eigen::Index row = 0; row < rows; ++row) {
    for (Eigen::Index col = 0; col < cols; ++col) {
      descriptor.triangleVertices.emplace_back(
          static_cast<double>(col) * scale.x() + xOffset,
          -static_cast<double>(row) * scale.y() + yOffset,
          static_cast<double>(heightmap(row, col)) * scale.z());
    }
  }

  const auto vertexIndex = [cols](Eigen::Index row, Eigen::Index col) {
    return static_cast<int>(row * cols + col);
  };

  descriptor.triangleIndices.reserve(
      static_cast<std::size_t>((rows - 1) * (cols - 1) * 2));
  for (Eigen::Index row = 1; row < rows; ++row) {
    for (Eigen::Index col = 1; col < cols; ++col) {
      const int p1 = vertexIndex(row - 1, col - 1);
      const int p2 = vertexIndex(row - 1, col);
      const int p3 = vertexIndex(row, col - 1);
      const int current = vertexIndex(row, col);
      descriptor.triangleIndices.emplace_back(p1, p3, p2);
      descriptor.triangleIndices.emplace_back(p2, p3, current);
    }
  }
}

void setSoftMeshTriangleData(
    GeometryDescriptor& descriptor, const dynamics::SoftMeshShape& softMesh)
{
  const auto triMesh = softMesh.getTriMesh();
  if (triMesh == nullptr || triMesh->getTriangles().empty()) {
    return;
  }

  const auto* softBody = softMesh.getSoftBodyNode();
  const std::size_t vertexCount = softBody != nullptr
                                      ? softBody->getNumPointMasses()
                                      : triMesh->getVertices().size();
  if (vertexCount == 0u
      || vertexCount
             > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return;
  }

  descriptor.triangleVertices.clear();
  descriptor.triangleIndices.clear();
  descriptor.triangleNormals.clear();
  descriptor.triangleVertices.reserve(vertexCount);
  for (std::size_t i = 0u; i < vertexCount; ++i) {
    if (softBody != nullptr) {
      const auto* pointMass = softBody->getPointMass(i);
      if (pointMass == nullptr) {
        descriptor.triangleVertices.clear();
        descriptor.triangleIndices.clear();
        descriptor.triangleNormals.clear();
        return;
      }
      descriptor.triangleVertices.push_back(pointMass->getLocalPosition());
    } else {
      descriptor.triangleVertices.push_back(triMesh->getVertices()[i]);
    }
  }

  descriptor.triangleIndices.reserve(triMesh->getTriangles().size());
  for (const auto& triangle : triMesh->getTriangles()) {
    if (triangle[0] >= vertexCount || triangle[1] >= vertexCount
        || triangle[2] >= vertexCount) {
      continue;
    }
    descriptor.triangleIndices.emplace_back(
        static_cast<int>(triangle[0]),
        static_cast<int>(triangle[1]),
        static_cast<int>(triangle[2]));
  }
}

void setSoftMeshBounds(
    GeometryDescriptor& descriptor, const dynamics::SoftMeshShape& softMesh)
{
  const auto* softBody = softMesh.getSoftBodyNode();
  if (softBody != nullptr && softBody->getNumPointMasses() > 0u) {
    const auto* firstPointMass = softBody->getPointMass(0u);
    if (firstPointMass == nullptr) {
      return;
    }

    Eigen::Vector3d min = firstPointMass->getLocalPosition();
    Eigen::Vector3d max = min;
    for (std::size_t i = 1u; i < softBody->getNumPointMasses(); ++i) {
      const auto* pointMass = softBody->getPointMass(i);
      if (pointMass == nullptr) {
        continue;
      }
      const Eigen::Vector3d& vertex = pointMass->getLocalPosition();
      min = min.cwiseMin(vertex);
      max = max.cwiseMax(vertex);
    }
    descriptor.size = max - min;
    setLocalBounds(descriptor, min, max);
    return;
  }

  const auto triMesh = softMesh.getTriMesh();
  if (triMesh != nullptr && !triMesh->getVertices().empty()) {
    Eigen::Vector3d min = triMesh->getVertices().front();
    Eigen::Vector3d max = min;
    for (const Eigen::Vector3d& vertex : triMesh->getVertices()) {
      min = min.cwiseMin(vertex);
      max = max.cwiseMax(vertex);
    }
    descriptor.size = max - min;
    setLocalBounds(descriptor, min, max);
  }
}

#if DART_HAVE_OCTOMAP
void setVoxelGridData(
    GeometryDescriptor& descriptor, const dynamics::VoxelGridShape& voxelGrid)
{
  const auto tree = voxelGrid.getOctree();
  if (tree == nullptr) {
    return;
  }

  descriptor.voxelSize = tree->getResolution();
  const Eigen::Vector3d halfExtent
      = Eigen::Vector3d::Constant(descriptor.voxelSize * 0.5);
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  bool hasBounds = false;

  descriptor.voxelCenters.reserve(tree->getNumLeafNodes());
  const auto threshold = tree->getOccupancyThres();
  for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end;
       ++it) {
    if (it->getOccupancy() < threshold) {
      continue;
    }

    const auto coordinate = it.getCoordinate();
    const Eigen::Vector3d center(
        static_cast<double>(coordinate.x()),
        static_cast<double>(coordinate.y()),
        static_cast<double>(coordinate.z()));
    descriptor.voxelCenters.push_back(center);
    if (!hasBounds) {
      min = center - halfExtent;
      max = center + halfExtent;
      hasBounds = true;
    } else {
      min = min.cwiseMin(center - halfExtent);
      max = max.cwiseMax(center + halfExtent);
    }
  }

  if (hasBounds) {
    descriptor.size = max - min;
    setLocalBounds(descriptor, min, max);
  }
}
#endif

void setSymmetricLocalBounds(
    GeometryDescriptor& descriptor, const Eigen::Vector3d& halfExtents)
{
  setLocalBounds(descriptor, -halfExtents, halfExtents);
}

void setPlaneProxyBounds(
    GeometryDescriptor& descriptor,
    const Eigen::Vector3d& normal,
    double offset)
{
  static constexpr double halfExtent = 1.0;
  static constexpr double halfThickness = 0.02;

  Eigen::Vector3d unitNormal = normal;
  if (unitNormal.squaredNorm() < 1e-12) {
    unitNormal = Eigen::Vector3d::UnitZ();
  } else {
    unitNormal.normalize();
  }

  const Eigen::Vector3d seed = std::abs(unitNormal.z()) < 0.9
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d axisU = seed.cross(unitNormal).normalized();
  const Eigen::Vector3d axisV = unitNormal.cross(axisU).normalized();
  const Eigen::Vector3d center = unitNormal * offset;

  Eigen::Vector3d min = center;
  Eigen::Vector3d max = center;
  const std::array<Eigen::Vector3d, 6> points
      = {center - axisU * halfExtent - axisV * halfExtent,
         center + axisU * halfExtent - axisV * halfExtent,
         center + axisU * halfExtent + axisV * halfExtent,
         center - axisU * halfExtent + axisV * halfExtent,
         center - unitNormal * halfThickness,
         center + unitNormal * halfThickness};
  for (const Eigen::Vector3d& point : points) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }

  setLocalBounds(descriptor, min, max);
}

void setDiagnosticIfMissingGeometry(
    GeometryDescriptor& descriptor, const std::string& reason)
{
  if (descriptor.unsupportedReason.empty()) {
    descriptor.unsupportedReason = reason;
  }
}

void hashCombine(std::size_t& seed, std::size_t value)
{
  constexpr auto kHashMixConstant
      = static_cast<std::size_t>(0x9e3779b97f4a7c15ULL);
  seed ^= value + kHashMixConstant + (seed << 6) + (seed >> 2);
}

void hashVector3d(std::size_t& seed, const Eigen::Vector3d& value)
{
  for (int axis = 0; axis < 3; ++axis) {
    hashCombine(seed, std::hash<double>{}(value[axis]));
  }
}

void hashVector4d(std::size_t& seed, const Eigen::Vector4d& value)
{
  for (int axis = 0; axis < 4; ++axis) {
    hashCombine(seed, std::hash<double>{}(value[axis]));
  }
}

std::size_t computeRenderResourceVersion(
    const GeometryDescriptor& geometry,
    const MaterialDescriptor& material,
    std::size_t shapeVersion)
{
  std::size_t seed = shapeVersion;
  hashCombine(seed, static_cast<std::size_t>(geometry.kind));
  hashCombine(seed, std::hash<double>{}(material.rgba.w()));
  if (geometry.kind == ShapeKind::Mesh) {
    hashCombine(seed, geometry.meshUsesMaterialColors ? 1u : 0u);
    hashCombine(seed, static_cast<std::size_t>(geometry.meshAlphaMode));
  }
  if (geometry.kind == ShapeKind::PointCloud) {
    hashCombine(seed, std::hash<double>{}(geometry.pointSize));
    hashCombine(seed, geometry.pointCloudColors.size());
    for (const Eigen::Vector4d& color : geometry.pointCloudColors) {
      hashVector4d(seed, color);
    }
  }
  if (geometry.kind == ShapeKind::SoftMesh) {
    hashCombine(seed, geometry.triangleVertices.size());
    for (const Eigen::Vector3d& vertex : geometry.triangleVertices) {
      hashVector3d(seed, vertex);
    }
    hashCombine(seed, geometry.triangleIndices.size());
    for (const Eigen::Vector3i& triangle : geometry.triangleIndices) {
      for (int axis = 0; axis < 3; ++axis) {
        hashCombine(seed, std::hash<int>{}(triangle[axis]));
      }
    }
  }
  return seed;
}

} // namespace

std::optional<GeometryDescriptor> describeShape(const dynamics::Shape& shape)
{
  GeometryDescriptor descriptor;
  descriptor.shapeType = std::string(shape.getType());

  if (const auto* box = dynamic_cast<const dynamics::BoxShape*>(&shape)) {
    descriptor.kind = ShapeKind::Box;
    descriptor.size = box->getSize();
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* sphere = dynamic_cast<const dynamics::SphereShape*>(&shape)) {
    descriptor.kind = ShapeKind::Sphere;
    descriptor.radius = sphere->getRadius();
    descriptor.size = Eigen::Vector3d::Constant(sphere->getRadius() * 2.0);
    setSymmetricLocalBounds(
        descriptor, Eigen::Vector3d::Constant(sphere->getRadius()));
    return descriptor;
  }

  if (const auto* ellipsoid
      = dynamic_cast<const dynamics::EllipsoidShape*>(&shape)) {
    descriptor.kind = ShapeKind::Ellipsoid;
    descriptor.size = ellipsoid->getDiameters();
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* cylinder
      = dynamic_cast<const dynamics::CylinderShape*>(&shape)) {
    descriptor.kind = ShapeKind::Cylinder;
    descriptor.radius = cylinder->getRadius();
    descriptor.height = cylinder->getHeight();
    descriptor.size = Eigen::Vector3d(
        cylinder->getRadius() * 2.0,
        cylinder->getRadius() * 2.0,
        cylinder->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* capsule
      = dynamic_cast<const dynamics::CapsuleShape*>(&shape)) {
    descriptor.kind = ShapeKind::Capsule;
    descriptor.radius = capsule->getRadius();
    descriptor.height = capsule->getHeight();
    descriptor.size = Eigen::Vector3d(
        capsule->getRadius() * 2.0,
        capsule->getRadius() * 2.0,
        capsule->getHeight() + capsule->getRadius() * 2.0);
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* cone = dynamic_cast<const dynamics::ConeShape*>(&shape)) {
    descriptor.kind = ShapeKind::Cone;
    descriptor.radius = cone->getRadius();
    descriptor.height = cone->getHeight();
    descriptor.size = Eigen::Vector3d(
        cone->getRadius() * 2.0, cone->getRadius() * 2.0, cone->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* pyramid
      = dynamic_cast<const dynamics::PyramidShape*>(&shape)) {
    descriptor.kind = ShapeKind::Pyramid;
    descriptor.height = pyramid->getHeight();
    descriptor.size = Eigen::Vector3d(
        pyramid->getBaseWidth(), pyramid->getBaseDepth(), pyramid->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* multiSphere
      = dynamic_cast<const dynamics::MultiSphereConvexHullShape*>(&shape)) {
    descriptor.kind = ShapeKind::MultiSphere;
    const auto& spheres = multiSphere->getSpheres();
    descriptor.sphereCenters.reserve(spheres.size());
    descriptor.sphereRadii.reserve(spheres.size());
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    bool hasBounds = false;
    for (const auto& [radius, center] : spheres) {
      if (radius <= 0.0) {
        continue;
      }
      descriptor.sphereRadii.push_back(radius);
      descriptor.sphereCenters.push_back(center);
      descriptor.radius = std::max(descriptor.radius, radius);
      const Eigen::Vector3d extent = Eigen::Vector3d::Constant(radius);
      if (!hasBounds) {
        min = center - extent;
        max = center + extent;
        hasBounds = true;
      } else {
        min = min.cwiseMin(center - extent);
        max = max.cwiseMax(center + extent);
      }
    }
    if (hasBounds) {
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    } else {
      setDiagnosticIfMissingGeometry(
          descriptor,
          "MultiSphereConvexHullShape has no positive-radius spheres");
    }
    return descriptor;
  }

  if (const auto* lineSegments
      = dynamic_cast<const dynamics::LineSegmentShape*>(&shape)) {
    descriptor.kind = ShapeKind::LineSegments;
    descriptor.lineThickness = lineSegments->getThickness();
    const auto vertices = lineSegments->getVertices();
    descriptor.lineVertices.reserve(vertices.size());
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    bool hasBounds = false;
    for (const Eigen::Vector3d& vertex : vertices) {
      descriptor.lineVertices.push_back(vertex);
      if (!hasBounds) {
        min = vertex;
        max = vertex;
        hasBounds = true;
      } else {
        min = min.cwiseMin(vertex);
        max = max.cwiseMax(vertex);
      }
    }

    const auto connections = lineSegments->getConnections();
    descriptor.lineConnections.reserve(connections.size());
    for (const Eigen::Vector2i& connection : connections) {
      if (connection.x() < 0 || connection.y() < 0) {
        continue;
      }
      const auto first = static_cast<std::size_t>(connection.x());
      const auto second = static_cast<std::size_t>(connection.y());
      if (first >= descriptor.lineVertices.size()
          || second >= descriptor.lineVertices.size()) {
        continue;
      }
      descriptor.lineConnections.push_back(connection);
    }

    if (hasBounds) {
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    } else {
      setDiagnosticIfMissingGeometry(
          descriptor, "LineSegmentShape has no vertices");
    }
    if (descriptor.lineConnections.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "LineSegmentShape has no valid line connections");
    }
    return descriptor;
  }

  if (const auto* convexMesh
      = dynamic_cast<const dynamics::ConvexMeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::ConvexMesh;
    const auto& mesh = convexMesh->getMesh();
    if (mesh != nullptr && !mesh->getVertices().empty()) {
      setTriangleMeshData(descriptor, *mesh);
      Eigen::Vector3d min = mesh->getVertices().front();
      Eigen::Vector3d max = min;
      for (const Eigen::Vector3d& vertex : mesh->getVertices()) {
        min = min.cwiseMin(vertex);
        max = max.cwiseMax(vertex);
      }
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    }
    if (descriptor.triangleVertices.empty()
        || descriptor.triangleIndices.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "ConvexMeshShape has no triangle mesh data");
    }
    return descriptor;
  }

  if (const auto* pointCloud
      = dynamic_cast<const dynamics::PointCloudShape*>(&shape)) {
    descriptor.kind = ShapeKind::PointCloud;
    descriptor.pointSize = pointCloud->getVisualSize();
    const auto points = pointCloud->getPoints();
    descriptor.pointCloudPoints.reserve(points.size());
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    bool hasBounds = false;
    const Eigen::Vector3d extent
        = Eigen::Vector3d::Constant(std::max(0.0, descriptor.pointSize) * 0.5);
    for (const Eigen::Vector3d& point : points) {
      descriptor.pointCloudPoints.push_back(point);
      const Eigen::Vector3d pointMin = point - extent;
      const Eigen::Vector3d pointMax = point + extent;
      if (!hasBounds) {
        min = pointMin;
        max = pointMax;
        hasBounds = true;
      } else {
        min = min.cwiseMin(pointMin);
        max = max.cwiseMax(pointMax);
      }
    }

    const auto colors = pointCloud->getColors();
    if (pointCloud->getColorMode() == dynamics::PointCloudShape::BIND_OVERALL
        && !colors.empty()) {
      descriptor.pointCloudColors.push_back(colors.front());
    } else if (
        pointCloud->getColorMode() == dynamics::PointCloudShape::BIND_PER_POINT
        && colors.size() == points.size()) {
      descriptor.pointCloudColors.assign(colors.begin(), colors.end());
    }

    if (hasBounds) {
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    } else {
      setDiagnosticIfMissingGeometry(
          descriptor, "PointCloudShape has no points");
    }
    return descriptor;
  }

  if (const auto* heightmap
      = dynamic_cast<const dynamics::HeightmapShapef*>(&shape)) {
    descriptor.kind = ShapeKind::Heightmap;
    setHeightmapTriangleData(descriptor, *heightmap);
    setShapeBoundingBoxBounds(descriptor, *heightmap);
    if (descriptor.triangleVertices.empty()
        || descriptor.triangleIndices.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "HeightmapShape has no triangle mesh data");
    }
    return descriptor;
  }

  if (const auto* heightmap
      = dynamic_cast<const dynamics::HeightmapShaped*>(&shape)) {
    descriptor.kind = ShapeKind::Heightmap;
    setHeightmapTriangleData(descriptor, *heightmap);
    setShapeBoundingBoxBounds(descriptor, *heightmap);
    if (descriptor.triangleVertices.empty()
        || descriptor.triangleIndices.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "HeightmapShape has no triangle mesh data");
    }
    return descriptor;
  }

  if (const auto* softMesh
      = dynamic_cast<const dynamics::SoftMeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::SoftMesh;
    setSoftMeshTriangleData(descriptor, *softMesh);
    setSoftMeshBounds(descriptor, *softMesh);
    if (descriptor.triangleVertices.empty()
        || descriptor.triangleIndices.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "SoftMeshShape has no triangle mesh data");
    }
    return descriptor;
  }

#if DART_HAVE_OCTOMAP
  if (const auto* voxelGrid
      = dynamic_cast<const dynamics::VoxelGridShape*>(&shape)) {
    descriptor.kind = ShapeKind::VoxelGrid;
    setVoxelGridData(descriptor, *voxelGrid);
    if (descriptor.voxelCenters.empty()) {
      setDiagnosticIfMissingGeometry(
          descriptor, "VoxelGridShape has no occupied voxels");
    }
    return descriptor;
  }
#endif

  if (const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::Mesh;
    descriptor.scale = mesh->getScale();
    descriptor.meshUri = mesh->getMeshUri();
    descriptor.meshUsesMaterialColors
        = mesh->getColorMode() == dynamics::MeshShape::MATERIAL_COLOR
          && mesh->getNumMaterials() > 0u;
    descriptor.meshAlphaMode = makeMeshAlphaMode(mesh->getAlphaMode());
    descriptor.meshTextureCoordComponents = mesh->getTextureCoordComponents();
    for (const auto& material : mesh->getMaterials()) {
      descriptor.meshMaterials.push_back(makeMeshMaterialDescriptor(material));
    }
    for (const auto& range : mesh->getSubMeshRanges()) {
      descriptor.meshParts.push_back(makeMeshPartDescriptor(range));
    }
    const auto triMesh = mesh->getTriMesh();
    if (triMesh != nullptr && !triMesh->getVertices().empty()) {
      setTriangleMeshData(descriptor, *triMesh, descriptor.scale);
      const auto textureCoords = mesh->getTextureCoords();
      if (descriptor.meshTextureCoordComponents >= 2
          && textureCoords.size() == descriptor.triangleVertices.size()) {
        descriptor.meshTextureCoordinates.assign(
            textureCoords.begin(), textureCoords.end());
      }
      Eigen::Vector3d min
          = descriptor.scale.cwiseProduct(triMesh->getVertices().front());
      Eigen::Vector3d max = min;
      for (const Eigen::Vector3d& vertex : triMesh->getVertices()) {
        const Eigen::Vector3d scaled = descriptor.scale.cwiseProduct(vertex);
        min = min.cwiseMin(scaled);
        max = max.cwiseMax(scaled);
      }
      setLocalBounds(descriptor, min, max);
    }
    if (descriptor.triangleVertices.empty()
        || descriptor.triangleIndices.empty()) {
      std::string reason = "MeshShape has no triangle mesh data";
      if (!descriptor.meshUri.empty()) {
        reason += " for '" + descriptor.meshUri + "'";
      }
      setDiagnosticIfMissingGeometry(descriptor, reason);
    }
    return descriptor;
  }

  if (const auto* plane = dynamic_cast<const dynamics::PlaneShape*>(&shape)) {
    descriptor.kind = ShapeKind::Plane;
    descriptor.normal = plane->getNormal();
    descriptor.offset = plane->getOffset();
    setPlaneProxyBounds(descriptor, descriptor.normal, descriptor.offset);
    return descriptor;
  }

  descriptor.kind = ShapeKind::Unsupported;
  descriptor.unsupportedReason
      = "Shape type '" + descriptor.shapeType
        + "' is not supported by dart::gui::experimental";
  return descriptor;
}

std::optional<RenderableDescriptor> makeRenderableDescriptor(
    const dynamics::ShapeFrame& shapeFrame,
    const dynamics::VisualAspect& visualAspect,
    const std::string& skeletonName,
    const std::string& bodyName,
    const std::string& shapeNodeName,
    const dynamics::WeakConstSimpleFramePtr& simpleFrame = {})
{
  const auto shape = shapeFrame.getShape();
  if (!shape) {
    return std::nullopt;
  }

  auto geometry = describeShape(*shape);
  if (!geometry) {
    return std::nullopt;
  }

  RenderableDescriptor descriptor;
  descriptor.id = makeRenderableId(shapeFrame);
  descriptor.shapeFrame = &shapeFrame;
  descriptor.shapeNode = shapeFrame.asShapeNode();
  descriptor.shape = shape.get();
  if (descriptor.shapeNode != nullptr) {
    descriptor.skeleton = descriptor.shapeNode->getSkeleton();
  }
  descriptor.simpleFrame = simpleFrame;
  descriptor.skeletonName = skeletonName;
  descriptor.bodyName = bodyName;
  descriptor.shapeFrameName = shapeFrame.getName();
  descriptor.shapeNodeName = shapeNodeName;
  descriptor.geometry = std::move(*geometry);
  descriptor.material.rgba = visualAspect.getRGBA();
  descriptor.material.visible = !visualAspect.isHidden();
  descriptor.material.castsShadows = visualAspect.getShadowed();
  descriptor.material.receivesShadows = visualAspect.getShadowed();
  descriptor.worldTransform = shapeFrame.getWorldTransform();
  descriptor.shapeFrameVersion = shapeFrame.getVersion();
  descriptor.shapeNodeVersion = descriptor.shapeNode != nullptr
                                    ? descriptor.shapeNode->getVersion()
                                    : 0;
  descriptor.shapeVersion = shape->getVersion();
  descriptor.renderResourceVersion = computeRenderResourceVersion(
      descriptor.geometry, descriptor.material, descriptor.shapeVersion);
  return descriptor;
}

std::vector<RenderableDescriptor> extractRenderables(
    const simulation::World& world)
{
  std::vector<RenderableDescriptor> renderables;

  for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
       ++skeletonIndex) {
    const auto skeleton = world.getSkeleton(skeletonIndex);
    if (!skeleton) {
      continue;
    }

    skeleton->eachBodyNode([&](const dynamics::BodyNode* bodyNode) {
      bodyNode->eachShapeNodeWith<dynamics::VisualAspect>(
          [&](const dynamics::ShapeNode* shapeNode) {
            const auto* visualAspect = shapeNode->getVisualAspect();
            auto descriptor = makeRenderableDescriptor(
                *shapeNode,
                *visualAspect,
                skeleton->getName(),
                bodyNode->getName(),
                shapeNode->getName());
            if (descriptor) {
              renderables.push_back(std::move(*descriptor));
            }
          });
    });
  }

  for (std::size_t i = 0; i < world.getNumSimpleFrames(); ++i) {
    const auto simpleFrame = world.getSimpleFrame(i);
    if (simpleFrame == nullptr || !simpleFrame->hasVisualAspect()) {
      continue;
    }

    auto descriptor = makeRenderableDescriptor(
        *simpleFrame, *simpleFrame->getVisualAspect(), {}, {}, {}, simpleFrame);
    if (descriptor) {
      renderables.push_back(std::move(*descriptor));
    }
  }

  return renderables;
}

} // namespace dart::gui::experimental
