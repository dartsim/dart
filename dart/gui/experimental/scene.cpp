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

struct LocalBoundsHit
{
  double distance = 0.0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

std::optional<LocalBoundsHit> intersectLocalTriangle(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const Eigen::Vector3d edgeAB = b - a;
  const Eigen::Vector3d edgeAC = c - a;
  const Eigen::Vector3d normal = edgeAB.cross(edgeAC);
  if (!normal.allFinite() || normal.squaredNorm() <= 1e-18) {
    return std::nullopt;
  }

  const Eigen::Vector3d p = direction.cross(edgeAC);
  const double determinant = edgeAB.dot(p);
  if (std::abs(determinant) <= 1e-12 || !std::isfinite(determinant)) {
    return std::nullopt;
  }

  const double inverseDeterminant = 1.0 / determinant;
  const Eigen::Vector3d offset = origin - a;
  const double u = offset.dot(p) * inverseDeterminant;
  if (u < -1e-12 || u > 1.0 + 1e-12 || !std::isfinite(u)) {
    return std::nullopt;
  }

  const Eigen::Vector3d q = offset.cross(edgeAB);
  const double v = direction.dot(q) * inverseDeterminant;
  if (v < -1e-12 || u + v > 1.0 + 1e-12 || !std::isfinite(v)) {
    return std::nullopt;
  }

  const double distance = edgeAC.dot(q) * inverseDeterminant;
  if (distance < 0.0 || !std::isfinite(distance)) {
    return std::nullopt;
  }

  LocalBoundsHit hit;
  hit.distance = distance;
  hit.normal = normal.normalized();
  return hit;
}

std::optional<LocalBoundsHit> intersectLocalTriangleMesh(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<Eigen::Vector3i>& triangles)
{
  if (!origin.allFinite() || !direction.allFinite() || vertices.empty()
      || triangles.empty()) {
    return std::nullopt;
  }

  std::optional<LocalBoundsHit> nearest;
  for (const Eigen::Vector3i& triangle : triangles) {
    if ((triangle.array() < 0).any()) {
      continue;
    }

    const auto first = static_cast<std::size_t>(triangle.x());
    const auto second = static_cast<std::size_t>(triangle.y());
    const auto third = static_cast<std::size_t>(triangle.z());
    if (first >= vertices.size() || second >= vertices.size()
        || third >= vertices.size()) {
      continue;
    }

    const auto hit = intersectLocalTriangle(
        origin, direction, vertices[first], vertices[second], vertices[third]);
    if (hit && (!nearest || hit->distance < nearest->distance)) {
      nearest = hit;
    }
  }

  return nearest;
}

std::optional<LocalBoundsHit> intersectLocalBounds(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& boundsMin,
    const Eigen::Vector3d& boundsMax)
{
  static constexpr double epsilon = 1e-12;

  double tMin = 0.0;
  double tMax = std::numeric_limits<double>::infinity();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < epsilon) {
      if (origin[axis] < boundsMin[axis] || origin[axis] > boundsMax[axis]) {
        return std::nullopt;
      }
      continue;
    }

    double t0 = (boundsMin[axis] - origin[axis]) / direction[axis];
    double t1 = (boundsMax[axis] - origin[axis]) / direction[axis];
    double normalSign = -1.0;
    if (t0 > t1) {
      std::swap(t0, t1);
      normalSign = 1.0;
    }
    if (t0 > tMin) {
      tMin = t0;
      normal = Eigen::Vector3d::Zero();
      normal[axis] = normalSign;
    }
    tMax = std::min(tMax, t1);
    if (tMax < tMin) {
      return std::nullopt;
    }
  }

  if (normal.squaredNorm() <= epsilon) {
    normal = -direction;
  }

  LocalBoundsHit hit;
  hit.distance = tMin;
  hit.normal = normal.normalized();
  return hit;
}

std::optional<LocalBoundsHit> intersectLocalEllipsoid(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& radii)
{
  if (!origin.allFinite() || !direction.allFinite() || !radii.allFinite()
      || (radii.array() <= 0.0).any()) {
    return std::nullopt;
  }

  const Eigen::Vector3d scaledOrigin = origin.cwiseQuotient(radii);
  const Eigen::Vector3d scaledDirection = direction.cwiseQuotient(radii);
  const double a = scaledDirection.dot(scaledDirection);
  const double b = 2.0 * scaledOrigin.dot(scaledDirection);
  const double c = scaledOrigin.dot(scaledOrigin) - 1.0;
  if (a <= 1e-18 || !std::isfinite(a) || !std::isfinite(b)
      || !std::isfinite(c)) {
    return std::nullopt;
  }

  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0 || !std::isfinite(discriminant)) {
    return std::nullopt;
  }

  const double sqrtDiscriminant = std::sqrt(discriminant);
  const double t0 = (-b - sqrtDiscriminant) / (2.0 * a);
  const double t1 = (-b + sqrtDiscriminant) / (2.0 * a);
  double distance = std::numeric_limits<double>::infinity();
  if (t0 >= 0.0) {
    distance = t0;
  } else if (t1 >= 0.0) {
    distance = t1;
  } else {
    return std::nullopt;
  }

  const Eigen::Vector3d point = origin + direction * distance;
  Eigen::Vector3d normal(
      point.x() / (radii.x() * radii.x()),
      point.y() / (radii.y() * radii.y()),
      point.z() / (radii.z() * radii.z()));
  if (!normal.allFinite() || normal.squaredNorm() <= 1e-18) {
    normal = -direction;
  }

  LocalBoundsHit hit;
  hit.distance = distance;
  hit.normal = normal.normalized();
  return hit;
}

std::optional<LocalBoundsHit> intersectLocalCylinder(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    double radius,
    double height)
{
  if (!origin.allFinite() || !direction.allFinite() || !std::isfinite(radius)
      || !std::isfinite(height) || radius <= 0.0 || height <= 0.0) {
    return std::nullopt;
  }

  const double halfHeight = height * 0.5;
  std::optional<LocalBoundsHit> nearest;
  const auto considerHit = [&](double distance, const Eigen::Vector3d& normal) {
    if (distance < 0.0 || !std::isfinite(distance) || !normal.allFinite()
        || normal.squaredNorm() <= 1e-18) {
      return;
    }

    if (!nearest || distance < nearest->distance) {
      LocalBoundsHit hit;
      hit.distance = distance;
      hit.normal = normal.normalized();
      nearest = hit;
    }
  };

  const double a
      = direction.x() * direction.x() + direction.y() * direction.y();
  const double b
      = 2.0 * (origin.x() * direction.x() + origin.y() * direction.y());
  const double c
      = origin.x() * origin.x() + origin.y() * origin.y() - radius * radius;
  if (a > 1e-18 && std::isfinite(a) && std::isfinite(b) && std::isfinite(c)) {
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant >= 0.0 && std::isfinite(discriminant)) {
      const double sqrtDiscriminant = std::sqrt(discriminant);
      for (double distance :
           {(-b - sqrtDiscriminant) / (2.0 * a),
            (-b + sqrtDiscriminant) / (2.0 * a)}) {
        const Eigen::Vector3d point = origin + direction * distance;
        if (point.z() >= -halfHeight - 1e-12
            && point.z() <= halfHeight + 1e-12) {
          considerHit(distance, Eigen::Vector3d(point.x(), point.y(), 0.0));
        }
      }
    }
  }

  if (std::abs(direction.z()) > 1e-12) {
    for (double capZ : {-halfHeight, halfHeight}) {
      const double distance = (capZ - origin.z()) / direction.z();
      const Eigen::Vector3d point = origin + direction * distance;
      if (point.x() * point.x() + point.y() * point.y()
          <= radius * radius + 1e-12) {
        const Eigen::Vector3d normal = capZ < 0.0
                                           ? Eigen::Vector3d(0.0, 0.0, -1.0)
                                           : Eigen::Vector3d::UnitZ();
        considerHit(distance, normal);
      }
    }
  }

  return nearest;
}

std::optional<LocalBoundsHit> intersectLocalMultiSphere(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const std::vector<Eigen::Vector3d>& centers,
    const std::vector<double>& radii)
{
  if (!origin.allFinite() || !direction.allFinite()) {
    return std::nullopt;
  }

  std::optional<LocalBoundsHit> nearest;
  const std::size_t sphereCount = std::min(centers.size(), radii.size());
  for (std::size_t i = 0; i < sphereCount; ++i) {
    const double radius = radii[i];
    if (!centers[i].allFinite() || !std::isfinite(radius) || radius <= 0.0) {
      continue;
    }

    const Eigen::Vector3d localOrigin = origin - centers[i];
    const Eigen::Vector3d sphereRadii = Eigen::Vector3d::Constant(radius);
    const auto hit
        = intersectLocalEllipsoid(localOrigin, direction, sphereRadii);
    if (hit && (!nearest || hit->distance < nearest->distance)) {
      nearest = hit;
    }
  }

  return nearest;
}

std::optional<LocalBoundsHit> intersectLocalBoxCloud(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const std::vector<Eigen::Vector3d>& centers,
    double size)
{
  if (!origin.allFinite() || !direction.allFinite() || !std::isfinite(size)) {
    return std::nullopt;
  }

  const double halfSize = std::max(size, 1e-4) * 0.5;
  const Eigen::Vector3d extent = Eigen::Vector3d::Constant(halfSize);
  std::optional<LocalBoundsHit> nearest;
  for (const Eigen::Vector3d& center : centers) {
    if (!center.allFinite()) {
      continue;
    }

    const auto hit = intersectLocalBounds(
        origin, direction, center - extent, center + extent);
    if (hit && (!nearest || hit->distance < nearest->distance)) {
      nearest = hit;
    }
  }

  return nearest;
}

std::optional<LocalBoundsHit> intersectLocalCone(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    double radius,
    double height)
{
  if (!origin.allFinite() || !direction.allFinite() || !std::isfinite(radius)
      || !std::isfinite(height) || radius <= 0.0 || height <= 0.0) {
    return std::nullopt;
  }

  const double halfHeight = height * 0.5;
  const double slope = radius / height;
  const double slopeSquared = slope * slope;
  std::optional<LocalBoundsHit> nearest;
  const auto considerHit = [&](double distance, const Eigen::Vector3d& normal) {
    if (distance < 0.0 || !std::isfinite(distance) || !normal.allFinite()
        || normal.squaredNorm() <= 1e-18) {
      return;
    }

    if (!nearest || distance < nearest->distance) {
      LocalBoundsHit hit;
      hit.distance = distance;
      hit.normal = normal.normalized();
      nearest = hit;
    }
  };

  const double originTipOffset = halfHeight - origin.z();
  const double directionTipOffset = -direction.z();
  const double a = direction.x() * direction.x() + direction.y() * direction.y()
                   - slopeSquared * directionTipOffset * directionTipOffset;
  const double b = 2.0
                   * (origin.x() * direction.x() + origin.y() * direction.y()
                      - slopeSquared * originTipOffset * directionTipOffset);
  const double c = origin.x() * origin.x() + origin.y() * origin.y()
                   - slopeSquared * originTipOffset * originTipOffset;

  const auto considerSideDistance = [&](double distance) {
    const Eigen::Vector3d point = origin + direction * distance;
    if (point.z() < -halfHeight - 1e-12 || point.z() > halfHeight + 1e-12) {
      return;
    }

    considerHit(
        distance,
        Eigen::Vector3d(
            point.x(), point.y(), slopeSquared * (halfHeight - point.z())));
  };

  if (std::isfinite(a) && std::isfinite(b) && std::isfinite(c)) {
    if (std::abs(a) <= 1e-18) {
      if (std::abs(b) > 1e-18) {
        considerSideDistance(-c / b);
      }
    } else {
      const double discriminant = b * b - 4.0 * a * c;
      if (discriminant >= 0.0 && std::isfinite(discriminant)) {
        const double sqrtDiscriminant = std::sqrt(discriminant);
        considerSideDistance((-b - sqrtDiscriminant) / (2.0 * a));
        considerSideDistance((-b + sqrtDiscriminant) / (2.0 * a));
      }
    }
  }

  if (std::abs(direction.z()) > 1e-12) {
    const double distance = (-halfHeight - origin.z()) / direction.z();
    const Eigen::Vector3d point = origin + direction * distance;
    if (point.x() * point.x() + point.y() * point.y()
        <= radius * radius + 1e-12) {
      considerHit(distance, -Eigen::Vector3d::UnitZ());
    }
  }

  return nearest;
}

std::optional<LocalBoundsHit> intersectLocalPyramid(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& size)
{
  if (!origin.allFinite() || !direction.allFinite() || !size.allFinite()
      || (size.array() <= 0.0).any()) {
    return std::nullopt;
  }

  const double halfWidth = size.x() * 0.5;
  const double halfDepth = size.y() * 0.5;
  const double halfHeight = size.z() * 0.5;
  const std::array<Eigen::Vector3d, 5> points
      = {Eigen::Vector3d(0.0, 0.0, halfHeight),
         Eigen::Vector3d(-halfWidth, -halfDepth, -halfHeight),
         Eigen::Vector3d(halfWidth, -halfDepth, -halfHeight),
         Eigen::Vector3d(halfWidth, halfDepth, -halfHeight),
         Eigen::Vector3d(-halfWidth, halfDepth, -halfHeight)};
  const std::array<std::array<std::size_t, 3>, 6> faces = {{
      {0u, 1u, 2u},
      {0u, 2u, 3u},
      {0u, 3u, 4u},
      {0u, 4u, 1u},
      {1u, 4u, 3u},
      {1u, 3u, 2u},
  }};

  std::optional<LocalBoundsHit> nearest;
  for (const auto& face : faces) {
    const auto hit = intersectLocalTriangle(
        origin, direction, points[face[0]], points[face[1]], points[face[2]]);
    if (hit && (!nearest || hit->distance < nearest->distance)) {
      nearest = hit;
    }
  }

  return nearest;
}

std::optional<LocalBoundsHit> intersectLocalPlaneProxy(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& normal,
    double offset)
{
  static constexpr double halfExtent = 1.0;

  if (!origin.allFinite() || !direction.allFinite() || !normal.allFinite()
      || !std::isfinite(offset)) {
    return std::nullopt;
  }

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
  const std::array<Eigen::Vector3d, 4> points
      = {center - axisU * halfExtent - axisV * halfExtent,
         center + axisU * halfExtent - axisV * halfExtent,
         center + axisU * halfExtent + axisV * halfExtent,
         center - axisU * halfExtent + axisV * halfExtent};

  std::optional<LocalBoundsHit> nearest;
  for (const auto& face :
       {std::array<std::size_t, 3>{0u, 1u, 2u},
        std::array<std::size_t, 3>{0u, 2u, 3u}}) {
    const auto hit = intersectLocalTriangle(
        origin, direction, points[face[0]], points[face[1]], points[face[2]]);
    if (hit && (!nearest || hit->distance < nearest->distance)) {
      nearest = hit;
    }
  }

  return nearest;
}

std::optional<LocalBoundsHit> intersectLocalCapsule(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    double radius,
    double height)
{
  if (!origin.allFinite() || !direction.allFinite() || !std::isfinite(radius)
      || !std::isfinite(height) || radius <= 0.0 || height < 0.0) {
    return std::nullopt;
  }

  const double halfHeight = height * 0.5;
  std::optional<LocalBoundsHit> nearest;
  const auto considerHit = [&](double distance, const Eigen::Vector3d& normal) {
    if (distance < 0.0 || !std::isfinite(distance) || !normal.allFinite()
        || normal.squaredNorm() <= 1e-18) {
      return;
    }

    if (!nearest || distance < nearest->distance) {
      LocalBoundsHit hit;
      hit.distance = distance;
      hit.normal = normal.normalized();
      nearest = hit;
    }
  };

  const double sideA
      = direction.x() * direction.x() + direction.y() * direction.y();
  const double sideB
      = 2.0 * (origin.x() * direction.x() + origin.y() * direction.y());
  const double sideC
      = origin.x() * origin.x() + origin.y() * origin.y() - radius * radius;
  if (sideA > 1e-18 && std::isfinite(sideA) && std::isfinite(sideB)
      && std::isfinite(sideC)) {
    const double discriminant = sideB * sideB - 4.0 * sideA * sideC;
    if (discriminant >= 0.0 && std::isfinite(discriminant)) {
      const double sqrtDiscriminant = std::sqrt(discriminant);
      for (double distance :
           {(-sideB - sqrtDiscriminant) / (2.0 * sideA),
            (-sideB + sqrtDiscriminant) / (2.0 * sideA)}) {
        const Eigen::Vector3d point = origin + direction * distance;
        if (point.z() >= -halfHeight - 1e-12
            && point.z() <= halfHeight + 1e-12) {
          considerHit(distance, Eigen::Vector3d(point.x(), point.y(), 0.0));
        }
      }
    }
  }

  const auto considerCap = [&](const Eigen::Vector3d& center, bool top) {
    const Eigen::Vector3d offset = origin - center;
    const double a = direction.dot(direction);
    const double b = 2.0 * offset.dot(direction);
    const double c = offset.dot(offset) - radius * radius;
    if (a <= 1e-18 || !std::isfinite(a) || !std::isfinite(b)
        || !std::isfinite(c)) {
      return;
    }

    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0 || !std::isfinite(discriminant)) {
      return;
    }

    const double sqrtDiscriminant = std::sqrt(discriminant);
    for (double distance :
         {(-b - sqrtDiscriminant) / (2.0 * a),
          (-b + sqrtDiscriminant) / (2.0 * a)}) {
      const Eigen::Vector3d point = origin + direction * distance;
      if ((top && point.z() < halfHeight - 1e-12)
          || (!top && point.z() > -halfHeight + 1e-12)) {
        continue;
      }
      considerHit(distance, point - center);
    }
  };

  considerCap(Eigen::Vector3d(0.0, 0.0, halfHeight), true);
  considerCap(Eigen::Vector3d(0.0, 0.0, -halfHeight), false);
  return nearest;
}

std::optional<LocalBoundsHit> intersectLocalGeometry(
    const GeometryDescriptor& geometry,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction)
{
  if (geometry.kind == ShapeKind::Sphere) {
    const Eigen::Vector3d radii = Eigen::Vector3d::Constant(geometry.radius);
    return intersectLocalEllipsoid(origin, direction, radii);
  }

  if (geometry.kind == ShapeKind::Ellipsoid) {
    return intersectLocalEllipsoid(origin, direction, geometry.size * 0.5);
  }

  if (geometry.kind == ShapeKind::Cylinder) {
    return intersectLocalCylinder(
        origin, direction, geometry.radius, geometry.height);
  }

  if (geometry.kind == ShapeKind::Capsule) {
    return intersectLocalCapsule(
        origin, direction, geometry.radius, geometry.height);
  }

  if (geometry.kind == ShapeKind::Cone) {
    return intersectLocalCone(
        origin, direction, geometry.radius, geometry.height);
  }

  if (geometry.kind == ShapeKind::MultiSphere) {
    return intersectLocalMultiSphere(
        origin, direction, geometry.sphereCenters, geometry.sphereRadii);
  }

  if (geometry.kind == ShapeKind::Pyramid) {
    return intersectLocalPyramid(origin, direction, geometry.size);
  }

  if (geometry.kind == ShapeKind::Plane) {
    return intersectLocalPlaneProxy(
        origin, direction, geometry.normal, geometry.offset);
  }

  if (geometry.kind == ShapeKind::ConvexMesh
      || geometry.kind == ShapeKind::Heightmap
      || geometry.kind == ShapeKind::SoftMesh
      || geometry.kind == ShapeKind::Mesh) {
    if (!geometry.triangleVertices.empty()
        && !geometry.triangleIndices.empty()) {
      return intersectLocalTriangleMesh(
          origin,
          direction,
          geometry.triangleVertices,
          geometry.triangleIndices);
    }
  }

  if (geometry.kind == ShapeKind::PointCloud) {
    return intersectLocalBoxCloud(
        origin, direction, geometry.pointCloudPoints, geometry.pointSize);
  }

  if (geometry.kind == ShapeKind::VoxelGrid) {
    return intersectLocalBoxCloud(
        origin, direction, geometry.voxelCenters, geometry.voxelSize);
  }

  return intersectLocalBounds(
      origin, direction, geometry.localBoundsMin, geometry.localBoundsMax);
}

bool containsRenderableId(const std::vector<RenderableId>& ids, RenderableId id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
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

RenderableSetUpdatePlan planRenderableSetUpdate(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<RenderableId>& activeRenderableIds)
{
  std::vector<ActiveRenderableState> activeStates;
  activeStates.reserve(activeRenderableIds.size());
  for (const RenderableId id : activeRenderableIds) {
    ActiveRenderableState state;
    state.id = id;
    const auto descriptor = std::find_if(
        descriptors.begin(),
        descriptors.end(),
        [id](const RenderableDescriptor& candidate) {
          return candidate.material.visible && candidate.id == id;
        });
    if (descriptor != descriptors.end()) {
      state.shapeVersion = descriptor->shapeVersion;
      state.renderResourceVersion = descriptor->renderResourceVersion;
    }
    activeStates.push_back(state);
  }

  return planRenderableSetUpdate(descriptors, activeStates);
}

RenderableSetUpdatePlan planRenderableSetUpdate(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<ActiveRenderableState>& activeRenderableStates)
{
  RenderableSetUpdatePlan plan;

  std::vector<RenderableId> desiredIds;
  desiredIds.reserve(descriptors.size());
  for (const RenderableDescriptor& descriptor : descriptors) {
    if (!descriptor.material.visible || descriptor.id == 0
        || containsRenderableId(desiredIds, descriptor.id)) {
      continue;
    }
    desiredIds.push_back(descriptor.id);
  }

  std::vector<RenderableId> retainedActiveIds;
  retainedActiveIds.reserve(activeRenderableStates.size());
  for (std::size_t i = 0; i < activeRenderableStates.size(); ++i) {
    const ActiveRenderableState& active = activeRenderableStates[i];
    const RenderableId id = active.id;
    const auto descriptor = std::find_if(
        descriptors.begin(),
        descriptors.end(),
        [id](const RenderableDescriptor& candidate) {
          return candidate.material.visible && candidate.id == id;
        });
    const bool resourceChanged
        = descriptor != descriptors.end()
          && (active.renderResourceVersion != 0u
                  ? active.renderResourceVersion
                        != descriptor->renderResourceVersion
                  : active.shapeVersion != descriptor->shapeVersion);
    if (id == 0 || !containsRenderableId(desiredIds, id)
        || containsRenderableId(retainedActiveIds, id) || resourceChanged) {
      plan.activeRenderableIndicesToRemove.push_back(i);
      continue;
    }
    retainedActiveIds.push_back(id);
  }

  std::vector<RenderableId> idsToAdd;
  idsToAdd.reserve(descriptors.size());
  for (std::size_t i = 0; i < descriptors.size(); ++i) {
    const RenderableDescriptor& descriptor = descriptors[i];
    if (!descriptor.material.visible || descriptor.id == 0
        || containsRenderableId(retainedActiveIds, descriptor.id)
        || containsRenderableId(idsToAdd, descriptor.id)) {
      continue;
    }
    plan.descriptorIndicesToAdd.push_back(i);
    idsToAdd.push_back(descriptor.id);
  }

  return plan;
}

std::optional<double> intersectRenderable(
    const RenderableDescriptor& renderable, const PickRay& ray)
{
  if (!renderable.material.visible || !renderable.geometry.hasLocalBounds) {
    return std::nullopt;
  }

  const double directionNorm = ray.direction.norm();
  if (!std::isfinite(directionNorm) || directionNorm < 1e-12) {
    return std::nullopt;
  }

  const Eigen::Vector3d direction = ray.direction / directionNorm;
  const Eigen::Isometry3d localFromWorld = renderable.worldTransform.inverse();
  const Eigen::Vector3d localOrigin = localFromWorld * ray.origin;
  const Eigen::Vector3d localDirection
      = (localFromWorld.linear() * direction).normalized();

  const auto hit = intersectLocalGeometry(
      renderable.geometry, localOrigin, localDirection);
  if (!hit) {
    return std::nullopt;
  }
  return hit->distance;
}

std::optional<PickHit> pickNearestRenderable(
    const std::vector<RenderableDescriptor>& renderables,
    const PickRay& ray,
    double maxDistance)
{
  std::optional<PickHit> nearest;
  for (std::size_t i = 0; i < renderables.size(); ++i) {
    const auto& renderable = renderables[i];
    if (!renderable.material.visible || !renderable.geometry.hasLocalBounds) {
      continue;
    }

    const double directionNorm = ray.direction.norm();
    if (!std::isfinite(directionNorm) || directionNorm < 1e-12) {
      continue;
    }

    const Eigen::Vector3d direction = ray.direction / directionNorm;
    const Eigen::Isometry3d localFromWorld
        = renderable.worldTransform.inverse();
    const Eigen::Vector3d localOrigin = localFromWorld * ray.origin;
    const Eigen::Vector3d localDirection
        = (localFromWorld.linear() * direction).normalized();

    const auto localHit = intersectLocalGeometry(
        renderable.geometry, localOrigin, localDirection);
    if (!localHit || localHit->distance > maxDistance) {
      continue;
    }
    if (!nearest || localHit->distance < nearest->distance) {
      PickHit hit;
      hit.id = renderable.id;
      hit.renderableIndex = i;
      hit.distance = localHit->distance;
      hit.point = ray.origin + direction * localHit->distance;
      hit.normal = (renderable.worldTransform.linear() * localHit->normal)
                       .normalized();
      nearest = hit;
    }
  }

  return nearest;
}

std::optional<Eigen::Vector3d> intersectPlane(
    const PickRay& ray,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal)
{
  if (!ray.origin.allFinite() || !ray.direction.allFinite()
      || !planePoint.allFinite() || !planeNormal.allFinite()) {
    return std::nullopt;
  }

  const double directionNorm = ray.direction.norm();
  const double normalNorm = planeNormal.norm();
  if (directionNorm < 1e-12 || normalNorm < 1e-12) {
    return std::nullopt;
  }

  const Eigen::Vector3d direction = ray.direction / directionNorm;
  const Eigen::Vector3d normal = planeNormal / normalNorm;
  const double denom = direction.dot(normal);
  if (std::abs(denom) < 1e-12) {
    return std::nullopt;
  }

  const double distance = (planePoint - ray.origin).dot(normal) / denom;
  if (distance < 0.0) {
    return std::nullopt;
  }

  return ray.origin + direction * distance;
}

std::optional<Eigen::Vector3d> computePlaneDragTranslation(
    const PickRay& previousRay,
    const PickRay& currentRay,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal)
{
  const auto previousPoint
      = intersectPlane(previousRay, planePoint, planeNormal);
  const auto currentPoint = intersectPlane(currentRay, planePoint, planeNormal);
  if (!previousPoint || !currentPoint) {
    return std::nullopt;
  }
  return *currentPoint - *previousPoint;
}

bool translateFreeJointRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation)
{
  if (!worldTranslation.allFinite()) {
    return false;
  }

  const auto skeleton = renderable.skeleton.lock();
  if (!skeleton || renderable.bodyName.empty()
      || renderable.shapeNodeName.empty()) {
    return false;
  }

  const auto* bodyNode = skeleton->getBodyNode(renderable.bodyName);
  if (bodyNode == nullptr) {
    return false;
  }
  const dynamics::ShapeNode* shapeNode = nullptr;
  bodyNode->eachShapeNode([&](const dynamics::ShapeNode* candidate) {
    if (candidate->getName() == renderable.shapeNodeName) {
      shapeNode = candidate;
      return false;
    }
    return true;
  });
  if (shapeNode == nullptr || shapeNode != renderable.shapeNode
      || shapeNode->getVersion() != renderable.shapeNodeVersion) {
    return false;
  }

  auto* mutableBodyNode = const_cast<dynamics::BodyNode*>(bodyNode);
  if (dynamic_cast<dynamics::FreeJoint*>(mutableBodyNode->getParentJoint())
      == nullptr) {
    return false;
  }

  Eigen::Isometry3d transform = mutableBodyNode->getWorldTransform();
  transform.translation() += worldTranslation;
  dynamics::FreeJoint::setTransformOf(mutableBodyNode, transform);
  return true;
}

bool translateSimpleFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation)
{
  if (!worldTranslation.allFinite()) {
    return false;
  }

  const auto simpleFrame = renderable.simpleFrame.lock();
  if (simpleFrame == nullptr || simpleFrame.get() != renderable.shapeFrame
      || simpleFrame->getVersion() != renderable.shapeFrameVersion) {
    return false;
  }

  auto* mutableSimpleFrame
      = const_cast<dynamics::SimpleFrame*>(simpleFrame.get());
  Eigen::Isometry3d transform = mutableSimpleFrame->getWorldTransform();
  transform.translation() += worldTranslation;
  mutableSimpleFrame->setTransform(transform, dynamics::Frame::World());
  return true;
}

bool translateFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation)
{
  return translateFreeJointRenderable(renderable, worldTranslation)
         || translateSimpleFrameRenderable(renderable, worldTranslation);
}

} // namespace dart::gui::experimental
