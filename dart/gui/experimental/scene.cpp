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

#include <dart/collision/collision_result.hpp>
#include <dart/collision/contact.hpp>

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

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <fstream>
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

  if (geometry.kind == ShapeKind::Pyramid) {
    return intersectLocalPyramid(origin, direction, geometry.size);
  }

  return intersectLocalBounds(
      origin, direction, geometry.localBoundsMin, geometry.localBoundsMax);
}

Eigen::Vector4d rgba(double red, double green, double blue, double alpha = 1.0)
{
  return {red, green, blue, alpha};
}

void appendLine(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector4d& color,
    std::string label = {})
{
  DebugLineDescriptor line;
  line.from = from;
  line.to = to;
  line.rgba = color;
  line.label = std::move(label);
  lines.push_back(std::move(line));
}

void appendArrowLines(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector4d& color,
    const std::string& label)
{
  appendLine(lines, from, to, color, label);

  const Eigen::Vector3d vector = to - from;
  const double length = vector.norm();
  if (!std::isfinite(length) || length <= 1e-12) {
    return;
  }

  const Eigen::Vector3d direction = vector / length;
  const Eigen::Vector3d seed = std::abs(direction.z()) < 0.9
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d side = direction.cross(seed).normalized();
  const double headLength = length * 0.25;
  const double headWidth = headLength * 0.45;
  const Eigen::Vector3d base = to - direction * headLength;
  appendLine(lines, to, base + side * headWidth, color, label);
  appendLine(lines, to, base - side * headWidth, color, label);
}

void appendFrameAxes(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Isometry3d& transform,
    double axisLength,
    const std::string& labelPrefix)
{
  if (axisLength <= 0.0 || !std::isfinite(axisLength)) {
    return;
  }

  const Eigen::Vector3d origin = transform.translation();
  const Eigen::Matrix3d rotation = transform.linear();
  appendLine(
      lines,
      origin,
      origin + rotation.col(0).normalized() * axisLength,
      rgba(0.9, 0.28, 0.28),
      labelPrefix.empty() ? "x" : labelPrefix + ".x");
  appendLine(
      lines,
      origin,
      origin + rotation.col(1).normalized() * axisLength,
      rgba(0.31, 0.75, 0.43),
      labelPrefix.empty() ? "y" : labelPrefix + ".y");
  appendLine(
      lines,
      origin,
      origin + rotation.col(2).normalized() * axisLength,
      rgba(0.28, 0.47, 0.92),
      labelPrefix.empty() ? "z" : labelPrefix + ".z");
}

void appendCenterOfMassMarker(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    double radius,
    const std::string& labelPrefix);

void appendAxisMarker(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    double radius,
    const Eigen::Vector4d& color,
    const std::string& labelPrefix,
    const std::string& markerName)
{
  if (radius <= 0.0 || !std::isfinite(radius) || !center.allFinite()) {
    return;
  }

  const std::string markerPrefix
      = labelPrefix.empty() ? markerName : labelPrefix + "." + markerName;
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitX() * radius,
      center + Eigen::Vector3d::UnitX() * radius,
      color,
      markerPrefix + ".x");
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitY() * radius,
      center + Eigen::Vector3d::UnitY() * radius,
      color,
      markerPrefix + ".y");
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitZ() * radius,
      center + Eigen::Vector3d::UnitZ() * radius,
      color,
      markerPrefix + ".z");
}

void appendCenterOfMassMarker(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    double radius,
    const std::string& labelPrefix)
{
  appendAxisMarker(
      lines, center, radius, rgba(0.22, 0.82, 0.86), labelPrefix, "com");
}

void appendBoxEdges(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& axes,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Vector4d& color,
    const std::string& label)
{
  if (!center.allFinite() || !axes.allFinite() || !halfExtents.allFinite()
      || (halfExtents.array() <= 0.0).any()) {
    return;
  }

  std::array<Eigen::Vector3d, 8> corners;
  std::size_t index = 0u;
  for (double x : {-1.0, 1.0}) {
    for (double y : {-1.0, 1.0}) {
      for (double z : {-1.0, 1.0}) {
        corners[index++] = center + axes.col(0) * halfExtents.x() * x
                           + axes.col(1) * halfExtents.y() * y
                           + axes.col(2) * halfExtents.z() * z;
      }
    }
  }

  const std::array<std::pair<std::size_t, std::size_t>, 12> edges
      = {std::pair<std::size_t, std::size_t>{0, 4},
         {1, 5},
         {2, 6},
         {3, 7},
         {0, 2},
         {1, 3},
         {4, 6},
         {5, 7},
         {0, 1},
         {2, 3},
         {4, 5},
         {6, 7}};
  for (const auto& [from, to] : edges) {
    appendLine(lines, corners[from], corners[to], color, label);
  }
}

void appendTransformedBoundsEdges(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& boundsMin,
    const Eigen::Vector3d& boundsMax,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector4d& color,
    const std::string& label)
{
  if (!boundsMin.allFinite() || !boundsMax.allFinite()
      || !transform.matrix().allFinite()) {
    return;
  }

  const Eigen::Vector3d min = boundsMin.cwiseMin(boundsMax);
  const Eigen::Vector3d max = boundsMin.cwiseMax(boundsMax);
  if ((max - min).cwiseAbs().maxCoeff() < 1e-12) {
    return;
  }

  std::array<Eigen::Vector3d, 8> corners
      = {Eigen::Vector3d(min.x(), min.y(), min.z()),
         Eigen::Vector3d(max.x(), min.y(), min.z()),
         Eigen::Vector3d(max.x(), max.y(), min.z()),
         Eigen::Vector3d(min.x(), max.y(), min.z()),
         Eigen::Vector3d(min.x(), min.y(), max.z()),
         Eigen::Vector3d(max.x(), min.y(), max.z()),
         Eigen::Vector3d(max.x(), max.y(), max.z()),
         Eigen::Vector3d(min.x(), max.y(), max.z())};
  for (Eigen::Vector3d& corner : corners) {
    corner = transform * corner;
  }

  const std::array<std::pair<std::size_t, std::size_t>, 12> edges
      = {std::pair<std::size_t, std::size_t>{0, 1},
         {1, 2},
         {2, 3},
         {3, 0},
         {4, 5},
         {5, 6},
         {6, 7},
         {7, 4},
         {0, 4},
         {1, 5},
         {2, 6},
         {3, 7}};

  for (const auto& [from, to] : edges) {
    appendLine(lines, corners[from], corners[to], color, label);
  }
}

bool containsRenderableId(const std::vector<RenderableId>& ids, RenderableId id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
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
    }
    return descriptor;
  }

  if (const auto* convexMesh
      = dynamic_cast<const dynamics::ConvexMeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::ConvexMesh;
    const auto& mesh = convexMesh->getMesh();
    if (mesh != nullptr && !mesh->getVertices().empty()) {
      Eigen::Vector3d min = mesh->getVertices().front();
      Eigen::Vector3d max = min;
      for (const Eigen::Vector3d& vertex : mesh->getVertices()) {
        min = min.cwiseMin(vertex);
        max = max.cwiseMax(vertex);
      }
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
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
    }
    return descriptor;
  }

  if (const auto* heightmap
      = dynamic_cast<const dynamics::HeightmapShapef*>(&shape)) {
    descriptor.kind = ShapeKind::Heightmap;
    setShapeBoundingBoxBounds(descriptor, *heightmap);
    return descriptor;
  }

  if (const auto* heightmap
      = dynamic_cast<const dynamics::HeightmapShaped*>(&shape)) {
    descriptor.kind = ShapeKind::Heightmap;
    setShapeBoundingBoxBounds(descriptor, *heightmap);
    return descriptor;
  }

  if (const auto* softMesh
      = dynamic_cast<const dynamics::SoftMeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::SoftMesh;
    setSoftMeshBounds(descriptor, *softMesh);
    return descriptor;
  }

#if DART_HAVE_OCTOMAP
  if (const auto* voxelGrid
      = dynamic_cast<const dynamics::VoxelGridShape*>(&shape)) {
    descriptor.kind = ShapeKind::VoxelGrid;
    setVoxelGridData(descriptor, *voxelGrid);
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
    descriptor.meshTextureCoordComponents = mesh->getTextureCoordComponents();
    for (const auto& material : mesh->getMaterials()) {
      descriptor.meshMaterials.push_back(makeMeshMaterialDescriptor(material));
    }
    for (const auto& range : mesh->getSubMeshRanges()) {
      descriptor.meshParts.push_back(makeMeshPartDescriptor(range));
    }
    const auto triMesh = mesh->getTriMesh();
    if (triMesh != nullptr && !triMesh->getVertices().empty()) {
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
  retainedActiveIds.reserve(activeRenderableIds.size());
  for (std::size_t i = 0; i < activeRenderableIds.size(); ++i) {
    const RenderableId id = activeRenderableIds[i];
    if (id == 0 || !containsRenderableId(desiredIds, id)
        || containsRenderableId(retainedActiveIds, id)) {
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

void normalizeRunOptions(RunOptions& options)
{
  options.width = std::max(1, options.width);
  options.height = std::max(1, options.height);
  if (options.headless && options.maxFrames < 0) {
    options.maxFrames = 1;
  }
  if (!options.screenshotPath.empty() && options.maxFrames < 0) {
    options.maxFrames = 1;
  }
}

bool shouldRequestScreenshot(
    const RunOptions& options, int renderedFrames, bool screenshotRequested)
{
  if (options.screenshotPath.empty() || screenshotRequested) {
    return false;
  }
  if (options.maxFrames < 0) {
    return true;
  }
  return renderedFrames + 1 >= options.maxFrames;
}

bool shouldStopAfterFrame(const RunOptions& options, int renderedFrames)
{
  return options.maxFrames >= 0 && renderedFrames >= options.maxFrames;
}

void togglePaused(ViewerLifecycleState& state)
{
  state.paused = !state.paused;
}

void requestSingleStep(ViewerLifecycleState& state, bool pause)
{
  if (pause) {
    state.paused = true;
  }
  state.stepOnce = true;
}

bool shouldAdvanceSimulation(const ViewerLifecycleState& state)
{
  return !state.paused || state.stepOnce;
}

void markSimulationAdvanced(ViewerLifecycleState& state)
{
  state.stepOnce = false;
}

bool shouldRequestScreenshot(
    const RunOptions& options, const ViewerLifecycleState& state)
{
  return shouldRequestScreenshot(
      options, state.renderedFrames, state.screenshotRequested);
}

void markScreenshotRequested(ViewerLifecycleState& state)
{
  state.screenshotRequested = true;
}

void markFrameRendered(ViewerLifecycleState& state)
{
  if (state.renderedFrames < std::numeric_limits<int>::max()) {
    ++state.renderedFrames;
  }
  state.skippedFrames = 0;
}

void markFrameSkipped(ViewerLifecycleState& state)
{
  if (state.skippedFrames < std::numeric_limits<int>::max()) {
    ++state.skippedFrames;
  }
}

bool shouldStopAfterFrame(
    const RunOptions& options, const ViewerLifecycleState& state)
{
  return shouldStopAfterFrame(options, state.renderedFrames);
}

bool writeRgbaPpm(
    const std::string& path,
    std::uint32_t width,
    std::uint32_t height,
    const std::vector<std::uint8_t>& rgbaPixels,
    bool originBottomLeft,
    std::string* errorMessage)
{
  const auto fail = [errorMessage](const std::string& message) {
    if (errorMessage != nullptr) {
      *errorMessage = message;
    }
    return false;
  };

  if (width == 0 || height == 0) {
    return fail("PPM dimensions must be nonzero");
  }

  constexpr std::size_t channels = 4;
  const auto maxSize = std::numeric_limits<std::size_t>::max();
  const auto imageWidth = static_cast<std::size_t>(width);
  const auto imageHeight = static_cast<std::size_t>(height);
  if (imageWidth > maxSize / imageHeight
      || imageWidth * imageHeight > maxSize / channels) {
    return fail("PPM image dimensions overflow the addressable buffer size");
  }

  const std::size_t expectedSize = imageWidth * imageHeight * channels;
  if (rgbaPixels.size() != expectedSize) {
    return fail("PPM RGBA buffer size does not match width * height * 4 bytes");
  }

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return fail("Failed to open PPM output path: " + path);
  }

  out << "P6\n" << width << " " << height << "\n255\n";
  for (std::uint32_t y = 0; y < height; ++y) {
    const std::uint32_t sourceY = originBottomLeft ? height - 1 - y : y;
    const std::size_t row = static_cast<std::size_t>(sourceY) * imageWidth * 4;
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto* pixel = &rgbaPixels[row + static_cast<std::size_t>(x) * 4];
      out.write(reinterpret_cast<const char*>(pixel), 3);
    }
  }

  if (!out) {
    return fail("Failed while writing PPM output path: " + path);
  }

  return true;
}

OrbitCameraBasis makeOrbitCameraBasis(const OrbitCamera& camera)
{
  OrbitCameraBasis basis;
  basis.eye = cameraEye(camera);
  basis.forward = camera.target - basis.eye;
  if (basis.forward.squaredNorm() < 1e-12) {
    basis.forward = -Eigen::Vector3d::UnitX();
  } else {
    basis.forward.normalize();
  }

  const Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();
  basis.right = basis.forward.cross(worldUp);
  if (basis.right.squaredNorm() < 1e-12) {
    basis.right = Eigen::Vector3d::UnitX();
  } else {
    basis.right.normalize();
  }
  basis.up = basis.right.cross(basis.forward).normalized();
  return basis;
}

Eigen::Vector3d cameraEye(const OrbitCamera& camera)
{
  return camera.target
         + Eigen::Vector3d(
             camera.distance * std::cos(camera.pitch) * std::cos(camera.yaw),
             camera.distance * std::cos(camera.pitch) * std::sin(camera.yaw),
             camera.distance * std::sin(camera.pitch));
}

void updateOrbitCamera(OrbitCamera& camera, const OrbitCameraUpdate& update)
{
  if (update.orbit) {
    camera.yaw -= update.deltaX * update.orbitScale;
    camera.pitch += update.deltaY * update.orbitScale;
  }

  const double minPitch = std::min(update.minPitch, update.maxPitch);
  const double maxPitch = std::max(update.minPitch, update.maxPitch);
  camera.pitch = std::clamp(camera.pitch, minPitch, maxPitch);

  if (update.pan) {
    const OrbitCameraBasis basis = makeOrbitCameraBasis(camera);
    const double panScale = camera.distance * update.panScale;
    camera.target -= basis.right * update.deltaX * panScale;
    camera.target += basis.up * update.deltaY * panScale;
  }

  if (update.scrollDelta != 0.0) {
    camera.distance *= std::exp(-update.scrollDelta * update.scrollScale);
  }

  const double minDistance = std::max(0.0, update.minDistance);
  const double maxDistance = std::max(minDistance, update.maxDistance);
  camera.distance = std::clamp(camera.distance, minDistance, maxDistance);
}

PickRay makePerspectivePickRay(
    const OrbitCamera& camera,
    double cursorX,
    double cursorY,
    int width,
    int height,
    double verticalFovRadians)
{
  if (!std::isfinite(verticalFovRadians) || verticalFovRadians <= 0.0) {
    verticalFovRadians = 0.7853981633974483;
  }

  const int safeWidth = std::max(1, width);
  const int safeHeight = std::max(1, height);
  const OrbitCameraBasis basis = makeOrbitCameraBasis(camera);
  const double aspect
      = static_cast<double>(safeWidth) / static_cast<double>(safeHeight);
  const double ndcX = (2.0 * cursorX / static_cast<double>(safeWidth)) - 1.0;
  const double ndcY = 1.0 - (2.0 * cursorY / static_cast<double>(safeHeight));
  const double halfHeight = std::tan(verticalFovRadians * 0.5);

  PickRay ray;
  ray.origin = basis.eye;
  ray.direction = (basis.forward + basis.right * ndcX * aspect * halfHeight
                   + basis.up * ndcY * halfHeight)
                      .normalized();
  return ray;
}

std::vector<DebugLineDescriptor> makeGridDebugLines(
    const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawGrid || options.gridHalfExtent <= 0.0
      || options.gridSpacing <= 0.0 || !std::isfinite(options.gridHalfExtent)
      || !std::isfinite(options.gridSpacing)) {
    return lines;
  }

  const auto steps = static_cast<int>(
      std::ceil(options.gridHalfExtent / options.gridSpacing));
  lines.reserve(static_cast<std::size_t>(steps * 4 + 4));
  const Eigen::Vector4d gridColor = rgba(0.46, 0.49, 0.5, 0.59);
  for (int i = -steps; i <= steps; ++i) {
    const double coordinate = static_cast<double>(i) * options.gridSpacing;
    if (std::abs(coordinate) > options.gridHalfExtent + 1e-12) {
      continue;
    }

    appendLine(
        lines,
        {-options.gridHalfExtent, coordinate, options.gridZ},
        {options.gridHalfExtent, coordinate, options.gridZ},
        gridColor,
        "grid");
    appendLine(
        lines,
        {coordinate, -options.gridHalfExtent, options.gridZ},
        {coordinate, options.gridHalfExtent, options.gridZ},
        gridColor,
        "grid");
  }

  return lines;
}

std::vector<DebugLineDescriptor> makeFrameDebugLines(
    const Eigen::Isometry3d& transform,
    double axisLength,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  lines.reserve(3);
  appendFrameAxes(lines, transform, axisLength, labelPrefix);
  return lines;
}

std::vector<DebugLineDescriptor> makeSelectionDebugLines(
    const RenderableDescriptor& renderable,
    const Eigen::Vector4d& rgba,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!renderable.material.visible || !renderable.geometry.hasLocalBounds) {
    return lines;
  }

  const std::string label
      = labelPrefix.empty() ? "selection.bounds" : labelPrefix + ".bounds";
  lines.reserve(12u);
  appendTransformedBoundsEdges(
      lines,
      renderable.geometry.localBoundsMin,
      renderable.geometry.localBoundsMax,
      renderable.worldTransform,
      rgba,
      label);

  return lines;
}

std::vector<DebugLineDescriptor> makeInertiaDebugLines(
    const dynamics::BodyNode& bodyNode,
    const DebugDrawOptions& options,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawInertiaBoxes || options.inertiaBoxScale <= 0.0
      || !std::isfinite(options.inertiaBoxScale)) {
    return lines;
  }

  const double mass = bodyNode.getMass();
  if (mass <= 0.0 || !std::isfinite(mass)) {
    return lines;
  }

  const Eigen::Matrix3d moment = bodyNode.getInertia().getMoment();
  if (!moment.allFinite()) {
    return lines;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(moment);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()
      || !solver.eigenvectors().allFinite()) {
    return lines;
  }

  const Eigen::Vector3d principalMoments = solver.eigenvalues();
  const Eigen::Vector3d dimensionsSquared
      = (Eigen::Vector3d(
             principalMoments.y() + principalMoments.z() - principalMoments.x(),
             principalMoments.x() + principalMoments.z() - principalMoments.y(),
             principalMoments.x() + principalMoments.y() - principalMoments.z())
         * (6.0 / mass));
  if (!dimensionsSquared.allFinite()
      || (dimensionsSquared.array() <= 1e-18).any()) {
    return lines;
  }

  const Eigen::Vector3d halfExtents
      = dimensionsSquared.cwiseSqrt() * 0.5 * options.inertiaBoxScale;
  const Eigen::Matrix3d worldAxes
      = bodyNode.getWorldTransform().linear() * solver.eigenvectors();
  const std::string label
      = labelPrefix.empty() ? "inertia" : labelPrefix + ".inertia";
  lines.reserve(12u);
  appendBoxEdges(
      lines,
      bodyNode.getCOM(),
      worldAxes,
      halfExtents,
      rgba(0.58, 0.44, 0.95, 0.82),
      label);
  return lines;
}

std::vector<DebugLineDescriptor> makeCollisionShapeDebugLines(
    const dynamics::ShapeNode& shapeNode,
    const DebugDrawOptions& options,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawCollisionShapeBounds
      || !std::isfinite(options.collisionBoundsPadding)
      || options.collisionBoundsPadding < 0.0) {
    return lines;
  }

  if (shapeNode.getCollisionAspect() == nullptr) {
    return lines;
  }

  const auto shape = shapeNode.getShape();
  if (!shape) {
    return lines;
  }

  auto geometry = describeShape(*shape);
  if (!geometry || !geometry->hasLocalBounds) {
    return lines;
  }

  const Eigen::Vector3d padding
      = Eigen::Vector3d::Constant(options.collisionBoundsPadding);
  const std::string label = labelPrefix.empty()
                                ? "collision.bounds"
                                : labelPrefix + ".collision_bounds";
  lines.reserve(12u);
  appendTransformedBoundsEdges(
      lines,
      geometry->localBoundsMin - padding,
      geometry->localBoundsMax + padding,
      shapeNode.getWorldTransform(),
      rgba(0.2, 0.86, 0.43, 0.72),
      label);
  return lines;
}

std::vector<DebugLineDescriptor> makeSupportPolygonDebugLines(
    const dynamics::Skeleton& skeleton,
    const DebugDrawOptions& options,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawSupportPolygons
      || !std::isfinite(options.supportPolygonElevation)) {
    return lines;
  }

  const math::SupportPolygon& polygon = skeleton.getSupportPolygon();
  if (polygon.empty()) {
    return lines;
  }

  const auto& axes = skeleton.getSupportAxes();
  if (!axes.first.allFinite() || !axes.second.allFinite()) {
    return lines;
  }

  const Eigen::Vector3d up = axes.first.cross(axes.second);
  if (!up.allFinite() || up.squaredNorm() <= 1e-18) {
    return lines;
  }

  const Eigen::Vector3d elevation
      = up.normalized() * options.supportPolygonElevation;
  const auto toWorldPoint = [&](const Eigen::Vector2d& point) {
    return axes.first * point.x() + axes.second * point.y() + elevation;
  };

  const std::size_t edgeCount
      = polygon.size() == 2u ? 1u : (polygon.size() > 2u ? polygon.size() : 0u);
  lines.reserve(edgeCount + (options.drawSupportCentroids ? 3u : 0u));

  const Eigen::Vector4d polygonColor = rgba(0.97, 0.78, 0.24, 0.86);
  const std::string polygonLabel = labelPrefix.empty()
                                       ? "support_polygon"
                                       : labelPrefix + ".support_polygon";
  if (polygon.size() >= 2u) {
    for (std::size_t i = 0; i + 1u < polygon.size(); ++i) {
      appendLine(
          lines,
          toWorldPoint(polygon[i]),
          toWorldPoint(polygon[i + 1u]),
          polygonColor,
          polygonLabel);
    }

    if (polygon.size() > 2u) {
      appendLine(
          lines,
          toWorldPoint(polygon.back()),
          toWorldPoint(polygon.front()),
          polygonColor,
          polygonLabel);
    }
  }

  if (options.drawSupportCentroids) {
    const Eigen::Vector2d& centroid = skeleton.getSupportCentroid();
    if (centroid.allFinite()) {
      appendAxisMarker(
          lines,
          toWorldPoint(centroid),
          options.supportCentroidMarkerRadius,
          rgba(0.16, 0.78, 0.58, 0.9),
          labelPrefix,
          "support_centroid");
    }
  }

  return lines;
}

std::vector<DebugLineDescriptor> extractContactDebugLines(
    const collision::CollisionResult& result, const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawContacts) {
    return lines;
  }

  const auto contacts = result.getContacts();
  lines.reserve(contacts.size() * 8u);
  const Eigen::Vector4d pointColor = rgba(1.0, 0.92, 0.38);
  const Eigen::Vector4d normalColor = rgba(1.0, 0.75, 0.25);
  const Eigen::Vector4d forceColor = rgba(0.93, 0.31, 0.67);
  for (const auto& contact : contacts) {
    const Eigen::Vector3d point = contact.point;
    appendLine(
        lines,
        point - Eigen::Vector3d::UnitX() * options.contactMarkerHalfExtent,
        point + Eigen::Vector3d::UnitX() * options.contactMarkerHalfExtent,
        pointColor,
        "contact.point");
    appendLine(
        lines,
        point - Eigen::Vector3d::UnitY() * options.contactMarkerHalfExtent,
        point + Eigen::Vector3d::UnitY() * options.contactMarkerHalfExtent,
        pointColor,
        "contact.point");

    if (options.drawContactNormals
        && contact.normal.squaredNorm()
               > collision::Contact::getNormalEpsilonSquared()) {
      appendArrowLines(
          lines,
          point,
          point + contact.normal.normalized() * options.contactNormalLength,
          normalColor,
          "contact.normal");
    }

    const double forceNorm = contact.force.norm();
    if (options.drawContactForces && forceNorm > 1e-9) {
      const double forceLength = std::clamp(
          forceNorm * options.contactForceScale,
          options.contactForceMinLength,
          options.contactForceMaxLength);
      appendArrowLines(
          lines,
          point,
          point + contact.force.normalized() * forceLength,
          forceColor,
          "contact.force");
    }
  }

  return lines;
}

std::vector<DebugLineDescriptor> extractDebugLines(
    const simulation::World& world, const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines = makeGridDebugLines(options);

  if (options.drawWorldFrame) {
    auto worldFrameLines = makeFrameDebugLines(
        Eigen::Isometry3d::Identity(), options.worldFrameAxisLength, "world");
    lines.insert(lines.end(), worldFrameLines.begin(), worldFrameLines.end());
  }

  if (options.drawBodyFrames) {
    for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
         ++skeletonIndex) {
      const auto skeleton = world.getSkeleton(skeletonIndex);
      if (!skeleton) {
        continue;
      }

      skeleton->eachBodyNode([&](const dynamics::BodyNode* bodyNode) {
        const std::string label
            = skeleton->getName() + "/" + bodyNode->getName();
        auto bodyFrameLines = makeFrameDebugLines(
            bodyNode->getWorldTransform(), options.bodyFrameAxisLength, label);
        lines.insert(lines.end(), bodyFrameLines.begin(), bodyFrameLines.end());
      });
    }
  }

  if (options.drawCentersOfMass) {
    for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
         ++skeletonIndex) {
      const auto skeleton = world.getSkeleton(skeletonIndex);
      if (!skeleton || skeleton->getMass() <= 0.0
          || !std::isfinite(skeleton->getMass())) {
        continue;
      }

      appendCenterOfMassMarker(
          lines,
          skeleton->getCOM(),
          options.centerOfMassMarkerRadius,
          skeleton->getName());
    }
  }

  if (options.drawInertiaBoxes) {
    for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
         ++skeletonIndex) {
      const auto skeleton = world.getSkeleton(skeletonIndex);
      if (!skeleton) {
        continue;
      }

      skeleton->eachBodyNode([&](const dynamics::BodyNode* bodyNode) {
        const std::string label
            = skeleton->getName() + "/" + bodyNode->getName();
        auto inertiaLines = makeInertiaDebugLines(*bodyNode, options, label);
        lines.insert(lines.end(), inertiaLines.begin(), inertiaLines.end());
      });
    }
  }

  if (options.drawCollisionShapeBounds) {
    for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
         ++skeletonIndex) {
      const auto skeleton = world.getSkeleton(skeletonIndex);
      if (!skeleton) {
        continue;
      }

      skeleton->eachBodyNode([&](const dynamics::BodyNode* bodyNode) {
        bodyNode->eachShapeNodeWith<dynamics::CollisionAspect>(
            [&](const dynamics::ShapeNode* shapeNode) {
              const std::string label = skeleton->getName() + "/"
                                        + bodyNode->getName() + "/"
                                        + shapeNode->getName();
              auto boundsLines
                  = makeCollisionShapeDebugLines(*shapeNode, options, label);
              lines.insert(lines.end(), boundsLines.begin(), boundsLines.end());
            });
      });
    }
  }

  if (options.drawSupportPolygons) {
    for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
         ++skeletonIndex) {
      const auto skeleton = world.getSkeleton(skeletonIndex);
      if (!skeleton) {
        continue;
      }

      auto supportLines = makeSupportPolygonDebugLines(
          *skeleton, options, skeleton->getName());
      lines.insert(lines.end(), supportLines.begin(), supportLines.end());
    }
  }

  auto contactLines
      = extractContactDebugLines(world.getLastCollisionResult(), options);
  lines.insert(lines.end(), contactLines.begin(), contactLines.end());

  return lines;
}

} // namespace dart::gui::experimental
