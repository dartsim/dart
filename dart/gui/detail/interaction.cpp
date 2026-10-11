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

#include <dart/gui/detail/debug.hpp>
#include <dart/gui/detail/geometry.hpp>
#include <dart/gui/detail/interaction.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::gui {
namespace {

struct LocalBoundsHit
{
  double distance = 0.0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d barycentric = Eigen::Vector3d::Zero();
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
  hit.barycentric = {1 - u - v, u, v};
  return hit;
}

std::optional<LocalBoundsHit> intersectLocalTriangleMesh(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<Eigen::Vector3i>& triangles,
    const std::vector<Eigen::Vector3d>& normals = {})
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

    auto hit = intersectLocalTriangle(
        origin, direction, vertices[first], vertices[second], vertices[third]);
    if (hit && (!nearest || hit->distance < nearest->distance)) {
      if (normals.size() == vertices.size()) {
        const Eigen::Vector3d normal = normals[first] * hit->barycentric.x()
                                       + normals[second] * hit->barycentric.y()
                                       + normals[third] * hit->barycentric.z();
        if (normal.allFinite() && normal.squaredNorm() > 1e-18)
          hit->normal = normal.normalized();
      }
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
  constexpr double epsilon = 1e-12;
  if (!origin.allFinite() || !direction.allFinite() || !boundsMin.allFinite()
      || !boundsMax.allFinite()
      || (boundsMin.array() > boundsMax.array()).any())
    return std::nullopt;
  double entry = -std::numeric_limits<double>::infinity();
  double exit = std::numeric_limits<double>::infinity();
  Eigen::Vector3d entryNormal = Eigen::Vector3d::Zero(),
                  exitNormal = entryNormal;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < epsilon) {
      if (origin[axis] < boundsMin[axis] || origin[axis] > boundsMax[axis])
        return std::nullopt;
      continue;
    }
    double a = (boundsMin[axis] - origin[axis]) / direction[axis];
    double b = (boundsMax[axis] - origin[axis]) / direction[axis];
    double sign = -1;
    if (a > b) {
      std::swap(a, b);
      sign = 1;
    }
    if (a > entry) {
      entry = a;
      entryNormal = Eigen::Vector3d::Zero();
      entryNormal[axis] = sign;
    }
    if (b < exit) {
      exit = b;
      exitNormal = Eigen::Vector3d::Zero();
      exitNormal[axis] = -sign;
    }
    if (exit < entry)
      return std::nullopt;
  }
  const double distance = entry >= 0 ? entry : exit;
  if (!std::isfinite(distance) || distance < 0)
    return std::nullopt;
  return LocalBoundsHit{distance, entry >= 0 ? entryNormal : exitNormal};
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

std::optional<LocalBoundsHit> intersectLocalBoxCloud(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const std::vector<Eigen::Vector3d>& centers,
    double size,
    const std::vector<double>& sizes = {})
{
  if (!origin.allFinite() || !direction.allFinite() || !std::isfinite(size)) {
    return std::nullopt;
  }

  std::optional<LocalBoundsHit> nearest;
  for (std::size_t i = 0; i < centers.size(); ++i) {
    const auto& center = centers[i];
    const double leafSize = i < sizes.size() ? sizes[i] : size;
    if (!std::isfinite(leafSize) || leafSize <= 0)
      continue;
    const Eigen::Vector3d extent = Eigen::Vector3d::Constant(leafSize * 0.5);
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

  if (geometry.kind == ShapeKind::Pyramid) {
    return intersectLocalPyramid(origin, direction, geometry.size);
  }

  if (geometry.kind == ShapeKind::Plane) {
    return intersectLocalPlaneProxy(
        origin, direction, geometry.normal, geometry.offset);
  }

  if (geometry.kind == ShapeKind::MultiSphere
      || geometry.kind == ShapeKind::ConvexMesh
      || geometry.kind == ShapeKind::Heightmap
      || geometry.kind == ShapeKind::SoftMesh
      || geometry.kind == ShapeKind::Mesh) {
    if (!geometry.triangleVertices.empty()
        && !geometry.triangleIndices.empty()) {
      return intersectLocalTriangleMesh(
          origin,
          direction,
          geometry.triangleVertices,
          geometry.triangleIndices,
          geometry.triangleNormals);
    }
    return std::nullopt;
  }

  if (geometry.kind == ShapeKind::PointCloud) {
    return geometry.pointShapeType == PointShapeType::Box
               ? intersectLocalBoxCloud(
                   origin,
                   direction,
                   geometry.pointCloudPoints,
                   std::max(geometry.pointSize, 1e-4))
               : std::nullopt;
  }

  if (geometry.kind == ShapeKind::VoxelGrid) {
    return intersectLocalBoxCloud(
        origin,
        direction,
        geometry.voxelCenters,
        geometry.voxelSize,
        geometry.voxelSizes);
  }

  if (geometry.kind == ShapeKind::Box)
    return intersectLocalBounds(
        origin, direction, geometry.localBoundsMin, geometry.localBoundsMax);
  return std::nullopt;
}

} // namespace

std::optional<PickHit> intersectRenderableHit(
    const RenderableDescriptor& renderable,
    const PickRay& ray,
    const VisualPickOptions& options)
{
  if (!renderable.material.visible || !renderable.geometry.hasLocalBounds
      || !ray.origin.allFinite() || !ray.direction.allFinite()
      || !renderable.worldTransform.matrix().allFinite())
    return std::nullopt;
  const double norm = ray.direction.norm();
  if (norm < 1e-12 || !std::isfinite(norm))
    return std::nullopt;
  const double determinant = renderable.worldTransform.linear().determinant();
  if (!std::isfinite(determinant) || std::abs(determinant) < 1e-15)
    return std::nullopt;
  const Eigen::Vector3d direction = ray.direction / norm;
  const Eigen::Affine3d localFromWorld(
      renderable.worldTransform.matrix().inverse());
  const Eigen::Vector3d origin = localFromWorld * ray.origin;
  const Eigen::Vector3d localDirection = localFromWorld.linear() * direction;
  const auto& geometry = renderable.geometry;
  std::optional<LocalBoundsHit> localHit
      = intersectLocalGeometry(geometry, origin, localDirection);
  const bool meshPixels = geometry.kind == ShapeKind::Mesh
                          && (!geometry.meshPointIndices.empty()
                              || !geometry.meshLineIndices.empty());
  const bool cameraDependent
      = geometry.kind == ShapeKind::LineSegments || meshPixels
        || (geometry.kind == ShapeKind::PointCloud
            && geometry.pointShapeType != PointShapeType::Box);
  const bool validCamera
      = options.width > 0 && options.height > 0 && options.eye.allFinite()
        && options.center.allFinite() && options.up.allFinite()
        && (options.center - options.eye).squaredNorm() >= 1e-18
        && (options.center - options.eye).cross(options.up).squaredNorm()
               >= 1e-18
        && std::isfinite(options.fovyDegrees) && options.fovyDegrees > 0
        && options.fovyDegrees < 180;
  if (cameraDependent && validCamera) {
    const auto considerTriangle = [&](const Eigen::Vector3d& a,
                                      const Eigen::Vector3d& b,
                                      const Eigen::Vector3d& c) {
      const auto hit = intersectLocalTriangle(origin, localDirection, a, b, c);
      if (hit && (!localHit || hit->distance < localHit->distance))
        localHit = hit;
    };
    if (meshPixels) {
      MeshPartDescriptor part;
      part.pointCount = geometry.meshPointIndices.size();
      part.lineCount = geometry.meshLineIndices.size();
      const auto mesh = makeMeshPointLineGeometry(
          renderable,
          part,
          options.eye,
          options.center,
          options.up,
          options.fovyDegrees,
          options.height);
      for (const auto& triangle : mesh.triangles)
        considerTriangle(
            mesh.vertices[triangle.a].position.cast<double>(),
            mesh.vertices[triangle.b].position.cast<double>(),
            mesh.vertices[triangle.c].position.cast<double>());
    } else {
      const auto triangles = geometry.kind == ShapeKind::LineSegments
                                 ? makeLineSegmentVisualTriangles(
                                     renderable,
                                     options.eye,
                                     options.center,
                                     options.fovyDegrees,
                                     options.height)
                                 : makePointCloudVisualTriangles(
                                     renderable,
                                     options.eye,
                                     options.center,
                                     options.up,
                                     options.fovyDegrees,
                                     options.height);
      for (const auto& triangle : triangles)
        considerTriangle(triangle.a, triangle.b, triangle.c);
    }
  }
  if (!localHit)
    return std::nullopt;
  PickHit hit;
  hit.id = renderable.id;
  hit.distance = localHit->distance;
  hit.point = ray.origin + direction * hit.distance;
  hit.normal
      = (localFromWorld.linear().transpose() * localHit->normal).normalized();
  if (!hit.point.allFinite() || !hit.normal.allFinite())
    return std::nullopt;
  return hit;
}
std::optional<double> intersectRenderable(
    const RenderableDescriptor& renderable,
    const PickRay& ray,
    const VisualPickOptions& options)
{
  const auto hit = intersectRenderableHit(renderable, ray, options);
  return hit ? std::optional<double>{hit->distance} : std::nullopt;
}
std::optional<PickHit> pickNearestRenderable(
    const std::vector<RenderableDescriptor>& renderables,
    const PickRay& ray,
    double maxDistance,
    const VisualPickOptions& options)
{
  if (std::isnan(maxDistance) || maxDistance < 0)
    return std::nullopt;
  std::optional<PickHit> nearest;
  for (std::size_t index = 0; index < renderables.size(); ++index) {
    auto hit = intersectRenderableHit(renderables[index], ray, options);
    if (hit && hit->distance <= maxDistance
        && (!nearest || hit->distance < nearest->distance)) {
      hit->renderableIndex = index;
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
  if (!std::isfinite(distance) || distance < 0.0) {
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

std::optional<Eigen::Vector3d> computeAxisDragTranslation(
    const PickRay& previousRay,
    const PickRay& currentRay,
    const Eigen::Vector3d& axisPoint,
    const Eigen::Vector3d& axisDirection)
{
  if (!axisPoint.allFinite() || !axisDirection.allFinite()) {
    return std::nullopt;
  }

  const double axisNorm = axisDirection.norm();
  if (!std::isfinite(axisNorm) || axisNorm < 1e-12) {
    return std::nullopt;
  }

  const Eigen::Vector3d axis = axisDirection / axisNorm;
  const auto closestAxisParameter = [&](const PickRay& ray) {
    if (!ray.origin.allFinite() || !ray.direction.allFinite()) {
      return std::optional<double>{};
    }

    const double rayNorm = ray.direction.norm();
    if (!std::isfinite(rayNorm) || rayNorm < 1e-12) {
      return std::optional<double>{};
    }

    const Eigen::Vector3d direction = ray.direction / rayNorm;
    const double axisDotRay = axis.dot(direction);
    const double denom = 1.0 - axisDotRay * axisDotRay;
    if (!std::isfinite(denom) || std::abs(denom) < 1e-12) {
      return std::optional<double>{};
    }

    const Eigen::Vector3d offset = axisPoint - ray.origin;
    const double axisOffset = axis.dot(offset);
    const double rayOffset = direction.dot(offset);
    const double axisParameter = (axisDotRay * rayOffset - axisOffset) / denom;
    const double rayParameter = axisDotRay * axisParameter + rayOffset;
    if (!std::isfinite(axisParameter) || !std::isfinite(rayParameter)
        || rayParameter < 0.0) {
      return std::optional<double>{};
    }

    return std::optional<double>{axisParameter};
  };

  const auto previousParameter = closestAxisParameter(previousRay);
  const auto currentParameter = closestAxisParameter(currentRay);
  if (!previousParameter || !currentParameter) {
    return std::nullopt;
  }

  return axis * (*currentParameter - *previousParameter);
}

} // namespace dart::gui
