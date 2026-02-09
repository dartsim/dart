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

#include "dart/collision/experimental_backend/shape_adapter.hpp"
#include "dart/common/logging.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/capsule_shape.hpp"
#include "dart/dynamics/cone_shape.hpp"
#include "dart/dynamics/convex_mesh_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/heightmap_shape.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/multi_sphere_convex_hull_shape.hpp"
#include "dart/dynamics/plane_shape.hpp"
#include "dart/dynamics/pyramid_shape.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/math/tri_mesh.hpp"

#include <numbers>
#include <vector>

#include <cmath>

namespace dart {
namespace collision {

std::unique_ptr<experimental::Shape> adaptShape(const dynamics::ShapePtr& shape)
{
  return adaptShape(dynamics::ConstShapePtr(shape));
}

std::unique_ptr<experimental::Shape> adaptShape(
    const dynamics::ConstShapePtr& shape)
{
  if (!shape) {
    return nullptr;
  }

  const auto& shapeType = shape->getType();

  if (shapeType == dynamics::SphereShape::getStaticType()) {
    const auto& sphere
        = std::static_pointer_cast<const dynamics::SphereShape>(shape);
    return std::make_unique<experimental::SphereShape>(sphere->getRadius());
  }

  if (shapeType == dynamics::BoxShape::getStaticType()) {
    const auto& box = std::static_pointer_cast<const dynamics::BoxShape>(shape);
    return std::make_unique<experimental::BoxShape>(box->getSize() * 0.5);
  }

  if (shapeType == dynamics::CapsuleShape::getStaticType()) {
    const auto& capsule
        = std::static_pointer_cast<const dynamics::CapsuleShape>(shape);
    return std::make_unique<experimental::CapsuleShape>(
        capsule->getRadius(), capsule->getHeight());
  }

  if (shapeType == dynamics::CylinderShape::getStaticType()) {
    const auto& cylinder
        = std::static_pointer_cast<const dynamics::CylinderShape>(shape);
    return std::make_unique<experimental::CylinderShape>(
        cylinder->getRadius(), cylinder->getHeight());
  }

  if (shapeType == dynamics::ConeShape::getStaticType()) {
    const auto& cone
        = std::static_pointer_cast<const dynamics::ConeShape>(shape);
    const double radius = cone->getRadius();
    const double height = cone->getHeight();

    std::vector<Eigen::Vector3d> vertices;
    constexpr int numBasePoints = 32;
    vertices.reserve(numBasePoints + 1);
    vertices.emplace_back(0.0, 0.0, height * 0.5);

    constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
    for (int i = 0; i < numBasePoints; ++i) {
      const double angle = kTwoPi * static_cast<double>(i) / numBasePoints;
      vertices.emplace_back(
          radius * std::cos(angle), radius * std::sin(angle), -height * 0.5);
    }

    return std::make_unique<experimental::ConvexShape>(std::move(vertices));
  }

  if (shapeType == dynamics::PlaneShape::getStaticType()) {
    const auto& plane
        = std::static_pointer_cast<const dynamics::PlaneShape>(shape);
    return std::make_unique<experimental::PlaneShape>(
        plane->getNormal(), plane->getOffset());
  }

  if (shapeType == dynamics::PyramidShape::getStaticType()) {
    const auto& pyramid
        = std::static_pointer_cast<const dynamics::PyramidShape>(shape);
    const double w = pyramid->getBaseWidth();
    const double d = pyramid->getBaseDepth();
    const double h = pyramid->getHeight();

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(5);
    vertices.emplace_back(0.0, 0.0, h * 0.5);          // apex
    vertices.emplace_back(w * 0.5, d * 0.5, -h * 0.5); // base corners
    vertices.emplace_back(-w * 0.5, d * 0.5, -h * 0.5);
    vertices.emplace_back(-w * 0.5, -d * 0.5, -h * 0.5);
    vertices.emplace_back(w * 0.5, -d * 0.5, -h * 0.5);

    return std::make_unique<experimental::ConvexShape>(std::move(vertices));
  }

  if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto& ellipsoid
        = std::static_pointer_cast<const dynamics::EllipsoidShape>(shape);
    if (ellipsoid->isSphere()) {
      const auto radii = ellipsoid->getRadii();
      return std::make_unique<experimental::SphereShape>(radii.x());
    }

    const auto radii = ellipsoid->getRadii();
    std::vector<Eigen::Vector3d> vertices;
    constexpr int numLatitude = 8;
    constexpr int numLongitude = 16;
    vertices.reserve(numLatitude * numLongitude + 2);
    vertices.emplace_back(0.0, 0.0, radii.z());
    vertices.emplace_back(0.0, 0.0, -radii.z());

    constexpr double kPi = std::numbers::pi_v<double>;
    constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
    for (int lat = 1; lat < numLatitude; ++lat) {
      const double theta = kPi * static_cast<double>(lat) / numLatitude;
      const double sinTheta = std::sin(theta);
      const double cosTheta = std::cos(theta);

      for (int lon = 0; lon < numLongitude; ++lon) {
        const double phi = kTwoPi * static_cast<double>(lon) / numLongitude;
        vertices.emplace_back(
            radii.x() * sinTheta * std::cos(phi),
            radii.y() * sinTheta * std::sin(phi),
            radii.z() * cosTheta);
      }
    }

    return std::make_unique<experimental::ConvexShape>(std::move(vertices));
  }

  if (shapeType == dynamics::HeightmapShape<double>::getStaticType()
      || shapeType == dynamics::HeightmapShape<float>::getStaticType()) {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<experimental::MeshShape::Triangle> triangles;

    auto buildMeshFromHeightmap
        = [&vertices, &triangles](const auto* heightmap) -> bool {
      if (!heightmap) {
        return false;
      }

      const auto width = heightmap->getWidth();
      const auto depth = heightmap->getDepth();
      if (width < 2 || depth < 2) {
        DART_WARN(
            "[ExperimentalCollisionDetector] HeightmapShape is too small to "
            "triangulate (width={}, depth={}).",
            width,
            depth);
        return false;
      }

      const auto& field = heightmap->getHeightField();
      const auto scale = heightmap->getScale().template cast<double>();
      const double xOffset = 0.5 * static_cast<double>(width - 1);
      const double yOffset = 0.5 * static_cast<double>(depth - 1);

      vertices.reserve(width * depth);
      for (std::size_t y = 0; y < depth; ++y) {
        for (std::size_t x = 0; x < width; ++x) {
          const double height = static_cast<double>(field(y, x));
          vertices.emplace_back(
              scale.x() * (static_cast<double>(x) - xOffset),
              -scale.y() * (static_cast<double>(y) - yOffset),
              scale.z() * height);
        }
      }

      triangles.reserve((width - 1) * (depth - 1) * 2);
      for (std::size_t y = 0; y + 1 < depth; ++y) {
        for (std::size_t x = 0; x + 1 < width; ++x) {
          const int v00 = static_cast<int>(y * width + x);
          const int v10 = static_cast<int>(y * width + (x + 1));
          const int v01 = static_cast<int>((y + 1) * width + x);
          const int v11 = static_cast<int>((y + 1) * width + (x + 1));

          triangles.emplace_back(v00, v10, v01);
          triangles.emplace_back(v10, v11, v01);
        }
      }

      return true;
    };

    if (const auto* heightmapDouble = shape->as<dynamics::HeightmapShaped>()) {
      if (buildMeshFromHeightmap(heightmapDouble)) {
        return std::make_unique<experimental::MeshShape>(
            std::move(vertices), std::move(triangles));
      }
    } else if (
        const auto* heightmapFloat = shape->as<dynamics::HeightmapShapef>()) {
      if (buildMeshFromHeightmap(heightmapFloat)) {
        return std::make_unique<experimental::MeshShape>(
            std::move(vertices), std::move(triangles));
      }
    }

    return nullptr;
  }

  if (shapeType == dynamics::MultiSphereConvexHullShape::getStaticType()) {
    const auto& multiSphere
        = std::static_pointer_cast<const dynamics::MultiSphereConvexHullShape>(
            shape);
    const auto& spheres = multiSphere->getSpheres();
    if (spheres.empty()) {
      DART_WARN(
          "[ExperimentalCollisionDetector] MultiSphereConvexHullShape has no "
          "spheres.");
      return nullptr;
    }

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(spheres.size() * 6);
    for (const auto& sphere : spheres) {
      const double radius = sphere.first;
      const Eigen::Vector3d& center = sphere.second;
      vertices.emplace_back(center + Eigen::Vector3d(radius, 0.0, 0.0));
      vertices.emplace_back(center + Eigen::Vector3d(-radius, 0.0, 0.0));
      vertices.emplace_back(center + Eigen::Vector3d(0.0, radius, 0.0));
      vertices.emplace_back(center + Eigen::Vector3d(0.0, -radius, 0.0));
      vertices.emplace_back(center + Eigen::Vector3d(0.0, 0.0, radius));
      vertices.emplace_back(center + Eigen::Vector3d(0.0, 0.0, -radius));
    }

    return std::make_unique<experimental::ConvexShape>(std::move(vertices));
  }

  if (shapeType == dynamics::MeshShape::getStaticType()) {
    const auto& mesh
        = std::static_pointer_cast<const dynamics::MeshShape>(shape);
    const auto triMesh = mesh->getTriMesh();
    if (!triMesh) {
      DART_WARN(
          "[ExperimentalCollisionDetector] MeshShape has no TriMesh data.");
      return nullptr;
    }

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(triMesh->getVertices().size());

    const Eigen::Vector3d scale = mesh->getScale();
    for (const auto& vertex : triMesh->getVertices()) {
      vertices.push_back(vertex.cwiseProduct(scale));
    }

    std::vector<experimental::MeshShape::Triangle> triangles;
    triangles.reserve(triMesh->getTriangles().size());
    for (const auto& triangle : triMesh->getTriangles()) {
      triangles.emplace_back(
          static_cast<int>(triangle[0]),
          static_cast<int>(triangle[1]),
          static_cast<int>(triangle[2]));
    }

    return std::make_unique<experimental::MeshShape>(
        std::move(vertices), std::move(triangles));
  }

  if (shapeType == dynamics::ConvexMeshShape::getStaticType()) {
    const auto& convex
        = std::static_pointer_cast<const dynamics::ConvexMeshShape>(shape);
    const auto& mesh = convex->getMesh();
    if (!mesh) {
      DART_WARN(
          "[ExperimentalCollisionDetector] ConvexMeshShape has no mesh data.");
      return nullptr;
    }

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(mesh->getVertices().size());
    for (const auto& vertex : mesh->getVertices()) {
      vertices.push_back(vertex);
    }

    return std::make_unique<experimental::ConvexShape>(std::move(vertices));
  }

  DART_WARN(
      "[ExperimentalCollisionDetector] Unsupported shape type: {}", shapeType);
  return nullptr;
}

} // namespace collision
} // namespace dart
