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
#include "dart/dynamics/convex_mesh_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/plane_shape.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/math/tri_mesh.hpp"

#include <vector>

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

  if (shapeType == dynamics::PlaneShape::getStaticType()) {
    const auto& plane
        = std::static_pointer_cast<const dynamics::PlaneShape>(shape);
    return std::make_unique<experimental::PlaneShape>(
        plane->getNormal(), plane->getOffset());
  }

  if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto& ellipsoid
        = std::static_pointer_cast<const dynamics::EllipsoidShape>(shape);
    if (ellipsoid->isSphere()) {
      const auto radii = ellipsoid->getRadii();
      return std::make_unique<experimental::SphereShape>(radii.x());
    }
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
