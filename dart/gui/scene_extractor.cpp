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

#include "dart/gui/scene_extractor.hpp"

#include "dart/common/logging.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/capsule_shape.hpp"
#include "dart/dynamics/cone_shape.hpp"
#include "dart/dynamics/convex_mesh_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/line_segment_shape.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/multi_sphere_convex_hull_shape.hpp"
#include "dart/dynamics/plane_shape.hpp"
#include "dart/dynamics/point_cloud_shape.hpp"
#include "dart/dynamics/pyramid_shape.hpp"
#include "dart/dynamics/shape_frame.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/simulation/world.hpp"

namespace dart {
namespace gui {

Scene SceneExtractor::extract(const dart::simulation::World& world) const
{
  Scene scene;

  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    auto skeleton = world.getSkeleton(i);
    if (!skeleton) {
      continue;
    }

    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      auto* bodyNode = skeleton->getBodyNode(j);
      if (!bodyNode) {
        continue;
      }

      for (std::size_t k = 0; k < bodyNode->getNumShapeNodes(); ++k) {
        auto* shapeNode = bodyNode->getShapeNode(k);
        if (!shapeNode) {
          continue;
        }

        auto* visual = shapeNode->getVisualAspect();
        if (!visual) {
          continue;
        }

        auto shape = shapeNode->getShape();
        if (!shape) {
          continue;
        }

        auto shapeData = convertShape(*shape);
        if (!shapeData) {
          DART_WARN("Skipping unsupported shape type: {}", shape->getType());
          continue;
        }

        SceneNode node;
        node.id = shape->getID();
        node.transform = shapeNode->getWorldTransform();
        node.shape = std::move(*shapeData);
        node.material = extractMaterial(*shapeNode);
        node.visible = !visual->isHidden();
        scene.nodes.push_back(std::move(node));
      }
    }
  }

  return scene;
}

std::optional<ShapeData> SceneExtractor::convertShape(
    const dart::dynamics::Shape& shape) const
{
  if (auto box = dynamic_cast<const dart::dynamics::BoxShape*>(&shape)) {
    return ShapeData{BoxData{box->getSize()}};
  }

  if (auto sphere = dynamic_cast<const dart::dynamics::SphereShape*>(&shape)) {
    return ShapeData{SphereData{sphere->getRadius()}};
  }

  if (auto cylinder
      = dynamic_cast<const dart::dynamics::CylinderShape*>(&shape)) {
    return ShapeData{
        CylinderData{cylinder->getRadius(), cylinder->getHeight()}};
  }

  if (auto capsule
      = dynamic_cast<const dart::dynamics::CapsuleShape*>(&shape)) {
    return ShapeData{CapsuleData{capsule->getRadius(), capsule->getHeight()}};
  }

  if (auto cone = dynamic_cast<const dart::dynamics::ConeShape*>(&shape)) {
    return ShapeData{ConeData{cone->getRadius(), cone->getHeight()}};
  }

  if (auto ellipsoid
      = dynamic_cast<const dart::dynamics::EllipsoidShape*>(&shape)) {
    return ShapeData{EllipsoidData{ellipsoid->getRadii()}};
  }

  if (auto plane = dynamic_cast<const dart::dynamics::PlaneShape*>(&shape)) {
    return ShapeData{PlaneData{plane->getNormal(), plane->getOffset()}};
  }

  if (auto convexMesh
      = dynamic_cast<const dart::dynamics::ConvexMeshShape*>(&shape)) {
    auto triMesh = convexMesh->getMesh();
    if (!triMesh || !triMesh->hasVertices() || !triMesh->hasTriangles()) {
      return std::nullopt;
    }

    MeshData meshData;
    const auto& vertices = triMesh->getVertices();
    meshData.vertices.reserve(vertices.size());

    for (const auto& vertex : vertices) {
      meshData.vertices.emplace_back(vertex.cast<float>());
    }

    if (triMesh->hasVertexNormals()
        && triMesh->getVertexNormals().size() == vertices.size()) {
      const auto& normals = triMesh->getVertexNormals();
      meshData.normals.reserve(normals.size());
      for (const auto& normal : normals) {
        meshData.normals.emplace_back(normal.normalized().cast<float>());
      }
    }

    const auto& triangles = triMesh->getTriangles();
    meshData.indices.reserve(triangles.size() * 3);
    for (const auto& triangle : triangles) {
      meshData.indices.push_back(static_cast<uint32_t>(triangle[0]));
      meshData.indices.push_back(static_cast<uint32_t>(triangle[1]));
      meshData.indices.push_back(static_cast<uint32_t>(triangle[2]));
    }

    return ShapeData{std::move(meshData)};
  }

  if (auto mesh = dynamic_cast<const dart::dynamics::MeshShape*>(&shape)) {
    auto triMesh = mesh->getTriMesh();
    if (!triMesh || !triMesh->hasVertices() || !triMesh->hasTriangles()) {
      return std::nullopt;
    }

    MeshData meshData;
    meshData.texture_path = mesh->getMeshUri();
    const auto& vertices = triMesh->getVertices();
    meshData.vertices.reserve(vertices.size());

    const Eigen::Vector3d scale = mesh->getScale();
    const double epsilon = 1e-12;
    const bool validScale = (scale.array().abs() > epsilon).all();
    Eigen::Vector3d invScale = scale.cwiseInverse();
    if (!validScale) {
      invScale.setOnes();
    }

    for (const auto& vertex : vertices) {
      Eigen::Vector3d scaled = vertex.cwiseProduct(scale);
      meshData.vertices.emplace_back(scaled.cast<float>());
    }

    if (triMesh->hasVertexNormals()
        && triMesh->getVertexNormals().size() == vertices.size()) {
      const auto& normals = triMesh->getVertexNormals();
      meshData.normals.reserve(normals.size());
      for (const auto& normal : normals) {
        Eigen::Vector3d transformed = normal;
        if (validScale) {
          transformed = normal.cwiseProduct(invScale);
          if (transformed.norm() > epsilon) {
            transformed.normalize();
          }
        }
        meshData.normals.emplace_back(transformed.cast<float>());
      }
    }

    const auto& triangles = triMesh->getTriangles();
    meshData.indices.reserve(triangles.size() * 3);
    for (const auto& triangle : triangles) {
      meshData.indices.push_back(static_cast<uint32_t>(triangle[0]));
      meshData.indices.push_back(static_cast<uint32_t>(triangle[1]));
      meshData.indices.push_back(static_cast<uint32_t>(triangle[2]));
    }

    return ShapeData{std::move(meshData)};
  }

  if (auto lines
      = dynamic_cast<const dart::dynamics::LineSegmentShape*>(&shape)) {
    LineData lineData;
    auto vertices = lines->getVertices();
    auto connections = lines->getConnections();

    lineData.segments.reserve(connections.size());
    for (const auto& connection : connections) {
      if (connection[0] < 0 || connection[1] < 0) {
        continue;
      }

      const std::size_t startIdx = static_cast<std::size_t>(connection[0]);
      const std::size_t endIdx = static_cast<std::size_t>(connection[1]);
      if (startIdx >= vertices.size() || endIdx >= vertices.size()) {
        continue;
      }

      lineData.segments.emplace_back(vertices[startIdx], vertices[endIdx]);
    }

    return ShapeData{std::move(lineData)};
  }

  if (auto pyramid
      = dynamic_cast<const dart::dynamics::PyramidShape*>(&shape)) {
    return ShapeData{PyramidData{
        pyramid->getBaseWidth(),
        pyramid->getBaseDepth(),
        pyramid->getHeight()}};
  }

  if (auto pc = dynamic_cast<const dart::dynamics::PointCloudShape*>(&shape)) {
    PointCloudData data;
    auto points = pc->getPoints();
    data.points.assign(points.begin(), points.end());
    data.visual_size = pc->getVisualSize();
    switch (pc->getPointShapeType()) {
      case dart::dynamics::PointCloudShape::BOX:
        data.point_shape_type = PointShapeType::Box;
        break;
      case dart::dynamics::PointCloudShape::BILLBOARD_SQUARE:
        data.point_shape_type = PointShapeType::BillboardSquare;
        break;
      case dart::dynamics::PointCloudShape::BILLBOARD_CIRCLE:
        data.point_shape_type = PointShapeType::BillboardCircle;
        break;
      case dart::dynamics::PointCloudShape::POINT:
        data.point_shape_type = PointShapeType::Point;
        break;
    }
    switch (pc->getColorMode()) {
      case dart::dynamics::PointCloudShape::USE_SHAPE_COLOR:
        data.color_mode = PointCloudColorMode::ShapeColor;
        break;
      case dart::dynamics::PointCloudShape::BIND_OVERALL:
        data.color_mode = PointCloudColorMode::Overall;
        break;
      case dart::dynamics::PointCloudShape::BIND_PER_POINT:
        data.color_mode = PointCloudColorMode::PerPoint;
        break;
    }
    auto colors = pc->getColors();
    data.colors.assign(colors.begin(), colors.end());
    if (!data.colors.empty()) {
      data.overall_color = data.colors.front();
    }
    return ShapeData{std::move(data)};
  }

  if (auto multiSphere
      = dynamic_cast<const dart::dynamics::MultiSphereConvexHullShape*>(
          &shape)) {
    MultiSphereData data;
    data.spheres = multiSphere->getSpheres();
    return ShapeData{std::move(data)};
  }

  return std::nullopt;
}

Material SceneExtractor::extractMaterial(
    const dart::dynamics::ShapeNode& node) const
{
  Material material;
  auto* visual = node.getVisualAspect();
  if (visual) {
    material.color = visual->getRGBA();
  }

  return material;
}

} // namespace gui
} // namespace dart
