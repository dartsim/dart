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

#include <dart/gui/renderable.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <algorithm>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cstddef>

namespace dart::gui {

RenderableId makeRenderableId(const dynamics::ShapeFrame& shapeFrame)
{
  return reinterpret_cast<RenderableId>(&shapeFrame);
}

RenderableId makeRenderableId(const dynamics::ShapeNode& shapeNode)
{
  return makeRenderableId(static_cast<const dynamics::ShapeFrame&>(shapeNode));
}

namespace {

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
  // PBR overrides are applied at renderable creation, so changing them must
  // bump the resource version to force a rebuild that re-applies the new
  // values. The empty-optional sentinel is a probabilistic distinguisher, not
  // a guaranteed-disjoint one: some double's hash could equal it, in which
  // case the worst outcome is a missed rebuild on a runtime edit.
  const auto hashOptional = [&seed](const std::optional<double>& value) {
    hashCombine(
        seed,
        value.has_value() ? std::hash<double>{}(*value)
                          : static_cast<std::size_t>(0x9e3779b97f4a7c15ULL));
  };
  hashOptional(material.metallic);
  hashOptional(material.roughness);
  hashOptional(material.reflectance);
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

std::size_t computeDeformableSurfaceRenderResourceVersion(
    std::span<const Eigen::Vector3d> positions,
    std::span<const Eigen::Vector3i> triangles,
    const DeformableSurfaceRenderOptions& options)
{
  std::size_t seed = options.version;
  hashCombine(seed, positions.size());
  for (const Eigen::Vector3d& position : positions) {
    hashVector3d(seed, position);
  }

  hashCombine(seed, triangles.size());
  for (const Eigen::Vector3i& triangle : triangles) {
    for (int axis = 0; axis < 3; ++axis) {
      hashCombine(seed, std::hash<int>{}(triangle[axis]));
    }
  }

  hashVector4d(seed, options.surfaceColor);
  hashCombine(seed, std::hash<double>{}(options.roughnessFactor));
  hashCombine(seed, std::hash<double>{}(options.metallicFactor));
  hashCombine(seed, std::hash<double>{}(options.boundsPadding));
  hashCombine(seed, static_cast<std::size_t>(MeshAlphaMode::ShapeAlpha));
  return seed;
}

} // namespace

// Assembles a descriptor from an already-built geometry. Splitting this out
// from describeShape() lets RenderableExtractor inject a cached
// GeometryDescriptor while keeping the rest of the per-frame fields (transform,
// material, versions) fresh.
RenderableDescriptor assembleRenderableDescriptor(
    const dynamics::ShapeFrame& shapeFrame,
    const dynamics::VisualAspect& visualAspect,
    const std::string& skeletonName,
    const std::string& bodyName,
    const std::string& shapeNodeName,
    const dynamics::WeakConstSimpleFramePtr& simpleFrame,
    const dynamics::Shape& shape,
    GeometryDescriptor geometry)
{
  RenderableDescriptor descriptor;
  descriptor.id = makeRenderableId(shapeFrame);
  descriptor.shapeFrame = &shapeFrame;
  descriptor.shapeNode = shapeFrame.asShapeNode();
  descriptor.shape = &shape;
  if (descriptor.shapeNode != nullptr) {
    descriptor.skeleton = descriptor.shapeNode->getSkeleton();
  }
  descriptor.simpleFrame = simpleFrame;
  descriptor.skeletonName = skeletonName;
  descriptor.bodyName = bodyName;
  descriptor.shapeFrameName = shapeFrame.getName();
  descriptor.shapeNodeName = shapeNodeName;
  descriptor.geometry = std::move(geometry);
  descriptor.material.rgba = visualAspect.getRGBA();
  descriptor.material.visible = !visualAspect.isHidden();
  descriptor.material.castsShadows = visualAspect.getShadowed();
  descriptor.material.receivesShadows = visualAspect.getShadowed();
  // Negative aspect PBR values mean "renderer default"; only forward explicit,
  // non-negative overrides into the descriptor.
  if (visualAspect.getMetallic() >= 0.0) {
    descriptor.material.metallic = visualAspect.getMetallic();
  }
  if (visualAspect.getRoughness() >= 0.0) {
    descriptor.material.roughness = visualAspect.getRoughness();
  }
  if (visualAspect.getReflectance() >= 0.0) {
    descriptor.material.reflectance = visualAspect.getReflectance();
  }
  descriptor.worldTransform = shapeFrame.getWorldTransform();
  descriptor.shapeFrameVersion = shapeFrame.getVersion();
  descriptor.shapeNodeVersion = descriptor.shapeNode != nullptr
                                    ? descriptor.shapeNode->getVersion()
                                    : 0;
  descriptor.shapeVersion = shape.getVersion();
  descriptor.renderResourceVersion = computeRenderResourceVersion(
      descriptor.geometry, descriptor.material, descriptor.shapeVersion);
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

  return assembleRenderableDescriptor(
      shapeFrame,
      visualAspect,
      skeletonName,
      bodyName,
      shapeNodeName,
      simpleFrame,
      *shape,
      std::move(*geometry));
}

std::vector<RenderableDescriptor> extractRenderables(const simulation::World&)
{
  return {};
}

std::optional<RenderableDescriptor> describeShapeFrame(
    const dynamics::ShapeFrame& shapeFrame)
{
  const auto* visualAspect = shapeFrame.getVisualAspect();
  if (visualAspect == nullptr) {
    return std::nullopt;
  }

  return makeRenderableDescriptor(
      shapeFrame, *visualAspect, {}, {}, shapeFrame.getName(), {});
}

std::optional<RenderableDescriptor> makeDeformableSurfaceRenderable(
    RenderableId id,
    std::span<const Eigen::Vector3d> positions,
    std::span<const Eigen::Vector3i> triangles,
    const DeformableSurfaceRenderOptions& options)
{
  if (id == 0u || positions.empty() || triangles.empty()) {
    return std::nullopt;
  }

  Eigen::Vector3d boundsMin
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d boundsMax
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(positions.size());
  for (const Eigen::Vector3d& position : positions) {
    if (!position.allFinite()) {
      return std::nullopt;
    }
    vertices.push_back(position);
    boundsMin = boundsMin.cwiseMin(position);
    boundsMax = boundsMax.cwiseMax(position);
  }

  std::vector<Eigen::Vector3d> normals(
      positions.size(), Eigen::Vector3d::Zero());
  for (const Eigen::Vector3i& triangle : triangles) {
    if (triangle.x() < 0 || triangle.y() < 0 || triangle.z() < 0) {
      return std::nullopt;
    }

    const auto a = static_cast<std::size_t>(triangle.x());
    const auto b = static_cast<std::size_t>(triangle.y());
    const auto c = static_cast<std::size_t>(triangle.z());
    if (a >= positions.size() || b >= positions.size()
        || c >= positions.size()) {
      return std::nullopt;
    }

    const Eigen::Vector3d normal
        = (positions[b] - positions[a]).cross(positions[c] - positions[a]);
    if (normal.squaredNorm() <= 1e-16) {
      continue;
    }

    const Eigen::Vector3d unitNormal = normal.normalized();
    normals[a] += unitNormal;
    normals[b] += unitNormal;
    normals[c] += unitNormal;
  }

  for (auto& normal : normals) {
    if (normal.squaredNorm() <= 1e-16) {
      normal = Eigen::Vector3d::UnitZ();
    } else {
      normal.normalize();
    }
  }

  GeometryDescriptor geometry;
  geometry.kind = ShapeKind::Mesh;
  geometry.triangleVertices = std::move(vertices);
  geometry.triangleIndices.assign(triangles.begin(), triangles.end());
  geometry.triangleNormals = std::move(normals);
  geometry.hasLocalBounds = true;
  const double padding = std::max(0.0, options.boundsPadding);
  geometry.localBoundsMin = boundsMin - Eigen::Vector3d::Constant(padding);
  geometry.localBoundsMax = boundsMax + Eigen::Vector3d::Constant(padding);
  geometry.meshUsesMaterialColors = true;
  geometry.meshAlphaMode = MeshAlphaMode::ShapeAlpha;

  MeshMaterialDescriptor surfaceMaterial;
  surfaceMaterial.diffuse = options.surfaceColor;
  surfaceMaterial.metallicFactor = options.metallicFactor;
  surfaceMaterial.roughnessFactor = options.roughnessFactor;
  geometry.meshMaterials.push_back(std::move(surfaceMaterial));
  geometry.meshParts.push_back(
      MeshPartDescriptor{
          .vertexOffset = 0u,
          .vertexCount = geometry.triangleVertices.size(),
          .triangleOffset = 0u,
          .triangleCount = geometry.triangleIndices.size(),
          .materialIndex = 0u,
      });

  RenderableDescriptor descriptor;
  descriptor.id = id;
  descriptor.geometry = std::move(geometry);
  descriptor.material.rgba = options.surfaceColor;
  descriptor.material.visible = options.visible;
  descriptor.material.castsShadows = options.castsShadows;
  descriptor.material.receivesShadows = options.receivesShadows;
  descriptor.worldTransform = Eigen::Isometry3d::Identity();
  descriptor.shapeVersion = options.version;
  descriptor.renderResourceVersion
      = computeDeformableSurfaceRenderResourceVersion(
          positions, triangles, options);
  return descriptor;
}

std::vector<Eigen::Vector3i> makeGridSurfaceTriangles(
    std::size_t columns, std::size_t rows)
{
  if (columns < 2u || rows < 2u) {
    return {};
  }

  constexpr std::size_t maxIndex
      = static_cast<std::size_t>(std::numeric_limits<int>::max());
  if (columns > maxIndex / rows) {
    return {};
  }

  std::vector<Eigen::Vector3i> triangles;
  triangles.reserve((columns - 1u) * (rows - 1u) * 2u);
  const auto index = [columns](std::size_t column, std::size_t row) {
    return static_cast<int>(row * columns + column);
  };

  for (std::size_t row = 0u; row + 1u < rows; ++row) {
    for (std::size_t column = 0u; column + 1u < columns; ++column) {
      const int a = index(column, row);
      const int b = index(column + 1u, row);
      const int c = index(column, row + 1u);
      const int d = index(column + 1u, row + 1u);
      triangles.emplace_back(a, b, c);
      triangles.emplace_back(b, d, c);
    }
  }

  return triangles;
}

// Whether a shape kind's geometry only changes when the shape version changes
// (so it is safe to cache keyed by version). Kinds whose geometry can change
// in place without a version bump must be rebuilt every frame; these are
// exactly the kinds computeRenderResourceVersion() vertex-hashes (soft meshes,
// point clouds) plus voxel grids, whose describeShape() reads live cell data.
[[maybe_unused]] bool isCacheableGeometryKind(ShapeKind kind)
{
  switch (kind) {
    case ShapeKind::SoftMesh:
    case ShapeKind::PointCloud:
    case ShapeKind::VoxelGrid:
      return false;
    default:
      return true;
  }
}

std::vector<RenderableDescriptor> RenderableExtractor::extract(
    const simulation::World&)
{
  mGeometryCache.clear();
  return {};
}

} // namespace dart::gui
