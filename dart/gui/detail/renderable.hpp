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

#pragma once

#include <dart/gui/Export.hpp>

namespace dart::simulation {
class World;
}

#include <dart/dynamics/SmartPointer.hpp>

namespace dart::dynamics {
class Frame;
class Shape;
class ShapeNode;
class ShapeFrame;
class VisualAspect;
class BodyNode;
} // namespace dart::dynamics

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::gui {

using RenderableId = std::uintptr_t;

enum class ShapeKind
{
  Box,
  Sphere,
  Ellipsoid,
  Cylinder,
  Capsule,
  Cone,
  Pyramid,
  MultiSphere,
  LineSegments,
  ConvexMesh,
  PointCloud,
  Heightmap,
  SoftMesh,
  VoxelGrid,
  Mesh,
  Plane,
  Unsupported
};

enum class PointShapeType
{
  Box,
  BillboardSquare,
  BillboardCircle,
  Point
};

enum class MeshAlphaMode
{
  Blend,
  Auto,
  ShapeAlpha
};

struct MeshMaterialDescriptor
{
  Eigen::Vector4d ambient{0.2, 0.2, 0.2, 1.0};
  Eigen::Vector4d diffuse{0.8, 0.8, 0.8, 1.0};
  Eigen::Vector4d specular{0.0, 0.0, 0.0, 1.0};
  Eigen::Vector4d emissive{0.0, 0.0, 0.0, 1.0};
  double shininess = 0.0;
  double shininessStrength = 1.0;
  bool hasPbrMetallic = false;
  bool hasPbrRoughness = false;
  double metallicFactor = 0.0;
  double roughnessFactor = 1.0;
  std::string baseColorTexturePath;
  std::string metallicTexturePath;
  std::string roughnessTexturePath;
  std::string metallicRoughnessTexturePath;
  std::string normalTexturePath;
  std::string occlusionTexturePath;
  std::string emissiveTexturePath;
  std::vector<std::string> textureImagePaths;
};

struct MeshPartDescriptor
{
  std::size_t vertexOffset = 0u;
  std::size_t vertexCount = 0u;
  std::size_t triangleOffset = 0u;
  std::size_t triangleCount = 0u;
  std::size_t pointOffset = 0u;
  std::size_t pointCount = 0u;
  std::size_t lineOffset = 0u;
  std::size_t lineCount = 0u;
  unsigned int materialIndex = 0u;
  bool usesVertexColors = false;
};

struct GeometryDescriptor
{
  ShapeKind kind = ShapeKind::Unsupported;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d localBoundsMin = Eigen::Vector3d::Zero();
  Eigen::Vector3d localBoundsMax = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> sphereCenters;
  std::vector<double> sphereRadii;
  std::vector<Eigen::Vector3d> lineVertices;
  std::vector<Eigen::Vector2i> lineConnections;
  std::vector<Eigen::Vector3d> triangleVertices;
  std::vector<Eigen::Vector3i> triangleIndices;
  std::vector<int> meshPointIndices;
  std::vector<Eigen::Vector2i> meshLineIndices;
  std::vector<Eigen::Vector3d> triangleNormals;
  std::vector<Eigen::Vector3d> pointCloudPoints;
  std::vector<Eigen::Vector4d> pointCloudColors;
  std::vector<Eigen::Vector3d> voxelCenters;
  std::vector<double> voxelSizes;
  PointShapeType pointShapeType = PointShapeType::Box;
  bool meshUsesVertexColors = false;
  std::vector<Eigen::Vector4d> meshVertexColors;
  double radius = 0.0;
  double height = 0.0;
  double offset = 0.0;
  double lineThickness = 1.0;
  double pointSize = 1.0;
  double voxelSize = 1.0;
  bool hasLocalBounds = false;
  bool meshUsesMaterialColors = false;
  MeshAlphaMode meshAlphaMode = MeshAlphaMode::Blend;
  int meshTextureCoordComponents = 0;
  std::vector<Eigen::Vector3d> meshTextureCoordinates;
  std::string meshUri;
  std::string shapeType;
  std::string unsupportedReason;
  std::vector<MeshMaterialDescriptor> meshMaterials;
  std::vector<MeshPartDescriptor> meshParts;
};

struct MaterialDescriptor
{
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  bool visible = true;
  bool castsShadows = true;
  bool receivesShadows = true;
  /// Optional PBR overrides for primitive shapes. When unset, the renderer uses
  /// its per-shape default. Loaded meshes keep their own asset materials.
  std::optional<double> metallic;
  std::optional<double> roughness;
  std::optional<double> reflectance;
};

struct RenderableDescriptor
{
  RenderableId id = 0;
  const simulation::World* world = nullptr;
  const dynamics::ShapeFrame* shapeFrame = nullptr;
  const dynamics::ShapeNode* shapeNode = nullptr;
  const dynamics::Shape* shape = nullptr;
  dynamics::WeakConstSkeletonPtr skeleton;
  dynamics::WeakConstSimpleFramePtr simpleFrame;
  std::string skeletonName;
  std::string bodyName;
  std::string shapeFrameName;
  std::string shapeNodeName;
  GeometryDescriptor geometry;
  MaterialDescriptor material;
  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();
  std::size_t shapeFrameVersion = 0;
  std::size_t shapeNodeVersion = 0;
  std::size_t shapeVersion = 0;
  /// Geometry and alpha changes rebuild GPU resources; transforms and RGB
  /// update in place. Manually constructed descriptors must change this key
  /// after a resource edit.
  std::size_t renderResourceVersion = 0;
};

struct RenderableSetUpdatePlan
{
  std::vector<std::size_t> descriptorIndicesToAdd;
  std::vector<std::size_t> activeRenderableIndicesToRemove;
};

struct ActiveRenderableState
{
  RenderableId id = 0;
  std::size_t shapeVersion = 0;
  std::size_t renderResourceVersion = 0;
};

DART_GUI_API RenderableId
makeRenderableId(const dynamics::ShapeFrame& shapeFrame);

DART_GUI_API RenderableId
makeRenderableId(const dynamics::ShapeNode& shapeNode);

DART_GUI_API std::optional<GeometryDescriptor> describeShape(
    const dynamics::Shape& shape);

DART_GUI_API std::vector<RenderableDescriptor> extractRenderables(
    const simulation::World& world);

DART_GUI_API std::optional<RenderableDescriptor> describeShapeFrame(
    const dynamics::ShapeFrame& shapeFrame);

/// Stateful equivalent of extractRenderables() for the per-frame render loop.
///
/// Geometry (the GeometryDescriptor built by describeShape) only changes when a
/// shape's version changes, but transforms change every frame. This extractor
/// caches each shape's GeometryDescriptor keyed by the shape's stable id
/// (`Shape::getID()`) and its version, so describeShape — which rebuilds full
/// vertex/index arrays for meshes, heightmaps, soft meshes, and voxel grids —
/// is not re-run every frame for unchanged geometry. Transforms, materials, and
/// versions are always refreshed, so output is equivalent to
/// extractRenderables(). Reuses the same version-based caching assumption the
/// Filament resource sync already relies on. The key is the shape id rather
/// than the shape pointer so a deleted shape whose heap address is later reused
/// by a new shape cannot alias a stale cache entry (shape ids are never
/// reused).
class DART_GUI_API RenderableExtractor
{
public:
  const std::vector<RenderableDescriptor>& extract(
      const simulation::World& world);
  const std::vector<RenderableDescriptor>& extract(
      const std::vector<std::shared_ptr<simulation::World>>& worlds);

private:
  struct CachedGeometry
  {
    std::size_t version = 0;
    GeometryDescriptor geometry;
    std::size_t resourceVersion = 0;
    std::size_t generation = 0;
  };

  struct CachedRenderable
  {
    std::size_t index = 0;
    std::size_t generation = 0;
    std::size_t shapeId = 0;
    std::size_t geometryVersion = 0;
  };

  const std::vector<RenderableDescriptor>& extractWorlds(
      const simulation::World* const* worlds, std::size_t count);

  std::vector<RenderableDescriptor> mDescriptors;
  std::vector<const simulation::World*> mWorlds;
  std::unordered_map<RenderableId, CachedRenderable> mRenderableIndices;
  std::unordered_map<const dynamics::Frame*, std::size_t> mVisitedFrames;
  std::size_t mGeneration = 0;

  std::unordered_map<std::size_t, CachedGeometry> mGeometryCache;
};

DART_GUI_API RenderableSetUpdatePlan planRenderableSetUpdate(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<RenderableId>& activeRenderableIds);

DART_GUI_API RenderableSetUpdatePlan planRenderableSetUpdate(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<ActiveRenderableState>& activeRenderableStates);

} // namespace dart::gui
