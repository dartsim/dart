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

#ifndef DART_GUI_EXPERIMENTAL_SCENE_HPP_
#define DART_GUI_EXPERIMENTAL_SCENE_HPP_

#include <dart/gui/experimental/viewer.hpp>
#include <dart/gui/export.hpp>

#include <dart/collision/fwd.hpp>

#include <dart/dynamics/fwd.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <cstdint>

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::gui::experimental {

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
  double metallicFactor = 1.0;
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
  unsigned int materialIndex = 0u;
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
  std::vector<Eigen::Vector3d> triangleNormals;
  std::vector<Eigen::Vector3d> pointCloudPoints;
  std::vector<Eigen::Vector4d> pointCloudColors;
  std::vector<Eigen::Vector3d> voxelCenters;
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
};

struct RenderableDescriptor
{
  RenderableId id = 0;
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

struct PickHit
{
  RenderableId id = 0;
  std::size_t renderableIndex = 0;
  double distance = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

struct DebugLineDescriptor
{
  Eigen::Vector3d from = Eigen::Vector3d::Zero();
  Eigen::Vector3d to = Eigen::Vector3d::Zero();
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  std::string label;
};

struct DebugDrawOptions
{
  bool drawGrid = true;
  bool drawWorldFrame = true;
  bool drawBodyFrames = false;
  bool drawCentersOfMass = false;
  bool drawInertiaBoxes = false;
  bool drawCollisionShapeBounds = false;
  bool drawSupportPolygons = false;
  bool drawSupportCentroids = true;
  bool drawContacts = true;
  bool drawContactNormals = true;
  bool drawContactForces = true;
  double gridHalfExtent = 4.0;
  double gridSpacing = 0.5;
  double gridZ = 0.08;
  double worldFrameAxisLength = 0.9;
  double bodyFrameAxisLength = 0.22;
  double centerOfMassMarkerRadius = 0.08;
  double inertiaBoxScale = 1.0;
  double collisionBoundsPadding = 0.0;
  double supportPolygonElevation = 0.02;
  double supportCentroidMarkerRadius = 0.06;
  double contactMarkerHalfExtent = 0.035;
  double contactNormalLength = 0.22;
  double contactForceScale = 0.002;
  double contactForceMinLength = 0.04;
  double contactForceMaxLength = 0.45;
};

DART_GUI_API RenderableId
makeRenderableId(const dynamics::ShapeFrame& shapeFrame);

DART_GUI_API RenderableId
makeRenderableId(const dynamics::ShapeNode& shapeNode);

DART_GUI_API std::optional<GeometryDescriptor> describeShape(
    const dynamics::Shape& shape);

DART_GUI_API std::vector<RenderableDescriptor> extractRenderables(
    const simulation::World& world);

DART_GUI_API RenderableSetUpdatePlan planRenderableSetUpdate(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<RenderableId>& activeRenderableIds);

DART_GUI_API RenderableSetUpdatePlan planRenderableSetUpdate(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<ActiveRenderableState>& activeRenderableStates);

DART_GUI_API std::optional<double> intersectRenderable(
    const RenderableDescriptor& renderable, const PickRay& ray);

DART_GUI_API std::optional<PickHit> pickNearestRenderable(
    const std::vector<RenderableDescriptor>& renderables,
    const PickRay& ray,
    double maxDistance = std::numeric_limits<double>::infinity());

DART_GUI_API std::optional<Eigen::Vector3d> intersectPlane(
    const PickRay& ray,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal);

DART_GUI_API std::optional<Eigen::Vector3d> computePlaneDragTranslation(
    const PickRay& previousRay,
    const PickRay& currentRay,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal);

DART_GUI_API bool translateFreeJointRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation);

DART_GUI_API bool translateSimpleFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation);

DART_GUI_API bool translateFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation);

DART_GUI_API std::vector<DebugLineDescriptor> makeGridDebugLines(
    const DebugDrawOptions& options = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeFrameDebugLines(
    const Eigen::Isometry3d& transform,
    double axisLength,
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeSelectionDebugLines(
    const RenderableDescriptor& renderable,
    const Eigen::Vector4d& rgba = Eigen::Vector4d(1.0, 0.84, 0.18, 1.0),
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeInertiaDebugLines(
    const dynamics::BodyNode& bodyNode,
    const DebugDrawOptions& options = {},
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeCollisionShapeDebugLines(
    const dynamics::ShapeNode& shapeNode,
    const DebugDrawOptions& options = {},
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeSupportPolygonDebugLines(
    const dynamics::Skeleton& skeleton,
    const DebugDrawOptions& options = {},
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> extractContactDebugLines(
    const collision::CollisionResult& result,
    const DebugDrawOptions& options = {});

DART_GUI_API std::vector<DebugLineDescriptor> extractDebugLines(
    const simulation::World& world, const DebugDrawOptions& options = {});

} // namespace dart::gui::experimental

#endif // DART_GUI_EXPERIMENTAL_SCENE_HPP_
