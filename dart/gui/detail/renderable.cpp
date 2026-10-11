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

#include <dart/gui/detail/renderable.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <algorithm>
#include <deque>
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

std::size_t computeGeometryResourceVersion(
    const GeometryDescriptor& geometry, std::size_t shapeVersion)
{
  std::size_t seed = shapeVersion;
  hashCombine(seed, static_cast<std::size_t>(geometry.kind));
  if (geometry.kind == ShapeKind::Mesh) {
    for (const auto& part : geometry.meshParts) {
      hashCombine(seed, part.vertexOffset);
      hashCombine(seed, part.vertexCount);
      hashCombine(seed, part.triangleOffset);
      hashCombine(seed, part.triangleCount);
      hashCombine(seed, part.pointOffset);
      hashCombine(seed, part.pointCount);
      hashCombine(seed, part.lineOffset);
      hashCombine(seed, part.lineCount);
      hashCombine(seed, part.materialIndex);
      hashCombine(seed, part.usesVertexColors ? 1u : 0u);
    }
    for (int index : geometry.meshPointIndices)
      hashCombine(seed, std::hash<int>{}(index));
    for (const auto& line : geometry.meshLineIndices) {
      hashCombine(seed, std::hash<int>{}(line.x()));
      hashCombine(seed, std::hash<int>{}(line.y()));
    }
    for (const auto& uv : geometry.meshTextureCoordinates)
      hashVector3d(seed, uv);
    for (const auto& meshMaterial : geometry.meshMaterials) {
      hashVector4d(seed, meshMaterial.ambient);
      hashVector4d(seed, meshMaterial.diffuse);
      hashVector4d(seed, meshMaterial.specular);
      hashCombine(seed, std::hash<double>{}(meshMaterial.shininess));
      hashCombine(seed, std::hash<double>{}(meshMaterial.shininessStrength));
      hashCombine(seed, meshMaterial.hasPbrMetallic ? 1u : 0u);
      hashCombine(seed, meshMaterial.hasPbrRoughness ? 1u : 0u);
      hashVector4d(seed, meshMaterial.emissive);
      hashCombine(seed, std::hash<double>{}(meshMaterial.metallicFactor));
      hashCombine(seed, std::hash<double>{}(meshMaterial.roughnessFactor));
      hashCombine(
          seed, std::hash<std::string>{}(meshMaterial.baseColorTexturePath));
      hashCombine(
          seed, std::hash<std::string>{}(meshMaterial.metallicTexturePath));
      hashCombine(
          seed, std::hash<std::string>{}(meshMaterial.roughnessTexturePath));
      hashCombine(
          seed,
          std::hash<std::string>{}(meshMaterial.metallicRoughnessTexturePath));
      hashCombine(
          seed, std::hash<std::string>{}(meshMaterial.normalTexturePath));
      hashCombine(
          seed, std::hash<std::string>{}(meshMaterial.occlusionTexturePath));
      hashCombine(
          seed, std::hash<std::string>{}(meshMaterial.emissiveTexturePath));
    }
    hashCombine(seed, geometry.meshUsesMaterialColors ? 1u : 0u);
    hashCombine(seed, geometry.meshUsesVertexColors ? 1u : 0u);
    for (const auto& color : geometry.meshVertexColors)
      hashVector4d(seed, color);
    hashCombine(seed, static_cast<std::size_t>(geometry.meshAlphaMode));
  }
  if (geometry.kind == ShapeKind::LineSegments) {
    hashCombine(seed, std::hash<double>{}(geometry.lineThickness));
    for (const auto& point : geometry.lineVertices)
      hashVector3d(seed, point);
    for (const auto& connection : geometry.lineConnections) {
      hashCombine(seed, std::hash<int>{}(connection.x()));
      hashCombine(seed, std::hash<int>{}(connection.y()));
    }
  }
  if (geometry.kind == ShapeKind::PointCloud) {
    hashCombine(seed, static_cast<std::size_t>(geometry.pointShapeType));
    for (const auto& point : geometry.pointCloudPoints)
      hashVector3d(seed, point);
    hashCombine(seed, std::hash<double>{}(geometry.pointSize));
    hashCombine(seed, geometry.pointCloudColors.size());
    for (const Eigen::Vector4d& color : geometry.pointCloudColors) {
      hashVector4d(seed, color);
    }
  }
  if (geometry.kind == ShapeKind::VoxelGrid) {
    for (const auto& center : geometry.voxelCenters)
      hashVector3d(seed, center);
    for (double size : geometry.voxelSizes)
      hashCombine(seed, std::hash<double>{}(size));
  }
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
  hashCombine(seed, geometry.triangleNormals.size());
  for (const auto& normal : geometry.triangleNormals)
    hashVector3d(seed, normal);
  return seed;
}

std::size_t computeRenderResourceVersion(
    const GeometryDescriptor& geometry,
    const MaterialDescriptor& material,
    std::size_t shapeVersion,
    std::optional<std::size_t> cachedGeometryVersion)
{
  std::size_t seed = cachedGeometryVersion ? *cachedGeometryVersion
                                           : computeGeometryResourceVersion(
                                               geometry, shapeVersion);
  hashCombine(seed, std::hash<double>{}(material.rgba.w()));
  // Resetting an override also needs a rebuild to restore its default.
  const auto hashOptional = [&seed](const std::optional<double>& value) {
    hashCombine(
        seed,
        value.has_value() ? std::hash<double>{}(*value)
                          : static_cast<std::size_t>(0x9e3779b97f4a7c15ULL));
  };
  hashOptional(material.metallic);
  hashOptional(material.roughness);
  hashOptional(material.reflectance);
  return seed;
}

} // namespace

// Assembles a descriptor from an already-built geometry. Splitting this out
// from describeShape() lets RenderableExtractor inject a cached
// GeometryDescriptor while keeping the rest of the per-frame fields (transform,
// material, versions) fresh.
void refreshRenderableDescriptor(
    RenderableDescriptor& descriptor,
    const dynamics::ShapeFrame& shapeFrame,
    const dynamics::VisualAspect& visualAspect,
    const std::string& skeletonName,
    const std::string& bodyName,
    const std::string& shapeNodeName,
    const dynamics::WeakConstSimpleFramePtr& simpleFrame,
    const dynamics::Shape& shape,
    std::size_t cachedGeometryVersion)
{
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
  descriptor.material.rgba = visualAspect.getRGBA();
  descriptor.material.visible = !visualAspect.isHidden();
  descriptor.material.castsShadows = visualAspect.getShadowed();
  descriptor.material.receivesShadows = visualAspect.getShadowed();
  descriptor.worldTransform = shapeFrame.getWorldTransform();
  descriptor.shapeFrameVersion = shapeFrame.getVersion();
  descriptor.shapeNodeVersion = descriptor.shapeNode != nullptr
                                    ? descriptor.shapeNode->getVersion()
                                    : 0;
  descriptor.shapeVersion = shape.getVersion();
  descriptor.renderResourceVersion = computeRenderResourceVersion(
      GeometryDescriptor{},
      descriptor.material,
      descriptor.shapeVersion,
      cachedGeometryVersion);
  hashCombine(descriptor.renderResourceVersion, shape.getID());
}

RenderableDescriptor assembleRenderableDescriptor(
    const dynamics::ShapeFrame& shapeFrame,
    const dynamics::VisualAspect& visualAspect,
    const std::string& skeletonName,
    const std::string& bodyName,
    const std::string& shapeNodeName,
    const dynamics::WeakConstSimpleFramePtr& simpleFrame,
    const dynamics::Shape& shape,
    GeometryDescriptor geometry,
    std::optional<std::size_t> cachedGeometryVersion = std::nullopt)
{
  RenderableDescriptor descriptor;
  if (!cachedGeometryVersion)
    cachedGeometryVersion
        = computeGeometryResourceVersion(geometry, shape.getVersion());
  refreshRenderableDescriptor(
      descriptor,
      shapeFrame,
      visualAspect,
      skeletonName,
      bodyName,
      shapeNodeName,
      simpleFrame,
      shape,
      *cachedGeometryVersion);
  descriptor.geometry = std::move(geometry);
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

std::vector<RenderableDescriptor> extractRenderables(
    const simulation::World& world)
{
  RenderableExtractor extractor;
  return extractor.extract(world);
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

const std::vector<RenderableDescriptor>& RenderableExtractor::extract(
    const simulation::World& world)
{
  const auto* pointer = &world;
  return extractWorlds(&pointer, 1);
}

const std::vector<RenderableDescriptor>& RenderableExtractor::extract(
    const std::vector<std::shared_ptr<simulation::World>>& worlds)
{
  mWorlds.clear();
  mWorlds.reserve(worlds.size());
  for (const auto& world : worlds) {
    if (world)
      mWorlds.push_back(world.get());
  }
  return extractWorlds(mWorlds.data(), mWorlds.size());
}

const std::vector<RenderableDescriptor>& RenderableExtractor::extractWorlds(
    const simulation::World* const* worlds, std::size_t count)
{
  ++mGeneration;
  const auto visit = [&](const dynamics::Frame* root,
                         const dynamics::WeakConstSimpleFramePtr& owner,
                         const simulation::World* world) {
    std::deque<const dynamics::Frame*> frames{root};
    while (!frames.empty()) {
      const auto* frame = frames.front();
      frames.pop_front();
      if (!frame || mVisitedFrames[frame] == mGeneration)
        continue;
      mVisitedFrames[frame] = mGeneration;
      // The const overload copies the child set; traversal only reads the live
      // set.
      for (const auto* child :
           const_cast<dynamics::Frame*>(frame)->getChildFrames())
        frames.push_back(child);
      const auto* shapeFrame = frame->asShapeFrame();
      if (!shapeFrame || !shapeFrame->getVisualAspect()
          || !shapeFrame->getShape())
        continue;
      const auto shape = shapeFrame->getShape();
      const auto id = shape->getID();
      const auto cached = mGeometryCache.find(id);
      std::size_t cacheVersion = shape->getVersion();
      if (const auto* mesh
          = dynamic_cast<const dynamics::MeshShape*>(shape.get())) {
        hashCombine(
            cacheVersion, static_cast<std::size_t>(mesh->getColorMode()));
        hashCombine(
            cacheVersion, static_cast<std::size_t>(mesh->getAlphaMode()));
        hashCombine(cacheVersion, std::hash<int>{}(mesh->getColorIndex()));
      }
      const auto variance = shape->getDataVariance();
      const auto dynamicGeometry = dynamics::Shape::DYNAMIC_PRIMITIVE
                                   | dynamics::Shape::DYNAMIC_COLOR
                                   | dynamics::Shape::DYNAMIC_VERTICES
                                   | dynamics::Shape::DYNAMIC_ELEMENTS;
      bool refresh = cached == mGeometryCache.end()
                     || cached->second.version != cacheVersion
                     || (variance & dynamicGeometry) != 0;
      if (cached != mGeometryCache.end()) {
        const auto kind = cached->second.geometry.kind;
        refresh = refresh || kind == ShapeKind::SoftMesh
                  || kind == ShapeKind::PointCloud
                  || kind == ShapeKind::VoxelGrid;
        refresh = refresh && cached->second.generation != mGeneration;
      }
      if (refresh) {
        auto geometry = describeShape(*shape);
        if (!geometry)
          continue;
        const auto resourceVersion
            = computeGeometryResourceVersion(*geometry, shape->getVersion());
        mGeometryCache[id] = {
            cacheVersion, std::move(*geometry), resourceVersion, mGeneration};
      }
      auto& geometry = mGeometryCache.at(id);
      geometry.generation = mGeneration;
      const auto* node = shapeFrame->asShapeNode();
      const auto* body = node ? node->getBodyNode() : nullptr;
      const auto skeleton = node ? node->getSkeleton() : nullptr;
      const auto renderableId = makeRenderableId(*shapeFrame);
      auto existing = mRenderableIndices.find(renderableId);
      if (existing == mRenderableIndices.end()) {
        mDescriptors.emplace_back();
        mDescriptors.back().geometry = geometry.geometry;
        existing = mRenderableIndices
                       .emplace(
                           renderableId,
                           CachedRenderable{
                               mDescriptors.size() - 1,
                               mGeneration,
                               id,
                               geometry.resourceVersion})
                       .first;
      }
      auto& state = existing->second;
      auto& descriptor = mDescriptors[state.index];
      descriptor.world = world;
      if (state.shapeId != id
          || state.geometryVersion != geometry.resourceVersion) {
        descriptor.geometry = geometry.geometry;
        state.shapeId = id;
        state.geometryVersion = geometry.resourceVersion;
      }
      state.generation = mGeneration;
      refreshRenderableDescriptor(
          descriptor,
          *shapeFrame,
          *shapeFrame->getVisualAspect(),
          skeleton ? skeleton->getName() : std::string{},
          body ? body->getName() : std::string{},
          shapeFrame->getName(),
          owner,
          *shape,
          geometry.resourceVersion);
    }
  };
  for (std::size_t worldIndex = 0; worldIndex < count; ++worldIndex) {
    const auto& world = *worlds[worldIndex];
    for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
      const auto skeleton = world.getSkeleton(i);
      for (std::size_t root = 0; root < skeleton->getNumTrees(); ++root)
        visit(skeleton->getRootBodyNode(root), {}, &world);
    }
    for (std::size_t i = 0; i < world.getNumSimpleFrames(); ++i) {
      const auto frame = world.getSimpleFrame(i);
      visit(frame.get(), frame, &world);
    }
  }
  std::size_t retained = 0;
  for (std::size_t i = 0; i < mDescriptors.size(); ++i) {
    const auto entry = mRenderableIndices.find(mDescriptors[i].id);
    if (entry->second.generation != mGeneration) {
      mRenderableIndices.erase(entry);
      continue;
    }
    if (retained != i)
      mDescriptors[retained] = std::move(mDescriptors[i]);
    entry->second.index = retained++;
  }
  mDescriptors.resize(retained);
  for (auto it = mGeometryCache.begin(); it != mGeometryCache.end();) {
    if (it->second.generation != mGeneration)
      it = mGeometryCache.erase(it);
    else
      ++it;
  }
  for (auto it = mVisitedFrames.begin(); it != mVisitedFrames.end();) {
    if (it->second != mGeneration)
      it = mVisitedFrames.erase(it);
    else
      ++it;
  }
  return mDescriptors;
}

RenderableSetUpdatePlan planRenderableSetUpdate(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<RenderableId>& activeRenderableIds)
{
  std::unordered_map<RenderableId, const RenderableDescriptor*> desired;
  for (const auto& descriptor : descriptors) {
    if (descriptor.material.visible)
      desired.emplace(descriptor.id, &descriptor);
  }
  std::vector<ActiveRenderableState> states;
  states.reserve(activeRenderableIds.size());
  for (const auto id : activeRenderableIds) {
    const auto it = desired.find(id);
    states.push_back(
        it == desired.end() ? ActiveRenderableState{id, 0, 0}
                            : ActiveRenderableState{
                                id,
                                it->second->shapeVersion,
                                it->second->renderResourceVersion});
  }
  return planRenderableSetUpdate(descriptors, states);
}

RenderableSetUpdatePlan planRenderableSetUpdate(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<ActiveRenderableState>& activeRenderableStates)
{
  RenderableSetUpdatePlan plan;
  std::unordered_map<RenderableId, const RenderableDescriptor*> desired;
  for (const auto& descriptor : descriptors) {
    if (descriptor.material.visible && descriptor.id != 0)
      desired.emplace(descriptor.id, &descriptor);
  }
  std::unordered_set<RenderableId> retained;
  for (std::size_t i = 0; i < activeRenderableStates.size(); ++i) {
    const auto& active = activeRenderableStates[i];
    const auto it = desired.find(active.id);
    const bool changed
        = it != desired.end()
          && (active.renderResourceVersion != 0
                      || it->second->renderResourceVersion != 0
                  ? active.renderResourceVersion
                        != it->second->renderResourceVersion
                  : active.shapeVersion != it->second->shapeVersion);
    if (it == desired.end() || changed || !retained.insert(active.id).second)
      plan.activeRenderableIndicesToRemove.push_back(i);
  }
  for (std::size_t i = 0; i < descriptors.size(); ++i) {
    const auto& descriptor = descriptors[i];
    if (descriptor.material.visible && descriptor.id != 0
        && retained.insert(descriptor.id).second)
      plan.descriptorIndicesToAdd.push_back(i);
  }
  return plan;
}

} // namespace dart::gui
