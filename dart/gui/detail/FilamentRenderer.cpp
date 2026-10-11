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

#include "FilamentRenderer.hpp"

#include "depth_material.hpp"
#include "geometry.hpp"
#include "imgui_overlay.hpp"
#include "render_context.hpp"
#include "render_environment.hpp"
#include "renderable_factory.hpp"
#include "renderable_resources.hpp"
#include "renderable_sync.hpp"
#include "screenshot.hpp"

#include <dart/gui/detail/viewer_types.hpp>

#include <backend/BufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <cmath>

namespace dart::gui::detail {
namespace {
// OSG's displayed DEPTH mode fits its perspective clips to visible primitives.
// Clip triangles, rather than their bounds, so a floor crossing the eye and
// frustum does not collapse the useful depth range.
struct VisibleDepthRange
{
  Eigen::Vector3d eye;
  Eigen::Matrix3d worldToCamera;
  std::array<Eigen::Vector3d, 5> planes;
  double nearDepth = std::numeric_limits<double>::infinity();
  double farDepth = 0;

  VisibleDepthRange(
      const Eigen::Vector3d& eye_,
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& up,
      double fovy,
      double aspect)
    : eye(eye_)
  {
    const Eigen::Vector3d forward = (center - eye).normalized();
    const Eigen::Vector3d right = forward.cross(up).normalized();
    worldToCamera.row(0) = right.transpose();
    worldToCamera.row(1) = right.cross(forward).transpose();
    worldToCamera.row(2) = forward.transpose();
    const double tanY = std::tan(fovy * 3.14159265358979323846 / 360);
    const double tanX = tanY * aspect;
    planes
        = {Eigen::Vector3d(1, 0, tanX),
           Eigen::Vector3d(-1, 0, tanX),
           Eigen::Vector3d(0, 1, tanY),
           Eigen::Vector3d(0, -1, tanY),
           Eigen::Vector3d(0, 0, 1)};
  }

  void add(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c)
  {
    if (!a.allFinite() || !b.allFinite() || !c.allFinite())
      return;
    // A triangle clipped by five planes has at most eight corners.
    std::array<Eigen::Vector3d, 8> polygon, clipped;
    polygon[0] = worldToCamera * (a - eye);
    polygon[1] = worldToCamera * (b - eye);
    polygon[2] = worldToCamera * (c - eye);
    std::size_t count = 3;
    for (const auto& plane : planes) {
      std::size_t output = 0;
      for (std::size_t i = 0; i < count; ++i) {
        const auto& from = polygon[i];
        const auto& to = polygon[(i + 1) % count];
        const double fromDistance = plane.dot(from);
        const double toDistance = plane.dot(to);
        if ((fromDistance >= 0) != (toDistance >= 0))
          clipped[output++]
              = from
                + (to - from) * (fromDistance / (fromDistance - toDistance));
        if (toDistance >= 0)
          clipped[output++] = to;
      }
      if (output == 0)
        return;
      for (std::size_t i = 0; i < output; ++i)
        polygon[i] = clipped[i];
      count = output;
    }
    for (std::size_t i = 0; i < count; ++i) {
      nearDepth = std::min(nearDepth, polygon[i].z());
      farDepth = std::max(farDepth, polygon[i].z());
    }
  }
};

void addDescriptorDepthRange(
    VisibleDepthRange& range,
    const RenderableDescriptor& descriptor,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up,
    double fovy,
    int viewportHeight)
{
  const auto add = [&](const auto& a, const auto& b, const auto& c) {
    range.add(
        descriptor.worldTransform * a.template cast<double>(),
        descriptor.worldTransform * b.template cast<double>(),
        descriptor.worldTransform * c.template cast<double>());
  };
  const auto addMesh = [&](const MeshGeometry& mesh) {
    for (const auto& triangle : mesh.triangles)
      add(mesh.vertices[triangle.a].position,
          mesh.vertices[triangle.b].position,
          mesh.vertices[triangle.c].position);
  };
  const auto addTriangles = [&] {
    const auto& geometry = descriptor.geometry;
    for (const auto& indices : geometry.triangleIndices) {
      if ((indices.array() < 0).any()
          || static_cast<std::size_t>(indices.maxCoeff())
                 >= geometry.triangleVertices.size())
        continue;
      add(geometry.triangleVertices[indices.x()],
          geometry.triangleVertices[indices.y()],
          geometry.triangleVertices[indices.z()]);
    }
  };
  const auto addVisualTriangles = [&](const auto& triangles) {
    for (const auto& triangle : triangles)
      add(triangle.a, triangle.b, triangle.c);
  };
  const auto& geometry = descriptor.geometry;
  switch (geometry.kind) {
    case ShapeKind::Box:
      addMesh(makeBoxMeshGeometry(geometry.size));
      break;
    case ShapeKind::Sphere:
      addMesh(makeEllipsoidMeshGeometry(
          Eigen::Vector3d::Constant(geometry.radius)));
      break;
    case ShapeKind::Ellipsoid:
      addMesh(makeEllipsoidMeshGeometry(geometry.size * .5));
      break;
    case ShapeKind::Cylinder:
      addMesh(makeCylinderMeshGeometry(geometry.radius, geometry.height));
      break;
    case ShapeKind::Cone:
      addMesh(makeConeMeshGeometry(geometry.radius, geometry.height));
      break;
    case ShapeKind::Pyramid:
      addMesh(makePyramidMeshGeometry(geometry.size));
      break;
    case ShapeKind::Capsule:
      addMesh(makeCapsuleMeshGeometry(geometry.radius, geometry.height));
      break;
    case ShapeKind::MultiSphere:
      if (geometry.triangleIndices.empty())
        addMesh(makeMultiSphereMeshGeometry(
            geometry.sphereCenters, geometry.sphereRadii));
      else
        addTriangles();
      break;
    case ShapeKind::ConvexMesh:
    case ShapeKind::Heightmap:
    case ShapeKind::SoftMesh:
      addTriangles();
      break;
    case ShapeKind::Mesh: {
      addTriangles();
      if (geometry.meshParts.empty()) {
        MeshPartDescriptor part;
        part.pointCount = geometry.meshPointIndices.size();
        part.lineCount = geometry.meshLineIndices.size();
        addMesh(makeMeshPointLineGeometry(
            descriptor, part, range.eye, center, up, fovy, viewportHeight));
      } else {
        for (const auto& part : geometry.meshParts)
          if (part.pointCount || part.lineCount)
            addMesh(makeMeshPointLineGeometry(
                descriptor, part, range.eye, center, up, fovy, viewportHeight));
      }
      break;
    }
    case ShapeKind::PointCloud:
      if (geometry.pointShapeType == PointShapeType::Box) {
        const double size = std::max(geometry.pointSize, 1e-4);
        for (const auto& point : geometry.pointCloudPoints) {
          MeshGeometry mesh;
          appendBoxMeshGeometry(mesh, point, Eigen::Vector3d::Constant(size));
          addMesh(mesh);
        }
      } else {
        addVisualTriangles(makePointCloudVisualTriangles(
            descriptor, range.eye, center, up, fovy, viewportHeight));
      }
      break;
    case ShapeKind::LineSegments:
      addVisualTriangles(makeLineSegmentVisualTriangles(
          descriptor, range.eye, center, fovy, viewportHeight));
      break;
    case ShapeKind::VoxelGrid:
      for (std::size_t i = 0; i < geometry.voxelCenters.size(); ++i) {
        const double size
            = geometry.voxelSizes.size() == geometry.voxelCenters.size()
                  ? geometry.voxelSizes[i]
                  : std::max(geometry.voxelSize, 1e-4);
        MeshGeometry mesh;
        appendBoxMeshGeometry(
            mesh, geometry.voxelCenters[i], Eigen::Vector3d::Constant(size));
        addMesh(mesh);
      }
      break;
    case ShapeKind::Plane: {
      const Eigen::Vector3d normal = geometry.normal.squaredNorm() < 1e-12
                                         ? Eigen::Vector3d::UnitZ().eval()
                                         : geometry.normal.normalized().eval();
      const Eigen::Vector3d seed = std::abs(normal.z()) < .9
                                       ? Eigen::Vector3d::UnitZ().eval()
                                       : Eigen::Vector3d::UnitX().eval();
      const Eigen::Vector3d u = seed.cross(normal).normalized();
      const Eigen::Vector3d v = normal.cross(u).normalized();
      const Eigen::Vector3d origin = normal * geometry.offset;
      add(origin - u - v, origin + u - v, origin + u + v);
      add(origin - u - v, origin + u + v, origin - u + v);
      break;
    }
    case ShapeKind::Unsupported:
      break;
  }
}
} // namespace

class FilamentRenderer::Impl
{
public:
  Impl(int width_, int height_, bool headless_, void* nativeWindow)
    : width(width_),
      height(height_),
      headless(headless_),
      context(
          createFilamentRenderContext(width, height, headless, nativeWindow))
  {
    try {
      colorGrading = createDebugColorGrading(*context.engine);
      configureMainView(*context.view, colorGrading);
      // Render and capture share the same quality settings, independent of the
      // device.
      // DART clear colors are display values. A skybox or an opaque View sends
      // them through the scene tone mapper; a translucent View preserves the
      // Renderer's native clear color in untouched pixels (SDK 1.77.3).
      context.view->setBlendMode(::filament::View::BlendMode::TRANSLUCENT);
      ::filament::Renderer::ClearOptions clear;
      clear.clear = true;
      clear.clearColor = backgroundColor;
      context.renderer->setClearOptions(clear);
      indirectLight = createNeutralIndirectLight(*context.engine);
      materials = createMaterialResources(*context.engine);
      if (!materials.defaultLit || !materials.texturedLit
          || !materials.transparentLit || !materials.transparentTexturedLit
          || !materials.debugColor || !materials.pointColor
          || !materials.opaquePointColor || !materials.legacyLit
          || !materials.legacyTexturedLit || !materials.legacyTransparentLit
          || !materials.legacyTransparentTexturedLit)
        throw std::runtime_error(
            "Filament material package could not be loaded");
      lights = createSceneLights(*context.engine, false, false, 1.0);
      attachSceneEnvironment(*context.scene, indirectLight, nullptr, lights);
      depthMaterial = ::filament::Material::Builder()
                          .package(kDepthMaterial, kDepthMaterialSize)
                          .build(*context.engine);
      if (depthMaterial == nullptr)
        throw std::runtime_error("Filament depth material could not be loaded");
      depthInstance = depthMaterial->createInstance();
      depthTwoSidedInstance = depthMaterial->createInstance();
      depthNoOutputInstance = depthMaterial->createInstance();
      depthInstance->setCullingMode(
          ::filament::MaterialInstance::CullingMode::BACK,
          ::filament::MaterialInstance::CullingMode::NONE);
      depthNoOutputInstance->setDepthWrite(false);
      depthNoOutputInstance->setColorWrite(false);
      setCamera(
          {3.0, 3.0, 2.0},
          {0.0, 0.0, 0.0},
          {0.0, 0.0, 1.0},
          45.0,
          0.01,
          1000.0);
      std::cerr << "[dart::gui] Filament backend=" << context.backendName
                << " device=" << context.deviceName << '\n';
    } catch (...) {
      destroy();
      throw;
    }
  }
  ~Impl()
  {
    destroy();
  }
  void destroy()
  {
    if (!context.engine)
      return;
    context.engine->flushAndWait();
    context.engine->pumpMessageQueues();
    if (overlay.view)
      destroyImGuiOverlay(*context.engine, overlay);
    for (auto& debugRenderable : debugRenderables) {
      if (debugRenderable) {
        removeRenderableFromScene(*context.scene, *debugRenderable);
        destroyRenderable(*context.engine, *debugRenderable);
        debugRenderable.reset();
      }
    }
    for (auto& renderable : renderables) {
      removeRenderableFromScene(*context.scene, renderable.renderable);
      destroyRenderable(*context.engine, renderable.renderable);
    }
    renderables.clear();
    detachSceneEnvironment(*context.scene, lights);
    destroySceneLights(*context.engine, lights);
    clearMainViewColorGrading(*context.view);
    destroyRenderEnvironmentResources(
        *context.engine, indirectLight, nullptr, colorGrading);
    destroyMaterialResources(*context.engine, materials);
    if (depthInstance)
      context.engine->destroy(depthInstance);
    if (depthTwoSidedInstance)
      context.engine->destroy(depthTwoSidedInstance);
    if (depthNoOutputInstance)
      context.engine->destroy(depthNoOutputInstance);
    if (depthMaterial)
      context.engine->destroy(depthMaterial);
    destroyFilamentRenderContext(context);
  }
  void setCamera(
      const Eigen::Vector3d& eye,
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& up,
      double fovy,
      double nearPlane,
      double farPlane)
  {
    if (!eye.allFinite() || !center.allFinite() || !up.allFinite()
        || (eye - center).squaredNorm() <= 1e-20 || up.squaredNorm() <= 1e-20
        || (eye - center).cross(up).squaredNorm() <= 1e-20
        || !std::isfinite(fovy) || fovy <= 0 || fovy >= 180
        || !std::isfinite(nearPlane) || !std::isfinite(farPlane)
        || nearPlane <= 0 || farPlane <= nearPlane)
      throw std::invalid_argument("Invalid Filament camera configuration");
    context.view->setViewport(
        {0,
         0,
         static_cast<std::uint32_t>(width),
         static_cast<std::uint32_t>(height)});
    context.camera->setProjection(
        fovy, static_cast<double>(width) / height, nearPlane, farPlane);
    context.camera->lookAt(
        {eye.x(), eye.y(), eye.z()},
        {center.x(), center.y(), center.z()},
        {up.x(), up.y(), up.z()});
    camera.target = center;
    camera.up = up.normalized();
    camera.distance = (eye - center).norm();
    const Eigen::Vector3d direction = (eye - center).normalized();
    camera.yaw = std::atan2(direction.y(), direction.x());
    camera.pitch = std::asin(std::clamp(direction.z(), -1.0, 1.0));
    cameraEye = eye;
    cameraCenter = center;
    cameraUp = up;
    cameraFovy = fovy;
    cameraNear = nearPlane;
    cameraFar = farPlane;
    const Eigen::Vector3d headlightDirection = (center - eye).normalized();
    updateCameraHeadlight(
        *context.engine,
        lights,
        {static_cast<float>(headlightDirection.x()),
         static_cast<float>(headlightDirection.y()),
         static_cast<float>(headlightDirection.z())});
  }
  void synchronize(const std::vector<RenderableDescriptor>& descriptors)
  {
    const auto materialSet = materials.materialSet();
    synchronizeSceneRenderables(
        *context.engine,
        *context.scene,
        descriptors,
        renderables,
        unsupported,
        [&](const RenderableDescriptor& descriptor) {
          return createRenderableFromDescriptor(
              *context.engine, materialSet, materials.textureCache, descriptor);
        });
    if (!materials.textureCache.bindings.empty()) {
      std::unordered_set<::filament::Texture*> activeTextures;
      activeTextures.reserve(materials.textureCache.bindings.size());
      for (const auto& entry : renderables)
        activeTextures.insert(
            entry.renderable.sampledTextures.begin(),
            entry.renderable.sampledTextures.end());
      releaseUnusedTextures(
          *context.engine, materials.textureCache, activeTextures);
    }
    updateSceneRenderablesFromDescriptors(
        *context.engine,
        descriptors,
        renderables,
        0,
        settings,
        camera,
        width,
        height);
    renderableEntities.reserve(renderables.size());
    for (const auto& entry : renderables)
      renderableEntities.insert_or_assign(entry.id, entry.renderable.entity);
    if (renderableEntities.size() > renderables.size()) {
      std::unordered_set<RenderableId> active;
      for (const auto& entry : renderables)
        active.insert(entry.id);
      for (auto it = renderableEntities.begin();
           it != renderableEntities.end();) {
        if (active.count(it->first) == 0)
          it = renderableEntities.erase(it);
        else
          ++it;
      }
    }
    const bool hasBillboards = std::any_of(
        descriptors.begin(), descriptors.end(), [](const auto& descriptor) {
          return (descriptor.geometry.kind == ShapeKind::PointCloud
                  && descriptor.geometry.pointShapeType != PointShapeType::Box)
                 || descriptor.geometry.kind == ShapeKind::LineSegments
                 || !descriptor.geometry.meshPointIndices.empty()
                 || !descriptor.geometry.meshLineIndices.empty();
        });
    if (hasBillboards) {
      std::unordered_map<RenderableId, Renderable*> byId;
      byId.reserve(renderables.size());
      for (auto& entry : renderables)
        byId.emplace(entry.id, &entry.renderable);
      for (const auto& descriptor : descriptors) {
        const bool billboard
            = descriptor.geometry.kind == ShapeKind::PointCloud
              && descriptor.geometry.pointShapeType != PointShapeType::Box;
        const bool thickLine
            = descriptor.geometry.kind == ShapeKind::LineSegments
              || !descriptor.geometry.meshPointIndices.empty()
              || !descriptor.geometry.meshLineIndices.empty();
        if (!billboard && !thickLine)
          continue;
        const auto it = byId.find(descriptor.id);
        if (it != byId.end())
          updateScreenSpaceGeometry(
              *context.engine,
              *it->second,
              descriptor,
              cameraEye,
              cameraCenter,
              cameraUp,
              cameraFovy,
              height);
      }
    }
    if (!wireframeOverrides.empty()) {
      std::unordered_set<RenderableId> activeIds;
      activeIds.reserve(renderables.size());
      for (const auto& entry : renderables)
        activeIds.insert(entry.id);
      for (auto it = wireframeOverrides.begin();
           it != wireframeOverrides.end();) {
        if (activeIds.count(*it) == 0)
          it = wireframeOverrides.erase(it);
        else
          ++it;
      }
    }
    updateWireframeGeometry();
    updateDepthMaterials();
    if (depthMode)
      updateDepthProjection(descriptors);
  }
  void updateDepthMaterials()
  {
    auto& manager = context.engine->getRenderableManager();
    for (auto& entry : renderables) {
      auto& renderable = entry.renderable;
      if (renderable.depthApplied == depthMode)
        continue;
      renderable.depthApplied = depthMode;
      const auto instance = manager.getInstance(renderable.entity);
      if (renderable.originalPrimitiveMaterials.empty()) {
        for (std::size_t i = 0; i < manager.getPrimitiveCount(instance); ++i)
          renderable.originalPrimitiveMaterials.push_back(
              manager.getMaterialInstanceAt(instance, i));
      }
      for (std::size_t i = 0; i < renderable.originalPrimitiveMaterials.size();
           ++i) {
        auto* original = renderable.originalPrimitiveMaterials[i];
        // Preserve each primitive's color culling when replacing its shader.
        // In particular, SoftMesh retains the original two-sided policy.
        // OSG displays its depth texture, so an original material that does
        // not write depth must not paint the direct depth-color output either.
        auto* depth = !original->isDepthWriteEnabled() ? depthNoOutputInstance
                      : original->getCullingMode()
                              == ::filament::MaterialInstance::CullingMode::NONE
                          ? depthTwoSidedInstance
                          : depthInstance;
        manager.setMaterialInstanceAt(
            instance, i, depthMode ? depth : original);
      }
    }
    context.view->setPostProcessingEnabled(!depthMode);
  }
  void updateDepthProjection(
      const std::vector<RenderableDescriptor>& descriptors)
  {
    VisibleDepthRange range(
        cameraEye,
        cameraCenter,
        cameraUp,
        cameraFovy,
        static_cast<double>(width) / height);
    for (const auto& descriptor : descriptors)
      if (descriptor.material.visible
          && renderableEntities.count(descriptor.id) != 0)
        addDescriptorDepthRange(
            range, descriptor, cameraCenter, cameraUp, cameraFovy, height);
    double nearDepth = cameraNear, farDepth = cameraFar;
    if (std::isfinite(range.nearDepth) && range.farDepth > 0) {
      // osg::clampProjectionMatrix widens degenerate ranges, adds a two
      // percent margin and retains its default near/far ratio of .0005.
      if (range.farDepth < range.nearDepth + 1e-6) {
        const double average = (range.nearDepth + range.farDepth) * .5;
        range.nearDepth = average - 1e-6;
        range.farDepth = average + 1e-6;
      }
      nearDepth = std::max(range.nearDepth * .98, range.farDepth * .0005);
      farDepth = range.farDepth * 1.02;
    }
    context.camera->setProjection(
        cameraFovy, static_cast<double>(width) / height, nearDepth, farDepth);
    // SDK setProjection renders with an infinite far plane. Its finite GL
    // culling matrix, used as a supported custom projection, preserves the
    // retained OSG nonlinear displayed depth range on every backend.
    const auto projection = context.camera->getCullingProjectionMatrix();
    context.camera->setCustomProjection(projection, nearDepth, farDepth);
  }
  void updateClearColor()
  {
    auto clear = context.renderer->getClearOptions();
    // Retained OSG depth output clears to far depth (white), independently of
    // the authored color background. Restore its latest value in COLOR mode.
    clear.clearColor
        = depthMode ? ::filament::math::float4{1, 1, 1, 1} : backgroundColor;
    context.renderer->setClearOptions(clear);
  }
  void updateWireframeGeometry()
  {
    auto& manager = context.engine->getRenderableManager();
    for (auto& entry : renderables) {
      const bool wireframe
          = entry.renderable.wireframeEligible
            && (this->wireframe || wireframeOverrides.count(entry.id) != 0);
      auto& renderable = entry.renderable;
      if (renderable.wireframeApplied == wireframe)
        continue;
      renderable.wireframeApplied = wireframe;
      const auto instance = manager.getInstance(renderable.entity);
      if (wireframe && !renderable.wireframeIndexBuffer) {
        std::vector<std::uint32_t> indices;
        for (const auto& range : renderable.geometryRanges) {
          Renderable::GeometryRange wireRange{indices.size(), 0, false};
          if (range.triangles && range.wireframeEligible) {
            for (std::size_t i = range.offset; i < range.offset + range.count;
                 i += 3) {
              const auto a = renderable.cpuIndices[i],
                         b = renderable.cpuIndices[i + 1],
                         c = renderable.cpuIndices[i + 2];
              indices.insert(indices.end(), {a, b, b, c, c, a});
            }
          } else {
            wireRange.triangles = range.triangles;
            indices.insert(
                indices.end(),
                renderable.cpuIndices.begin() + range.offset,
                renderable.cpuIndices.begin() + range.offset + range.count);
          }
          wireRange.count = indices.size() - wireRange.offset;
          renderable.wireframeRanges.push_back(wireRange);
        }
        if (indices.empty())
          continue;
        renderable.wireframeIndexBuffer
            = ::filament::IndexBuffer::Builder()
                  .indexCount(indices.size())
                  .bufferType(::filament::IndexBuffer::IndexType::UINT)
                  .build(*context.engine);
        auto* owned = new std::vector<std::uint32_t>(std::move(indices));
        renderable.wireframeIndexBuffer->setBuffer(
            *context.engine,
            ::filament::backend::BufferDescriptor(
                owned->data(),
                owned->size() * sizeof(std::uint32_t),
                [](void*, std::size_t, void* user) {
                  delete static_cast<std::vector<std::uint32_t>*>(user);
                },
                owned));
      }
      const auto& ranges
          = wireframe ? renderable.wireframeRanges : renderable.geometryRanges;
      for (std::size_t i = 0; i < ranges.size(); ++i) {
        const auto& range = ranges[i];
        manager.setGeometryAt(
            instance,
            i,
            range.triangles
                ? ::filament::RenderableManager::PrimitiveType::TRIANGLES
                : ::filament::RenderableManager::PrimitiveType::LINES,
            renderable.vertexBuffer,
            wireframe ? renderable.wireframeIndexBuffer
                      : renderable.indexBuffer,
            range.offset,
            range.count);
      }
    }
  }
  bool render(bool capture)
  {
    const bool ready = context.renderer->beginFrame(context.swapChain);
    if (!ready && !headless)
      return false;
    context.renderer->render(context.view);
    if (overlay.view && overlay.mesh.entity
        && context.engine->getRenderableManager().hasComponent(
            overlay.mesh.entity))
      context.renderer->render(overlay.view);
    if (capture)
      requestScreenshot(context, screenshot, width, height);
    context.renderer->endFrame();
    const auto projection = context.camera->getProjectionMatrix();
    lastDepthProjection = Eigen::Vector4d(
        projection[2][2], projection[3][2], projection[2][3], projection[3][3]);
    if (headless)
      context.engine->flushAndWait();
    return true;
  }
  int width, height;
  bool headless;
  FilamentRenderContext context;
  ::filament::ColorGrading* colorGrading = nullptr;
  ::filament::IndirectLight* indirectLight = nullptr;
  SceneLights lights;
  MaterialResources materials;
  std::vector<SceneRenderable> renderables;
  std::unordered_map<RenderableId, utils::Entity> renderableEntities;
  std::vector<RenderableId> unsupported;
  RenderSettings settings;
  OrbitCamera camera;
  Eigen::Vector3d cameraEye, cameraCenter, cameraUp;
  double cameraFovy = 45, cameraNear = 0.01, cameraFar = 1000;
  std::optional<Eigen::Vector4d> lastDepthProjection;
  ScreenshotCapture screenshot;
  ImGuiOverlay overlay;
  std::array<std::optional<Renderable>, 2> debugRenderables;
  DebugScene cachedDebugScene;
  ::filament::Material* depthMaterial = nullptr;
  ::filament::MaterialInstance* depthInstance = nullptr;
  ::filament::MaterialInstance* depthTwoSidedInstance = nullptr;
  ::filament::MaterialInstance* depthNoOutputInstance = nullptr;
  ::filament::math::float4 backgroundColor = {.30, .33, .38, 1.0};
  bool depthMode = false;
  bool wireframe = false;
  std::unordered_set<RenderableId> wireframeOverrides;
  bool headlightsEnabled = true;
  Eigen::Vector3d lightingUp = Eigen::Vector3d::UnitZ();
};

FilamentRenderer::FilamentRenderer(
    int width, int height, bool headless, void* nativeWindow)
  : mImpl(std::make_unique<Impl>(width, height, headless, nativeWindow))
{
}
FilamentRenderer::~FilamentRenderer() = default;
void FilamentRenderer::synchronize(
    const std::vector<RenderableDescriptor>& descriptors)
{
  mImpl->synchronize(descriptors);
}
void FilamentRenderer::setCamera(
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up,
    double fovy,
    double nearPlane,
    double farPlane)
{
  mImpl->setCamera(eye, center, up, fovy, nearPlane, farPlane);
}
void FilamentRenderer::resize(int width, int height)
{
  if (width <= 0 || height <= 0)
    throw std::invalid_argument(
        "Filament requires positive framebuffer dimensions");
  if (width == mImpl->width && height == mImpl->height)
    return;
  mImpl->context.engine->flushAndWait();
  if (mImpl->headless) {
    mImpl->context.engine->destroy(mImpl->context.swapChain);
    mImpl->context.swapChain = mImpl->context.engine->createSwapChain(
        static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height));
  }
  mImpl->width = width;
  mImpl->height = height;
  mImpl->setCamera(
      mImpl->cameraEye,
      mImpl->cameraCenter,
      mImpl->cameraUp,
      mImpl->cameraFovy,
      mImpl->cameraNear,
      mImpl->cameraFar);
}
bool FilamentRenderer::frame(bool captureRGBA)
{
  return mImpl->render(captureRGBA);
}
std::optional<Eigen::Vector4d> FilamentRenderer::getLastDepthProjection() const
{
  return mImpl->lastDepthProjection;
}
std::vector<std::uint8_t> FilamentRenderer::consumeCapturedPixelsRGBA()
{
  {
    std::lock_guard<std::mutex> lock(mImpl->screenshot.mutex);
    if (mImpl->screenshot.pixels.empty()
        || (!mImpl->screenshot.done && !mImpl->screenshot.inFlight))
      throw std::runtime_error("No Filament capture was queued");
  }
  if (!waitForScreenshot(mImpl->context, mImpl->screenshot))
    throw std::runtime_error("Timed out waiting for Filament readback");
  // Filament accepts bottom-left read regions but writes PixelBufferDescriptor
  // rows in top-left order on every backend. Preserve that orientation.
  std::lock_guard<std::mutex> lock(mImpl->screenshot.mutex);
  mImpl->screenshot.done = false;
  return std::move(mImpl->screenshot.pixels);
}
std::vector<std::uint8_t> FilamentRenderer::readPixelsRGBA()
{
  // Headless frame() already completes the frame. Native capture retains the
  // preflush that makes a standalone beginFrame available after presentation.
  if (!mImpl->headless)
    mImpl->context.engine->flushAndWait();
  if (!mImpl->render(true))
    throw std::runtime_error("No Filament frame was available for capture");
  return consumeCapturedPixelsRGBA();
}
void FilamentRenderer::renderImGui(const ImDrawData* drawData)
{
  if (!mImpl->overlay.view)
    mImpl->overlay = createImGuiOverlay(*mImpl->context.engine);
  updateImGuiOverlay(
      *mImpl->context.engine,
      mImpl->overlay,
      drawData,
      mImpl->width,
      mImpl->height);
}
void FilamentRenderer::setShadowsEnabled(bool enabled)
{
  if (mImpl->settings.shadowsEnabled == enabled)
    return;
  mImpl->settings.shadowsEnabled = enabled;
  applyRenderSettings(*mImpl->context.view, mImpl->settings);
}
void FilamentRenderer::setHeadlightsEnabled(bool enabled)
{
  if (mImpl->headlightsEnabled == enabled)
    return;
  mImpl->headlightsEnabled = enabled;
  setCameraHeadlightEnabled(*mImpl->context.engine, mImpl->lights, enabled);
}

void FilamentRenderer::setShadowsEnabled(
    RenderableId id, bool casts, bool receives)
{
  const auto found = mImpl->renderableEntities.find(id);
  if (found == mImpl->renderableEntities.end())
    return;
  auto& manager = mImpl->context.engine->getRenderableManager();
  const auto instance = manager.getInstance(found->second);
  manager.setCastShadows(instance, casts);
  manager.setReceiveShadows(instance, receives);
  manager.setScreenSpaceContactShadows(instance, casts || receives);
}

void FilamentRenderer::setLightingUp(const Eigen::Vector3d& up)
{
  const Eigen::Vector3d normalized = up.allFinite() && up.squaredNorm() > 1e-12
                                         ? up.normalized().eval()
                                         : Eigen::Vector3d::UnitZ().eval();
  if ((normalized - mImpl->lightingUp).squaredNorm() < 1e-24)
    return;
  mImpl->lightingUp = normalized;
  rotateSceneLights(*mImpl->context.engine, mImpl->lights, normalized);
}
void FilamentRenderer::setBackgroundColor(const Eigen::Vector4d& color)
{
  if (!color.allFinite())
    throw std::invalid_argument("Filament background color must be finite");
  const auto rgba = color.cwiseMax(0).cwiseMin(1);
  mImpl->backgroundColor = {rgba[0], rgba[1], rgba[2], rgba[3]};
  mImpl->updateClearColor();
}
std::size_t FilamentRenderer::getTextureCount() const
{
  return mImpl->context.engine->getTextureCount();
}
void FilamentRenderer::setDepthMode(bool enabled)
{
  if (mImpl->depthMode == enabled)
    return;
  mImpl->depthMode = enabled;
  mImpl->updateDepthMaterials();
  mImpl->updateClearColor();
  if (!enabled)
    mImpl->context.camera->setProjection(
        mImpl->cameraFovy,
        static_cast<double>(mImpl->width) / mImpl->height,
        mImpl->cameraNear,
        mImpl->cameraFar);
}
void FilamentRenderer::setWireframe(bool enabled)
{
  if (mImpl->wireframe == enabled)
    return;
  mImpl->wireframe = enabled;
  mImpl->updateWireframeGeometry();
}
void FilamentRenderer::setWireframe(RenderableId id, bool enabled)
{
  const bool changed = enabled ? mImpl->wireframeOverrides.insert(id).second
                               : mImpl->wireframeOverrides.erase(id) != 0;
  if (changed)
    mImpl->updateWireframeGeometry();
}
void FilamentRenderer::setDebugScene(const DebugScene& debug)
{
  const auto& cached = mImpl->cachedDebugScene;
  bool same = cached.lines.size() == debug.lines.size()
              && cached.triangles.size() == debug.triangles.size();
  for (std::size_t i = 0; same && i < debug.lines.size(); ++i) {
    const auto& a = cached.lines[i];
    const auto& b = debug.lines[i];
    same = (a.from.array() == b.from.array()).all()
           && (a.to.array() == b.to.array()).all()
           && (a.rgba.array() == b.rgba.array()).all()
           && a.thickness == b.thickness && a.pixelWidth == b.pixelWidth
           && a.alwaysOnTop == b.alwaysOnTop;
  }
  for (std::size_t i = 0; same && i < debug.triangles.size(); ++i) {
    const auto& a = cached.triangles[i];
    const auto& b = debug.triangles[i];
    same = (a.a.array() == b.a.array()).all()
           && (a.b.array() == b.b.array()).all()
           && (a.c.array() == b.c.array()).all()
           && (a.rgba.array() == b.rgba.array()).all()
           && a.alwaysOnTop == b.alwaysOnTop;
  }
  if (same)
    return;
  std::array<DebugScene, 2> layers;
  for (const auto& line : debug.lines) {
    // Pixel-width overlay lines are drawn by the owned ImGui overlay adapter.
    if (line.pixelWidth > 0)
      continue;
    layers[line.alwaysOnTop ? 1 : 0].lines.push_back(line);
  }
  for (const auto& triangle : debug.triangles)
    layers[triangle.alwaysOnTop ? 1 : 0].triangles.push_back(triangle);
  for (std::size_t i = 0; i < layers.size(); ++i) {
    auto& renderable = mImpl->debugRenderables[i];
    if (renderable) {
      removeRenderableFromScene(*mImpl->context.scene, *renderable);
      destroyRenderable(*mImpl->context.engine, *renderable);
      renderable.reset();
    }
    auto* material
        = i == 0 ? mImpl->materials.pointColor : mImpl->materials.debugColor;
    renderable = createDebugLineRenderable(
        *mImpl->context.engine,
        *material,
        layers[i].lines,
        layers[i].triangles);
    if (renderable)
      addRenderableToScene(*mImpl->context.scene, *renderable);
  }
  mImpl->cachedDebugScene.lines = debug.lines;
  mImpl->cachedDebugScene.triangles = debug.triangles;
}
const std::string& FilamentRenderer::getBackendName() const
{
  return mImpl->context.backendName;
}
const std::string& FilamentRenderer::getDeviceName() const
{
  return mImpl->context.deviceName;
}
} // namespace dart::gui::detail
