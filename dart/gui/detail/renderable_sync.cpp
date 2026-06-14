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

#include "renderable_sync.hpp"

#include <dart/gui/viewer.hpp>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <math/mat4.h>
#include <math/vec4.h>

#include <algorithm>
#include <iostream>
#include <unordered_map>

#include <cmath>

namespace dart::gui::detail {
namespace {

using dart::gui::ActiveRenderableState;
using dart::gui::planRenderableSetUpdate;
using dart::gui::RenderableDescriptor;
using dart::gui::RenderableId;
using dart::gui::ShapeKind;
using ::filament::math::float4;
using ::filament::math::mat4f;

mat4f toFilamentTransform(const Eigen::Isometry3d& transform)
{
  const Eigen::Matrix4f matrix = transform.matrix().cast<float>();
  mat4f out;
  // filament mat4f and Eigen are both column-major; copy column by column.
  for (int col = 0; col < 4; ++col) {
    Eigen::Map<Eigen::Vector4f>(out[col].v) = matrix.col(col);
  }
  return out;
}

float4 toRgba(const Eigen::Vector4d& rgba)
{
  return {
      static_cast<float>(rgba.x()),
      static_cast<float>(rgba.y()),
      static_cast<float>(rgba.z()),
      static_cast<float>(rgba.w())};
}

std::optional<float4> makeRenderOutputOverrideColor(
    const RenderableDescriptor& descriptor,
    const dart::gui::RenderSettings& renderSettings,
    const dart::gui::OrbitCamera& camera,
    int viewportWidth,
    int viewportHeight)
{
  switch (renderSettings.outputMode) {
    case dart::gui::RenderOutputMode::Color:
      return std::nullopt;
    case dart::gui::RenderOutputMode::Depth: {
      const dart::gui::PerspectiveProjection projection
          = dart::gui::makePerspectiveProjection(
              camera, viewportWidth, viewportHeight);
      const dart::gui::OrbitCameraBasis cameraBasis
          = dart::gui::makeOrbitCameraBasis(camera);
      const double depth
          = (descriptor.worldTransform.translation() - cameraBasis.eye)
                .dot(cameraBasis.forward);
      const double safeDepth
          = std::isfinite(depth) ? depth : projection.farPlane;
      const double depthRange
          = std::max(1e-9, projection.farPlane - projection.nearPlane);
      const double normalizedDepth = std::clamp(
          (safeDepth - projection.nearPlane) / depthRange, 0.0, 1.0);
      const float shade
          = static_cast<float>(0.12 + 0.88 * (1.0 - normalizedDepth));
      const float alpha = static_cast<float>(
          std::clamp(descriptor.material.rgba.w(), 0.0, 1.0));
      return float4{shade, shade, shade, alpha};
    }
  }

  return std::nullopt;
}

bool containsRenderableId(const std::vector<RenderableId>& ids, RenderableId id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

std::vector<ActiveRenderableState> collectActiveRenderableStates(
    const std::vector<SceneRenderable>& sceneRenderables)
{
  std::vector<ActiveRenderableState> states;
  states.reserve(sceneRenderables.size());
  for (const SceneRenderable& sceneRenderable : sceneRenderables) {
    ActiveRenderableState state;
    state.id = sceneRenderable.id;
    state.shapeVersion = sceneRenderable.shapeVersion;
    state.renderResourceVersion = sceneRenderable.renderResourceVersion;
    states.push_back(state);
  }
  return states;
}

void logUnsupportedRenderableDescriptor(const RenderableDescriptor& descriptor)
{
  std::cerr << "DART shape";
  if (!descriptor.geometry.shapeType.empty()) {
    std::cerr << " '" << descriptor.geometry.shapeType << "'";
  }
  if (!descriptor.shapeFrameName.empty()) {
    std::cerr << " in shape frame '" << descriptor.shapeFrameName << "'";
  }
  if (!descriptor.skeletonName.empty()) {
    std::cerr << " on skeleton '" << descriptor.skeletonName << "'";
  }
  std::cerr << " will not be rendered";
  if (!descriptor.geometry.unsupportedReason.empty()) {
    std::cerr << ": " << descriptor.geometry.unsupportedReason;
  }
  std::cerr << "\n";
}

} // namespace

void setRenderableTransform(
    ::filament::Engine& engine,
    const Renderable& renderable,
    const Eigen::Isometry3d& transform)
{
  auto& transforms = engine.getTransformManager();
  transforms.setTransform(
      transforms.getInstance(renderable.entity),
      toFilamentTransform(transform));
}

void addRenderableToScene(
    ::filament::Scene& scene, const Renderable& renderable)
{
  scene.addEntity(renderable.entity);
}

void removeRenderableFromScene(
    ::filament::Scene& scene, const Renderable& renderable)
{
  scene.remove(renderable.entity);
}

void updateSceneRenderableFromDescriptor(
    ::filament::Engine& engine,
    SceneRenderable& sceneRenderable,
    const RenderableDescriptor& descriptor,
    const dart::gui::RenderSettings& renderSettings,
    const dart::gui::OrbitCamera& camera,
    int viewportWidth,
    int viewportHeight,
    bool selected)
{
  sceneRenderable.shapeVersion = descriptor.shapeVersion;
  sceneRenderable.renderResourceVersion = descriptor.renderResourceVersion;
  setRenderableTransform(
      engine, sceneRenderable.renderable, descriptor.worldTransform);
  updateRenderableSelection(
      sceneRenderable.renderable,
      toRgba(descriptor.material.rgba),
      selected,
      makeRenderOutputOverrideColor(
          descriptor, renderSettings, camera, viewportWidth, viewportHeight));
  applyRenderableShadowSettings(
      engine, sceneRenderable.renderable, descriptor.material);
}

bool updateSceneRenderablesFromDescriptors(
    ::filament::Engine& engine,
    const std::vector<RenderableDescriptor>& descriptors,
    std::vector<SceneRenderable>& sceneRenderables,
    RenderableId selectedRenderableId,
    const dart::gui::RenderSettings& renderSettings,
    const dart::gui::OrbitCamera& camera,
    int viewportWidth,
    int viewportHeight)
{
  bool selectedRenderableStillVisible = selectedRenderableId == 0;
  // Index descriptors by id once so the per-renderable lookup is O(1); matching
  // every scene renderable with std::find_if was O(N^2) at large scene scale.
  std::unordered_map<RenderableId, const RenderableDescriptor*> descriptorById;
  descriptorById.reserve(descriptors.size());
  for (const RenderableDescriptor& descriptor : descriptors) {
    descriptorById.emplace(descriptor.id, &descriptor);
  }

  for (SceneRenderable& sceneRenderable : sceneRenderables) {
    const auto found = descriptorById.find(sceneRenderable.id);
    if (found == descriptorById.end() || !found->second->material.visible) {
      continue;
    }
    const RenderableDescriptor& descriptor = *found->second;

    const bool isSelected = descriptor.id == selectedRenderableId;
    if (isSelected) {
      selectedRenderableStillVisible = true;
    }
    updateSceneRenderableFromDescriptor(
        engine,
        sceneRenderable,
        descriptor,
        renderSettings,
        camera,
        viewportWidth,
        viewportHeight,
        isSelected);
  }
  return selectedRenderableStillVisible;
}

void logUnsupportedRenderableDescriptorOnce(
    std::vector<RenderableId>& loggedUnsupportedRenderableIds,
    const RenderableDescriptor& descriptor)
{
  if (descriptor.id != 0
      && containsRenderableId(loggedUnsupportedRenderableIds, descriptor.id)) {
    return;
  }

  logUnsupportedRenderableDescriptor(descriptor);
  if (descriptor.id != 0) {
    loggedUnsupportedRenderableIds.push_back(descriptor.id);
  }
}

void synchronizeSceneRenderables(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    const std::vector<RenderableDescriptor>& descriptors,
    std::vector<SceneRenderable>& sceneRenderables,
    std::vector<RenderableId>& loggedUnsupportedRenderableIds,
    const RenderableFactory& createRenderable)
{
  const auto plan = planRenderableSetUpdate(
      descriptors, collectActiveRenderableStates(sceneRenderables));

  for (auto indexIt = plan.activeRenderableIndicesToRemove.rbegin();
       indexIt != plan.activeRenderableIndicesToRemove.rend();
       ++indexIt) {
    const std::size_t index = *indexIt;
    if (index >= sceneRenderables.size()) {
      continue;
    }

    SceneRenderable& sceneRenderable = sceneRenderables[index];
    removeRenderableFromScene(scene, sceneRenderable.renderable);
    destroyRenderable(engine, sceneRenderable.renderable);
    sceneRenderables.erase(sceneRenderables.begin() + index);
  }

  for (const std::size_t descriptorIndex : plan.descriptorIndicesToAdd) {
    if (descriptorIndex >= descriptors.size()) {
      continue;
    }

    const RenderableDescriptor& descriptor = descriptors[descriptorIndex];
    auto renderable = createRenderable(descriptor);
    if (!renderable) {
      if (descriptor.geometry.kind == ShapeKind::Unsupported
          || !descriptor.geometry.unsupportedReason.empty()) {
        logUnsupportedRenderableDescriptorOnce(
            loggedUnsupportedRenderableIds, descriptor);
      }
      continue;
    }

    SceneRenderable sceneRenderable;
    sceneRenderable.id = descriptor.id;
    sceneRenderable.shapeVersion = descriptor.shapeVersion;
    sceneRenderable.renderResourceVersion = descriptor.renderResourceVersion;
    sceneRenderable.renderable = *renderable;
    addRenderableToScene(scene, sceneRenderable.renderable);
    setRenderableTransform(
        engine, sceneRenderable.renderable, descriptor.worldTransform);
    sceneRenderables.push_back(sceneRenderable);
  }
}

} // namespace dart::gui::detail
