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

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <math/mat4.h>
#include <math/vec4.h>

#include <algorithm>
#include <iostream>

namespace dart::examples::filament_gui {
namespace {

using dart::gui::experimental::ActiveRenderableState;
using dart::gui::experimental::RenderableDescriptor;
using dart::gui::experimental::RenderableId;
using dart::gui::experimental::ShapeKind;
using dart::gui::experimental::planRenderableSetUpdate;
using filament::math::float4;
using filament::math::mat4f;

mat4f toFilamentTransform(const Eigen::Isometry3d& transform)
{
  const Eigen::Matrix4d matrix = transform.matrix();
  mat4f out;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      out[col][row] = static_cast<float>(matrix(row, col));
    }
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

bool containsRenderableId(
    const std::vector<RenderableId>& ids, RenderableId id)
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
    filament::Engine& engine,
    const Renderable& renderable,
    const Eigen::Isometry3d& transform)
{
  auto& transforms = engine.getTransformManager();
  transforms.setTransform(
      transforms.getInstance(renderable.entity), toFilamentTransform(transform));
}

void updateSceneRenderableFromDescriptor(
    filament::Engine& engine,
    SceneRenderable& sceneRenderable,
    const RenderableDescriptor& descriptor,
    bool selected)
{
  sceneRenderable.shapeVersion = descriptor.shapeVersion;
  sceneRenderable.renderResourceVersion = descriptor.renderResourceVersion;
  setRenderableTransform(engine, sceneRenderable.renderable, descriptor.worldTransform);
  updateRenderableSelection(
      sceneRenderable.renderable, toRgba(descriptor.material.rgba), selected);
  applyRenderableShadowSettings(
      engine, sceneRenderable.renderable, descriptor.material);
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
    filament::Engine& engine,
    filament::Scene& scene,
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
    scene.remove(sceneRenderable.renderable.entity);
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
    scene.addEntity(sceneRenderable.renderable.entity);
    setRenderableTransform(engine, sceneRenderable.renderable, descriptor.worldTransform);
    sceneRenderables.push_back(sceneRenderable);
  }
}

} // namespace dart::examples::filament_gui
