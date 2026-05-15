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

#include <dart/gui/experimental/detail/filament/scene_startup.hpp>

#include <dart/gui/experimental/detail/filament/debug_overlay.hpp>
#include <dart/gui/experimental/detail/filament/renderable_factory.hpp>
#include <dart/gui/experimental/detail/filament/renderable_sync.hpp>
#include <dart/gui/experimental/detail/filament/scene_requirements.hpp>
#include <dart/gui/experimental/detail/filament/scenes.hpp>
#include <dart/gui/experimental/scene.hpp>
#include <dart/simulation/world.hpp>

#include <ostream>

namespace dart::gui::experimental::filament {

std::optional<InitialSceneState> createInitialSceneState(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    const MaterialSet& materials,
    MaterialResources& materialResources,
    ExampleScene exampleScene,
    DartScene& dartScene,
    std::ostream& errors)
{
  const auto initialDescriptors
      = dart::gui::experimental::extractRenderables(*dartScene.world);
  const SceneContentCounts expectedSceneContent
      = countSceneContent(initialDescriptors);
  if (!validateSceneDescriptorContent(
          exampleScene, expectedSceneContent, errors)) {
    return std::nullopt;
  }

  InitialSceneState state;
  synchronizeSceneRenderables(
      engine,
      scene,
      initialDescriptors,
      state.sceneRenderables,
      state.loggedUnsupportedRenderableIds,
      [&](const dart::gui::experimental::RenderableDescriptor& descriptor) {
        return createRenderableFromDescriptor(
            engine, materials, materialResources.textureCache, descriptor);
      });
  if (state.sceneRenderables.empty()) {
    errors << "No supported visible DART renderables were extracted\n";
    return std::nullopt;
  }

  const SceneContentCounts createdSceneContent
      = countCreatedSceneContent(initialDescriptors, state.sceneRenderables);
  if (!validateCreatedSceneContent(
          exampleScene, expectedSceneContent, createdSceneContent, errors)) {
    return std::nullopt;
  }

  state.debugOverlays = makeDebugOverlayController(exampleScene == ExampleScene::G1);
  refreshStaticDebugOverlay(
      engine, scene, materials.debugColor, *dartScene.world, state.debugOverlays);
  if (!state.debugOverlays.staticOverlay) {
    errors << "No debug overlay lines were extracted\n";
    return std::nullopt;
  }

  return state;
}

} // namespace dart::gui::experimental::filament
