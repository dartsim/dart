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

#include <dart/gui/detail/application_teardown.hpp>
#include <dart/gui/detail/renderable_sync.hpp>

namespace dart::gui::detail {

void destroyApplicationResources(
    FilamentRenderContext& renderContext,
    ImGuiOverlay& imguiOverlay,
    SceneLights& lights,
    ::filament::IndirectLight* indirectLight,
    ::filament::Skybox* skybox,
    ::filament::ColorGrading* colorGrading,
    std::vector<SceneRenderable>& sceneRenderables,
    DebugOverlayController& debugOverlays,
    std::optional<Renderable>& selectionDebugOverlay,
    MaterialResources& materialResources)
{
  auto& engine = *renderContext.engine;
  auto& scene = *renderContext.scene;

  destroyConfiguredImGuiOverlay(engine, imguiOverlay);

  detachSceneEnvironment(scene, lights);
  for (auto* view : renderContext.views) {
    if (view != nullptr) {
      clearMainViewColorGrading(*view);
    }
  }
  for (const SceneRenderable& sceneRenderable : sceneRenderables) {
    removeRenderableFromScene(scene, sceneRenderable.renderable);
  }
  clearDebugOverlays(engine, scene, debugOverlays);
  clearDebugLineOverlay(engine, scene, selectionDebugOverlay);
  destroySceneLights(engine, lights);
  destroyRenderEnvironmentResources(
      engine, indirectLight, skybox, colorGrading);
  for (SceneRenderable& sceneRenderable : sceneRenderables) {
    destroyRenderable(engine, sceneRenderable.renderable);
  }
  destroyMaterialResources(engine, materialResources);
  destroyFilamentRenderContext(renderContext);
}

} // namespace dart::gui::detail
