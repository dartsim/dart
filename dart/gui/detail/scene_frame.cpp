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

#include <dart/gui/detail/debug_overlay.hpp>
#include <dart/gui/detail/frame_viewport.hpp>
#include <dart/gui/detail/render_environment.hpp>
#include <dart/gui/detail/renderable_factory.hpp>
#include <dart/gui/detail/renderable_sync.hpp>
#include <dart/gui/detail/scene_frame.hpp>
#include <dart/gui/detail/scene_startup.hpp>
#include <dart/gui/detail/scenes.hpp>
#include <dart/gui/detail/selection.hpp>
#include <dart/gui/detail/simulation_stepper.hpp>
#include <dart/gui/profile.hpp>
#include <dart/gui/scene.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/collision_result.hpp>

#include <dart/common/profile.hpp>

#include <chrono>

namespace dart::gui::detail {

SceneFrameUpdater::SceneFrameUpdater(
    GLFWwindow* window,
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    const MaterialSet& materials,
    MaterialResources& materialResources,
    const dart::gui::RunOptions& options,
    DartScene& dartScene,
    const dart::gui::OrbitCameraController& cameraController,
    SelectionController& selectionController,
    InitialSceneState& sceneState,
    std::optional<Renderable>& selectionDebugOverlay,
    dart::gui::ViewerLifecycleState& lifecycle,
    SimulationStepper& simulationStepper,
    SceneLights& lights,
    dart::gui::ProfileAccumulator::Clock::time_point orbitStartClock,
    dart::gui::ProfileAccumulator& profile)
  : mWindow(window),
    mEngine(engine),
    mScene(scene),
    mMaterials(materials),
    mMaterialResources(materialResources),
    mOptions(options),
    mDartScene(dartScene),
    mCameraController(cameraController),
    mSelectionController(selectionController),
    mSceneState(sceneState),
    mSelectionDebugOverlay(selectionDebugOverlay),
    mLifecycle(lifecycle),
    mSimulationStepper(simulationStepper),
    mLights(lights),
    mOrbitStartClock(orbitStartClock),
    mProfile(profile)
{
}

void SceneFrameUpdater::update(
    const FrameViewport& viewport,
    bool showUi,
    bool uiCapturesMouse,
    bool orbitLight,
    double orbitLightPeriodSeconds)
{
  DART_PROFILE_SCOPED_N("SceneFrameUpdater::update");
  const std::size_t simulationStepsToRun = mSimulationStepper.stepsToRun(
      mOptions, mLifecycle, mDartScene.world->getTimeStep());

  if (advanceSimulationSteps(
          mDartScene, simulationStepsToRun, mLifecycle, mProfile)) {
    auto phaseStart = dart::gui::ProfileAccumulator::Clock::now();
    refreshContactDebugOverlay(
        mEngine,
        mScene,
        mMaterials.debugColor,
        mDartScene.world->getLastCollisionResult(),
        mSceneState.debugOverlays);
    mProfile.contactDebugMs += dart::gui::elapsedMs(phaseStart);
  }

  auto phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  auto descriptors = mExtractor.extract(*mDartScene.world);
  mProfile.extractionMs += dart::gui::elapsedMs(phaseStart);

  phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  mSelectionController.applyKeyboardNudge(
      mWindow,
      mCameraController.camera,
      mDartScene,
      descriptors,
      mLifecycle,
      0.035);
  mProfile.interactionMs += dart::gui::elapsedMs(phaseStart);

  phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  synchronizeSceneRenderables(
      mEngine,
      mScene,
      descriptors,
      mSceneState.sceneRenderables,
      mSceneState.loggedUnsupportedRenderableIds,
      [&](const dart::gui::RenderableDescriptor& descriptor) {
        return createRenderableFromDescriptor(
            mEngine, mMaterials, mMaterialResources.textureCache, descriptor);
      });
  if (!updateSceneRenderablesFromDescriptors(
          mEngine,
          descriptors,
          mSceneState.sceneRenderables,
          mSelectionController.selectedRenderableId(),
          mDartScene.renderSettings,
          mCameraController.camera,
          viewport.width,
          viewport.height)) {
    mSelectionController.clear();
  }
  mProfile.syncMs += dart::gui::elapsedMs(phaseStart);

  if (orbitLight) {
    const double orbitElapsedSeconds
        = std::chrono::duration<double>(
              dart::gui::ProfileAccumulator::Clock::now() - mOrbitStartClock)
              .count();
    updateOrbitingKeyLight(
        mEngine, mLights, orbitElapsedSeconds, orbitLightPeriodSeconds);
  }

  phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  mSelectionController.updateMouseSelection(
      mWindow,
      mCameraController.camera,
      viewport.width,
      viewport.height,
      showUi,
      uiCapturesMouse,
      mOptions.guiScale,
      mDartScene,
      descriptors,
      mLifecycle);
  mProfile.interactionMs += dart::gui::elapsedMs(phaseStart);

  phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  refreshGizmoDebugOverlay(
      mEngine,
      mScene,
      mMaterials.debugColor,
      mDartScene.gizmos,
      mOptions.guiScale,
      mSelectionController.highlightedGizmoHandle(),
      mSceneState.debugOverlays);
  mProfile.selectionDebugMs += dart::gui::elapsedMs(phaseStart);

  phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  refreshSelectionDebugLineOverlay(
      mEngine,
      mScene,
      mMaterials.debugColor,
      descriptors,
      mSelectionController.selectionDebugRenderableId(),
      mSelectionDebugOverlay);
  mProfile.selectionDebugMs += dart::gui::elapsedMs(phaseStart);
}

} // namespace dart::gui::detail
