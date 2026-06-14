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

#include <dart/collision/collision_result.hpp>

#include <dart/common/profile.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <string_view>

namespace dart::gui::detail {
namespace {

const dart::gui::RenderableDescriptor* findRenderableDescriptorById(
    const std::vector<dart::gui::RenderableDescriptor>& descriptors,
    dart::gui::RenderableId id)
{
  const auto found = std::find_if(
      descriptors.begin(),
      descriptors.end(),
      [id](const dart::gui::RenderableDescriptor& candidate) {
        return candidate.id == id;
      });
  return found == descriptors.end() ? nullptr : &*found;
}

void syncExternalRenderableSelection(
    SelectionController& selectionController,
    const DartScene& scene,
    const std::vector<dart::gui::RenderableDescriptor>& descriptors)
{
  if (!scene.selectedRenderableProvider) {
    return;
  }

  const dart::gui::RenderableSelection selection
      = scene.selectedRenderableProvider();
  if (selection.id == 0) {
    if (selectionController.selectedRenderableId() != 0) {
      selectionController.clear();
    }
    return;
  }

  if (findRenderableDescriptorById(descriptors, selection.id) == nullptr) {
    if (selectionController.selectedRenderableId() != 0) {
      selectionController.clear();
    }
    return;
  }

  if (selectionController.selectedRenderableId() != selection.id
      || selectionController.selectedLabel() != selection.label) {
    selectionController.select(selection.id, selection.label);
  }
}

void notifyRenderableSelectionChanged(
    const DartScene& scene,
    dart::gui::RenderableId before,
    dart::gui::RenderableId after,
    bool mouseSelectionCommitted)
{
  if (scene.onRenderableSelected
      && (before != after || (mouseSelectionCommitted && after == 0))) {
    scene.onRenderableSelected(after);
  }
}

std::string jsonEscape(std::string_view value)
{
  std::string out;
  out.reserve(value.size() + 2);
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(ch);
        break;
    }
  }
  return out;
}

void appendScriptedForceDragEvent(
    const ScriptedForceDrag* scriptedForceDrag,
    std::string_view event,
    int frame,
    std::string_view status)
{
  if (scriptedForceDrag == nullptr || scriptedForceDrag->eventLogPath.empty()) {
    return;
  }

  std::ofstream out(scriptedForceDrag->eventLogPath, std::ios::app);
  if (!out) {
    return;
  }

  out << "{\"source\":\"viewer\",\"event\":\"" << jsonEscape(event)
      << "\",\"frame\":" << frame << ",\"target\":\""
      << jsonEscape(scriptedForceDrag->target) << '"';
  if (scriptedForceDrag->usePointer) {
    const Eigen::Vector2d& start = scriptedForceDrag->startCursor;
    const Eigen::Vector2d& delta = scriptedForceDrag->cursorDelta;
    out << ",\"start_pixel\":[" << start.x() << ',' << start.y()
        << "],\"delta_pixels\":[" << delta.x() << ',' << delta.y() << ']';
  } else {
    const Eigen::Vector3d& offset = scriptedForceDrag->targetOffset;
    out << ",\"offset\":[" << offset.x() << ',' << offset.y() << ','
        << offset.z() << ']';
  }
  out << ",\"status\":\"" << jsonEscape(status) << "\"}\n";
}

} // namespace

SceneFrameUpdater::SceneFrameUpdater(
    GLFWwindow* window,
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    const MaterialSet& materials,
    MaterialResources& materialResources,
    const dart::gui::RunOptions& options,
    DartScene& dartScene,
    SelectionController& selectionController,
    InitialSceneState& sceneState,
    std::optional<Renderable>& selectionDebugOverlay,
    dart::gui::ViewerLifecycleState& lifecycle,
    SimulationStepper& simulationStepper,
    SceneLights& lights,
    dart::gui::ProfileAccumulator::Clock::time_point orbitStartClock,
    dart::gui::ProfileAccumulator& profile,
    ScriptedForceDrag* scriptedForceDrag)
  : mWindow(window),
    mEngine(engine),
    mScene(scene),
    mMaterials(materials),
    mMaterialResources(materialResources),
    mOptions(options),
    mDartScene(dartScene),
    mSelectionController(selectionController),
    mSceneState(sceneState),
    mSelectionDebugOverlay(selectionDebugOverlay),
    mLifecycle(lifecycle),
    mSimulationStepper(simulationStepper),
    mLights(lights),
    mOrbitStartClock(orbitStartClock),
    mProfile(profile),
    mScriptedForceDrag(scriptedForceDrag)
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
      mOptions, mLifecycle, mDartScene.timeStep);

  advanceSimulationSteps(
      mDartScene, simulationStepsToRun, mLifecycle, mProfile);

  auto phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  auto descriptors = collectSceneRenderables(mDartScene);
  syncExternalRenderableSelection(
      mSelectionController, mDartScene, descriptors);
  mProfile.extractionMs += dart::gui::elapsedMs(phaseStart);

  if (mScriptedForceDrag != nullptr && !mScriptedForceDrag->completed) {
    const int currentFrame = mLifecycle.renderedFrames;
    if (!mScriptedForceDrag->started
        && currentFrame >= mScriptedForceDrag->afterFrames) {
      const bool started
          = mScriptedForceDrag->usePointer
                ? mSelectionController.beginScriptedForceDragAtPointer(
                      viewport,
                      mDartScene,
                      descriptors,
                      mScriptedForceDrag->startCursor,
                      mScriptedForceDrag->startPoint,
                      mLifecycle)
                : mSelectionController.beginScriptedForceDrag(
                      viewport,
                      mDartScene,
                      descriptors,
                      mScriptedForceDrag->target,
                      mScriptedForceDrag->startPoint,
                      mLifecycle);
      if (started) {
        mScriptedForceDrag->started = true;
        appendScriptedForceDragEvent(
            mScriptedForceDrag,
            "force_drag_started",
            currentFrame,
            mSelectionController.interactionStatus());
      } else {
        mScriptedForceDrag->completed = true;
        appendScriptedForceDragEvent(
            mScriptedForceDrag,
            "force_drag_target_missing",
            currentFrame,
            "scripted external-force target was not found or cannot receive "
            "force events");
      }
    }

    if (mScriptedForceDrag->started && !mScriptedForceDrag->completed) {
      const int duration = std::max(1, mScriptedForceDrag->durationFrames);
      const double alpha
          = static_cast<double>(mScriptedForceDrag->elapsedFrames + 1)
            / static_cast<double>(duration);
      const bool updated
          = mScriptedForceDrag->usePointer
                ? mSelectionController.updateScriptedForceDragAtPointer(
                      viewport,
                      mDartScene,
                      descriptors,
                      mScriptedForceDrag->startCursor
                          + mScriptedForceDrag->cursorDelta
                                * std::min(1.0, alpha))
                : mSelectionController.updateScriptedForceDragToTarget(
                      viewport,
                      mDartScene,
                      descriptors,
                      mScriptedForceDrag->startPoint
                          + mScriptedForceDrag->targetOffset
                                * std::min(1.0, alpha));
      if (!updated) {
        mSelectionController.cancelActiveDrag(mDartScene);
        mScriptedForceDrag->completed = true;
        appendScriptedForceDragEvent(
            mScriptedForceDrag,
            "force_drag_target_unreachable",
            currentFrame,
            mScriptedForceDrag->usePointer
                ? "scripted external-force pixel path cannot update"
                : "scripted external-force target moved outside the active "
                  "viewport");
      } else {
        appendScriptedForceDragEvent(
            mScriptedForceDrag,
            "force_drag_updated",
            currentFrame,
            mSelectionController.interactionStatus());
        ++mScriptedForceDrag->elapsedFrames;
        if (mScriptedForceDrag->elapsedFrames >= duration) {
          mSelectionController.cancelActiveDrag(mDartScene);
          mScriptedForceDrag->completed = true;
          appendScriptedForceDragEvent(
              mScriptedForceDrag,
              "force_drag_released",
              currentFrame,
              "scripted external-force drag completed");
        }
      }
    }
  }

  const ViewportPaneFrame& activePane = activeViewportPane(viewport);

  phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  mSelectionController.applyKeyboardNudge(
      mWindow, activePane.camera, mDartScene, descriptors, mLifecycle, 0.035);
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
          activePane.camera,
          activePane.width,
          activePane.height)) {
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
  const dart::gui::RenderableId selectionBeforeMouse
      = mSelectionController.selectedRenderableId();
  const bool mouseSelectionCommitted
      = mSelectionController.updateMouseSelection(
          mWindow,
          viewport,
          showUi,
          uiCapturesMouse,
          mOptions.guiScale,
          mDartScene,
          descriptors,
          mLifecycle);
  notifyRenderableSelectionChanged(
      mDartScene,
      selectionBeforeMouse,
      mSelectionController.selectedRenderableId(),
      mouseSelectionCommitted);
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
      mSelectionController.forceDragDebugLines(),
      mSelectionDebugOverlay);
  mProfile.selectionDebugMs += dart::gui::elapsedMs(phaseStart);

  // Application-supplied debug geometry (lines/triangles into the overlay,
  // labels stashed for the UI pass). Called once per frame here so headless
  // runs without a UI still render provider lines/triangles.
  if (mDartScene.debugProvider) {
    dart::gui::DebugScene providerScene = mDartScene.debugProvider();
    refreshProviderDebugOverlay(
        mEngine,
        mScene,
        mMaterials.debugColor,
        providerScene.lines,
        providerScene.triangles,
        mSceneState.debugOverlays);
    mSceneState.debugOverlays.providerLabels = std::move(providerScene.labels);
  } else {
    // Clear provider geometry and labels whenever no provider is active. A
    // labels-only provider leaves providerOverlay empty while providerLabels
    // stays populated, so clearing only the overlay would leak stale labels
    // into updateFrameUi after the provider stops emitting.
    clearProviderDebugOverlay(mEngine, mScene, mSceneState.debugOverlays);
  }
}

void SceneFrameUpdater::releaseScriptedForceDragIfActive(
    std::string_view status)
{
  if (mScriptedForceDrag == nullptr || !mScriptedForceDrag->started
      || mScriptedForceDrag->completed) {
    return;
  }

  mSelectionController.cancelActiveDrag(mDartScene);
  mScriptedForceDrag->completed = true;
  appendScriptedForceDragEvent(
      mScriptedForceDrag,
      "force_drag_released",
      mLifecycle.renderedFrames,
      status);
}

} // namespace dart::gui::detail
