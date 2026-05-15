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

#include <dart/gui/experimental/detail/filament/application.hpp>
#include <dart/gui/experimental/detail/filament/application_teardown.hpp>
#include <dart/gui/experimental/detail/filament/scenes.hpp>
#include <dart/gui/experimental/detail/filament/selection.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/common/profile.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/gui/experimental/detail/filament/debug_overlay.hpp>
#include <dart/gui/experimental/detail/filament/frame_renderer.hpp>
#include <dart/gui/experimental/detail/filament/frame_viewport.hpp>
#include <dart/gui/experimental/detail/filament/imgui_overlay.hpp>
#include <dart/gui/experimental/detail/filament/input.hpp>
#include <dart/gui/experimental/detail/filament/native_window.hpp>
#include <dart/gui/experimental/detail/filament/renderable_factory.hpp>
#include <dart/gui/experimental/detail/filament/renderable_resources.hpp>
#include <dart/gui/experimental/detail/filament/renderable_sync.hpp>
#include <dart/gui/experimental/detail/filament/render_environment.hpp>
#include <dart/gui/experimental/detail/filament/render_context.hpp>
#include <dart/gui/experimental/detail/filament/scene_requirements.hpp>
#include <dart/gui/experimental/detail/filament/screenshot.hpp>
#include <dart/gui/experimental/detail/filament/simulation_stepper.hpp>
#include <dart/gui/experimental/detail/filament/ui_frame.hpp>
#include <dart/gui/experimental/profile.hpp>
#include <dart/gui/experimental/scene.hpp>
#include <dart/simulation/world.hpp>

#include <GLFW/glfw3.h>

#include <imgui.h>

#include <chrono>
#include <cstddef>
#include <iostream>
#include <optional>
#include <vector>

namespace {

using dart::gui::experimental::OrbitCameraController;
using dart::gui::experimental::ProfileAccumulator;
using dart::gui::experimental::RenderableDescriptor;
using dart::gui::experimental::RenderableId;
using dart::gui::experimental::RunOptions;
using dart::gui::experimental::ViewerLifecycleState;
using dart::gui::experimental::elapsedMs;
using dart::gui::experimental::extractRenderables;
using dart::gui::experimental::printProfile;
using dart::gui::experimental::filament::ApplicationInputState;
using dart::gui::experimental::filament::ApplicationWindow;
using dart::gui::experimental::filament::FilamentRenderContext;
using dart::gui::experimental::filament::FrameRenderResult;
using dart::gui::experimental::filament::FrameViewport;
using dart::gui::experimental::filament::ImGuiOverlay;
using dart::gui::experimental::filament::MaterialResources;
using dart::gui::experimental::filament::MaterialSet;
using dart::gui::experimental::filament::Renderable;
using dart::gui::experimental::filament::SceneRenderable;
using dart::gui::experimental::filament::SceneLights;
using dart::gui::experimental::filament::ScreenshotCapture;
using dart::gui::experimental::filament::SceneContentCounts;
using dart::gui::experimental::filament::SelectionController;
using dart::gui::experimental::filament::SimulationStepper;
using dart::gui::experimental::filament::advanceSimulationSteps;
using dart::gui::experimental::filament::attachSceneEnvironment;
using dart::gui::experimental::filament::configureMainView;
using dart::gui::experimental::filament::countCreatedSceneContent;
using dart::gui::experimental::filament::countSceneContent;
using dart::gui::experimental::filament::createFilamentRenderContext;
using dart::gui::experimental::filament::createApplicationWindow;
using dart::gui::experimental::filament::createDebugColorGrading;
using dart::gui::experimental::filament::createConfiguredImGuiOverlay;
using dart::gui::experimental::filament::createNeutralIndirectLight;
using dart::gui::experimental::filament::createNeutralSkybox;
using dart::gui::experimental::filament::createMaterialResources;
using dart::gui::experimental::filament::createRenderableFromDescriptor;
using dart::gui::experimental::filament::createSceneLights;
using dart::gui::experimental::filament::DebugOverlayController;
using dart::gui::experimental::filament::destroyApplicationResources;
using dart::gui::experimental::filament::finalizeScreenshotCapture;
using dart::gui::experimental::filament::getNativeWindow;
using dart::gui::experimental::filament::handleScroll;
using dart::gui::experimental::filament::makeDebugOverlayController;
using dart::gui::experimental::filament::pollApplicationInput;
using dart::gui::experimental::filament::renderApplicationFrame;
using dart::gui::experimental::filament::refreshContactDebugOverlay;
using dart::gui::experimental::filament::refreshSelectionDebugLineOverlay;
using dart::gui::experimental::filament::refreshStaticDebugOverlay;
using dart::gui::experimental::filament::synchronizeSceneRenderables;
using dart::gui::experimental::filament::updateFrameViewport;
using dart::gui::experimental::filament::updateFrameUi;
using dart::gui::experimental::filament::updateOrbitingKeyLight;
using dart::gui::experimental::filament::updateSceneRenderablesFromDescriptors;
using dart::gui::experimental::filament::AppOptions;
using dart::gui::experimental::filament::DartScene;
using dart::gui::experimental::filament::ExampleScene;
using dart::gui::experimental::filament::createDartScene;
using dart::gui::experimental::filament::initialCameraForScene;
using dart::gui::experimental::filament::parseOptions;
using dart::gui::experimental::filament::validateCreatedSceneContent;
using dart::gui::experimental::filament::validateSceneDescriptorContent;

int runFilamentGuiApplicationImpl(int argc, char* argv[])
{
  const AppOptions appOptions = parseOptions(argc, argv);
  const RunOptions& options = appOptions.run;

  ApplicationWindow appWindow = createApplicationWindow(options, std::cerr);
  GLFWwindow* window = appWindow.get();
  if (!options.headless && window == nullptr) {
    return 1;
  }

  OrbitCameraController cameraController;
  cameraController.camera = initialCameraForScene(appOptions.scene);
  if (window != nullptr) {
    glfwSetWindowUserPointer(window, &cameraController);
    glfwSetScrollCallback(window, handleScroll);
  }

  FilamentRenderContext renderContext = createFilamentRenderContext(
      options, options.headless ? nullptr : getNativeWindow(window));
  auto* engine = renderContext.engine;
  auto* view = renderContext.view;
  auto* scene = renderContext.scene;
  auto* camera = renderContext.camera;
  auto* colorGrading = createDebugColorGrading(*engine);
  configureMainView(*view, colorGrading, options.headless);
  auto* indirectLight = createNeutralIndirectLight(*engine);
  auto* skybox = createNeutralSkybox(*engine);

  MaterialResources materialResources = createMaterialResources(*engine);
  const MaterialSet materials = materialResources.materialSet();

  DartScene dartScene = createDartScene(appOptions);
  const auto initialDescriptors = extractRenderables(*dartScene.world);
  const SceneContentCounts expectedSceneContent
      = countSceneContent(initialDescriptors);
  if (!validateSceneDescriptorContent(
          appOptions.scene, expectedSceneContent, std::cerr)) {
    return 1;
  }

  std::vector<SceneRenderable> sceneRenderables;
  std::vector<RenderableId> loggedUnsupportedRenderableIds;
  synchronizeSceneRenderables(
      *engine,
      *scene,
      initialDescriptors,
      sceneRenderables,
      loggedUnsupportedRenderableIds,
      [&](const RenderableDescriptor& descriptor) {
        return createRenderableFromDescriptor(
            *engine, materials, materialResources.textureCache, descriptor);
      });
  if (sceneRenderables.empty()) {
    std::cerr << "No supported visible DART renderables were extracted\n";
    return 1;
  }
  const SceneContentCounts createdSceneContent
      = countCreatedSceneContent(initialDescriptors, sceneRenderables);
  if (!validateCreatedSceneContent(
          appOptions.scene,
          expectedSceneContent,
          createdSceneContent,
          std::cerr)) {
    return 1;
  }

  DebugOverlayController debugOverlays
      = makeDebugOverlayController(appOptions.scene == ExampleScene::G1);
  refreshStaticDebugOverlay(
      *engine, *scene, materials.debugColor, *dartScene.world, debugOverlays);
  if (!debugOverlays.staticOverlay) {
    std::cerr << "No debug overlay lines were extracted\n";
    return 1;
  }

  std::optional<Renderable> selectionDebugOverlay;

  bool orbitLight = appOptions.orbitLight;
  SceneLights lights = createSceneLights(
      *engine, options.headless, orbitLight, appOptions.orbitLightPeriodSeconds);
  attachSceneEnvironment(*scene, indirectLight, skybox, lights);

  const float guiScale = static_cast<float>(options.guiScale);
  ImGuiOverlay imguiOverlay = createConfiguredImGuiOverlay(*engine, guiScale);
  auto& imguiIo = ImGui::GetIO();

  ViewerLifecycleState lifecycle;
  ApplicationInputState inputState;
  SelectionController selectionController;
  ScreenshotCapture screenshotCapture;
  ProfileAccumulator profile;
  SimulationStepper simulationStepper;
  const auto orbitStartClock = ProfileAccumulator::Clock::now();

  while (options.headless || !glfwWindowShouldClose(window)) {
    const auto frameStart = ProfileAccumulator::Clock::now();
    auto phaseStart = ProfileAccumulator::Clock::now();
    pollApplicationInput(
        window, dartScene, selectionController, lifecycle, inputState);
    profile.inputMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    const FrameViewport viewport = updateFrameViewport(
        window,
        *view,
        *camera,
        cameraController,
        selectionController,
        imguiIo,
        options.width,
        options.height,
        dartScene.world->getTimeStep(),
        appOptions.showUi,
        options.guiScale);
    profile.viewportCameraMs += elapsedMs(phaseStart);

    const std::size_t simulationStepsToRun
        = simulationStepper.stepsToRun(
            options, lifecycle, dartScene.world->getTimeStep());

    if (advanceSimulationSteps(
            *dartScene.world, simulationStepsToRun, lifecycle, profile)) {
      phaseStart = ProfileAccumulator::Clock::now();
      refreshContactDebugOverlay(
          *engine,
          *scene,
          materials.debugColor,
          dartScene.world->getLastCollisionResult(),
          debugOverlays);
      profile.contactDebugMs += elapsedMs(phaseStart);
    }

    phaseStart = ProfileAccumulator::Clock::now();
    auto descriptors = extractRenderables(*dartScene.world);
    profile.extractionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    selectionController.applyKeyboardNudge(
        window, cameraController.camera, dartScene, descriptors, lifecycle, 0.035);
    profile.interactionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    synchronizeSceneRenderables(
        *engine,
        *scene,
        descriptors,
        sceneRenderables,
        loggedUnsupportedRenderableIds,
        [&](const RenderableDescriptor& descriptor) {
          return createRenderableFromDescriptor(
              *engine, materials, materialResources.textureCache, descriptor);
        });
    if (!updateSceneRenderablesFromDescriptors(
            *engine,
            descriptors,
            sceneRenderables,
            selectionController.selectedRenderableId())) {
      selectionController.clear();
    }
    profile.syncMs += elapsedMs(phaseStart);

    if (orbitLight) {
      const double orbitElapsedSeconds = std::chrono::duration<double>(
                                             ProfileAccumulator::Clock::now()
                                             - orbitStartClock)
                                             .count();
      updateOrbitingKeyLight(
          *engine, lights, orbitElapsedSeconds, appOptions.orbitLightPeriodSeconds);
    }

    phaseStart = ProfileAccumulator::Clock::now();
    selectionController.updateMouseSelection(
        window,
        cameraController.camera,
        viewport.width,
        viewport.height,
        appOptions.showUi,
        options.guiScale,
        dartScene,
        descriptors,
        lifecycle);
    profile.interactionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    refreshSelectionDebugLineOverlay(
        *engine,
        *scene,
        materials.debugColor,
        descriptors,
        selectionController.selectedRenderableId(),
        selectionDebugOverlay);
    profile.selectionDebugMs += elapsedMs(phaseStart);

    if (appOptions.showUi) {
      updateFrameUi(
          window,
          *engine,
          *scene,
          materials.debugColor,
          imguiOverlay,
          imguiIo,
          viewport,
          appOptions.scene,
          dartScene,
          selectionController,
          orbitLight,
          debugOverlays,
          lifecycle,
          guiScale,
          profile);
    }

    const FrameRenderResult frameRenderResult = renderApplicationFrame(
        renderContext,
        appOptions.showUi ? imguiOverlay.view : nullptr,
        options,
        viewport.width,
        viewport.height,
        frameStart,
        screenshotCapture,
        lifecycle,
        profile);
    if (frameRenderResult.continueLoop) {
      continue;
    }
    if (frameRenderResult.stopLoop) {
      break;
    }
  }

  const bool screenshotSucceeded = finalizeScreenshotCapture(
      renderContext,
      screenshotCapture,
      options.screenshotPath,
      lifecycle.screenshotRequested,
      profile);
  if (options.maxFrames >= 0) {
    std::cout << "Final contacts: "
              << dartScene.world->getLastCollisionResult().getNumContacts()
              << "\n";
  }
  if (appOptions.profile) {
    printProfile(profile);
    DART_PROFILE_TEXT_DUMP();
  }

  destroyApplicationResources(
      renderContext,
      imguiOverlay,
      lights,
      indirectLight,
      skybox,
      colorGrading,
      sceneRenderables,
      debugOverlays,
      selectionDebugOverlay,
      materialResources);

  return screenshotSucceeded ? 0 : 1;
}

} // namespace

namespace dart::gui::experimental::filament {

int runFilamentGuiApplication(int argc, char* argv[])
{
  return runFilamentGuiApplicationImpl(argc, argv);
}

} // namespace dart::gui::experimental::filament
