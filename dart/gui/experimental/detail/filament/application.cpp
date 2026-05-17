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

#include <dart/gui/application.hpp>
#include <dart/gui/experimental/detail/application.hpp>
#include <dart/gui/experimental/detail/filament/application.hpp>
#include <dart/gui/experimental/detail/filament/application_teardown.hpp>
#include <dart/gui/experimental/detail/filament/debug_overlay.hpp>
#include <dart/gui/experimental/detail/filament/frame_renderer.hpp>
#include <dart/gui/experimental/detail/filament/frame_viewport.hpp>
#include <dart/gui/experimental/detail/filament/imgui_overlay.hpp>
#include <dart/gui/experimental/detail/filament/input.hpp>
#include <dart/gui/experimental/detail/filament/native_window.hpp>
#include <dart/gui/experimental/detail/filament/render_context.hpp>
#include <dart/gui/experimental/detail/filament/render_environment.hpp>
#include <dart/gui/experimental/detail/filament/renderable_resources.hpp>
#include <dart/gui/experimental/detail/filament/scene_frame.hpp>
#include <dart/gui/experimental/detail/filament/scene_startup.hpp>
#include <dart/gui/experimental/detail/filament/scenes.hpp>
#include <dart/gui/experimental/detail/filament/screenshot.hpp>
#include <dart/gui/experimental/detail/filament/selection.hpp>
#include <dart/gui/experimental/detail/filament/simulation_stepper.hpp>
#include <dart/gui/experimental/detail/filament/ui_frame.hpp>
#include <dart/gui/profile.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/collision_result.hpp>

#include <dart/dynamics/body_node.hpp>

#include <dart/common/profile.hpp>

#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <cstddef>

namespace {

using dart::gui::elapsedMs;
using dart::gui::OrbitCameraController;
using dart::gui::printProfile;
using dart::gui::ProfileAccumulator;
using dart::gui::RunOptions;
using dart::gui::ViewerLifecycleState;
using dart::gui::filament::ApplicationInputState;
using dart::gui::filament::ApplicationWindow;
using dart::gui::filament::AppOptions;
using dart::gui::filament::attachOrbitCameraController;
using dart::gui::filament::attachSceneEnvironment;
using dart::gui::filament::configureMainView;
using dart::gui::filament::createApplicationWindow;
using dart::gui::filament::createConfiguredImGuiOverlay;
using dart::gui::filament::createDartScene;
using dart::gui::filament::createDebugColorGrading;
using dart::gui::filament::createFilamentRenderContext;
using dart::gui::filament::createInitialSceneState;
using dart::gui::filament::createMaterialResources;
using dart::gui::filament::createNeutralIndirectLight;
using dart::gui::filament::createNeutralSkybox;
using dart::gui::filament::createSceneLights;
using dart::gui::filament::DartScene;
using dart::gui::filament::destroyApplicationResources;
using dart::gui::filament::ExampleScene;
using dart::gui::filament::FilamentRenderContext;
using dart::gui::filament::finalizeScreenshotCapture;
using dart::gui::filament::FrameRenderResult;
using dart::gui::filament::FrameViewport;
using dart::gui::filament::getCurrentImGuiIo;
using dart::gui::filament::getNativeWindow;
using dart::gui::filament::ImGuiOverlay;
using dart::gui::filament::initialCameraForScene;
using dart::gui::filament::InitialSceneState;
using dart::gui::filament::MaterialResources;
using dart::gui::filament::MaterialSet;
using dart::gui::filament::parseOptions;
using dart::gui::filament::pollApplicationInput;
using dart::gui::filament::Renderable;
using dart::gui::filament::renderApplicationFrame;
using dart::gui::filament::SceneFrameUpdater;
using dart::gui::filament::SceneLights;
using dart::gui::filament::ScreenshotCapture;
using dart::gui::filament::SelectionController;
using dart::gui::filament::shouldContinueApplicationLoop;
using dart::gui::filament::SimulationStepper;
using dart::gui::filament::updateFrameUi;
using dart::gui::filament::updateFrameViewport;

bool hasSceneOption(int argc, char* argv[]);

int runFilamentGuiApplicationImpl(
    int argc,
    char* argv[],
    const dart::gui::ApplicationOptions& applicationOptions)
{
  AppOptions appOptions
      = parseOptions(argc, argv, applicationOptions.runDefaults);
  appOptions.camera = applicationOptions.camera;
  if (!hasSceneOption(argc, argv)) {
    appOptions.world = applicationOptions.world;
    appOptions.preStep = applicationOptions.preStep;
    appOptions.gizmos = applicationOptions.gizmos;
    appOptions.ikHandles = applicationOptions.ikHandles;
    appOptions.keyboardActions = applicationOptions.keyboardActions;
  }
  appOptions.panels = applicationOptions.panels;
  const RunOptions& runOptions = appOptions.run;

  if (!runOptions.frameOutputDirectory.empty()) {
    std::error_code error;
    const std::filesystem::path outputDirectory(
        runOptions.frameOutputDirectory);
    std::filesystem::create_directories(outputDirectory, error);
    if (error) {
      std::cerr << "Failed to create frame output directory '"
                << runOptions.frameOutputDirectory << "': " << error.message()
                << "\n";
      return 1;
    }
    if (!std::filesystem::is_directory(outputDirectory, error)) {
      std::cerr << "Frame output path is not a directory: "
                << runOptions.frameOutputDirectory << "\n";
      return 1;
    }
  }

  ApplicationWindow appWindow = createApplicationWindow(runOptions, std::cerr);
  GLFWwindow* window = appWindow.get();
  if (!runOptions.headless && window == nullptr) {
    return 1;
  }

  OrbitCameraController cameraController;
  cameraController.camera
      = appOptions.camera.value_or(initialCameraForScene(appOptions.scene));
  const auto homeCamera = cameraController.camera;
  attachOrbitCameraController(window, cameraController);

  FilamentRenderContext renderContext = createFilamentRenderContext(
      runOptions, runOptions.headless ? nullptr : getNativeWindow(window));
  auto* engine = renderContext.engine;
  auto* view = renderContext.view;
  auto* scene = renderContext.scene;
  auto* camera = renderContext.camera;
  auto* colorGrading = createDebugColorGrading(*engine);
  configureMainView(*view, colorGrading, runOptions.headless);
  auto* indirectLight = createNeutralIndirectLight(*engine);
  auto* skybox = createNeutralSkybox(*engine);

  MaterialResources materialResources = createMaterialResources(*engine);
  const MaterialSet materials = materialResources.materialSet();

  const bool validateFixtureRequirements = appOptions.world == nullptr;
  DartScene dartScene = createDartScene(appOptions);
  std::optional<InitialSceneState> maybeInitialSceneState
      = createInitialSceneState(
          *engine,
          *scene,
          materials,
          materialResources,
          appOptions.scene,
          dartScene,
          validateFixtureRequirements,
          std::cerr);
  if (!maybeInitialSceneState) {
    return 1;
  }
  InitialSceneState sceneState = std::move(*maybeInitialSceneState);
  auto& sceneRenderables = sceneState.sceneRenderables;
  auto& debugOverlays = sceneState.debugOverlays;

  std::optional<Renderable> selectionDebugOverlay;

  bool orbitLight = appOptions.orbitLight;
  bool headlightsEnabled = true;
  SceneLights lights = createSceneLights(
      *engine,
      runOptions.headless,
      orbitLight,
      appOptions.orbitLightPeriodSeconds);
  attachSceneEnvironment(*scene, indirectLight, skybox, lights);

  const float guiScale = static_cast<float>(runOptions.guiScale);
  ImGuiOverlay imguiOverlay = createConfiguredImGuiOverlay(*engine, guiScale);
  auto& imguiIo = getCurrentImGuiIo();

  ViewerLifecycleState lifecycle;
  ApplicationInputState inputState;
  SelectionController selectionController;
  ScreenshotCapture screenshotCapture;
  ProfileAccumulator profile;
  bool frameCaptureSucceeded = true;
  SimulationStepper simulationStepper;
  const auto orbitStartClock = ProfileAccumulator::Clock::now();
  SceneFrameUpdater sceneFrameUpdater(
      window,
      *engine,
      *scene,
      materials,
      materialResources,
      runOptions,
      dartScene,
      cameraController,
      selectionController,
      sceneState,
      selectionDebugOverlay,
      lifecycle,
      simulationStepper,
      lights,
      orbitStartClock,
      profile);

  while (!lifecycle.exitRequested
         && shouldContinueApplicationLoop(runOptions.headless, window)) {
    const auto frameStart = ProfileAccumulator::Clock::now();
    auto phaseStart = ProfileAccumulator::Clock::now();
    pollApplicationInput(
        window,
        dartScene,
        selectionController,
        lifecycle,
        cameraController,
        homeCamera,
        inputState);
    profile.inputMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    const FrameViewport viewport = updateFrameViewport(
        window,
        *view,
        *camera,
        cameraController,
        selectionController,
        imguiIo,
        runOptions.width,
        runOptions.height,
        dartScene.world->getTimeStep(),
        appOptions.showUi,
        runOptions.guiScale);
    profile.viewportCameraMs += elapsedMs(phaseStart);

    sceneFrameUpdater.update(
        viewport,
        appOptions.showUi,
        orbitLight,
        appOptions.orbitLightPeriodSeconds);

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
          cameraController,
          selectionController,
          orbitLight,
          headlightsEnabled,
          debugOverlays,
          appOptions.panels,
          lifecycle,
          guiScale,
          profile);
    }
    setSceneLightsEnabled(*engine, lights, headlightsEnabled);

    const FrameRenderResult frameRenderResult = renderApplicationFrame(
        renderContext,
        appOptions.showUi ? imguiOverlay.view : nullptr,
        runOptions,
        viewport.width,
        viewport.height,
        frameStart,
        screenshotCapture,
        lifecycle,
        profile);
    if (frameRenderResult.failed) {
      frameCaptureSucceeded = false;
      break;
    }
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
      runOptions.screenshotPath,
      lifecycle.screenshotRequested,
      profile);
  if (runOptions.maxFrames >= 0) {
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

  return screenshotSucceeded && frameCaptureSucceeded ? 0 : 1;
}

bool hasSceneOption(int argc, char* argv[])
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i] != nullptr && std::string_view(argv[i]) == "--scene") {
      return true;
    }
  }

  return false;
}

} // namespace

namespace dart::gui {

int runApplication(int argc, char* argv[])
{
  return runApplication(argc, argv, ApplicationOptions{});
}

int runApplication(int argc, char* argv[], const char* defaultScene)
{
  ApplicationOptions options;
  options.defaultScene = defaultScene == nullptr ? "" : defaultScene;
  return runApplication(argc, argv, options);
}

int runApplication(
    int argc, char* argv[], const ApplicationOptions& applicationOptions)
{
  if (applicationOptions.world != nullptr
      || applicationOptions.defaultScene.empty()
      || hasSceneOption(argc, argv)) {
    return filament::runFilamentGuiApplication(argc, argv, applicationOptions);
  }

  std::vector<std::string> argumentStorage;
  argumentStorage.reserve(static_cast<std::size_t>(argc) + 2u);
  for (int i = 0; i < argc; ++i) {
    argumentStorage.emplace_back(argv[i] == nullptr ? "" : argv[i]);
  }
  argumentStorage.emplace_back("--scene");
  argumentStorage.emplace_back(applicationOptions.defaultScene);

  std::vector<char*> rewrittenArguments;
  rewrittenArguments.reserve(argumentStorage.size());
  for (std::string& argument : argumentStorage) {
    rewrittenArguments.push_back(argument.data());
  }

  return filament::runFilamentGuiApplication(
      static_cast<int>(rewrittenArguments.size()),
      rewrittenArguments.data(),
      applicationOptions);
}

} // namespace dart::gui

namespace dart::gui::filament {

int runFilamentGuiApplication(int argc, char* argv[])
{
  return runFilamentGuiApplication(argc, argv, dart::gui::ApplicationOptions{});
}

int runFilamentGuiApplication(
    int argc,
    char* argv[],
    const dart::gui::ApplicationOptions& applicationOptions)
{
  return runFilamentGuiApplicationImpl(argc, argv, applicationOptions);
}

} // namespace dart::gui::filament

namespace dart::gui::detail {

int runGuiApplication(int argc, char* argv[])
{
  return dart::gui::runApplication(argc, argv);
}

} // namespace dart::gui::detail
