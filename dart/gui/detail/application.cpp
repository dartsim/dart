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
#include <dart/gui/detail/application.hpp>
#include <dart/gui/detail/application_runner.hpp>
#include <dart/gui/detail/application_teardown.hpp>
#include <dart/gui/detail/debug_overlay.hpp>
#include <dart/gui/detail/frame_renderer.hpp>
#include <dart/gui/detail/frame_viewport.hpp>
#include <dart/gui/detail/imgui_overlay.hpp>
#include <dart/gui/detail/input.hpp>
#include <dart/gui/detail/native_window.hpp>
#include <dart/gui/detail/perf_hud.hpp>
#include <dart/gui/detail/render_context.hpp>
#include <dart/gui/detail/render_environment.hpp>
#include <dart/gui/detail/renderable_resources.hpp>
#include <dart/gui/detail/scene_frame.hpp>
#include <dart/gui/detail/scene_startup.hpp>
#include <dart/gui/detail/scenes.hpp>
#include <dart/gui/detail/screenshot.hpp>
#include <dart/gui/detail/selection.hpp>
#include <dart/gui/detail/simulation_stepper.hpp>
#include <dart/gui/detail/ui_frame.hpp>
#include <dart/gui/profile.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/collision_result.hpp>

#include <dart/dynamics/body_node.hpp>

#include <dart/common/profile.hpp>

#include <utils/Log.h>

#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdlib>

namespace {

using dart::gui::elapsedMs;
using dart::gui::OrbitCameraController;
using dart::gui::OrbitCameraControlOptions;
using dart::gui::printProfile;
using dart::gui::ProfileAccumulator;
using dart::gui::RunOptions;
using dart::gui::ViewerLifecycleState;
using dart::gui::detail::ApplicationInputState;
using dart::gui::detail::ApplicationWindow;
using dart::gui::detail::applyRenderSettings;
using dart::gui::detail::AppOptions;
using dart::gui::detail::attachOrbitCameraController;
using dart::gui::detail::attachSceneEnvironment;
using dart::gui::detail::configureMainView;
using dart::gui::detail::createApplicationWindow;
using dart::gui::detail::createConfiguredImGuiOverlay;
using dart::gui::detail::createDartScene;
using dart::gui::detail::createDebugColorGrading;
using dart::gui::detail::createFilamentRenderContext;
using dart::gui::detail::createInitialSceneState;
using dart::gui::detail::createMaterialResources;
using dart::gui::detail::createNeutralIndirectLight;
using dart::gui::detail::createNeutralSkybox;
using dart::gui::detail::createSceneLights;
using dart::gui::detail::DartScene;
using dart::gui::detail::destroyApplicationResources;
using dart::gui::detail::ExampleScene;
using dart::gui::detail::FilamentRenderContext;
using dart::gui::detail::finalizeScreenshotCapture;
using dart::gui::detail::FrameRenderResult;
using dart::gui::detail::FrameViewport;
using dart::gui::detail::getCurrentImGuiIo;
using dart::gui::detail::getNativeWindow;
using dart::gui::detail::ImGuiOverlay;
using dart::gui::detail::initialCameraForScene;
using dart::gui::detail::InitialSceneState;
using dart::gui::detail::isSceneMouseInputCapturedByUi;
using dart::gui::detail::latestGpuFrameMs;
using dart::gui::detail::MaterialResources;
using dart::gui::detail::MaterialSet;
using dart::gui::detail::parseOptions;
using dart::gui::detail::PerfHudState;
using dart::gui::detail::pollApplicationInput;
using dart::gui::detail::Renderable;
using dart::gui::detail::renderApplicationFrame;
using dart::gui::detail::SceneFrameUpdater;
using dart::gui::detail::SceneLights;
using dart::gui::detail::ScreenshotCapture;
using dart::gui::detail::SelectionController;
using dart::gui::detail::shouldContinueApplicationLoop;
using dart::gui::detail::SimulationStepper;
using dart::gui::detail::updateFrameUi;
using dart::gui::detail::updateFrameViewport;

bool hasSceneOption(int argc, char* argv[]);

bool isTruthyEnvironmentVariable(const char* name)
{
  const char* value = std::getenv(name);
  if (value == nullptr) {
    return false;
  }

  const std::string_view text(value);
  return text == "1" || text == "ON" || text == "on" || text == "TRUE"
         || text == "true" || text == "YES" || text == "yes";
}

void discardFilamentLog(void*, const char*) {}

void configureFilamentLogging()
{
  if (isTruthyEnvironmentVariable("DART_VERBOSE")
      || isTruthyEnvironmentVariable("DART_GUI_VERBOSE")) {
    return;
  }

  ::utils::slog.d.setConsumer(discardFilamentLog, nullptr);
  ::utils::slog.i.setConsumer(discardFilamentLog, nullptr);
  ::utils::slog.v.setConsumer(discardFilamentLog, nullptr);
}

int runGuiBackendApplicationImpl(
    int argc,
    char* argv[],
    const dart::gui::ApplicationOptions& applicationOptions)
{
  AppOptions appOptions
      = parseOptions(argc, argv, applicationOptions.runDefaults);
  const bool renderOutputModeExplicit = appOptions.renderOutputModeExplicit;
  const auto renderOutputMode = appOptions.renderSettings.outputMode;
  appOptions.camera = applicationOptions.camera;
  appOptions.cameraControlsProvider = applicationOptions.cameraControlsProvider;
  appOptions.cameraUpdater = applicationOptions.cameraUpdater;
  appOptions.viewportLayoutProvider = applicationOptions.viewportLayoutProvider;
  if (!hasSceneOption(argc, argv)) {
    appOptions.world = applicationOptions.world;
    appOptions.renderableProvider = applicationOptions.renderableProvider;
    appOptions.selectedRenderableProvider
        = applicationOptions.selectedRenderableProvider;
    appOptions.onRenderableSelected = applicationOptions.onRenderableSelected;
    appOptions.dockingEnabled = applicationOptions.dockingEnabled;
    appOptions.preStep = applicationOptions.preStep;
    appOptions.postStep = applicationOptions.postStep;
    appOptions.renderSettings = applicationOptions.renderSettings;
    if (renderOutputModeExplicit) {
      appOptions.renderSettings.outputMode = renderOutputMode;
    }
    appOptions.gizmos = applicationOptions.gizmos;
    appOptions.debugLabels = applicationOptions.debugLabels;
    appOptions.ikHandles = applicationOptions.ikHandles;
    appOptions.bodyNodeDragHandles = applicationOptions.bodyNodeDragHandles;
    appOptions.keyboardActions = applicationOptions.keyboardActions;
  }
  appOptions.preRender = applicationOptions.preRender;
  appOptions.postRender = applicationOptions.postRender;
  appOptions.simulateWorld = applicationOptions.simulateWorld;
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

  configureFilamentLogging();

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
  auto* scene = renderContext.scene;
  auto* colorGrading = createDebugColorGrading(*engine);
  for (auto* view : renderContext.views) {
    if (view != nullptr) {
      configureMainView(*view, colorGrading, runOptions.headless);
    }
  }
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
  // Mutable so the F2 key can toggle the perf HUD at runtime (initialized from
  // the --perf-hud option).
  bool showPerfHud = appOptions.showPerfHud;
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
  lifecycle.frameOutputDirectory = runOptions.frameOutputDirectory;
  lifecycle.frameOutputEnabled = !runOptions.frameOutputDirectory.empty();
  ApplicationInputState inputState;
  SelectionController selectionController;
  ScreenshotCapture screenshotCapture;
  ProfileAccumulator profile;
  PerfHudState perfHud;
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
    DART_PROFILE_FRAME;
    DART_PROFILE_SCOPED_N("GUI frame");
    const auto frameStart = ProfileAccumulator::Clock::now();
    auto phaseStart = ProfileAccumulator::Clock::now();
    {
      DART_PROFILE_SCOPED_N("GUI input");
      pollApplicationInput(
          window,
          dartScene,
          selectionController,
          lifecycle,
          cameraController,
          homeCamera,
          inputState,
          showPerfHud);
    }
    profile.inputMs += elapsedMs(phaseStart);

    if (appOptions.cameraUpdater) {
      appOptions.cameraUpdater(cameraController.camera);
    }

    phaseStart = ProfileAccumulator::Clock::now();
    FrameViewport viewport;
    {
      DART_PROFILE_SCOPED_N("GUI viewport camera");
      const OrbitCameraControlOptions cameraControls
          = appOptions.cameraControlsProvider
                ? appOptions.cameraControlsProvider()
                : OrbitCameraControlOptions{};
      dart::gui::ViewportLayoutOptions viewportLayout
          = appOptions.viewportLayoutProvider
                ? appOptions.viewportLayoutProvider(cameraController.camera)
                : dart::gui::ViewportLayoutOptions{};
      if (!appOptions.viewportLayoutProvider) {
        viewportLayout.panes[0].camera = cameraController.camera;
        viewportLayout.panes[0].active = true;
      }
      viewport = updateFrameViewport(
          window,
          renderContext.views,
          renderContext.cameras,
          cameraController,
          selectionController,
          imguiIo,
          runOptions.width,
          runOptions.height,
          dartScene.world->getTimeStep(),
          appOptions.showUi,
          runOptions.guiScale,
          cameraControls,
          viewportLayout);
      renderContext.activeViewCount = viewport.paneCount;
    }
    profile.viewportCameraMs += elapsedMs(phaseStart);

    const bool uiCapturesMouse
        = isSceneMouseInputCapturedByUi(appOptions.showUi, imguiIo);
    {
      DART_PROFILE_SCOPED_N("GUI scene update");
      sceneFrameUpdater.update(
          viewport,
          appOptions.showUi,
          uiCapturesMouse,
          orbitLight,
          appOptions.orbitLightPeriodSeconds);
    }

    if (appOptions.showUi) {
      DART_PROFILE_SCOPED_N("GUI ui update");
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
          profile,
          showPerfHud,
          perfHud,
          renderContext.backendName);
    }
    setSceneLightsEnabled(*engine, lights, headlightsEnabled);
    for (std::size_t i = 0; i < renderContext.views.size(); ++i) {
      if (renderContext.views[i] != nullptr) {
        applyRenderSettings(*renderContext.views[i], dartScene.renderSettings);
      }
    }

    if (appOptions.preRender) {
      appOptions.preRender();
    }
    FrameRenderResult frameRenderResult;
    {
      DART_PROFILE_SCOPED_N("GUI render frame");
      frameRenderResult = renderApplicationFrame(
          renderContext,
          appOptions.showUi ? imguiOverlay.view : nullptr,
          runOptions,
          viewport.width,
          viewport.height,
          frameStart,
          screenshotCapture,
          lifecycle,
          profile);
    }
    if (showPerfHud) {
      const double gpuMs = latestGpuFrameMs(renderContext);
      if (gpuMs > 0.0) {
        profile.gpuFrameMs = gpuMs;
        if (gpuMs > profile.maxGpuFrameMs) {
          profile.maxGpuFrameMs = gpuMs;
        }
      }
    }
    if (!frameRenderResult.failed && appOptions.postRender) {
      appOptions.postRender();
    }
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
    return detail::runGuiBackendApplication(argc, argv, applicationOptions);
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

  return detail::runGuiBackendApplication(
      static_cast<int>(rewrittenArguments.size()),
      rewrittenArguments.data(),
      applicationOptions);
}

} // namespace dart::gui

namespace dart::gui::detail {

int runGuiBackendApplication(int argc, char* argv[])
{
  return runGuiBackendApplication(argc, argv, dart::gui::ApplicationOptions{});
}

int runGuiBackendApplication(
    int argc,
    char* argv[],
    const dart::gui::ApplicationOptions& applicationOptions)
{
  return runGuiBackendApplicationImpl(argc, argv, applicationOptions);
}

} // namespace dart::gui::detail

namespace dart::gui::detail {

int runGuiApplication(int argc, char* argv[])
{
  return dart::gui::runApplication(argc, argv);
}

} // namespace dart::gui::detail
