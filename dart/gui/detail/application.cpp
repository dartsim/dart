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
#include <dart/gui/detail/gui_scale.hpp>
#include <dart/gui/detail/imgui_overlay.hpp>
#include <dart/gui/detail/input.hpp>
#include <dart/gui/detail/native_window.hpp>
#include <dart/gui/detail/offscreen_parity.hpp>
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

#include <array>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace {

using dart::gui::elapsedMs;
using dart::gui::OrbitCameraController;
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
using dart::gui::detail::destroyPersistentApplicationResources;
using dart::gui::detail::destroySceneRenderables;
using dart::gui::detail::ExampleScene;
using dart::gui::detail::FilamentRenderContext;
using dart::gui::detail::finalizeScreenshotCapture;
using dart::gui::detail::FrameRenderResult;
using dart::gui::detail::FrameViewport;
using dart::gui::detail::getCurrentImGuiIo;
using dart::gui::detail::getNativeWindow;
using dart::gui::detail::GuiScaleState;
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
using dart::gui::detail::resizeAutomaticApplicationWindow;
using dart::gui::detail::resolveWindowDpiScale;
using dart::gui::detail::runOffscreenParitySelfCheck;
using dart::gui::detail::SceneFrameUpdater;
using dart::gui::detail::SceneLights;
using dart::gui::detail::ScreenshotCapture;
using dart::gui::detail::SelectionController;
using dart::gui::detail::shouldContinueApplicationLoop;
using dart::gui::detail::SimulationStepper;
using dart::gui::detail::updateConfiguredImGuiOverlayScale;
using dart::gui::detail::updateFrameUi;
using dart::gui::detail::updateFrameViewport;

// Number of frames each scene renders in `--cycle-scenes` mode before the host
// advances to the next scene. A few frames are enough to exercise scene build,
// extraction, render, and teardown without making the smoke test slow.
constexpr int kCycleFramesPerScene = 4;

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

// Copy the per-scene fields from a user-provided ApplicationOptions into the
// parsed AppOptions. Used by the demos host when (re)building the active scene.
void applySceneOptions(
    AppOptions& appOptions,
    const dart::gui::ApplicationOptions& src,
    bool renderOutputModeExplicit,
    dart::gui::RenderOutputMode renderOutputMode)
{
  appOptions.world = src.world;
  appOptions.renderableProvider = src.renderableProvider;
  appOptions.dockingEnabled = src.dockingEnabled;
  appOptions.preStep = src.preStep;
  appOptions.postStep = src.postStep;
  appOptions.renderSettings = src.renderSettings;
  if (renderOutputModeExplicit) {
    appOptions.renderSettings.outputMode = renderOutputMode;
  }
  appOptions.gizmos = src.gizmos;
  appOptions.debugLabels = src.debugLabels;
  appOptions.ikHandles = src.ikHandles;
  appOptions.bodyNodeDragHandles = src.bodyNodeDragHandles;
  appOptions.keyboardActions = src.keyboardActions;
  appOptions.preRender = src.preRender;
  appOptions.postRender = src.postRender;
  appOptions.simulateWorld = src.simulateWorld;
  appOptions.panels = src.panels;
  appOptions.camera = src.camera;
  appOptions.cameraControlsProvider = src.cameraControlsProvider;
  appOptions.cameraUpdater = src.cameraUpdater;
  appOptions.viewportLayoutProvider = src.viewportLayoutProvider;
  appOptions.onViewportPaneActivated = src.onViewportPaneActivated;
}

int demoSceneIndex(
    const std::vector<dart::gui::DemoSceneEntry>& scenes,
    std::string_view id,
    int fallback)
{
  for (std::size_t i = 0; i < scenes.size(); ++i) {
    if (scenes[i].id == id) {
      return static_cast<int>(i);
    }
  }
  return fallback;
}

// Built-in sidebar panel listing the demo catalog grouped by category (ordered
// by first appearance). Selecting a different scene requests a runtime switch.
dart::gui::Panel makeDemoSidebarPanel(
    const std::vector<dart::gui::DemoSceneEntry>& scenes, int activeIndex)
{
  dart::gui::Panel panel;
  panel.title = "Demos";
  panel.dockSide = dart::gui::DockSide::Left;
  panel.initialPosition = std::array<double, 2>{12.0, 12.0};
  panel.initialSize = std::array<double, 2>{280.0, 560.0};
  panel.autoResize = false;
  panel.buildWithContext = [&scenes, activeIndex](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    if (activeIndex >= 0 && activeIndex < static_cast<int>(scenes.size())) {
      const auto& active = scenes[static_cast<std::size_t>(activeIndex)];
      builder.text("Current: " + active.title);
      if (!active.summary.empty()) {
        builder.text(active.summary);
      }
    }
    builder.separator();

    // Tree-style catalog: each category is a collapsible header, scenes
    // within are indented list rows (selectable text instead of buttons).
    std::string lastCategory;
    bool categoryOpen = true;
    bool inCategory = false;
    for (std::size_t i = 0; i < scenes.size(); ++i) {
      const auto& entry = scenes[i];
      if (i == 0 || entry.category != lastCategory) {
        if (inCategory) {
          builder.unindent();
          inCategory = false;
        }
        lastCategory = entry.category;
        categoryOpen = builder.collapsingHeader(entry.category, true);
        if (categoryOpen) {
          builder.indent();
          inCategory = true;
        }
      }
      if (!categoryOpen) {
        continue;
      }
      const bool isActive = static_cast<int>(i) == activeIndex;
      const std::string label = entry.title + "##demo_" + entry.id;
      const bool clicked = builder.selectable(label, isActive);
      // Explain what each scene demonstrates when the row is hovered.
      builder.itemTooltip(entry.summary);
      if (clicked && !isActive && context.lifecycle != nullptr) {
        dart::gui::requestSceneSwitch(*context.lifecycle, entry.id);
      }
    }
    if (inCategory) {
      builder.unindent();
    }
  };
  return panel;
}

int runGuiBackendApplicationImpl(
    int argc,
    char* argv[],
    const dart::gui::ApplicationOptions& applicationOptions,
    const std::vector<dart::gui::DemoSceneEntry>* demoCatalog,
    int initialDemoIndex,
    bool cycleScenes)
{
  AppOptions appOptions
      = parseOptions(argc, argv, applicationOptions.runDefaults);
  const bool renderOutputModeExplicit = appOptions.renderOutputModeExplicit;
  const auto renderOutputMode = appOptions.renderSettings.outputMode;
  appOptions.camera = applicationOptions.camera;
  // Single-scene wiring (one world/options passed in). The demos host instead
  // (re)applies each scene's options inside the scene loop below.
  if (demoCatalog == nullptr) {
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
    appOptions.cameraControlsProvider
        = applicationOptions.cameraControlsProvider;
    appOptions.cameraUpdater = applicationOptions.cameraUpdater;
    appOptions.viewportLayoutProvider
        = applicationOptions.viewportLayoutProvider;
    appOptions.onViewportPaneActivated
        = applicationOptions.onViewportPaneActivated;
  }
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

  ApplicationWindow appWindow = createApplicationWindow(
      runOptions,
      !appOptions.windowWidthExplicit,
      !appOptions.windowHeightExplicit,
      std::cerr);
  GLFWwindow* window = appWindow.get();
  if (!runOptions.headless && window == nullptr) {
    return 1;
  }

  OrbitCameraController cameraController;
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

  GuiScaleState guiScale = dart::gui::detail::makeGuiScaleState(
      runOptions.guiScale, resolveWindowDpiScale(window));
  double automaticWindowEffectiveScale = guiScale.effectiveScale;
  bool automaticWindowSizeResolved
      = runOptions.headless
        || (appOptions.windowWidthExplicit && appOptions.windowHeightExplicit);
  ImGuiOverlay imguiOverlay = createConfiguredImGuiOverlay(
      *engine, static_cast<float>(guiScale.effectiveScale));
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

  int activeIndex = initialDemoIndex;
  bool keepRunning = true;
  std::size_t finalContacts = 0;

  // Outer scene loop. For the single-scene path this runs exactly once; for the
  // demos host it rebuilds the scene-bound state when a switch is requested,
  // while the window, engine, materials, lights, and ImGui overlay persist.
  while (keepRunning) {
    if (demoCatalog != nullptr) {
      const auto& demoEntry
          = (*demoCatalog)[static_cast<std::size_t>(activeIndex)];
      dart::gui::ApplicationOptions sceneOptions;
      try {
        sceneOptions = demoEntry.factory();
      } catch (const std::exception& error) {
        // Soft-fail: a scene that cannot build (e.g. a missing asset) must not
        // crash the host. Show an empty world so the sidebar stays usable.
        std::cerr << "demo scene '" << demoEntry.id
                  << "' failed to load: " << error.what() << "\n";
        sceneOptions = dart::gui::ApplicationOptions{};
      }
      if (sceneOptions.world == nullptr) {
        sceneOptions.world = dart::simulation::World::create("(empty)");
      }
      applySceneOptions(
          appOptions, sceneOptions, renderOutputModeExplicit, renderOutputMode);
      std::vector<dart::gui::Panel> panels;
      panels.reserve(appOptions.panels.size() + 1);
      panels.push_back(makeDemoSidebarPanel(*demoCatalog, activeIndex));
      for (auto& scenePanel : appOptions.panels) {
        panels.push_back(std::move(scenePanel));
      }
      appOptions.panels = std::move(panels);
      lifecycle.sceneSwitchRequested = false;
      lifecycle.requestedScene.clear();
      // Per-scene transient controllers are reset; renderable ids from the
      // previous scene must not leak into the next one.
      selectionController.clear();
      simulationStepper = SimulationStepper{};
    }

    cameraController.camera
        = appOptions.camera.value_or(initialCameraForScene(appOptions.scene));
    if (appOptions.cameraUpdater) {
      appOptions.cameraUpdater(cameraController.camera);
    }
    const auto homeCamera = cameraController.camera;

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
            applicationOptions.allowEmptyScene,
            std::cerr);
    if (!maybeInitialSceneState) {
      frameCaptureSucceeded = false;
      keepRunning = false;
      break;
    }
    InitialSceneState sceneState = std::move(*maybeInitialSceneState);
    auto& sceneRenderables = sceneState.sceneRenderables;
    auto& debugOverlays = sceneState.debugOverlays;

    std::optional<Renderable> selectionDebugOverlay;

    SceneFrameUpdater sceneFrameUpdater(
        window,
        *engine,
        *scene,
        materials,
        materialResources,
        runOptions,
        dartScene,
        selectionController,
        sceneState,
        selectionDebugOverlay,
        lifecycle,
        simulationStepper,
        lights,
        orbitStartClock,
        profile);

    bool sceneFrameFailed = false;
    bool cycleAdvance = false;
    int framesThisScene = 0;

    while (!lifecycle.exitRequested && !lifecycle.sceneSwitchRequested
           && !cycleAdvance
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

      guiScale = dart::gui::detail::makeGuiScaleState(
          runOptions.guiScale, resolveWindowDpiScale(window));
      if (!automaticWindowSizeResolved
          || std::abs(guiScale.effectiveScale - automaticWindowEffectiveScale)
                 > 1e-4) {
        resizeAutomaticApplicationWindow(
            window,
            runOptions,
            guiScale,
            !appOptions.windowWidthExplicit,
            !appOptions.windowHeightExplicit);
        automaticWindowEffectiveScale = guiScale.effectiveScale;
        automaticWindowSizeResolved = true;
      }
      updateConfiguredImGuiOverlayScale(
          *engine, imguiOverlay, static_cast<float>(guiScale.effectiveScale));

      phaseStart = ProfileAccumulator::Clock::now();
      FrameViewport viewport;
      {
        DART_PROFILE_SCOPED_N("GUI viewport camera");
        const dart::gui::OrbitCameraControlOptions cameraControls
            = appOptions.cameraControlsProvider
                  ? appOptions.cameraControlsProvider()
                  : dart::gui::OrbitCameraControlOptions{};
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
          applyRenderSettings(
              *renderContext.views[i], dartScene.renderSettings);
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
        sceneFrameFailed = true;
        break;
      }
      if (frameRenderResult.continueLoop) {
        continue;
      }
      if (frameRenderResult.stopLoop) {
        break;
      }
      if (cycleScenes && ++framesThisScene >= kCycleFramesPerScene) {
        cycleAdvance = true;
      }
    }

    finalContacts = dartScene.world->getLastCollisionResult().getNumContacts();
    destroySceneRenderables(
        *engine,
        *scene,
        sceneRenderables,
        debugOverlays,
        selectionDebugOverlay);

    if (sceneFrameFailed) {
      frameCaptureSucceeded = false;
      keepRunning = false;
    } else if (demoCatalog != nullptr && lifecycle.sceneSwitchRequested) {
      activeIndex
          = demoSceneIndex(*demoCatalog, lifecycle.requestedScene, activeIndex);
    } else if (demoCatalog != nullptr && cycleScenes && cycleAdvance) {
      if (activeIndex + 1 < static_cast<int>(demoCatalog->size())) {
        ++activeIndex;
      } else {
        keepRunning = false;
      }
    } else {
      keepRunning = false;
    }
  }

  const bool screenshotSucceeded = finalizeScreenshotCapture(
      renderContext,
      screenshotCapture,
      runOptions.screenshotPath,
      lifecycle.screenshotRequested,
      profile);

  // Phase-1 gate of the Filament offscreen-viewport spike: when
  // DART_GUI_OFFSCREEN_PARITY is set, render the live scene to an offscreen
  // RenderTarget and confirm it matches the swapchain render (diagnostic only;
  // see docs/design/dartsim_gui_toolkit_decisions.md Decision 3).
  bool offscreenParitySucceeded = true;
  if (runOptions.headless
      && isTruthyEnvironmentVariable("DART_GUI_OFFSCREEN_PARITY")) {
    if (std::string_view(renderContext.backendName) == "noop") {
      std::cout << "[offscreen-parity] SKIP: noop backend produces no pixels\n";
    } else {
      offscreenParitySucceeded = runOffscreenParitySelfCheck(
          renderContext,
          static_cast<std::uint32_t>(
              runOptions.width > 0 ? runOptions.width : 1),
          static_cast<std::uint32_t>(
              runOptions.height > 0 ? runOptions.height : 1),
          std::cout);
    }
  }

  if (runOptions.maxFrames >= 0) {
    std::cout << "Final contacts: " << finalContacts << "\n";
  }
  if (appOptions.profile) {
    printProfile(profile);
    DART_PROFILE_TEXT_DUMP();
  }

  destroyPersistentApplicationResources(
      renderContext,
      imguiOverlay,
      lights,
      indirectLight,
      skybox,
      colorGrading,
      materialResources);

  return screenshotSucceeded && frameCaptureSucceeded
                 && offscreenParitySucceeded
             ? 0
             : 1;
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

int runDemos(int argc, char* argv[], std::vector<DemoSceneEntry> scenes)
{
  if (scenes.empty()) {
    std::cerr << "runDemos: no scenes registered\n";
    return 1;
  }

  std::string initialId;
  if (const char* env = std::getenv("DART_DEMOS_SCENE")) {
    initialId = env;
  }
  bool cycleScenes = false;

  std::vector<char*> filteredArguments;
  filteredArguments.reserve(static_cast<std::size_t>(argc));
  if (argc > 0) {
    filteredArguments.push_back(argv[0]);
  }
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i] == nullptr ? "" : argv[i];
    if (arg == "--cycle-scenes") {
      cycleScenes = true;
      continue;
    }
    if (arg == "--scene") {
      if (i + 1 < argc && argv[i + 1] != nullptr) {
        initialId = argv[i + 1];
        ++i;
      }
      continue;
    }
    filteredArguments.push_back(argv[i]);
  }

  int index = 0;
  if (!initialId.empty()) {
    index = ::demoSceneIndex(scenes, initialId, 0);
  }

  return runGuiBackendApplicationImpl(
      static_cast<int>(filteredArguments.size()),
      filteredArguments.data(),
      ApplicationOptions{},
      &scenes,
      index,
      cycleScenes);
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
  return runGuiBackendApplicationImpl(
      argc, argv, applicationOptions, nullptr, 0, false);
}

} // namespace dart::gui::detail

namespace dart::gui::detail {

int runGuiApplication(int argc, char* argv[])
{
  return dart::gui::runApplication(argc, argv);
}

} // namespace dart::gui::detail
