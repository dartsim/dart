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
#include <dart/gui/experimental/detail/filament/scenes.hpp>
#include <dart/gui/experimental/detail/filament/selection.hpp>
#include <dart/all.hpp>
#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/profile.hpp>
#include <dart/config.hpp>
#include <dart/gui/experimental/detail/filament/debug_overlay.hpp>
#include <dart/gui/experimental/detail/filament/frame_renderer.hpp>
#include <dart/gui/experimental/detail/filament/imgui_overlay.hpp>
#include <dart/gui/experimental/detail/filament/input.hpp>
#include <dart/gui/experimental/detail/filament/native_window.hpp>
#include <dart/gui/experimental/detail/filament/panel.hpp>
#include <dart/gui/experimental/detail/filament/renderable_factory.hpp>
#include <dart/gui/experimental/detail/filament/renderable_resources.hpp>
#include <dart/gui/experimental/detail/filament/renderable_sync.hpp>
#include <dart/gui/experimental/detail/filament/render_environment.hpp>
#include <dart/gui/experimental/detail/filament/render_context.hpp>
#include <dart/gui/experimental/detail/filament/scene_requirements.hpp>
#include <dart/gui/experimental/detail/filament/screenshot.hpp>
#include <dart/gui/experimental/detail/filament/simulation_stepper.hpp>
#include <dart/gui/experimental/profile.hpp>
#include <dart/gui/experimental/scene.hpp>
#include <dart/io/read.hpp>
#include <dart/utils/composite_resource_retriever.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/http_resource_retriever.hpp>
#include <dart/utils/mesh_loader.hpp>
#include <dart/utils/package_resource_retriever.hpp>
#include <dart/utils/urdf/All.hpp>

#include <GLFW/glfw3.h>

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cstdint>
#include <cstdlib>

namespace {

using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::ConvexMeshShape;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::HeightmapShaped;
using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::PointCloudShape;
using dart::dynamics::Shape;
using dart::dynamics::ShapePtr;
using dart::dynamics::ShapeNode;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Skeleton;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::SphereShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;

#if DART_HAVE_OCTOMAP
using dart::dynamics::VoxelGridShape;
#endif

using dart::gui::experimental::DebugDrawOptions;
using dart::gui::experimental::OrbitCamera;
using dart::gui::experimental::OrbitCameraController;
using dart::gui::experimental::ProfileAccumulator;
using dart::gui::experimental::RenderableDescriptor;
using dart::gui::experimental::RenderableId;
using dart::gui::experimental::RunOptions;
using dart::gui::experimental::ShapeKind;
using dart::gui::experimental::ViewerLifecycleState;
using dart::gui::experimental::elapsedMs;
using dart::gui::experimental::extractContactDebugLines;
using dart::gui::experimental::extractDebugLines;
using dart::gui::experimental::extractRenderables;
using dart::gui::experimental::markSimulationAdvanced;
using dart::gui::experimental::makeRenderableId;
using dart::gui::experimental::normalizeRunOptions;
using dart::gui::experimental::printProfile;
using dart::gui::experimental::filament::ApplicationInputState;
using dart::gui::experimental::filament::FilamentRenderContext;
using dart::gui::experimental::filament::FrameRenderResult;
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
using dart::gui::experimental::filament::addRenderableToScene;
using dart::gui::experimental::filament::accumulateSceneContent;
using dart::gui::experimental::filament::attachSceneEnvironment;
using dart::gui::experimental::filament::clearDebugLineOverlay;
using dart::gui::experimental::filament::clearMainViewColorGrading;
using dart::gui::experimental::filament::configureMainView;
using dart::gui::experimental::filament::configureViewportCamera;
using dart::gui::experimental::filament::countSceneContent;
using dart::gui::experimental::filament::createFilamentRenderContext;
using dart::gui::experimental::filament::createDebugColorGrading;
using dart::gui::experimental::filament::createConfiguredImGuiOverlay;
using dart::gui::experimental::filament::createNeutralIndirectLight;
using dart::gui::experimental::filament::createNeutralSkybox;
using dart::gui::experimental::filament::createMaterialResources;
using dart::gui::experimental::filament::createRenderableFromDescriptor;
using dart::gui::experimental::filament::createSceneLights;
using dart::gui::experimental::filament::destroyMaterialResources;
using dart::gui::experimental::filament::destroyRenderable;
using dart::gui::experimental::filament::destroyRenderEnvironmentResources;
using dart::gui::experimental::filament::destroyFilamentRenderContext;
using dart::gui::experimental::filament::destroyConfiguredImGuiOverlay;
using dart::gui::experimental::filament::destroySceneLights;
using dart::gui::experimental::filament::detachSceneEnvironment;
using dart::gui::experimental::filament::getNativeWindow;
using dart::gui::experimental::filament::handleScroll;
using dart::gui::experimental::filament::isInsideStatusPanel;
using dart::gui::experimental::filament::pollApplicationInput;
using dart::gui::experimental::filament::renderBuiltInStatusPanel;
using dart::gui::experimental::filament::renderApplicationFrame;
using dart::gui::experimental::filament::refreshDebugLineOverlay;
using dart::gui::experimental::filament::refreshSelectionDebugLineOverlay;
using dart::gui::experimental::filament::saveScreenshot;
using dart::gui::experimental::filament::logUnsupportedRenderableDescriptorOnce;
using dart::gui::experimental::filament::removeRenderableFromScene;
using dart::gui::experimental::filament::setRenderableTransform;
using dart::gui::experimental::filament::synchronizeSceneRenderables;
using dart::gui::experimental::filament::updateCameraController;
using dart::gui::experimental::filament::updateImGuiOverlay;
using dart::gui::experimental::filament::updateImGuiMouseInput;
using dart::gui::experimental::filament::updateOrbitingKeyLight;
using dart::gui::experimental::filament::updateSceneRenderableFromDescriptor;
using dart::gui::experimental::filament::waitForScreenshot;
using dart::simulation::World;
using dart::gui::experimental::filament::AppOptions;
using dart::gui::experimental::filament::DartScene;
using dart::gui::experimental::filament::ExampleScene;
using dart::gui::experimental::filament::createDartScene;
using dart::gui::experimental::filament::initialCameraForScene;
using dart::gui::experimental::filament::parseOptions;
using dart::gui::experimental::filament::sceneName;
using dart::gui::experimental::filament::validateCreatedSceneContent;
using dart::gui::experimental::filament::validateSceneDescriptorContent;

int runFilamentGuiApplicationImpl(int argc, char* argv[])
{
  const AppOptions appOptions = parseOptions(argc, argv);
  const RunOptions& options = appOptions.run;

  GLFWwindow* window = nullptr;
#if defined(__linux__)
  if (!options.headless) {
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
  }
#endif
  if (!options.headless) {
    if (!glfwInit()) {
      std::cerr << "Failed to initialize GLFW\n";
      return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window = glfwCreateWindow(
        options.width,
        options.height,
        "DART + Filament (experimental)",
        nullptr,
        nullptr);
    if (window == nullptr) {
      std::cerr << "Failed to create GLFW window\n";
      glfwTerminate();
      return 1;
    }
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
  SceneContentCounts createdSceneContent;
  for (const RenderableDescriptor& descriptor : initialDescriptors) {
    if (!descriptor.material.visible) {
      continue;
    }

    auto renderable = createRenderableFromDescriptor(
        *engine, materials, materialResources.textureCache, descriptor);
    if (!renderable) {
      if (descriptor.geometry.kind == ShapeKind::Unsupported) {
        logUnsupportedRenderableDescriptorOnce(
            loggedUnsupportedRenderableIds, descriptor);
      }
      continue;
    }
    accumulateSceneContent(createdSceneContent, descriptor);

    SceneRenderable sceneRenderable;
    sceneRenderable.id = descriptor.id;
    sceneRenderable.renderable = *renderable;
    addRenderableToScene(*scene, sceneRenderable.renderable);
    setRenderableTransform(
        *engine, sceneRenderable.renderable, descriptor.worldTransform);
    sceneRenderables.push_back(sceneRenderable);
  }
  if (sceneRenderables.empty()) {
    std::cerr << "No supported visible DART renderables were extracted\n";
    return 1;
  }
  if (!validateCreatedSceneContent(
          appOptions.scene,
          expectedSceneContent,
          createdSceneContent,
          std::cerr)) {
    return 1;
  }

  DebugDrawOptions staticDebugOptions;
  staticDebugOptions.drawBodyFrames = true;
  staticDebugOptions.drawCentersOfMass = true;
  staticDebugOptions.drawInertiaBoxes = false;
  staticDebugOptions.drawCollisionShapeBounds = false;
  staticDebugOptions.drawSupportPolygons
      = appOptions.scene == ExampleScene::G1;
  staticDebugOptions.drawContacts = false;
  std::optional<Renderable> debugOverlay;
  auto refreshDebugOverlay = [&]() {
    refreshDebugLineOverlay(
        *engine,
        *scene,
        materials.debugColor,
        extractDebugLines(*dartScene.world, staticDebugOptions),
        debugOverlay);
  };
  refreshDebugOverlay();
  if (!debugOverlay) {
    std::cerr << "No debug overlay lines were extracted\n";
    return 1;
  }

  DebugDrawOptions contactDebugOptions;
  contactDebugOptions.drawGrid = false;
  contactDebugOptions.drawWorldFrame = false;
  contactDebugOptions.drawBodyFrames = false;
  contactDebugOptions.drawCentersOfMass = false;
  std::optional<Renderable> contactDebugOverlay;
  auto refreshContactDebugOverlay = [&]() {
    refreshDebugLineOverlay(
        *engine,
        *scene,
        materials.debugColor,
        extractContactDebugLines(
            dartScene.world->getLastCollisionResult(), contactDebugOptions),
        contactDebugOverlay);
  };

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
  bool screenshotSucceeded = options.screenshotPath.empty();
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
    int width = options.width;
    int height = options.height;
    if (window != nullptr) {
      glfwGetFramebufferSize(window, &width, &height);
    }
    width = std::max(1, width);
    height = std::max(1, height);
    imguiIo.DisplaySize
        = ImVec2(static_cast<float>(width), static_cast<float>(height));
    imguiIo.DeltaTime = static_cast<float>(dartScene.world->getTimeStep());
    configureViewportCamera(*view, *camera, cameraController.camera, width, height);
    if (window != nullptr) {
      double cursorX = 0.0;
      double cursorY = 0.0;
      glfwGetCursorPos(window, &cursorX, &cursorY);
      const bool suppressCameraOrbit
          = selectionController.isDraggingSelection()
            || (appOptions.showUi
                && isInsideStatusPanel(cursorX, cursorY, options.guiScale));
      updateCameraController(window, cameraController, suppressCameraOrbit);
    }
    profile.viewportCameraMs += elapsedMs(phaseStart);

    const std::size_t simulationStepsToRun
        = simulationStepper.stepsToRun(
            options, lifecycle, dartScene.world->getTimeStep());

    if (simulationStepsToRun > 0) {
      phaseStart = ProfileAccumulator::Clock::now();
      for (std::size_t i = 0; i < simulationStepsToRun; ++i) {
        const double timeStep = dartScene.world->getTimeStep();
        dartScene.world->step();
        profile.simulatedMs += timeStep * 1000.0;
      }
      markSimulationAdvanced(lifecycle);
      profile.simulationSteps += simulationStepsToRun;
      profile.simulationMs += elapsedMs(phaseStart);

      phaseStart = ProfileAccumulator::Clock::now();
      refreshContactDebugOverlay();
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
    bool selectedRenderableStillVisible
        = selectionController.selectedRenderableId() == 0;
    for (SceneRenderable& sceneRenderable : sceneRenderables) {
      const auto descriptor = std::find_if(
          descriptors.begin(),
          descriptors.end(),
          [&](const RenderableDescriptor& candidate) {
            return candidate.id == sceneRenderable.id;
          });
      if (descriptor == descriptors.end() || !descriptor->material.visible) {
        continue;
      }

      const bool isSelected
          = descriptor->id == selectionController.selectedRenderableId();
      if (isSelected) {
        selectedRenderableStillVisible = true;
      }
      updateSceneRenderableFromDescriptor(
          *engine, sceneRenderable, *descriptor, isSelected);
    }
    if (!selectedRenderableStillVisible) {
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
        width,
        height,
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
      phaseStart = ProfileAccumulator::Clock::now();
      updateImGuiMouseInput(window, imguiIo, width, height);
      ImGui::NewFrame();
      const bool debugOptionsChanged = renderBuiltInStatusPanel(
          sceneName(appOptions.scene),
          dartScene.world->getTime(),
          dartScene.world->getLastCollisionResult().getNumContacts(),
          selectionController.selectedLabel(),
          !dartScene.ikHandles.empty(),
          orbitLight,
          staticDebugOptions,
          contactDebugOptions,
          lifecycle,
          guiScale);
      if (debugOptionsChanged) {
        refreshDebugOverlay();
        refreshContactDebugOverlay();
      }
      ImGui::Render();
      updateImGuiOverlay(
          *engine,
          imguiOverlay,
          ImGui::GetDrawData(),
          static_cast<std::uint32_t>(width),
          static_cast<std::uint32_t>(height));
      profile.uiMs += elapsedMs(phaseStart);
    }

    const FrameRenderResult frameRenderResult = renderApplicationFrame(
        renderContext,
        appOptions.showUi ? imguiOverlay.view : nullptr,
        options,
        width,
        height,
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

  if (!options.screenshotPath.empty() && !lifecycle.screenshotRequested) {
    std::cerr << "No rendered frame was available for screenshot capture\n";
  }
  if (lifecycle.screenshotRequested) {
    const auto screenshotWaitStart = ProfileAccumulator::Clock::now();
    screenshotSucceeded = waitForScreenshot(renderContext, screenshotCapture);
    profile.screenshotWaitMs += elapsedMs(screenshotWaitStart);
    if (screenshotSucceeded) {
      const auto screenshotSaveStart = ProfileAccumulator::Clock::now();
      saveScreenshot(screenshotCapture, options.screenshotPath);
      profile.screenshotSaveMs += elapsedMs(screenshotSaveStart);
    } else {
      std::cerr << "Timed out waiting for Filament screenshot readback\n";
    }
  }
  if (options.maxFrames >= 0) {
    std::cout << "Final contacts: "
              << dartScene.world->getLastCollisionResult().getNumContacts()
              << "\n";
  }
  if (appOptions.profile) {
    printProfile(profile);
    DART_PROFILE_TEXT_DUMP();
  }

  destroyConfiguredImGuiOverlay(*engine, imguiOverlay);

  detachSceneEnvironment(*scene, lights);
  clearMainViewColorGrading(*view);
  for (const SceneRenderable& sceneRenderable : sceneRenderables) {
    removeRenderableFromScene(*scene, sceneRenderable.renderable);
  }
  clearDebugLineOverlay(*engine, *scene, debugOverlay);
  clearDebugLineOverlay(*engine, *scene, contactDebugOverlay);
  clearDebugLineOverlay(*engine, *scene, selectionDebugOverlay);
  destroySceneLights(*engine, lights);
  destroyRenderEnvironmentResources(
      *engine, indirectLight, skybox, colorGrading);
  for (SceneRenderable& sceneRenderable : sceneRenderables) {
    destroyRenderable(*engine, sceneRenderable.renderable);
  }
  destroyMaterialResources(*engine, materialResources);
  destroyFilamentRenderContext(renderContext);

  if (window != nullptr) {
    glfwDestroyWindow(window);
    glfwTerminate();
  }
  return screenshotSucceeded ? 0 : 1;
}

} // namespace

namespace dart::gui::experimental::filament {

int runFilamentGuiApplication(int argc, char* argv[])
{
  return runFilamentGuiApplicationImpl(argc, argv);
}

} // namespace dart::gui::experimental::filament
