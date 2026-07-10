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
#include <dart/gui/detail/render_context.hpp>
#include <dart/gui/detail/render_environment.hpp>
#include <dart/gui/detail/renderable_factory.hpp>
#include <dart/gui/detail/renderable_resources.hpp>
#include <dart/gui/detail/renderable_sync.hpp>
#include <dart/gui/detail/screenshot.hpp>
#include <dart/gui/offscreen.hpp>
#include <dart/gui/viewer.hpp>

#include <filament/Engine.h>
#include <filament/Scene.h>

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <cctype>

namespace dart::gui {
namespace {

std::string toLowerAscii(std::string value)
{
  std::transform(
      value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
  return value;
}

OffscreenRenderOptions validatedOptions(OffscreenRenderOptions options)
{
  if (options.width <= 0 || options.height <= 0) {
    throw std::invalid_argument(
        "OffscreenRenderer requires a positive render size");
  }
  if (options.warmupFrames < 0) {
    throw std::invalid_argument(
        "OffscreenRenderer warmup_frames must be non-negative");
  }

  const std::string backend = toLowerAscii(options.renderBackend);
  if (backend == "noop" || backend == "none") {
    throw std::runtime_error(
        "OffscreenRenderer requires a real Filament backend; noop is not "
        "supported");
  }
  return options;
}

} // namespace

struct OffscreenRenderer::Impl
{
  explicit Impl(const OffscreenRenderOptions& rawOptions)
    : options(validatedOptions(rawOptions))
  {
    RunOptions runOptions;
    runOptions.width = options.width;
    runOptions.height = options.height;
    runOptions.headless = true;
    runOptions.renderBackend = options.renderBackend;

    context = detail::createFilamentRenderContext(runOptions, nullptr);
    if (context.engine == nullptr) {
      throw std::runtime_error(
          "Failed to create a Filament engine for offscreen rendering");
    }
    if (std::string(context.backendName) == "noop") {
      destroyContextOnly();
      throw std::runtime_error(
          "OffscreenRenderer requires a real Filament backend; resolved noop "
          "is not supported");
    }
    if (context.renderer == nullptr || context.swapChain == nullptr
        || context.scene == nullptr || context.view == nullptr
        || context.camera == nullptr) {
      destroyContextOnly();
      throw std::runtime_error(
          "Filament offscreen render context was not fully initialized");
    }

    context.activeViewCount = 1u;
    colorGrading = detail::createDebugColorGrading(*context.engine);
    detail::configureMainView(
        *context.view, colorGrading, !options.highFidelity);
    indirectLight = detail::createNeutralIndirectLight(*context.engine);
    skybox = detail::createNeutralSkybox(*context.engine);
    materialResources = detail::createMaterialResources(*context.engine);
    materialResourcesCreated = true;
    lights = detail::createSceneLights(
        *context.engine, !options.highFidelity, false, 1.0);
    lightsCreated = true;
    detail::attachSceneEnvironment(
        *context.scene, indirectLight, skybox, lights);
    environmentAttached = true;
  }

  ~Impl()
  {
    destroy();
  }

  RenderedImage render(
      const std::vector<RenderableDescriptor>& descriptors,
      const OrbitCamera& camera,
      const DebugScene& debug)
  {
    const auto materials = materialResources.materialSet();
    // Refresh replaces the previous overlay entity, so an empty DebugScene
    // clears any overlay left by an earlier debug render.
    detail::refreshProviderDebugOverlay(
        *context.engine,
        *context.scene,
        materials.debugColor,
        debug.lines,
        debug.triangles,
        debugOverlay);
    return renderWithCurrentOverlay(descriptors, camera);
  }

  RenderedImage render(
      const std::vector<RenderableDescriptor>& descriptors,
      const OrbitCamera& camera)
  {
    // A plain render must not inherit the overlay from an earlier debug
    // render; clearing keeps every call a pure function of its arguments.
    detail::clearProviderDebugOverlay(
        *context.engine, *context.scene, debugOverlay);
    return renderWithCurrentOverlay(descriptors, camera);
  }

  RenderedImage renderWithCurrentOverlay(
      const std::vector<RenderableDescriptor>& descriptors,
      const OrbitCamera& camera)
  {
    const auto materials = materialResources.materialSet();
    detail::synchronizeSceneRenderables(
        *context.engine,
        *context.scene,
        descriptors,
        sceneRenderables,
        loggedUnsupportedRenderableIds,
        [&](const RenderableDescriptor& descriptor)
            -> std::optional<detail::Renderable> {
          return detail::createRenderableFromDescriptor(
              *context.engine,
              materials,
              materialResources.textureCache,
              descriptor);
        });

    RenderSettings renderSettings;
    detail::updateSceneRenderablesFromDescriptors(
        *context.engine,
        descriptors,
        sceneRenderables,
        0,
        renderSettings,
        camera,
        options.width,
        options.height);
    detail::configureViewportCamera(
        *context.view, *context.camera, camera, options.width, options.height);

    detail::ScreenshotCapture capture;
    // Warm-up frames let temporal effects settle; flush each so the Filament
    // frame skipper does not back up and start rejecting beginFrame(). The
    // capture frame then renders and reads back unconditionally: a one-shot
    // offscreen grab must not be dropped by the pacing skipper, mirroring the
    // proven single-frame sequence in offscreen_parity.cpp.
    const int warmupFrames = std::max(0, options.warmupFrames);
    for (int i = 0; i < warmupFrames; ++i) {
      detail::beginFilamentFrame(context);
      detail::renderFilamentViews(context, nullptr);
      detail::endFilamentFrame(context);
      context.engine->flushAndWait();
    }

    detail::beginFilamentFrame(context);
    detail::renderFilamentViews(context, nullptr);
    detail::requestScreenshot(
        context,
        capture,
        static_cast<std::uint32_t>(options.width),
        static_cast<std::uint32_t>(options.height));
    detail::endFilamentFrame(context);

    if (!detail::waitForScreenshot(context, capture)) {
      throw std::runtime_error(
          "Timed out waiting for offscreen render readback");
    }

    RenderedImage image;
    image.width = capture.width;
    image.height = capture.height;
    image.channels = 4;
    image.pixels = std::move(capture.pixels);
    return image;
  }

  void destroy()
  {
    if (context.engine == nullptr) {
      return;
    }

    if (context.scene != nullptr) {
      detail::clearProviderDebugOverlay(
          *context.engine, *context.scene, debugOverlay);
    }

    for (auto& sceneRenderable : sceneRenderables) {
      if (context.scene != nullptr) {
        detail::removeRenderableFromScene(
            *context.scene, sceneRenderable.renderable);
      }
      detail::destroyRenderable(*context.engine, sceneRenderable.renderable);
    }
    sceneRenderables.clear();

    if (environmentAttached && context.scene != nullptr) {
      detail::detachSceneEnvironment(*context.scene, lights);
      environmentAttached = false;
    }
    if (lightsCreated) {
      detail::destroySceneLights(*context.engine, lights);
      lightsCreated = false;
    }
    if (indirectLight != nullptr || skybox != nullptr
        || colorGrading != nullptr) {
      detail::destroyRenderEnvironmentResources(
          *context.engine, indirectLight, skybox, colorGrading);
      indirectLight = nullptr;
      skybox = nullptr;
      colorGrading = nullptr;
    }
    if (materialResourcesCreated) {
      detail::destroyMaterialResources(*context.engine, materialResources);
      materialResourcesCreated = false;
    }

    destroyContextOnly();
  }

  void destroyContextOnly()
  {
    if (context.engine != nullptr) {
      detail::destroyFilamentRenderContext(context);
    }
  }

  OffscreenRenderOptions options;
  detail::FilamentRenderContext context;
  ::filament::ColorGrading* colorGrading = nullptr;
  ::filament::IndirectLight* indirectLight = nullptr;
  ::filament::Skybox* skybox = nullptr;
  detail::SceneLights lights;
  bool lightsCreated = false;
  bool environmentAttached = false;
  detail::MaterialResources materialResources;
  bool materialResourcesCreated = false;
  std::vector<detail::SceneRenderable> sceneRenderables;
  std::vector<RenderableId> loggedUnsupportedRenderableIds;
  detail::DebugOverlayController debugOverlay;
};

OffscreenRenderer::OffscreenRenderer(const OffscreenRenderOptions& options)
  : mImpl(std::make_unique<Impl>(options))
{
}

OffscreenRenderer::~OffscreenRenderer() = default;

OffscreenRenderer::OffscreenRenderer(OffscreenRenderer&&) noexcept = default;

OffscreenRenderer& OffscreenRenderer::operator=(OffscreenRenderer&&) noexcept
    = default;

RenderedImage OffscreenRenderer::render(
    const std::vector<RenderableDescriptor>& descriptors,
    const OrbitCamera& camera)
{
  return mImpl->render(descriptors, camera);
}

RenderedImage OffscreenRenderer::render(
    const std::vector<RenderableDescriptor>& descriptors,
    const OrbitCamera& camera,
    const DebugScene& debug)
{
  return mImpl->render(descriptors, camera, debug);
}

} // namespace dart::gui
