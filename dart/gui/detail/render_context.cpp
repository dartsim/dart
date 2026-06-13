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

#include "render_context.hpp"

#include <dart/gui/viewer.hpp>

#include <dart/common/logging.hpp>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/View.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <optional>
#include <string>

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

#if defined(__linux__)
  #include <sys/resource.h>
#endif

namespace dart::gui::detail {

namespace {

using Backend = ::filament::Engine::Backend;

#if defined(__linux__)
class ScopedHeadlessCircularBufferFileLimit
{
public:
  explicit ScopedHeadlessCircularBufferFileLimit(bool enabled)
  {
    if (!enabled) {
      return;
    }

    if (getrlimit(RLIMIT_FSIZE, &mOriginal) != 0) {
      return;
    }

    constexpr rlim_t kLimit = 5u * 1024u * 1024u;
    const rlim_t requestedSoft
        = mOriginal.rlim_max == RLIM_INFINITY
              ? kLimit
              : std::min<rlim_t>(mOriginal.rlim_max, kLimit);
    if (mOriginal.rlim_cur <= requestedSoft) {
      return;
    }

    rlimit next = mOriginal;
    next.rlim_cur = requestedSoft;
    mActive = setrlimit(RLIMIT_FSIZE, &next) == 0;
  }

  ScopedHeadlessCircularBufferFileLimit(
      const ScopedHeadlessCircularBufferFileLimit&) = delete;
  ScopedHeadlessCircularBufferFileLimit& operator=(
      const ScopedHeadlessCircularBufferFileLimit&) = delete;

  ~ScopedHeadlessCircularBufferFileLimit()
  {
    if (mActive) {
      setrlimit(RLIMIT_FSIZE, &mOriginal);
    }
  }

private:
  rlimit mOriginal{};
  bool mActive = false;
};
#endif

std::string toLowerAscii(std::string value)
{
  std::transform(
      value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
  return value;
}

const char* backendName(Backend backend)
{
  switch (backend) {
    case Backend::DEFAULT:
      return "default";
    case Backend::OPENGL:
      return "opengl";
    case Backend::VULKAN:
      return "vulkan";
    case Backend::METAL:
      return "metal";
    case Backend::WEBGPU:
      return "webgpu";
    case Backend::NOOP:
      return "noop";
  }
  return "unknown";
}

std::optional<Backend> parseBackend(const std::string& raw)
{
  const std::string value = toLowerAscii(raw);
  if (value.empty() || value == "default" || value == "auto") {
    // "default"/"auto" map to OpenGL (the build's safe default), not Filament's
    // platform auto-pick, which could choose a backend with no embedded
    // materials (e.g. Metal on macOS — materials are compiled GL/Vulkan only).
    return Backend::OPENGL;
  }
  if (value == "opengl" || value == "gl" || value == "opengles") {
    return Backend::OPENGL;
  }
  if (value == "vulkan" || value == "vk") {
    return Backend::VULKAN;
  }
  if (value == "noop" || value == "none") {
    return Backend::NOOP;
  }
  // metal/webgpu are intentionally not accepted: embedded materials are only
  // compiled for OpenGL and Vulkan (see backend_sources.cmake), so selecting a
  // backend without matching material variants would fail at material load
  // rather than fall back. Unknown tokens fall back to OpenGL in the caller.
  return std::nullopt;
}

// Resolve the requested backend from (highest precedence first) the
// DART_FILAMENT_BACKEND environment variable, then RunOptions::backend, then a
// safe default that preserves DART's historical behavior (OpenGL).
Backend resolveRequestedBackend(const dart::gui::RunOptions& options)
{
  std::string source;
  if (const char* env = std::getenv("DART_FILAMENT_BACKEND");
      env != nullptr && env[0] != '\0') {
    source = env;
  } else if (!options.renderBackend.empty()) {
    source = options.renderBackend;
  } else {
    // Historical default: OpenGL, the only backend DART previously created.
    return Backend::OPENGL;
  }

  if (const std::optional<Backend> parsed = parseBackend(source); parsed) {
    return *parsed;
  }
  std::cerr << "[dart::gui] Unknown render backend '" << source
            << "' (DART_FILAMENT_BACKEND / --render-backend); falling back to "
               "OpenGL.\n";
  return Backend::OPENGL;
}

// Create the Filament engine, attempting the requested backend and then a small
// ordered fallback list so a request for an unavailable backend degrades to a
// working one instead of leaving the engine null. On success, *chosenName is
// set to the static name of the backend that was actually created.
::filament::Engine* createEngineWithFallback(
    Backend requested, const char** chosenName, bool headless)
{
  // Fall back only to backends with embedded materials (OpenGL, Vulkan), never
  // to Filament's platform default which could select a material-less backend.
  std::array<Backend, 3> candidates{
      requested, Backend::OPENGL, Backend::VULKAN};
  Backend previous = requested;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const Backend backend = candidates[i];
    if (i > 0 && backend == previous) {
      continue;
    }
    previous = backend;
#if defined(__linux__)
    // Filament's Linux command-buffer path first tries a /tmp-backed
    // ashmem-style double mapping. On tmpfs-pressure hosts that path can SIGBUS
    // while zeroing the mapping during Engine creation. Headless capture does
    // not need the file-backed path, so force Filament's anonymous fallback
    // only while the engine is built; screenshots and frame files are written
    // after the original file-size limit is restored.
    ScopedHeadlessCircularBufferFileLimit circularBufferLimit(headless);
#else
    static_cast<void>(headless);
#endif
    if (::filament::Engine* engine
        = ::filament::Engine::Builder().backend(backend).build()) {
      if (backend != requested) {
        std::cerr << "[dart::gui] Filament backend '" << backendName(requested)
                  << "' unavailable; using '" << backendName(backend) << "'.\n";
      } else {
        DART_DEBUG("Filament render backend: {}", backendName(backend));
      }
      if (chosenName != nullptr) {
        *chosenName = backendName(backend);
      }
      return engine;
    }
  }
  return nullptr;
}

} // namespace

FilamentRenderContext createFilamentRenderContext(
    const dart::gui::RunOptions& options, void* nativeWindow)
{
  FilamentRenderContext context;
  context.engine = createEngineWithFallback(
      resolveRequestedBackend(options), &context.backendName, options.headless);
  context.renderer = context.engine->createRenderer();
  if (options.headless) {
    ::filament::Renderer::DisplayInfo displayInfo;
    displayInfo.refreshRate = 0.0f;
    context.renderer->setDisplayInfo(displayInfo);
  }
  context.swapChain = options.headless
                          ? context.engine->createSwapChain(
                                static_cast<std::uint32_t>(options.width),
                                static_cast<std::uint32_t>(options.height))
                          : context.engine->createSwapChain(nativeWindow);
  context.scene = context.engine->createScene();
  for (std::size_t i = 0; i < context.views.size(); ++i) {
    context.views[i] = context.engine->createView();
    context.cameraEntities[i] = utils::EntityManager::get().create();
    context.cameras[i]
        = context.engine->createCamera(context.cameraEntities[i]);
    context.views[i]->setScene(context.scene);
    context.views[i]->setCamera(context.cameras[i]);
  }
  context.view = context.views[0];
  context.cameraEntity = context.cameraEntities[0];
  context.camera = context.cameras[0];
  return context;
}

bool beginFilamentFrame(FilamentRenderContext& context)
{
  return context.renderer->beginFrame(context.swapChain);
}

bool shouldSkipRenderedWorkAfterFrameSkip(
    const FilamentRenderContext& context, bool headless)
{
  return !headless || !context.renderer->shouldRenderFrame();
}

void renderFilamentViews(
    FilamentRenderContext& context, ::filament::View* overlayView)
{
  const std::size_t activeViewCount = std::clamp<std::size_t>(
      context.activeViewCount, 1u, context.views.size());
  for (std::size_t i = 0; i < activeViewCount; ++i) {
    if (context.views[i] != nullptr) {
      context.renderer->render(context.views[i]);
    }
  }
  if (overlayView != nullptr) {
    context.renderer->render(overlayView);
  }
}

void endFilamentFrame(FilamentRenderContext& context)
{
  context.renderer->endFrame();
}

double latestGpuFrameMs(const FilamentRenderContext& context)
{
  if (context.renderer == nullptr) {
    return 0.0;
  }
  const auto history = context.renderer->getFrameInfoHistory(8);
  for (std::size_t i = history.size(); i-- > 0;) {
    const auto duration = history[i].denoisedGpuFrameDuration;
    if (duration > 0) {
      return static_cast<double>(duration) / 1.0e6; // ns -> ms
    }
  }
  return 0.0;
}

void destroyFilamentRenderContext(FilamentRenderContext& context)
{
  for (std::size_t i = 0; i < context.views.size(); ++i) {
    context.engine->destroyCameraComponent(context.cameraEntities[i]);
    utils::EntityManager::get().destroy(context.cameraEntities[i]);
    if (context.views[i] != nullptr) {
      context.engine->destroy(context.views[i]);
    }
  }
  context.engine->destroy(context.scene);
  context.engine->destroy(context.renderer);
  context.engine->destroy(context.swapChain);
  ::filament::Engine::destroy(&context.engine);
  context = {};
}

} // namespace dart::gui::detail
