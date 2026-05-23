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
#include <cstdint>
#include <cstdlib>

namespace dart::gui::detail {

namespace {

using Backend = ::filament::Engine::Backend;

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
    return Backend::DEFAULT;
  }
  if (value == "opengl" || value == "gl" || value == "opengles") {
    return Backend::OPENGL;
  }
  if (value == "vulkan" || value == "vk") {
    return Backend::VULKAN;
  }
  if (value == "metal") {
    return Backend::METAL;
  }
  if (value == "webgpu") {
    return Backend::WEBGPU;
  }
  if (value == "noop" || value == "none") {
    return Backend::NOOP;
  }
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
    Backend requested, const char** chosenName)
{
  std::array<Backend, 3> candidates{
      requested, Backend::OPENGL, Backend::DEFAULT};
  Backend previous = requested;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const Backend backend = candidates[i];
    if (i > 0 && backend == previous) {
      continue;
    }
    previous = backend;
    if (::filament::Engine* engine
        = ::filament::Engine::Builder().backend(backend).build()) {
      if (backend != requested) {
        std::cerr << "[dart::gui] Filament backend '" << backendName(requested)
                  << "' unavailable; using '" << backendName(backend) << "'.\n";
      } else {
        std::cout << "[dart::gui] Filament render backend: "
                  << backendName(backend) << "\n";
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
      resolveRequestedBackend(options), &context.backendName);
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
  context.view = context.engine->createView();
  context.scene = context.engine->createScene();
  context.cameraEntity = utils::EntityManager::get().create();
  context.camera = context.engine->createCamera(context.cameraEntity);
  context.view->setScene(context.scene);
  context.view->setCamera(context.camera);
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
  context.renderer->render(context.view);
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
  context.engine->destroyCameraComponent(context.cameraEntity);
  utils::EntityManager::get().destroy(context.cameraEntity);
  context.engine->destroy(context.view);
  context.engine->destroy(context.scene);
  context.engine->destroy(context.renderer);
  context.engine->destroy(context.swapChain);
  ::filament::Engine::destroy(&context.engine);
  context = {};
}

} // namespace dart::gui::detail
