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
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <cctype>
#include <cmath>

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

/// Compact 3x5 uppercase bitmap font for deterministic, dependency-free
/// debug-label text on headless captures. Each glyph is five rows of three
/// bits (most significant bit = left column).
struct GlyphRows
{
  char character;
  std::uint8_t rows[5];
};

constexpr GlyphRows kDebugLabelFont[] = {
    {'A', {0b010, 0b101, 0b111, 0b101, 0b101}},
    {'B', {0b110, 0b101, 0b110, 0b101, 0b110}},
    {'C', {0b011, 0b100, 0b100, 0b100, 0b011}},
    {'D', {0b110, 0b101, 0b101, 0b101, 0b110}},
    {'E', {0b111, 0b100, 0b110, 0b100, 0b111}},
    {'F', {0b111, 0b100, 0b110, 0b100, 0b100}},
    {'G', {0b011, 0b100, 0b101, 0b101, 0b011}},
    {'H', {0b101, 0b101, 0b111, 0b101, 0b101}},
    {'I', {0b111, 0b010, 0b010, 0b010, 0b111}},
    {'J', {0b001, 0b001, 0b001, 0b101, 0b010}},
    {'K', {0b101, 0b110, 0b100, 0b110, 0b101}},
    {'L', {0b100, 0b100, 0b100, 0b100, 0b111}},
    {'M', {0b101, 0b111, 0b111, 0b101, 0b101}},
    {'N', {0b101, 0b111, 0b111, 0b111, 0b101}},
    {'O', {0b010, 0b101, 0b101, 0b101, 0b010}},
    {'P', {0b110, 0b101, 0b110, 0b100, 0b100}},
    {'Q', {0b010, 0b101, 0b101, 0b011, 0b001}},
    {'R', {0b110, 0b101, 0b110, 0b110, 0b101}},
    {'S', {0b011, 0b100, 0b010, 0b001, 0b110}},
    {'T', {0b111, 0b010, 0b010, 0b010, 0b010}},
    {'U', {0b101, 0b101, 0b101, 0b101, 0b111}},
    {'V', {0b101, 0b101, 0b101, 0b010, 0b010}},
    {'W', {0b101, 0b101, 0b111, 0b111, 0b101}},
    {'X', {0b101, 0b010, 0b010, 0b010, 0b101}},
    {'Y', {0b101, 0b101, 0b010, 0b010, 0b010}},
    {'Z', {0b111, 0b001, 0b010, 0b100, 0b111}},
    {'0', {0b111, 0b101, 0b101, 0b101, 0b111}},
    {'1', {0b010, 0b110, 0b010, 0b010, 0b111}},
    {'2', {0b111, 0b001, 0b111, 0b100, 0b111}},
    {'3', {0b111, 0b001, 0b011, 0b001, 0b111}},
    {'4', {0b101, 0b101, 0b111, 0b001, 0b001}},
    {'5', {0b111, 0b100, 0b111, 0b001, 0b111}},
    {'6', {0b111, 0b100, 0b111, 0b101, 0b111}},
    {'7', {0b111, 0b001, 0b001, 0b010, 0b010}},
    {'8', {0b111, 0b101, 0b111, 0b101, 0b111}},
    {'9', {0b111, 0b101, 0b111, 0b001, 0b111}},
    {'-', {0b000, 0b000, 0b111, 0b000, 0b000}},
    {'_', {0b000, 0b000, 0b000, 0b000, 0b111}},
    {'.', {0b000, 0b000, 0b000, 0b000, 0b010}},
    {':', {0b000, 0b010, 0b000, 0b010, 0b000}},
    {'/', {0b001, 0b001, 0b010, 0b100, 0b100}},
    {'+', {0b000, 0b010, 0b111, 0b010, 0b000}},
    {'(', {0b010, 0b100, 0b100, 0b100, 0b010}},
    {')', {0b010, 0b001, 0b001, 0b001, 0b010}},
    {' ', {0b000, 0b000, 0b000, 0b000, 0b000}},
};

const std::uint8_t* debugLabelGlyph(char character)
{
  const char upper
      = static_cast<char>(std::toupper(static_cast<unsigned char>(character)));
  for (const auto& glyph : kDebugLabelFont) {
    if (glyph.character == upper) {
      return glyph.rows;
    }
  }
  return kDebugLabelFont[std::size(kDebugLabelFont) - 1].rows; // space
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
    RenderedImage image = renderWithCurrentOverlay(descriptors, camera);
    // The viewer draws labels in its ImGui pass; the offscreen path
    // composites them here so the full DebugScene renders headlessly.
    compositeDebugLabels(image, camera, debug.labels);
    return image;
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

//==============================================================================
void drawDebugLabelText(
    std::uint8_t* pixels,
    int width,
    int height,
    int channels,
    const std::string& text,
    int originX,
    int originY,
    const Eigen::Vector4d& rgba,
    int scale)
{
  if (pixels == nullptr || width <= 0 || height <= 0 || channels < 3) {
    return;
  }
  const int safeScale = std::max(1, scale);
  const auto channel = [](double value) {
    return static_cast<std::uint8_t>(std::clamp(value, 0.0, 1.0) * 255.0 + 0.5);
  };
  const std::uint8_t red = channel(rgba[0]);
  const std::uint8_t green = channel(rgba[1]);
  const std::uint8_t blue = channel(rgba[2]);

  int cursorX = originX;
  for (const char character : text) {
    const std::uint8_t* rows = debugLabelGlyph(character);
    for (int row = 0; row < 5; ++row) {
      for (int column = 0; column < 3; ++column) {
        if ((rows[row] & (0b100 >> column)) == 0) {
          continue;
        }
        for (int dy = 0; dy < safeScale; ++dy) {
          const int y = originY + row * safeScale + dy;
          if (y < 0 || y >= height) {
            continue;
          }
          for (int dx = 0; dx < safeScale; ++dx) {
            const int x = cursorX + column * safeScale + dx;
            if (x < 0 || x >= width) {
              continue;
            }
            const std::size_t offset
                = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width)
                   + static_cast<std::size_t>(x))
                  * static_cast<std::size_t>(channels);
            pixels[offset] = red;
            pixels[offset + 1] = green;
            pixels[offset + 2] = blue;
            if (channels > 3) {
              pixels[offset + 3] = 255;
            }
          }
        }
      }
    }
    cursorX += (3 + 1) * safeScale;
  }
}

//==============================================================================
void compositeDebugLabels(
    RenderedImage& image,
    const OrbitCamera& camera,
    const std::vector<DebugLabelDescriptor>& labels,
    int scale,
    const ProjectionOptions& options)
{
  if (labels.empty() || image.pixels.empty() || image.channels < 3) {
    return;
  }
  const int width = static_cast<int>(image.width);
  const int height = static_cast<int>(image.height);
  const int safeScale = std::max(1, scale);
  for (const auto& label : labels) {
    const Eigen::Vector3d projected
        = projectToPixels(camera, width, height, label.position, options);
    if (projected[2] <= 0.0 || !projected.allFinite()) {
      continue;
    }
    drawDebugLabelText(
        image.pixels.data(),
        width,
        height,
        static_cast<int>(image.channels),
        label.text,
        static_cast<int>(std::lround(projected[0])) + 2,
        static_cast<int>(std::lround(projected[1])) - 3 * safeScale,
        label.rgba,
        safeScale);
  }
}

} // namespace dart::gui
