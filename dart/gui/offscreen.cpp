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
#include <imgui.h>

#include <algorithm>
#include <array>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cctype>
#include <cmath>
#include <cstdint>

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

/// A `scale` unit maps to this many pixels of font height, so the historical
/// default scale of 2 resolves to a legible ~16 px label. `fontSizePixels`
/// overrides this mapping when a caller wants an explicit pixel height.
constexpr double kScaleToPixels = 8.0;

/// Pixel height the debug-label atlas is rasterized at. Labels are drawn by
/// bilinearly resampling this base into the requested size, so a larger base
/// keeps typical 12-32 px labels crisp while bounding the one-time atlas cost.
constexpr float kDebugFontBaseSize = 32.0f;

/// One cached glyph from the anti-aliased debug-label atlas: its coverage rect
/// in atlas pixels plus the layout offsets/advance at the base font size.
struct DebugFontGlyph
{
  bool visible = false;
  float advance = 0.0f;
  float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
  int u0 = 0, v0 = 0, u1 = 0, v1 = 0;
};

/// A CPU-side, anti-aliased, mixed-case proportional font baked once from Dear
/// ImGui's default font atlas (`ImFontAtlas::AddFontDefault` +
/// `GetTexDataAsAlpha8`). The offscreen path has no ImGui draw pass, so the
/// alpha coverage and per-glyph metrics are copied out of a standalone atlas
/// and owned here; nothing else touches an ImGui context. Immutable after
/// construction, so the shared instance is safe to read from any thread.
struct DebugFont
{
  std::vector<std::uint8_t> alpha; // atlasWidth * atlasHeight coverage
  int atlasWidth = 0;
  int atlasHeight = 0;
  float baseSize = 0.0f;
  float lineHeight = 0.0f;
  std::array<DebugFontGlyph, 128> glyphs{}; // indexed by ASCII code point
};

DebugFont buildDebugFont()
{
  DebugFont font;
  ImFontConfig config;
  config.SizePixels = kDebugFontBaseSize;
  ImFontAtlas atlas;
  ImFont* imFont = atlas.AddFontDefault(&config);
  if (imFont == nullptr) {
    return font;
  }

  ImFontBaked* baked = imFont->GetFontBaked(kDebugFontBaseSize);
  if (baked == nullptr) {
    return font;
  }
  // Force every printable ASCII glyph to bake before the atlas is read, so the
  // dynamic atlas has finished growing and the glyph vector no longer moves.
  for (int code = 32; code < 127; ++code) {
    baked->FindGlyph(static_cast<ImWchar>(code));
  }

  unsigned char* pixels = nullptr;
  int width = 0;
  int height = 0;
  int bytesPerPixel = 0;
  atlas.GetTexDataAsAlpha8(&pixels, &width, &height, &bytesPerPixel);
  if (pixels == nullptr || width <= 0 || height <= 0) {
    return font;
  }

  font.baseSize = kDebugFontBaseSize;
  font.atlasWidth = width;
  font.atlasHeight = height;
  font.alpha.assign(
      pixels,
      pixels
          + static_cast<std::size_t>(width) * static_cast<std::size_t>(height));

  // GetTexDataAsAlpha8() runs the legacy Build() path, which rebuilds and
  // compacts the dynamic atlas and reallocates the per-size ImFontBaked cache.
  // That invalidates the `baked` pointer obtained before the build (the header
  // warns LastBaked must never be cached across such calls), so re-acquire it
  // now that the atlas is final before reading any metrics or glyphs from it.
  baked = imFont->GetFontBaked(kDebugFontBaseSize);
  if (baked == nullptr) {
    return font;
  }

  font.lineHeight = baked->Ascent - baked->Descent;

  for (int code = 32; code < 127; ++code) {
    const ImFontGlyph* glyph = baked->FindGlyph(static_cast<ImWchar>(code));
    if (glyph == nullptr) {
      continue;
    }
    DebugFontGlyph& out = font.glyphs[static_cast<std::size_t>(code)];
    out.visible = glyph->Visible != 0;
    out.advance = glyph->AdvanceX;
    out.x0 = glyph->X0;
    out.y0 = glyph->Y0;
    out.x1 = glyph->X1;
    out.y1 = glyph->Y1;
    out.u0 = static_cast<int>(std::lround(glyph->U0 * width));
    out.v0 = static_cast<int>(std::lround(glyph->V0 * height));
    out.u1 = static_cast<int>(std::lround(glyph->U1 * width));
    out.v1 = static_cast<int>(std::lround(glyph->V1 * height));
  }
  return font;
}

const DebugFont& debugFont()
{
  // Thread-safe, one-time initialization guaranteed by the C++ static-local
  // rules; the atlas is built on first label render and never mutated after.
  static const DebugFont font = buildDebugFont();
  return font;
}

/// Bilinearly samples the atlas alpha coverage at fractional pixel (u, v).
double sampleFontAlpha(const DebugFont& font, double u, double v)
{
  if (font.alpha.empty()) {
    return 0.0;
  }
  const double clampedU
      = std::clamp(u, 0.0, static_cast<double>(font.atlasWidth - 1));
  const double clampedV
      = std::clamp(v, 0.0, static_cast<double>(font.atlasHeight - 1));
  const int x0 = static_cast<int>(std::floor(clampedU));
  const int y0 = static_cast<int>(std::floor(clampedV));
  const int x1 = std::min(x0 + 1, font.atlasWidth - 1);
  const int y1 = std::min(y0 + 1, font.atlasHeight - 1);
  const double tx = clampedU - x0;
  const double ty = clampedV - y0;
  const auto at = [&](int x, int y) {
    return static_cast<double>(
        font.alpha[static_cast<std::size_t>(y) * font.atlasWidth + x]);
  };
  const double top = at(x0, y0) * (1.0 - tx) + at(x1, y0) * tx;
  const double bottom = at(x0, y1) * (1.0 - tx) + at(x1, y1) * tx;
  return top * (1.0 - ty) + bottom * ty;
}

/// Alpha-composites an RGB color over one pixel of a tightly packed buffer.
void blendPixel(
    std::uint8_t* pixels,
    std::size_t offset,
    int channels,
    const std::uint8_t rgb[3],
    double alpha)
{
  if (alpha <= 0.0) {
    return;
  }
  const double a = std::clamp(alpha, 0.0, 1.0);
  for (int channel = 0; channel < 3; ++channel) {
    const double base = static_cast<double>(pixels[offset + channel]);
    pixels[offset + channel] = static_cast<std::uint8_t>(
        base * (1.0 - a) + static_cast<double>(rgb[channel]) * a + 0.5);
  }
  if (channels > 3) {
    pixels[offset + 3] = 255;
  }
}

double resolveFontHeight(int scale, double fontSizePixels)
{
  if (fontSizePixels > 0.0) {
    return fontSizePixels;
  }
  return static_cast<double>(std::max(1, scale)) * kScaleToPixels;
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
    int scale,
    double fontSizePixels,
    bool backdrop)
{
  if (pixels == nullptr || width <= 0 || height <= 0 || channels < 3
      || text.empty()) {
    return;
  }
  const DebugFont& font = debugFont();
  if (font.alpha.empty() || font.baseSize <= 0.0f) {
    return;
  }

  const double glyphScale
      = resolveFontHeight(scale, fontSizePixels) / font.baseSize;
  const auto toByte = [](double value) {
    return static_cast<std::uint8_t>(std::clamp(value, 0.0, 1.0) * 255.0 + 0.5);
  };
  const std::uint8_t rgb[3]
      = {toByte(rgba[0]), toByte(rgba[1]), toByte(rgba[2])};
  const double textAlpha = std::clamp(rgba[3], 0.0, 1.0);

  const auto glyphFor = [&](char character) -> const DebugFontGlyph& {
    auto code = static_cast<unsigned char>(character);
    if (code < 32 || code >= 127) {
      code = static_cast<unsigned char>('?');
    }
    return font.glyphs[code];
  };

  double advanceTotal = 0.0;
  for (const char character : text) {
    advanceTotal += glyphFor(character).advance;
  }
  const double textWidth = advanceTotal * glyphScale;
  const double textHeight = font.lineHeight * glyphScale;

  const auto blendRect = [&](double minX,
                             double minY,
                             double maxX,
                             double maxY,
                             const std::uint8_t color[3],
                             double alpha) {
    const int startX = std::max(0, static_cast<int>(std::floor(minX)));
    const int startY = std::max(0, static_cast<int>(std::floor(minY)));
    const int endX = std::min(width, static_cast<int>(std::ceil(maxX)));
    const int endY = std::min(height, static_cast<int>(std::ceil(maxY)));
    for (int y = startY; y < endY; ++y) {
      for (int x = startX; x < endX; ++x) {
        const std::size_t offset
            = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width)
               + static_cast<std::size_t>(x))
              * static_cast<std::size_t>(channels);
        blendPixel(pixels, offset, channels, color, alpha);
      }
    }
  };

  if (backdrop && textWidth > 0.0 && textAlpha > 0.0) {
    const double pad = std::max(1.0, glyphScale * font.baseSize * 0.12);
    const std::uint8_t dark[3] = {0, 0, 0};
    blendRect(
        originX - pad,
        originY - pad,
        originX + textWidth + pad,
        originY + textHeight + pad,
        dark,
        0.5 * textAlpha);
  }

  double cursor = originX;
  for (const char character : text) {
    const DebugFontGlyph& glyph = glyphFor(character);
    if (glyph.visible && glyph.u1 > glyph.u0 && glyph.v1 > glyph.v0) {
      const double destX0 = cursor + glyph.x0 * glyphScale;
      const double destY0 = originY + glyph.y0 * glyphScale;
      const double destX1 = cursor + glyph.x1 * glyphScale;
      const double destY1 = originY + glyph.y1 * glyphScale;
      const int startX = static_cast<int>(std::floor(destX0));
      const int startY = static_cast<int>(std::floor(destY0));
      const int endX = static_cast<int>(std::ceil(destX1));
      const int endY = static_cast<int>(std::ceil(destY1));
      for (int y = startY; y < endY; ++y) {
        if (y < 0 || y >= height) {
          continue;
        }
        for (int x = startX; x < endX; ++x) {
          if (x < 0 || x >= width) {
            continue;
          }
          const double fx
              = destX1 > destX0 ? (x + 0.5 - destX0) / (destX1 - destX0) : 0.0;
          const double fy
              = destY1 > destY0 ? (y + 0.5 - destY0) / (destY1 - destY0) : 0.0;
          if (fx < 0.0 || fx > 1.0 || fy < 0.0 || fy > 1.0) {
            continue;
          }
          const double u = glyph.u0 + fx * (glyph.u1 - glyph.u0);
          const double v = glyph.v0 + fy * (glyph.v1 - glyph.v0);
          const double coverage
              = sampleFontAlpha(font, u, v) / 255.0 * textAlpha;
          if (coverage <= 0.003) {
            continue;
          }
          const std::size_t offset
              = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width)
                 + static_cast<std::size_t>(x))
                * static_cast<std::size_t>(channels);
          blendPixel(pixels, offset, channels, rgb, coverage);
        }
      }
    }
    cursor += glyph.advance * glyphScale;
  }
}

//==============================================================================
void compositeDebugLabels(
    RenderedImage& image,
    const OrbitCamera& camera,
    const std::vector<DebugLabelDescriptor>& labels,
    int scale,
    const ProjectionOptions& options,
    double fontSizePixels,
    bool backdrop)
{
  if (labels.empty() || image.pixels.empty() || image.channels < 3) {
    return;
  }
  const int width = static_cast<int>(image.width);
  const int height = static_cast<int>(image.height);
  const double fontHeight = resolveFontHeight(scale, fontSizePixels);
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
        static_cast<int>(std::lround(projected[0])) + 3,
        static_cast<int>(std::lround(projected[1] - fontHeight)),
        label.rgba,
        scale,
        fontSizePixels,
        backdrop);
  }
}

} // namespace dart::gui
