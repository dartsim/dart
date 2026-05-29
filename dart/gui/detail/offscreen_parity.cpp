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

#include "offscreen_parity.hpp"

#include "render_context.hpp"

#include <backend/PixelBufferDescriptor.h>
#include <filament/Engine.h>
#include <filament/RenderTarget.h>
#include <filament/Renderer.h>
#include <filament/Texture.h>
#include <filament/View.h>

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <vector>

namespace dart::gui::detail {

namespace {

using ::filament::Engine;
using ::filament::RenderTarget;
using ::filament::Renderer;
using ::filament::Texture;
using ::filament::View;
using PixelBufferDescriptor = ::filament::backend::PixelBufferDescriptor;

// Maximum allowed per-channel difference (0-255) between the offscreen and
// swapchain renders. Software GL rasterization is deterministic for the same
// view, so parity is near-exact; a few levels of slack absorbs any post-process
// rounding without hiding a genuine offscreen-pipeline divergence.
constexpr int kPixelTolerance = 4;

// The image must vary by at least this much (max-min over all channels) for the
// content to count as non-trivial, so a uniform clear cannot trivially pass.
constexpr int kNonTrivialRange = 12;

// Read the renderer's current target (swapchain when renderTarget is null, the
// offscreen RenderTarget otherwise) back into an RGBA8 byte buffer. readPixels
// must be issued inside a frame; the caller flushes after endFrame.
std::vector<std::uint8_t> readBackRgba(
    Renderer& renderer,
    RenderTarget* renderTarget,
    std::uint32_t width,
    std::uint32_t height,
    bool& done)
{
  std::vector<std::uint8_t> pixels(
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u,
      0u);
  done = false;
  PixelBufferDescriptor descriptor(
      pixels.data(),
      pixels.size(),
      PixelBufferDescriptor::PixelDataFormat::RGBA,
      PixelBufferDescriptor::PixelDataType::UBYTE,
      1,
      0,
      0,
      width,
      [](void*, std::size_t, void* user) { *static_cast<bool*>(user) = true; },
      &done);
  if (renderTarget != nullptr) {
    renderer.readPixels(
        renderTarget, 0, 0, width, height, std::move(descriptor));
  } else {
    renderer.readPixels(0, 0, width, height, std::move(descriptor));
  }
  return pixels;
}

// Drive the engine until the readback callback has fired (bounded). On OpenGL a
// single flushAndWait completes the copy; the loop tolerates backends that
// signal completion slightly after the flush returns.
void completeReadback(Engine& engine, const bool& done)
{
  for (int attempt = 0; attempt < 64 && !done; ++attempt) {
    engine.flushAndWait();
  }
}

// Largest per-channel difference between two equally sized RGBA buffers, with
// `second` optionally read bottom-up (GL RenderTarget readback can be y-flipped
// relative to the swapchain readback).
int maxChannelDelta(
    const std::vector<std::uint8_t>& first,
    const std::vector<std::uint8_t>& second,
    std::uint32_t width,
    std::uint32_t height,
    bool flipSecondVertically)
{
  const std::size_t rowBytes = static_cast<std::size_t>(width) * 4u;
  int maxDelta = 0;
  for (std::uint32_t y = 0; y < height; ++y) {
    const std::size_t firstRow = static_cast<std::size_t>(y) * rowBytes;
    const std::uint32_t secondY = flipSecondVertically ? (height - 1u - y) : y;
    const std::size_t secondRow = static_cast<std::size_t>(secondY) * rowBytes;
    for (std::size_t i = 0; i < rowBytes; ++i) {
      const int delta = std::abs(
          static_cast<int>(first[firstRow + i])
          - static_cast<int>(second[secondRow + i]));
      maxDelta = std::max(maxDelta, delta);
    }
  }
  return maxDelta;
}

int channelRange(const std::vector<std::uint8_t>& pixels)
{
  if (pixels.empty()) {
    return 0;
  }
  std::uint8_t lo = 255;
  std::uint8_t hi = 0;
  for (const std::uint8_t value : pixels) {
    lo = std::min(lo, value);
    hi = std::max(hi, value);
  }
  return static_cast<int>(hi) - static_cast<int>(lo);
}

} // namespace

bool runOffscreenParitySelfCheck(
    FilamentRenderContext& context,
    std::uint32_t width,
    std::uint32_t height,
    std::ostream& log)
{
  if (context.engine == nullptr || context.renderer == nullptr
      || context.views[0] == nullptr || context.swapChain == nullptr) {
    log << "[offscreen-parity] FAIL: render context not initialized\n";
    return false;
  }
  if (width == 0u || height == 0u) {
    log << "[offscreen-parity] FAIL: zero framebuffer size\n";
    return false;
  }

  Engine& engine = *context.engine;
  Renderer& renderer = *context.renderer;
  View* view = context.views[0];

  // 1) Render the primary view to the swapchain and read it back (path A).
  view->setRenderTarget(nullptr);
  bool swapChainReadDone = false;
  std::vector<std::uint8_t> swapChainPixels;
  {
    renderer.beginFrame(context.swapChain);
    renderer.render(view);
    swapChainPixels
        = readBackRgba(renderer, nullptr, width, height, swapChainReadDone);
    renderer.endFrame();
    completeReadback(engine, swapChainReadDone);
  }

  // 2) Render the same view to an offscreen RenderTarget backed by a color
  //    texture and read that back (path B).
  Texture* colorTexture
      = Texture::Builder()
            .width(width)
            .height(height)
            .levels(1)
            .format(Texture::InternalFormat::RGBA8)
            .usage(
                Texture::Usage::COLOR_ATTACHMENT | Texture::Usage::SAMPLEABLE
                | Texture::Usage::BLIT_SRC)
            .build(engine);
  Texture* depthTexture = Texture::Builder()
                              .width(width)
                              .height(height)
                              .levels(1)
                              .format(Texture::InternalFormat::DEPTH32F)
                              .usage(Texture::Usage::DEPTH_ATTACHMENT)
                              .build(engine);
  if (colorTexture == nullptr || depthTexture == nullptr) {
    log << "[offscreen-parity] FAIL: could not allocate offscreen textures\n";
    if (colorTexture != nullptr) {
      engine.destroy(colorTexture);
    }
    if (depthTexture != nullptr) {
      engine.destroy(depthTexture);
    }
    return false;
  }

  RenderTarget* renderTarget
      = RenderTarget::Builder()
            .texture(RenderTarget::AttachmentPoint::COLOR0, colorTexture)
            .texture(RenderTarget::AttachmentPoint::DEPTH, depthTexture)
            .build(engine);

  bool offscreenReadDone = false;
  std::vector<std::uint8_t> offscreenPixels;
  if (renderTarget != nullptr) {
    view->setRenderTarget(renderTarget);
    renderer.beginFrame(context.swapChain);
    renderer.render(view);
    offscreenPixels
        = readBackRgba(renderer, renderTarget, width, height, offscreenReadDone);
    renderer.endFrame();
    completeReadback(engine, offscreenReadDone);
  }

  // Restore the view's default (swapchain) target and release the offscreen
  // resources before reporting.
  view->setRenderTarget(nullptr);
  if (renderTarget != nullptr) {
    engine.destroy(renderTarget);
  }
  engine.destroy(colorTexture);
  engine.destroy(depthTexture);

  if (renderTarget == nullptr) {
    log << "[offscreen-parity] FAIL: could not build offscreen render target\n";
    return false;
  }
  if (!swapChainReadDone || !offscreenReadDone) {
    // Not fatal on its own: the content/parity checks below catch unfinished or
    // empty readbacks, but record the unexpected completion state.
    log << "[offscreen-parity] WARN: readback completion signal not observed\n";
  }

  // RenderTarget readback orientation can differ from the swapchain readback on
  // OpenGL; accept whichever orientation matches and report which one did.
  const int deltaSameY
      = maxChannelDelta(swapChainPixels, offscreenPixels, width, height, false);
  const int deltaFlipY
      = maxChannelDelta(swapChainPixels, offscreenPixels, width, height, true);
  const bool flipped = deltaFlipY < deltaSameY;
  const int maxDelta = std::min(deltaSameY, deltaFlipY);
  const int contentRange = channelRange(swapChainPixels);

  const bool parity = maxDelta <= kPixelTolerance;
  const bool nonTrivial = contentRange >= kNonTrivialRange;
  const bool passed = parity && nonTrivial;

  log << "[offscreen-parity] size=" << width << "x" << height
      << " maxDelta=" << maxDelta << " (tolerance " << kPixelTolerance << ")"
      << " orientation=" << (flipped ? "y-flipped" : "same")
      << " contentRange=" << contentRange << " backend=" << context.backendName
      << (passed ? " PASS" : " FAIL") << "\n";
  if (!parity) {
    log << "[offscreen-parity] offscreen render diverged from swapchain render\n";
  }
  if (!nonTrivial) {
    log << "[offscreen-parity] content too uniform to be a meaningful parity "
           "check (need a rendered scene, not a blank clear)\n";
  }
  return passed;
}

} // namespace dart::gui::detail
