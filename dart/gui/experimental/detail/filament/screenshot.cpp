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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; OR
 *   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *   DAMAGE.
 */

#include "screenshot.hpp"

#include "render_context.hpp"

#include <dart/gui/experimental/viewer.hpp>

#include <backend/PixelBufferDescriptor.h>
#include <filament/Engine.h>
#include <filament/Renderer.h>

#include <chrono>
#include <cstdlib>
#include <iostream>

namespace dart::gui::experimental::filament {

using dart::gui::experimental::ProfileAccumulator;
using dart::gui::experimental::elapsedMs;

void saveScreenshot(const ScreenshotCapture& capture, const std::string& path)
{
  std::string error;
  if (!dart::gui::experimental::writeRgbaPpm(
          path,
          capture.width,
          capture.height,
          capture.pixels,
          false,
          &error)) {
    std::cerr << error << "\n";
    std::exit(1);
  }
}

void requestScreenshot(
    FilamentRenderContext& context,
    ScreenshotCapture& capture,
    std::uint32_t width,
    std::uint32_t height)
{
  {
    std::lock_guard<std::mutex> lock(capture.mutex);
    capture.width = width;
    capture.height = height;
    capture.done = false;
    capture.pixels.assign(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4,
        0);
  }

  ::filament::backend::PixelBufferDescriptor pixels(
      capture.pixels.data(),
      capture.pixels.size(),
      ::filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
      ::filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
      1,
      0,
      0,
      width,
      [](void*, std::size_t, void* user) {
        auto* capture = static_cast<ScreenshotCapture*>(user);
        {
          std::lock_guard<std::mutex> lock(capture->mutex);
          capture->done = true;
        }
        capture->condition.notify_one();
      },
      &capture);
  context.renderer->readPixels(0, 0, width, height, std::move(pixels));
}

bool waitForScreenshot(
    FilamentRenderContext& context, ScreenshotCapture& capture)
{
  using namespace std::chrono_literals;
  context.engine->flushAndWait();

  std::unique_lock<std::mutex> lock(capture.mutex);
  return capture.done || capture.condition.wait_for(lock, 5s, [&capture] {
    return capture.done;
  });
}

bool finalizeScreenshotCapture(
    FilamentRenderContext& context,
    ScreenshotCapture& capture,
    const std::string& path,
    bool screenshotRequested,
    ProfileAccumulator& profile)
{
  bool screenshotSucceeded = path.empty();
  if (!path.empty() && !screenshotRequested) {
    std::cerr << "No rendered frame was available for screenshot capture\n";
  }
  if (screenshotRequested) {
    const auto screenshotWaitStart = ProfileAccumulator::Clock::now();
    screenshotSucceeded = waitForScreenshot(context, capture);
    profile.screenshotWaitMs += elapsedMs(screenshotWaitStart);
    if (screenshotSucceeded) {
      const auto screenshotSaveStart = ProfileAccumulator::Clock::now();
      saveScreenshot(capture, path);
      profile.screenshotSaveMs += elapsedMs(screenshotSaveStart);
    } else {
      std::cerr << "Timed out waiting for Filament screenshot readback\n";
    }
  }
  return screenshotSucceeded;
}

} // namespace dart::gui::experimental::filament
