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

#include <dart/gui/experimental/detail/filament/frame_renderer.hpp>
#include <dart/gui/experimental/detail/filament/render_context.hpp>
#include <dart/gui/experimental/detail/filament/screenshot.hpp>

#include <algorithm>
#include <chrono>
#include <thread>

#include <cstdint>

namespace dart::gui::filament {

using dart::gui::elapsedMs;
using dart::gui::markFrameRendered;
using dart::gui::markFrameSkipped;
using dart::gui::markScreenshotRequested;
using dart::gui::ProfileAccumulator;
using dart::gui::RunOptions;
using dart::gui::shouldRequestScreenshot;
using dart::gui::shouldStopAfterFrame;
using dart::gui::ViewerLifecycleState;

FrameRenderResult renderApplicationFrame(
    FilamentRenderContext& renderContext,
    ::filament::View* overlayView,
    const RunOptions& options,
    int framebufferWidth,
    int framebufferHeight,
    ProfileAccumulator::Clock::time_point frameStart,
    ScreenshotCapture& screenshotCapture,
    ViewerLifecycleState& lifecycle,
    ProfileAccumulator& profile)
{
  const bool shouldCaptureScreenshot
      = shouldRequestScreenshot(options, lifecycle);

  const auto renderFrameStart = ProfileAccumulator::Clock::now();
  const bool shouldRenderFrame = beginFilamentFrame(renderContext);
  profile.beginFrameMs += elapsedMs(renderFrameStart);
  if (!shouldRenderFrame) {
    markFrameSkipped(lifecycle);
    ++profile.skippedFrames;
    if (shouldSkipRenderedWorkAfterFrameSkip(renderContext, options.headless)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      const double frameMs = elapsedMs(frameStart);
      profile.frameMs += frameMs;
      profile.maxFrameMs = std::max(profile.maxFrameMs, frameMs);
      ++profile.frames;
      return {.continueLoop = true};
    }
    // Filament allows callers to ignore a pacing-only false return. Headless
    // software GL can report skips almost every frame, so keep deterministic
    // offscreen captures while still skipping backend failures above.
  }

  const auto renderStart = ProfileAccumulator::Clock::now();
  renderFilamentViews(renderContext, overlayView);
  if (shouldCaptureScreenshot) {
    requestScreenshot(
        renderContext,
        screenshotCapture,
        static_cast<std::uint32_t>(framebufferWidth),
        static_cast<std::uint32_t>(framebufferHeight));
    markScreenshotRequested(lifecycle);
  }
  endFilamentFrame(renderContext);
  ++profile.renderedFrames;
  const double renderMs = elapsedMs(renderStart);
  profile.renderMs += renderMs;
  profile.maxRenderMs = std::max(profile.maxRenderMs, renderMs);

  const double frameMs = elapsedMs(frameStart);
  profile.frameMs += frameMs;
  profile.maxFrameMs = std::max(profile.maxFrameMs, frameMs);
  ++profile.frames;
  markFrameRendered(lifecycle);
  return {.stopLoop = shouldStopAfterFrame(options, lifecycle)};
}

} // namespace dart::gui::filament
