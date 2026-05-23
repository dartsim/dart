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

#include <dart/gui/detail/frame_renderer.hpp>
#include <dart/gui/detail/render_context.hpp>
#include <dart/gui/detail/screenshot.hpp>

#include <dart/common/profile.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include <cstdint>

namespace dart::gui::detail {

using dart::gui::elapsedMs;
using dart::gui::makeFrameOutputPath;
using dart::gui::markFrameRendered;
using dart::gui::markFrameSkipped;
using dart::gui::markScreenshotRequested;
using dart::gui::ProfileAccumulator;
using dart::gui::RunOptions;
using dart::gui::shouldCaptureFrameOutput;
using dart::gui::shouldRequestScreenshot;
using dart::gui::shouldStopAfterFrame;
using dart::gui::ViewerLifecycleState;

namespace {

bool ensureFrameOutputParentDirectory(const std::string& frameOutputPath)
{
  const std::filesystem::path outputPath(frameOutputPath);
  const std::filesystem::path outputDirectory = outputPath.parent_path();
  if (outputDirectory.empty()) {
    return true;
  }

  std::error_code error;
  std::filesystem::create_directories(outputDirectory, error);
  if (error) {
    std::cerr << "Failed to create frame output directory '"
              << outputDirectory.string() << "': " << error.message() << "\n";
    return false;
  }

  if (!std::filesystem::is_directory(outputDirectory, error)) {
    std::cerr << "Frame output path parent is not a directory: "
              << outputDirectory.string() << "\n";
    return false;
  }

  return true;
}

} // namespace

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
  DART_PROFILE_SCOPED_N("renderApplicationFrame");
  const bool shouldCaptureScreenshot
      = shouldRequestScreenshot(options, lifecycle);
  const bool shouldCaptureFrame = shouldCaptureFrameOutput(options, lifecycle);
  const bool shouldCaptureRenderedFrame
      = shouldCaptureScreenshot || shouldCaptureFrame;

  const auto renderFrameStart = ProfileAccumulator::Clock::now();
  bool shouldRenderFrame = false;
  {
    DART_PROFILE_SCOPED_N("beginFilamentFrame");
    shouldRenderFrame = beginFilamentFrame(renderContext);
  }
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
  {
    DART_PROFILE_SCOPED_N("renderFilamentViews");
    renderFilamentViews(renderContext, overlayView);
  }
  if (shouldCaptureRenderedFrame) {
    requestScreenshot(
        renderContext,
        screenshotCapture,
        static_cast<std::uint32_t>(framebufferWidth),
        static_cast<std::uint32_t>(framebufferHeight));
  }
  if (shouldCaptureScreenshot) {
    markScreenshotRequested(lifecycle);
  }
  endFilamentFrame(renderContext);
  ++profile.renderedFrames;
  const double renderMs = elapsedMs(renderStart);
  profile.renderMs += renderMs;
  profile.maxRenderMs = std::max(profile.maxRenderMs, renderMs);

  if (shouldCaptureFrame) {
    const std::string frameOutputPath
        = makeFrameOutputPath(options, lifecycle, lifecycle.renderedFrames + 1);
    if (!ensureFrameOutputParentDirectory(frameOutputPath)) {
      return {.stopLoop = true, .failed = true};
    }
    if (!saveCompletedScreenshotCapture(
            renderContext, screenshotCapture, frameOutputPath, profile)) {
      return {.stopLoop = true, .failed = true};
    }
  }

  const double frameMs = elapsedMs(frameStart);
  profile.frameMs += frameMs;
  profile.maxFrameMs = std::max(profile.maxFrameMs, frameMs);
  ++profile.frames;
  markFrameRendered(lifecycle);
  return {.stopLoop = shouldStopAfterFrame(options, lifecycle)};
}

} // namespace dart::gui::detail
