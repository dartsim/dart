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

#ifndef DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCREENSHOT_HPP_
#define DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCREENSHOT_HPP_

#include <dart/gui/profile.hpp>

#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include <cstdint>

namespace dart::gui::filament {

struct FilamentRenderContext;

struct ScreenshotCapture
{
  std::vector<std::uint8_t> pixels;
  std::mutex mutex;
  std::condition_variable condition;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  bool done = false;
};

void saveScreenshot(const ScreenshotCapture& capture, const std::string& path);

void requestScreenshot(
    FilamentRenderContext& context,
    ScreenshotCapture& capture,
    std::uint32_t width,
    std::uint32_t height);

bool waitForScreenshot(
    FilamentRenderContext& context, ScreenshotCapture& capture);

bool saveCompletedScreenshotCapture(
    FilamentRenderContext& context,
    ScreenshotCapture& capture,
    const std::string& path,
    dart::gui::ProfileAccumulator& profile);

bool finalizeScreenshotCapture(
    FilamentRenderContext& context,
    ScreenshotCapture& capture,
    const std::string& path,
    bool screenshotRequested,
    dart::gui::ProfileAccumulator& profile);

} // namespace dart::gui::filament

#endif // DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCREENSHOT_HPP_
