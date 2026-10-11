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

#pragma once
#include <dart/gui/Export.hpp>

#include <algorithm>
#include <limits>
#include <string>

#include <cmath>
#include <cstdint>
struct GLFWwindow;
namespace filament {
class Engine;
}
namespace dart::gui::detail {
struct NativeWindowRectangle
{
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

inline NativeWindowRectangle fitDefaultWindow(
    const NativeWindowRectangle& workArea,
    float monitorScale,
    double requestedScale,
    bool logicalCoordinates)
{
  const double detected
      = std::isfinite(monitorScale) && monitorScale > 0 ? monitorScale : 1.0;
  const bool manual = std::isfinite(requestedScale) && requestedScale > 0;
  double scale = manual ? std::clamp(requestedScale, 0.5, 4.0) : detected;
  if (logicalCoordinates)
    scale = manual ? scale / detected : 1.0;
  double width = 1600 * scale;
  double height = 1000 * scale;
  const bool bounded = workArea.width > 0 && workArea.height > 0;
  if (bounded) {
    const double fit = std::min(
        {1.0,
         std::max(1.0, std::floor(workArea.width * 0.9)) / width,
         std::max(1.0, std::floor(workArea.height * 0.9)) / height});
    width *= fit;
    height *= fit;
  }
  const auto extent = [](double value) {
    return static_cast<int>(std::lround(std::clamp(
        value, 1.0, static_cast<double>(std::numeric_limits<int>::max()))));
  };
  NativeWindowRectangle result{
      workArea.x, workArea.y, extent(width), extent(height)};
  if (bounded) {
    const auto centered = [](int origin, int available, int size) {
      return static_cast<int>(std::clamp<std::int64_t>(
          std::int64_t{origin} + (std::int64_t{available} - size) / 2,
          std::numeric_limits<int>::min(),
          std::numeric_limits<int>::max()));
    };
    result.x = centered(workArea.x, workArea.width, result.width);
    result.y = centered(workArea.y, workArea.height, result.height);
  }
  return result;
}

DART_GUI_API void* getFilamentNativeWindow(GLFWwindow* window);
DART_GUI_API void resizeFilamentNativeWindow(
    GLFWwindow* window, int width, int height);
DART_GUI_API void releaseFilamentNativeWindow(GLFWwindow* window);
#if defined(__APPLE__)
std::string configureFilamentMetalWindow(
    ::filament::Engine& engine, void* nativeWindow);
#endif
} // namespace dart::gui::detail
