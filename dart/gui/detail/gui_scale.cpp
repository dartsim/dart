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

#include <dart/gui/detail/gui_scale.hpp>

#include <algorithm>
#include <string>

#include <cmath>
#include <cstdlib>

namespace dart::gui::detail {
namespace {

constexpr double kMinGuiUserScale = 0.5;
constexpr double kMaxGuiUserScale = 4.0;
constexpr double kMinGuiDpiScale = 0.5;
constexpr double kMaxGuiDpiScale = 4.0;
constexpr int kDefaultAutomaticWindowWidth = 1600;
constexpr int kDefaultAutomaticWindowHeight = 900;
constexpr double kAutomaticWindowWorkAreaTargetFraction = 0.70;
constexpr double kAutomaticWindowWorkAreaMaxFraction = 0.85;

int scaledDimension(int value, double scale)
{
  return std::max(
      1, static_cast<int>(std::lround(static_cast<double>(value) * scale)));
}

int automaticWindowDimension(
    int baseSize, int minimumSize, int maxSize, double scale)
{
  int size = std::max(std::max(1, baseSize), minimumSize);
  size = scaledDimension(size, scale);

  if (maxSize <= 0) {
    return size;
  }

  const int workAreaTarget = scaledDimension(
      maxSize, kAutomaticWindowWorkAreaTargetFraction * scale);
  const int workAreaCap
      = scaledDimension(maxSize, kAutomaticWindowWorkAreaMaxFraction);
  return std::min(std::max(size, workAreaTarget), workAreaCap);
}

} // namespace

double normalizeGuiUserScale(double scale)
{
  if (!std::isfinite(scale) || scale <= 0.0) {
    return 1.0;
  }
  return std::clamp(scale, kMinGuiUserScale, kMaxGuiUserScale);
}

double normalizeGuiDpiScale(double scale)
{
  if (!std::isfinite(scale) || scale <= 0.0) {
    return 1.0;
  }
  return std::clamp(scale, kMinGuiDpiScale, kMaxGuiDpiScale);
}

std::optional<double> parseGuiDpiScaleOverride(std::string_view text)
{
  if (text.empty()) {
    return std::nullopt;
  }

  const std::string value(text);
  char* end = nullptr;
  const double scale = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(scale)
      || scale <= 0.0) {
    return std::nullopt;
  }

  return normalizeGuiDpiScale(scale);
}

std::optional<double> guiDpiScaleOverrideFromEnvironment()
{
  const char* value = std::getenv("DART_GUI_DPI_SCALE");
  if (value == nullptr) {
    return std::nullopt;
  }
  return parseGuiDpiScaleOverride(value);
}

GuiScaleState makeGuiScaleState(double userScale, double dpiScale)
{
  GuiScaleState state;
  state.userScale = normalizeGuiUserScale(userScale);
  state.dpiScale = normalizeGuiDpiScale(dpiScale);
  state.effectiveScale = state.userScale * state.dpiScale;
  return state;
}

GuiWindowSize resolveAutomaticGuiWindowSize(
    int baseWidth,
    int baseHeight,
    double effectiveScale,
    int maxWidth,
    int maxHeight,
    bool automaticWidth,
    bool automaticHeight)
{
  GuiWindowSize size;
  size.width = std::max(1, baseWidth);
  size.height = std::max(1, baseHeight);
  if (!automaticWidth && !automaticHeight) {
    return size;
  }

  const double windowScale
      = std::isfinite(effectiveScale) && effectiveScale > 1.0 ? effectiveScale
                                                              : 1.0;
  if (automaticWidth) {
    size.width = automaticWindowDimension(
        size.width, kDefaultAutomaticWindowWidth, maxWidth, windowScale);
  }
  if (automaticHeight) {
    size.height = automaticWindowDimension(
        size.height, kDefaultAutomaticWindowHeight, maxHeight, windowScale);
  }
  return size;
}

} // namespace dart::gui::detail
