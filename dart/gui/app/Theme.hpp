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

#ifndef DART_GUI_APP_THEME_HPP_
#define DART_GUI_APP_THEME_HPP_

#include <dart/gui/Viewer.hpp>

#include <dart/external/imgui/imgui.h>

namespace dart::gui::app {

//==============================================================================
/// A restrained, cool-neutral dark palette with a single blue accent. Sets
/// ImGui::GetStyle().Colors[]; call once before the first frame.
void applyModernDarkColors();

//==============================================================================
/// Companion spacing/rounding metrics to applyModernDarkColors, ported from
/// applyModernDarkMetrics. Docking-only bits (docking separators, dock
/// preview colors) require the Dear ImGui docking release. Sets base
/// (unscaled) style metrics; per-frame GUI scaling is applied on top of these
/// by dart::gui::Viewer::setGuiScale() via
/// ImGuiStyle::ScaleAllSizes, so callers should not pre-multiply these values
/// by the GUI scale. Call once before the first frame, after
/// applyModernDarkColors().
void applyModernDarkMetrics();

/// Restore this demo's themed metrics before each newly scaled frame.
class GuiScaleTheme
{
public:
  explicit GuiScaleTheme(const dart::gui::Viewer& handler);
  ~GuiScaleTheme();
  GuiScaleTheme(const GuiScaleTheme&) = delete;
  GuiScaleTheme& operator=(const GuiScaleTheme&) = delete;

private:
  const dart::gui::Viewer& mHandler;
  ImGuiContext* mContext;
  ImGuiStyle mBaseStyle;
  ImGuiID mHookId;
  double mAppliedScale = 0.0;
};

} // namespace dart::gui::app

#endif // DART_GUI_APP_THEME_HPP_
