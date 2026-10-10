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

#ifndef DART_EXAMPLES_DEMOS_THEME_HPP_
#define DART_EXAMPLES_DEMOS_THEME_HPP_

#include <dart/gui/osg/IncludeImGui.hpp>

namespace dart::gui::osg {
class ImGuiHandler;
}

namespace dart_demos {

//==============================================================================
/// A restrained, cool-neutral dark palette with a single blue accent. Sets
/// ImGui::GetStyle().Colors[]; call once before the first frame.
void applyModernDarkColors();

//==============================================================================
/// Companion spacing/rounding metrics to applyModernDarkColors, ported from
/// applyModernDarkMetrics. Docking-only bits (docking separators, dock
/// preview colors) apply when the build uses the bundled docking-branch ImGui
/// (IMGUI_HAS_DOCK) and are skipped on vanilla system ImGui. Sets base
/// (unscaled) style metrics; per-frame GUI scaling is applied on top of these
/// by dart::gui::osg::ImGuiHandler::setGuiScale() via
/// ImGuiStyle::ScaleAllSizes, so callers should not pre-multiply these values
/// by the GUI scale. Call once before the first frame, after
/// applyModernDarkColors().
void applyModernDarkMetrics();

/// Restore this demo's themed metrics before each newly scaled frame.
class GuiScaleTheme
{
public:
  explicit GuiScaleTheme(const dart::gui::osg::ImGuiHandler& handler);
  ~GuiScaleTheme();
  GuiScaleTheme(const GuiScaleTheme&) = delete;
  GuiScaleTheme& operator=(const GuiScaleTheme&) = delete;

private:
  const dart::gui::osg::ImGuiHandler& mHandler;
  ImGuiContext* mContext;
  ImGuiStyle mBaseStyle;
  ImGuiID mHookId;
  double mAppliedScale = 0.0;
};

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_THEME_HPP_
