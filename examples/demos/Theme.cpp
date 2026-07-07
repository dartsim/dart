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

#include "Theme.hpp"

#include <dart/gui/osg/osg.hpp>

namespace dart_demos {

namespace {

//==============================================================================
ImVec4 fromHex(unsigned r, unsigned g, unsigned b, float alpha = 1.0f)
{
  return ImVec4(
      static_cast<float>(r) / 255.0f,
      static_cast<float>(g) / 255.0f,
      static_cast<float>(b) / 255.0f,
      alpha);
}

//==============================================================================
ImVec4 lighten(const ImVec4& color, float amount)
{
  auto lerp = [amount](float channel) {
    return channel + (1.0f - channel) * amount;
  };
  return ImVec4(lerp(color.x), lerp(color.y), lerp(color.z), color.w);
}

//==============================================================================
ImVec4 withAlpha(const ImVec4& color, float alpha)
{
  return ImVec4(color.x, color.y, color.z, alpha);
}

} // namespace

//==============================================================================
void applyModernDarkColors()
{
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();
  auto& colors = style.Colors;

  // Cool neutral surfaces.
  const ImVec4 window = fromHex(0x1b, 0x1d, 0x23);
  const ImVec4 panel = fromHex(0x21, 0x24, 0x2b);
  const ImVec4 header = fromHex(0x16, 0x18, 0x1d);
  const ImVec4 text = fromHex(0xcd, 0xd3, 0xde);
  const ImVec4 textDim = fromHex(0x7e, 0x87, 0x94);
  const ImVec4 border = withAlpha(fromHex(0x32, 0x38, 0x43), 0.60f);

  // Single blue accent, with hover/active variants lightened progressively
  // and a low-alpha "soft" variant for subtle highlights.
  const ImVec4 accent = fromHex(0x4c, 0x8c, 0xf0);
  const ImVec4 accentHover = lighten(accent, 0.12f);
  const ImVec4 accentActive = lighten(accent, 0.22f);
  const ImVec4 accentSoft = withAlpha(accent, 0.35f);

  colors[ImGuiCol_Text] = text;
  colors[ImGuiCol_TextDisabled] = textDim;
  colors[ImGuiCol_WindowBg] = window;
  colors[ImGuiCol_ChildBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  colors[ImGuiCol_PopupBg] = window;
  colors[ImGuiCol_Border] = border;
  colors[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

  colors[ImGuiCol_FrameBg] = panel;
  colors[ImGuiCol_FrameBgHovered] = lighten(panel, 0.10f);
  colors[ImGuiCol_FrameBgActive] = lighten(panel, 0.18f);

  colors[ImGuiCol_TitleBg] = header;
  colors[ImGuiCol_TitleBgActive] = header;
  colors[ImGuiCol_TitleBgCollapsed] = header;
  colors[ImGuiCol_MenuBarBg] = header;

  colors[ImGuiCol_ScrollbarBg] = header;
  colors[ImGuiCol_ScrollbarGrab] = panel;
  colors[ImGuiCol_ScrollbarGrabHovered] = lighten(panel, 0.15f);
  colors[ImGuiCol_ScrollbarGrabActive] = lighten(panel, 0.25f);

  colors[ImGuiCol_CheckMark] = accent;
  colors[ImGuiCol_SliderGrab] = accent;
  colors[ImGuiCol_SliderGrabActive] = accentActive;

  colors[ImGuiCol_Button] = panel;
  colors[ImGuiCol_ButtonHovered] = accentHover;
  colors[ImGuiCol_ButtonActive] = accentActive;

  colors[ImGuiCol_Header] = accentSoft;
  colors[ImGuiCol_HeaderHovered] = accentHover;
  colors[ImGuiCol_HeaderActive] = accentActive;

  colors[ImGuiCol_Separator] = border;
  colors[ImGuiCol_SeparatorHovered] = accentHover;
  colors[ImGuiCol_SeparatorActive] = accentActive;

  colors[ImGuiCol_ResizeGrip] = accentSoft;
  colors[ImGuiCol_ResizeGripHovered] = accentHover;
  colors[ImGuiCol_ResizeGripActive] = accentActive;

  colors[ImGuiCol_Tab] = panel;
  colors[ImGuiCol_TabHovered] = accentHover;
  colors[ImGuiCol_TabSelected] = accentSoft;

  colors[ImGuiCol_PlotLines] = accent;
  colors[ImGuiCol_PlotLinesHovered] = accentHover;
  colors[ImGuiCol_PlotHistogram] = accent;
  colors[ImGuiCol_PlotHistogramHovered] = accentHover;

  colors[ImGuiCol_TextSelectedBg] = accentSoft;
  colors[ImGuiCol_DragDropTarget] = accent;
  colors[ImGuiCol_NavHighlight] = accent;
}

//==============================================================================
void applyModernDarkMetrics()
{
  ImGuiStyle& style = ImGui::GetStyle();

  style.WindowPadding = ImVec2(12.0f, 10.0f);
  style.FramePadding = ImVec2(8.0f, 5.0f);
  style.ItemSpacing = ImVec2(9.0f, 7.0f);
  style.ItemInnerSpacing = ImVec2(6.0f, 5.0f);
  style.ScrollbarSize = 14.0f;
  style.GrabMinSize = 10.0f;

  style.WindowRounding = 7.0f;
  style.ChildRounding = 5.0f;
  style.FrameRounding = 5.0f;
  style.PopupRounding = 5.0f;
  style.TabRounding = 6.0f;
  style.ScrollbarRounding = 9.0f;
  style.GrabRounding = 5.0f;

  style.WindowBorderSize = 1.0f;
  style.FrameBorderSize = 0.0f;
  style.PopupBorderSize = 1.0f;

  style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
}

} // namespace dart_demos
