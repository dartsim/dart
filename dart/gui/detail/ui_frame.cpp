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

#include <dart/gui/detail/debug_overlay.hpp>
#include <dart/gui/detail/frame_viewport.hpp>
#include <dart/gui/detail/gui_scale.hpp>
#include <dart/gui/detail/imgui_overlay.hpp>
#include <dart/gui/detail/input.hpp>
#include <dart/gui/detail/native_window.hpp>
#include <dart/gui/detail/panel.hpp>
#include <dart/gui/detail/perf_hud.hpp>
#include <dart/gui/detail/scenes.hpp>
#include <dart/gui/detail/selection.hpp>
#include <dart/gui/detail/ui_frame.hpp>
#include <dart/gui/profile.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/collision_result.hpp>

#include <dart/common/profile.hpp>

#include <imgui.h>
#ifdef IMGUI_HAS_DOCK
  #include <imgui_internal.h>
#endif

#include <algorithm>
#include <array>
#include <charconv>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart::gui::detail {
namespace {

constexpr std::size_t kMaxDebugLabels = 96u;
constexpr double kPi = 3.14159265358979323846;
#ifdef IMGUI_HAS_DOCK
constexpr char kDockSignatureSeparator = '\x1f';
#endif

std::optional<ImVec2> projectDebugLabel(
    const FrameViewport& viewport, const Eigen::Vector3d& position)
{
  if (!position.allFinite() || viewport.width <= 0 || viewport.height <= 0) {
    return std::nullopt;
  }

  const ViewportPaneFrame& pane = activeViewportPane(viewport);
  if (pane.width <= 0 || pane.height <= 0) {
    return std::nullopt;
  }

  const dart::gui::OrbitCameraBasis basis
      = dart::gui::makeOrbitCameraBasis(pane.camera);
  const Eigen::Vector3d cameraSpace = position - basis.eye;
  const double depth = cameraSpace.dot(basis.forward);
  const dart::gui::PerspectiveProjection projection
      = dart::gui::makePerspectiveProjection(
          pane.camera, pane.width, pane.height);
  if (!std::isfinite(depth) || depth <= projection.nearPlane
      || depth >= projection.farPlane) {
    return std::nullopt;
  }

  const double verticalFovRadians = projection.verticalFovDegrees * kPi / 180.0;
  const double halfHeight = std::tan(verticalFovRadians * 0.5) * depth;
  const double halfWidth = halfHeight * projection.aspectRatio;
  if (!std::isfinite(halfWidth) || !std::isfinite(halfHeight)
      || halfWidth <= 1e-12 || halfHeight <= 1e-12) {
    return std::nullopt;
  }

  const double ndcX = cameraSpace.dot(basis.right) / halfWidth;
  const double ndcY = cameraSpace.dot(basis.up) / halfHeight;
  if (!std::isfinite(ndcX) || !std::isfinite(ndcY) || ndcX < -1.0 || ndcX > 1.0
      || ndcY < -1.0 || ndcY > 1.0) {
    return std::nullopt;
  }

  return ImVec2(
      static_cast<float>(pane.x + (ndcX + 1.0) * 0.5 * pane.width),
      static_cast<float>(pane.y + (1.0 - ndcY) * 0.5 * pane.height));
}

ImU32 toImGuiColor(const Eigen::Vector4d& rgba, double alphaScale = 1.0)
{
  return ImGui::ColorConvertFloat4ToU32(ImVec4(
      static_cast<float>(std::clamp(rgba.x(), 0.0, 1.0)),
      static_cast<float>(std::clamp(rgba.y(), 0.0, 1.0)),
      static_cast<float>(std::clamp(rgba.z(), 0.0, 1.0)),
      static_cast<float>(std::clamp(rgba.w() * alphaScale, 0.0, 1.0))));
}

std::string compactDebugLabel(std::string text)
{
  constexpr std::size_t kMaxLabelLength = 32u;
  if (text.size() <= kMaxLabelLength) {
    return text;
  }
  text.resize(kMaxLabelLength - 3u);
  text += "...";
  return text;
}

void renderDebugLabels(
    const std::vector<dart::gui::DebugLabelDescriptor>& labels,
    const FrameViewport& viewport,
    double effectiveScale)
{
  if (labels.empty()) {
    return;
  }

  auto* drawList = ImGui::GetBackgroundDrawList();
  if (drawList == nullptr) {
    return;
  }

  const float scale = static_cast<float>(std::max(0.5, effectiveScale));
  const ImVec2 padding(4.0f * scale, 2.0f * scale);
  const ImU32 background
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.02f, 0.02f, 0.02f, 0.58f));
  const ImU32 border
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.95f, 0.95f, 0.95f, 0.20f));

  std::size_t rendered = 0u;
  for (const dart::gui::DebugLabelDescriptor& label : labels) {
    if (rendered >= kMaxDebugLabels) {
      break;
    }
    if (label.text.empty()) {
      continue;
    }

    const std::optional<ImVec2> anchor
        = projectDebugLabel(viewport, label.position);
    if (!anchor.has_value()) {
      continue;
    }

    const std::string text = compactDebugLabel(label.text);
    const ImVec2 textSize = ImGui::CalcTextSize(text.c_str());
    const ImVec2 textPos(
        anchor->x + 6.0f * scale, anchor->y - textSize.y * 0.5f);
    const ImVec2 rectMin(textPos.x - padding.x, textPos.y - padding.y);
    const ImVec2 rectMax(
        textPos.x + textSize.x + padding.x, textPos.y + textSize.y + padding.y);

    drawList->AddCircleFilled(
        *anchor, 2.5f * scale, toImGuiColor(label.rgba, 0.95));
    drawList->AddRectFilled(rectMin, rectMax, background, 3.0f * scale);
    drawList->AddRect(rectMin, rectMax, border, 3.0f * scale);
    drawList->AddText(textPos, toImGuiColor(label.rgba), text.c_str());
    ++rendered;
  }
}

bool paneLabelFits(
    const std::string& text,
    float maxWidth,
    float maxHeight,
    const ImVec2& padding)
{
  const ImVec2 textSize = ImGui::CalcTextSize(text.c_str());
  return textSize.x + padding.x * 2.0f <= maxWidth
         && textSize.y + padding.y * 2.0f <= maxHeight;
}

void renderViewportPaneLabels(const FrameViewport& viewport, double guiScale)
{
  if (viewport.paneCount <= 1u) {
    return;
  }

  auto* drawList = ImGui::GetBackgroundDrawList();
  if (drawList == nullptr) {
    return;
  }

  const float scale = static_cast<float>(std::max(0.5, guiScale));
  const ImVec2 padding(6.0f * scale, 3.0f * scale);
  const float inset = 6.0f * scale;
  const ImU32 paneBorder
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.82f, 0.88f, 0.94f, 0.22f));
  const ImU32 activeBorder
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.33f, 0.62f, 0.96f, 0.90f));
  const ImU32 labelBackground
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.03f, 0.04f, 0.05f, 0.68f));
  const ImU32 activeLabelBackground
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.05f, 0.12f, 0.20f, 0.82f));
  const ImU32 labelBorder
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.88f, 0.92f, 0.96f, 0.24f));
  const ImU32 activeLabelBorder
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.43f, 0.72f, 1.0f, 0.78f));
  const ImU32 textColor
      = ImGui::ColorConvertFloat4ToU32(ImVec4(0.92f, 0.95f, 0.98f, 0.96f));

  for (std::size_t i = 0;
       i < viewport.paneCount && i < dart::gui::kMaxViewportPanes;
       ++i) {
    const ViewportPaneFrame& pane = viewport.panes[i];
    if (pane.width <= 0 || pane.height <= 0) {
      continue;
    }

    const ImVec2 paneMin(
        static_cast<float>(pane.x), static_cast<float>(pane.y));
    const ImVec2 paneMax(
        static_cast<float>(pane.x + pane.width),
        static_cast<float>(pane.y + pane.height));
    const ViewportPaneLabelState labelState
        = viewportPaneLabelState(viewport, i);
    drawList->AddRect(
        paneMin,
        paneMax,
        labelState.active ? activeBorder : paneBorder,
        0.0f,
        0,
        labelState.active ? 2.0f * scale : 1.0f * scale);

    const float maxLabelWidth = static_cast<float>(pane.width) - inset * 2.0f;
    const float maxLabelHeight = static_cast<float>(pane.height) - inset * 2.0f;
    if (maxLabelWidth <= 0.0f || maxLabelHeight <= 0.0f) {
      continue;
    }

    std::string text(labelState.text);
    if (labelState.active) {
      const std::string activeText = text + "  Active";
      if (paneLabelFits(activeText, maxLabelWidth, maxLabelHeight, padding)) {
        text = activeText;
      }
    }
    if (!paneLabelFits(text, maxLabelWidth, maxLabelHeight, padding)) {
      continue;
    }

    const ImVec2 textSize = ImGui::CalcTextSize(text.c_str());
    const ImVec2 rectMin(paneMin.x + inset, paneMin.y + inset);
    const ImVec2 rectMax(
        rectMin.x + textSize.x + padding.x * 2.0f,
        rectMin.y + textSize.y + padding.y * 2.0f);
    const ImVec2 textPos(rectMin.x + padding.x, rectMin.y + padding.y);

    drawList->PushClipRect(paneMin, paneMax, true);
    drawList->AddRectFilled(
        rectMin,
        rectMax,
        labelState.active ? activeLabelBackground : labelBackground,
        4.0f * scale);
    drawList->AddRect(
        rectMin,
        rectMax,
        labelState.active ? activeLabelBorder : labelBorder,
        4.0f * scale);
    drawList->AddText(textPos, textColor, text.c_str());
    drawList->PopClipRect();
  }
}

#ifdef IMGUI_HAS_DOCK
void clearDockNodeResizeLocks(ImGuiID dockId)
{
  ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockId);
  if (node == nullptr) {
    return;
  }

  constexpr ImGuiDockNodeFlags kNoResizeFlags
      = static_cast<int>(ImGuiDockNodeFlags_NoResize)
        | static_cast<int>(ImGuiDockNodeFlags_NoResizeX)
        | static_cast<int>(ImGuiDockNodeFlags_NoResizeY);
  node->SharedFlags &= ~kNoResizeFlags;
  node->LocalFlags &= ~kNoResizeFlags;
  node->LocalFlagsInWindows &= ~kNoResizeFlags;
  node->UpdateMergedFlags();

  for (ImGuiDockNode* child : node->ChildNodes) {
    if (child != nullptr) {
      clearDockNodeResizeLocks(child->ID);
    }
  }
}

bool hasDefaultDockSide(const dart::gui::Panel& panel)
{
  return !panel.title.empty() && panel.dockSide != dart::gui::DockSide::None;
}

std::string defaultDockSignature(const dart::gui::Panel& panel)
{
  std::string signature = panel.title;
  signature.push_back(kDockSignatureSeparator);
  signature += std::to_string(static_cast<int>(panel.dockSide));
  return signature;
}

std::vector<std::string> defaultDockLayoutSignature(
    const std::vector<dart::gui::Panel>& panels)
{
  std::vector<std::string> signature;
  signature.reserve(panels.size());
  for (const auto& panel : panels) {
    if (hasDefaultDockSide(panel)) {
      signature.push_back(defaultDockSignature(panel));
    }
  }
  std::sort(signature.begin(), signature.end());
  return signature;
}

bool containsSignature(
    const std::vector<std::string>& signatures, const std::string& value)
{
  return std::binary_search(signatures.begin(), signatures.end(), value);
}

std::string_view dockTitleFromSignature(const std::string& signature)
{
  const std::size_t separator = signature.rfind(kDockSignatureSeparator);
  if (separator == std::string::npos) {
    return {};
  }
  return std::string_view(signature.data(), separator);
}

std::optional<dart::gui::DockSide> dockSideFromSignature(
    const std::string& signature)
{
  const std::size_t separator = signature.rfind(kDockSignatureSeparator);
  if (separator == std::string::npos || separator + 1u >= signature.size()) {
    return std::nullopt;
  }

  int value = 0;
  const char* const begin = signature.data() + separator + 1u;
  const char* const end = signature.data() + signature.size();
  const auto [cursor, error] = std::from_chars(begin, end, value);
  if (error != std::errc{} || cursor != end) {
    return std::nullopt;
  }

  const auto side = static_cast<dart::gui::DockSide>(value);
  switch (side) {
    case dart::gui::DockSide::None:
    case dart::gui::DockSide::Left:
    case dart::gui::DockSide::Right:
    case dart::gui::DockSide::Top:
    case dart::gui::DockSide::Bottom:
    case dart::gui::DockSide::Center:
      return side;
  }
  return std::nullopt;
}

ImGuiID findDockTargetForPreviousSide(
    const std::vector<std::string>& previousSignature, dart::gui::DockSide side)
{
  for (const auto& signature : previousSignature) {
    if (dockSideFromSignature(signature) != side) {
      continue;
    }

    const std::string_view title = dockTitleFromSignature(signature);
    if (title.empty()) {
      continue;
    }

    const std::string titleValue(title);
    ImGuiWindow* window = ImGui::FindWindowByName(titleValue.c_str());
    if (window != nullptr && window->DockNode != nullptr) {
      return window->DockNode->ID;
    }
  }
  return 0;
}

bool containsDockSide(
    const std::vector<std::string>& signature, dart::gui::DockSide side)
{
  return std::ranges::any_of(signature, [side](const std::string& entry) {
    return dockSideFromSignature(entry) == side;
  });
}

std::vector<std::string> mergeDockLayoutSideHistory(
    const std::vector<std::string>& currentSignature,
    const std::vector<std::string>& previousHistory)
{
  std::vector<std::string> history = currentSignature;
  for (const std::string& entry : previousHistory) {
    const auto side = dockSideFromSignature(entry);
    if (!side.has_value() || containsDockSide(history, *side)) {
      continue;
    }
    history.push_back(entry);
  }
  return history;
}

// Builds a default IDE-style dock layout: top/bottom bars span full width and
// left/right columns fill the remaining middle, leaving a transparent central
// node for the 3D viewport. Each panel docks into the region named by its
// DockSide; panels sharing a side become tabs. Only sides actually used are
// split, so unused regions take no space.
void buildDefaultDockLayout(
    ImGuiID dockId, const std::vector<dart::gui::Panel>& panels)
{
  using dart::gui::DockSide;

  bool useTop = false;
  // Reserve a bottom region for either application-owned bottom panels or the
  // built-in status panel. When an application owns the bottom region (for
  // example a replay timeline), the status panel moves to a side tab so the
  // primary scene control remains visible on first launch.
  bool useApplicationBottom = false;
  bool useBottom = true;
  bool useLeft = false;
  bool useRight = false;
  float topFraction = 0.07f;
  float bottomFraction = 0.12f;
  for (const auto& panel : panels) {
    switch (panel.dockSide) {
      case DockSide::Top:
        useTop = true;
        if (panel.initialSize.has_value()) {
          const float viewportHeight = ImGui::GetMainViewport()->Size.y;
          if (viewportHeight > 0.0f) {
            topFraction = std::max(
                topFraction,
                static_cast<float>((*panel.initialSize)[1]) / viewportHeight);
          }
        }
        break;
      case DockSide::Bottom:
        useApplicationBottom = true;
        useBottom = true;
        if (panel.initialSize.has_value()) {
          const float viewportHeight = ImGui::GetMainViewport()->Size.y;
          if (viewportHeight > 0.0f) {
            bottomFraction = std::max(
                bottomFraction,
                static_cast<float>((*panel.initialSize)[1]) / viewportHeight);
          }
        }
        break;
      case DockSide::Left:
        useLeft = true;
        break;
      case DockSide::Right:
        useRight = true;
        break;
      case DockSide::Center:
      case DockSide::None:
        break;
    }
  }

  ImGui::DockBuilderRemoveNode(dockId);
  // ImGuiDockNodeFlags_DockSpace is a private flag while PassthruCentralNode is
  // public; OR-ing the two enum types directly trips C++20's
  // -Wdeprecated-enum-enum-conversion (and DART builds with -Werror), so
  // combine them as ints.
  ImGui::DockBuilderAddNode(
      dockId,
      static_cast<int>(ImGuiDockNodeFlags_DockSpace)
          | static_cast<int>(ImGuiDockNodeFlags_PassthruCentralNode));
  ImGui::DockBuilderSetNodeSize(dockId, ImGui::GetMainViewport()->Size);
  topFraction = std::clamp(topFraction, 0.07f, 0.18f);
  bottomFraction = std::clamp(bottomFraction, 0.10f, 0.45f);

  ImGuiID center = dockId;
  ImGuiID top = 0;
  ImGuiID bottom = 0;
  ImGuiID left = 0;
  ImGuiID right = 0;
  if (useTop) {
    top = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Up, topFraction, nullptr, &center);
  }
  if (useBottom) {
    bottom = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Down, bottomFraction, nullptr, &center);
  }
  if (useLeft) {
    left = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Left, 0.24f, nullptr, &center);
  }
  if (useRight) {
    right = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Right, 0.34f, nullptr, &center);
  }

  for (const auto& panel : panels) {
    if (panel.title.empty()) {
      continue;
    }
    ImGuiID target = 0;
    switch (panel.dockSide) {
      case DockSide::Top:
        target = top;
        break;
      case DockSide::Bottom:
        target = bottom;
        break;
      case DockSide::Left:
        target = left;
        break;
      case DockSide::Right:
        target = right;
        break;
      case DockSide::Center:
        target = center;
        break;
      case DockSide::None:
        continue;
    }
    ImGui::DockBuilderDockWindow(panel.title.c_str(), target);
  }

  // Fold the built-in scene/debug status panel into the docked layout instead
  // of floating over the viewport. Application-owned bottom panels are primary
  // controls, so keep them visible by docking status into a side region when
  // possible.
  if (!useApplicationBottom && bottom != 0) {
    ImGui::DockBuilderDockWindow(kBuiltInStatusPanelTitle, bottom);
  } else if (left != 0) {
    ImGui::DockBuilderDockWindow(kBuiltInStatusPanelTitle, left);
  } else if (right != 0) {
    ImGui::DockBuilderDockWindow(kBuiltInStatusPanelTitle, right);
  } else if (bottom != 0) {
    ImGui::DockBuilderDockWindow(kBuiltInStatusPanelTitle, bottom);
  }

  clearDockNodeResizeLocks(dockId);
  ImGui::DockBuilderFinish(dockId);
}

std::vector<std::string_view> initialFocusPanelsForDefaultDockLayout(
    const std::vector<dart::gui::Panel>& panels);

ImGuiID findExistingDockTargetForSide(
    const std::vector<dart::gui::Panel>& panels,
    dart::gui::DockSide side,
    std::string_view excludedTitle)
{
  for (const auto& panel : panels) {
    if (panel.title.empty() || std::string_view(panel.title) == excludedTitle
        || panel.dockSide != side) {
      continue;
    }

    ImGuiWindow* window = ImGui::FindWindowByName(panel.title.c_str());
    if (window != nullptr && window->DockNode != nullptr) {
      return window->DockNode->ID;
    }
  }
  return 0;
}

std::vector<std::string_view> dockPanelsIntroducedBySceneSwitch(
    ImGuiID dockId,
    const std::vector<dart::gui::Panel>& panels,
    const std::vector<std::string>& previousSignature,
    const std::vector<std::string>& sideHistory)
{
  std::vector<std::string_view> focusedTitles;
  for (const auto& panel : panels) {
    if (!hasDefaultDockSide(panel)
        || containsSignature(previousSignature, defaultDockSignature(panel))) {
      continue;
    }

    const ImGuiID target
        = findExistingDockTargetForSide(panels, panel.dockSide, panel.title);
    const ImGuiID previousTarget = target != 0
                                       ? target
                                       : findDockTargetForPreviousSide(
                                             previousSignature, panel.dockSide);
    const ImGuiID historicalTarget
        = previousTarget != 0
              ? previousTarget
              : findDockTargetForPreviousSide(sideHistory, panel.dockSide);
    if (historicalTarget == 0) {
      // A new side appeared and there is no existing dock node to join. Fall
      // back to the default builder so the new panel is still docked instead of
      // floating over the viewport.
      buildDefaultDockLayout(dockId, panels);
      return initialFocusPanelsForDefaultDockLayout(panels);
    }

    ImGui::DockBuilderDockWindow(panel.title.c_str(), historicalTarget);
    focusedTitles.push_back(panel.title);
  }
  return focusedTitles;
}

std::vector<std::string_view> initialFocusPanelsForDefaultDockLayout(
    const std::vector<dart::gui::Panel>& panels)
{
  bool useBottom = false;
  bool useLeft = false;
  bool useRight = false;
  std::vector<std::string_view> titles;
  titles.reserve(3u);
  for (const auto& panel : panels) {
    if (panel.title.empty()) {
      continue;
    }
    switch (panel.dockSide) {
      case dart::gui::DockSide::Bottom:
        if (!useBottom) {
          titles.push_back(panel.title);
          useBottom = true;
        }
        break;
      case dart::gui::DockSide::Left:
        if (!useLeft) {
          titles.push_back(panel.title);
          useLeft = true;
        }
        break;
      case dart::gui::DockSide::Right:
        if (!useRight) {
          titles.push_back(panel.title);
          useRight = true;
        }
        break;
      case dart::gui::DockSide::Top:
      case dart::gui::DockSide::Center:
      case dart::gui::DockSide::None:
        break;
    }
  }
  return titles;
}

void selectInitialDockedPanels(std::span<const std::string_view> titles)
{
  for (const std::string_view title : titles) {
    if (title.empty()) {
      continue;
    }
    const std::string titleValue(title);
    ImGuiWindow* window = ImGui::FindWindowByName(titleValue.c_str());
    if (window == nullptr || window->DockNode == nullptr) {
      continue;
    }
    window->DockNode->SelectedTabId = window->TabId;
    if (window->DockNode->TabBar != nullptr) {
      window->DockNode->TabBar->SelectedTabId = window->TabId;
      window->DockNode->TabBar->NextSelectedTabId = window->TabId;
    }
  }
}
#endif // IMGUI_HAS_DOCK

} // namespace

void updateFrameUi(
    GLFWwindow* window,
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& debugMaterial,
    ImGuiOverlay& imguiOverlay,
    ImGuiIO& imguiIo,
    const FrameViewport& viewport,
    ExampleScene exampleScene,
    DartScene& dartScene,
    dart::gui::OrbitCameraController& cameraController,
    const SelectionController& selectionController,
    bool& orbitLight,
    bool& headlightsEnabled,
    DebugOverlayController& debugOverlays,
    std::vector<dart::gui::Panel>& panels,
    dart::gui::ViewerLifecycleState& lifecycle,
    const GuiScaleState& guiScale,
    dart::gui::ProfileAccumulator& profile,
    bool showPerfHud,
    PerfHudState& perfHud,
    const char* backendName)
{
  DART_PROFILE_SCOPED_N("updateFrameUi");
  auto phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  updateImGuiMouseInput(window, imguiIo, viewport.width, viewport.height);
  ImGui::NewFrame();
#ifdef IMGUI_HAS_DOCK
  std::vector<std::string_view> initialFocusPanelTitles;
  if (dartScene.dockingEnabled) {
    const ImGuiID dockId = ImHashStr("DARTMainDockSpace");
    const auto dockSignature = defaultDockLayoutSignature(panels);
    const bool resetDockLayout
        = dart::gui::consumeDockLayoutResetRequest(lifecycle);
    if (resetDockLayout || !lifecycle.dockLayoutInitialized) {
      lifecycle.dockLayoutInitialized = true;
      lifecycle.dockedPanelLayoutSignature = dockSignature;
      lifecycle.dockedPanelLayoutSideHistory = dockSignature;
      initialFocusPanelTitles = initialFocusPanelsForDefaultDockLayout(panels);
      // Apply the default layout deterministically on startup. The py-demos
      // workspace is an examples browser first; a stale imgui.ini layout should
      // not make its first frame look broken or obscure the viewport.
      buildDefaultDockLayout(dockId, panels);
    } else if (dockSignature != lifecycle.dockedPanelLayoutSignature) {
      initialFocusPanelTitles = dockPanelsIntroducedBySceneSwitch(
          dockId,
          panels,
          lifecycle.dockedPanelLayoutSignature,
          lifecycle.dockedPanelLayoutSideHistory);
      lifecycle.dockedPanelLayoutSignature = dockSignature;
      lifecycle.dockedPanelLayoutSideHistory = mergeDockLayoutSideHistory(
          dockSignature, lifecycle.dockedPanelLayoutSideHistory);
    }
    ImGui::DockSpaceOverViewport(
        dockId,
        ImGui::GetMainViewport(),
        ImGuiDockNodeFlags_PassthruCentralNode);
  }
#else
  dart::gui::consumeDockLayoutResetRequest(lifecycle);
#endif
  const std::string selectedLabel = selectionController.selectedLabel();
  const auto cameraBasis
      = dart::gui::makeOrbitCameraBasis(cameraController.camera);
  dart::gui::PanelContext::UiState uiState;
  uiState.displaySize
      = Eigen::Vector2d(imguiIo.DisplaySize.x, imguiIo.DisplaySize.y);
  uiState.framebufferScale = Eigen::Vector2d(
      imguiIo.DisplayFramebufferScale.x, imguiIo.DisplayFramebufferScale.y);
  uiState.fontSize = ImGui::GetFontSize();
  uiState.fontGlobalScale = imguiIo.FontGlobalScale;
  uiState.userScale = guiScale.userScale;
  uiState.dpiScale = guiScale.dpiScale;
  uiState.uiScale = guiScale.effectiveScale;
  if (imguiIo.Fonts != nullptr) {
    unsigned char* pixels = nullptr;
    int width = 0;
    int height = 0;
    imguiIo.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
    if (pixels != nullptr && width > 0 && height > 0) {
      uiState.fontTextureSize = std::array<int, 2>{width, height};
    }
  }
  dart::gui::PanelContext panelContext{
      &lifecycle,
      selectedLabel,
      selectionController.selectedPoint(),
      selectionController.selectedNormal(),
      dartScene.time,
      dartScene.contactCount,
      {cameraBasis.eye,
       cameraController.camera.target,
       cameraBasis.up,
       cameraController.camera,
       [&cameraController](const dart::gui::OrbitCamera& camera) {
         cameraController.camera = camera;
         dart::gui::resetOrbitCameraTracking(cameraController);
       }},
      {&headlightsEnabled},
      {&dartScene.renderSettings},
      uiState};
  panelContext.nativeWindow
      = window == nullptr ? nullptr : getNativeWindow(window);
  // Float the built-in status HUD to the right of any left-docked sidebar (the
  // demos catalog) so the two panels do not overlap by default. Standalone
  // viewers have no left sidebar, so the inset stays 0 and the HUD keeps its
  // {20,20} corner. Docking, when enabled, places the HUD via the dock layout
  // and ignores this floating position.
  double statusPanelLeftPx = 20.0;
  for (const auto& panel : panels) {
    if (panel.dockSide == dart::gui::DockSide::Left
        && panel.initialPosition.has_value() && panel.initialSize.has_value()) {
      statusPanelLeftPx = std::max(
          statusPanelLeftPx,
          (*panel.initialPosition)[0] + (*panel.initialSize)[0] + 16.0);
    }
  }
  const bool debugOptionsChanged = renderBuiltInStatusPanel(
      sceneName(exampleScene),
      panelContext.simulationTime,
      panelContext.contactCount,
      panelContext.selectedLabel,
      selectionController.interactionStatus(),
      !dartScene.ikHandles.empty(),
      orbitLight,
      debugOverlays.staticOptions,
      debugOverlays.contactOptions,
      lifecycle,
      guiScale.effectiveScale,
      dartScene.dockingEnabled,
      statusPanelLeftPx);
  if (debugOptionsChanged) {
    refreshStaticDebugOverlay(engine, scene, debugMaterial, debugOverlays);
  }
  renderApplicationPanels(
      panels, panelContext, guiScale.effectiveScale, dartScene.dockingEnabled);
#ifdef IMGUI_HAS_DOCK
  selectInitialDockedPanels(initialFocusPanelTitles);
#endif
  updateViewportPaneActivation(
      dartScene.viewportPaneActivation,
      window,
      viewport,
      imguiIo,
      true,
      cameraController,
      dartScene.onViewportPaneActivated);
  renderViewportPaneLabels(viewport, guiScale.effectiveScale);
  if (dartScene.debugLabels) {
    renderDebugLabels(
        dartScene.debugLabels(), viewport, guiScale.effectiveScale);
  }
  if (!debugOverlays.providerLabels.empty()) {
    renderDebugLabels(
        debugOverlays.providerLabels, viewport, guiScale.effectiveScale);
  }
  if (showPerfHud) {
    drawPerfHud(perfHud, profile, backendName);
  }
  ImGui::Render();
  updateImGuiMouseCursor(window, imguiIo);
  updateImGuiOverlay(
      engine,
      imguiOverlay,
      ImGui::GetDrawData(),
      static_cast<std::uint32_t>(viewport.width),
      static_cast<std::uint32_t>(viewport.height));
  profile.uiMs += dart::gui::elapsedMs(phaseStart);
}

} // namespace dart::gui::detail
