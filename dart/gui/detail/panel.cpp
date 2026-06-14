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

#include <dart/gui/application.hpp>
#include <dart/gui/detail/panel.hpp>

#include <imgui.h>

#include <algorithm>
#include <array>
#include <limits>
#include <string>

namespace {

// Bounds for the built-in debug-tuning sliders (doubles so they can back
// ImGui::SliderScalar with ImGuiDataType_Double directly).
constexpr double kDebugGridSpacingMin = 0.1;
constexpr double kDebugGridSpacingMax = 2.0;
constexpr double kDebugAxisLengthMin = 0.05;
constexpr double kDebugAxisLengthMax = 1.5;
constexpr double kDebugForceScaleMin = 0.0001;
constexpr double kDebugForceScaleMax = 0.02;

std::string visiblePanelLabel(std::string_view label)
{
  const auto idMarker = label.find("##");
  if (idMarker == std::string_view::npos) {
    return std::string(label);
  }
  return std::string(label.substr(0, idMarker));
}

void renderPlotLines(
    std::string_view label, std::span<const double> values, float height)
{
  const std::string labelValue(label);
  const std::string displayLabel = visiblePanelLabel(label);
  const auto valueGetter = [](void* data, int index) {
    const auto* values = static_cast<const std::span<const double>*>(data);
    return static_cast<float>((*values)[static_cast<std::size_t>(index)]);
  };
  const auto valueCount = values.size() > static_cast<std::size_t>(
                              std::numeric_limits<int>::max())
                              ? std::numeric_limits<int>::max()
                              : static_cast<int>(values.size());
  ImGui::PushID(labelValue.c_str());
  if (!displayLabel.empty()) {
    ImGui::TextUnformatted(displayLabel.c_str());
  }
  ImGui::SetNextItemWidth(-std::numeric_limits<float>::min());
  ImGui::PlotLines(
      "##lines",
      valueGetter,
      &values,
      valueCount,
      0,
      nullptr,
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(),
      ImVec2(0.0f, height));
  ImGui::PopID();
}

class ImGuiPanelBuilder final : public dart::gui::PanelBuilder
{
public:
  void text(std::string_view value) override
  {
    const std::string textValue(value);
    ImGui::TextWrapped("%s", textValue.c_str());
  }

  void separator() override
  {
    ImGui::Separator();
  }

  void sameLine() override
  {
    ImGui::SameLine();
  }

  bool button(std::string_view label) override
  {
    const std::string labelValue(label);
    return ImGui::Button(labelValue.c_str());
  }

  bool selectable(std::string_view label, bool selected) override
  {
    const std::string labelValue(label);
    return ImGui::Selectable(labelValue.c_str(), selected);
  }

  void itemTooltip(std::string_view text) override
  {
    if (text.empty() || !ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
      return;
    }
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0F);
    ImGui::TextUnformatted(text.data(), text.data() + text.size());
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }

  void indent(double width) override
  {
    ImGui::Indent(static_cast<float>(width));
  }

  void unindent(double width) override
  {
    ImGui::Unindent(static_cast<float>(width));
  }

  bool checkbox(std::string_view label, bool& value) override
  {
    const std::string labelValue(label);
    return ImGui::Checkbox(labelValue.c_str(), &value);
  }

  bool textInput(std::string_view label, std::string& value) override
  {
    const std::string labelValue(label);
    std::array<char, 256> buffer{};
    value.copy(buffer.data(), buffer.size() - 1);
    if (!ImGui::InputText(labelValue.c_str(), buffer.data(), buffer.size())) {
      return false;
    }

    value = buffer.data();
    return true;
  }

  bool slider(
      std::string_view label,
      double& value,
      double minimum,
      double maximum) override
  {
    const std::string labelValue(label);
    const std::string displayLabel = visiblePanelLabel(label);
    ImGui::PushID(labelValue.c_str());
    if (!displayLabel.empty()) {
      ImGui::TextUnformatted(displayLabel.c_str());
    }
    ImGui::SetNextItemWidth(-std::numeric_limits<float>::min());
    const bool changed = ImGui::SliderScalar(
        "##value", ImGuiDataType_Double, &value, &minimum, &maximum);
    ImGui::PopID();
    return changed;
  }

  bool timeline(
      std::string_view label,
      double& value,
      double minimum,
      double maximum,
      std::span<const double> valueTrack,
      std::span<const double> markerTrack,
      std::span<const double> cursorTrack,
      std::string_view valueTrackLabel) override
  {
    const std::string labelValue(label);
    const std::string displayLabel = visiblePanelLabel(label);
    ImGui::PushID(labelValue.c_str());
    if (!displayLabel.empty()) {
      ImGui::TextUnformatted(displayLabel.c_str());
    }
    ImGui::SetNextItemWidth(-std::numeric_limits<float>::min());
    const bool changed = ImGui::SliderScalar(
        "##value", ImGuiDataType_Double, &value, &minimum, &maximum);

    if (!valueTrack.empty()) {
      renderPlotLines(valueTrackLabel, valueTrack, 44.0f);
    }
    if (!markerTrack.empty()) {
      renderPlotLines("Marks", markerTrack, 20.0f);
    }
    if (!cursorTrack.empty()) {
      renderPlotLines("Cursor", cursorTrack, 20.0f);
    }

    ImGui::PopID();
    return changed;
  }

  bool colorEdit(std::string_view label, Eigen::Vector4d& rgba) override
  {
    const std::string labelValue(label);
    float color[4]
        = {static_cast<float>(rgba[0]),
           static_cast<float>(rgba[1]),
           static_cast<float>(rgba[2]),
           static_cast<float>(rgba[3])};
    if (!ImGui::ColorEdit4(labelValue.c_str(), color)) {
      return false;
    }

    rgba = Eigen::Vector4d(color[0], color[1], color[2], color[3]);
    return true;
  }

  bool select(
      std::string_view label,
      int& selectedIndex,
      std::span<const std::string_view> choices) override
  {
    if (choices.empty()) {
      return false;
    }

    selectedIndex
        = std::clamp(selectedIndex, 0, static_cast<int>(choices.size()) - 1);
    const std::string labelValue(label);
    const std::string preview(choices[static_cast<std::size_t>(selectedIndex)]);
    bool changed = false;
    if (ImGui::BeginCombo(labelValue.c_str(), preview.c_str())) {
      for (std::size_t i = 0; i < choices.size(); ++i) {
        const bool selected = static_cast<int>(i) == selectedIndex;
        const std::string choice(choices[i]);
        if (ImGui::Selectable(choice.c_str(), selected)) {
          selectedIndex = static_cast<int>(i);
          changed = true;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    return changed;
  }

  void colorSwatch(std::string_view label, const Eigen::Vector4d& rgba) override
  {
    const std::string labelValue(label);
    const std::string buttonId = "##color_" + labelValue;
    ImGui::ColorButton(
        buttonId.c_str(),
        ImVec4(
            static_cast<float>(rgba[0]),
            static_cast<float>(rgba[1]),
            static_cast<float>(rgba[2]),
            static_cast<float>(rgba[3])),
        ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoInputs,
        ImVec2(18.0f, 18.0f));
    ImGui::SameLine();
    ImGui::TextUnformatted(labelValue.c_str());
  }

  void plotLines(
      std::string_view label, std::span<const double> values) override
  {
    renderPlotLines(label, values, 72.0f);
  }

  bool beginTable(
      std::string_view label,
      std::span<const std::string_view> columns) override
  {
    const std::string labelValue(label);
    const ImGuiTableFlags flags
        = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
          | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_Resizable;
    if (!ImGui::BeginTable(
            labelValue.c_str(), static_cast<int>(columns.size()), flags)) {
      return false;
    }

    for (std::string_view column : columns) {
      const std::string columnValue(column);
      ImGui::TableSetupColumn(columnValue.c_str());
    }
    ImGui::TableHeadersRow();
    return true;
  }

  void tableNextRow() override
  {
    ImGui::TableNextRow();
  }

  bool tableNextColumn() override
  {
    return ImGui::TableNextColumn();
  }

  void endTable() override
  {
    ImGui::EndTable();
  }

  bool collapsingHeader(std::string_view label, bool defaultOpen) override
  {
    const std::string labelValue(label);
    ImGuiTreeNodeFlags flags = 0;
    if (defaultOpen) {
      flags |= ImGuiTreeNodeFlags_DefaultOpen;
    }
    return ImGui::CollapsingHeader(labelValue.c_str(), flags);
  }

  bool beginMenuBar() override
  {
    return ImGui::BeginMenuBar();
  }

  void endMenuBar() override
  {
    ImGui::EndMenuBar();
  }

  bool beginMenu(std::string_view label) override
  {
    const std::string labelValue(label);
    return ImGui::BeginMenu(labelValue.c_str());
  }

  void endMenu() override
  {
    ImGui::EndMenu();
  }

  bool menuItem(std::string_view label) override
  {
    const std::string labelValue(label);
    return ImGui::MenuItem(labelValue.c_str());
  }

  void openModal(std::string_view label, bool& open) override
  {
    const std::string labelValue(label);
    open = true;
    ImGui::OpenPopup(labelValue.c_str());
  }

  bool beginModal(std::string_view label, bool& open) override
  {
    const std::string labelValue(label);
    return ImGui::BeginPopupModal(
        labelValue.c_str(), &open, ImGuiWindowFlags_AlwaysAutoResize);
  }

  void endModal() override
  {
    ImGui::EndPopup();
  }
};

} // namespace

namespace dart::gui::detail {

bool renderBuiltInStatusPanel(
    const char* sceneName,
    double simulationTime,
    std::size_t contactCount,
    const std::string& selectedLabel,
    const std::string& interactionStatus,
    bool showIkHint,
    bool& orbitLight,
    DebugDrawOptions& staticDebugOptions,
    DebugDrawOptions& contactDebugOptions,
    ViewerLifecycleState& lifecycle,
    double guiScale,
    bool dockingEnabled,
    double leftMarginPx)
{
  const bool dockingActive = dockingEnabled && dart::gui::isDockingAvailable();
  ImGui::SetNextWindowPos(
      {static_cast<float>(leftMarginPx) * static_cast<float>(guiScale),
       20.0f * static_cast<float>(guiScale)},
      ImGuiCond_FirstUseEver);
  // When docked, this panel joins the dock layout: keep it opaque and let
  // ImGui persist/manage its size, rather than a translucent auto-resizing
  // floating overlay.
  ImGuiWindowFlags statusFlags
      = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings;
  if (dockingActive) {
    statusFlags = 0;
  }
  ImGui::SetNextWindowBgAlpha(dockingActive ? 1.0f : 0.72f);
  ImGui::Begin(kBuiltInStatusPanelTitle, nullptr, statusFlags);

  const auto renderViewerHelp = [&] {
    ImGui::PushTextWrapPos(
        ImGui::GetCursorPosX() + 300.0f * static_cast<float>(guiScale));
    ImGui::TextWrapped(
        "DART scene viewer: inspect renderables, shadows, and debug overlays.");
    ImGui::TextWrapped(
        "Mouse: left-drag empty space to orbit; left-drag a dynamic body or sx "
        "renderable to apply external force with a visible spring line; "
        "right/middle pan, wheel zoom, click select.");
    ImGui::TextWrapped(
        "Keys: Space pause, N step, arrows/Pg or Ctrl-left drag selected, "
        "Esc exit.");
    if (showIkHint) {
      ImGui::TextWrapped(
          "IK: press 1-4 to show/select a target, then drag its gizmo.");
    }
    ImGui::PopTextWrapPos();
  };

  if (!dockingActive) {
    renderViewerHelp();
    ImGui::Separator();
  }

  if (dockingActive) {
    ImGui::Text(
        "scene: %s  time: %.3f  contacts: %zu  selected: %s",
        sceneName,
        simulationTime,
        contactCount,
        selectedLabel.c_str());
    if (!interactionStatus.empty()) {
      ImGui::Text("%s", interactionStatus.c_str());
    }
  } else {
    ImGui::Text("scene: %s", sceneName);
    ImGui::Text("time: %.3f", simulationTime);
    ImGui::Text("contacts: %zu", contactCount);
    ImGui::PushTextWrapPos(
        ImGui::GetCursorPosX() + 300.0f * static_cast<float>(guiScale));
    ImGui::Text("selected: %s", selectedLabel.c_str());
    if (!interactionStatus.empty()) {
      ImGui::Text("%s", interactionStatus.c_str());
    }
    ImGui::PopTextWrapPos();
  }
  if (!dockingActive) {
    if (ImGui::Button(lifecycle.paused ? "Resume" : "Pause")) {
      togglePaused(lifecycle);
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
      requestSingleStep(lifecycle);
    }
    ImGui::SameLine();
  }
  ImGui::Checkbox("Orbit light", &orbitLight);

  bool debugOptionsChanged = false;
  const auto renderDebugControls = [&] {
    bool changed = false;
    changed |= ImGui::Checkbox("Grid", &staticDebugOptions.drawGrid);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("World", &staticDebugOptions.drawWorldFrame);
    changed |= ImGui::Checkbox("Contacts", &contactDebugOptions.drawContacts);
    ImGui::SameLine();
    changed
        |= ImGui::Checkbox("Normals", &contactDebugOptions.drawContactNormals);
    ImGui::SameLine();
    changed |= ImGui::Checkbox(
        "Contact forces", &contactDebugOptions.drawContactForces);

    if (ImGui::TreeNode("Debug tuning")) {
      const float sliderWidth = 140.0f * static_cast<float>(guiScale);
      ImGui::PushItemWidth(sliderWidth);
      changed |= ImGui::SliderScalar(
          "Grid spacing",
          ImGuiDataType_Double,
          &staticDebugOptions.gridSpacing,
          &kDebugGridSpacingMin,
          &kDebugGridSpacingMax,
          "%.2f");
      changed |= ImGui::SliderScalar(
          "World axis len",
          ImGuiDataType_Double,
          &staticDebugOptions.worldFrameAxisLength,
          &kDebugAxisLengthMin,
          &kDebugAxisLengthMax,
          "%.2f");
      changed |= ImGui::SliderScalar(
          "Contact force scale",
          ImGuiDataType_Double,
          &contactDebugOptions.contactForceScale,
          &kDebugForceScaleMin,
          &kDebugForceScaleMax,
          "%.4f");
      ImGui::PopItemWidth();
      ImGui::TreePop();
    }
    return changed;
  };

  if (dockingActive) {
    ImGui::SameLine();
    if (ImGui::Button("Debug overlays")) {
      ImGui::OpenPopup("DART debug overlays");
    }
    if (ImGui::BeginPopup("DART debug overlays")) {
      debugOptionsChanged |= renderDebugControls();
      ImGui::EndPopup();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Viewer help");
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
      ImGui::BeginTooltip();
      renderViewerHelp();
      ImGui::EndTooltip();
    }
    ImGui::End();
    return debugOptionsChanged;
  }

  debugOptionsChanged |= renderDebugControls();
  ImGui::End();

  return debugOptionsChanged;
}

void renderApplicationPanels(
    std::vector<dart::gui::Panel>& panels,
    dart::gui::PanelContext& context,
    double guiScale,
    bool dockingEnabled)
{
  const bool dockingActive = dockingEnabled && dart::gui::isDockingAvailable();
  ImGuiPanelBuilder builder;
  std::size_t defaultPositionIndex = 0u;
  for (auto& panel : panels) {
    if (!panel.build && !panel.buildWithContext) {
      continue;
    }

    const char* title
        = panel.title.empty() ? "DART Controls" : panel.title.c_str();
    const float scale = static_cast<float>(guiScale);
    if (panel.initialPosition.has_value()) {
      ImGui::SetNextWindowPos(
          {static_cast<float>((*panel.initialPosition)[0] * scale),
           static_cast<float>((*panel.initialPosition)[1] * scale)},
          ImGuiCond_FirstUseEver);
    } else {
      const float panelWidth = static_cast<float>(
          panel.initialSize.has_value() ? (*panel.initialSize)[0] * guiScale
                                        : 320.0 * guiScale);
      const float margin = 20.0f * scale;
      const float x = std::max(
          margin,
          static_cast<float>(context.ui.displaySize.x()) - panelWidth - margin);
      const float y
          = margin + static_cast<float>(defaultPositionIndex) * 96.0f * scale;
      ImGui::SetNextWindowPos({x, y}, ImGuiCond_FirstUseEver);
      ++defaultPositionIndex;
    }
    // Floating overlay panels are translucent so the 3D scene shows through;
    // docked panels sit in opaque side regions where translucency hurts
    // legibility, so they default to fully opaque.
    const double defaultAlpha = dockingActive ? 1.0 : 0.72;
    ImGui::SetNextWindowBgAlpha(
        static_cast<float>(panel.backgroundAlpha.value_or(defaultAlpha)));
    if (panel.initialSize.has_value()) {
      ImGui::SetNextWindowSize(
          {static_cast<float>((*panel.initialSize)[0] * guiScale),
           static_cast<float>((*panel.initialSize)[1] * guiScale)},
          ImGuiCond_FirstUseEver);
    } else {
      ImGui::SetNextWindowSize({320.0f * scale, 0.0f}, ImGuiCond_FirstUseEver);
    }

    // The default dock builder owns startup placement when docking is active;
    // those builder assignments flow through ImGui's window settings before
    // the first Begin(). Floating overlays stay unsaved so they remain
    // deterministic outside a dockspace.
    ImGuiWindowFlags windowFlags = 0;
    if (!dockingActive) {
      windowFlags |= ImGuiWindowFlags_NoSavedSettings;
    }
    if (panel.autoResize && !dockingActive) {
      windowFlags |= ImGuiWindowFlags_AlwaysAutoResize;
    }
    if (panel.horizontalScrollbar) {
      windowFlags |= ImGuiWindowFlags_HorizontalScrollbar;
    }
    if (panel.menuBar) {
      windowFlags |= ImGuiWindowFlags_MenuBar;
    }
    ImGui::Begin(title, nullptr, windowFlags);
    if (panel.build) {
      panel.build(builder);
    }
    if (panel.buildWithContext) {
      panel.buildWithContext(builder, context);
    }
    ImGui::End();
  }
}

} // namespace dart::gui::detail
