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
#include <dart/gui/detail/imgui_overlay.hpp>
#include <dart/gui/detail/input.hpp>
#include <dart/gui/detail/panel.hpp>
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
#include <optional>
#include <string>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart::gui::detail {
namespace {

constexpr std::size_t kMaxDebugLabels = 96u;
constexpr double kPi = 3.14159265358979323846;

std::optional<ImVec2> projectDebugLabel(
    const dart::gui::OrbitCamera& camera,
    const FrameViewport& viewport,
    const Eigen::Vector3d& position)
{
  if (!position.allFinite() || viewport.width <= 0 || viewport.height <= 0) {
    return std::nullopt;
  }

  const dart::gui::OrbitCameraBasis basis
      = dart::gui::makeOrbitCameraBasis(camera);
  const Eigen::Vector3d cameraSpace = position - basis.eye;
  const double depth = cameraSpace.dot(basis.forward);
  const dart::gui::PerspectiveProjection projection
      = dart::gui::makePerspectiveProjection(
          camera, viewport.width, viewport.height);
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
      static_cast<float>((ndcX + 1.0) * 0.5 * viewport.width),
      static_cast<float>((1.0 - ndcY) * 0.5 * viewport.height));
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
    const dart::gui::OrbitCamera& camera,
    const FrameViewport& viewport,
    double guiScale)
{
  if (labels.empty()) {
    return;
  }

  auto* drawList = ImGui::GetBackgroundDrawList();
  if (drawList == nullptr) {
    return;
  }

  const float scale = static_cast<float>(std::max(0.5, guiScale));
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
        = projectDebugLabel(camera, viewport, label.position);
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

#ifdef IMGUI_HAS_DOCK
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
  bool useBottom = false;
  bool useLeft = false;
  bool useRight = false;
  for (const auto& panel : panels) {
    switch (panel.dockSide) {
      case DockSide::Top:
        useTop = true;
        break;
      case DockSide::Bottom:
        useBottom = true;
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

  ImGuiID center = dockId;
  ImGuiID top = 0;
  ImGuiID bottom = 0;
  ImGuiID left = 0;
  ImGuiID right = 0;
  if (useTop) {
    top = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Up, 0.07f, nullptr, &center);
  }
  if (useBottom) {
    bottom = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Down, 0.26f, nullptr, &center);
  }
  if (useLeft) {
    left = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Left, 0.20f, nullptr, &center);
  }
  if (useRight) {
    right = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Right, 0.25f, nullptr, &center);
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

  // Fold the built-in scene/debug status panel into the bottom region so it
  // joins the docked layout instead of floating over the viewport.
  if (bottom != 0) {
    ImGui::DockBuilderDockWindow(kBuiltInStatusPanelTitle, bottom);
  } else if (left != 0) {
    ImGui::DockBuilderDockWindow(kBuiltInStatusPanelTitle, left);
  }

  ImGui::DockBuilderFinish(dockId);
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
    const dart::gui::OrbitCameraController& cameraController,
    const SelectionController& selectionController,
    bool& orbitLight,
    bool& headlightsEnabled,
    DebugOverlayController& debugOverlays,
    std::vector<dart::gui::Panel>& panels,
    dart::gui::ViewerLifecycleState& lifecycle,
    double guiScale,
    dart::gui::ProfileAccumulator& profile)
{
  DART_PROFILE_SCOPED_N("updateFrameUi");
  auto phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  updateImGuiMouseInput(window, imguiIo, viewport.width, viewport.height);
  ImGui::NewFrame();
#ifdef IMGUI_HAS_DOCK
  if (dartScene.dockingEnabled) {
    const ImGuiID dockId = ImGui::DockSpaceOverViewport(
        0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
    if (!dartScene.dockLayoutInitialized) {
      dartScene.dockLayoutInitialized = true;
      // Apply the default layout only when no saved layout exists; a restored
      // imgui.ini layout leaves the root dock node already split.
      const ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockId);
      if (node == nullptr || node->IsLeafNode()) {
        buildDefaultDockLayout(dockId, panels);
      }
    }
  }
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
  uiState.uiScale = guiScale;
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
      dartScene.world.get(),
      &lifecycle,
      selectedLabel,
      selectionController.selectedPoint(),
      selectionController.selectedNormal(),
      dartScene.world->getTime(),
      dartScene.world->getLastCollisionResult().getNumContacts(),
      {cameraBasis.eye, cameraController.camera.target, cameraBasis.up},
      {&headlightsEnabled},
      {&dartScene.renderSettings},
      uiState};
  const bool debugOptionsChanged = renderBuiltInStatusPanel(
      sceneName(exampleScene),
      panelContext.simulationTime,
      panelContext.contactCount,
      panelContext.selectedLabel,
      !dartScene.ikHandles.empty(),
      orbitLight,
      debugOverlays.staticOptions,
      debugOverlays.contactOptions,
      lifecycle,
      guiScale,
      dartScene.dockingEnabled);
  if (debugOptionsChanged) {
    refreshStaticDebugOverlay(
        engine, scene, debugMaterial, *dartScene.world, debugOverlays);
    refreshContactDebugOverlay(
        engine,
        scene,
        debugMaterial,
        dartScene.world->getLastCollisionResult(),
        debugOverlays);
  }
  renderApplicationPanels(
      panels, panelContext, guiScale, dartScene.dockingEnabled);
  if (dartScene.debugLabels) {
    renderDebugLabels(
        dartScene.debugLabels(), cameraController.camera, viewport, guiScale);
  }
  ImGui::Render();
  updateImGuiOverlay(
      engine,
      imguiOverlay,
      ImGui::GetDrawData(),
      static_cast<std::uint32_t>(viewport.width),
      static_cast<std::uint32_t>(viewport.height));
  profile.uiMs += dart::gui::elapsedMs(phaseStart);
}

} // namespace dart::gui::detail
