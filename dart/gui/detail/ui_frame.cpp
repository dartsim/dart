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

#include <imgui.h>

#include <array>
#include <string>
#include <vector>

#include <cstdint>

namespace dart::gui::detail {

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
  auto phaseStart = dart::gui::ProfileAccumulator::Clock::now();
  updateImGuiMouseInput(window, imguiIo, viewport.width, viewport.height);
  ImGui::NewFrame();
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
      guiScale);
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
  renderApplicationPanels(panels, panelContext, guiScale);
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
