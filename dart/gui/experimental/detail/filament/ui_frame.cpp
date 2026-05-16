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

#include <dart/gui/experimental/detail/filament/debug_overlay.hpp>
#include <dart/gui/experimental/detail/filament/frame_viewport.hpp>
#include <dart/gui/experimental/detail/filament/imgui_overlay.hpp>
#include <dart/gui/experimental/detail/filament/input.hpp>
#include <dart/gui/experimental/detail/filament/panel.hpp>
#include <dart/gui/experimental/detail/filament/scenes.hpp>
#include <dart/gui/experimental/detail/filament/selection.hpp>
#include <dart/gui/experimental/detail/filament/ui_frame.hpp>
#include <dart/gui/profile.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/collision_result.hpp>

#include <imgui.h>

#include <string>
#include <vector>

#include <cstdint>

namespace dart::gui::filament {

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
    const SelectionController& selectionController,
    bool& orbitLight,
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
  dart::gui::PanelContext panelContext{
      dartScene.world.get(),
      &lifecycle,
      selectedLabel,
      dartScene.world->getTime(),
      dartScene.world->getLastCollisionResult().getNumContacts()};
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

} // namespace dart::gui::filament
