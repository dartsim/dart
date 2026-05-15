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

#include <dart/gui/experimental/detail/filament/panel.hpp>

#include <imgui.h>

namespace dart::gui::experimental::filament {

bool renderBuiltInStatusPanel(
    const char* sceneName,
    double simulationTime,
    std::size_t contactCount,
    const std::string& selectedLabel,
    bool showIkHint,
    bool& orbitLight,
    DebugDrawOptions& staticDebugOptions,
    DebugDrawOptions& contactDebugOptions,
    ViewerLifecycleState& lifecycle,
    double guiScale)
{
  ImGui::SetNextWindowPos(
      {20.0f * static_cast<float>(guiScale),
       20.0f * static_cast<float>(guiScale)},
      ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(0.72f);
  ImGui::Begin(
      "DART",
      nullptr,
      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
  ImGui::PushTextWrapPos(
      ImGui::GetCursorPosX() + 300.0f * static_cast<float>(guiScale));
  ImGui::TextWrapped(
      "DART scene viewer: inspect renderables, shadows, and debug overlays.");
  ImGui::TextWrapped(
      "Mouse: left orbit, right/middle pan, wheel zoom, click select.");
  ImGui::TextWrapped(
      "Keys: Space pause, N step, arrows/Pg or Ctrl-left drag selected, "
      "Esc exit.");
  if (showIkHint) {
    ImGui::TextWrapped(
        "IK: press 1-4 or click a colored target, then move it.");
  }
  ImGui::PopTextWrapPos();
  ImGui::Separator();
  ImGui::Text("scene: %s", sceneName);
  ImGui::Text("time: %.3f", simulationTime);
  ImGui::Text("contacts: %zu", contactCount);
  ImGui::PushTextWrapPos(
      ImGui::GetCursorPosX() + 300.0f * static_cast<float>(guiScale));
  ImGui::Text("selected: %s", selectedLabel.c_str());
  ImGui::PopTextWrapPos();
  if (ImGui::Button(lifecycle.paused ? "Resume" : "Pause")) {
    togglePaused(lifecycle);
  }
  ImGui::SameLine();
  if (ImGui::Button("Step")) {
    requestSingleStep(lifecycle);
  }
  ImGui::SameLine();
  ImGui::Checkbox("Orbit light", &orbitLight);

  bool debugOptionsChanged = false;
  debugOptionsChanged |= ImGui::Checkbox("Grid", &staticDebugOptions.drawGrid);
  ImGui::SameLine();
  debugOptionsChanged
      |= ImGui::Checkbox("World", &staticDebugOptions.drawWorldFrame);
  ImGui::SameLine();
  debugOptionsChanged
      |= ImGui::Checkbox("Body", &staticDebugOptions.drawBodyFrames);
  debugOptionsChanged
      |= ImGui::Checkbox("COM", &staticDebugOptions.drawCentersOfMass);
  ImGui::SameLine();
  debugOptionsChanged
      |= ImGui::Checkbox("Inertia", &staticDebugOptions.drawInertiaBoxes);
  ImGui::SameLine();
  debugOptionsChanged |= ImGui::Checkbox(
      "Collision", &staticDebugOptions.drawCollisionShapeBounds);
  ImGui::SameLine();
  debugOptionsChanged
      |= ImGui::Checkbox("Contacts", &contactDebugOptions.drawContacts);
  ImGui::SameLine();
  debugOptionsChanged
      |= ImGui::Checkbox("Support", &staticDebugOptions.drawSupportPolygons);
  debugOptionsChanged
      |= ImGui::Checkbox("Normals", &contactDebugOptions.drawContactNormals);
  ImGui::SameLine();
  debugOptionsChanged
      |= ImGui::Checkbox("Forces", &contactDebugOptions.drawContactForces);
  ImGui::End();

  return debugOptionsChanged;
}

} // namespace dart::gui::experimental::filament
