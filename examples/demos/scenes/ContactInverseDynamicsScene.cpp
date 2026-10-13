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

// Kinematic squat replay and contact-consistent inverse dynamics live in the
// GUI-free model shared with the Python catalog. The per-frame work runs in
// preRefresh, including while paused; the panel only presents controls/status.

#include "GuiBackend.hpp"
#include "Scenes.hpp"
#include "ZUp.hpp"
#include "demo_models/ContactInverseDynamicsModel.hpp"

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>

#include <cmath>

namespace dart_demos {

//==============================================================================
DemoScene makeContactInverseDynamicsScene()
{
  DemoScene scene;
  scene.id = "contact_inverse_dynamics";
  scene.title = "Contact Inverse Dynamics";
  scene.category = "Control & IK";
  scene.summary
      = "Kinematic squat replay solved for contact-consistent joint torques.";

  scene.factory = [] {
    auto world = dart::utils::SkelParser::readWorld(
        "dart://sample/skel/fullbody1.skel");
    if (!world)
      throw std::runtime_error(
          "failed to load dart://sample/skel/fullbody1.skel");
    reorientWorldToZUp(world);

    auto state = std::make_shared<ContactInverseDynamicsModel>(world);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        GuiVector3(3.0, -3.0, 1.5),
        GuiVector3(0.0, 0.0, 0.0),
        GuiVector3(0.0, 0.0, 1.0)};

    setup.preRefresh = [state] {
      state->updateFrame();
    };
    setup.renderPanel = [state] {
      if (ImGui::CollapsingHeader("Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Play", &state->playing);

        // Every slider commits through a clamped, finite local copy so a NaN
        // typed via Ctrl+click (which AlwaysClamp does not reject) can never
        // reach state -- a NaN playbackSpeed permanently poisons state.time
        // and every downstream spline/solver evaluation, and a NaN squatDepth
        // would poison all 60 keyframes on the next regeneration.
        auto playbackSpeed = state->playbackSpeed;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::SliderFloat(
                "Playback Speed",
                &playbackSpeed,
                0.1f,
                2.0f,
                "%.2f",
                ImGuiSliderFlags_AlwaysClamp)
            && std::isfinite(playbackSpeed))
          state->playbackSpeed = std::clamp(playbackSpeed, 0.1f, 2.0f);

        auto squatDepth = state->squatDepth;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::SliderFloat(
                "Squat Depth (m)",
                &squatDepth,
                0.0f,
                0.25f,
                "%.3f",
                ImGuiSliderFlags_AlwaysClamp)
            && std::isfinite(squatDepth))
          state->squatDepth = std::clamp(squatDepth, 0.0f, 0.25f);
        // Regenerate on drag release (IsItemDeactivatedAfterEdit alone -- the
        // slider does not report a value change on the release frame, so an
        // additional edited-this-frame conjunct would make mouse drags a
        // no-op), deferred to the next frame via regenerateRequested.
        if (ImGui::IsItemDeactivatedAfterEdit())
          state->regenerateRequested = true;
        if (ImGui::Button("Regenerate Keyframes"))
          state->regenerateRequested = true;

        auto frictionCoeff = state->frictionCoeff;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::SliderFloat(
                "Friction Coefficient",
                &frictionCoeff,
                0.0f,
                1.5f,
                "%.2f",
                ImGuiSliderFlags_AlwaysClamp)
            && std::isfinite(frictionCoeff))
          state->frictionCoeff = std::clamp(frictionCoeff, 0.0f, 1.5f);

        auto forceScale = state->forceScale;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::SliderFloat(
                "Force Arrow Scale",
                &forceScale,
                0.0001f,
                0.01f,
                "%.4f",
                ImGuiSliderFlags_AlwaysClamp)
            && std::isfinite(forceScale))
          state->forceScale = std::clamp(forceScale, 0.0001f, 0.01f);

        const Eigen::Vector3d gravity = state->world->getGravity();
        ImGui::Text(
            "Gravity: (%.2f, %.2f, %.2f)", gravity[0], gravity[1], gravity[2]);
      }

      if (ImGui::CollapsingHeader("Status", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (state->feasible)
          ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "FEASIBLE");
        else
          ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "INFEASIBLE");

        ImGui::Text("Residual Norm: %.4f", state->residualNorm);

        const double bodyMass = state->biped->getMass();
        const double bodyWeight = bodyMass * state->world->getGravity().norm();
        ImGui::Text("Total Vertical Force: %.2f N", state->totalVerticalForce);
        ImGui::Text("Body Weight: %.2f N", bodyWeight);
        ImGui::Text(
            "Ratio: %.2f",
            bodyWeight > 1e-6 ? state->totalVerticalForce / bodyWeight : 0.0);

        ImGui::Separator();
        ImGui::Text("Contact Forces:");
        for (std::size_t i = 0; i < state->contactForces.size(); ++i)
          ImGui::Text(
              "  Contact %zu: %.2f N", i, state->contactForces[i].norm());
      }

      if (ImGui::CollapsingHeader("Plots", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (std::size_t i = 0; i < state->torqueNames.size(); ++i) {
          ImGui::PlotLines(
              state->torqueNames[i].c_str(),
              state->torquePlots[i].data(),
              static_cast<int>(state->torquePlots[i].size()),
              0,
              nullptr,
              -200.0f,
              200.0f,
              ImVec2(0, ImGui::GetFontSize() * 5.0f));
        }
        ImGui::PlotLines(
            "Total Vertical GRF (N)",
            state->plotSamples.data(),
            static_cast<int>(state->plotSamples.size()),
            0,
            nullptr,
            0.0f,
            1000.0f,
            ImVec2(0, ImGui::GetFontSize() * 5.0f));
      }
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
