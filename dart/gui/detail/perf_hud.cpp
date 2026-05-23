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

#include <dart/gui/detail/perf_hud.hpp>
#include <dart/gui/profile.hpp>

#include <imgui.h>

#include <algorithm>

#include <cfloat>

namespace dart::gui::detail {
namespace {

// Green under 60 FPS budget, amber under 30 FPS, red beyond.
ImVec4 budgetColor(double ms)
{
  if (ms <= 0.0) {
    return ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
  }
  if (ms <= 16.7) {
    return ImVec4(0.40f, 0.90f, 0.45f, 1.0f);
  }
  if (ms <= 33.4) {
    return ImVec4(0.95f, 0.85f, 0.30f, 1.0f);
  }
  return ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
}

void phaseRow(const char* name, double ms, double frameMs)
{
  const double fraction = frameMs > 0.0 ? ms / frameMs : 0.0;
  ImGui::Text("%-9s %6.2f ms", name, ms);
  ImGui::SameLine();
  ImGui::ProgressBar(
      static_cast<float>(std::clamp(fraction, 0.0, 1.0)),
      ImVec2(-1.0f, 0.0f),
      "");
}

} // namespace

void drawPerfHud(
    PerfHudState& state,
    const dart::gui::ProfileAccumulator& profile,
    const char* backendName)
{
  const double deltaFrames = static_cast<double>(profile.frames)
                             - static_cast<double>(state.prevFrames);
  const auto perFrame = [&](double current, double previous) {
    return (state.hasPrev && deltaFrames > 0.0)
               ? (current - previous) / deltaFrames
               : 0.0;
  };

  const double cpuFrameMs = perFrame(profile.frameMs, state.prevFrameMs);
  const double inputMs = perFrame(profile.inputMs, state.prevInputMs);
  const double viewportMs
      = perFrame(profile.viewportCameraMs, state.prevViewportMs);
  const double simulationMs
      = perFrame(profile.simulationMs, state.prevSimulationMs);
  const double extractionMs
      = perFrame(profile.extractionMs, state.prevExtractionMs);
  const double syncMs = perFrame(profile.syncMs, state.prevSyncMs);
  const double uiMs = perFrame(profile.uiMs, state.prevUiMs);
  const double beginMs = perFrame(profile.beginFrameMs, state.prevBeginFrameMs);
  const double renderMs = perFrame(profile.renderMs, state.prevRenderMs);
  const double gpuMs = profile.gpuFrameMs;

  if (state.hasPrev && deltaFrames > 0.0) {
    state.cpuHistory[static_cast<std::size_t>(state.historyOffset)]
        = static_cast<float>(cpuFrameMs);
    state.gpuHistory[static_cast<std::size_t>(state.historyOffset)]
        = static_cast<float>(gpuMs);
    state.historyOffset = (state.historyOffset + 1) % PerfHudState::kHistory;
    if (state.historyCount < PerfHudState::kHistory) {
      ++state.historyCount;
    }
  }

  state.prevFrames = profile.frames;
  state.prevFrameMs = profile.frameMs;
  state.prevInputMs = profile.inputMs;
  state.prevViewportMs = profile.viewportCameraMs;
  state.prevSimulationMs = profile.simulationMs;
  state.prevExtractionMs = profile.extractionMs;
  state.prevSyncMs = profile.syncMs;
  state.prevUiMs = profile.uiMs;
  state.prevBeginFrameMs = profile.beginFrameMs;
  state.prevRenderMs = profile.renderMs;
  state.hasPrev = true;

  ImGui::SetNextWindowBgAlpha(0.65f);
  if (!ImGui::Begin(
          "Performance",
          nullptr,
          ImGuiWindowFlags_NoNav | ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::End();
    return;
  }

  const double fps = cpuFrameMs > 0.0 ? 1000.0 / cpuFrameMs : 0.0;
  ImGui::TextColored(
      budgetColor(cpuFrameMs), "CPU %6.2f ms  (%5.1f FPS)", cpuFrameMs, fps);
  if (gpuMs > 0.0) {
    ImGui::TextColored(budgetColor(gpuMs), "GPU %6.2f ms", gpuMs);
  } else {
    ImGui::TextDisabled("GPU    n/a (no timer queries)");
  }
  ImGui::Text("backend: %s", backendName != nullptr ? backendName : "?");

  if (state.historyCount > 1) {
    const int plotOffset
        = state.historyCount < PerfHudState::kHistory ? 0 : state.historyOffset;
    ImGui::Separator();
    ImGui::PlotLines(
        "CPU",
        state.cpuHistory.data(),
        state.historyCount,
        plotOffset,
        "frame ms",
        0.0f,
        FLT_MAX,
        ImVec2(220.0f, 50.0f));
    ImGui::PlotLines(
        "GPU",
        state.gpuHistory.data(),
        state.historyCount,
        plotOffset,
        nullptr,
        0.0f,
        FLT_MAX,
        ImVec2(220.0f, 40.0f));
  }

  ImGui::Separator();
  phaseRow("input", inputMs, cpuFrameMs);
  phaseRow("viewport", viewportMs, cpuFrameMs);
  phaseRow("sim", simulationMs, cpuFrameMs);
  phaseRow("extract", extractionMs, cpuFrameMs);
  phaseRow("sync", syncMs, cpuFrameMs);
  phaseRow("ui", uiMs, cpuFrameMs);
  phaseRow("begin", beginMs, cpuFrameMs);
  phaseRow("render", renderMs, cpuFrameMs);

  ImGui::Separator();
  ImGui::Text(
      "frames %zu  rendered %zu  skipped %zu",
      profile.frames,
      profile.renderedFrames,
      profile.skippedFrames);
  ImGui::End();
}

} // namespace dart::gui::detail
