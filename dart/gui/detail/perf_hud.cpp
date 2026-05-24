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
#include <array>
#include <string_view>

namespace dart::gui::detail {
namespace {

// Frame-time budget for 60 FPS, and the fixed graph scale (2x budget) so the
// plots do not auto-rescale and jump around every frame.
constexpr double kTargetFrameMs = 1000.0 / 60.0;
constexpr float kPlotScaleMaxMs = static_cast<float>(kTargetFrameMs * 2.0);
constexpr double kEmaAlpha = 0.1;

// Green within the 60 FPS budget, amber within 30 FPS, red beyond.
ImVec4 budgetColor(double ms)
{
  if (ms <= 0.0) {
    return ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
  }
  if (ms <= kTargetFrameMs) {
    return ImVec4(0.40f, 0.90f, 0.45f, 1.0f);
  }
  if (ms <= kTargetFrameMs * 2.0) {
    return ImVec4(0.95f, 0.85f, 0.30f, 1.0f);
  }
  return ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
}

// Green near real time, amber when noticeably slow, red when far behind.
ImVec4 realTimeColor(double factor)
{
  if (factor >= 0.95) {
    return ImVec4(0.40f, 0.90f, 0.45f, 1.0f);
  }
  if (factor >= 0.5) {
    return ImVec4(0.95f, 0.85f, 0.30f, 1.0f);
  }
  return ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
}

// opengl/vulkan/metal/webgpu render on the GPU; noop performs no rendering.
const char* backendDescription(const char* name)
{
  if (name == nullptr) {
    return "?";
  }
  const std::string_view view(name);
  if (view == "opengl") {
    return "opengl (GPU)";
  }
  if (view == "vulkan") {
    return "vulkan (GPU)";
  }
  if (view == "metal") {
    return "metal (GPU)";
  }
  if (view == "webgpu") {
    return "webgpu (GPU)";
  }
  if (view == "noop") {
    return "noop (no render)";
  }
  return name;
}

void updateEma(double& ema, double value, double alpha)
{
  ema += alpha * (value - ema);
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
  const double gpuMs = profile.gpuFrameMs;
  const std::array<double, PerfHudState::kPhaseCount> phaseMs{
      perFrame(profile.inputMs, state.prevInputMs),
      perFrame(profile.viewportCameraMs, state.prevViewportMs),
      perFrame(profile.simulationMs, state.prevSimulationMs),
      perFrame(profile.extractionMs, state.prevExtractionMs),
      perFrame(profile.syncMs, state.prevSyncMs),
      perFrame(profile.uiMs, state.prevUiMs),
      perFrame(profile.beginFrameMs, state.prevBeginFrameMs),
      perFrame(profile.renderMs, state.prevRenderMs)};
  const double simulatedDeltaMs
      = state.hasPrev ? profile.simulatedMs - state.prevSimulatedMs : 0.0;
  const double wallDeltaMs
      = state.hasPrev ? profile.frameMs - state.prevFrameMs : 0.0;

  if (state.hasPrev && deltaFrames > 0.0) {
    const bool seed = state.historyCount == 0;
    if (seed) {
      state.emaCpuMs = cpuFrameMs;
      state.emaPhaseMs = phaseMs;
    } else {
      updateEma(state.emaCpuMs, cpuFrameMs, kEmaAlpha);
      for (int i = 0; i < PerfHudState::kPhaseCount; ++i) {
        updateEma(
            state.emaPhaseMs[static_cast<std::size_t>(i)],
            phaseMs[static_cast<std::size_t>(i)],
            kEmaAlpha);
      }
    }
    if (gpuMs > 0.0) {
      state.emaGpuMs = state.emaGpuMs <= 0.0 ? gpuMs : state.emaGpuMs;
      updateEma(state.emaGpuMs, gpuMs, kEmaAlpha);
    }
    // Real-time factor only when the simulation actually advanced this frame
    // (otherwise the readout would collapse toward 0% while paused).
    if (simulatedDeltaMs > 0.0 && wallDeltaMs > 0.0) {
      const double instant = simulatedDeltaMs / wallDeltaMs;
      if (!state.hasRealTimeFactor) {
        state.emaRealTimeFactor = instant;
        state.hasRealTimeFactor = true;
      } else {
        updateEma(state.emaRealTimeFactor, instant, kEmaAlpha);
      }
    }

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
  state.prevSimulatedMs = profile.simulatedMs;
  state.hasPrev = true;

  ImGui::SetNextWindowBgAlpha(0.65f);
  // Pin to the top-right corner on first use and never persist the position, so
  // the HUD always appears on-screen regardless of a stale imgui.ini or a
  // different window size between runs.
  if (const ImGuiViewport* viewport = ImGui::GetMainViewport()) {
    const ImVec2 corner(
        viewport->WorkPos.x + viewport->WorkSize.x - 10.0f,
        viewport->WorkPos.y + 10.0f);
    ImGui::SetNextWindowPos(corner, ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
  }
  if (!ImGui::Begin(
          "Performance",
          nullptr,
          ImGuiWindowFlags_NoNav | ImGuiWindowFlags_AlwaysAutoResize
              | ImGuiWindowFlags_NoSavedSettings)) {
    ImGui::End();
    return;
  }

  const double cpu = state.emaCpuMs;
  const double fps = cpu > 0.0 ? 1000.0 / cpu : 0.0;
  ImGui::TextColored(budgetColor(cpu), "CPU %6.2f ms  %5.1f FPS", cpu, fps);
  ImGui::SameLine();
  ImGui::TextDisabled("/ %.1f (60)", kTargetFrameMs);
  if (state.emaGpuMs > 0.0) {
    ImGui::TextColored(
        budgetColor(state.emaGpuMs), "GPU %6.2f ms", state.emaGpuMs);
    ImGui::SameLine();
    ImGui::TextDisabled("/ %.1f", kTargetFrameMs);
  } else {
    ImGui::TextDisabled("GPU    n/a (no timer queries)");
  }
  if (state.hasRealTimeFactor) {
    ImGui::TextColored(
        realTimeColor(state.emaRealTimeFactor),
        "real-time %5.1f%%",
        state.emaRealTimeFactor * 100.0);
    ImGui::SameLine();
    ImGui::TextDisabled("/ 100 (sim vs wall)");
  } else {
    ImGui::TextDisabled("real-time   n/a (paused)");
  }
  ImGui::Text("render: %s", backendDescription(backendName));

  if (state.historyCount > 1) {
    const int plotOffset
        = state.historyCount < PerfHudState::kHistory ? 0 : state.historyOffset;
    ImGui::Separator();
    ImGui::PlotLines(
        "CPU",
        state.cpuHistory.data(),
        state.historyCount,
        plotOffset,
        "ms (0..33)",
        0.0f,
        kPlotScaleMaxMs,
        ImVec2(220.0f, 50.0f));
    ImGui::PlotLines(
        "GPU",
        state.gpuHistory.data(),
        state.historyCount,
        plotOffset,
        nullptr,
        0.0f,
        kPlotScaleMaxMs,
        ImVec2(220.0f, 40.0f));
  }

  ImGui::Separator();
  static constexpr std::array<const char*, PerfHudState::kPhaseCount>
      kPhaseNames{
          "input",
          "viewport",
          "sim",
          "extract",
          "sync",
          "ui",
          "begin",
          "render"};
  for (int i = 0; i < PerfHudState::kPhaseCount; ++i) {
    phaseRow(
        kPhaseNames[static_cast<std::size_t>(i)],
        state.emaPhaseMs[static_cast<std::size_t>(i)],
        cpu);
  }

  ImGui::Separator();
  ImGui::Text(
      "frames %zu  rendered %zu  skipped %zu",
      profile.frames,
      profile.renderedFrames,
      profile.skippedFrames);
  ImGui::End();
}

} // namespace dart::gui::detail
