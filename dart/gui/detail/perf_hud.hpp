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

#ifndef DART_GUI_DETAIL_PERF_HUD_HPP_
#define DART_GUI_DETAIL_PERF_HUD_HPP_

#include <array>

#include <cstddef>

namespace dart::gui {

struct ProfileAccumulator;

} // namespace dart::gui

namespace dart::gui::detail {

// Rolling state for the live performance overlay. Persists across frames so the
// HUD can show instantaneous per-frame values (derived as deltas of the
// run-long ProfileAccumulator) and a short history plot.
struct PerfHudState
{
  static constexpr int kHistory = 120;

  bool hasPrev = false;
  std::size_t prevFrames = 0;
  double prevFrameMs = 0.0;
  double prevInputMs = 0.0;
  double prevViewportMs = 0.0;
  double prevSimulationMs = 0.0;
  double prevExtractionMs = 0.0;
  double prevSyncMs = 0.0;
  double prevUiMs = 0.0;
  double prevBeginFrameMs = 0.0;
  double prevRenderMs = 0.0;
  double prevSimulatedMs = 0.0;

  // Smoothed (exponential moving average) display values. Raw per-frame numbers
  // change too fast to read, so the headline figures use these while the graphs
  // show the raw history.
  static constexpr int kPhaseCount = 8;
  double emaCpuMs = 0.0;
  double emaGpuMs = 0.0;
  double emaRealTimeFactor = 0.0;
  bool hasRealTimeFactor = false;
  std::array<double, kPhaseCount> emaPhaseMs{};

  std::array<float, kHistory> cpuHistory{};
  std::array<float, kHistory> gpuHistory{};
  int historyOffset = 0;
  int historyCount = 0;
};

// Draws the performance overlay as an ImGui window. Must be called between
// ImGui::NewFrame() and ImGui::Render(). Reads per-frame values from `profile`
// (the single source of truth) and shows the active graphics backend.
void drawPerfHud(
    PerfHudState& state,
    const dart::gui::ProfileAccumulator& profile,
    const char* backendName);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_PERF_HUD_HPP_
