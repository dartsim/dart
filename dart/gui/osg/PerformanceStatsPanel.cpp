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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *   OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/osg/PerformanceStatsPanel.hpp"

#include "dart/gui/osg/IncludeImGui.hpp"

#include <algorithm>

#include <cstddef>
#include <cstdio>

namespace dart {
namespace gui {
namespace osg {
namespace {

void drawTextValueRow(const char* label, const char* value)
{
  ImGui::TextUnformatted(label);
  ImGui::NextColumn();
  ImGui::TextUnformatted(value);
  ImGui::NextColumn();
}

template <typename... Args>
void drawValueRow(const char* label, const char* format, Args... args)
{
  char buffer[128];
  std::snprintf(buffer, sizeof(buffer), format, args...);
  drawTextValueRow(label, buffer);
}

float graphMax(const std::vector<float>& values, double currentMax)
{
  float maximum = static_cast<float>(std::max(1.0, currentMax));
  for (const float value : values)
    maximum = std::max(maximum, value);

  return std::max(1.0f, maximum * 1.15f);
}

} // namespace

//==============================================================================
PerformanceStatsPanel::PerformanceStatsPanel(std::size_t historyCapacity)
  : mHistoryCapacity(std::max<std::size_t>(1u, historyCapacity)),
    mLastMeasuredSteps(0u)
{
  mStepMsHistory.reserve(mHistoryCapacity);
}

//==============================================================================
void PerformanceStatsPanel::reset()
{
  mLastMeasuredSteps = 0u;
  mStepMsHistory.clear();
}

//==============================================================================
void PerformanceStatsPanel::setHistoryCapacity(std::size_t historyCapacity)
{
  mHistoryCapacity = std::max<std::size_t>(1u, historyCapacity);
  if (mStepMsHistory.size() > mHistoryCapacity) {
    mStepMsHistory.erase(
        mStepMsHistory.begin(),
        mStepMsHistory.end() - static_cast<std::ptrdiff_t>(mHistoryCapacity));
  }
  mStepMsHistory.reserve(mHistoryCapacity);
}

//==============================================================================
std::size_t PerformanceStatsPanel::getHistoryCapacity() const
{
  return mHistoryCapacity;
}

//==============================================================================
void PerformanceStatsPanel::render(
    const PerformanceStatsData& data,
    const PerformanceStatsPanelOptions& options)
{
  recordSample(data);

  const double simStepsPerSecond = data.movingAverageStepMs > 0.0
                                       ? 1000.0 / data.movingAverageStepMs
                                       : 0.0;
  const double physicsRealTimeFactor = simStepsPerSecond * data.timeStep;

  ImGui::Columns(2, "dart_performance_stats", false);
  ImGui::SetColumnWidth(0, ImGui::GetFontSize() * 8.5f);
  drawValueRow("Frame", "%d", data.frame);
  drawValueRow("Time", "%.3f s", data.simTime);
  drawTextValueRow("Scene", data.sceneName.c_str());
  drawTextValueRow("Collision", data.collisionDetectorName.c_str());
  drawValueRow("Threads", "%zu", data.simulationThreads);
  drawValueRow("Render", "%.0f FPS", data.renderFps);
  drawValueRow("Target RTF", "%.2f", data.targetRealTimeFactor);
  drawValueRow("Physics avg", "%.3f ms", data.movingAverageStepMs);
  drawValueRow("Physics last", "%.3f ms", data.lastStepMs);
  drawValueRow("Physics rate", "%.1f steps/s", simStepsPerSecond);
  drawValueRow("Physics RTF", "%.2f", physicsRealTimeFactor);
  drawValueRow("Samples", "%zu", data.measuredSteps);
  drawValueRow("Step min", "%.3f ms", data.minStepMs);
  drawValueRow("Step max", "%.3f ms", data.maxStepMs);
  drawValueRow("Contacts", "%zu", data.contacts);
  if (options.showSceneCounts) {
    drawValueRow("Soft bodies", "%zu", data.softBodies);
    drawValueRow("Point masses", "%zu", data.pointMasses);
    drawValueRow("Skeletons", "%zu", data.skeletons);
    drawValueRow("Body nodes", "%zu", data.bodyNodes);
  }
  ImGui::Columns(1);

  if (options.showGraph && !mStepMsHistory.empty()) {
    const float height = std::max(24.0f, options.graphHeight);
    ImGui::PlotLines(
        "##dart_step_ms_history",
        mStepMsHistory.data(),
        static_cast<int>(mStepMsHistory.size()),
        0,
        nullptr,
        0.0f,
        graphMax(mStepMsHistory, data.maxStepMs),
        ImVec2(ImGui::GetContentRegionAvail().x, height));
  }
}

//==============================================================================
void PerformanceStatsPanel::recordSample(const PerformanceStatsData& data)
{
  if (data.measuredSteps < mLastMeasuredSteps)
    reset();

  if (data.measuredSteps == mLastMeasuredSteps
      || data.movingAverageStepMs <= 0.0)
    return;

  mLastMeasuredSteps = data.measuredSteps;
  mStepMsHistory.push_back(static_cast<float>(data.movingAverageStepMs));
  if (mStepMsHistory.size() > mHistoryCapacity)
    mStepMsHistory.erase(mStepMsHistory.begin());
}

} // namespace osg
} // namespace gui
} // namespace dart
