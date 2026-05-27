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

#pragma once

#include <dartsim_engine/event_bus.hpp>
#include <dartsim_engine/scene_object.hpp>
#include <dartsim_engine/sim_engine.hpp>

#include <span>
#include <string>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dartsim::ui {

inline constexpr std::size_t kDefaultWatchHistoryCapacity = 240;

enum class WatchValueKind
{
  TranslationX,
  TranslationY,
  TranslationZ,
  Mass,
  JointPosition,
  SensorRange,
  SensorFieldOfView,
  SensorUpdateRate,
  SimulationTime,
  FrameCount,
};

struct WatchSeries
{
  ObjectId object = kNoObject;
  std::uint64_t projectGeneration = 0;
  WatchValueKind kind = WatchValueKind::SimulationTime;
  std::string label;
  std::vector<double> samples;
};

struct WatchTarget
{
  ObjectId id = kNoObject;
  std::uint64_t projectGeneration = 0;
};

[[nodiscard]] std::vector<WatchValueKind> defaultWatchChartSignals();

struct WatchState
{
  std::vector<WatchTarget> targets;
  std::vector<WatchSeries> series;
  std::vector<WatchValueKind> chartSignals = defaultWatchChartSignals();
  std::size_t historyCapacity = kDefaultWatchHistoryCapacity;
};

struct WatchValue
{
  WatchValueKind kind = WatchValueKind::TranslationX;
  std::string label;
  double value = 0.0;
};

struct WatchRow
{
  ObjectId id = kNoObject;
  std::string name;
  std::string type;
  bool exists = false;
  bool selected = false;
  std::vector<WatchValue> values;
};

struct WatchChartSeries
{
  ObjectId object = kNoObject;
  std::uint64_t projectGeneration = 0;
  WatchValueKind kind = WatchValueKind::SimulationTime;
  std::string label;
  std::span<const double> samples;
};

struct WatchSignalOption
{
  WatchValueKind kind = WatchValueKind::SimulationTime;
  std::string label;
  bool enabled = false;
};

struct WatchPresetOption
{
  std::string name;
  std::size_t targetCount = 0;
  std::size_t missingTargetCount = 0;
  std::size_t signalCount = 0;
  std::size_t ignoredSignalCount = 0;
};

struct WatchStatus
{
  std::string summary;
  std::vector<WatchSignalOption> signalOptions;
  std::vector<WatchPresetOption> presetOptions;
  std::vector<WatchRow> rows;
  std::vector<WatchChartSeries> series;
};

struct WatchActionResult
{
  bool ok = false;
  std::string message;
};

/// Add every currently selected object to the watch list.
WatchActionResult watchSelectedObjects(
    WatchState& state, const SimEngine& engine);

/// Add a single object to the watch list.
WatchActionResult watchObject(
    WatchState& state, const SimEngine& engine, ObjectId id);

/// Remove one object from the watch list and any object-owned chart series.
WatchActionResult unwatchObject(WatchState& state, ObjectId id);

/// Clear all watch rows and chart samples.
WatchActionResult clearWatch(WatchState& state);

/// Enable or disable one plotted signal without mutating the scene.
WatchActionResult setWatchChartSignalEnabled(
    WatchState& state, WatchValueKind kind, bool enabled);

/// Save the current watch targets and chart signals as a project preset.
WatchActionResult saveWatchPreset(
    WatchState& state, SimEngine& engine, std::string name);

/// Apply a saved project preset to the session-local Watch panel state.
WatchActionResult applyWatchPreset(
    WatchState& state, const SimEngine& engine, std::string_view name);

/// Delete a saved project preset.
WatchActionResult deleteWatchPreset(SimEngine& engine, std::string_view name);

/// Keep watch state scoped to the current project when engine events replace
/// the scene.
void handleWatchEvent(WatchState& state, const Event& event);

/// Capture one sample for the global simulation series and watched objects.
void recordWatchSample(WatchState& state, const SimEngine& engine);

/// Build the panel-facing watch table and chart series.
[[nodiscard]] WatchStatus buildWatchStatus(
    const WatchState& state, const SimEngine& engine);

} // namespace dartsim::ui
