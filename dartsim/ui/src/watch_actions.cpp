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

#include <Eigen/Geometry>
#include <dartsim_engine/commands.hpp>
#include <dartsim_ui/outliner_actions.hpp>
#include <dartsim_ui/watch_actions.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cstddef>

namespace dartsim::ui {

namespace {

bool containsTarget(
    const WatchState& state, const SimEngine& engine, ObjectId id)
{
  return std::find_if(
             state.targets.begin(),
             state.targets.end(),
             [&engine, id](const WatchTarget& target) {
               return target.id == id
                      && target.projectGeneration == engine.projectGeneration();
             })
         != state.targets.end();
}

const std::vector<WatchValueKind>& allWatchChartSignals()
{
  static const std::vector<WatchValueKind> kSignals{
      WatchValueKind::SimulationTime,
      WatchValueKind::FrameCount,
      WatchValueKind::TranslationX,
      WatchValueKind::TranslationY,
      WatchValueKind::TranslationZ,
      WatchValueKind::Mass,
      WatchValueKind::JointPosition,
      WatchValueKind::SensorRange,
      WatchValueKind::SensorFieldOfView,
      WatchValueKind::SensorUpdateRate,
  };
  return kSignals;
}

bool isChartSignalEnabled(const WatchState& state, WatchValueKind kind)
{
  return std::find(state.chartSignals.begin(), state.chartSignals.end(), kind)
         != state.chartSignals.end();
}

bool isChartSignalKind(WatchValueKind kind)
{
  const std::vector<WatchValueKind>& signals = allWatchChartSignals();
  return std::find(signals.begin(), signals.end(), kind) != signals.end();
}

std::string signalKey(WatchValueKind kind)
{
  switch (kind) {
    case WatchValueKind::TranslationX:
      return "translation_x";
    case WatchValueKind::TranslationY:
      return "translation_y";
    case WatchValueKind::TranslationZ:
      return "translation_z";
    case WatchValueKind::Mass:
      return "mass";
    case WatchValueKind::JointPosition:
      return "joint_position";
    case WatchValueKind::SensorRange:
      return "sensor_range";
    case WatchValueKind::SensorFieldOfView:
      return "sensor_fov";
    case WatchValueKind::SensorUpdateRate:
      return "sensor_rate";
    case WatchValueKind::SimulationTime:
      return "simulation_time";
    case WatchValueKind::FrameCount:
      return "frame_count";
  }
  return "value";
}

std::optional<WatchValueKind> signalFromKey(std::string_view key)
{
  for (const WatchValueKind kind : allWatchChartSignals()) {
    if (signalKey(kind) == key) {
      return kind;
    }
  }
  return std::nullopt;
}

std::string valueLabel(WatchValueKind kind)
{
  switch (kind) {
    case WatchValueKind::TranslationX:
      return "x";
    case WatchValueKind::TranslationY:
      return "y";
    case WatchValueKind::TranslationZ:
      return "z";
    case WatchValueKind::Mass:
      return "mass";
    case WatchValueKind::JointPosition:
      return "joint position";
    case WatchValueKind::SensorRange:
      return "sensor range";
    case WatchValueKind::SensorFieldOfView:
      return "sensor FOV";
    case WatchValueKind::SensorUpdateRate:
      return "sensor rate";
    case WatchValueKind::SimulationTime:
      return "Simulation time";
    case WatchValueKind::FrameCount:
      return "Frame";
  }
  return "value";
}

std::string trim(std::string name)
{
  const std::string whitespace = " \t\r\n";
  const std::size_t first = name.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return {};
  }
  const std::size_t last = name.find_last_not_of(whitespace);
  return name.substr(first, last - first + 1);
}

std::vector<WatchValueKind> normalizedChartSignals(
    const std::vector<WatchValueKind>& signals)
{
  std::vector<WatchValueKind> normalized;
  for (const WatchValueKind supported : allWatchChartSignals()) {
    if (std::find(signals.begin(), signals.end(), supported) != signals.end()) {
      normalized.push_back(supported);
    }
  }
  return normalized;
}

std::vector<std::string> chartSignalKeys(
    const std::vector<WatchValueKind>& signals)
{
  std::vector<std::string> keys;
  for (const WatchValueKind kind : normalizedChartSignals(signals)) {
    keys.push_back(signalKey(kind));
  }
  return keys;
}

std::vector<WatchValueKind> chartSignalsFromKeys(
    const std::vector<std::string>& keys)
{
  std::vector<WatchValueKind> signals;
  for (const std::string& key : keys) {
    const std::optional<WatchValueKind> kind = signalFromKey(key);
    if (!kind.has_value()
        || std::find(signals.begin(), signals.end(), *kind) != signals.end()) {
      continue;
    }
    signals.push_back(*kind);
  }
  return signals;
}

const WorkspaceWatchPreset* findPreset(
    const SimEngine& engine, std::string_view name)
{
  const std::vector<WorkspaceWatchPreset>& presets
      = engine.objects().model().workspace.watchPresets;
  const auto it = std::find_if(
      presets.begin(),
      presets.end(),
      [name](const WorkspaceWatchPreset& preset) {
        return preset.name == name;
      });
  return it == presets.end() ? nullptr : &*it;
}

std::string seriesLabel(
    const SimEngine& engine, ObjectId object, WatchValueKind kind)
{
  if (object == kNoObject) {
    return valueLabel(kind);
  }

  const SceneObject* sceneObject = engine.objects().model().find(object);
  const std::string name = sceneObject == nullptr
                               ? ("missing #" + std::to_string(object))
                               : sceneObject->name;
  return name + " " + valueLabel(kind);
}

bool supportsJointPosition(const SceneObject& object)
{
  return object.type == ObjectType::Link && object.parentLink != kNoObject
         && (object.jointType == JointKind::Revolute
             || object.jointType == JointKind::Prismatic
             || object.jointType == JointKind::Screw);
}

std::optional<double> valueFor(
    const SimEngine& engine, ObjectId object, WatchValueKind kind)
{
  if (object == kNoObject) {
    switch (kind) {
      case WatchValueKind::SimulationTime:
        return engine.simulation().simTime();
      case WatchValueKind::FrameCount:
        return static_cast<double>(engine.simulation().frameCount());
      case WatchValueKind::TranslationX:
      case WatchValueKind::TranslationY:
      case WatchValueKind::TranslationZ:
      case WatchValueKind::Mass:
      case WatchValueKind::JointPosition:
      case WatchValueKind::SensorRange:
      case WatchValueKind::SensorFieldOfView:
      case WatchValueKind::SensorUpdateRate:
        return std::nullopt;
    }
  }

  const SceneObject* sceneObject = engine.objects().model().find(object);
  if (sceneObject == nullptr) {
    return std::nullopt;
  }

  switch (kind) {
    case WatchValueKind::TranslationX:
    case WatchValueKind::TranslationY:
    case WatchValueKind::TranslationZ: {
      const std::optional<Eigen::Isometry3d> transform
          = engine.objects().worldTransformOf(object);
      const Eigen::Vector3d translation
          = transform.has_value() ? transform->translation()
                                  : sceneObject->transform.translation();
      if (kind == WatchValueKind::TranslationX) {
        return translation.x();
      }
      if (kind == WatchValueKind::TranslationY) {
        return translation.y();
      }
      return translation.z();
    }
    case WatchValueKind::Mass:
      if (sceneObject->type == ObjectType::RigidBody) {
        return sceneObject->mass;
      }
      return std::nullopt;
    case WatchValueKind::JointPosition:
      if (supportsJointPosition(*sceneObject)) {
        return sceneObject->jointPosition;
      }
      return std::nullopt;
    case WatchValueKind::SensorRange:
      if (sceneObject->type == ObjectType::Sensor) {
        return sceneObject->sensor.range;
      }
      return std::nullopt;
    case WatchValueKind::SensorFieldOfView:
      if (sceneObject->type == ObjectType::Sensor) {
        return sceneObject->sensor.fieldOfView;
      }
      return std::nullopt;
    case WatchValueKind::SensorUpdateRate:
      if (sceneObject->type == ObjectType::Sensor) {
        return sceneObject->sensor.updateRate;
      }
      return std::nullopt;
    case WatchValueKind::SimulationTime:
    case WatchValueKind::FrameCount:
      return std::nullopt;
  }
  return std::nullopt;
}

std::vector<WatchValueKind> valueKindsForObject(const SceneObject& object)
{
  std::vector<WatchValueKind> kinds;
  switch (object.type) {
    case ObjectType::RigidBody:
    case ObjectType::Link:
    case ObjectType::FreeFrame:
    case ObjectType::FixedFrame:
    case ObjectType::Sensor:
      kinds.push_back(WatchValueKind::TranslationX);
      kinds.push_back(WatchValueKind::TranslationY);
      kinds.push_back(WatchValueKind::TranslationZ);
      break;
    case ObjectType::MultiBody:
    case ObjectType::Joint:
      break;
  }
  if (object.type == ObjectType::RigidBody) {
    kinds.push_back(WatchValueKind::Mass);
  }
  if (supportsJointPosition(object)) {
    kinds.push_back(WatchValueKind::JointPosition);
  }
  if (object.type == ObjectType::Sensor) {
    kinds.push_back(WatchValueKind::SensorRange);
    kinds.push_back(WatchValueKind::SensorFieldOfView);
    kinds.push_back(WatchValueKind::SensorUpdateRate);
  }
  return kinds;
}

WatchSeries& ensureSeries(
    WatchState& state,
    const SimEngine& engine,
    ObjectId object,
    std::uint64_t projectGeneration,
    WatchValueKind kind)
{
  const auto it = std::find_if(
      state.series.begin(),
      state.series.end(),
      [object, projectGeneration, kind](const WatchSeries& series) {
        return series.object == object
               && series.projectGeneration == projectGeneration
               && series.kind == kind;
      });
  if (it != state.series.end()) {
    it->label = seriesLabel(engine, object, kind);
    return *it;
  }

  WatchSeries series;
  series.object = object;
  series.projectGeneration = projectGeneration;
  series.kind = kind;
  series.label = seriesLabel(engine, object, kind);
  state.series.push_back(std::move(series));
  return state.series.back();
}

void pushSample(WatchState& state, WatchSeries& series, double value)
{
  if (state.historyCapacity == 0) {
    series.samples.clear();
    return;
  }

  series.samples.push_back(value);
  if (series.samples.size() > state.historyCapacity) {
    const std::size_t overflow = series.samples.size() - state.historyCapacity;
    series.samples.erase(
        series.samples.begin(),
        series.samples.begin() + static_cast<std::ptrdiff_t>(overflow));
  }
}

WorkspaceWatchPreset captureWatchPreset(
    const WatchState& state, const SimEngine& engine, std::string name)
{
  WorkspaceWatchPreset preset;
  preset.name = std::move(name);
  preset.chartSignals = chartSignalKeys(state.chartSignals);
  for (const WatchTarget& target : state.targets) {
    if (target.projectGeneration != engine.projectGeneration()
        || !engine.objects().model().contains(target.id)
        || std::find(preset.targets.begin(), preset.targets.end(), target.id)
               != preset.targets.end()) {
      continue;
    }
    preset.targets.push_back(target.id);
  }
  return preset;
}

} // namespace

std::vector<WatchValueKind> defaultWatchChartSignals()
{
  return {
      WatchValueKind::SimulationTime,
      WatchValueKind::FrameCount,
      WatchValueKind::TranslationZ,
      WatchValueKind::JointPosition,
  };
}

WatchActionResult watchSelectedObjects(
    WatchState& state, const SimEngine& engine)
{
  if (engine.selection().empty()) {
    return {false, "No selection to watch"};
  }

  std::size_t added = 0;
  for (const ObjectId id : engine.selection().selected()) {
    if (engine.objects().model().contains(id)
        && !containsTarget(state, engine, id)) {
      state.targets.push_back({id, engine.projectGeneration()});
      ++added;
    }
  }

  if (added == 0) {
    return {false, "Selection already watched"};
  }
  if (added == 1) {
    return {true, "Watching selection"};
  }
  return {true, "Watching " + std::to_string(added) + " selections"};
}

WatchActionResult watchObject(
    WatchState& state, const SimEngine& engine, ObjectId id)
{
  if (!engine.objects().model().contains(id)) {
    return {false, "Missing object"};
  }
  if (containsTarget(state, engine, id)) {
    return {false, "Object already watched"};
  }
  state.targets.push_back({id, engine.projectGeneration()});
  return {true, "Watching object"};
}

WatchActionResult unwatchObject(WatchState& state, ObjectId id)
{
  const auto oldSize = state.targets.size();
  state.targets.erase(
      std::remove_if(
          state.targets.begin(),
          state.targets.end(),
          [id](const WatchTarget& target) { return target.id == id; }),
      state.targets.end());
  state.series.erase(
      std::remove_if(
          state.series.begin(),
          state.series.end(),
          [id](const WatchSeries& series) { return series.object == id; }),
      state.series.end());

  if (state.targets.size() == oldSize) {
    return {false, "Object was not watched"};
  }
  return {true, "Removed watch"};
}

WatchActionResult clearWatch(WatchState& state)
{
  if (state.targets.empty() && state.series.empty()) {
    return {false, "Watch list already empty"};
  }
  state.targets.clear();
  state.series.clear();
  return {true, "Cleared watch list"};
}

WatchActionResult setWatchChartSignalEnabled(
    WatchState& state, WatchValueKind kind, bool enabled)
{
  if (!isChartSignalKind(kind)) {
    return {false, "Unsupported watch signal"};
  }

  const bool wasEnabled = isChartSignalEnabled(state, kind);
  if (wasEnabled == enabled) {
    return {
        false,
        std::string("Watch signal already ")
            + (enabled ? "enabled: " : "disabled: ") + valueLabel(kind)};
  }

  if (enabled) {
    state.chartSignals.push_back(kind);
    return {true, "Enabled watch signal: " + valueLabel(kind)};
  }

  state.chartSignals.erase(
      std::remove(state.chartSignals.begin(), state.chartSignals.end(), kind),
      state.chartSignals.end());
  state.series.erase(
      std::remove_if(
          state.series.begin(),
          state.series.end(),
          [kind](const WatchSeries& series) { return series.kind == kind; }),
      state.series.end());
  return {true, "Disabled watch signal: " + valueLabel(kind)};
}

WatchActionResult saveWatchPreset(
    WatchState& state, SimEngine& engine, std::string name)
{
  name = trim(std::move(name));
  if (name.empty()) {
    return {false, "Preset name is required"};
  }
  if (!engine.canEditScene()) {
    return {false, "Watch presets locked during Simulation Mode"};
  }

  std::vector<WorkspaceWatchPreset> presets
      = engine.objects().model().workspace.watchPresets;
  WorkspaceWatchPreset preset = captureWatchPreset(state, engine, name);
  const auto it = std::find_if(
      presets.begin(),
      presets.end(),
      [&name](const WorkspaceWatchPreset& candidate) {
        return candidate.name == name;
      });
  if (it == presets.end()) {
    presets.push_back(std::move(preset));
  } else {
    *it = std::move(preset);
  }

  if (presets == engine.objects().model().workspace.watchPresets) {
    return {false, "Watch preset already saved: " + name};
  }

  engine.execute(commands::setWorkspaceWatchPresets(std::move(presets)));
  if (findPreset(engine, name) == nullptr) {
    return {false, "Failed to save watch preset: " + name};
  }
  return {true, "Saved watch preset: " + name};
}

WatchActionResult applyWatchPreset(
    WatchState& state, const SimEngine& engine, std::string_view name)
{
  const std::string normalizedName = trim(std::string(name));
  if (normalizedName.empty()) {
    return {false, "Preset name is required"};
  }
  const WorkspaceWatchPreset* preset = findPreset(engine, normalizedName);
  if (preset == nullptr) {
    return {false, "Unknown watch preset: " + normalizedName};
  }

  state.targets.clear();
  state.series.clear();
  state.chartSignals = chartSignalsFromKeys(preset->chartSignals);

  std::size_t skipped = 0;
  for (const ObjectId target : preset->targets) {
    if (!engine.objects().model().contains(target)) {
      ++skipped;
      continue;
    }
    state.targets.push_back({target, engine.projectGeneration()});
  }

  if (skipped > 0) {
    return {
        true,
        "Applied watch preset: " + preset->name + " (" + std::to_string(skipped)
            + " missing target" + (skipped == 1 ? "" : "s") + " skipped)"};
  }
  return {true, "Applied watch preset: " + preset->name};
}

WatchActionResult deleteWatchPreset(SimEngine& engine, std::string_view name)
{
  const std::string normalizedName = trim(std::string(name));
  if (normalizedName.empty()) {
    return {false, "Preset name is required"};
  }
  if (!engine.canEditScene()) {
    return {false, "Watch presets locked during Simulation Mode"};
  }

  std::vector<WorkspaceWatchPreset> presets
      = engine.objects().model().workspace.watchPresets;
  const std::size_t oldSize = presets.size();
  presets.erase(
      std::remove_if(
          presets.begin(),
          presets.end(),
          [&normalizedName](const WorkspaceWatchPreset& preset) {
            return preset.name == normalizedName;
          }),
      presets.end());
  if (presets.size() == oldSize) {
    return {false, "Unknown watch preset: " + normalizedName};
  }

  engine.execute(commands::setWorkspaceWatchPresets(std::move(presets)));
  return {true, "Deleted watch preset: " + normalizedName};
}

void handleWatchEvent(WatchState& state, const Event& event)
{
  if (event.type == EventType::ProjectCreated
      || event.type == EventType::ProjectLoaded) {
    state.targets.clear();
    state.series.clear();
  }
}

void recordWatchSample(WatchState& state, const SimEngine& engine)
{
  for (const WatchValueKind kind :
       {WatchValueKind::SimulationTime, WatchValueKind::FrameCount}) {
    if (!isChartSignalEnabled(state, kind)) {
      continue;
    }
    WatchSeries& series = ensureSeries(
        state, engine, kNoObject, engine.projectGeneration(), kind);
    const std::optional<double> value = valueFor(engine, kNoObject, kind);
    if (value.has_value()) {
      pushSample(state, series, *value);
    }
  }

  for (const WatchTarget& target : state.targets) {
    if (target.projectGeneration != engine.projectGeneration()) {
      continue;
    }
    const SceneObject* object = engine.objects().model().find(target.id);
    if (object == nullptr) {
      continue;
    }
    for (const WatchValueKind kind : allWatchChartSignals()) {
      if (!isChartSignalEnabled(state, kind)) {
        continue;
      }
      const std::optional<double> value = valueFor(engine, target.id, kind);
      if (!value.has_value()) {
        continue;
      }
      WatchSeries& series = ensureSeries(
          state, engine, target.id, target.projectGeneration, kind);
      pushSample(state, series, *value);
    }
  }
}

WatchStatus buildWatchStatus(const WatchState& state, const SimEngine& engine)
{
  WatchStatus status;
  status.summary = state.targets.empty()
                       ? "No watched objects"
                       : std::to_string(state.targets.size())
                             + " watched object"
                             + (state.targets.size() == 1 ? "" : "s");
  status.signalOptions.reserve(allWatchChartSignals().size());
  for (const WatchValueKind kind : allWatchChartSignals()) {
    status.signalOptions.push_back(
        {kind, valueLabel(kind), isChartSignalEnabled(state, kind)});
  }
  status.presetOptions.reserve(
      engine.objects().model().workspace.watchPresets.size());
  for (const WorkspaceWatchPreset& preset :
       engine.objects().model().workspace.watchPresets) {
    std::size_t targetCount = 0;
    std::size_t missingTargetCount = 0;
    for (const ObjectId target : preset.targets) {
      if (engine.objects().model().contains(target)) {
        ++targetCount;
      } else {
        ++missingTargetCount;
      }
    }
    const std::vector<WatchValueKind> signals
        = chartSignalsFromKeys(preset.chartSignals);
    const std::size_t ignoredSignalCount
        = preset.chartSignals.size() >= signals.size()
              ? preset.chartSignals.size() - signals.size()
              : 0;
    status.presetOptions.push_back(
        {preset.name,
         targetCount,
         missingTargetCount,
         signals.size(),
         ignoredSignalCount});
  }

  status.series.reserve(state.series.size());
  for (const WatchSeries& series : state.series) {
    status.series.push_back(
        {series.object,
         series.projectGeneration,
         series.kind,
         series.label,
         std::span<const double>(
             series.samples.data(), series.samples.size())});
  }

  status.rows.reserve(state.targets.size());
  for (const WatchTarget& target : state.targets) {
    WatchRow row;
    row.id = target.id;
    row.selected = target.projectGeneration == engine.projectGeneration()
                   && engine.selection().isSelected(target.id);

    const SceneObject* object
        = target.projectGeneration == engine.projectGeneration()
              ? engine.objects().model().find(target.id)
              : nullptr;
    if (object == nullptr) {
      row.exists = false;
      row.name = "missing #" + std::to_string(target.id);
      row.type = "Missing";
      status.rows.push_back(std::move(row));
      continue;
    }

    row.exists = true;
    row.name = object->name;
    row.type = objectTypeLabel(object->type);
    for (const WatchValueKind kind : valueKindsForObject(*object)) {
      const std::optional<double> value = valueFor(engine, target.id, kind);
      if (!value.has_value()) {
        continue;
      }
      row.values.push_back({kind, valueLabel(kind), *value});
    }
    status.rows.push_back(std::move(row));
  }

  return status;
}

} // namespace dartsim::ui
