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
#include <dartsim_ui/outliner_actions.hpp>
#include <dartsim_ui/watch_actions.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <utility>

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
    case WatchValueKind::SimulationTime:
      return "Simulation time";
    case WatchValueKind::FrameCount:
      return "Frame";
  }
  return "value";
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
  return kinds;
}

std::vector<WatchValueKind> chartKindsForObject(const SceneObject& object)
{
  std::vector<WatchValueKind> kinds;
  const std::vector<WatchValueKind> valueKinds = valueKindsForObject(object);
  if (std::find(
          valueKinds.begin(), valueKinds.end(), WatchValueKind::TranslationZ)
      != valueKinds.end()) {
    kinds.push_back(WatchValueKind::TranslationZ);
  }
  if (std::find(
          valueKinds.begin(), valueKinds.end(), WatchValueKind::JointPosition)
      != valueKinds.end()) {
    kinds.push_back(WatchValueKind::JointPosition);
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

} // namespace

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
  WatchSeries& simTime = ensureSeries(
      state,
      engine,
      kNoObject,
      engine.projectGeneration(),
      WatchValueKind::SimulationTime);
  pushSample(state, simTime, engine.simulation().simTime());

  WatchSeries& frame = ensureSeries(
      state,
      engine,
      kNoObject,
      engine.projectGeneration(),
      WatchValueKind::FrameCount);
  pushSample(
      state, frame, static_cast<double>(engine.simulation().frameCount()));

  for (const WatchTarget& target : state.targets) {
    if (target.projectGeneration != engine.projectGeneration()) {
      continue;
    }
    const SceneObject* object = engine.objects().model().find(target.id);
    if (object == nullptr) {
      continue;
    }
    for (const WatchValueKind kind : chartKindsForObject(*object)) {
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
