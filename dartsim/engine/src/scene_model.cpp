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

#include <dartsim_engine/scene_model.hpp>

#include <algorithm>
#include <string>
#include <unordered_set>

#include <cctype>

namespace dartsim {

namespace {
const std::vector<ObjectId> kEmptyChildren;

std::string trim(std::string value)
{
  const auto first
      = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
          return std::isspace(c) != 0;
        });
  const auto last
      = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
          return std::isspace(c) != 0;
        }).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

bool isWorkspaceSignalKey(std::string_view signal)
{
  if (signal.empty()) {
    return false;
  }
  return std::all_of(signal.begin(), signal.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '_' || c == '-' || c == '.' || c == ':'
           || c == '/';
  });
}

} // namespace

bool normalizeWorkspaceSettings(WorkspaceSettings& workspace)
{
  std::vector<WorkspaceWatchPreset> normalizedPresets;
  normalizedPresets.reserve(workspace.watchPresets.size());
  std::unordered_set<std::string> presetNames;

  for (WorkspaceWatchPreset preset : workspace.watchPresets) {
    preset.name = trim(std::move(preset.name));
    if (preset.name.empty() || !presetNames.insert(preset.name).second) {
      return false;
    }

    std::vector<ObjectId> targets;
    targets.reserve(preset.targets.size());
    std::unordered_set<ObjectId> seenTargets;
    for (const ObjectId target : preset.targets) {
      if (target == kNoObject || !seenTargets.insert(target).second) {
        continue;
      }
      targets.push_back(target);
    }
    preset.targets = std::move(targets);

    std::vector<std::string> chartSignals;
    chartSignals.reserve(preset.chartSignals.size());
    std::unordered_set<std::string> seenSignals;
    for (std::string signal : preset.chartSignals) {
      signal = trim(std::move(signal));
      if (!isWorkspaceSignalKey(signal)) {
        return false;
      }
      if (seenSignals.insert(signal).second) {
        chartSignals.push_back(std::move(signal));
      }
    }
    preset.chartSignals = std::move(chartSignals);

    normalizedPresets.push_back(std::move(preset));
  }

  workspace.watchPresets = std::move(normalizedPresets);
  return true;
}

ObjectId SceneModel::add(SceneObject object)
{
  if (object.id == kNoObject) {
    object.id = m_nextId++;
  } else if (object.id >= m_nextId) {
    m_nextId = object.id + 1;
  }

  const ObjectId id = object.id;
  const ObjectId parent = object.parent;
  object.children.clear();
  m_objects[id] = std::move(object);

  if (parent == kNoObject) {
    m_rootOrder.push_back(id);
  } else if (auto* parentObj = find(parent)) {
    parentObj->children.push_back(id);
  } else {
    // Unknown parent: attach to the world root so the node is not orphaned.
    m_objects[id].parent = kNoObject;
    m_rootOrder.push_back(id);
  }
  return id;
}

void SceneModel::remove(ObjectId id)
{
  auto it = m_objects.find(id);
  if (it == m_objects.end()) {
    return;
  }

  // Copy children first; removal mutates the parent's vector.
  const std::vector<ObjectId> children = it->second.children;
  for (const ObjectId child : children) {
    remove(child);
  }

  unlinkFromParent(id);
  m_objects.erase(id);
}

void SceneModel::unlinkFromParent(ObjectId id)
{
  const auto it = m_objects.find(id);
  if (it == m_objects.end()) {
    return;
  }
  const ObjectId parent = it->second.parent;
  std::vector<ObjectId>& siblings
      = (parent == kNoObject) ? m_rootOrder : m_objects[parent].children;
  siblings.erase(
      std::remove(siblings.begin(), siblings.end(), id), siblings.end());
}

bool SceneModel::reparent(ObjectId id, ObjectId newParent)
{
  if (!contains(id)) {
    return false;
  }
  if (newParent != kNoObject && !contains(newParent)) {
    return false;
  }
  if (id == newParent || isAncestorOf(id, newParent)) {
    return false;
  }

  unlinkFromParent(id);
  m_objects[id].parent = newParent;
  if (newParent == kNoObject) {
    m_rootOrder.push_back(id);
  } else {
    m_objects[newParent].children.push_back(id);
  }
  return true;
}

bool SceneModel::isAncestorOf(ObjectId ancestor, ObjectId node) const
{
  ObjectId cursor = node;
  while (cursor != kNoObject) {
    const auto it = m_objects.find(cursor);
    if (it == m_objects.end()) {
      return false;
    }
    if (it->second.parent == ancestor) {
      return true;
    }
    cursor = it->second.parent;
  }
  return false;
}

SceneObject* SceneModel::find(ObjectId id)
{
  const auto it = m_objects.find(id);
  return it == m_objects.end() ? nullptr : &it->second;
}

const SceneObject* SceneModel::find(ObjectId id) const
{
  const auto it = m_objects.find(id);
  return it == m_objects.end() ? nullptr : &it->second;
}

bool SceneModel::contains(ObjectId id) const
{
  return m_objects.find(id) != m_objects.end();
}

const std::vector<ObjectId>& SceneModel::childrenOf(ObjectId parent) const
{
  if (parent == kNoObject) {
    return m_rootOrder;
  }
  const auto it = m_objects.find(parent);
  return it == m_objects.end() ? kEmptyChildren : it->second.children;
}

std::vector<ObjectId> SceneModel::allIds() const
{
  std::vector<ObjectId> ids;
  ids.reserve(m_objects.size());
  for (const auto& [id, object] : m_objects) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

void SceneModel::clear()
{
  m_objects.clear();
  m_rootOrder.clear();
  m_nextId = 1;
}

std::optional<ObjectId> SceneModel::findChildByName(
    ObjectId parent, std::string_view name) const
{
  for (const ObjectId child : childrenOf(parent)) {
    const auto it = m_objects.find(child);
    if (it != m_objects.end() && it->second.name == name) {
      return child;
    }
  }
  return std::nullopt;
}

bool SceneModel::isNameAvailable(
    ObjectId parent, std::string_view name, ObjectId except) const
{
  for (const ObjectId child : childrenOf(parent)) {
    if (child == except) {
      continue;
    }
    const auto it = m_objects.find(child);
    if (it != m_objects.end() && it->second.name == name) {
      return false;
    }
  }
  return true;
}

bool SceneModel::hasSameSceneContents(const SceneModel& other) const
{
  return timeStep == other.timeStep && m_objects == other.m_objects
         && m_rootOrder == other.m_rootOrder && m_nextId == other.m_nextId;
}

} // namespace dartsim
