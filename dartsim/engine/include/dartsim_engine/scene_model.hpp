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

#include <dartsim_engine/scene_object.hpp>

#include <optional>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace dartsim {

/// The authoritative, editable description of a scene.
///
/// SceneModel is the single source of truth for the editor. The derived
/// experimental World is rebuilt from it (the experimental World has no
/// per-object removal), and undo/redo is implemented by copying SceneModel
/// snapshots, so this type is intentionally plain, copyable data.
class SceneModel
{
public:
  /// World time step seconds, applied to the rebuilt World.
  double timeStep = 0.001;

  /// Insert a new object and return its assigned id.
  ///
  /// If `object.id` is kNoObject a fresh id is allocated; otherwise the given
  /// id is used and the next-id counter advances past it (used when loading).
  /// The object is linked under its `parent` (kNoObject = world root) in
  /// insertion order.
  ObjectId add(SceneObject object);

  /// Remove an object and all of its descendants.
  void remove(ObjectId id);

  /// Re-link `id` under `newParent` (kNoObject = world root).
  ///
  /// Returns false if the move is invalid (unknown id, or moving a node under
  /// one of its own descendants).
  bool reparent(ObjectId id, ObjectId newParent);

  [[nodiscard]] SceneObject* find(ObjectId id);
  [[nodiscard]] const SceneObject* find(ObjectId id) const;
  [[nodiscard]] bool contains(ObjectId id) const;

  /// Ordered child ids of the world root.
  [[nodiscard]] const std::vector<ObjectId>& rootChildren() const
  {
    return m_rootOrder;
  }

  /// Ordered child ids of `parent` (use rootChildren() for the world root).
  [[nodiscard]] const std::vector<ObjectId>& childrenOf(ObjectId parent) const;

  /// All object ids in ascending order (deterministic; for serialization).
  [[nodiscard]] std::vector<ObjectId> allIds() const;

  [[nodiscard]] std::size_t size() const
  {
    return m_objects.size();
  }

  [[nodiscard]] bool empty() const
  {
    return m_objects.empty();
  }

  /// Remove every object and reset the id counter.
  void clear();

  /// Find a direct child of `parent` with the given name, if any.
  [[nodiscard]] std::optional<ObjectId> findChildByName(
      ObjectId parent, std::string_view name) const;

  /// True when no sibling of `parent` (other than `except`) already uses
  /// `name`.
  [[nodiscard]] bool isNameAvailable(
      ObjectId parent,
      std::string_view name,
      ObjectId except = kNoObject) const;

  /// Next id that add() would allocate (for tests and deterministic snapshots).
  [[nodiscard]] ObjectId peekNextId() const
  {
    return m_nextId;
  }

private:
  std::unordered_map<ObjectId, SceneObject> m_objects;
  std::vector<ObjectId> m_rootOrder;
  ObjectId m_nextId = 1;

  void unlinkFromParent(ObjectId id);
  bool isAncestorOf(ObjectId ancestor, ObjectId node) const;
};

} // namespace dartsim
