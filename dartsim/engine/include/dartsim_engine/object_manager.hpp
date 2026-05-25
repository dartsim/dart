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

#include <Eigen/Geometry>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/scene_object.hpp>

#include <memory>
#include <vector>

namespace dart::simulation::experimental {
class World;
} // namespace dart::simulation::experimental

namespace dartsim {

/// A renderable primitive resolved from the world for one editor object.
struct RenderItem
{
  ObjectId id = kNoObject;
  ShapeType shape = ShapeType::Box;
  Eigen::Vector3d dimensions = Eigen::Vector3d(1.0, 1.0, 1.0);
  Eigen::Vector4d color = Eigen::Vector4d(0.8, 0.8, 0.85, 1.0);
  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();
};

/// Owns the editable SceneModel and the experimental World derived from it.
///
/// Because the experimental World has no per-object removal, every structural
/// change rebuilds a fresh World from the model. The model is the source of
/// truth; the World is a derived runtime queried by name for rendering and
/// stepping. See docs/design/dartsim_gui_simulator.md.
class ObjectManager
{
public:
  ObjectManager();
  ~ObjectManager();

  ObjectManager(const ObjectManager&) = delete;
  ObjectManager& operator=(const ObjectManager&) = delete;

  [[nodiscard]] SceneModel& model()
  {
    return m_model;
  }
  [[nodiscard]] const SceneModel& model() const
  {
    return m_model;
  }

  /// Replace the whole model and rebuild the World (used by undo/redo and
  /// load).
  void setModel(SceneModel model);

  /// Rebuild the derived World from the current model and refresh kinematics.
  ///
  /// Call after editing model() directly. Building never throws on invalid
  /// user input; out-of-range values are sanitized so the editor stays alive.
  void rebuild();

  /// The derived experimental World (valid after construction; never null).
  [[nodiscard]] dart::simulation::experimental::World& world()
  {
    return *m_world;
  }
  [[nodiscard]] const dart::simulation::experimental::World& world() const
  {
    return *m_world;
  }

  /// Resolve current world transforms into renderable primitives.
  [[nodiscard]] std::vector<RenderItem> computeRenderItems() const;

  /// World transform of one object, if it currently exists in the World.
  [[nodiscard]] std::optional<Eigen::Isometry3d> worldTransformOf(
      ObjectId id) const;

private:
  SceneModel m_model;
  std::unique_ptr<dart::simulation::experimental::World> m_world;

  void buildMultiBody(const SceneObject& multiBodyObject);
};

} // namespace dartsim
