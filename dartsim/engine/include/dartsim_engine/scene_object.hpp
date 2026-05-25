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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

#include <cstdint>

namespace dartsim {

/// Stable, editor-side identifier for a scene object.
///
/// Editor IDs are owned by the SceneModel and remain stable across undo/redo
/// and across rebuilds of the derived experimental World (whose handles can be
/// invalidated). The world root uses kNoObject (0).
using ObjectId = std::uint64_t;

inline constexpr ObjectId kNoObject = 0;

/// The kind of object an editor node represents.
///
/// The experimental World has no per-object removal and no public shape API, so
/// the editor keeps its own typed scene description and rebuilds the World from
/// it. See docs/design/dartsim_gui_simulator.md.
enum class ObjectType
{
  RigidBody,
  MultiBody,
  Link,
  Joint,
  FreeFrame,
  FixedFrame,
};

/// Editor-side primitive shape used for rendering and picking.
///
/// The experimental World does not expose collision/visual geometry yet, so a
/// shape descriptor lives on the editor object until that API lands.
enum class ShapeType
{
  Box,
  Sphere,
  Cylinder,
  Capsule,
  Plane,
};

/// Joint kind mirrored from the experimental joint type.
///
/// Mirrored (rather than aliased) so the editor model does not depend on the
/// experimental header layout and stays serializable on its own.
enum class JointKind
{
  Fixed,
  Revolute,
  Prismatic,
  Screw,
  Universal,
  Ball,
  Planar,
  Free,
};

/// Editor-side shape descriptor.
///
/// dimensions interpretation:
/// - Box: full extents (x, y, z)
/// - Sphere: x = radius
/// - Cylinder/Capsule: x = radius, y = height
/// - Plane: (x, y, z) = plane normal
struct ShapeDesc
{
  ShapeType type = ShapeType::Box;
  Eigen::Vector3d dimensions = Eigen::Vector3d(1.0, 1.0, 1.0);
  Eigen::Vector4d color = Eigen::Vector4d(0.8, 0.8, 0.85, 1.0);

  friend bool operator==(const ShapeDesc& lhs, const ShapeDesc& rhs)
  {
    return lhs.type == rhs.type && lhs.dimensions == rhs.dimensions
           && lhs.color == rhs.color;
  }
};

/// A single node in the editable scene tree.
///
/// One plain-data struct covers every object type; only the fields relevant to
/// `type` are meaningful. This keeps the model copyable (for snapshot-based
/// undo/redo) and serializable without per-type class hierarchies.
struct SceneObject
{
  ObjectId id = kNoObject;
  ObjectType type = ObjectType::RigidBody;
  std::string name;
  ObjectId parent = kNoObject; ///< kNoObject = child of the world root
  std::vector<ObjectId> children;
  bool visible = true;

  // Spatial state. For RigidBody this is the world transform; for frames it is
  // the local transform relative to the parent frame.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  // Rigid-body dynamics (RigidBody and, where applicable, Link).
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  double mass = 1.0;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();

  // Rendering / picking (RigidBody and Link).
  ShapeDesc shape;

  // MultiBody membership (Link, Joint).
  ObjectId multiBody = kNoObject;  ///< owning MultiBody object id
  ObjectId parentLink = kNoObject; ///< parent Link id (kNoObject = root link)

  // Joint configuration (Link's parent joint).
  JointKind jointType = JointKind::Revolute;
  Eigen::Vector3d jointAxis = Eigen::Vector3d::UnitZ();
  double jointPosition = 0.0; ///< single-DOF generalized position

  // Exact equality, used to detect no-op edits when snapshotting for undo/redo.
  // Eigen::Isometry3d has no operator==, so compare the underlying matrix.
  friend bool operator==(const SceneObject& lhs, const SceneObject& rhs)
  {
    return lhs.id == rhs.id && lhs.type == rhs.type && lhs.name == rhs.name
           && lhs.parent == rhs.parent && lhs.children == rhs.children
           && lhs.visible == rhs.visible
           && lhs.transform.matrix() == rhs.transform.matrix()
           && lhs.linearVelocity == rhs.linearVelocity
           && lhs.angularVelocity == rhs.angularVelocity && lhs.mass == rhs.mass
           && lhs.inertia == rhs.inertia && lhs.shape == rhs.shape
           && lhs.multiBody == rhs.multiBody && lhs.parentLink == rhs.parentLink
           && lhs.jointType == rhs.jointType && lhs.jointAxis == rhs.jointAxis
           && lhs.jointPosition == rhs.jointPosition;
  }
};

} // namespace dartsim
