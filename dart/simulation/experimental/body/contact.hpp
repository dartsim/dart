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

#include <dart/simulation/experimental/body/collision_body.hpp>

#include <Eigen/Core>

#include <cstddef>

namespace dart::simulation::experimental {

/// A single contact point produced by a collision query.
///
/// This is query output (no constraint solving): it reports where two bodies'
/// collision shapes overlap. Each body is a `CollisionBody` (a rigid body or a
/// multibody link). The contact normal points from `bodyA` toward `bodyB` and
/// `depth` is the penetration depth (positive when overlapping), both in world
/// coordinates.
struct Contact
{
  CollisionBody bodyA;
  CollisionBody bodyB;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double depth = 0.0;

  /// Index of the colliding shape within each body's collision geometry
  /// (`CollisionGeometry::shapes[shapeIndexA]` for `bodyA`, likewise for
  /// `bodyB`). Lets downstream consumers identify which shape of a compound
  /// body produced the contact. Defaults to 0, the only shape of a
  /// single-shape body.
  std::size_t shapeIndexA = 0;
  std::size_t shapeIndexB = 0;
};

} // namespace dart::simulation::experimental
