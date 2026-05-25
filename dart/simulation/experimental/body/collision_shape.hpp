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

namespace dart::simulation::experimental {

/// The geometric family of a collision shape.
///
/// Names describe geometry only; they do not name a collision backend.
enum class CollisionShapeType
{
  Sphere,
  Box,
};

/// Public value object describing a body's collision geometry.
///
/// This is a backend-neutral facade: the experimental World maps it onto the
/// maintained native collision engine when running collision queries. The shape
/// is expressed in the owning body's frame, centered at the body frame origin.
///
/// Only the fields relevant to `type` are used. Prefer the named constructors
/// (`makeSphere`, `makeBox`) for clarity.
struct CollisionShape
{
  /// Geometric family selecting which fields below are used.
  CollisionShapeType type = CollisionShapeType::Sphere;

  /// Sphere radius (used when type == Sphere). Must be positive.
  double radius = 0.5;

  /// Box half extents along the body x/y/z axes (used when type == Box). Each
  /// component must be positive.
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Constant(0.5);

  /// Create a sphere collision shape.
  [[nodiscard]] static CollisionShape makeSphere(double radius)
  {
    CollisionShape shape;
    shape.type = CollisionShapeType::Sphere;
    shape.radius = radius;
    return shape;
  }

  /// Create a box collision shape from half extents.
  [[nodiscard]] static CollisionShape makeBox(
      const Eigen::Vector3d& halfExtents)
  {
    CollisionShape shape;
    shape.type = CollisionShapeType::Box;
    shape.halfExtents = halfExtents;
    return shape;
  }
};

} // namespace dart::simulation::experimental
