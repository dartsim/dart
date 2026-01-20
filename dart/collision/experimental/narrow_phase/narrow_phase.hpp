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

#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/types.hpp>

#include <Eigen/Core>

namespace dart::collision::experimental {

class DART_COLLISION_EXPERIMENTAL_API NarrowPhase
{
public:
  static bool collide(
      const Shape* shape1,
      const Eigen::Isometry3d& tf1,
      const Shape* shape2,
      const Eigen::Isometry3d& tf2,
      const CollisionOption& option,
      CollisionResult& result);

  static bool collide(
      const CollisionObject& obj1,
      const CollisionObject& obj2,
      const CollisionOption& option,
      CollisionResult& result);

  static double distance(
      const CollisionObject& obj1,
      const CollisionObject& obj2,
      const DistanceOption& option,
      DistanceResult& result);

  static bool isSupported(ShapeType type1, ShapeType type2);

  static bool isDistanceSupported(ShapeType type1, ShapeType type2);

  static bool raycast(
      const Ray& ray,
      const CollisionObject& object,
      const RaycastOption& option,
      RaycastResult& result);

  static bool isRaycastSupported(ShapeType type);

  static bool sphereCast(
      const Eigen::Vector3d& sphereStart,
      const Eigen::Vector3d& sphereEnd,
      double sphereRadius,
      const CollisionObject& target,
      const CcdOption& option,
      CcdResult& result);

  static bool isSphereCastSupported(ShapeType type);

  static bool capsuleCast(
      const Eigen::Isometry3d& capsuleStart,
      const Eigen::Isometry3d& capsuleEnd,
      const CapsuleShape& capsule,
      const CollisionObject& target,
      const CcdOption& option,
      CcdResult& result);

  static bool isCapsuleCastSupported(ShapeType type);
};

} // namespace dart::collision::experimental
