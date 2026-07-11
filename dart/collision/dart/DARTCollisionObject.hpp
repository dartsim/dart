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

#ifndef DART_COLLISION_DART_DARTCOLLISIONOBJECT_HPP_
#define DART_COLLISION_DART_DARTCOLLISIONOBJECT_HPP_

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/dart/Aabb.hpp>
#include <dart/collision/dart/shapes/Shape.hpp>

#include <Eigen/Dense>

#include <limits>
#include <memory>

#include <cstddef>

namespace dart {
namespace collision {

class DARTCollisionDetector;
class DARTCollisionGroup;

class DARTCollisionObject : public CollisionObject
{
public:
  friend class DARTCollisionDetector;
  friend class DARTCollisionGroup;

  const native::Shape* getNativeShape() const;

  const Eigen::Isometry3d& getNativeTransform() const;

  const native::Aabb& getNativeAabb() const;

protected:
  DARTCollisionObject(
      CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame);

  // Documentation inherited
  void updateEngineData() override;

private:
  void rebuildNativeShape();

  std::unique_ptr<native::Shape> mNativeShape;
  Eigen::Isometry3d mNativeTransform{Eigen::Isometry3d::Identity()};
  /// Local-space AABB of mNativeShape, recomputed only when the native shape
  /// is rebuilt; shapes are immutable between version bumps, so recomputing
  /// it per step (a virtual call per object per step) is wasted work.
  native::Aabb mNativeLocalAabb;
  native::Aabb mNativeAabb;
  std::size_t mLastKnownShapeId{std::numeric_limits<std::size_t>::max()};
  std::size_t mLastKnownShapeVersion{0u};
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DART_DARTCOLLISIONOBJECT_HPP_
