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

#ifndef DART_COLLISION_NATIVE_NATIVECOLLISIONOBJECT_HPP_
#define DART_COLLISION_NATIVE_NATIVECOLLISIONOBJECT_HPP_

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/native/Aabb.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <Eigen/Dense>

#include <limits>
#include <memory>

#include <cstddef>

namespace dart {
namespace collision {

class NativeCollisionDetector;
class NativeCollisionGroup;
class DARTCollisionObject;

class NativeCollisionObject : public CollisionObject
{
public:
  friend class NativeCollisionDetector;
  friend class NativeCollisionGroup;

  ~NativeCollisionObject() override;

  const native::Shape* getNativeShape() const;

  const Eigen::Isometry3d& getNativeTransform() const;

  const native::Aabb& getNativeAabb() const;

  DARTCollisionObject* getDartFallbackObject();

  bool usesDartFallbackShape() const;

  bool isPlaneShape() const;

protected:
  NativeCollisionObject(
      CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame);

  // Documentation inherited
  void updateEngineData() override;

private:
  void rebuildNativeShape();

  std::unique_ptr<native::Shape> mNativeShape;
  std::unique_ptr<DARTCollisionObject> mDartFallbackObject;
  Eigen::Isometry3d mNativeTransform{Eigen::Isometry3d::Identity()};
  native::Aabb mNativeAabb;
  bool mUsesDartFallbackShape{false};
  bool mIsPlaneShape{false};
  std::size_t mLastKnownShapeId{std::numeric_limits<std::size_t>::max()};
  std::size_t mLastKnownShapeVersion{0u};
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_NATIVE_NATIVECOLLISIONOBJECT_HPP_
