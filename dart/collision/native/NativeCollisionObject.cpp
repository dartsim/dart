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

#include "dart/collision/native/NativeCollisionObject.hpp"

#include "dart/collision/native/detail/NativeShapeConversion.hpp"
#include "dart/dynamics/Shape.hpp"

#include <limits>

namespace dart {
namespace collision {
namespace {

constexpr std::size_t kNoShapeId = std::numeric_limits<std::size_t>::max();

} // namespace

//==============================================================================
NativeCollisionObject::NativeCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : CollisionObject(collisionDetector, shapeFrame)
{
  updateEngineData();
}

//==============================================================================
const native::Shape* NativeCollisionObject::getNativeShape() const
{
  return mNativeShape.get();
}

//==============================================================================
const Eigen::Isometry3d& NativeCollisionObject::getNativeTransform() const
{
  return mNativeTransform;
}

//==============================================================================
const native::Aabb& NativeCollisionObject::getNativeAabb() const
{
  return mNativeAabb;
}

//==============================================================================
void NativeCollisionObject::updateEngineData()
{
  const auto shape = getShape();
  const auto* shapePtr = shape.get();
  const std::size_t shapeId = shapePtr ? shapePtr->getID() : kNoShapeId;
  const std::size_t shapeVersion = shapePtr ? shapePtr->getVersion() : 0u;

  if (shapeId != mLastKnownShapeId || shapeVersion != mLastKnownShapeVersion)
    rebuildNativeShape();

  mNativeTransform = getTransform();
  if (mNativeShape) {
    mNativeAabb = native::Aabb::transformed(
        mNativeShape->computeLocalAabb(), mNativeTransform);
  } else {
    mNativeAabb = native::Aabb();
  }
}

//==============================================================================
void NativeCollisionObject::rebuildNativeShape()
{
  const auto shape = getShape();
  mLastKnownShapeId = shape ? shape->getID() : kNoShapeId;
  mLastKnownShapeVersion = shape ? shape->getVersion() : 0u;

  if (!shape) {
    mNativeShape.reset();
    mNativeAabb = native::Aabb();
    return;
  }

  mNativeShape = detail::NativeShapeConversion::create(*shape);
}

} // namespace collision
} // namespace dart
